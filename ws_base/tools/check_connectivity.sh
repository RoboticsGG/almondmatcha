#!/bin/bash
# check_connectivity.sh — Lightweight pre-flight connectivity test
#
# Verifies network, multicast, and DDS discovery between all rover nodes
# WITHOUT launching the full POC experiment.  Run this before the POC script
# to confirm the communication layer is healthy.
#
# Usage:
#   bash ws_base/tools/check_connectivity.sh
#
# Checks performed:
#   1. Ping all rover nodes (RPi, Jetson, STM32 chassis, STM32 sensors)
#   2. Verify SPDP multicast packets from STM32 boards (tcpdump)
#   3. Verify FastDDS metatraffic multicast port is open (ss)
#   4. ros2 topic list — check if STM32 topics appear (with retry)
#   5. ros2 topic hz — check if data is actually flowing
#
# Discovery reliability note:
#   Steps 4 and 5 share a SINGLE FastDDS participant (one bash subshell)
#   to avoid the cost of re-discovery.  Each `ros2` CLI invocation creates
#   an ephemeral DDS participant that must complete the full SPDP→SEDP
#   cycle with the STM32 boards before topics become visible:
#
#     1. FastDDS sends SPDP announcement → STM32 receives on 239.255.0.1:8650
#     2. STM32 processes SPDP, adds remote participant, sends SEDP via multicast
#     3. FastDDS receives SEDP, matches endpoints → topics visible
#
#   Worst-case timing:  STM32 SPDP period = 500 ms, FastDDS default ≈ 3 s.
#   The CLI participant may arrive just AFTER an STM32 SPDP send, waiting up
#   to 500 ms for the next one.  Then SEDP exchange takes 1–2 heartbeat
#   cycles (HB period = 1000 ms).  Total worst-case: ~3.5 s per direction.
#
#   lwIP pbuf pool (PBUF_POOL_SIZE=20) can also briefly exhaust during
#   discovery bursts, causing silent packet drops.  Retries absorb this.
#
# Exit code: 0 = all checks passed, 1 = one or more checks failed

set -uo pipefail

WORKSPACE="$(cd "$(dirname "$0")/../.." && pwd)"
NIC="${NIC:-enp0s31f6}"

# Colors
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
RST='\033[0m'

PASS=0
FAIL=0
WARN=0

ok()   { echo -e "  ${GRN}[PASS]${RST} $*"; PASS=$((PASS + 1)); }
fail() { echo -e "  ${RED}[FAIL]${RST} $*"; FAIL=$((FAIL + 1)); }
warn() { echo -e "  ${YLW}[WARN]${RST} $*"; WARN=$((WARN + 1)); }
info() { echo -e "  ${CYN}[INFO]${RST} $*"; }

# ── Step 1: Ping all nodes ──────────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════"
echo "  Rover Connectivity Check"
echo "═══════════════════════════════════════════════════════"
echo ""
echo "Step 1/5: Ping all rover nodes"

declare -A NODES=(
    [rpi]=192.168.1.1
    [chassis_stm32]=192.168.1.2
    [base_pc]=192.168.1.4
    [jetson]=192.168.1.5
    [sensors_stm32]=192.168.1.6
)

for name in rpi chassis_stm32 base_pc jetson sensors_stm32; do
    ip="${NODES[$name]}"
    if ping -c 1 -W 2 "$ip" &>/dev/null; then
        ok "$name ($ip) — reachable"
    else
        fail "$name ($ip) — unreachable"
    fi
done

# ── Step 2: Check SPDP multicast from STM32 boards ─────────────────────────
echo ""
echo "Step 2/5: SPDP multicast from STM32 boards (tcpdump, 10s max)"
info "Listening on $NIC for SPDP packets to 239.255.0.1:8650..."

# Check chassis (192.168.1.2)
if sudo timeout 10 tcpdump -i "$NIC" -c 1 \
    'src host 192.168.1.2 and dst host 239.255.0.1 and udp port 8650' \
    -nn -q 2>&1 | grep -q "1 packet"; then
    ok "Chassis STM32 (192.168.1.2) — SPDP multicast detected"
else
    fail "Chassis STM32 (192.168.1.2) — no SPDP multicast within 10s"
fi

# Check sensors (192.168.1.6)
if sudo timeout 10 tcpdump -i "$NIC" -c 1 \
    'src host 192.168.1.6 and dst host 239.255.0.1 and udp port 8650' \
    -nn -q 2>&1 | grep -q "1 packet"; then
    ok "Sensors STM32 (192.168.1.6) — SPDP multicast detected"
else
    fail "Sensors STM32 (192.168.1.6) — no SPDP multicast within 10s"
fi

# ── Step 3: Check FastDDS multicast port binding ────────────────────────────
echo ""
echo "Step 3/5: FastDDS metatraffic multicast port (local)"
info "Checking if any process listens on UDP 8650 (SPDP multicast for domain 5)..."

# The ros2 topic list call in Step 4 will create a FastDDS participant that
# binds port 8650.  Check after step 4 if needed.  For now, just check if
# the multicast group is joined.
if ip maddr show dev "$NIC" 2>/dev/null | grep -q "239.255.0.1"; then
    ok "Multicast group 239.255.0.1 joined on $NIC"
else
    warn "Multicast group 239.255.0.1 NOT joined on $NIC yet (will join when ROS2 node starts)"
fi

# Check that metatraffic UNICAST ports are NOT open (expected with our XML config)
for port in 8660 8662 8664; do
    if ss -ulnp 2>/dev/null | grep -q ":${port} "; then
        warn "Metatraffic unicast port $port is open (unexpected with metatrafficMulticastLocatorList XML)"
    fi
done

# ── Step 4: ros2 topic list (with retry) ────────────────────────────────────
echo ""
echo "Step 4/5: ROS2 topic discovery (domain 5, up to 3 attempts)"

# Discovery is inherently racy: each ros2 CLI invocation creates an ephemeral
# FastDDS participant that must complete the SPDP→SEDP exchange with both
# STM32 boards.  Worst-case timing per attempt:
#   - Wait for next STM32 SPDP: 0–500 ms
#   - SPDP processing + SEDP exchange: 1–2 HB cycles × 1000 ms
#   - Total: up to ~3.5 s
#
# lwIP's pbuf pool (PBUF_POOL_SIZE=20) can also drop packets during
# discovery bursts, requiring a fresh attempt.  3 retries with increasing
# timeout absorb transient failures without masking real problems.
#
# The ROS2 environment is sourced ONCE and reused for step 5 to keep the
# same FastDDS participant alive (avoids re-discovery).

STM32_TOPICS=("/tpc_chassis_imu" "/tpc_chassis_sensors")
TOPIC_LIST=""
DISCOVERY_OK=false

for attempt in 1 2 3; do
    # Increasing timeout: 8s → 10s → 14s
    ros2_timeout=$((6 + attempt * 2 + (attempt - 1) * 2))
    info "Attempt $attempt/3 (timeout ${ros2_timeout}s)..."

    TOPIC_LIST=$(bash -c "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source '$WORKSPACE/common_ifaces/install/setup.bash' 2>/dev/null
        source '$WORKSPACE/ws_base/install/setup.bash' 2>/dev/null
        export ROS_DOMAIN_ID=5
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export FASTRTPS_DEFAULT_PROFILES_FILE='$WORKSPACE/ws_base/fastdds_base.xml'
        timeout ${ros2_timeout} ros2 topic list 2>/dev/null
    " 2>/dev/null) || true

    all_found=true
    for t in "${STM32_TOPICS[@]}"; do
        if ! echo "$TOPIC_LIST" | grep -q "^${t}$"; then
            all_found=false
            break
        fi
    done

    if [ "$all_found" = true ]; then
        DISCOVERY_OK=true
        break
    fi

    if [ "$attempt" -lt 3 ]; then
        info "Not all topics found — retrying in 2s..."
        sleep 2
    fi
done

for t in "${STM32_TOPICS[@]}"; do
    if echo "$TOPIC_LIST" | grep -q "^${t}$"; then
        ok "Topic $t discovered"
    else
        fail "Topic $t NOT discovered (after 3 attempts)"
    fi
done

# Also show all discovered topics for reference
if [ -n "$TOPIC_LIST" ]; then
    info "All topics found:"
    echo "$TOPIC_LIST" | sed 's/^/         /'
fi

# ── Step 5: Data flow check ──────────────────────────────────────────────────
echo ""
echo "Step 5/5: Data flow check"

# Use `ros2 topic echo --once` to verify at least 1 message arrives per
# topic.  This is a binary check — if one message arrives, the full
# serialization→DDS→deserialization pipeline works.
#
# Each `ros2 topic echo` creates an ephemeral FastDDS participant.  To
# avoid multiple SPDP→SEDP cycles, we check both topics in sequence
# inside one helper script, then parse the combined output.
#
# 3 attempts with increasing timeout per topic: 10s / 14s / 20s.

ECHO_HELPER=$(mktemp)
trap "rm -f '$ECHO_HELPER'" EXIT
cat > "$ECHO_HELPER" << 'HELPEREOF'
#!/bin/bash
# Check data flow for multiple topics in parallel.
# Each ros2 topic echo runs in the background so all topics start
# discovery simultaneously, reducing total wait time and avoiding
# sequential participant churn on the STM32 boards.
source /opt/ros/humble/setup.bash 2>/dev/null
source "$1/common_ifaces/install/setup.bash" 2>/dev/null
source "$1/ws_base/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$1/ws_base/fastdds_base.xml"
TIMEOUT=$2
OUTDIR=$(mktemp -d)
shift 2
pids=()
for topic in "$@"; do
    (timeout "$TIMEOUT" ros2 topic echo "$topic" --once 2>/dev/null \
        > "$OUTDIR/$(echo "$topic" | tr '/' '_')" ) &
    pids+=($!)
done
wait "${pids[@]}" 2>/dev/null
for topic in "$@"; do
    fname="$OUTDIR/$(echo "$topic" | tr '/' '_')"
    echo "=== $topic ==="
    if [ -s "$fname" ]; then
        cat "$fname"
    else
        echo "=== TIMEOUT ==="
    fi
done
rm -rf "$OUTDIR"
HELPEREOF
chmod +x "$ECHO_HELPER"

for echo_attempt in 1 2 3; do
    echo_timeout=$((6 + echo_attempt * 4 + (echo_attempt - 1) * 2))
    info "Attempt $echo_attempt/3 (timeout ${echo_timeout}s per topic)..."

    combined_output=$(bash "$ECHO_HELPER" "$WORKSPACE" "$echo_timeout" \
        "${STM32_TOPICS[@]}" 2>/dev/null) || true

    all_topics_ok=true
    for t in "${STM32_TOPICS[@]}"; do
        # Extract block between "=== /topic ===" markers (use # as sed delimiter for / in topic names)
        block=$(echo "$combined_output" | sed -n "\\#^=== ${t} ===#,\\#^===#p" | head -10)
        if echo "$block" | grep -qE "_msg|accel_|gyro_"; then
            : # topic ok
        else
            all_topics_ok=false
        fi
    done

    if [ "$all_topics_ok" = true ]; then
        break
    fi

    if [ "$echo_attempt" -lt 3 ]; then
        info "Not all topics received data — retrying in 2s..."
        sleep 2
    fi
done

# Report per-topic results
for t in "${STM32_TOPICS[@]}"; do
    block=$(echo "$combined_output" | sed -n "\\#^=== ${t} ===#,\\#^===#p" | head -10)
    if echo "$block" | grep -qE "_msg|accel_|gyro_"; then
        ok "$t — data received"
    else
        fail "$t — no data after 3 attempts"
    fi
done

# ── Summary ─────────────────────────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════"
if [ "$FAIL" -eq 0 ]; then
    echo -e "  ${GRN}ALL CHECKS PASSED${RST} ($PASS passed, $WARN warnings)"
    echo "  Ready to run POC experiment."
else
    echo -e "  ${RED}$FAIL CHECK(S) FAILED${RST} ($PASS passed, $WARN warnings)"
    echo ""
    echo "  Troubleshooting tips:"
    echo "    - STM32 topics not discovered?"
    echo "        Check serial console: minicom -D /dev/ttyACM0 -b 115200"
    echo "        Look for the build banner (Firmware/Built/Board IP lines)"
    echo "    - No SPDP multicast from STM32?"
    echo "        Reset board (black button) and wait 5s for lwIP init"
    echo "        Verify patch 005 firmware: build date in serial banner"
    echo "    - Intermittent Step 4/5 failures?"
    echo "        Normal if boards just booted — SPDP+SEDP takes ~3.5s worst-case"
    echo "        lwIP pbuf pool (20 slots) can drop frames during burst discovery"
    echo "        Re-run script after boards have been up for 10+ seconds"
fi
echo "═══════════════════════════════════════════════════════"
echo ""

exit "$FAIL"
