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
#   4. ros2 topic list — check if STM32 topics appear
#   5. ros2 topic hz — check if data is actually flowing
#
# Exit code: 0 = all checks passed, 1 = one or more checks failed

set -euo pipefail

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

ok()   { echo -e "  ${GRN}[PASS]${RST} $*"; ((PASS++)); }
fail() { echo -e "  ${RED}[FAIL]${RST} $*"; ((FAIL++)); }
warn() { echo -e "  ${YLW}[WARN]${RST} $*"; ((WARN++)); }
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

# ── Step 4: ros2 topic list ─────────────────────────────────────────────────
echo ""
echo "Step 4/5: ROS2 topic discovery (domain 5)"
info "Running 'ros2 topic list' with FastDDS profile..."

TOPIC_LIST=$(bash -c "
    source /opt/ros/humble/setup.bash 2>/dev/null
    source '$WORKSPACE/common_ifaces/install/setup.bash' 2>/dev/null
    export ROS_DOMAIN_ID=5
    export FASTRTPS_DEFAULT_PROFILES_FILE='$WORKSPACE/ws_base/fastdds_base.xml'
    timeout 8 ros2 topic list 2>/dev/null
" 2>/dev/null) || true

STM32_TOPICS=("/tpc_chassis_imu" "/tpc_chassis_sensors")
for t in "${STM32_TOPICS[@]}"; do
    if echo "$TOPIC_LIST" | grep -q "^${t}$"; then
        ok "Topic $t discovered"
    else
        fail "Topic $t NOT discovered"
    fi
done

# Also show all discovered topics for reference
if [ -n "$TOPIC_LIST" ]; then
    info "All topics found:"
    echo "$TOPIC_LIST" | sed 's/^/         /'
fi

# ── Step 5: ros2 topic hz (data flow check) ─────────────────────────────────
echo ""
echo "Step 5/5: Data flow check (ros2 topic hz, 6s sample)"

for t in "${STM32_TOPICS[@]}"; do
    hz_output=$(bash -c "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source '$WORKSPACE/common_ifaces/install/setup.bash' 2>/dev/null
        source '$WORKSPACE/ws_base/install/setup.bash' 2>/dev/null
        export ROS_DOMAIN_ID=5
        export FASTRTPS_DEFAULT_PROFILES_FILE='$WORKSPACE/ws_base/fastdds_base.xml'
        timeout 6 ros2 topic hz '$t' --window 3 2>&1 | head -3
    " 2>/dev/null) || true

    if echo "$hz_output" | grep -q "average rate"; then
        rate=$(echo "$hz_output" | grep -oP 'average rate:\s*\K[0-9.]+' | head -1)
        ok "$t flowing at ${rate} Hz"
    else
        fail "$t — no data within 6s"
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
    if echo "$TOPIC_LIST" | grep -qv "tpc_chassis"; then
        echo "  Troubleshooting tips:"
        echo "    - STM32 topics not discovered? Check serial console for boot banner"
        echo "    - No SPDP multicast? Reset STM32 board (press black button)"
        echo "    - Ensure patch 005 firmware is flashed (check build date in serial)"
        echo "    - Run: minicom -D /dev/ttyACM0 -b 115200  (or ttyACM1)"
    fi
fi
echo "═══════════════════════════════════════════════════════"
echo ""

exit "$FAIL"
