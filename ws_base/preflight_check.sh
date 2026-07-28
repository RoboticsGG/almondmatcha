#!/bin/bash
# preflight_check.sh — Field self-diagnosis for the base PC
#
# Answers one question: "if I launch right now, will the base PC actually see
# the rover?"  Every check prints the exact command that fixes it.
#
# Run standalone at any time:
#     bash ws_base/preflight_check.sh
#
# launch_field.sh runs this automatically and refuses to launch on a FAIL.
#
# Exit codes:  0 = all critical checks passed   1 = at least one FAIL
#
# Checks are ordered cheapest-and-most-likely-first, so the first FAIL you see
# is usually the root cause.

set -uo pipefail

WORKSPACE="${WORKSPACE:-$HOME/almondmatcha}"
RPI_IP="192.168.1.1"
JETSON_IP="192.168.1.5"
STM32_CHASSIS_IP="192.168.1.2"
STM32_SENSORS_IP="192.168.1.6"
EXPECTED_DOMAIN=5
SPDP_MCAST="239.255.0.1"
PROFILE="$WORKSPACE/ws_base/fastdds_base.xml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

FAILS=0; WARNS=0
pass() { echo -e "  ${GREEN}PASS${NC}  $*"; }
fail() { echo -e "  ${RED}FAIL${NC}  $*"; FAILS=$((FAILS+1)); }
warn() { echo -e "  ${YELLOW}WARN${NC}  $*"; WARNS=$((WARNS+1)); }
fix()  { echo -e "        ${CYAN}fix:${NC} $*"; }
hdr()  { echo ""; echo -e "${BOLD}$*${NC}"; }

echo -e "${BOLD}╔════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║   ALMONDMATCHA FIELD PRE-FLIGHT SELF-CHECK     ║${NC}"
echo -e "${BOLD}╚════════════════════════════════════════════════╝${NC}"

# ============================================================================
# 1. Local ROS environment — the most common cause of "I see no topics"
# ============================================================================
hdr "1. Base PC ROS environment"

if [[ -z "${ROS_DISTRO:-}" ]]; then
    fail "ROS 2 not sourced (ROS_DISTRO unset)"
    fix "source /opt/ros/humble/setup.bash"
else
    pass "ROS 2 sourced (\$ROS_DISTRO=$ROS_DISTRO)"
fi

# ROS_DOMAIN_ID: unset means domain 0, which shares NO traffic with domain 5.
# The rover, both STM32 boards and every RPi/Jetson node are on domain 5, so an
# unset value makes the base PC silently blind to the entire system.
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    fail "ROS_DOMAIN_ID is UNSET — defaults to 0, rover is on $EXPECTED_DOMAIN. You will see nothing."
    fix "export ROS_DOMAIN_ID=$EXPECTED_DOMAIN     (and add it to ~/.bashrc)"
elif [[ "$ROS_DOMAIN_ID" != "$EXPECTED_DOMAIN" ]]; then
    fail "ROS_DOMAIN_ID=$ROS_DOMAIN_ID but the rover runs on domain $EXPECTED_DOMAIN"
    fix "export ROS_DOMAIN_ID=$EXPECTED_DOMAIN     (domain 4 is telemetry-only, not control)"
else
    pass "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

if [[ "${ROS_LOCALHOST_ONLY:-0}" == "1" ]]; then
    fail "ROS_LOCALHOST_ONLY=1 — all off-machine DDS traffic is blocked"
    fix "unset ROS_LOCALHOST_ONLY   (or export ROS_LOCALHOST_ONLY=0)"
else
    pass "ROS_LOCALHOST_ONLY not blocking"
fi

RMW="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "$RMW" != "rmw_fastrtps_cpp" ]]; then
    fail "RMW_IMPLEMENTATION=$RMW — the STM32 boards speak Fast-DDS/embeddedRTPS only"
    fix "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
else
    pass "RMW_IMPLEMENTATION=$RMW"
fi

# ============================================================================
# 2. Fast-DDS profile — without it FastDDS may bind to WiFi and miss the LAN
# ============================================================================
hdr "2. Fast-DDS profile"

if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
    fail "FASTRTPS_DEFAULT_PROFILES_FILE is UNSET"
    fix "export FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE"
    fix "without it FastDDS picks any NIC (usually WiFi) and never joins the rover LAN"
elif [[ ! -r "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]]; then
    fail "Profile file not readable: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
    fix "export FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE"
else
    pass "Profile: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
    PROFILE="${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi

# ============================================================================
# 3. This machine's IP must be in the profile's interfaceWhiteList.
#    A whitelist that matches no local address yields a transport that binds
#    nothing — FastDDS starts cleanly and then hears absolutely nothing.
# ============================================================================
hdr "3. Base PC address vs. profile whitelist"

MY_IPS=$(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1)
if [[ -z "$MY_IPS" ]]; then
    fail "No global IPv4 address on any interface"
    fix "bring up the rover Ethernet link and set a static IP"
else
    echo "        local addresses: $(echo $MY_IPS | tr '\n' ' ')"
fi

if [[ -r "$PROFILE" ]]; then
    WL=$(sed -n '/<interfaceWhiteList>/,/<\/interfaceWhiteList>/p' "$PROFILE" \
         | grep -oE '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+')
    echo "        profile whitelist: $(echo $WL | tr '\n' ' ')"
    MATCH=""
    for ip in $MY_IPS; do
        for w in $WL; do [[ "$ip" == "$w" ]] && MATCH="$ip"; done
    done
    if [[ -n "$MATCH" ]]; then
        pass "This PC holds whitelisted address $MATCH"
    else
        fail "NO local address matches the profile whitelist — FastDDS will bind nothing"
        fix "either set this PC's rover-LAN IP to one of: $(echo $WL | tr '\n' ' ')"
        fix "or add this PC's address to <interfaceWhiteList> in $PROFILE"
        fix "  sudo nmcli con mod \"Wired connection 1\" ipv4.addresses 192.168.1.11/24 ipv4.method manual"
        fix "  sudo nmcli con up \"Wired connection 1\""
    fi
else
    warn "Cannot read profile — skipping whitelist cross-check"
fi

# ============================================================================
# 4. Link-level reachability. The STM32 boards have no SSH, so ping is the
#    only pre-launch evidence that they are powered and on the network.
# ============================================================================
hdr "4. Network reachability"

ping_host() {
    local ip="$1" name="$2" critical="$3"
    if ping -c 2 -W 1 "$ip" &>/dev/null; then
        pass "$name ($ip) reachable"
    elif [[ "$critical" == "critical" ]]; then
        fail "$name ($ip) NOT reachable"
        fix "check power, Ethernet cable, and switch port for $name"
    else
        warn "$name ($ip) not responding to ping"
    fi
}
ping_host "$RPI_IP"           "RPi"            critical
ping_host "$JETSON_IP"        "Jetson"         critical
ping_host "$STM32_CHASSIS_IP" "STM32 chassis"  critical
ping_host "$STM32_SENSORS_IP" "STM32 sensors"  critical

# Multicast: the STM32 boards announce themselves via SPDP on 239.255.0.1:8650.
#
# It is not enough for a multicast route to exist — it must leave the SAME
# interface that holds the rover-LAN address. On a laptop the default route is
# normally WiFi, so multicast follows WiFi while the rover LAN sits on Ethernet.
# SPDP announcements are then transmitted out of the wrong NIC and the STM32
# boards are never discovered, even though ping to them succeeds (unicast is
# routed correctly by the 192.168.1.0/24 subnet route). This failure is
# invisible without an explicit check.
ROVER_IF=""
if [[ -n "${MATCH:-}" ]]; then
    ROVER_IF=$(ip -4 -o addr show scope global 2>/dev/null | awk -v a="$MATCH" '$4 ~ "^"a"/" {print $2}' | head -1)
fi

if ip route get "$SPDP_MCAST" &>/dev/null; then
    MCIF=$(ip route get "$SPDP_MCAST" 2>/dev/null | grep -oP 'dev \K\S+' | head -1)
    if [[ -n "$ROVER_IF" && -n "$MCIF" && "$MCIF" != "$ROVER_IF" ]]; then
        fail "Multicast leaves '$MCIF' but the rover LAN ($MATCH) is on '$ROVER_IF'"
        fix "STM32 SPDP is going out the wrong NIC — ping works, discovery cannot"
        fix "sudo ip route add 224.0.0.0/4 dev $ROVER_IF metric 0"
        fix "verify with: ip route get $SPDP_MCAST   (must say 'dev $ROVER_IF')"
    elif [[ -n "$MCIF" ]]; then
        pass "Multicast route to $SPDP_MCAST via $MCIF${ROVER_IF:+ (rover LAN NIC)}"
    else
        warn "Could not determine the multicast egress interface"
    fi
    if [[ -n "${MCIF:-}" ]] && ! ip link show "$MCIF" 2>/dev/null | grep -q MULTICAST; then
        fail "Interface $MCIF does not have the MULTICAST flag — STM32 discovery cannot work"
        fix "sudo ip link set $MCIF multicast on"
    fi
else
    fail "No route for multicast $SPDP_MCAST — STM32 SPDP discovery will not arrive"
    fix "sudo ip route add 224.0.0.0/4 dev ${ROVER_IF:-<rover-ethernet-iface>}"
fi

if command -v ufw &>/dev/null && ufw status 2>/dev/null | grep -q "Status: active"; then
    if ufw status 2>/dev/null | grep -q "192.168.1.0/24"; then
        pass "Firewall active with a rover-subnet allow rule"
    else
        fail "ufw is ACTIVE with no 192.168.1.0/24 rule — DDS UDP will be dropped"
        fix "sudo ufw allow from 192.168.1.0/24"
    fi
else
    pass "No blocking host firewall detected"
fi

# ============================================================================
# 5. Remote machines — same environment traps apply there
# ============================================================================
hdr "5. Remote node environment"

check_remote() {
    local host="$1" name="$2" ws="$3" prof="$4"
    if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$host" true 2>/dev/null; then
        fail "$name: SSH failed ($host)"
        fix "ssh-copy-id $host    (or check the --rpi/--jetson argument)"
        return
    fi
    local out
    out=$(ssh -o ConnectTimeout=5 "$host" \
        "source ~/.bashrc >/dev/null 2>&1; echo \"\${ROS_DOMAIN_ID:-UNSET}|\${FASTRTPS_DEFAULT_PROFILES_FILE:-UNSET}|\$(test -d ~/almondmatcha/$ws/install && echo BUILT || echo MISSING)\"" 2>/dev/null)
    local dom="${out%%|*}"; local rest="${out#*|}"
    local pf="${rest%%|*}"; local built="${rest##*|}"

    [[ "$dom" == "$EXPECTED_DOMAIN" ]] \
        && pass "$name ROS_DOMAIN_ID=$dom" \
        || { fail "$name ROS_DOMAIN_ID=$dom (expected $EXPECTED_DOMAIN)"; \
             fix "on $name: echo 'export ROS_DOMAIN_ID=$EXPECTED_DOMAIN' >> ~/.bashrc"; }
    [[ "$pf" != "UNSET" ]] \
        && pass "$name DDS profile set" \
        || { fail "$name FASTRTPS_DEFAULT_PROFILES_FILE unset"; \
             fix "on $name: echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/almondmatcha/$prof' >> ~/.bashrc"; }
    [[ "$built" == "BUILT" ]] \
        && pass "$name $ws built" \
        || { fail "$name $ws not built"; fix "on $name: cd ~/almondmatcha/$ws && colcon build"; }
}
check_remote "${RPI_HOST:-curry@192.168.1.1}"  "RPi"    "ws_rpi"    "ws_rpi/fastdds_rover.xml"
check_remote "${JETSON_HOST:-yupi@192.168.1.5}" "Jetson" "ws_jetson" "ws_jetson/fastdds_jetson.xml"

# ============================================================================
# 6. Live DDS discovery — the only check that proves the stack actually works
# ============================================================================
hdr "6. Live DDS discovery (10 s listen)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
    warn "ROS not sourced — skipping live discovery probe"
else
    NODES=$(timeout 10 ros2 node list 2>/dev/null | grep -v '^$' || true)
    TOPICS=$(timeout 10 ros2 topic list 2>/dev/null | grep -v '^/parameter_events$\|^/rosout$\|^$' || true)
    N_NODES=$(echo "$NODES"  | grep -c . || true)
    N_TOPIC=$(echo "$TOPICS" | grep -c . || true)

    if [[ "$N_NODES" -eq 0 && "$N_TOPIC" -eq 0 ]]; then
        if [[ $FAILS -eq 0 ]]; then
            warn "No participants visible — expected if the rover nodes are not started yet"
            fix "this is only meaningful AFTER launch_rover_tmux.sh / launch_jetson_tmux.sh are up"
        else
            fail "No DDS participants visible — fix the FAILs above first"
        fi
    else
        pass "Discovered $N_NODES node(s), $N_TOPIC topic(s)"
        for t in /tpc_chassis_sensors /tpc_chassis_imu /tpc_chassis_cmd; do
            echo "$TOPICS" | grep -qx "$t" \
                && pass "  STM32 topic $t present" \
                || warn "  STM32 topic $t NOT seen (board down, or SPDP multicast not arriving)"
        done
    fi
fi

# ============================================================================
# Verdict
# ============================================================================
echo ""
if [[ $FAILS -gt 0 ]]; then
    echo -e "${RED}${BOLD}PRE-FLIGHT FAILED — $FAILS critical, $WARNS warning(s)${NC}"
    echo -e "${RED}Fix the FAIL lines above before launching. Do not run the rover.${NC}"
    exit 1
fi
echo -e "${GREEN}${BOLD}PRE-FLIGHT PASSED${NC}${WARNS:+ — $WARNS warning(s), review above}"
exit 0
