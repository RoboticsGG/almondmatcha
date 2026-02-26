#!/bin/bash
# Launch Base Station Mission Monitoring (Domain 4)
# Subscribes to /tpc_telemetry_relay from Domain 4 (cross-domain relay from RPi)

cd ~/almondmatcha/ws_base
source install/setup.bash

# Set Domain 4 for base station monitoring (reduces Domain 5 participant count)
export ROS_DOMAIN_ID=4

echo "======================================"
echo "Mission Monitoring Node (Base Station)"
echo "======================================"
echo ""
echo "Subscribing to: /tpc_telemetry_relay"
echo "Domain: 4 (base monitoring)"
echo ""
echo "Architecture: Cross-domain relay (RPi publishes D4←D5 to reduce STM32 load)"
echo "Mission_monitoring_node_rpi aggregates D5 topics and relays to D4"
echo ""

ros2 run mission_control mission_monitoring_node_pc
