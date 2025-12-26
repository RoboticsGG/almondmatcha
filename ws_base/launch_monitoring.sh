#!/bin/bash
# Launch Base Station Monitoring (Domain 4)
# Subscribes to /tpc_rover_status from Domain 4 (relayed from Domain 5)

cd ~/almondmatcha/ws_base
source install/setup.bash

# Set Domain 4 for monitoring
export ROS_DOMAIN_ID=4

echo "======================================"
echo "Base Station Monitoring (Domain 4)"
echo "======================================"
echo ""
echo "Subscribing to: /tpc_rover_status"
echo "Domain: 4 (monitoring domain)"
echo ""
echo "Make sure domain_relay.py is running on ws_rpi!"
echo ""

ros2 run mission_control base_monitoring_node
