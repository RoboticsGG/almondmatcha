#!/bin/bash
# setup_tracing.sh — Install LTTng + ros2_tracing dependencies
#
# Run once on EACH target machine (RPi and Jetson) via SSH:
#   ssh pi@<rpi_ip>    "bash -s" < setup_tracing.sh
#   ssh yupi@<jetson_ip> "bash -s" < setup_tracing.sh
#
# Or source it locally on the target.
#
# What this does:
#   1. Installs lttng-tools, lttng-modules, liblttng-ust-dev
#   2. Installs ros-humble-tracetools and ros-humble-ros2trace
#   3. Adds current user to the 'tracing' group (needed to start sessions without sudo)
#   4. Verifies the tracepoints are visible in rclcpp

set -e

ROS_DISTRO="${ROS_DISTRO:-humble}"

# Detect Jetson (Tegra BSP kernel)
# lttng-modules-dkms cannot build against the Tegra BSP kernel because
# linux-headers-*-tegra are NOT available as standard apt packages.
KERNEL_VER=$(uname -r)
echo "[INFO] Detected kernel: $KERNEL_VER"

IS_TEGRA=false
if echo "$KERNEL_VER" | grep -qi "tegra"; then
    IS_TEGRA=true
    echo "[INFO] Tegra/Jetson kernel detected — skipping lttng-modules-dkms"
fi

echo "=== Installing LTTng kernel/userspace tools ==="
sudo apt-get update -qq || true

if [ "$IS_TEGRA" = true ]; then
    sudo apt-get install -y \
        lttng-tools \
        liblttng-ust-dev \
        python3-lttng \
        babeltrace2
    echo "[WARN] Kernel LTTng modules skipped — Tegra BSP kernel has no apt headers."
    echo "       Userspace (rclcpp/rcl/rmw) tracepoints are still fully available."
else
    sudo apt-get install -y \
        lttng-tools \
        lttng-modules-dkms \
        liblttng-ust-dev \
        python3-lttng \
        babeltrace2
fi

echo "=== Installing ROS2 tracetools for $ROS_DISTRO ==="
sudo apt-get install -y \
    "ros-${ROS_DISTRO}-tracetools" \
    "ros-${ROS_DISTRO}-tracetools-launch" \
    "ros-${ROS_DISTRO}-ros2trace" \
    "ros-${ROS_DISTRO}-tracetools-read" \
    "ros-${ROS_DISTRO}-tracetools-analysis" 2>/dev/null || \
    echo "[WARN] Some tracetools packages not available in apt — may need to build from source"

echo "=== Adding user to 'tracing' group ==="
sudo usermod -aG tracing "$USER" || true
echo "[INFO] Group change takes effect on next login; for now run: newgrp tracing"

echo "=== Verifying LTTng userspace providers ==="
# Check that ros2 tracepoints are registered when rclcpp is loaded
lttng list --userspace 2>&1 | grep -i ros || \
    echo "[WARN] No ros tracepoints visible yet — start a node and re-check"

echo ""
echo "=== Setup complete ==="
if [ "$IS_TEGRA" = true ]; then
    echo "Jetson: kernel tracing UNAVAILABLE (Tegra BSP kernel has no apt headers)."
    echo "Userspace tracing (rclcpp, rcl, rmw) is fully functional."
else
    echo "Kernel tracing requires lttng-modules-dkms to be loaded:"
    echo "   sudo modprobe lttng-probe-sched lttng-probe-irq"
fi
echo "Next: run start_trace.sh on this machine (or remotely from the base PC)."
