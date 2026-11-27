#!/usr/bin/env bash
# Start the ROS 2 motor driver on Jetson (gs_usb, classic CAN 1 Mbps).
#
# Jetson 上板载 mttcan 通常占用 can0，USB2CANFD 刷 gs_usb 后一般是 can1。
# 使用前先确认：ip -d link show can1  ->  driver=gs_usb, can state ERROR-ACTIVE
# 并配置：sudo ./scripts/setup_can_1m.sh can1
#
# Usage: ./scripts/start_driver_jetson.sh [config.yaml]
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"

CFG="${1:-$ROOT/ros2/zfy_arm_sim2real/config/motors_socketcan.yaml}"
if [ ! -f "$CFG" ]; then
  CFG="$(ros2 pkg prefix zfy_arm_sim2real)/share/zfy_arm_sim2real/config/motors_socketcan.yaml"
fi

exec ros2 launch zfy_arm_sim2real motor_driver.launch.py \
  config:="$CFG" \
  sim:=false
