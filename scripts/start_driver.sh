#!/usr/bin/env bash
# Start the ROS 2 motor driver against can0 (classic CAN 1 Mbps).
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"

exec ros2 launch zfy_arm_sim2real motor_driver.launch.py \
  config:="$ROOT/config/motors_socketcan_wsl.yaml" \
  sim:=false
