#!/usr/bin/env bash
# 启动 ZFY 实机机械臂上位机（同时 source 驱动与接口包）
set -e
source /opt/ros/humble/setup.bash
source "$HOME/zfy_arm/Zfy_Arm_real_control/install/setup.bash"
exec python3 -u "$(dirname "$0")/arm_hmi.py" "$@"
