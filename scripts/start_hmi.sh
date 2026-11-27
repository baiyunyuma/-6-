#!/usr/bin/env bash
# Start the full-arm PyQt HMI (ROS 2 + PyQt5) via WSLg.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"

# MuJoCo 3D: EGL backend is stable under WSLg; set ZFY_NO_3D=1 to disable.
export MUJOCO_GL="${MUJOCO_GL:-egl}"
# Default no-3D: the MuJoCo/OpenGL render thread crashes under WSLg.
# Set ZFY_NO_3D=0 explicitly to try 3D (currently unstable).
export ZFY_NO_3D="${ZFY_NO_3D:-1}"

exec python3 -u "$ROOT/ros2/hmi/arm_hmi.py" "$@"
