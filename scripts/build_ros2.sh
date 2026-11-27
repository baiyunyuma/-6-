#!/usr/bin/env bash
# Build zfy_arm_interfaces + zfy_arm_sim2real in the bundled ros2/ workspace.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
cd "$ROOT/ros2"
colcon build \
  --base-paths zfy_arm_interfaces zfy_arm_sim2real \
  --packages-select zfy_arm_interfaces zfy_arm_sim2real \
  --symlink-install

echo "build done: source $ROOT/ros2/install/setup.bash"
