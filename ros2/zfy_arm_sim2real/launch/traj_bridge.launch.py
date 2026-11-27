"""Launch the MoveIt -> /motor_driver real-arm trajectory bridge."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("zfy_arm_sim2real")
    config = os.path.join(share, "config", "traj_bridge.yaml")
    return LaunchDescription([
        Node(
            package="zfy_arm_sim2real",
            executable="traj_bridge",
            name="zfy_arm_traj_bridge",
            output="screen",
            parameters=[config],
        ),
    ])
