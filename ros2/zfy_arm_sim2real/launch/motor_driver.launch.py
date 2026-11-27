import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _find_hmi_py():
    candidates = []
    env = os.environ.get("ZFY_HMI_PY")
    if env:
        candidates.append(Path(env).expanduser())
    here = Path(__file__).resolve()
    for base in (here.parent, *here.parents):
        candidates.append(base / "py")
        candidates.append(base)
    for cand in candidates:
        try:
            cand = cand.resolve()
        except OSError:
            continue
        if (cand / "core.py").is_file() and (cand / "motor_config.py").is_file():
            return cand
    return None


def generate_launch_description():
    package_share = get_package_share_directory("zfy_arm_sim2real")
    default_config = os.path.join(package_share, "config", "motors.yaml")

    config = LaunchConfiguration("config")
    sim = LaunchConfiguration("sim")

    # The Damiao SDK ships a newer libstdc++/libusb than Ubuntu 22.04.
    # rclpy has already loaded the system libstdc++ by the time the node
    # imports dm_backend, so RTLD_GLOBAL preloading is too late.
    node_env = {}
    hmi_py = _find_hmi_py()
    if hmi_py is not None:
        lib_dir = hmi_py / "dm_backend" / "lib"
        if lib_dir.is_dir():
            old = os.environ.get("LD_LIBRARY_PATH", "")
            node_env["LD_LIBRARY_PATH"] = f"{lib_dir}:{old}" if old else str(lib_dir)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="电机注册表与增益的 YAML 文件",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                description="true = 不打开 CAN；实机必须显式 sim:=false",
            ),
            Node(
                package="zfy_arm_sim2real",
                executable="motor_driver",
                name="motor_driver",
                output="screen",
                additional_env=node_env,
                parameters=[
                    config,
                    {"sim": ParameterValue(sim, value_type=bool)},
                ],
            ),
        ]
    )
