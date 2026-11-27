from glob import glob

from setuptools import setup

package_name = "zfy_arm_sim2real"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sky",
    maintainer_email="sky@example.com",
    description="ROS 2 driver for the real ZFY arm (Damiao MIT motors).",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_driver = zfy_arm_sim2real.motor_driver:main",
            "mit_demo = zfy_arm_sim2real.mit_demo:main",
            "safety_supervisor = zfy_arm_sim2real.safety_supervisor:main",
            "traj_bridge = zfy_arm_sim2real.traj_bridge:main",
            "ident_excite = zfy_arm_sim2real.ident_excite:main",
            "gripper_driver = zfy_arm_sim2real.gripper_driver:main",
        ],
    },
)
