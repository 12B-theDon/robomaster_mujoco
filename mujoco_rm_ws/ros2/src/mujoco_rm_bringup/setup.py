from setuptools import find_packages, setup


package_name = "mujoco_rm_bringup"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/joy_mujoco_cam.launch.py"]),
        ("share/" + package_name + "/launch", ["launch/teleop_linux.launch.xml"]),
        ("share/" + package_name + "/config", ["config/teleop_twist_joy.yaml"]),
        ("share/" + package_name + "/config", ["config/xbox_piloting.yaml"]),
        ("share/" + package_name + "/config", ["config/xbox_joy_config_mujoco.yaml"]),
        ("share/" + package_name + "/config", ["config/servos.yaml"]),
        # Colcon (non-ament) build workaround: ensure ROS2 can discover this package by
        # extending AMENT_PREFIX_PATH when sourcing the workspace.
        ("share/" + package_name + "/hook", ["hook/ament_prefix_path.dsv"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@localhost",
    description="ROS 2 bringup for MuJoCo robot with joystick teleop and camera streaming.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mujoco_sim = mujoco_rm_bringup.mujoco_sim_node:main",
            "joy_servo_teleop = mujoco_rm_bringup.joy_servo_teleop_node:main",
        ],
    },
)
