from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    model_path = LaunchConfiguration("model_path")
    camera_name = LaunchConfiguration("camera_name")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    teleop_yaml = PathJoinSubstitution(
        [FindPackageShare("mujoco_rm_bringup"), "config", "teleop_twist_joy.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="/mujoco_rm_ws/models/walker/walker_soft_ball.xml",
                description="Path to MJCF model.",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="arm_camera",
                description="MuJoCo camera name to publish.",
            ),
            DeclareLaunchArgument("camera_width", default_value="640"),
            DeclareLaunchArgument("camera_height", default_value="480"),
            DeclareLaunchArgument("camera_fps", default_value="15.0"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[
                    {"publish_stamped_twist": False},
                    teleop_yaml,
                ],
                remappings=[("cmd_vel", "/cmd_vel")],
            ),
            Node(
                package="mujoco_rm_bringup",
                executable="mujoco_sim",
                name="mujoco_sim",
                output="screen",
                parameters=[
                    {"model_path": model_path},
                    {"camera_name": camera_name},
                    {"camera_width": camera_width},
                    {"camera_height": camera_height},
                    {"camera_fps": camera_fps},
                ],
            ),
        ]
    )
