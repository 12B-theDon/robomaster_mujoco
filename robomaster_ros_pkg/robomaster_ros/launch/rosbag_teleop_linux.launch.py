from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bag_path = LaunchConfiguration('bag_path')
    loop     = LaunchConfiguration('loop')  # 'true' or 'false'

    piloting_yaml = PathJoinSubstitution([
        FindPackageShare('robomaster_ros'), 'config', 'xbox_piloting.yaml'
    ])
    joymap_yaml = PathJoinSubstitution([
        FindPackageShare('robomaster_ros'), 'config', 'xbox_joy_config_ep.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value=''),
        DeclareLaunchArgument('loop', default_value='true'),

        # ros2 bag play (loop ON)  → 반드시 --clock 포함
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--rate', '1.0'],
            output='screen',
            condition=IfCondition(loop)
        ),
        # ros2 bag play (loop OFF) → 여기에도 --clock 포함!
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
            output='screen',
            condition=UnlessCondition(loop)
        ),

        # teleop_twist_joy (cmd_vel)  — sim time 사용 + YAML 로드
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'use_sim_time': True}, {'autorepeat_rate': 0.0}, piloting_yaml],
        ),

        # joy_teleop (LED/그리퍼/팔) — sim time 사용 + 매핑 YAML
        Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'use_sim_time': True}, {'autorepeat_rate': 0.0}, joymap_yaml],
        ),
    ])
