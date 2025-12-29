from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robomaster_ros',
            executable='robomaster_driver',  # setup.py entry_points에 등록된 이름
            name='robomaster',
            output='screen',
        ),
    ])
