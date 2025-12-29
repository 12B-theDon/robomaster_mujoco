from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robomaster_ros')
    param_file = os.path.join(pkg_share, 'config', 'episode_recorder.yaml')

    episode_node = Node(
        package='robomaster_ros',
        executable='episode_recorder',
        name='episode_recorder',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([episode_node])
