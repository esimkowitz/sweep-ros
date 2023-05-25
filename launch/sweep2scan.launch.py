import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sweep_ros'),
        'config',
        'config.yaml'
        )

    return LaunchDescription([
        Node(
            package='sweep_ros',
            name='sweep_node',
            executable='sweep_node',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            parameters=[config]
        )
    ])