import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('sweep_ros'),
        'config',
        'config.yaml'
        )

    sweep_node = Node(
        package='sweep_ros',
        name='sweep_node',
        executable='sweep_node',
        parameters=[config],
        output='screen'
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        parameters=[config]
    )

    ld.add_action(sweep_node)
    ld.add_action(pointcloud_to_laserscan_node)
    return ld