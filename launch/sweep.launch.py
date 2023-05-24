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
        executable='sweep_node',
        name='sweep_node',
        parameters=[config],
        output='screen'
    )

    ld.add_action(sweep_node)
    return ld