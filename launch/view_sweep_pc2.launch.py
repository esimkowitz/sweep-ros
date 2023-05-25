import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    sweep_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sweep_ros'), 'launch'),
            '/sweep.launch.py'])
        )
    rviz_node = Node(
        package='rviz2',
        name='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('sweep_ros'), 'rviz', 'sweep_pc2.rviz')],
    )
    return LaunchDescription([
        sweep_ld,
        rviz_node
    ])