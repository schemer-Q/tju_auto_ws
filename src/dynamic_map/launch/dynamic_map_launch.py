import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dynamic_map'),
        'cfg',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='dynamic_map',
            executable='dynamic_map_node',
            name='dynamic_map_node',
            output='screen',
            parameters=[config]
        )
    ])
