from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_map',
            executable='dynamic_map_node',
            name='dynamic_map_node',
            output='screen',
        )
    ])
