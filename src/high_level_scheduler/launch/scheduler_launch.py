from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='high_level_scheduler',
            executable='scheduler_node',
            name='scheduler_node',
            output='screen',
        )
    ])
