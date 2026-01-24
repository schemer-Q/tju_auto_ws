from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='planner_node',
            name='planner_node',
            output='screen',
        )
    ])
