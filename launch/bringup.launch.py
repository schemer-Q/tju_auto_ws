from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='dynamic_map', executable='dynamic_map_node', name='dynamic_map_node', output='screen'),
        Node(package='high_level_scheduler', executable='scheduler_node', name='scheduler_node', output='screen'),
        Node(package='path_planner', executable='planner_node', name='planner_node', output='screen'),
    ])
