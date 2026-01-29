import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('path_planner'),
        'cfg',
        'config.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('path_planner'),
        'rviz',
        'planner_view.rviz'
    )

    return LaunchDescription([
        Node(
            package='path_planner',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
