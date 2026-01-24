from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='$(find-pkg-share path_planner)/rviz/planner_view.rviz',
        description='RViz config file'
    )
    return LaunchDescription([
        config_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen'
        )
    ])
