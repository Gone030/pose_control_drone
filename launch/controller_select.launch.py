from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marker_velocity_control_cpp',
            executable='controller',
            name='controller_node',
            output='screen',
        ),
        Node(
            package='marker_velocity_control_cpp',
            executable='body_rate_select_node',
            name='body_rate_select_node',
            output='screen',
        ),
    ])
