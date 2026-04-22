from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marker_velocity_control_cpp',
            executable='md',
            name='controller_node',
            output='screen',
        ),
        Node(
            package='marker_velocity_control_cpp',
            executable='vc',
            name='body_rate_select_node',
            output='screen',
        ),
    ])
