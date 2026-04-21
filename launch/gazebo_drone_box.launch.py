from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    cam_models_path = os.path.expanduser('~/ros2_ws/src/cam')
    iris_model = os.path.join(cam_models_path, 'iris_fpv_cam', 'iris_fpv_cam.sdf')
    box_model = os.path.join(cam_models_path, 'box', 'model.sdf')

    gazebo_model_path = os.pathsep.join(filter(None, [
        os.environ.get('GAZEBO_MODEL_PATH'),
        cam_models_path,
    ]))

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])
            ),
            launch_arguments={
                'world': PathJoinSubstitution([gazebo_ros_share, 'worlds', 'empty.world']),
            }.items(),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'iris_fpv_cam',
                '-file', iris_model,
                '-x', '-5',
                '-y', '0',
                '-z', '0.3',
                '-Y', '0',
            ],
            output='screen',
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'small_box_cam_target',
                '-file', box_model,
                '-x', '10',
                '-y', '0',
                '-z', '1',
                '-Y', '3.14',
            ],
            output='screen',
        ),
    ])
