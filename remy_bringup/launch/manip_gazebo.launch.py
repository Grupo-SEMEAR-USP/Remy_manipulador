"""Inicia o manipulador isolado no Gazebo com MoveIt e câmera fictícia."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    y_tol_arg = DeclareLaunchArgument(
        'y_tol',
        default_value='0.02',
        description='Tolerância lateral provisória, em metros.',
    )

    def incluir_launch(pacote, arquivo):
        caminho = os.path.join(get_package_share_directory(pacote), 'launch', arquivo)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(caminho),
            launch_arguments={'use_sim_time': 'true'}.items(),
        )

    gazebo = incluir_launch('manip_revis_description', 'gazebo.launch.py')
    moveit = incluir_launch('manip_revis_moveit_config', 'moveit.launch.py')

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_fake_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
        ],
        parameters=[{'use_sim_time': True}],
    )

    move_arm = Node(
        package='manip_manipulation',
        executable='move_arm',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'y_tol': ParameterValue(LaunchConfiguration('y_tol'), value_type=float),
        }],
    )

    return LaunchDescription([y_tol_arg, gazebo, moveit, camera_tf, move_arm])
