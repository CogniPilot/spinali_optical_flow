# Copyright (c) 2025 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_spinali_optical_flow_description = Path(get_package_share_directory('spinali_optical_flow_description'))
    urdf_file = pkg_spinali_optical_flow_description / 'urdf'/ 'spinali_optical_flow.urdf'
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_desc},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              choices=['true', 'false'],
                              description='use_sim_time')])

    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    return ld
