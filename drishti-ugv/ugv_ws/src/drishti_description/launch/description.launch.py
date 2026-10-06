# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
#
# Publishes the robot description and the base_link -> * transforms.
#
# This node owns exactly one part of the TF tree (SPEC.md 3.2 rule 1):
#   base_link -> camera_link, camera_*_optical, imu_link, wheels
# It must NOT publish map -> odom (RTAB-Map owns that, Phase 2) or
# odom -> base_link (the Gazebo DiffDrive plugin owns that).
#
# !! UNVERIFIED !! Never executed -- no machine on the project has ROS 2.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('drishti_description'), 'urdf', 'drishti.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='SPEC.md 3.2 rule 4: true for every node against the simulator.'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
