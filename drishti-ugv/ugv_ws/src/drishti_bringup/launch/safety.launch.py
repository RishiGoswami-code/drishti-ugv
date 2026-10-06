# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
#
# Brings up the deterministic safety supervisor on its own.
#
# This is deliberately the first launch file in the project: the supervisor can
# and should be run before Nav2, perception or SLAM exist. With no inputs
# arriving it will sit in STOP and publish zero velocity on /cmd_vel, which is
# exactly the correct behaviour and makes the fail-safe path observable from
# day one (SPEC.md section 9.4.2).
#
#   ros2 launch drishti_bringup safety.launch.py
#   ros2 topic echo /safety/state
#
# !! UNVERIFIED !! Never executed -- no machine on the project has ROS 2
# installed yet (STATUS.md B3).

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('drishti_bringup'), 'config', 'drishti.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='SPEC.md 3.2 rule 4: true for every node against the '
                        'simulator. Set false only for a real-hardware run.'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='rclcpp logger level for the supervisor.'),

        Node(
            package='drishti_safety',
            executable='safety_supervisor',
            name='safety_supervisor',
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            # The supervisor owns /cmd_vel. Nothing is remapped onto it here,
            # and nothing else in the system may publish to it (SPEC.md 9.4.1).
        ),
    ])
