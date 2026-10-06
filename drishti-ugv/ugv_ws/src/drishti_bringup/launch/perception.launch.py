# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
#
# Perception: detections, health and the nearest hazard distance.
#
# Until this runs, nothing publishes /perception/health, and the safety
# supervisor correctly holds the vehicle in STOP with reason CAMERA_STALE.
# That is why bringup.launch.py has a `supervisor:=false` escape for Phase 1
# bring-up.
#
# !! UNVERIFIED !! Never executed -- no machine on the project has ROS 2, and
# ultralytics is not installed.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('drishti_bringup'), 'config', 'perception.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'device', default_value='cuda:0',
            description='Set cpu to run without CUDA. Expect to miss the '
                        '100 ms perception budget in SPEC.md 8.'),

        Node(
            package='drishti_perception',
            executable='perception',
            name='perception',
            output='screen',
            emulate_tty=True,
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'device': LaunchConfiguration('device'),
            }],
        ),
    ])
