# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
#
# RTAB-Map stereo SLAM. Supplies map -> odom and /map, and nothing else.
#
# TF ownership after this launch (SPEC.md 3.2 rule 1):
#   map  -> odom        rtabmap, here
#   odom -> base_link   Gazebo DiffDrive, via the bridge
#   base_link -> *      robot_state_publisher
#
# NO GNSS and no simulator ground truth anywhere in this graph, including
# initialisation (CONTRIBUTING.md non-negotiable 5). /ground_truth/pose is recorded
# to the bag for drishti_eval to read afterwards; nothing subscribes to it
# live. tools/check_wiring.py checks that statically.
#
# !! UNVERIFIED !! Never executed -- no machine on the project has ROS 2.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('drishti_bringup'), 'config', 'rtabmap.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    # SPEC.md 4.1 names. Remapped here rather than in the config so the topic
    # contract stays readable in one place.
    stereo_remaps = [
        ('left/image_rect', '/camera/rgb/image_raw'),
        ('right/image_rect', '/camera/right/image_raw'),
        ('left/camera_info', '/camera/camera_info'),
        ('right/camera_info', '/camera/right/camera_info'),
        ('imu', '/imu/data'),
        ('odom', '/odom'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='true to localise against an existing database instead '
                        'of mapping. Phase 2 maps; later phases may localise.'),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='false',
            description='Diagnostic viewer. Off by default: it is expensive and '
                        'the stack must never depend on it.'),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='true',
            description='Start from an empty map. Keeping a database between '
                        'runs would let one mission quietly improve the next '
                        'one, which would make the Phase 6 suite dishonest.'),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'Mem/IncrementalMemory': 'false' if localization else 'true',
            }],
            remappings=stereo_remaps,
            arguments=['--delete_db_on_start'],
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=stereo_remaps,
        ),
    ])
