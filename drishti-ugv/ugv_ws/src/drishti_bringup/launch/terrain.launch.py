# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
#
# Terrain pipeline: point cloud -> elevation map -> traversability.
#
#   /camera/points  --> elevation_mapping_cupy --> /elevation_map
#                                                       |
#                                        traversability_fusion
#                                                       |
#                                                /traversability
#                                                       |
#                                     Nav2 TraversabilityLayer (nav2.yaml)
#
# elevation_mapping_cupy needs CUDA. STATUS.md D16 restored it to the primary
# path once an RTX 3050 became available; on a machine without an NVIDIA GPU
# this launch cannot work at all, and the CPU grid_map fallback in SPEC.md §2
# would be needed instead.
#
# SLOPE, ROUGHNESS AND STEP ARE NOT PUBLISHED BY DEFAULT. They come from
# elevation_mapping_cupy plugins that must be enabled in its own configuration.
# Without them the fusion node prices those cells as unobserved -- expensive,
# never free -- and warns once at startup naming each missing layer. That is
# the safe failure, but it is still a failure: check the warnings on first run
# rather than wondering why every cell costs 0.85.
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
    pkg_bringup = get_package_share_directory('drishti_bringup')
    traversability_params = os.path.join(
        pkg_bringup, 'config', 'traversability.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'elevation_mapping', default_value='true',
            description='Start elevation_mapping_cupy. Requires CUDA. Set '
                        'false to run the fusion node against an elevation map '
                        'from another source, or from a bag.'),

        # elevation_mapping_cupy carries its own YAML in its own share
        # directory and its own plugin list; it is configured there, not here.
        # Only the interface matters to us: it must publish /elevation_map in
        # a frame the fusion node can resolve.
        Node(
            condition=IfCondition(LaunchConfiguration('elevation_mapping')),
            package='elevation_mapping_cupy',
            executable='elevation_mapping_node',
            name='elevation_mapping',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/points', '/camera/points'),
                ('/elevation_map_raw', '/elevation_map'),
            ],
        ),

        Node(
            package='drishti_traversability',
            executable='traversability_fusion',
            name='traversability_fusion',
            output='screen',
            emulate_tty=True,
            parameters=[traversability_params, {'use_sim_time': use_sim_time}],
        ),
    ])
