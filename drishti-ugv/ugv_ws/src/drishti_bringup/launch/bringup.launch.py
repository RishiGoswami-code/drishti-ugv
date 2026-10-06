# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
#
# Phase 1 bringup: simulator + Nav2 + the safety supervisor.
#
# The command chain is deliberately explicit here, because it is the thing
# most likely to be broken by a well-meaning edit:
#
#     Nav2 controller ---> /cmd_vel_nav ---> safety_supervisor ---> /cmd_vel
#                                                                     |
#                                                          ros_gz_bridge
#                                                                     v
#                                                          Gazebo DiffDrive
#
# Nav2 has no route to /cmd_vel. The supervisor is the only publisher on it
# (SPEC.md 4.3, 9.4.1). tools/check_wiring.py enforces this statically;
# `ros2 topic info /cmd_vel --verbose` must confirm it at run time.
#
# Not included yet: RTAB-Map (Phase 2), elevation mapping and the
# traversability layer (Phase 3), perception (Phase 4). Until Phase 4 exists
# nothing publishes /perception/health, so the supervisor will hold the
# vehicle in STOP. That is correct, not a bug -- run with
# supervisor:=false to drive during Phase 1 bring-up.
#
# !! UNVERIFIED !! Never executed -- no machine on the project has ROS 2.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('drishti_bringup')
    pkg_sim = get_package_share_directory('drishti_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_bringup, 'config', 'drishti.yaml')
    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='easy.sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'supervisor', default_value='true',
            description='Start the safety supervisor. false only for Phase 1 '
                        'bring-up, when no perception exists to keep it happy.'),
        DeclareLaunchArgument(
            'nav2', default_value='true',
            description='Start Nav2. false to teleoperate.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'sim.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'headless': LaunchConfiguration('headless'),
            }.items(),
        ),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('nav2')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'params_file': nav2_params,
                        # No map server: there is no prior map. RTAB-Map
                        # provides map -> odom from Phase 2 onward.
                        'use_composition': 'False',
                    }.items(),
                ),
            ],
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('supervisor')),
            package='drishti_safety',
            executable='safety_supervisor',
            name='safety_supervisor',
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': True}],
        ),
    ])
