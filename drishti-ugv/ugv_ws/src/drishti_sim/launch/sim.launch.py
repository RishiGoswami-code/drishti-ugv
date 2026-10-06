# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
#
# Starts Gazebo Harmonic, spawns the UGV, and bridges the simulator onto the
# SPEC.md section 4.1 ROS topic contract.
#
# Simulator choice is Gazebo Harmonic, not Isaac Sim -- STATUS.md D15.
#
# !! UNVERIFIED !! Never executed -- no machine on the project has ROS 2.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)

from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('drishti_sim')
    pkg_description = get_package_share_directory('drishti_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    # -r starts the world unpaused; -s is server-only for batch runs (Phase 6).
    #
    # BUG FOUND 27 Sep 2026, first real headless run: `headless` was declared
    # as a launch argument and documented as "server only, no GUI" but was
    # never actually read anywhere in this file -- gz_args always launched
    # the GUI client regardless of the flag. On a display-less machine (a
    # GitHub Actions runner, this project's own free-tier CI) that crashed
    # outright: `qt.qpa.xcb: could not connect to display`. Fixed by actually
    # conditioning -s on the flag.
    gz_args = [
        '-r ',
        PythonExpression(["'-s ' if '", headless, "' == 'true' else ''"]),
        PathJoinSubstitution([pkg_sim, 'worlds', world]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='easy.sdf',
            description='World file in drishti_sim/worlds. EVALUATION.md names '
                        'Easy, Medium, Hard, Dynamic, Adversarial and Failure.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Server only, no GUI. Use true for batch mission runs.'),
        DeclareLaunchArgument(
            'x', default_value='0.0', description='Spawn x.'),
        DeclareLaunchArgument(
            'y', default_value='0.0', description='Spawn y.'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0', description='Spawn yaw, radians.'),
        DeclareLaunchArgument(
            'z', default_value='0.15',
            description='Spawn height. Worlds with raised terrain need this '
                        'raised too: hard.sdf drives on a 0.45 m platform, so '
                        'pass z:=0.60 or the vehicle starts inside the ditch.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items(),
        ),

        # robot_state_publisher owns base_link -> * and nothing else.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # Spawn from the /robot_description topic so the simulator and
        # robot_state_publisher cannot disagree about the model.
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_drishti',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'drishti',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
            ],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            parameters=[{
                'config_file': os.path.join(pkg_sim, 'config', 'bridge.yaml'),
                'use_sim_time': True,
            }],
        ),
    ])
