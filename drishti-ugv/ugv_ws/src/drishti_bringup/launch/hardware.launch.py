# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
#
# Hardware bring-up: the same stack, with real sensors instead of Gazebo.
#
# SPEC.md §12 is the whole point of the interface contract. Compare this file
# with sim.launch.py: what changes is where the messages come from and where
# /cmd_vel goes. SLAM, terrain, perception, Nav2 and the supervisor are
# launched from exactly the same files with exactly the same parameters.
#
#     Gazebo + ros_gz_bridge   -->  camera driver + base driver
#     use_sim_time: true       -->  use_sim_time: false
#     everything else          -->  unchanged
#
# If this file ever needs a remapping the simulator did not, or a node above it
# needs a different parameter on hardware, the contract has leaked and
# SPEC.md §4 should be fixed rather than patched around here.
#
# THE DRIVERS ARE NOT INCLUDED. Which camera and which base are unknown
# (STATUS.md A3: there is no vehicle), and guessing would produce a launch file
# that looks complete and works with nothing. Pass the vendor launch files in:
#
#   ros2 launch drishti_bringup hardware.launch.py \
#       camera_launch:=/opt/ros/jazzy/share/realsense2_camera/launch/rs_launch.py \
#       base_launch:=/path/to/base_driver.launch.py
#
# Their topics must be remapped onto the SPEC.md §4.1 names. config/hardware.yaml
# lists exactly which names, with nothing else in it.
#
# ===========================================================================
# DO NOT RUN THIS BEFORE THE BRING-UP SEQUENCE
#
# SPEC.md §12.1 lists nine steps and says "strictly in order". Enabling nav2
# here is step 8, and steps 1-7 -- bench-test the sensors, verify calibration
# and timestamps, verify TF while stationary, drive manually at very low speed,
# validate odometry, run SLAM without autonomous control, run perception while
# teleoperated -- all come first.
#
# From step 8 onward an EXTERNAL HARDWARE EMERGENCY STOP is mandatory
# (TASK.md Phase 7). The software supervisor is tested and deterministic, and
# it is still software on a computer that can lock up. It is not a substitute
# for a physical circuit that cuts motor power.
# ===========================================================================
#
# !! UNVERIFIED !! Never executed. There is no vehicle, and no machine on the
# project has ROS 2 (STATUS.md D17, A3).

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('drishti_bringup')
    pkg_description = get_package_share_directory('drishti_description')

    params = os.path.join(pkg_bringup, 'config', 'drishti.yaml')
    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2.yaml')

    # On hardware the clock is the wall clock. A node left on sim time would
    # timestamp against a /clock nobody publishes, and every staleness check in
    # the supervisor would fire at once.
    sim_time = 'false'

    nav2_on = LaunchConfiguration('nav2')
    estop = LaunchConfiguration('estop_confirmed')
    # Step 8 requires both: autonomy on, and the physical e-stop confirmed.
    nav2_permitted = PythonExpression(
        ["'", nav2_on, "' == 'true' and '", estop, "' == 'true'"])
    nav2_blocked = PythonExpression(
        ["'", nav2_on, "' == 'true' and '", estop, "' != 'true'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_launch', default_value='',
            description='Vendor camera driver launch file. Must publish the '
                        'SPEC.md 4.1 topics; see config/hardware.yaml.'),
        DeclareLaunchArgument(
            'base_launch', default_value='',
            description='Vendor base driver launch file. Must subscribe to '
                        '/cmd_vel and publish /odom.'),
        DeclareLaunchArgument(
            'v_max', default_value='0.35',
            description='Bring-up speed ceiling, m/s. SPEC.md 12.1 step 8 is '
                        'low speed; raise only after repeated collision-free '
                        'runs (step 9).'),
        DeclareLaunchArgument(
            'nav2', default_value='false',
            description='Autonomous navigation. Default false: steps 1-7 are '
                        'teleoperated, so this must be a deliberate choice.'),
        DeclareLaunchArgument(
            'estop_confirmed', default_value='false',
            description='Confirms an external hardware e-stop is wired and '
                        'tested. nav2 will not start without it.'),

        # ---- the parts that change -------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([LaunchConfiguration('camera_launch')]),
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('camera_launch'),
                                  "' != ''"])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([LaunchConfiguration('base_launch')]),
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('base_launch'),
                                  "' != ''"])),
        ),

        # ---- the parts that do not -------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'slam.launch.py')),
            launch_arguments={'use_sim_time': sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'terrain.launch.py')),
            launch_arguments={'use_sim_time': sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'perception.launch.py')),
            launch_arguments={'use_sim_time': sim_time}.items(),
        ),

        # The supervisor runs from step 1, long before anything is autonomous.
        # It owns /cmd_vel on hardware exactly as it does in simulation, and
        # starting it late would leave a window in which nothing does.
        Node(
            package='drishti_safety',
            executable='safety_supervisor',
            name='safety_supervisor',
            output='screen',
            emulate_tty=True,
            parameters=[params, {
                'use_sim_time': sim_time,
                'v_max': LaunchConfiguration('v_max'),
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py')),
            condition=IfCondition(nav2_permitted),
            launch_arguments={
                'use_sim_time': sim_time,
                'params_file': nav2_params,
                'use_composition': 'False',
            }.items(),
        ),

        LogInfo(
            condition=IfCondition(nav2_blocked),
            msg=('REFUSING to start Nav2: estop_confirmed is not true. '
                 'SPEC.md 12.1 step 8 requires an external hardware emergency '
                 'stop, wired and tested, before autonomous motion. The '
                 'software supervisor is not a substitute for a circuit that '
                 'cuts motor power. Pass estop_confirmed:=true only when it '
                 'physically exists.')),

        LogInfo(
            condition=UnlessCondition(nav2_on),
            msg=('Teleoperation mode: Nav2 is not running. This is bring-up '
                 'steps 1-7 of SPEC.md 12.1. The supervisor still owns '
                 '/cmd_vel and will stop the vehicle on stale sensors or lost '
                 'pose.')),
    ])
