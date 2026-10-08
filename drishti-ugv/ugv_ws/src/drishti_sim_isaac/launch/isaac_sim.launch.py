# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
#
# Starts Isaac Sim on one of the Easy/Medium/Hard worlds with the UGV spawned,
# publishing the SPEC.md section 4.1 contract, plus robot_state_publisher.
#
# The counterpart of drishti_sim/launch/sim.launch.py and takes the same `world`,
# `headless`, `x`, `y`, `z` and `yaw` arguments, so drishti_bringup can choose
# between the two with `sim:=isaac|gazebo`.
#
#   export ISAAC_SIM_PATH=/path/to/isaac-sim     # contains python.sh
#   ros2 launch drishti_sim_isaac isaac_sim.launch.py world:=easy.sdf
#
# Isaac Sim is started with its own python.sh. Source ROS 2 Jazzy in the shell
# first, so Isaac Sim picks up the system ROS libraries (see the Isaac Sim ROS 2
# installation notes: LD_LIBRARY_PATH must be right before the process starts).
#
# The xacro is expanded here, with the real xacro tool, and the result handed to
# the runtime script; the worlds are the SDF files in drishti_sim/worlds.
#
# !! UNVERIFIED !! Never executed -- no machine on the project can run Isaac Sim
# (STATUS.md D21).

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def launch_setup(context):
    isaac_path = LaunchConfiguration('isaac_sim_path').perform(context)
    if not isaac_path:
        raise RuntimeError(
            'Isaac Sim not found: set ISAAC_SIM_PATH or pass isaac_sim_path:=<dir with python.sh>')
    python_sh = os.path.join(isaac_path, 'python.sh')

    pkg_isaac = get_package_share_directory('drishti_sim_isaac')
    pkg_description = get_package_share_directory('drishti_description')
    pkg_sim = get_package_share_directory('drishti_sim')

    urdf_xml = xacro.process_file(
        os.path.join(pkg_description, 'urdf', 'drishti.urdf.xacro')).toxml()
    fd, urdf_path = tempfile.mkstemp(prefix='drishti_', suffix='.urdf')
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        f.write(urdf_xml)

    world = LaunchConfiguration('world').perform(context)
    cmd = [
        python_sh, os.path.join(pkg_isaac, 'scripts', 'run_isaac_sim.py'),
        '--urdf', urdf_path,
        '--world-file', os.path.join(pkg_sim, 'worlds', world),
        '--config', os.path.join(pkg_isaac, 'config', 'isaac_sim.json'),
        '--x', LaunchConfiguration('x').perform(context),
        '--y', LaunchConfiguration('y').perform(context),
        '--z', LaunchConfiguration('z').perform(context),
        '--yaw', LaunchConfiguration('yaw').perform(context),
    ]
    if LaunchConfiguration('headless').perform(context).lower() == 'true':
        cmd.append('--headless')
    return [ExecuteProcess(cmd=cmd, name='isaac_sim', output='screen')]


def generate_launch_description():
    pkg_description = get_package_share_directory('drishti_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'isaac_sim_path',
            default_value=EnvironmentVariable('ISAAC_SIM_PATH', default_value=''),
            description='Isaac Sim install directory (contains python.sh).'),
        DeclareLaunchArgument(
            'world', default_value='easy.sdf',
            description='World file in drishti_sim/worlds (shared with the Gazebo backend).'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='No window. Rendering still runs on the GPU, so cameras still publish.'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x.'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw, radians.'),
        DeclareLaunchArgument(
            'z', default_value='0.15',
            description='Spawn height. hard.sdf drives on a 0.45 m platform: pass z:=0.60.'),

        # robot_state_publisher owns base_link -> * and nothing else.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        OpaqueFunction(function=launch_setup),
    ])
