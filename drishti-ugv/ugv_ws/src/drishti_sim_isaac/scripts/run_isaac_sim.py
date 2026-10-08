#!/usr/bin/env python3
# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Run DRISHTI-UGV in Isaac Sim and publish the SPEC.md section 4.1 contract.

Launch with Isaac Sim's own interpreter, not the system one:

    $ISAAC_SIM_PATH/python.sh run_isaac_sim.py --urdf drishti.urdf \
        --world-file easy.sdf [--headless] [--x 0 --y 0 --z 0.15 --yaw 0]

ros2 launch drishti_sim_isaac isaac_sim.launch.py does this and expands the xacro
first. Written against the Isaac Sim 6.1 API: node type names and attribute names
come from the node definition files shipped with 6.1 (isaacsim.ros2.nodes,
isaacsim.core.nodes), the structure from NVIDIA's standalone examples
(isaacsim.ros2.bridge/clock.py, camera_periodic.py, isaacsim.sensors.experimental
.physics/imu_sensor.py). Those APIs moved between 5.x and 6.x; check them before
running a different release.

What it does and does not own (SPEC.md 3.2, 4.3, 9.4.1):
  * It SUBSCRIBES to /cmd_vel and never publishes it. The safety supervisor is the
    only publisher there.
  * It publishes odom -> base_link on /tf and nothing else. base_link -> sensors
    stays with robot_state_publisher, so no TF edge has two publishers.
  * /ground_truth/pose is evaluation-only; nothing at run time may subscribe to it.

!! UNVERIFIED !! Never run: no machine on the project meets Isaac Sim's hardware
minimum (STATUS.md D21). Known gaps are listed in STATUS.md.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import scene_model as sm  # noqa: E402  (pure Python: parse before paying for startup)

parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--urdf", required=True, help="expanded URDF (xacro output)")
parser.add_argument("--world-file", required=True, help="SDF world, e.g. easy.sdf")
parser.add_argument("--config", default=os.path.join(HERE, "..", "config", "isaac_sim.json"))
parser.add_argument("--headless", action="store_true")
parser.add_argument("--x", type=float, default=0.0)
parser.add_argument("--y", type=float, default=0.0)
parser.add_argument("--z", type=float, default=0.15)
parser.add_argument("--yaw", type=float, default=0.0)
parser.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0 = run until closed)")
args, _ = parser.parse_known_args()

with open(args.config, encoding="utf-8") as f:
    cfg = json.load(f)
with open(args.urdf, encoding="utf-8") as f:
    robot = sm.parse_urdf_text(f.read())
with open(args.world_file, encoding="utf-8") as f:
    world = sm.parse_sdf_world_text(f.read())

names = cfg["sensors"]
for key in ("left_camera", "right_camera", "depth_camera", "imu"):
    if names[key] not in robot.sensors:
        sys.exit("robot description has no sensor %r (config sensors.%s)" % (names[key], key))
left = robot.sensors[names["left_camera"]]
right = robot.sensors[names["right_camera"]]
depth = robot.sensors[names["depth_camera"]]
imu = robot.sensors[names["imu"]]
if depth.frame_id != left.frame_id:
    sys.exit("depth must be registered to the left camera (SPEC.md 4.1)")
if robot.diff_drive is None:
    sys.exit("robot description has no DiffDrive parameters")
dd = robot.diff_drive
topics = {t["source"]: t["ros_topic_name"] for t in cfg["topics"]}
step_hz = float(cfg["simulation"]["step_hz"])
dt = 1.0 / step_hz

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"renderer": cfg["render"]["renderer"], "headless": args.headless})

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.sensors.experimental.physics import IMU  # noqa: E402
from pxr import UsdGeom  # noqa: E402

import usd_builder as ub  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

omni.usd.get_context().new_stage()
simulation_app.update()
stage = omni.usd.get_context().get_stage()
stage_utils.set_stage_units(meters_per_unit=1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# ----------------------------------------------------------------- the scene
ub.build_world(stage, world, cfg["lighting"])
spawn = ((args.x, args.y, args.z), sm.rpy_to_quat(0.0, 0.0, args.yaw))
built = ub.build_robot(stage, robot, spawn, cfg["wheel_drive"])
base = built["base"]

cam_left = ub.add_camera(stage, base, left, robot.link_pose(left.frame_id))
cam_right = ub.add_camera(stage, base, right, robot.link_pose(right.frame_id))

imu_path = base + "/imu_sensor"
IMU.create(imu_path, translations=np.array([list(robot.link_pose(imu.frame_id)[0])]))

simulation_app.update()


# ------------------------------------------------------------- ROS 2 graphs
def sdf_path(path):
    return [usdrt.Sdf.Path(path)]


def skip_for(rate_hz):
    """ROS2CameraHelper frameSkipCount for a target publish rate."""
    return max(0, int(round(step_hz / rate_hz)) - 1) if rate_hz > 0 else 0


keys = og.Controller.Keys
frames = cfg["frames"]
G = "/ROS2_Bridge"

og.Controller.edit(
    {"graph_path": G, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("PublishGroundTruth", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PublishJointStates", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointStates.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ReadImu.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
            ("ComputeOdom.outputs:execOut", "PublishOdomTF.inputs:execIn"),
            ("ComputeOdom.outputs:execOut", "PublishGroundTruth.inputs:execIn"),
            ("ReadImu.outputs:execOut", "PublishImu.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishGroundTruth.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointStates.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),
            ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:position", "PublishOdomTF.inputs:translation"),
            ("ComputeOdom.outputs:orientation", "PublishOdomTF.inputs:rotation"),
            ("ReadImu.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
            ("ReadImu.outputs:angVel", "PublishImu.inputs:angularVelocity"),
            ("ReadImu.outputs:orientation", "PublishImu.inputs:orientation"),
        ],
        keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", topics["clock"]),
            ("ComputeOdom.inputs:chassisPrim", sdf_path(base)),
            ("PublishOdom.inputs:topicName", topics["odometry"]),
            ("PublishOdom.inputs:odomFrameId", frames["odom"]),
            ("PublishOdom.inputs:chassisFrameId", frames["base"]),
            ("PublishOdomTF.inputs:topicName", topics["odom_tf"]),
            ("PublishOdomTF.inputs:parentFrameId", frames["odom"]),
            ("PublishOdomTF.inputs:childFrameId", frames["base"]),
            ("PublishGroundTruth.inputs:topicName", topics["ground_truth"]),
            ("PublishGroundTruth.inputs:odomFrameId", frames["world"]),
            ("PublishGroundTruth.inputs:chassisFrameId", frames["base"]),
            ("PublishJointStates.inputs:topicName", topics["joint_states"]),
            ("PublishJointStates.inputs:targetPrim", sdf_path(base)),
            ("ReadImu.inputs:imuPrim", sdf_path(imu_path)),
            ("PublishImu.inputs:topicName", topics["imu"]),
            ("PublishImu.inputs:frameId", imu.frame_id),
            ("SubscribeTwist.inputs:topicName", topics["cmd_vel"]),
        ],
    },
)


def camera_graph(graph_path, camera_path, sensor, helpers):
    """On-demand camera pipeline, the pattern in NVIDIA's camera_periodic.py.

    helpers: list of (node name, node type, {attribute: value}).
    """
    skip = skip_for(sensor.rate_hz)
    nodes = [("OnTick", "omni.graph.action.OnTick"),
             ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct")]
    connect = [("OnTick.outputs:tick", "createRenderProduct.inputs:execIn")]
    values = [("createRenderProduct.inputs:cameraPrim", sdf_path(camera_path)),
              ("createRenderProduct.inputs:width", sensor.width),
              ("createRenderProduct.inputs:height", sensor.height)]
    for name, node_type, attrs in helpers:
        nodes.append((name, node_type))
        connect.append(("createRenderProduct.outputs:execOut", "%s.inputs:execIn" % name))
        connect.append(("createRenderProduct.outputs:renderProductPath",
                        "%s.inputs:renderProductPath" % name))
        values.append(("%s.inputs:frameSkipCount" % name, skip))
        values.append(("%s.inputs:frameId" % name, sensor.frame_id))
        values.extend(("%s.inputs:%s" % (name, k), v) for k, v in attrs.items())
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "push",
         "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {keys.CREATE_NODES: nodes, keys.CONNECT: connect, keys.SET_VALUES: values})
    og.Controller.evaluate_sync(graph)


HELPER = "isaacsim.ros2.bridge.ROS2CameraHelper"
INFO = "isaacsim.ros2.bridge.ROS2CameraInfoHelper"

# Depth, its point cloud and both camera_info topics are registered to the left camera.
camera_graph("/ROS2_Camera_Left", cam_left, left, [
    ("rgb", HELPER, {"topicName": topics["left_rgb"], "type": "rgb"}),
    ("depth", HELPER, {"topicName": topics["depth_image"], "type": "depth"}),
    ("points", HELPER, {"topicName": topics["depth_points"], "type": "depth_pcl"}),
    ("info", INFO, {"topicName": topics["left_info"]}),
    ("depthInfo", INFO, {"topicName": topics["depth_info"]}),
])
camera_graph("/ROS2_Camera_Right", cam_right, right, [
    ("rgb", HELPER, {"topicName": topics["right_rgb"], "type": "rgb"}),
    ("info", INFO, {"topicName": topics["right_info"]}),
])
simulation_app.update()

# ----------------------------------------------------------------- the loop
SimulationManager.setup_simulation(dt=dt, device=cfg["simulation"]["device"])
vehicle = Articulation(base)
app_utils.play()
simulation_app.update()
app_utils.update_app(steps=2)

left_idx = vehicle.get_dof_indices(dd.left_joints).numpy()
right_idx = vehicle.get_dof_indices(dd.right_joints).numpy()

attr = og.Controller.attribute
cmd_lin = attr(G + "/SubscribeTwist.outputs:linearVelocity")
cmd_ang = attr(G + "/SubscribeTwist.outputs:angularVelocity")
odom_pos = attr(G + "/ComputeOdom.outputs:position")
odom_quat = attr(G + "/ComputeOdom.outputs:orientation")
odom_lin = attr(G + "/ComputeOdom.outputs:linearVelocity")
odom_ang = attr(G + "/ComputeOdom.outputs:angularVelocity")
gt = {n: attr(G + "/PublishGroundTruth.inputs:" + n)
      for n in ("position", "orientation", "linearVelocity", "angularVelocity")}

print("[drishti] Isaac Sim backend up: world=%s step=%.0f Hz left=%d right=%d dofs=%d"
      % (world.name, step_hz, len(left_idx), len(right_idx), vehicle.num_dofs))

max_dv = dd.max_linear_acceleration * dt
v_applied = 0.0
frame = 0
while simulation_app.is_running():
    simulation_app.update()
    if not app_utils.is_playing():
        continue

    # Linear acceleration is limited as the Gazebo DiffDrive plugin limits it.
    lin, ang = cmd_lin.get(), cmd_ang.get()
    v_applied = sm.slew(v_applied, float(lin[0]), max_dv)
    wl, wr = sm.wheel_speeds(v_applied, float(ang[2]), dd.wheel_radius, dd.wheel_separation)
    targets = np.zeros(vehicle.num_dofs)
    targets[left_idx] = wl
    targets[right_idx] = wr
    vehicle.set_dof_velocity_targets(targets)

    # Ground truth: spawn pose composed with the start-relative odometry.
    pos, quat = sm.compose_ground_truth(spawn, tuple(odom_pos.get()), tuple(odom_quat.get()))
    og.Controller.set(gt["position"], list(pos))
    og.Controller.set(gt["orientation"], list(quat))
    og.Controller.set(gt["linearVelocity"], list(odom_lin.get()))
    og.Controller.set(gt["angularVelocity"], list(odom_ang.get()))

    frame += 1
    if args.max_frames and frame >= args.max_frames:
        break

app_utils.stop()
simulation_app.close()
