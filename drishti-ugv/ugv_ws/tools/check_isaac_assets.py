#!/usr/bin/env python3
"""Validate the Isaac Sim backend without Isaac Sim.

Nothing here proves the backend runs in Isaac Sim. It checks what can go wrong
silently before anyone has a machine that can run it:

  1. the backend publishes exactly the ROS topics, types and directions the
     Gazebo bridge does, so SPEC.md 4.1 holds whichever simulator is selected
  2. /cmd_vel is the only inbound topic and the ground truth is outbound only
  3. the real robot description (xacro expanded) parses, its wheels, sensors and
     drive parameters agree with each other, and its optical frames have the
     SPEC.md 3.2 orientation
  4. a USD camera built from an optical frame looks forward with up as up
  5. every SDF world parses into supported geometry, with collision matching visual
  6. the frame, drive and ground-truth maths behave
  7. the runtime scripts are valid Python and cannot publish /cmd_vel

    python tools/check_isaac_assets.py
"""
import glob
import json
import math
import os
import sys
from xml.etree import ElementTree as ET

import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(WS, "src")
ISAAC = os.path.join(SRC, "drishti_sim_isaac")
sys.path.insert(0, os.path.join(ISAAC, "scripts"))

import scene_model as sm  # noqa: E402
from xacro_subset import XACRO_NS, expand  # noqa: E402

failures = []


def fail(msg):
    failures.append(msg)
    print("  FAIL  " + msg)


def ok(msg):
    print("  ok    " + msg)


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def vec_close(a, b, tol=1e-9):
    return all(close(x, y, tol) for x, y in zip(a, b))


# --------------------------------------------------------------- 1 and 2: seam
print("1. topic parity with the Gazebo bridge")
cfg = json.load(open(os.path.join(ISAAC, "config", "isaac_sim.json"), encoding="utf-8"))
bridge = yaml.safe_load(open(os.path.join(SRC, "drishti_sim", "config", "bridge.yaml"),
                             encoding="utf-8"))
DIRECTION = {"GZ_TO_ROS": "SIM_TO_ROS", "ROS_TO_GZ": "ROS_TO_SIM"}
gz = {(e["ros_topic_name"], e["ros_type_name"], DIRECTION.get(e["direction"], e["direction"]))
      for e in bridge}
isaac = {(t["ros_topic_name"], t["ros_type_name"], t["direction"]) for t in cfg["topics"]}
if gz != isaac:
    for item in sorted(gz - isaac):
        fail("Gazebo bridges %s but the Isaac backend does not" % (item,))
    for item in sorted(isaac - gz):
        fail("Isaac backend publishes %s but the Gazebo bridge does not" % (item,))
else:
    ok("%d topics, identical names, types and directions" % len(isaac))
sources = [t["source"] for t in cfg["topics"]]
if len(sources) != len(set(sources)):
    fail("duplicate source ids in the topic list")

print("\n2. command and ground-truth direction")
inbound = [t for t in cfg["topics"] if t["direction"] == "ROS_TO_SIM"]
if [t["ros_topic_name"] for t in inbound] != ["/cmd_vel"]:
    fail("the only ROS_TO_SIM topic must be /cmd_vel, found %s"
         % [t["ros_topic_name"] for t in inbound])
else:
    ok("/cmd_vel is the single inbound topic")
gt = [t for t in cfg["topics"] if "ground_truth" in t["ros_topic_name"]]
if len(gt) != 1 or gt[0]["direction"] != "SIM_TO_ROS":
    fail("ground truth must be one SIM_TO_ROS topic")
else:
    ok("ground truth is outbound only")

# --------------------------------------------------------------------- 3: robot
print("\n3. robot description")
urdf_path = os.path.join(SRC, "drishti_description", "urdf", "drishti.urdf.xacro")
root_in = ET.parse(urdf_path).getroot()
expanded = ET.Element("robot", {"name": root_in.get("name", "")})
expand(root_in, {}, expanded, on_error=fail)
robot = sm.parse_urdf_text(ET.tostring(expanded, encoding="unicode"))
ok("parsed: %d links, %d joints, %d sensors" % (len(robot.links), len(robot.joints), len(robot.sensors)))

moving = robot.moving_joints()
if len(moving) != 4 or any(j.kind != "continuous" or not vec_close(j.axis, (0, 1, 0)) for j in moving):
    fail("expected four continuous wheel joints about +Y, found %s"
         % [(j.name, j.kind, j.axis) for j in moving])
else:
    ok("four continuous wheel joints about +Y")

dd = robot.diff_drive
if dd is None:
    fail("no DiffDrive parameters in the description")
else:
    left = sorted(j.name for j in moving if j.origin[0][1] > 0)
    right = sorted(j.name for j in moving if j.origin[0][1] < 0)
    if sorted(dd.left_joints) != left or sorted(dd.right_joints) != right:
        fail("DiffDrive left/right joints %s/%s disagree with wheel positions %s/%s"
             % (sorted(dd.left_joints), sorted(dd.right_joints), left, right))
    else:
        ok("DiffDrive left/right joint lists match the +y/-y wheels")
    sep = max(j.origin[0][1] for j in moving) - min(j.origin[0][1] for j in moving)
    if not close(sep, dd.wheel_separation, 1e-6):
        fail("wheel_separation %.4f disagrees with the wheel joints (%.4f)" % (dd.wheel_separation, sep))
    radii = {robot.links[j.child].visuals[0].dims[0] for j in moving}
    if radii != {dd.wheel_radius}:
        fail("wheel_radius %.4f disagrees with the wheel geometry %s" % (dd.wheel_radius, radii))
    if not failures:
        ok("wheel radius %.3f m and separation %.3f m agree with the geometry" % (dd.wheel_radius, sep))

names = cfg["sensors"]
missing = [v for v in names.values() if v not in robot.sensors]
if missing:
    fail("config names sensors the description lacks: %s" % missing)
else:
    ok("config sensors exist: %s" % ", ".join(sorted(names.values())))
    cams = [robot.sensors[names[k]] for k in ("left_camera", "right_camera", "depth_camera")]
    for s in cams:
        if s.frame_id not in robot.links:
            fail("sensor %s frame %r is not a link" % (s.name, s.frame_id))
        if not (s.width and s.height and s.hfov and s.rate_hz):
            fail("sensor %s lacks resolution, field of view or rate" % s.name)
    if robot.sensors[names["depth_camera"]].frame_id != robot.sensors[names["left_camera"]].frame_id:
        fail("depth must be registered to the left optical frame")
    else:
        ok("depth is registered to the left optical frame")

moving_children = {j.child for j in moving}
bad = []
for name, link in robot.links.items():
    for c in link.collisions:
        if not any(v.kind == c.kind and all(close(a, b) for a, b in zip(v.dims, c.dims))
                   for v in link.visuals):
            bad.append(name)
if bad:
    fail("collision geometry differs from visual on %s; the builder uses the visual" % bad)
else:
    ok("collision geometry equals visual geometry")

base_mass = robot.links[robot.base_link].mass
if not base_mass < robot.lumped_mass() < base_mass + 1.0:
    fail("lumped base mass %.3f looks wrong against base %.3f" % (robot.lumped_mass(), base_mass))
else:
    ok("base mass %.2f kg lumps to %.2f kg with the fixed mounts" % (base_mass, robot.lumped_mass()))

# --------------------------------------------------------------- 3 and 4: frames
print("\n4. optical frames and USD cameras")
for key in ("left_camera", "right_camera"):
    if names[key] not in robot.sensors:
        continue
    s = robot.sensors[names[key]]
    p = robot.link_pose(s.frame_id)
    z = sm.quat_rotate(p[1], (0, 0, 1))
    x = sm.quat_rotate(p[1], (1, 0, 0))
    y = sm.quat_rotate(p[1], (0, 1, 0))
    if (vec_close(z, (1, 0, 0), 1e-9) and vec_close(x, (0, -1, 0), 1e-9)
            and vec_close(y, (0, 0, -1), 1e-9)):
        ok("%s optical frame: z forward, x right, y down in base_link" % s.frame_id)
    else:
        fail("%s optical axes in base_link are x=%s y=%s z=%s" % (s.frame_id, x, y, z))
    q = sm.usd_camera_pose(p)[1]
    view = sm.quat_rotate(q, (0, 0, -1))
    up = sm.quat_rotate(q, (0, 1, 0))
    if vec_close(view, (1, 0, 0), 1e-9) and vec_close(up, (0, 0, 1), 1e-9):
        ok("%s USD camera looks along +x with +z up" % s.name)
    else:
        fail("%s USD camera looks along %s with up %s" % (s.name, view, up))
    if robot.link_pose(s.frame_id)[0][0] <= 0.0:
        fail("%s is not in front of the base centre" % s.name)

# ----------------------------------------------------------------------- worlds
print("\n5. worlds")
worlds = sorted(glob.glob(os.path.join(SRC, "drishti_sim", "worlds", "*.sdf")))
if not worlds:
    fail("no worlds found")
for path in worlds:
    label = os.path.basename(path)
    try:
        w = sm.parse_sdf_world_text(open(path, encoding="utf-8").read())
    except Exception as exc:  # noqa: BLE001
        fail("%s does not parse into supported geometry: %s" % (label, exc))
        continue
    planes = [p for p in w.prims if p.kind == "plane"]
    solids = [p for p in w.prims if p.kind != "plane"]
    problems = []
    if len(planes) != 1:
        problems.append("%d ground planes" % len(planes))
    if not solids:
        problems.append("no obstacles")
    if w.sun is None:
        problems.append("no directional light")
    if w.gravity[2] >= 0.0:
        problems.append("gravity does not point down")
    mismatched = [p.name for p in w.prims if not p.collision_matches_visual]
    if mismatched:
        problems.append("collision differs from visual on %s" % mismatched)
    if problems:
        fail("%s: %s" % (label, "; ".join(problems)))
    else:
        ok("%-11s %d solids on one ground plane, sun and gravity present" % (label, len(solids)))

# ------------------------------------------------------------------------ maths
print("\n6. frame, drive and ground-truth maths")
q = sm.rpy_to_quat(0.0, 0.0, math.pi / 2)
if vec_close(sm.quat_rotate(q, (1, 0, 0)), (0, 1, 0), 1e-12):
    ok("yaw of 90 degrees takes +x to +y")
else:
    fail("yaw rotation is wrong")
q = sm.rpy_to_quat(0.3, -0.2, 1.1)
manual = sm.quat_mul(sm.rpy_to_quat(0, 0, 1.1),
                     sm.quat_mul(sm.rpy_to_quat(0, -0.2, 0), sm.rpy_to_quat(0.3, 0, 0)))
if vec_close(q, manual, 1e-12):
    ok("rpy is R = Rz * Ry * Rx")
else:
    fail("rpy composition differs from Rz * Ry * Rx")
d = (-0.6, 0.4, -0.7)
n = math.sqrt(sum(c * c for c in d))
if vec_close(sm.quat_rotate(sm.look_rotation_neg_z(d), (0, 0, -1)), tuple(c / n for c in d), 1e-9):
    ok("light orientation takes -z onto the SDF direction")
else:
    fail("look_rotation_neg_z does not point -z along the direction")

l, r = sm.wheel_speeds(1.0, 0.0, 0.1, 0.44)
fwd = close(l, 10.0) and close(r, 10.0)
l, r = sm.wheel_speeds(0.0, 1.0, 0.1, 0.44)
turn = close(l, -2.2) and close(r, 2.2)
if fwd and turn:
    ok("forward drives both sides together; positive yaw rate speeds the right side")
else:
    fail("wheel_speeds sign convention is wrong")
if (close(sm.slew(0.0, 5.0, 0.1), 0.1) and close(sm.slew(0.0, -5.0, 0.1), -0.1)
        and close(sm.slew(0.0, 0.05, 0.1), 0.05)):
    ok("slew limits the step and passes small changes through")
else:
    fail("slew is wrong")

spawn = ((2.0, 3.0, 0.15), sm.rpy_to_quat(0.0, 0.0, math.pi / 2))
pos, quat = sm.compose_ground_truth(spawn, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
if vec_close(pos, (2.0, 4.0, 0.15), 1e-12) and vec_close(quat, sm.xyzw(spawn[1]), 1e-12):
    ok("ground truth composes the spawn pose with start-relative odometry")
else:
    fail("compose_ground_truth gave %s %s" % (pos, quat))

# ---------------------------------------------------------------- 7: the scripts
print("\n7. runtime scripts")
scripts = [os.path.join(ISAAC, "scripts", n) for n in ("scene_model.py", "usd_builder.py", "run_isaac_sim.py")]
scripts.append(os.path.join(ISAAC, "launch", "isaac_sim.launch.py"))
for path in scripts:
    try:
        compile(open(path, encoding="utf-8").read(), path, "exec")
    except SyntaxError as exc:
        fail("%s: %s" % (os.path.basename(path), exc))
if not any("syntax" in f.lower() or "SyntaxError" in f for f in failures):
    ok("all four files compile")

runtime = open(os.path.join(ISAAC, "scripts", "run_isaac_sim.py"), encoding="utf-8").read()
if "create_publisher" in runtime or "ROS2Publisher\"" in runtime or "ROS2PublishTwist" in runtime:
    fail("run_isaac_sim.py can publish a command; only the safety supervisor may publish /cmd_vel")
elif "ROS2SubscribeTwist" not in runtime:
    fail("run_isaac_sim.py does not subscribe to /cmd_vel")
else:
    ok("the runtime subscribes to /cmd_vel and has no publisher that could carry it")
if runtime.count("ROS2PublishRawTransformTree") != 1 or "ROS2PublishTransformTree" in runtime.replace(
        "ROS2PublishRawTransformTree", ""):
    fail("expected exactly one TF publisher (odom -> base_link) in the runtime")
else:
    ok("one TF publisher, odom -> base_link; base_link -> sensors stays with robot_state_publisher")

launch_dirs = glob.glob(os.path.join(SRC, "*", "launch"))
leaks = []
for d in launch_dirs:
    for name in os.listdir(d):
        if not name.endswith(".py"):
            continue
        code = [ln for ln in open(os.path.join(d, name), encoding="utf-8")
                if not ln.lstrip().startswith("#")]
        if any("ground_truth" in ln for ln in code):
            leaks.append(name)
if leaks:
    fail("launch files reference the ground-truth topic: %s" % leaks)
else:
    ok("no launch file references the ground-truth topic")

print()
if failures:
    print("%d PROBLEM(S)" % len(failures))
    sys.exit(1)
print("Isaac Sim backend consistent with the interface contract (not run in Isaac Sim)")
