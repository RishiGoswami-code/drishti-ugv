# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Neutral scene description for the Isaac Sim backend. No Isaac, no pxr, no ROS.

The robot comes from the expanded URDF and the worlds from the SDF files the
Gazebo backend already uses, so the two simulators cannot disagree about a
dimension, a mass, a sensor pose or where a rock is. Everything that can be
decided without a simulator -- parsing, frame composition, the camera
orientation, differential-drive kinematics -- lives here, where
tools/check_isaac_assets.py can exercise it. usd_builder.py and
run_isaac_sim.py only turn these values into USD and OmniGraph.

Conventions: quaternions are (w, x, y, z) throughout this module; OmniGraph and
ROS messages use (x, y, z, w), converted with xyzw(). Roll-pitch-yaw is the
fixed-axis convention URDF and SDF share, R = Rz(yaw) * Ry(pitch) * Rx(roll).

!! UNVERIFIED against Isaac Sim !! The geometry and kinematics here are checked
offline; whether the scene they describe loads and behaves in Isaac Sim is not.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from xml.etree import ElementTree as ET

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Pose = Tuple[Vec3, Quat]

IDENTITY: Pose = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
# Half a turn about X: the difference between a USD camera and a ROS optical frame.
RX_PI: Quat = (0.0, 1.0, 0.0, 0.0)


# ------------------------------------------------------------------ geometry
def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quat:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def quat_mul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    w, x, y, z = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


def pose_mul(a: Pose, b: Pose) -> Pose:
    """Pose of b expressed through a: apply b first, then a."""
    p = quat_rotate(a[1], b[0])
    return ((a[0][0] + p[0], a[0][1] + p[1], a[0][2] + p[2]), quat_mul(a[1], b[1]))


def xyzw(q: Quat) -> Quat:
    return (q[1], q[2], q[3], q[0])


def look_rotation_neg_z(direction: Vec3) -> Quat:
    """Smallest rotation taking -Z to `direction`.

    A USD DistantLight shines along its local -Z, an SDF directional light along
    its <direction>.
    """
    n = math.sqrt(sum(c * c for c in direction))
    if n == 0.0:
        raise ValueError("zero-length light direction")
    d = tuple(c / n for c in direction)
    dot = -d[2]
    if dot > 0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    if dot < -0.999999:
        return (0.0, 1.0, 0.0, 0.0)
    # cross((0, 0, -1), d)
    ax, ay, az = d[1], -d[0], 0.0
    q = (1.0 + dot, ax, ay, az)
    m = math.sqrt(sum(c * c for c in q))
    return tuple(c / m for c in q)


def usd_camera_pose(optical_pose: Pose) -> Pose:
    """Pose a USD camera needs to look out of a ROS optical frame.

    A ROS optical frame has +Z forward, +X right, +Y down. A USD camera looks
    along its local -Z with +Y up. Half a turn about X maps one onto the other;
    without it every image is upside down and looks backwards.
    """
    return pose_mul(optical_pose, ((0.0, 0.0, 0.0), RX_PI))


# --------------------------------------------------------------- drive maths
def wheel_speeds(v: float, w: float, wheel_radius: float,
                 wheel_separation: float) -> Tuple[float, float]:
    """Left and right wheel angular speed (rad/s) for a body twist.

    Positive speed rotates a wheel about +Y, which rolls it toward +X. Positive
    w turns counter-clockwise seen from above, so the right side runs faster.
    """
    left = (v - w * wheel_separation / 2.0) / wheel_radius
    right = (v + w * wheel_separation / 2.0) / wheel_radius
    return left, right


def slew(previous: float, target: float, max_step: float) -> float:
    """Move from previous toward target by at most max_step."""
    if max_step <= 0.0:
        return target
    delta = target - previous
    if delta > max_step:
        return previous + max_step
    if delta < -max_step:
        return previous - max_step
    return target


def compose_ground_truth(spawn: Pose, rel_position: Vec3, rel_quat_xyzw: Quat):
    """World pose from IsaacComputeOdometry output and the spawn pose.

    The node reports position as R0^-1 * (p - p0), in the frame the vehicle
    started in, and orientation as R * R0^-1. So the world position is
    p0 + R0 * position, and the world orientation is orientation * R0. The two
    conventions differ only when pitch or roll is non-zero.

    Returns (position, quaternion as x, y, z, w).
    """
    p = quat_rotate(spawn[1], rel_position)
    position = (spawn[0][0] + p[0], spawn[0][1] + p[1], spawn[0][2] + p[2])
    rel = (rel_quat_xyzw[3], rel_quat_xyzw[0], rel_quat_xyzw[1], rel_quat_xyzw[2])
    return position, xyzw(quat_mul(rel, spawn[1]))


# ------------------------------------------------------------------- parsing
def _floats(text: Optional[str], n: int, default=None):
    if text is None or not text.strip():
        return default
    vals = [float(v) for v in text.split()]
    vals += [0.0] * (n - len(vals))
    return tuple(vals[:n])


def _urdf_origin(el: Optional[ET.Element]) -> Pose:
    if el is None:
        return IDENTITY
    origin = el.find("origin")
    if origin is None:
        return IDENTITY
    xyz = _floats(origin.get("xyz"), 3, (0.0, 0.0, 0.0))
    rpy = _floats(origin.get("rpy"), 3, (0.0, 0.0, 0.0))
    return (xyz, rpy_to_quat(*rpy))


def _sdf_pose(el: Optional[ET.Element]) -> Pose:
    if el is None:
        return IDENTITY
    vals = _floats(el.findtext("pose"), 6, (0.0,) * 6)
    return ((vals[0], vals[1], vals[2]), rpy_to_quat(vals[3], vals[4], vals[5]))


@dataclass
class Shape:
    kind: str                        # 'box' | 'sphere' | 'cylinder' | 'plane'
    dims: Tuple[float, ...]          # box (x,y,z); sphere (r,); cylinder (r,length); plane (sx,sy)
    pose: Pose = IDENTITY
    rgba: Optional[Tuple[float, float, float, float]] = None


def _geometry(geom: ET.Element) -> Tuple[str, Tuple[float, ...]]:
    box = geom.find("box")
    if box is not None:
        return "box", _floats(box.get("size") or box.findtext("size"), 3)
    sphere = geom.find("sphere")
    if sphere is not None:
        r = sphere.get("radius") or sphere.findtext("radius")
        return "sphere", (float(r),)
    cyl = geom.find("cylinder")
    if cyl is not None:
        r = cyl.get("radius") or cyl.findtext("radius")
        length = cyl.get("length") or cyl.findtext("length")
        return "cylinder", (float(r), float(length))
    plane = geom.find("plane")
    if plane is not None:
        normal = _floats(plane.findtext("normal"), 3, (0.0, 0.0, 1.0))
        if abs(normal[0]) > 1e-9 or abs(normal[1]) > 1e-9 or abs(normal[2] - 1.0) > 1e-9:
            raise ValueError("only +Z planes are supported, got normal %s" % (normal,))
        return "plane", _floats(plane.findtext("size"), 2, (1.0, 1.0))
    kinds = [c.tag for c in geom]
    raise ValueError("unsupported geometry %s (supported: box, sphere, cylinder, plane)" % kinds)


# ---------------------------------------------------------------- the robot
@dataclass
class Link:
    name: str
    mass: float = 0.0
    visuals: List[Shape] = field(default_factory=list)
    collisions: List[Shape] = field(default_factory=list)
    mu: float = 1.0


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: Pose
    axis: Vec3


@dataclass
class SensorSpec:
    name: str
    kind: str                        # 'camera' | 'depth_camera' | 'imu'
    link: str
    frame_id: str
    rate_hz: float
    width: int = 0
    height: int = 0
    hfov: float = 0.0
    near: float = 0.0
    far: float = 0.0


@dataclass
class DiffDrive:
    left_joints: List[str]
    right_joints: List[str]
    wheel_separation: float
    wheel_radius: float
    max_linear_acceleration: float


@dataclass
class Robot:
    name: str
    links: Dict[str, Link]
    joints: List[Joint]
    sensors: Dict[str, SensorSpec]
    diff_drive: Optional[DiffDrive]
    base_link: str

    def parent_joint(self, link: str) -> Optional[Joint]:
        for j in self.joints:
            if j.child == link:
                return j
        return None

    def link_pose(self, link: str) -> Pose:
        """Pose of `link` in the base frame, composed along the joint chain.

        A moving joint is taken at zero displacement, which is its pose at spawn.
        """
        chain = []
        cur = link
        while cur != self.base_link:
            joint = self.parent_joint(cur)
            if joint is None:
                raise ValueError("link %r is not connected to %r" % (link, self.base_link))
            chain.append(joint.origin)
            cur = joint.parent
        pose = IDENTITY
        for origin in reversed(chain):
            pose = pose_mul(pose, origin)
        return pose

    def moving_joints(self) -> List[Joint]:
        return [j for j in self.joints if j.kind != "fixed"]

    def lumped_mass(self) -> float:
        """Base mass plus every link rigidly attached to it.

        Fixed-attached links (sensor mounts) are Xform children of the base in
        USD, not separate bodies, so their mass is added to the base's.
        """
        moving = {j.child for j in self.moving_joints()}
        return sum(l.mass for n, l in self.links.items() if n not in moving)


def parse_urdf_text(text: str) -> Robot:
    root = ET.fromstring(text)
    links: Dict[str, Link] = {}
    for el in root.findall("link"):
        link = Link(name=el.get("name"))
        mass = el.find("inertial/mass")
        if mass is not None:
            link.mass = float(mass.get("value"))
        for tag, bucket in (("visual", link.visuals), ("collision", link.collisions)):
            for v in el.findall(tag):
                geom = v.find("geometry")
                if geom is None:
                    continue
                kind, dims = _geometry(geom)
                color = v.find("material/color")
                rgba = _floats(color.get("rgba"), 4) if color is not None else None
                bucket.append(Shape(kind, dims, _urdf_origin(v), rgba))
        links[link.name] = link

    joints = []
    for el in root.findall("joint"):
        axis_el = el.find("axis")
        axis = _floats(axis_el.get("xyz"), 3, (1.0, 0.0, 0.0)) if axis_el is not None else (1.0, 0.0, 0.0)
        joints.append(Joint(el.get("name"), el.get("type"), el.find("parent").get("link"),
                            el.find("child").get("link"), _urdf_origin(el), axis))

    children = {j.child for j in joints}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        raise ValueError("expected one root link, found %s" % roots)

    sensors: Dict[str, SensorSpec] = {}
    diff_drive = None
    for gz in root.findall("gazebo"):
        ref = gz.get("reference")
        if ref in links and gz.findtext("mu1"):
            links[ref].mu = float(gz.findtext("mu1"))
        for s in gz.findall("sensor"):
            spec = SensorSpec(name=s.get("name"), kind=s.get("type"), link=ref,
                              frame_id=s.findtext("gz_frame_id") or ref,
                              rate_hz=float(s.findtext("update_rate") or 0.0))
            cam = s.find("camera")
            if cam is not None:
                spec.hfov = float(cam.findtext("horizontal_fov"))
                spec.width = int(cam.findtext("image/width"))
                spec.height = int(cam.findtext("image/height"))
                spec.near = float(cam.findtext("clip/near"))
                spec.far = float(cam.findtext("clip/far"))
            sensors[spec.name] = spec
        for p in gz.findall("plugin"):
            if "DiffDrive" in (p.get("name") or ""):
                diff_drive = DiffDrive(
                    left_joints=[e.text.strip() for e in p.findall("left_joint")],
                    right_joints=[e.text.strip() for e in p.findall("right_joint")],
                    wheel_separation=float(p.findtext("wheel_separation")),
                    wheel_radius=float(p.findtext("wheel_radius")),
                    max_linear_acceleration=float(p.findtext("max_linear_acceleration") or 0.0))
    return Robot(root.get("name"), links, joints, sensors, diff_drive, roots[0])


# ---------------------------------------------------------------- the world
@dataclass
class WorldPrim:
    name: str
    kind: str
    dims: Tuple[float, ...]
    pose: Pose
    rgba: Optional[Tuple[float, float, float, float]]
    mu: float
    collision_matches_visual: bool


@dataclass
class Sun:
    direction: Vec3
    diffuse: Tuple[float, float, float]


@dataclass
class World:
    name: str
    gravity: Vec3
    ambient: Optional[Tuple[float, float, float]]
    background: Optional[Tuple[float, float, float]]
    sun: Optional[Sun]
    prims: List[WorldPrim]


def parse_sdf_world_text(text: str) -> World:
    root = ET.fromstring(text)
    world = root.find("world")
    if world is None:
        raise ValueError("no <world> element")

    prims: List[WorldPrim] = []
    for model in world.findall("model"):
        model_pose = _sdf_pose(model)
        for link in model.findall("link"):
            link_pose = pose_mul(model_pose, _sdf_pose(link))
            visuals = link.findall("visual")
            collision = link.find("collision")
            mu = 1.0
            if collision is not None and collision.findtext("surface/friction/ode/mu"):
                mu = float(collision.findtext("surface/friction/ode/mu"))
            for i, vis in enumerate(visuals):
                kind, dims = _geometry(vis.find("geometry"))
                matches = False
                if collision is not None:
                    ckind, cdims = _geometry(collision.find("geometry"))
                    matches = (ckind == kind and len(cdims) == len(dims) and
                               all(abs(a - b) < 1e-9 for a, b in zip(cdims, dims)))
                diffuse = _floats(vis.findtext("material/diffuse"), 4)
                name = model.get("name") if len(visuals) == 1 else "%s_%d" % (model.get("name"), i)
                prims.append(WorldPrim(name, kind, dims, pose_mul(link_pose, _sdf_pose(vis)),
                                       diffuse, mu, matches))

    sun = None
    for light in world.findall("light"):
        if light.get("type") == "directional":
            direction = _floats(light.findtext("direction"), 3, (0.0, 0.0, -1.0))
            diffuse = _floats(light.findtext("diffuse"), 4, (1.0, 1.0, 1.0, 1.0))
            sun = Sun(direction, diffuse[:3])
            break

    ambient = _floats(world.findtext("scene/ambient"), 4)
    background = _floats(world.findtext("scene/background"), 4)
    return World(world.get("name"), _floats(world.findtext("gravity"), 3, (0.0, 0.0, -9.81)),
                 ambient[:3] if ambient else None, background[:3] if background else None,
                 sun, prims)
