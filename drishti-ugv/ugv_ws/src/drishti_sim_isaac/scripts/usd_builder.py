# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Turn a scene_model.World / Robot into USD prims. Runs only inside Isaac Sim.

Uses nothing but the standard USD and UsdPhysics schemas (pxr), not Isaac's URDF
importer: the importer's output layout changed between Isaac Sim releases, and a
builder with known prim paths is what the ROS graph and the articulation wrapper
need. Rigid bodies are siblings under the robot root, each with its own world
pose; links rigidly attached to the base (sensor mounts) are plain Xform
children of it.

!! UNVERIFIED !! Never executed: no machine on the project can run Isaac Sim.
"""
import math
import re

from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

import scene_model as sm

PHYSICS_MATERIALS = "/World/PhysicsMaterials"
LOOKS = "/World/Looks"
GROUND_SLAB_THICKNESS = 1.0
APERTURE_MM = 20.955


def safe(name):
    out = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return out if not out[0].isdigit() else "_" + out


def _set_pose(prim, pose, scale=None):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pose[0]))
    w, x, y, z = pose[1]
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    if scale is not None:
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))


def _look(stage, rgba, cache):
    """A UsdPreviewSurface material for a diffuse colour, one per colour."""
    key = tuple(round(c, 4) for c in (rgba or (0.6, 0.6, 0.6, 1.0))[:3])
    if key in cache:
        return cache[key]
    path = "%s/c_%s" % (LOOKS, "_".join(str(int(round(c * 1000))) for c in key))
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*key))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    cache[key] = mat
    return mat


def _friction(stage, mu, cache):
    key = round(mu, 3)
    if key in cache:
        return cache[key]
    path = "%s/mu_%s" % (PHYSICS_MATERIALS, str(key).replace(".", "_"))
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(mu))
    api.CreateDynamicFrictionAttr(float(mu))
    api.CreateRestitutionAttr(0.0)
    cache[key] = mat
    return mat


def _geom(stage, path, shape):
    """Create the gprim for a shape; returns (prim, scale-or-None)."""
    if shape.kind == "box":
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        return g.GetPrim(), shape.dims
    if shape.kind == "sphere":
        g = UsdGeom.Sphere.Define(stage, path)
        g.CreateRadiusAttr(float(shape.dims[0]))
        return g.GetPrim(), None
    if shape.kind == "cylinder":
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(float(shape.dims[0]))
        g.CreateHeightAttr(float(shape.dims[1]))
        g.CreateAxisAttr("Z")
        return g.GetPrim(), None
    raise ValueError("cannot build a %r" % shape.kind)


def _decorate(stage, prim, rgba, mu, collide, looks, frictions):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(_look(stage, rgba, looks))
    if collide:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            _friction(stage, mu, frictions), UsdShade.Tokens.weakerThanDescendants, "physics")


# -------------------------------------------------------------------- world
def build_world(stage, world, lighting):
    """Ground, obstacles, gravity and light for one SDF world."""
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Scene")
    UsdGeom.Scope.Define(stage, LOOKS)
    UsdGeom.Scope.Define(stage, PHYSICS_MATERIALS)

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    g = world.gravity
    mag = math.sqrt(sum(c * c for c in g))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(*(c / mag for c in g)))
    scene.CreateGravityMagnitudeAttr(float(mag))

    if world.sun is not None:
        sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
        sun.CreateIntensityAttr(float(lighting["sun_intensity"]))
        sun.CreateColorAttr(Gf.Vec3f(*world.sun.diffuse))
        sun.CreateAngleAttr(0.53)
        _set_pose(sun.GetPrim(), ((0.0, 0.0, 0.0), sm.look_rotation_neg_z(world.sun.direction)))
    sky = UsdLux.DomeLight.Define(stage, "/World/Sky")
    sky.CreateIntensityAttr(float(lighting["sky_intensity"]))
    if world.background is not None:
        sky.CreateColorAttr(Gf.Vec3f(*world.background))

    looks, frictions = {}, {}
    for prim_spec in world.prims:
        path = "/World/Scene/%s" % safe(prim_spec.name)
        if prim_spec.kind == "plane":
            # A plane has no thickness to collide against reliably; use a slab whose
            # top face is the plane.
            shape = sm.Shape("box", (prim_spec.dims[0], prim_spec.dims[1], GROUND_SLAB_THICKNESS))
            centre = sm.quat_rotate(prim_spec.pose[1], (0.0, 0.0, -GROUND_SLAB_THICKNESS / 2.0))
            pose = ((prim_spec.pose[0][0] + centre[0], prim_spec.pose[0][1] + centre[1],
                     prim_spec.pose[0][2] + centre[2]), prim_spec.pose[1])
        else:
            shape = sm.Shape(prim_spec.kind, prim_spec.dims)
            pose = prim_spec.pose
        prim, scale = _geom(stage, path, shape)
        _set_pose(prim, pose, scale)
        _decorate(stage, prim, prim_spec.rgba, prim_spec.mu, True, looks, frictions)


# -------------------------------------------------------------------- robot
def _axis_token(axis):
    for token, vec in (("X", (1, 0, 0)), ("Y", (0, 1, 0)), ("Z", (0, 0, 1))):
        if all(abs(abs(a) - b) < 1e-9 for a, b in zip(axis, vec)):
            return token
    raise ValueError("joint axis %s is not a coordinate axis" % (axis,))


def build_robot(stage, robot, spawn, drive):
    """Build the vehicle at `spawn` (a scene_model Pose in the world frame).

    Returns a dict of the prim paths the runtime needs.
    """
    root = "/World/%s" % safe(robot.name)
    UsdGeom.Xform.Define(stage, root)
    looks, frictions = {}, {}

    moving = {j.child: j for j in robot.moving_joints()}
    bodies = [robot.base_link] + list(moving)
    paths = {name: "%s/%s" % (root, safe(name)) for name in bodies}

    def add_shapes(link, parent_path, parent_pose, collide):
        for i, shape in enumerate(link.visuals):
            path = "%s/%s_%d" % (parent_path, safe(link.name), i)
            prim, scale = _geom(stage, path, shape)
            _set_pose(prim, sm.pose_mul(parent_pose, shape.pose), scale)
            _decorate(stage, prim, shape.rgba, link.mu, collide, looks, frictions)

    for name in bodies:
        link = robot.links[name]
        prim = UsdGeom.Xform.Define(stage, paths[name]).GetPrim()
        _set_pose(prim, sm.pose_mul(spawn, robot.link_pose(name)))
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass = robot.lumped_mass() if name == robot.base_link else link.mass
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        add_shapes(link, paths[name], sm.IDENTITY, bool(link.collisions))

    # Links rigidly attached to the base: visual-only children, no collision.
    for name, link in robot.links.items():
        if name in bodies or not link.visuals:
            continue
        holder = "%s/%s" % (paths[robot.base_link], safe(name))
        _set_pose(UsdGeom.Xform.Define(stage, holder).GetPrim(), robot.link_pose(name))
        add_shapes(link, holder, sm.IDENTITY, False)

    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(paths[robot.base_link]))

    UsdGeom.Scope.Define(stage, root + "/joints")
    for name, joint in moving.items():
        if joint.kind != "continuous" and joint.kind != "revolute":
            raise ValueError("joint %s: only revolute joints are supported" % joint.name)
        jp = "%s/joints/%s" % (root, safe(joint.name))
        rev = UsdPhysics.RevoluteJoint.Define(stage, jp)
        rev.CreateBody0Rel().SetTargets([Sdf.Path(paths[joint.parent])])
        rev.CreateBody1Rel().SetTargets([Sdf.Path(paths[joint.child])])
        # URDF: the joint origin is in the parent link's frame, and the child
        # link's frame is the joint frame, so the child side carries no offset.
        w, x, y, z = joint.origin[1]
        rev.CreateLocalPos0Attr(Gf.Vec3f(*joint.origin[0]))
        rev.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
        rev.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        rev.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        rev.CreateAxisAttr(_axis_token(joint.axis))
        # Wheels overlap the hull slightly; a jointed pair must not collide.
        rev.CreateCollisionEnabledAttr(False)
        drv = UsdPhysics.DriveAPI.Apply(rev.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(0.0)
        drv.CreateDampingAttr(float(drive["damping"]))
        drv.CreateMaxForceAttr(float(drive["max_force"]))

    return {"root": root, "base": paths[robot.base_link], "bodies": paths}


def add_camera(stage, base_path, sensor, optical_pose):
    """A USD camera that looks out of the sensor's ROS optical frame."""
    path = "%s/%s" % (base_path, safe(sensor.name))
    cam = UsdGeom.Camera.Define(stage, path)
    focal = APERTURE_MM / (2.0 * math.tan(sensor.hfov / 2.0))
    cam.CreateProjectionAttr("perspective")
    cam.CreateHorizontalApertureAttr(APERTURE_MM)
    cam.CreateVerticalApertureAttr(APERTURE_MM * sensor.height / sensor.width)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(float(sensor.near), float(sensor.far)))
    _set_pose(cam.GetPrim(), sm.usd_camera_pose(optical_pose))
    return path
