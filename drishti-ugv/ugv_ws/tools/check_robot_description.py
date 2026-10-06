#!/usr/bin/env python3
"""Validate the robot description against SPEC.md section 3 without ROS.

A URDF error is cheap to make and expensive to find: a wrong optical rotation
or a duplicated TF edge shows up three phases later as a map that drifts or a
point cloud that lands sideways. SPEC.md 3.2 calls frame problems "the single
most expensive bug in this system", so they get a check that runs anywhere.

This resolves the small xacro subset the description actually uses --
<xacro:property>, ${...} expressions and <xacro:macro> with parameters -- then
checks:

  1. the file is well-formed XML and every link/joint reference resolves
  2. the links form a single tree rooted at base_link, with no cycles and no
     link claimed by two joints (SPEC.md 3.2 rule 1)
  3. the SPEC.md 3.1 frames exist with exactly the specified parents
  4. optical frames carry the ROS body-to-optical rotation and are distinct
     from their mounting frame (SPEC.md 3.2 rule 2)
  5. every non-fixed joint has an axis, and every link with mass has a
     positive-definite-looking inertia
  6. Gazebo sensor gz_frame_id values name frames that exist

Needs neither ROS nor xacro.

    python tools/check_robot_description.py
"""
import math
import os
import re
import sys
from xml.etree import ElementTree as ET

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(WS, "src", "drishti_description", "urdf", "drishti.urdf.xacro")
XACRO_NS = "http://www.ros.org/wiki/xacro"

# SPEC.md 3.1: frame -> required parent. Wheels are checked separately.
SPEC_TREE = {
    "camera_link": "base_link",
    "camera_left_optical": "camera_link",
    "camera_right_optical": "camera_link",
    "imu_link": "base_link",
}
HALF_PI = math.pi / 2.0
# SPEC.md 3.2 rule 2: z forward, x right, y down.
OPTICAL_RPY = (-HALF_PI, 0.0, -HALF_PI)

failures = []


def fail(msg):
    failures.append(msg)
    print("  FAIL  " + msg)


def ok(msg):
    print("  ok    " + msg)


# --------------------------------------------------------------- xacro subset
def evaluate(expr, props):
    """Evaluate a ${...} body against the property table.

    Identifiers are replaced by repr() of their value, so a string property
    becomes a quoted literal rather than a bare name eval() would reject --
    that is what lets ${name}_link expand inside a macro.
    """
    def swap(m):
        key = m.group(1)
        return repr(props[key]) if key in props else key

    resolved = re.sub(r"\b([A-Za-z_]\w*)\b", swap, expr)
    try:
        # Arithmetic and string literals only; the description uses nothing else.
        return eval(resolved, {"__builtins__": {}}, {})  # noqa: S307
    except Exception:
        return None


def substitute(text, props):
    if text is None:
        return None
    out, guard = text, 0
    while "${" in out and guard < 10:
        guard += 1
        new = ""
        i = 0
        while i < len(out):
            if out.startswith("${", i):
                depth, j = 1, i + 2
                while j < len(out) and depth:
                    if out[j] == "{":
                        depth += 1
                    elif out[j] == "}":
                        depth -= 1
                    j += 1
                val = evaluate(out[i + 2:j - 1], props)
                new += out[i:j] if val is None else (
                    repr(round(val, 12)) if isinstance(val, float) else str(val))
                i = j
            else:
                new += out[i]
                i += 1
        if new == out:
            break
        out = new
    return out


def expand(node, props, out):
    """Walk the xacro tree, expanding properties and macros into `out`."""
    for child in node:
        tag = child.tag
        if tag == "{%s}property" % XACRO_NS:
            raw = child.get("value")
            val = substitute(raw, props)
            try:
                props[child.get("name")] = float(val)
            except (TypeError, ValueError):
                props[child.get("name")] = val
            continue
        if tag == "{%s}macro" % XACRO_NS:
            props.setdefault("__macros__", {})[child.get("name")] = child
            continue
        if tag.startswith("{%s}" % XACRO_NS):
            name = tag.split("}")[1]
            macro = props.get("__macros__", {}).get(name)
            if macro is None:
                fail("unknown xacro element: %s" % name)
                continue
            local = dict(props)
            for param in (macro.get("params") or "").split():
                local[param] = substitute(child.get(param), props)
                try:
                    local[param] = float(local[param])
                except (TypeError, ValueError):
                    pass
            expand(macro, local, out)
            continue

        clone = ET.Element(tag, {k: substitute(v, props) for k, v in child.attrib.items()})
        # Element text carries real content here -- <gz_frame_id>camera_left_optical
        # </gz_frame_id> is a text node, and dropping it made check 6 vacuous.
        clone.text = substitute(child.text, props)
        clone.tail = child.tail
        out.append(clone)
        expand(child, props, clone)


def floats(text, n):
    try:
        vals = [float(v) for v in (text or "").split()]
    except ValueError:
        return None
    return vals if len(vals) == n else None


# ------------------------------------------------------------------- validate
try:
    tree = ET.parse(URDF)
except ET.ParseError as exc:
    print("  FAIL  URDF is not well-formed XML: %s" % exc)
    sys.exit(1)

root_in = tree.getroot()
robot = ET.Element("robot", {"name": root_in.get("name", "")})
expand(root_in, {}, robot)

links = {el.get("name"): el for el in robot.findall("link")}
joints = robot.findall("joint")

print("expanded: %d links, %d joints" % (len(links), len(joints)))

print("\n1. references resolve")
bad_ref = False
for j in joints:
    for end in ("parent", "child"):
        el = j.find(end)
        name = el.get("link") if el is not None else None
        if name not in links:
            fail("joint %s: %s link %r does not exist" % (j.get("name"), end, name))
            bad_ref = True
if not bad_ref:
    ok("every joint parent/child names a declared link")

print("\n2. the links form one tree (SPEC 3.2 rule 1)")
parent_of, claimed_twice = {}, False
for j in joints:
    child = j.find("child").get("link")
    parent = j.find("parent").get("link")
    if child in parent_of:
        fail("link %r is the child of two joints (%s and %s) -- two publishers "
             "on one TF edge" % (child, parent_of[child][1], j.get("name")))
        claimed_twice = True
    parent_of[child] = (parent, j.get("name"))

roots = [n for n in links if n not in parent_of]
if roots != ["base_link"]:
    fail("expected exactly one root 'base_link', found %s" % sorted(roots))
elif not claimed_twice:
    ok("single root base_link, no link claimed twice")

for start in list(links):
    seen, cur = set(), start
    while cur in parent_of:
        if cur in seen:
            fail("cycle in the frame tree at %r" % cur)
            break
        seen.add(cur)
        cur = parent_of[cur][0]

print("\n3. SPEC 3.1 frames exist with the specified parent")
for frame, want_parent in SPEC_TREE.items():
    if frame not in links:
        fail("SPEC 3.1 frame %r is missing" % frame)
    elif parent_of.get(frame, (None,))[0] != want_parent:
        fail("%r parent is %r, SPEC 3.1 says %r"
             % (frame, parent_of.get(frame, (None,))[0], want_parent))
    else:
        ok("%-22s <- %s" % (frame, want_parent))

wheels = [n for n in links if n.startswith("wheel_")]
if len(wheels) != 4:
    fail("expected 4 wheel links, found %d: %s" % (len(wheels), sorted(wheels)))
elif any(parent_of[w][0] != "base_link" for w in wheels):
    fail("every wheel must hang off base_link")
else:
    ok("4 wheels, all parented to base_link")

print("\n4. optical frames (SPEC 3.2 rule 2)")
for frame in ("camera_left_optical", "camera_right_optical"):
    j = next((x for x in joints if x.find("child") is not None
              and x.find("child").get("link") == frame), None)
    if j is None:
        continue
    origin = j.find("origin")
    rpy = floats(origin.get("rpy") if origin is not None else None, 3)
    if rpy is None:
        fail("%s: origin rpy is missing or malformed" % frame)
    elif any(abs(a - b) > 1e-6 for a, b in zip(rpy, OPTICAL_RPY)):
        fail("%s: rpy %s is not the ROS body-to-optical rotation %s"
             % (frame, [round(v, 6) for v in rpy], [round(v, 6) for v in OPTICAL_RPY]))
    else:
        ok("%s carries the standard optical rotation" % frame)

    xyz = floats(origin.get("xyz"), 3)
    if xyz is not None and all(abs(v) < 1e-9 for v in xyz) and rpy == [0, 0, 0]:
        fail("%s is coincident with its mounting frame" % frame)

print("\n5. joints and inertias")
for j in joints:
    if j.get("type") in ("revolute", "continuous", "prismatic") and j.find("axis") is None:
        fail("joint %s is %s but declares no axis" % (j.get("name"), j.get("type")))
for name, el in links.items():
    inertial = el.find("inertial")
    if inertial is None:
        continue
    mass = inertial.find("mass")
    m = float(mass.get("value")) if mass is not None else 0.0
    if m <= 0.0:
        fail("link %s has non-positive mass %g" % (name, m))
    inertia = inertial.find("inertia")
    if inertia is None:
        fail("link %s has mass but no inertia" % name)
        continue
    diag = [float(inertia.get(k, "0")) for k in ("ixx", "iyy", "izz")]
    if any(v <= 0.0 for v in diag):
        fail("link %s has a non-positive inertia diagonal %s" % (name, diag))
    a, b, c = sorted(diag)
    if a + b < c - 1e-12:
        fail("link %s violates the triangle inequality on its inertia diagonal" % name)
if not failures:
    ok("all joints and inertias are well formed")
else:
    ok("joint/inertia pass complete")

print("\n6. gazebo sensor frames")
for gz in robot.findall("gazebo"):
    for sensor in gz.iter("sensor"):
        fid = sensor.findtext("gz_frame_id")
        if fid and fid not in links:
            fail("sensor %s: gz_frame_id %r is not a link" % (sensor.get("name"), fid))
        elif fid:
            ok("sensor %-14s -> %s" % (sensor.get("name"), fid))

print()
if failures:
    print("%d PROBLEM(S)" % len(failures))
    sys.exit(1)
print("robot description consistent with SPEC.md section 3")
