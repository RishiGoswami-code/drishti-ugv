#!/usr/bin/env python3
"""Validate the Gazebo worlds without Gazebo.

Written after the same mistake three times: a double hyphen inside an XML
comment. It is illegal in XML, it is invisible when you are writing prose, and
the parser error points at the comment rather than at the offending characters.
Gazebo would reject the world outright, but only once someone had a machine to
run it on -- weeks after the file was written. A checker is cheaper than that
memory.

Also verifies that each world can actually produce the data the stack needs:
a world missing the Sensors system loads perfectly and publishes no images,
which looks exactly like a broken camera driver.

    python tools/check_sim_assets.py
"""
import glob
import os
import re
import sys
from xml.etree import ElementTree as ET

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS = os.path.join(WS, "src", "drishti_sim", "worlds")

# gz-sim systems the stack depends on, and what breaks without each.
REQUIRED_SYSTEMS = {
    "gz-sim-physics-system": "nothing moves",
    "gz-sim-scene-broadcaster-system": "no GUI or state updates",
    "gz-sim-sensors-system": "cameras and depth publish nothing",
    "gz-sim-imu-system": "/imu/data is silent",
}

failures = []


def fail(msg):
    failures.append(msg)
    print("  FAIL  " + msg)


def ok(msg):
    print("  ok    " + msg)


def check_comment_hyphens(path, text):
    """Find '--' inside an XML comment before the parser does."""
    problems = []
    for block in re.finditer(r"<!--(.*?)(?:-->|$)", text, re.S):
        body = block.group(1)
        if "--" in body:
            line = text[:block.start()].count("\n") + 1
            problems.append(line)
    for line in problems:
        fail("%s: '--' inside the XML comment starting at line %d. XML forbids "
             "it; use an em dash or rephrase." % (os.path.basename(path), line))
    return not problems


def main():
    files = sorted(glob.glob(os.path.join(WORLDS, "*.sdf")))
    if not files:
        fail("no worlds found in %s" % WORLDS)
        return 1

    print("worlds: %s" % ", ".join(os.path.basename(f) for f in files))
    print()

    for path in files:
        name = os.path.basename(path)
        text = open(path, encoding="utf-8").read()

        if not check_comment_hyphens(path, text):
            continue

        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            fail("%s is not well-formed XML: %s" % (name, exc))
            continue

        world = root.find("world")
        if world is None:
            fail("%s has no <world> element" % name)
            continue

        systems = {p.get("filename") for p in world.findall("plugin")}
        missing = [s for s in REQUIRED_SYSTEMS if s not in systems]
        if missing:
            for s in missing:
                fail("%s is missing %s -- %s" % (name, s, REQUIRED_SYSTEMS[s]))
            continue

        models = world.findall("model")
        lights = world.findall("light")
        if not lights:
            # Cameras would render a black image, and every downstream failure
            # would be blamed on perception.
            fail("%s has no light; the cameras would see nothing" % name)
            continue
        if not models:
            fail("%s has no models at all" % name)
            continue

        ground = [m for m in models if "ground" in (m.get("name") or "")]
        if not ground:
            fail("%s has no ground model; the vehicle would fall" % name)
            continue

        # Every model needs a link, and static scenery needs collision geometry
        # or the vehicle drives straight through the obstacle it is meant to
        # avoid -- a silent failure that looks like a perception bug.
        for m in models:
            mname = m.get("name")
            links = m.findall("link")
            if not links:
                fail("%s: model %r has no link" % (name, mname))
                continue
            has_collision = any(ln.find("collision") is not None for ln in links)
            if not has_collision:
                fail("%s: model %r has no collision geometry; it would be "
                     "invisible to physics" % (name, mname))

        ok("%-11s %2d models, %d light(s), all required systems present"
           % (name, len(models), len(lights)))

    print()
    if failures:
        print("%d PROBLEM(S)" % len(failures))
        return 1
    print("simulation assets consistent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
