#!/usr/bin/env python3
"""Record scenario runs as compact JSON for the web viewer.

The viewer is a player, not a re-implementation. Every decision it shows was
computed HERE, in Python, by the cores that tools/check_parity.py proves
identical to the shipping C++. Nothing about the cost function or the
supervisor is rewritten in JavaScript, so the page cannot quietly drift away
from the system it is demonstrating.

    python tools/export_runs.py --out runs.json

Size matters -- the payload is embedded in a single HTML page. Two things keep
it small:

  * the terrain and the cost grid are static, so they ship once
  * a cell's cost never changes after it is first observed (the simulation
    caches exactly this), so each frame carries only the indices of cells
    observed for the first time
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO = os.path.dirname(HERE)
sys.path.insert(0, PROTO)

from drishti_proto.sim import Fault, Simulation          # noqa: E402
from drishti_proto.world import WORLDS                   # noqa: E402

SCENARIOS = [
    {
        "id": "hard",
        "title": "The ditch",
        "world": "hard",
        "faults": [],
        "headline": "A ditch is refused on geometry alone",
        "blurb": "Nothing sticks up, so a conventional occupancy grid sees "
                 "free space and drives in. The drop to the next cell is "
                 "0.55 m against a lethal threshold of 0.25 m, so the cost "
                 "function marks it impassable and the planner detours north. "
                 "No semantic model is involved.",
    },
    {
        "id": "freeze",
        "title": "Frozen camera",
        "world": "easy",
        "faults": [("camera_freeze", 9.0)],
        "headline": "The failure a liveness check cannot see",
        "blurb": "At 9 s the camera starts republishing one image with a "
                 "fresh timestamp. Frame age stays near zero, so the "
                 "staleness check never fires, while the view of the world "
                 "becomes arbitrarily old. A separate content signature "
                 "catches it and the vehicle halts at 11.0 s.",
    },
    {
        "id": "slam",
        "title": "Localisation lost",
        "world": "easy",
        "faults": [("slam_loss", 9.0)],
        "headline": "Pose-dependent navigation stops when the pose goes",
        "blurb": "Visual SLAM drops out at 9 s. The supervisor halts within "
                 "one tick. Nav2 is still asking for 1 m/s; the wheels get "
                 "zero.",
    },
    {
        "id": "medium",
        "title": "Slopes and trees",
        "world": "medium",
        "faults": [],
        "headline": "Terrain is scored, not just occupied",
        "blurb": "A climbable ramp costs more than flat ground but stays "
                 "passable. Tree trunks are lethal on geometry. The route "
                 "weighs both against the detour they would cost.",
    },
]


def record(spec, max_steps=2000):
    world = WORLDS[spec["world"]](0)
    faults = [Fault(t, kind, kind) for kind, t in spec["faults"]]
    sim = Simulation(world, faults=faults)

    w = world
    truth = []
    for cy in range(w.height):
        row = []
        for cx in range(w.width):
            row.append([round(w.height_at(cx, cy), 3), w.class_at(cx, cy)])
        truth.append(row)

    seen = [[False] * w.width for _ in range(w.height)]
    frames = []

    while not sim.finished and len(frames) < max_steps:
        t = sim.step()

        # Only cells observed for the first time this tick. Cost never changes
        # afterwards, so this is lossless.
        new = []
        for cy in range(w.height):
            for cx in range(w.width):
                if sim.observed[cy][cx] and not seen[cy][cx]:
                    seen[cy][cx] = True
                    new.append([cx, cy,
                                round(sim.cost[cy][cx], 3),
                                1 if sim.lethal[cy][cx] else 0])

        frames.append({
            "t": round(t.t, 2),
            "p": [round(sim.pose[0], 3), round(sim.pose[1], 3),
                  round(sim.pose[2], 4)],
            "a": int(t.action),
            "r": t.reason_text,
            "ci": [round(t.cmd_in[0], 3), round(t.cmd_in[1], 3)],
            "co": [round(t.cmd_out[0], 3), round(t.cmd_out[1], 3)],
            "ra": round(min(t.rgb_age, 99), 2),
            "da": round(min(t.depth_age, 99), 2),
            "sf": round(t.rgb_static_for, 2),
            "cf": round(t.confidence, 2),
            "ob": round(t.observed_fraction, 4),
            "dg": round(t.distance_to_goal, 2),
            "new": new,
            # The path is the expensive field; ship it every few frames and
            # subsample along its length. It is drawn as a guide, not measured.
            "path": ([[round(x, 2), round(y, 2)]
                      for x, y in sim.path[::3]]
                     if len(frames) % 3 == 0 else None),
            "f": list(t.active_faults),
        })

    return {
        "id": spec["id"],
        "title": spec["title"],
        "headline": spec["headline"],
        "blurb": spec["blurb"],
        "world": {
            "name": w.name,
            "w": w.width,
            "h": w.height,
            "res": w.resolution,
            "start": list(w.start),
            "goal": list(w.goal),
            "truth": truth,
        },
        "outcome": sim.outcome,
        "frames": frames,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", default=os.path.join(PROTO, "runs.json"))
    args = parser.parse_args(argv)

    runs = []
    for spec in SCENARIOS:
        run = record(spec)
        runs.append(run)
        print("%-8s %-18s %3d frames  outcome=%s"
              % (run["id"], run["world"]["name"], len(run["frames"]),
                 run["outcome"]))

    payload = {"runs": runs}
    text = json.dumps(payload, separators=(",", ":"))
    with open(args.out, "w", encoding="utf-8") as fh:
        fh.write(text)
    print("\nwrote %s  (%.1f KB)" % (args.out, len(text) / 1024.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
