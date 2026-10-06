#!/usr/bin/env python3
"""DRISHTI-UGV prototype — live demonstration.

Runs the REAL terrain cost function and the REAL safety supervisor over a small
simulated world, so you can watch the decisions the shipping stack would make.
tools/check_parity.py proves the ported logic matches the C++ on 8000 cases.

    python run_demo.py                       # live window, Hard world (ditch)
    python run_demo.py --world easy
    python run_demo.py --fault camera_freeze # the D19 failure
    python run_demo.py --headless            # terminal, no GUI
    python run_demo.py --list                # scenarios

Needs Python 3.8+ and nothing else. tkinter ships with Python; --headless works
without it.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drishti_proto.sim import Fault, Simulation, TAXONOMY_SOURCE  # noqa: E402
from drishti_proto.supervisor import Action                        # noqa: E402
from drishti_proto.world import WORLDS                             # noqa: E402

FAULTS = {
    "none": [],
    "camera_freeze": [Fault(9.0, "camera_freeze",
                            "T16 camera frozen (fresh stamps, stale content)")],
    "camera_silence": [Fault(9.0, "camera_silence", "T16 camera dropout")],
    "depth_silence": [Fault(9.0, "depth_silence", "T17 depth dropout")],
    "slam_loss": [Fault(9.0, "slam_loss", "T18 localisation lost")],
}

SCENARIOS = """
worlds
  easy      flat dirt, sparse rocks              the baseline
  medium    slopes, roughness, tree trunks       terrain reasoning
  hard      a ditch across the route (T07)       the one that matters

faults
  none            a clean run
  camera_freeze   T16 hard half: frames keep arriving with fresh timestamps
                  and unchanging content. Liveness checks see a healthy
                  camera; only the content signature catches it (D18/D19).
  camera_silence  T16 easy half: the stream stops
  depth_silence   T17
  slam_loss       T18
"""


def build(world_name: str, fault_name: str, seed: int) -> Simulation:
    return Simulation(WORLDS[world_name](seed), faults=FAULTS[fault_name],
                      seed=seed)


def headless(sim: Simulation, max_steps: int = 2000, quiet: bool = False) -> int:
    """Terminal run. Prints every supervisor state change, then a summary."""
    print("world      %s" % sim.world.name)
    print("           %s" % sim.world.description)
    print("taxonomy   %s" % TAXONOMY_SOURCE)
    if sim.faults:
        print("faults     %s" % ", ".join(f.label or f.kind for f in sim.faults))
    print()
    print("   t(s)   action  reason                                  "
          "v(m/s)  toGoal")
    print("   " + "-" * 74)

    last = None
    steps = 0
    while not sim.finished and steps < max_steps:
        t = sim.step()
        steps += 1
        key = (t.action, t.reason)
        if key != last:
            print("  %6.1f   %-6s  %-38s %5.2f   %5.2f"
                  % (t.t, t.action.name, t.reason_text or "-",
                     t.cmd_out[0], t.distance_to_goal))
            last = key
        elif not quiet and steps % 50 == 0:
            print("  %6.1f   %-6s  %-38s %5.2f   %5.2f"
                  % (t.t, t.action.name, "...", t.cmd_out[0],
                     t.distance_to_goal))

    t = sim.telemetry
    print("   " + "-" * 74)
    print()
    print("outcome        %s" % (sim.outcome or "still running").upper())
    print("elapsed        %.1f s" % t.t)
    print("distance left  %.2f m" % t.distance_to_goal)
    print("map observed   %.1f %%" % (100 * t.observed_fraction))
    print("lethal cells   %d" % t.lethal_cells)
    if t.active_faults:
        print("faults active  %s" % ", ".join(t.active_faults))
    return 0 if sim.outcome in ("success", "safe_abort") else 1


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=SCENARIOS)
    parser.add_argument("--world", default="hard", choices=sorted(WORLDS))
    parser.add_argument("--fault", default="none", choices=sorted(FAULTS))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--headless", action="store_true",
                        help="terminal only, no window")
    parser.add_argument("--speed", type=int, default=60,
                        help="ms between frames in the GUI; lower is faster")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--list", action="store_true",
                        help="show the scenarios and exit")
    args = parser.parse_args(argv)

    if args.list:
        print(SCENARIOS)
        return 0

    sim = build(args.world, args.fault, args.seed)

    if args.headless:
        return headless(sim, quiet=args.quiet)

    try:
        from drishti_proto.gui import DemoWindow
    except Exception as exc:                       # noqa: BLE001
        print("No GUI available (%s). Falling back to --headless.\n" % exc)
        return headless(sim, quiet=args.quiet)

    window = DemoWindow(sim, speed_ms=args.speed)
    window.set_restart_factory(lambda: build(args.world, args.fault, args.seed))
    window.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
