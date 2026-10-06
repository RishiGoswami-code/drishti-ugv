# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Emit a reproducible mission plan.

    python -m drishti_eval.plan_suite --count 100 --base-seed 0 --json plan.json

EVALUATION.md 7.2: without the seed and the parameter set, a result is an
anecdote. This writes both, so a suite result can be regenerated -- in whole or
one run at a time -- by anyone with the plan file.

Needs no ROS. Executing the plan does; that part is Phase 6's remaining work.
"""
import argparse
import json
import sys

from .faults import FAILURE_SCENARIOS
from .scenarios import coverage, missing_scenarios, suite


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--base-seed", type=int, default=0)
    parser.add_argument("--world", default=None,
                        help="restrict to one world; default is a mix")
    parser.add_argument("--fault-scenario", default=None,
                        choices=sorted(FAILURE_SCENARIOS),
                        help="inject a failure scenario into every mission")
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--require", nargs="*", default=[],
                        help="scenario ids the plan must cover, e.g. T01 T07")
    parser.add_argument("--json", help="write the full plan here")
    args = parser.parse_args(argv)

    missions = suite(args.count, base_seed=args.base_seed,
                     world=args.world, fault_scenario=args.fault_scenario,
                     mission_timeout_s=args.timeout)

    counts = coverage(missions)
    print("%d missions, seeds %d..%d"
          % (len(missions), args.base_seed, args.base_seed + args.count - 1))
    for scenario_id, n in sorted(counts.items()):
        print("  %-6s %4d" % (scenario_id, n))

    missing = missing_scenarios(missions, args.require)
    if missing:
        # A suite that never touched T07 has not tested the ditch, however good
        # its headline rate looks.
        print("\nNOT COVERED: %s" % ", ".join(missing))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({
                "base_seed": args.base_seed,
                "count": args.count,
                "coverage": counts,
                "missions": [m.as_dict() for m in missions],
            }, fh, indent=2, sort_keys=True)
        print("\nwrote %s" % args.json)

    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
