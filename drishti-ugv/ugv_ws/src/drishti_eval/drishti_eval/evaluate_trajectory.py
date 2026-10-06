# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""CLI: localisation error for one recorded run.

    ros2 run drishti_eval evaluate_trajectory <bag> [--json out.json]

!! UNVERIFIED !! Never executed; needs rosbag2. The metrics it calls ARE
tested (test/test_metrics.py).
"""
import argparse
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("--estimate-topic", default="/rtabmap/localization_pose")
    parser.add_argument("--ground-truth-topic", default="/ground_truth/pose")
    parser.add_argument("--rpe-delta", type=float, default=1.0)
    parser.add_argument("--rpe-unit", default="m", choices=["m", "s", "frames"])
    parser.add_argument("--max-difference", type=float, default=0.02,
                        help="association window, seconds")
    parser.add_argument(
        "--with-scale", action="store_true",
        help="Fit scale during alignment. ONLY for a monocular ablation: with "
             "stereo the scale is observable and fitting it hides real error.")
    parser.add_argument("--json", help="also write the full report here")
    args = parser.parse_args(argv)

    from .bag_reader import load_run
    from .report import evaluate, format_text, to_json

    estimate, truth = load_run(
        args.bag, args.estimate_topic, args.ground_truth_topic)
    report = evaluate(estimate, truth,
                      with_scale=args.with_scale,
                      rpe_delta=args.rpe_delta,
                      rpe_unit=args.rpe_unit,
                      max_difference=args.max_difference)

    print(format_text(report, title=args.bag))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            fh.write(to_json(report))
        print("\nwrote %s" % args.json)

    return 0 if report["ate"]["meets_target"] else 1


if __name__ == "__main__":
    sys.exit(main())
