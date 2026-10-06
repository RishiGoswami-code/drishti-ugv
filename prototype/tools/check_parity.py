#!/usr/bin/env python3
"""Prove the prototype's Python ports match the shipping C++ cores.

The demo is only worth showing if it runs the real decision logic. It cannot
import the C++ directly without ROS, so drishti_proto ports it -- and a port is
a liability until it is proven equal.

parity_oracle.cpp emits the C++ answer for a grid of inputs, including NaN,
infinities and every threshold boundary. This replays the identical grid
through the Python ports and demands the same answer.

    python tools/check_parity.py            # build the oracle and compare
    python tools/check_parity.py --csv f    # compare against an existing dump

Needs a C++17 compiler for the first form. If none is available it says so and
exits non-zero rather than quietly passing -- an unverified port is exactly
what this exists to prevent.
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO = os.path.dirname(HERE)
ROOT = os.path.dirname(PROTO)
sys.path.insert(0, PROTO)

from drishti_proto import supervisor as S      # noqa: E402
from drishti_proto import traversability as T  # noqa: E402

WS = os.path.join(ROOT, "drishti-ugv", "ugv_ws", "src")
SAFETY = os.path.join(WS, "drishti_safety")
TERRAIN = os.path.join(WS, "drishti_traversability")

failures = []
compared = 0


def fail(msg):
    failures.append(msg)
    if len(failures) <= 20:
        print("  FAIL  " + msg)


def build_oracle(out_path):
    """Compile parity_oracle.cpp against the real cores."""
    cmd = [
        "g++", "-std=c++17", "-O2",
        "-I" + os.path.join(SAFETY, "include"),
        "-I" + os.path.join(TERRAIN, "include"),
        os.path.join(SAFETY, "src", "supervisor_core.cpp"),
        os.path.join(TERRAIN, "src", "traversability_core.cpp"),
        os.path.join(HERE, "parity_oracle.cpp"),
        "-o", out_path,
    ]
    print("building the oracle against the shipping C++ cores...")
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        print(proc.stdout)
        print(proc.stderr)
        return False
    return True


def f(text):
    """Parse a C++ %.17g float, including inf and nan spellings."""
    t = text.strip().lower()
    if t in ("nan", "-nan"):
        return float("nan")
    if t in ("inf", "infinity"):
        return float("inf")
    if t in ("-inf", "-infinity"):
        return float("-inf")
    return float(text)


def same(a, b, tol=1e-12):
    """Equal, treating NaN as equal to NaN."""
    if isinstance(a, float) and isinstance(b, float):
        if math.isnan(a) and math.isnan(b):
            return True
        if math.isinf(a) or math.isinf(b):
            return a == b
        return abs(a - b) <= tol
    return a == b


def compare(csv_text):
    global compared
    section = None
    core_s = S.SupervisorCore(S.Params())
    core_t = T.TraversabilityCore(T.Weights(), T.Limits())

    for line in csv_text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            section = line[1:].strip()
            continue
        if line[0].isalpha():          # header row
            continue

        parts = line.split(",")

        if section == "supervisor":
            inp = S.Inputs(
                now=f(parts[0]), last_rgb_stamp=f(parts[1]),
                last_depth_stamp=f(parts[2]), rgb_static_for=f(parts[3]),
                pose_valid=parts[4] == "1", pose_covariance_max=f(parts[5]),
                nearest_obstacle=f(parts[6]), perception_confidence=f(parts[7]),
                path_valid=parts[8] == "1", cmd_linear_x=f(parts[9]),
                cmd_angular_z=f(parts[10]))
            want = (int(parts[11]), int(parts[12]), f(parts[13]),
                    f(parts[14]), f(parts[15]), parts[16] == "1")
            got = core_s.evaluate(inp)
            mine = (int(got.action), int(got.reason), got.v_limit,
                    got.linear_x, got.angular_z, got.stop)
            compared += 1
            if not all(same(a, b) for a, b in zip(mine, want)):
                fail("supervisor: inputs=%s\n          C++=%s\n          py =%s"
                     % (parts[:11], want, mine))

        elif section == "traversability":
            cell = T.Cell(
                observed=parts[0] == "1", slope=f(parts[1]),
                roughness=f(parts[2]), height_variance=f(parts[3]),
                step_height=f(parts[4]), semantic_cost=f(parts[5]),
                semantic_lethal=parts[6] == "1", visibility=f(parts[7]),
                confidence=f(parts[8]))
            want = (f(parts[9]), parts[10] == "1", parts[11] == "1",
                    int(parts[12]))
            got = core_t.evaluate(cell)
            mine = (got.cost, got.lethal, got.unknown,
                    T.TraversabilityCore.to_costmap(got))
            compared += 1
            if not all(same(a, b) for a, b in zip(mine, want)):
                fail("traversability: inputs=%s\n          C++=%s\n          py =%s"
                     % (parts[:9], want, mine))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--csv", help="use an existing oracle dump")
    args = parser.parse_args()

    if args.csv:
        csv_text = open(args.csv, encoding="utf-8").read()
    else:
        exe = os.path.join(tempfile.gettempdir(),
                           "drishti_parity_oracle" +
                           (".exe" if os.name == "nt" else ""))
        if not build_oracle(exe):
            print("\nCould not build the oracle. A C++17 compiler is required "
                  "to verify the ports.\nWithout it the prototype's logic is "
                  "UNVERIFIED against the shipping cores; do not present it as "
                  "the real thing.")
            return 2
        proc = subprocess.run([exe], capture_output=True, text=True)
        if proc.returncode != 0:
            print("oracle failed to run:\n" + proc.stderr)
            return 2
        csv_text = proc.stdout

    compare(csv_text)

    print("\ncompared %d decisions against the shipping C++ cores" % compared)
    if compared == 0:
        print("NOTHING WAS COMPARED -- the oracle produced no rows.")
        return 1
    if failures:
        print("%d MISMATCH(ES). The Python ports do not match the real logic."
              % len(failures))
        return 1
    print("parity: OK -- the prototype runs the same decisions as the "
          "shipping supervisor and cost function")
    return 0


if __name__ == "__main__":
    sys.exit(main())
