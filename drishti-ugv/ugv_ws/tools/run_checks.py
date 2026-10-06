#!/usr/bin/env python3
"""Run every offline check in one go.

None of these need ROS, a GPU or a simulator, so they run on any machine the
team happens to have open. They are the only automated confidence this project
has until the first colcon build (STATUS.md).

    python tools/run_checks.py
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)

# (title, path relative to the workspace root)
CHECKS = [
    ("contract sync (msg constants, param defaults)", "tools/check_contract_sync.py"),
    ("robot description (SPEC 3 frame tree)", "tools/check_robot_description.py"),
    ("command path (SPEC 9.4.1 /cmd_vel ownership)", "tools/check_wiring.py"),
    ("simulation assets (worlds, systems, collisions)", "tools/check_sim_assets.py"),
    ("localisation metrics (ATE, RPE, alignment)",
     "src/drishti_eval/test/test_metrics.py"),
    ("run report (drift target, alignment disclosure)",
     "src/drishti_eval/test/test_report.py"),
    ("perception (taxonomy, health, obstacle distance)",
     "src/drishti_perception/test/test_perception.py"),
    ("safety harness (fault schedules, stop latency)",
     "src/drishti_eval/test/test_safety_harness.py"),
    ("mission suite (outcome classification, seeded scenarios)",
     "src/drishti_eval/test/test_suite.py"),
    ("phase 7 gates (budgets, regression power)",
     "src/drishti_eval/test/test_phase7.py"),
]

results = []
for title, script in CHECKS:
    print("=" * 72)
    print(title)
    print("=" * 72)
    code = subprocess.call([sys.executable, os.path.join(WS, script)])
    results.append((title, code))
    print()

print("=" * 72)
for title, code in results:
    print("%-4s %s" % ("PASS" if code == 0 else "FAIL", title))

failed = [t for t, c in results if c != 0]
print()
print("NOTE: two C++ suites are separate and need a compiler:")
print("  cd src/drishti_safety && g++ -std=c++17 -Iinclude \\")
print("      src/supervisor_core.cpp test/test_supervisor_core.cpp -o t && ./t")
print("  cd src/drishti_traversability && g++ -std=c++17 -Iinclude \\")
print("      src/traversability_core.cpp test/test_traversability_core.cpp -o t && ./t")
sys.exit(1 if failed else 0)
