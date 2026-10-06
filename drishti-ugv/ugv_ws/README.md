# `ugv_ws` — ROS 2 workspace

The colcon workspace for DRISHTI-UGV. Built against **ROS 2 Jazzy** on Ubuntu
24.04 (SETUP.md §1).

```bash
cd ugv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> **`colcon build` is unverified.** Nothing here has been compiled by a ROS
> toolchain or executed — the team is building the stack ahead of the install
> (STATUS.md **D17**). Treat the first successful build as the real Phase 0
> acceptance event and record the exact toolchain versions in STATUS.md when it
> happens. Expect a batch of API-level fixes concentrated in the ROS wrappers.

### What *is* verified

The supervisor's decision core has no ROS dependency, by design, so it can be
compiled and tested on any machine with a C++17 compiler — including one that
cannot run ROS at all. **It compiles clean under `-Wall -Wextra -Wpedantic
-Werror` and passes 377 checks.**

```bash
cd src/drishti_safety
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
    src/supervisor_core.cpp test/test_supervisor_core.cpp -o test_sup && ./test_sup
```

The same file is registered as a CTest target, so `colcon test` runs it too.

```bash
python tools/run_checks.py
```

runs every offline suite: the contract-sync check (`SafetyState.msg` constants
against the C++ enums, header defaults against `drishti.yaml`), the robot
description against the SPEC.md §3 frame tree, the `/cmd_vel` ownership
invariant, and the `drishti_eval` metric tests. **Currently 5/5.**

The localisation metrics are worth calling out: ATE, RPE and drift are pure
numpy with no ROS dependency, and every test case has an answer derivable by
hand — a known rigid transform Umeyama must recover exactly, a sinusoidal error
whose RMS is A/√2, a scale error that rigid alignment must *not* hide. A subtly
wrong ATE would let us report a drift figure we had not earned.

Everything else — the ROS node, the launch file — is **uncompiled and
unexecuted**.

## Packages

| Package | Type | Contents | State |
|---|---|---|---|
| `drishti_msgs` | `ament_cmake` + rosidl | `SafetyState`, `PerceptionHealth` | syntax-checked |
| `drishti_bringup` | `ament_cmake` | shared params, `safety.launch.py` | syntax-checked |
| `drishti_safety` | `ament_cmake` | supervisor core + ROS node | **core tested**, node uncompiled |
| `drishti_description` | `ament_cmake` | UGV xacro, stereo + depth + IMU | frame tree checked |
| `drishti_sim` | `ament_cmake` | Gazebo worlds, `ros_gz` bridge | XML/YAML checked |
| `drishti_eval` | `ament_python` | ATE, RPE, drift, run report | **64 checks, tested** |
| `drishti_traversability` | `ament_cmake` | SPEC §6.1 cost, Nav2 layer | **1217 checks, tested** |
| `drishti_perception` | `ament_python` | taxonomy, health, obstacle distance | **198 checks, tested** |

Everything else in the system uses standard ROS 2 interfaces. Two custom
messages exist because nothing standard carries them (SPEC.md §4).

### `drishti_msgs`

**`SafetyState`** — published every supervisor tick on `/safety/state`. It is
the audit record of the stop decision: the action taken, the reason code, what
was actually published on `/cmd_vel`, and the evidence behind it. The evidence
fields are populated even when the action is `ACTION_PASS`, so a recorded run
can be replayed and the decision boundary inspected without re-running
perception.

Reason codes follow the SPEC.md §9.1 *listing* order, which is **not** the
evaluation order: every STOP condition is checked before the single SLOW
branch, so `LOW_CONFIDENCE` (5) is evaluated after `PATH_INVALID` (6) and
`COMMAND_INVALID` (7). Do not infer precedence from the numbers.

**`PerceptionHealth`** — published on `/perception/health`. Liveness and
quality only. The supervisor consumes this to decide whether perception can be
trusted at all; it never looks inside the model.

### `drishti_bringup`

Holds no algorithm code. It exists so every node reads its thresholds and its
clock setting from one file, `config/drishti.yaml`.

**The values in that file are initial defaults, not tuned numbers.** SPEC.md
§9.3 fixes the parameter names and meanings; it does not fix the values. Phase
5 tunes them against the randomised suite.

## Two things to check on the first real build

1. **`use_twist_stamped`** — Nav2 accepts `Twist` or `TwistStamped` depending
   on version and configuration. Pin it once against the Nav2 actually
   installed and keep it identical across Nav2, the supervisor and the base.
   Do not assume.
2. **The `/cmd_vel_nav` → `/cmd_vel` split** — Nav2 must publish to
   `cmd_vel_in` and must have no route to the base. The supervisor is the only
   publisher on `/cmd_vel`. If Nav2 can reach the base directly, the safety
   design is void (SPEC.md §4.3, §9.4.1).

### `drishti_safety`

The supervisor, split in two on purpose:

- **`supervisor_core.{hpp,cpp}`** — all the policy, zero dependencies beyond
  the standard library. One function, `evaluate()`, decides everything. It is
  pure: same inputs, same decision, no clock, no I/O.
- **`safety_supervisor_node.cpp`** — collects evidence, ticks on its own timer,
  calls `evaluate()`, publishes. Holds no policy.

That split is what makes invariant 9.4.5 real rather than aspirational.

**The evaluation order diverges from the SPEC.md §9.1 pseudocode as originally
written, and the spec has been corrected to match** — see §9.1 and D13. Briefly:
every STOP condition is now checked before the single SLOW branch, because the
original order let low perception confidence mask a missing path and forward a
command anyway.

## Layout

```
ugv_ws/
├── tools/                       offline checks, no ROS needed
│   ├── run_checks.py                runs all of the below
│   ├── check_contract_sync.py       .msg constants vs C++ enums vs YAML
│   ├── check_robot_description.py   SPEC 3 frame tree
│   └── check_wiring.py              SPEC 9.4.1 /cmd_vel ownership
└── src/
    ├── drishti_msgs/            SafetyState, PerceptionHealth
    ├── drishti_description/     drishti.urdf.xacro
    ├── drishti_sim/             worlds/{easy,medium}.sdf, config/bridge.yaml
    ├── drishti_bringup/         drishti.yaml, nav2.yaml, rtabmap.yaml, launch/
    ├── drishti_safety/          supervisor core + node + tests
    ├── drishti_traversability/  cost core + fusion node + Nav2 layer
    ├── drishti_perception/      taxonomy, health, obstacle, detector, node
    └── drishti_eval/            trajectory, metrics, report, bag_reader
```

## The pattern

Every package splits the same way: **judgement in a core with no ROS
dependency, plumbing in a thin wrapper.** The cores are the parts that would be
dangerous to get wrong, and they are the parts that are actually tested —
1217 + 377 + 198 + 64 checks, none of which need ROS, a GPU or a simulator.

The wrappers are uncompiled and unrun. That is the honest state, and every one
of them says so in its own header rather than reading as working code.

`build/`, `install/` and `log/` are ignored.
