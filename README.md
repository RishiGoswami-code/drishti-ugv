# DRISHTI-UGV

**Vision-first, GPS-denied navigation for outdoor ground vehicles, with a deterministic safety supervisor.**

---

## Abstract

Outdoor unmanned ground vehicles cannot count on GPS, benign light or benign
terrain. This project studies a camera-first navigation stack that drives a
vehicle from a start point to a goal with no GNSS anywhere in the runtime graph,
and that treats *failure of its own perception* as a first-class design problem
rather than an afterthought.

The stack is a hybrid. Mature open-source components carry the infrastructure
(RTAB-Map for visual SLAM, `elevation_mapping_cupy` for terrain, Nav2 with an
MPPI controller for planning and control). The original engineering is at the
three places where this problem is specific: a **camera-to-traversability cost
layer** that fuses terrain geometry with a small navigation-relevant semantic
vocabulary and prices unknown terrain as expensive; a **deterministic safety
supervisor** that sits outside every learned component and is the only publisher
of vehicle motion commands; and an **evaluation harness** that scores whole
missions against simulator ground truth rather than reporting detector accuracy.

This repository contains the architecture, the interface contract, the
implementation, and the evaluation protocol. It states plainly which parts have
been exercised and which have not (see *Status*): mission-level results over the
full test suite have **not** yet been produced, and none are claimed here.

---

## 1. Problem

Three coupled sub-problems:

1. **Path detection** — distinguishing safe, traversable ground from hazards
   (rocks, ditches, trees, water) in real time.
2. **Visual localisation** — estimating position and orientation without GPS.
3. **Collision avoidance** — re-routing around obstacles that appear after
   planning, toward a goal.

Why it is hard:

- *Free space is not safe space.* A ditch or a water surface is geometrically
  open and physically lethal; binary occupancy is the wrong representation.
- *Vision fails quietly.* Glare, shadow, low light and lens contamination degrade
  perception without raising an error.
- *There is no ground truth at run time.* Without GPS, a drifting pose has nothing
  to correct it except the visual map itself.
- *Integration, not algorithms, is the usual failure.* Frame, timestamp and
  calibration errors cause more failures than model accuracy does.

## 2. Contributions

1. **Cost-based terrain representation.** Traversability is a continuous cost
   fused from elevation-map geometry (step, slope, roughness) and a 19-class
   semantic vocabulary. Unobserved, low-visibility and low-confidence cells carry a
   high fixed cost, so the planner routes around ignorance instead of gambling on
   it. A negative obstacle such as a ditch is refused on step height alone, with
   no semantic model in the loop.
2. **A supervisor the stop decision can be audited through.** The supervisor is
   ROS-free in its decision core, contains no learned components, and owns
   `/cmd_vel` outright (Nav2 publishes to `/cmd_vel_nav`). All stop conditions are
   evaluated before the slow-down condition, so a low-confidence reading can never
   mask a harder fault. Loss of any input — camera, depth, pose, plan, command — is
   a stop, never a pass-through.
3. **A frozen-camera condition distinct from a stale one.** A camera that goes
   silent is caught by an age threshold. A camera that keeps republishing the same
   frame with a fresh timestamp is not: its age never grows. The perception node
   reports how long frame content has been unchanged, and the supervisor stops on
   that independently of age.
4. **A statistically honest evaluation harness.** Seeded scenario generation,
   outcome classification, fault injection for four failure families, stop-latency
   measurement, and a regression gate with an explicit power check: detecting a
   2-point drop from a 95% baseline needs roughly 1,470 missions per side, and
   below that the gate reports *underpowered* rather than *passed*.

## 3. System overview

```
 stereo / depth camera + IMU
        │
        ├─► perception ──► detections, semantic mask, health ─┐
        │                                                      ├─► traversability fusion ─► Nav2 costmap layer
        └─► RTAB-Map (visual SLAM) ─► map→odom, pose ─────────┘                                  │
                                                                                                  ▼
                                                              Nav2 planner + MPPI controller ─► /cmd_vel_nav
                                                                                                  │
          /perception/health · /rtabmap/localization_pose · /plan · obstacle range ─────────► safety supervisor ─► /cmd_vel ─► base
```

| Layer | Choice |
|---|---|
| OS / middleware | Ubuntu 24.04, ROS 2 Jazzy |
| Simulator | Isaac Sim 6.x (primary), Gazebo Harmonic (fallback, CI) |
| Localisation | RTAB-Map (stereo + IMU) |
| Terrain | `elevation_mapping_cupy` (CUDA) |
| Planning / control | Nav2, MPPI |
| Perception | pretrained detector / segmenter (not trained here), stereo depth |
| Safety | custom deterministic supervisor (C++) |

The interface between subsystems — topics, types, frames and clocks — is fixed in
[`SPEC.md`](drishti-ugv/SPEC.md) so the simulator and a real sensor driver publish
the same contract and hardware transfer is a driver swap.

## 4. Evaluation protocol

The score is the mission, not the detector. A system can post an excellent
detection mAP and still drive into a ditch.

| Metric | Prototype target | Stretch target |
|---|---|---|
| Collision-free completion | ≥ 95% of randomised missions | ≥ 99% |
| Goal completion | ≥ 97% | ≥ 99% |
| Emergency-stop response | < 200 ms | < 100 ms |
| Localisation drift | < 2% of distance | < 1–2% |
| Perception latency | ≤ 100 ms | ≤ 60 ms |

These are **targets**, not results. Every reported number must carry its seed and
parameter set; a number without them is an anecdote (see
[`EVALUATION.md`](drishti-ugv/EVALUATION.md)). Terrain difficulty runs from flat
dirt through slopes and a ditch across the route; the failure catalog covers
camera dropout, depth dropout, localisation loss and no valid path, plus the
frozen-camera case above.

## 5. Status

What has been verified, and how:

| Item | Evidence |
|---|---|
| Traversability cost function, supervisor core, perception logic, evaluation harness | ~5,000 offline assertions, no ROS or GPU required |
| Python/JS port of the decision logic matches the shipping C++ | 8,000-case oracle comparison; 520 baked decisions re-checked in the browser |
| Workspace builds | `colcon build`, all 8 packages, on a hosted Ubuntu 24.04 runner |
| Supervisor behaves on real ROS 2 | launched with no inputs it publishes `STOP` / `LOCALIZATION_LOST` with zero velocity; `/cmd_vel` has exactly one publisher |
| Gazebo + bridge | headless start, bridges created from the config |
| Isaac Sim backend | written against the 6.1 API and checked offline for consistency with the topic contract; **never run** |

What has **not** been done: Nav2 lifecycle bring-up is currently aborted by an
unconfigured `collision_monitor` node; the Isaac Sim backend has never been run, since no machine in hand meets its
hardware minimum; no SLAM run, drift measurement, GPU terrain
run, detector run or mission suite has been executed; there is no hardware. The
running log of what is open lives in [`STATUS.md`](drishti-ugv/STATUS.md).

## 6. Reproducing

Offline checks (Python 3 only; no ROS, GPU or simulator):

```bash
cd drishti-ugv/ugv_ws
python tools/run_checks.py
```

Supervisor core, with a C++17 compiler:

```bash
cd drishti-ugv/ugv_ws/src/drishti_safety
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
    src/supervisor_core.cpp test/test_supervisor_core.cpp -o test_sup && ./test_sup
```

Interactive demonstration of the decision logic (ROS-free, no install):

```bash
cd prototype && python run_demo.py
```

The full stack: [`drishti-ugv/FREE_SETUP.md`](drishti-ugv/FREE_SETUP.md) covers what
runs without paid compute (a devcontainer and a CI workflow are included);
[`drishti-ugv/CLOUD_SETUP.md`](drishti-ugv/CLOUD_SETUP.md) covers a rented GPU
machine and the container in `drishti-ugv/docker/`.

## 7. Repository layout

```
drishti-ugv/    documentation set, ROS 2 workspace (ugv_ws/) and container definition
prototype/      ROS-free demonstration of the decision logic, parity-checked against the C++
papers/         reading list for the literature behind the design (PDFs are kept out of git)
.devcontainer/  CPU-only development environment
.github/        CI: build and exercise the workspace on Ubuntu 24.04
```

Start with [`drishti-ugv/README.md`](drishti-ugv/README.md) for the index of
documents.

## 8. Limitations

- Simulation only. Sim-to-real gaps in lighting and texture are expected; numbers
  from simulation are relative, not absolute.
- The prototype assumes exact localisation. It demonstrates terrain reasoning and
  the safety gate, **not** that GPS-denied localisation works; that claim needs the
  RTAB-Map stack and a measured trajectory error.
- Pretrained perception only; no model is trained here, and no detector accuracy is
  claimed.
- No claim of 100% accuracy is made or implied.

## 9. Licence

Apache-2.0 (see [`LICENSE`](LICENSE)). Third-party licences were screened before
integration and are recorded in [`REFERENCES.md`](drishti-ugv/REFERENCES.md); notably
ORB-SLAM3 (GPLv3) is an offline benchmark only and is excluded from the build, and
the AGPL detector package is not installed by default.
