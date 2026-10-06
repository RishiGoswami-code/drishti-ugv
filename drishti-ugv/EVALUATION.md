# EVALUATION — DRISHTI-UGV

How we measure whether this works. Written before the code so the targets
cannot be retrofitted to whatever the system happens to do.

---

## 1. Principle

> A single accuracy number is insufficient. A navigation system can post an
> excellent object-detection mAP and still drive into a ditch.

Mission-level outcomes are the score. Component metrics (mAP, ATE, latency) are
diagnostics that explain *why* a mission score moved — they are never the
headline.

Every number reported anywhere — report, figure, demo — must be reproducible from
a recorded run by re-running the evaluation harness.

---

## 2. Metrics

| Metric | Definition | Interpretation |
|---|---|---|
| **Mission success rate** | successful A→B missions / total missions | primary outcome |
| **Collision rate** | missions with any collision / total | must approach zero |
| Near-miss rate | fraction of missions where minimum clearance falls below threshold | safety margin sensitivity |
| Localisation ATE | absolute trajectory error vs simulator ground truth | visual localisation quality |
| Localisation RPE | relative pose error over a fixed window | short-term odometry quality |
| Localisation drift | ATE as a percentage of distance travelled | comparable across route lengths |
| Path efficiency | shortest feasible path length / actual path length | planner quality |
| Planning latency | map change → valid command | responsiveness |
| Perception latency | frame arrival → published output | real-time feasibility |
| Emergency-stop latency | unsafe condition raised → command blocked | safety quality |
| Compute utilisation | peak CPU, GPU, RAM, VRAM | deployment feasibility |
| Recovery success | blocked missions recovered without collision / blocked missions | robustness |

### 2.1 Definitions that need pinning

Ambiguity here is how numbers become dishonest. Fix these once, in code:

- **Success** = goal pose reached within `goal_tolerance`, with zero collisions,
  within `mission_timeout`. A safe halt short of the goal is *not* a success —
  it is a separate `safe_abort` outcome and is reported separately.
- **Collision** = any contact between the vehicle body and a non-ground object,
  as reported by the simulator's contact sensor. Not a proximity threshold.
- **Distance travelled** = integrated ground-truth path length, not commanded
  velocity integrated over time.
- **Emergency-stop latency** = timestamp of the injected fault → timestamp of
  the first `/cmd_vel` message with zero velocity, taken from the bag.
- **Outcome classes** are exhaustive: `success`, `collision`, `safe_abort`,
  `timeout`, `planner_failure`, `harness_error`. Every mission lands in exactly
  one.

---

## 3. Targets

| Metric | Prototype | Stretch |
|---|---|---|
| Collision-free completion | ≥ 95% of randomised missions | ≥ 99% |
| Goal completion | ≥ 97% | ≥ 99% |
| Emergency-stop response | < 200 ms | < 100 ms where hardware permits |
| Localisation drift | < 2% of distance travelled | < 1–2% |
| Perception latency | ≤ 100 ms | ≤ 60 ms preferred |
| Control loop | ≥ 20 Hz | ≥ 30 Hz preferred |
| Planner update | ≥ 5 Hz | ≥ 10 Hz preferred |

These are **our engineering targets**, not specifications of the upstream
software we build on.

---

## 4. Test pyramid

```
        ┌────────────────────────────┐
        │ Mission-level random tests │   100 → 1000+ seeded runs
        │ full stack, headless       │
        └─────────────┬──────────────┘
                      │
        ┌─────────────▼──────────────┐
        │ Scenario tests             │   T01–T20, fixed seeds
        │ terrain / obstacles / fail │
        └─────────────┬──────────────┘
                      │
        ┌─────────────▼──────────────┐
        │ Component tests            │   SLAM drift, perception
        │ one subsystem at a time    │   latency, costmap output
        └─────────────┬──────────────┘
                      │
        ┌─────────────▼──────────────┐
        │ Unit tests                 │   safety logic, cost maths,
        │ no ROS, no simulator       │   TF helpers
        └────────────────────────────┘
```

The safety supervisor's decision logic must be testable at the bottom layer,
with no ROS and no simulator running.

---

## 5. Experiment matrix

| Experiment | Variable | Measures |
|---|---|---|
| Baseline navigation | flat terrain | goal success |
| Terrain | slope, roughness | collision, path efficiency |
| Perception | lighting | detection/segmentation degradation |
| Localisation | texture and feature density | ATE, RPE |
| Dynamic obstacle | obstacle speed | avoidance success |
| Sensor failure | camera/depth loss | time to stop |
| Planner stress | narrow passages | planning latency |
| Compute stress | higher sensor FPS | CPU/GPU/latency |
| Cost weights | `w_s, w_r, w_h, w_o, w_m, w_u` | mission success, path efficiency |
| Randomised missions | all variables | overall mission success |

**Cost-weight tuning is an experiment, not a preference.** Every weight set
gets a suite run, and the pair (weights → result) is recorded in STATUS.md.
Weights that were never scored do not ship.

---

## 6. Scenario catalog

Each scenario runs with fixed seeds for regression comparison, and with random
seeds inside the mission suite.

| ID | Scenario | Purpose | Severity |
|---|---|---|---|
| T01 | Flat open dirt | baseline | Low |
| T02 | Sparse rocks | obstacle detection | Low |
| T03 | Dense rocks | obstacle avoidance | Medium |
| T04 | Tree corridor | semantic obstacles | Medium |
| T05 | Positive slope | terrain geometry | Medium |
| T06 | Negative slope | terrain geometry | Medium |
| T07 | Ditch crossing | safety / traversability | High |
| T08 | Mud patch | semantic terrain | High |
| T09 | Water-like surface | semantic + geometry | High |
| T10 | Low light | vision robustness | High |
| T11 | Strong sunlight / glare | vision robustness | High |
| T12 | Moving pedestrian | dynamic avoidance | High |
| T13 | Moving vehicle | dynamic avoidance | High |
| T14 | Sudden obstacle | reaction latency | High |
| T15 | Featureless terrain | SLAM robustness | High |
| T16 | Camera dropout | safety | Critical |
| T17 | Depth dropout | safety | Critical |
| T18 | SLAM loss | safety / recovery | Critical |
| T19 | No valid path | planner recovery | Critical |
| T20 | Randomised mission | end to end | Critical |

**Critical scenarios have a different bar.** T16–T19 do not require reaching
the goal. They require a *safe halt*, correctly attributed on `/safety/state`,
within the stop-latency budget. Reaching the goal during a sensor failure would
be a failure of the safety design.

---

## 7. Method

### 7.1 Ground truth

The simulator provides exact vehicle pose and object geometry. Ground truth is
recorded **alongside** every rosbag2 run so localisation error, obstacle
distance and path deviation are computed objectively rather than estimated.

A run without its ground-truth track is not evaluable and does not count.
`drishti_eval.bag_reader.load_run` enforces that in code: it raises rather than
returning a half-populated result.

Ground truth reaches the bag on `/ground_truth/pose` and is **never**
subscribed at run time. `tools/check_wiring.py` fails the build if any launch
file references it outside a comment. A pose that leaks into the runtime graph
would make every localisation number in this project meaningless.

### 7.2 What every run records

- Full rosbag2: sensors, TF, pose, costmaps, plan, `/cmd_vel`, `/safety/state`
- Ground-truth pose track and object poses
- Scenario id, random seed, and the complete parameter set in effect
- Software versions (ROS 2, Gazebo, CUDA, driver, commit hash)

Without the seed and the parameter set, a result is an anecdote.

### 7.3 Reporting

The harness emits, from one command:

1. **Per-run record** — outcome class, metrics, and a link to the bag.
2. **Suite summary** — success and collision rates with counts, per-scenario
   breakdown, latency distributions.
3. **Regression view** — this suite versus the previous recorded suite.

Report **counts alongside percentages**. "95%" over 20 runs and over 1000 runs
are not the same claim, and the report must not blur them.

---

## 8. Demonstration

A 5-minute live demonstration sequence, derived from the same harness:

1. Show the vehicle and the destination.
2. Show the camera feed and semantic perception.
3. Show the live elevation / traversability map.
4. Show localisation running with no GPS in the graph.
5. Set Point A and Point B.
6. Let Nav2 drive.
7. Introduce a sudden obstacle.
8. Show the local replan or avoidance.
9. Introduce an unsafe condition — freeze the camera.
10. Show the automatic slow-down or emergency stop, with the reason code.
11. Display the mission metrics from the recorded suite.

### 8.1 What a reviewer should be able to see

- The robot understands *terrain*, not just a painted path.
- It localises visually, with no GPS.
- It reacts to obstacles that appear after planning.
- It behaves conservatively when uncertain — and we show that on purpose.
- The architecture is modular and can move to real hardware.
- The performance claims are backed by numbers we can reproduce live.

Step 9 is the most important one. A system that stops correctly under an
injected fault is a stronger result than one that completes a clean run.
