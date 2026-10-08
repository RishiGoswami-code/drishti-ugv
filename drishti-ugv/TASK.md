# TASK — DRISHTI-UGV

Phased backlog. **Phase N+1 does not start until Phase N ships its runnable
artefact.** This is the mechanism that stops a sophisticated model from hiding
a broken TF tree, odometry or navigation foundation.

Legend: `[ ]` todo · `[~]` in progress · `[x]` done · `[!]` blocked

Durations are estimates from the research baseline, assuming one workstation
and a small team working in parallel where tasks are independent.

---

## Phase 0 — Environment · 1–2 days

**Artefact:** a machine that builds and runs an empty ROS 2 workspace, with
the simulator verified, TF and clock health provable.

- [ ] Install Ubuntu 24.04 and a matching NVIDIA driver
- [x] Confirm GPU meets the Isaac Sim floor — **it does not; Isaac Sim is the primary simulator regardless (D21), Gazebo Harmonic the fallback and the CI simulator**
- [ ] Install ROS 2 Jazzy; verify `ros2 topic list` and a talker/listener pair
- [ ] Install Gazebo Harmonic and the `ros_gz` bridge packages for Jazzy
- [ ] Verify the bridge: a Gazebo sensor topic reaches ROS 2 via `ros_gz_bridge`
- [ ] Install Isaac Sim 6.x on a machine that meets NVIDIA's minimum and pass its Compatibility Checker
- [ ] Run `drishti_sim_isaac` and confirm every topic in `config/isaac_sim.json` arrives with sane timestamps — *written against the 6.1 API, never run (D21)*
- [x] Create the `colcon` workspace skeleton and `drishti_msgs` package
- [x] Add `config/` with a shared params file and `use_sim_time` set globally
- [ ] Pin versions: record exact ROS 2, Gazebo, CUDA and driver versions in STATUS.md
- [ ] Stand up `docker/` only if native dependencies conflict

**Acceptance**
- `colcon build` succeeds on a clean clone.
- The simulator publishes at least one ROS 2 topic that `ros2 topic echo` receives.
- `ros2 run tf2_tools view_frames` produces a tree with no duplicate publishers.
- Every running node reports `use_sim_time: true`.

**Do not yet:** any AI, any perception model, any training.

---

## Phase 1 — Simulated navigation · 2–4 days

**Artefact:** a virtual UGV that drives to a goal in Gazebo using simulator
odometry and Nav2.

> **Assets written, nothing run.** Everything below that could be checked
> without ROS has been; everything that needs a running system is still open.

- [x] Build or import a simple UGV (differential/skid-steer) with a stereo camera and IMU — *`drishti_description`, 4-wheel skid-steer, stereo + depth + IMU*
- [x] Publish robot description; verify the static TF chain from SPEC.md §3.1 — *tree checked offline by `tools/check_robot_description.py`; **not** verified in a running `tf2`*
- [ ] Verify `/camera/rgb/image_raw`, `/camera/depth/image_rect_raw`, `/camera/camera_info`, `/imu/data`, `/odom` — *bridged in `drishti_sim/config/bridge.yaml`; gz topic names need confirming against `gz topic -l`*
- [ ] Confirm every message carries a real sensor timestamp, not publish time
- [x] Install and configure Nav2; author `config/nav2.yaml` — *authored, not installed*
- [x] Wire Nav2 to publish `/cmd_vel_nav` (**not** `/cmd_vel`) — SPEC.md §4.3 — *enforced statically by `tools/check_wiring.py`*
- [ ] Pin `Twist` vs `TwistStamped` for the installed Nav2 version and record the choice
- [ ] Teleoperate the UGV; then send a fixed goal and reach it
- [ ] Set up RViz2 and Foxglove views; start recording rosbag2

**Acceptance**
- The UGV reaches a commanded goal repeatedly on flat terrain.
- The TF tree is clean while driving; no transform gaps or duplicate publishers.
- rosbag2 recordings replay.

**Do not yet:** SLAM, semantics, terrain cost.

---

## Phase 2 — Visual SLAM · 3–6 days

**Artefact:** GPS-free pose and a map, with drift quantified against ground truth.

> **Metrics done and tested; SLAM configured but never run.** The ATE/RPE
> implementation is pure maths with no ROS dependency, so it is genuinely
> verified (64 checks). Everything requiring RTAB-Map to actually run is open.

- [ ] Calibrate the simulated stereo pair; verify `camera_info` matches
- [x] Install `rtabmap_ros`; configure for stereo + IMU — *`config/rtabmap.yaml` + `slam.launch.py` authored, **not installed or run***
- [x] Establish `map → odom` from RTAB-Map only; remove any ground-truth pose from the graph — *`publish_tf` set on rtabmap alone; ground-truth exclusion enforced by `tools/check_wiring.py`*
- [ ] Verify loop closure on a revisited route
- [x] Log ground-truth pose from the simulator alongside every bag — *bridged GZ_TO_ROS, read-only*
- [x] Implement ATE/RPE computation in `drishti_eval` — *Umeyama alignment, ATE, RPE, drift %; 64 checks, 0 failures*
- [ ] Feed RTAB-Map pose into Nav2 and re-run the Phase 1 goal test
- [ ] Record baseline drift on Easy and Medium worlds — *both worlds authored*

**Acceptance**
- Nav2 reaches goals using RTAB-Map pose with no GNSS anywhere in the graph.
- ATE and RPE are reported automatically, not eyeballed.
- Baseline drift recorded in STATUS.md (target: < 2% of distance travelled).

**Do not yet:** semantic AI.

---

## Phase 3 — Terrain and traversability · 4–7 days

**Artefact:** a traversability costmap layer that Nav2 actually plans against.

> **Cost function done and tested (1217 checks); everything requiring a running
> map is open.** The fusion node and Nav2 layer are written but uncompiled.

- [x] Produce `/camera/points` from depth; verify the cloud in the correct frame — *bridged; frame **unverified***
- [ ] Install `elevation_mapping_cupy`; validate GPU/CUDA compatibility
- [ ] Publish `/elevation_map` with height, variance, slope, roughness, visibility — *slope/roughness/step need optional plugins enabled; the fusion node warns once per missing layer*
- [ ] Check for map "float" — pose/depth timestamp desynchronisation
- [x] Implement `drishti_traversability` fusion producing `/traversability` (SPEC.md §6.1) — *core **tested**, node uncompiled*
- [x] Implement the Nav2 costmap layer consuming it — *written, uncompiled; never lowers a cost, goes stale rather than authorising motion*
- [x] Implement the unknown-terrain rule: unobserved/low-visibility cells are expensive — *and a zero uncertainty weight is rejected at startup, so §6.2 cannot be configured away*
- [x] Author an initial weight set; make weights params, not constants — *`config/traversability.yaml`, logged at startup, guarded by `check_contract_sync.py`*
- [x] Build the Hard world (ditches, rocks, slopes, narrow corridors) — *raised platform split by a 1.6 m × 0.45 m ditch, with a flank detour so avoidance is testable*
- [ ] Verify the planner routes around a ditch rather than across it — *needs a running stack*

**Acceptance**
- The traversability layer is visible in RViz2 and demonstrably changes the plan (FR-01).
- A ditch scenario (T07) is avoided without any semantic model in the loop.
- Cost weights are in `config/`, and the current set is recorded with its result.

**Do not yet:** custom model training.

---

## Phase 4 — Perception · 5–10 days

**Artefact:** semantic hazard and terrain classification fused into the cost.

> **Taxonomy, health and obstacle distance are done and tested (198 checks).
> The model and every ROS wrapper are written but never run.**

- [x] Stand up `drishti_perception` with a pretrained lightweight YOLO detector — *wrapper written; ultralytics **not installed**, and deliberately not declared as a package dependency until its AGPL position is settled*
- [x] Publish `/perception/detections` and `/perception/health` — *node written; health goes out on a timer so silence stays a detectable failure*
- [ ] Add segmentation for the terrain tiers in SPEC.md §5.2 — keep the vocabulary small — *vocabulary defined and frozen (19 classes); no segmenter yet*
- [x] Publish `/perception/semantic_mask` with stable class ids — *ids frozen and pinned by a test; mask publication itself is V2*
- [ ] Project semantics into the elevation map as semantic layers
- [x] Wire the `semantic` and `uncertainty` terms into the cost function — *taxonomy feeds `semantic_cost`/`semantic_lethal`; layer names already in `traversability.yaml`*
- [ ] Measure perception latency against the ≤ 100 ms budget; apply TensorRT if needed — *latency is measured and published; the budget cannot be checked without hardware*
- [ ] Generate synthetic training data **only if** failure analysis demands it (Isaac Sim is the primary simulator — D21)
- [ ] Ablation: stereo depth vs Depth Anything V2 Small

**Acceptance**
- Hazards appear in the annotated stream and in the obstacle cost (FR-02).
- Mud/water scenarios (T08, T09) are handled better with semantics than without — measured, not asserted.
- Perception latency is inside budget on the target machine.

**Do not yet:** complex multi-model fusion.

---

## Phase 5 — Safety supervisor · 2–4 days

**Artefact:** a deterministic gate on `/cmd_vel` with measured stop latency.

> **Partly built early, during Phase 0 — see STATUS.md D14.** The decision core
> is pure logic with no ROS or hardware dependency, so it was the one
> substantial component fully verifiable on a machine that cannot run the
> stack. Everything still unchecked below needs a running system.

- [x] Define `drishti_msgs/SafetyState` with reason codes
- [x] Implement the C++ supervisor per SPEC.md §9, ticking on its own timer — *core done and tested; the ROS node wrapping it is **uncompiled***
- [ ] Enforce the invariant: supervisor is the sole `/cmd_vel` publisher — *coded, unproven; needs `ros2 topic info /cmd_vel` on a live graph*
- [x] Implement all seven stop/slow conditions with fixed evaluation order — *and corrected the order; SPEC.md §9.1 had a defect (D13)*
- [x] Move every threshold into one params file, logged at startup
- [x] Unit-test the decision logic **without ROS running** — *377 checks, clean under `-Werror`*
- [x] Build the Failure world: frozen camera, stale depth, SLAM loss, invalid commands — *`drishti_eval/faults.py`: 8 scenarios covering T16–T19 plus a NaN-command case; schedule logic tested, injector node **uncompiled***
- [x] Measure stop latency from fault injection to blocked command — *`drishti_eval/latency.py`, tested; refuses to measure from a stationary baseline and summarises on the **worst** case, not the mean*

**Acceptance**
- T16–T19 (camera dropout, depth dropout, SLAM loss, no valid path) all end in a safe halt. — **open**, needs a running stack
- Stop latency < 200 ms, reported from logs. — **open**; the measurement exists and is tested, the number does not
- [x] Decision logic has unit tests that run in CI without a simulator. — *377 checks on the supervisor core, 111 on the harness*

> **A gap this phase found and did not close.** `faults.py` distinguishes a
> camera that goes *silent* from one that *freezes* — keeps publishing the same
> image with a fresh timestamp. SPEC.md §9 catches silence through
> `t_camera_stale`, but nothing in the supervisor notices that frame content
> has stopped changing, so a frozen camera would read as perfectly healthy.
> T16 as written only tests the easy half. Closing this needs either a content
> hash in the perception health report or a frame-difference check; both are
> new work and neither is in TASK.md yet.

**Do not yet:** hardware.

---

## Phase 6 — Randomised testing · 5–10 days

**Artefact:** hundreds of automated missions with an auto-generated metrics report.

> **The harness is built and tested (2571 checks). Nothing has been run, so
> there are no numbers.** Everything below that produces a *number* rather than
> a *mechanism* is still open, and will stay open until a machine exists.

- [x] Implement the scenario runner in `drishti_eval` (headless, seeded, repeatable) — *generation and planning done; the executor needs ROS*
- [x] Implement domain randomisation (SPEC.md §10.3) — *sun, ambient, camera noise, depth dropout, friction; envelope asserted by test*
- [x] Implement all metrics in EVALUATION.md §2 — *ATE/RPE/drift, stop latency, outcome classification, suite roll-up*
- [ ] Automate the full scenario catalog T01–T20 — *T01–T07 and T16–T19 covered; T08–T15 and T20 need the Dynamic and Adversarial worlds*
- [ ] Run ≥ 100 randomised missions; then scale toward 1000
- [x] Generate a per-run report and a suite summary — *`report.py` and `outcome.format_summary`*
- [ ] Triage failures by category; feed fixes back into Phases 3–5
- [ ] Record the headline numbers in STATUS.md

**Acceptance**
- Collision-free completion ≥ 95% on the randomised suite. — **open**, no runs
- Goal completion ≥ 97%. — **open**, no runs
- The report regenerates from a single command, from bags plus ground truth. —
  *plan side done: `python -m drishti_eval.plan_suite --count 100 --json plan.json`
  reproduces any suite from two numbers; the bag-reading side needs ROS*

**Do not yet:** a real UGV.

---

## Phase 7 — Optimisation and hardware transfer · contingent

**Artefact:** a profiled stack, and a bring-up plan executed only if a vehicle exists.

> **The gates are built; the profiling is not.** Everything here that judges a
> measurement exists and is tested. Everything that *takes* one needs hardware.

- [ ] Profile CPU, GPU, RAM and VRAM under full load — *`budgets.py` judges the samples; nothing produces them yet*
- [ ] Reduce latency: model size, TensorRT, resolution, sensor rates
- [x] Re-run the suite after every optimisation to catch regressions — *`regression.py`: two-proportion test with an explicit power check; **underpowered is not a pass***
- [x] Containerise the validated stack — *`docker/Dockerfile`, CUDA + ROS 2 Jazzy; **never built***
- [x] Write the sensor driver shim that reproduces the SPEC.md §4 contract — *`config/hardware.yaml` + `hardware.launch.py`; the contract is machine-checked by `check_wiring.py`*
- [ ] Execute the bring-up sequence in SPEC.md §12.1 — **strictly in order** — *the nine steps are encoded in `hardware.yaml` and their order is checked*
- [x] Mandatory external hardware e-stop from bring-up step 8 onward — *`hardware.launch.py` refuses to start Nav2 without `estop_confirmed:=true`*

**Acceptance**
- Post-optimisation suite results are no worse than pre-optimisation. — *rule
  implemented and tested; note it requires **~1470 missions per side** to detect
  a 2-point regression at a 95% baseline (D20)*
- On hardware: repeated collision-free low-speed runs before any speed increase.
  — **open**, no vehicle

---

## Cross-cutting, do continuously

- [ ] Keep STATUS.md current — it is the handover document
- [ ] Update SPEC.md in the same commit as any interface change
- [ ] Screen every new dependency's licence before integrating (REFERENCES.md §3)
- [ ] Keep one ROS distribution across the whole project
- [ ] Record every experiment: parameters in, numbers out
