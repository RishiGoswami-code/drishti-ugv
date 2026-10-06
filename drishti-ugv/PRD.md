# PRD — DRISHTI-UGV

Requirements and success criteria for DRISHTI-UGV.

Status: **baseline agreed; implementation in progress (see STATUS.md)**
Owner: The Vikings
Last updated: 4 September 2026

---

## 1. Problem

Outdoor unmanned ground vehicles (UGVs) operate on unpredictable terrain, under changing light and
with unreliable GPS. Applications such as search-and-rescue, agriculture and delivery therefore
require autonomy that rests on onboard computer vision.

The task is an autonomous navigation system for a UGV operating in a
**GPS-denied outdoor environment using camera feeds as the primary sensor**,
solving three challenges:

1. **Path detection** — real-time identification of safe, traversable paths
   versus hazards (rocks, ditches, trees).
2. **Visual localisation** — estimating position and orientation without GPS.
3. **Collision avoidance** — dynamically routing around sudden obstacles
   toward a destination.

The expected solution is a **functional software module** comprising a
lightweight perception model, a visual SLAM/odometry pipeline, and a path
planner that translates visual data into motion commands. Success criterion:
collision-free navigation from Point A to Point B across outdoor scenarios.

### 1.1 What makes this hard

- **Free space is not safe space.** A ditch, water surface or steep slope is
  geometrically open and physically lethal. Binary occupancy is the wrong
  representation.
- **Vision fails quietly.** Glare, shadow, low light and lens contamination
  degrade perception without raising an error.
- **No ground truth at runtime.** Without GPS, a drifting pose estimate has
  nothing to correct it except the visual map itself.
- **Integration, not algorithms, is the usual failure.** Frame, timestamp and
  calibration errors cause more robotics failures than model accuracy does.

---

## 2. Users and stakeholders

| Stakeholder | Need |
|---|---|
| Evaluators / reviewers | A working software module, demonstrably collision-free, with evidence |
| Readers / reviewers | A clear, honest, measurable result, reproducible from the repository |
| Field operator (downstream) | Predictable behaviour, and a vehicle that stops when unsure |
| Our own team | A modular stack where each subsystem can be tested alone |

---

## 3. Goals

- **G1** Navigate a simulated UGV from an arbitrary Point A to Point B across
  unstructured outdoor terrain with no GPS, without collision.
- **G2** Represent terrain as a *cost*, fusing geometry and semantics, rather
  than as free/occupied.
- **G3** Make the vehicle behave conservatively under uncertainty — slow down
  or stop rather than guess.
- **G4** Prove performance with mission-level numbers over a randomised test
  suite, not with a single accuracy figure.
- **G5** Keep the ROS 2 interface identical in simulation and on hardware, so
  the transfer is a driver swap rather than a rewrite.

## 4. Non-goals

Explicitly out of scope for this work:

- Inventing new SLAM, planning or control algorithms.
- Building or procuring a physical UGV. Hardware transfer is designed for, not
  performed (Phase 7 is contingent).
- Multi-robot coordination, mapping at campus scale, or long-duration autonomy.
- A general-purpose terrain segmentation model. The class vocabulary stays
  small and navigation-relevant.
- Running two SLAM systems in production. ORB-SLAM3 is an offline benchmark
  only.
- Any claim of "100% accuracy". See §7.

---

## 5. Functional requirements

| ID | Requirement | Acceptance evidence |
|---|---|---|
| FR-01 | Detect and estimate traversable terrain | Traversability layer visible in RViz2 and demonstrably consumed by the planner |
| FR-02 | Detect hazardous objects and regions | Annotated camera stream plus corresponding obstacle cost in the local costmap |
| FR-03 | Estimate pose without GPS | RTAB-Map trajectory compared against Isaac Sim ground truth (ATE/RPE) |
| FR-04 | Plan and execute Point A → Point B | Nav2 reaches the goal repeatedly across randomised starts and goals |
| FR-05 | Avoid newly appearing obstacles | A dynamic obstacle provokes a local replan or a stop, logged with timestamps |
| FR-06 | Handle uncertainty explicitly | Low perception confidence or unknown terrain produces reduced speed or a stop |
| FR-07 | Recover from temporary blockage | Planner/controller recovery succeeds, or the vehicle halts safely |
| FR-08 | Run in real time | Perception, mapping and control meet the budgets in SPEC.md §8 |

## 6. Non-functional requirements

| ID | Requirement |
|---|---|
| NFR-01 | Modular ROS 2 nodes with explicit topic, message-type and TF contracts (SPEC.md §4) |
| NFR-02 | Reproducible environment — pinned versions, containerised where dependencies conflict |
| NFR-03 | All sensor, pose, planner and safety events logged to rosbag2 with ground truth alongside |
| NFR-04 | Emergency-stop behaviour is deterministic and independent of any model's confidence |
| NFR-05 | Sensor abstraction is hardware-agnostic: the simulator and a real driver publish the same topics and frames |
| NFR-06 | Every third-party component's licence is screened before integration (REFERENCES.md §3) |
| NFR-07 | Each subsystem is independently testable and independently launchable |

---

## 7. Success criteria

We state the engineering target as **high-probability, collision-free mission
completion with explicit uncertainty handling and a fail-safe stop policy** —
not perfect perception. No stack can honestly guarantee 100% real-world
accuracy under changing light, terrain ambiguity, sensor contamination and
localisation failure.

### 7.1 Acceptance gates

| Metric | Prototype target | Stretch target |
|---|---|---|
| Collision-free completion | ≥ 95% of randomised missions | ≥ 99% on the defined evaluation suite |
| Goal completion | ≥ 97% | ≥ 99% |
| Emergency-stop response | < 200 ms (software path) | < 100 ms where hardware permits |
| Localisation drift | < 2% of distance travelled | < 1–2% after tuning |
| Perception latency | ≤ 100 ms | ≤ 60 ms preferred |
| Control loop rate | ≥ 20 Hz | ≥ 30 Hz preferred |
| Planner update rate | ≥ 5 Hz | ≥ 10 Hz preferred |

These are **our project targets**, not specifications of the upstream software.
Definitions and measurement method: [EVALUATION.md](EVALUATION.md).

### 7.2 Why mission-level metrics

A navigation system can post an excellent object-detection mAP and still drive
into a ditch. Detector accuracy is a diagnostic; mission success is the score.

---

## 8. Deliverables

| # | Deliverable | Due |
|---|---|---|
| D1 | Written description of the architecture and evaluation protocol | Ongoing |
| D2 | Reproducible ROS 2 workspace with pinned dependencies | Phase 0 |
| D3 | Simulated UGV in Isaac Sim publishing the agreed ROS 2 contract | Phase 1 |
| D4 | GPS-free localisation with quantified drift | Phase 2 |
| D5 | Traversability costmap layer consumed by Nav2 | Phase 3 |
| D6 | Perception module with the agreed semantic taxonomy | Phase 4 |
| D7 | Deterministic safety supervisor with measured stop latency | Phase 5 |
| D8 | Randomised mission suite + automated metrics report | Phase 6 |
| D9 | Demonstration script and recorded runs | Phase 6 |
| D10 | Hardware bring-up plan (executed only if a vehicle is available) | Phase 7 |

Phase definitions and task-level acceptance criteria: [TASK.md](TASK.md).

---

## 9. Constraints

- **Vision-first.** Cameras are the primary sensor. LiDAR is not part of the
  design. A stereo or RGB-D camera is strongly preferred over monocular because
  metric depth simplifies elevation mapping and collision reasoning; if
  monocular input is mandated, Depth Anything V2 Small is the fallback depth
  source and must be evaluated, not assumed.
- **GPS-denied.** No GNSS input anywhere in the runtime graph, including for
  initialisation.
- **Single GPU workstation.** The full loop must run on one RTX-class machine
  (SETUP.md §1).
- **Licence-clean.** No GPLv3 code in the shipped build.
- **Category is Software.** The deliverable is a software module. Hardware is a
  transfer path, not a requirement.

---

## 10. Risks

Ranked by expected cost. Mitigations are specified in SPEC.md §11.

| # | Risk | Mitigation |
|---|---|---|
| R1 | TF / timestamp / calibration errors silently corrupt everything downstream | Validate the TF tree and clock sync in Phase 0, before any AI work |
| R2 | A ditch or water surface is classified as drivable | Fuse slope and height-variance gates with semantics; unknown is expensive |
| R3 | SLAM drift on featureless terrain | Stereo + IMU, speed limits, re-localisation, and a stop on lost pose |
| R4 | Isaac Sim GPU/VRAM demand exceeds available hardware | Gazebo Harmonic fallback; reduce sensor count and scene complexity |
| R5 | Simulation-to-reality gap in lighting and texture | Domain randomisation from Phase 1; treat sim numbers as relative, not absolute |
| R6 | Licence terms block redistribution | Screen before integrating; ORB-SLAM3 excluded from the build |
| R7 | Scope creep into model training before the loop works | Phase gates: Phase N+1 does not start until Phase N ships a runnable artefact |

---

## 11. Open questions

| # | Question | Blocks | Owner |
|---|---|---|---|
| Q2 | Is the available workstation GPU ≥ RTX 4080-class / 16 GB VRAM? | Isaac Sim vs Gazebo decision (Phase 0) | Team |
| Q3 | Will a physical UGV be available within the evaluation window? | Whether Phase 7 is scheduled or contingent | Stakeholders |
| Q4 | Is monocular-only input a hard constraint at evaluation? | Depth strategy; stereo is assumed today | Stakeholders |
| Q5 | Are domain-specific terrain classes or hazard types required? | Semantic taxonomy in SPEC.md §5.2 | Domain experts |

Q2 is the only question that blocks Phase 0. The rest can proceed under the
assumptions recorded in [STATUS.md](STATUS.md).
