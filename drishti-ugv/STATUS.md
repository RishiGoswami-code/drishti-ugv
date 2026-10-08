# STATUS — DRISHTI-UGV

Living state of the project. **This is the handover document** — if someone
picks this up cold, this file plus [TASK.md](TASK.md) should tell them exactly
where things stand and what to do next.

Update it whenever a phase moves, a decision is made, or a suite is run.

---

## Snapshot

| | |
|---|---|
| **Phase** | Phases 0–7 written and statically checked; **partly exercised in CI** (see *Verified in CI*) |
| **Last updated** | 8 October 2026 |
| **Next action** | Get Nav2 lifecycle bring-up through (open `collision_monitor` configuration issue), then SLAM and the mission suite |

> **Simulator: Isaac Sim, from 8 October 2026 (D21).** The 6 September decision
> (D15, Gazebo Harmonic) was reversed on the project owner's instruction. The
> hardware conflict behind D15 is **unchanged**: the available machine, a Lenovo
> LOQ with an RTX 3050 (4–6 GB VRAM), is far below NVIDIA's published Isaac Sim
> minimum (an RTX 4080-class GPU with 16 GB VRAM and 32 GB RAM). So the Isaac Sim
> backend is written and checked offline for consistency but **has never been
> run and cannot be run on the hardware in hand**. Gazebo Harmonic stays as the
> fallback and as the only simulator CI can run.
>
> D16 stands: **`elevation_mapping_cupy` stays on the primary path**, because the
> machine has CUDA. D12 is reversed.

---

## Done

| # | Item | Date |
|---|---|---|
| 1 | Research baseline and architecture (blueprint, 29 sections + appendices) | 4 Sep 2026 |
| 2 | Project documentation set written (PRD, SPEC, SETUP, TASK, EVALUATION, REFERENCES, CONTRIBUTING) | 4 Sep 2026 |
| 3 | **Workstation hardware audited — fails the Isaac Sim floor on every axis** (Q2 closed, A2 falsified) | 5 Sep 2026 |
| 4 | `ugv_ws` colcon skeleton: `drishti_msgs` (`SafetyState`, `PerceptionHealth`) and `drishti_bringup` (shared params, `use_sim_time` global) — syntax-checked, **not built** | 5 Sep 2026 |
| 5 | **Safety supervisor decision core written and tested — 377 checks, 0 failures, clean under `-Wall -Wextra -Wpedantic -Werror`.** ROS-free by design, so it was verifiable on the blocked machine | 5 Sep 2026 |
| 6 | `tools/check_contract_sync.py` — guards `.msg` constants against the C++ enums and the header defaults against `drishti.yaml`; negative-tested | 5 Sep 2026 |
| 7 | **SPEC.md §9.1 ordering defect found and corrected** (D13), plus §4.2 topic and three §9.3 parameters added | 5 Sep 2026 |
| 8 | **Phase 1 assets written**: `drishti_description` (skid-steer UGV, stereo + depth + IMU), `drishti_sim` (Gazebo Harmonic Easy world, ros_gz bridge), `nav2.yaml` and bringup launch | 6 Sep 2026 |
| 9 | `tools/check_robot_description.py` and `tools/check_wiring.py` — frame tree and `/cmd_vel` ownership enforced offline; both negative-tested | 6 Sep 2026 |
| 10 | **Phase 2 assets**: `drishti_eval` (ATE, RPE, drift, run report) — **64 checks, 0 failures**, no ROS dependency; RTAB-Map stereo config and `slam.launch.py`; Medium world | 6 Sep 2026 |
| 11 | **Phase 3 assets**: `drishti_traversability` — SPEC.md §6.1 cost function, **1217 checks, 0 failures**, clean under `-Werror`; fusion node, Nav2 costmap layer, weight config, Hard world with a real ditch | 6 Sep 2026 |
| 12 | `tools/check_sim_assets.py` — worlds parse, required gz systems present, every model has collision geometry; negative-tested | 6 Sep 2026 |
| 13 | **Phase 4 assets**: `drishti_perception` — frozen 19-class SPEC.md §5.2 vocabulary, perception health, nearest-hazard distance. **198 checks, 0 failures**; detector wrapper and ROS node written, not run | 7 Sep 2026 |
| 14 | `check_contract_sync.py` extended to cross-package rules: SPEC.md §6.2 unknown cost, and staleness agreement between supervisor and perception; both negative-tested | 7 Sep 2026 |
| 15 | **Phase 5 closed out**: fault schedules for T16–T19 and emergency-stop latency measurement — **111 checks, 0 failures**; injector node written, not run | 7 Sep 2026 |
| 16 | **D18 closed (D19)**: frozen-camera detection — `rgb_static_for` on `/perception/health`, `t_frame_static` in SPEC.md §9.3, new `CAMERA_FROZEN` reason. Supervisor now **388 checks** | 7 Sep 2026 |
| 17 | **Phase 6 harness**: outcome classification, seeded mission generation, domain randomisation, suite roll-up — **2571 checks, 0 failures** | 7 Sep 2026 |
| 18 | **Phase 7 gates**: SPEC.md §8 budget checking (tail percentiles, not means) and optimisation regression testing with an explicit power check — **79 checks, 0 failures** | 7 Sep 2026 |
| 19 | Hardware transfer contract (`config/hardware.yaml`, `hardware.launch.py`) now **machine-checked** by `check_wiring.py`; container definition written | 7 Sep 2026 |
| 20 | **Isaac Sim backend written, not run (D21)**: `drishti_sim_isaac` builds the robot and the Easy/Medium/Hard worlds in USD from the same URDF and SDF files Gazebo uses, publishes the same 14 topics through Isaac Sim's ROS 2 bridge, and is selected with `sim:=isaac\|gazebo` on `bringup.launch.py`. `tools/check_isaac_assets.py` checks topic parity with the Gazebo bridge, optical-frame and camera orientation, drive kinematics and world parsing offline | 8 Oct 2026 |

## In progress

**Build-ahead strategy (D17).** The team chose to write the full stack offline
and defer every install to a later session on the Lenovo LOQ or an AWS GPU
instance. So phases advance by *assets written and statically checked*, and the
phase-gate acceptance criteria — which all require a running system — stay open
behind them.

| Phase | Assets | Verified offline | Run |
|---|---|---|---|
| 0 Environment | workspace, msgs, shared params | contract sync | ✗ |
| 1 Sim navigation | description, Easy world, bridge, Nav2 config, bringup | frame tree, command path | ✗ |
| 2 Visual SLAM | `drishti_eval` metrics, RTAB-Map config, Medium world | **64 checks** on the metrics | ✗ |
| 3 Traversability | cost function, fusion node, Nav2 layer, Hard world | **1217 checks** on the cost function | ✗ |
| 4 Perception | taxonomy, health, obstacle distance, detector, node | **198 checks** on the pure logic | ✗ |
| 5 Safety | supervisor core, fault schedules, latency measurement | **388 + 111 checks** | nodes ✗ |
| 6 Suite | outcome classification, seeded scenarios, roll-up | **2571 checks** | ✗ |
| 7 Gates | budgets, regression power, hardware contract, container | **79 checks** | ✗ |

`python tools/run_checks.py` runs the ten Python suites; the two C++ suites
need a compiler and run separately. Currently **10/10, 388/388 and 1217/1217** —
roughly 5,000 assertions in total, none of which need ROS, a GPU or a
simulator.

**Only part of it has been run against a real system** (see *Verified in CI*). The
harness can say what a number *means*; the mission-level numbers themselves have
not been produced yet.

The pattern that keeps working: put the judgement in a core with no ROS
dependency, and the plumbing in a thin wrapper. The cores are the parts that
would be dangerous to get wrong, and they are the parts that are actually
tested.

## Verified in CI

The workspace is built and exercised on a hosted Ubuntu 24.04 runner (ROS 2 Jazzy,
Gazebo Harmonic, no GPU). What that has shown, from the uploaded run logs and not
from the pass/fail mark alone:

- `colcon build` succeeds for all 8 packages.
- The offline suites pass.
- The supervisor, launched with nothing feeding it, publishes `ACTION_STOP`,
  reason `LOCALIZATION_LOST`, zero velocity, and `/cmd_vel` has exactly one
  publisher (`safety_supervisor`).
- The `ros_gz_bridge` starts and creates its bridges from `bridge.yaml` (the log
  shows, for example, `/cmd_vel`, `/joint_states` and `/ground_truth/pose`), and
  Gazebo starts headless once `sim.launch.py` actually applies its `headless`
  argument (it previously declared the argument and never used it).

Still open: Nav2's lifecycle bring-up is aborted by `collision_monitor`, which
nav2_bringup includes unconditionally. The parameter loader rejects empty lists
for its array parameters, so `nav2.yaml` needs real, non-empty, inert
definitions that do not make the node a second publisher on `/cmd_vel`.

A CI mark only means the job did not time out: several steps deliberately do not
fail the job, so the log content is what counts.

## Next actions

Every phase that can be built without hardware now has been. What remains is
work that needs a running system:

- the Dynamic and Adversarial worlds (T08–T15, T20)
- a segmenter, and semantic layers written into the elevation map
- executing the suite and recording actual numbers
- Phase 7 optimisation, which is meaningless before a profile exists

The next real step is a machine: Ubuntu 24.04, ROS 2 Jazzy, then `colcon build`,
with Gazebo Harmonic for the simulator that runs anywhere and Isaac Sim on a
machine that meets NVIDIA's minimum (D21).

When a machine is available, in this order:

1. Ubuntu 24.04 + NVIDIA driver; record VRAM, RAM and free disk.
2. ROS 2 Jazzy, then a simulator: Gazebo Harmonic (any machine, and CI) and, on a
   qualifying GPU, Isaac Sim (D21).
3. `colcon build --symlink-install`. **First real build of anything here.**
   Expect to fix rclcpp API details in `safety_supervisor_node.cpp`.
4. Confirm the simulator's topic names: `gz topic -l` against
   `drishti_sim/config/bridge.yaml` for Gazebo, and `ros2 topic list` against
   `drishti_sim_isaac/config/isaac_sim.json` for Isaac Sim. A wrong name looks
   exactly like a dead sensor.
5. `ros2 topic info /cmd_vel --verbose` — exactly one publisher,
   `safety_supervisor`. This is the invariant the whole safety story rests on.
6. Fill *Pinned versions* below.

---

## Blockers

| # | Blocker | Blocks | Owner | Since |
|---|---|---|---|---|
| ~~B2~~ | ~~Workstation GPU capability unconfirmed~~ — **closed 5 Sep 2026**, audit below. Superseded by B3. | — | — | — |
| ~~B3~~ | ~~No machine available that can run the specified stack~~ — **closed 6 Sep 2026.** A Lenovo LOQ with RTX 3050 is available; simulator switched to Gazebo Harmonic (D15), CUDA path restored (D16) | — | — | — |

**No open blockers on Phase 0.**

### B3 — the hardware audit (superseded, kept for the record)

Measured on the **original** development machine, 5 September 2026. This
machine is no longer the target; the Lenovo LOQ replaces it. Kept so the
constraint is not re-litigated from memory if someone picks the laptop up again.

| Requirement (SETUP.md §1 / Isaac Sim 6.0.1 minimum) | Required | Actual | |
|---|---|---|---|
| GPU | NVIDIA RTX, RT cores required | **Intel Iris Xe (integrated)** | ✗ |
| VRAM | 16 GB | ~3.8 GB shared | ✗ |
| RAM | 32 GB | **7.7 GB** | ✗ |
| Free disk | 50 GB | 8.7 GB on C:, 56 GB on D: | ✗ on C: |
| CPU | 4+ cores | i5-1235U, 10c/12t | ✓ |
| OS | Ubuntu 24.04 | Windows 11, **WSL not installed** | ✗ |

Three consequences, in order of severity:

1. **Isaac Sim is impossible here, not merely slow.** It requires RT cores.
   Intel Iris Xe has none. This is a hard capability gap, not a performance
   one.
2. **`elevation_mapping_cupy` is equally impossible.** CuPy is CUDA; there is
   no NVIDIA GPU. The GPU-accelerated terrain layer — the component SPEC.md §6
   is built around — cannot run on this machine under any settings.
3. **RAM is the quiet constraint.** At 7.7 GB total, even the Gazebo fallback
   with Nav2, RTAB-Map and RViz2 is marginal, and WSL2 would take a share of
   it. Disk is fixable (install the distro to D:); RAM is not.

The Gazebo Harmonic fallback already named in SPEC.md §2 covers consequence 1.
It does **not** cover 2 — that needs a CPU traversability layer built on
`grid_map` instead, which is a real change to SPEC.md §6 and new work not
currently in TASK.md.

### The development machine, from 6 September 2026

| | Lenovo LOQ | Isaac Sim minimum | Gazebo Harmonic |
|---|---|---|---|
| GPU | **RTX 3050 laptop** | RTX 4080 | — |
| RT cores | **yes** | required | not required |
| CUDA | **yes** | required | not required |
| VRAM | 4–6 GB *(unconfirmed)* | 16 GB | comfortable |
| RAM | *unconfirmed* | 32 GB | comfortable |

Isaac Sim requirements re-verified against NVIDIA's live documentation on
6 September 2026, not recalled: minimum RTX 4080, 16 GB VRAM, 32 GB RAM, 50 GB
SSD, with the note that workloads "leveraging a large number of sensors are
particularly affected" below the minimum specification.

What changed, and what did not:

- **Changed:** the machine now has RT cores and CUDA. Consequence 2 above
  disappears — `elevation_mapping_cupy` is viable again (D16).
- **Not changed:** VRAM and RAM remain far below the Isaac Sim floor, so the
  simulator is Gazebo Harmonic (D15). The RTX 3050 would *launch* Isaac Sim,
  which is the trap: it fails on scene complexity and sensor count rather than
  at startup, so the cost is discovered late.
- **Unaffected either way:** everything above the simulator. That is what the
  SPEC.md §4 interface contract exists to guarantee.

The Apple-silicon laptop cannot run Isaac Sim under any configuration and has
no CUDA. It is usable for ROS 2 development, the ROS-free supervisor tests, and
documentation — not for simulation or training.

---

## Decision log

Decisions with consequences. Append; do not rewrite history.

| # | Date | Decision | Rationale | Reversible? |
|---|---|---|---|---|
| D1 | 4 Sep 2026 | Hybrid stack over building from scratch | The research gap is integration under a vision-first outdoor constraint, not new SLAM or planning algorithms | Yes, but expensive |
| D2 | 4 Sep 2026 | RTAB-Map as the single runtime SLAM system | ROS 2 support with stereo/RGB-D and Nav2 integration examples; ORB-SLAM3 is GPLv3 and carries an older integration burden | Yes — ORB-SLAM3 remains an offline benchmark |
| D3 | 4 Sep 2026 | Nav2 + MPPI for planning and control | Mature framework; MPPI evaluates future trajectories against the local environment rather than following a geometric line | Yes — RPP/DWB fallback |
| D4 | 4 Sep 2026 | `elevation_mapping_cupy` for terrain | GPU-accelerated, ROS 2 Jazzy releases, semantic layers, MIT | Yes — custom grid map fallback |
| D5 | 4 Sep 2026 | Safety supervisor sits outside the AI and owns `/cmd_vel` | The stop decision must never depend on a model's confidence | **No** — this is a core invariant |
| D6 | 4 Sep 2026 | Simulation-first with phase gates | Prevents a sophisticated model from masking a broken TF, odometry or navigation foundation | No |
| D7 | 4 Sep 2026 | Stereo/RGB-D preferred over monocular | Metric depth greatly simplifies elevation mapping and collision reasoning; Depth Anything V2 Small held as fallback | Yes — pending answer to Q4 |
| D8 | 4 Sep 2026 | Targets stated as mission-level metrics, never "100% accuracy" | Defensible, measurable, and a stronger scientific claim | No |
| D9 | 4 Sep 2026 | Framed as a **software module** | Hardware is a transfer path, not a requirement; the earlier working assumption that a physical vehicle was required was wrong | No |
| D10 | 4 Sep 2026 | *(withdrawn — concerned presentation tooling, not part of this repository)* | — | — |
| D11 | 5 Sep 2026 | **Phase 0 split: the machine-independent half was built, the rest held.** `ugv_ws` skeleton, `drishti_msgs` and the shared params file were written and syntax-checked without a ROS install | The interface contract (SPEC.md §4, §9) is fixed and does not depend on which simulator or machine wins. Writing it now keeps Phase 0 moving while B3 is open, and it transfers unchanged to whatever compute path is chosen | Yes — cheap to revise |
| D12 | 5 Sep 2026 | **`elevation_mapping_cupy` is not viable on the current machine** and D4's "custom grid map fallback" is now the live path unless a CUDA GPU is obtained | CuPy requires CUDA; the audited machine has no NVIDIA GPU. This is a capability gap, not a tuning problem | Yes — reverts if an RTX machine or cloud GPU is secured |
| D13 | 5 Sep 2026 | **SPEC.md §9.1 evaluation order corrected: all STOP conditions now precede the SLOW branch.** `command_not_finite` hoisted above the forwarding paths; `t_pose_stale`, `t_plan_stale`, `t_cmd_stale` added to §9.3; `/perception/nearest_obstacle` added to §4.2 | The original "first match wins" order forwarded a command whenever confidence was low — even with no valid path — so low confidence masked a harder fault. Reordering can only turn SLOW into STOP, never the reverse, so it cannot weaken the envelope. The extra thresholds close the same hole for the pose, plan and command streams | **No** — safety ordering |
| D14 | 5 Sep 2026 | **Safety supervisor built during Phase 0, ahead of its Phase 5 slot** | A deliberate exception to the phase gates (D6), taken because the core is pure logic with no ROS or hardware dependency and was the one substantial component fully verifiable on a machine that cannot run the stack. The gate's purpose — stopping a model from masking a broken foundation — is not weakened: this component has no model in it. The ROS node it wraps remains uncompiled and unproven | Yes |
| D15 | 6 Sep 2026 | **Gazebo Harmonic becomes the primary simulator; Isaac Sim held as contingent** | The development machine is a Lenovo LOQ with an RTX 3050 laptop GPU, 4–6 GB VRAM. NVIDIA's live requirements page (re-checked 6 Sep 2026) sets the Isaac Sim minimum at RTX 4080 / 16 GB VRAM / 32 GB RAM and warns that sensor-heavy workloads suffer below it — a stereo pair, depth and IMU over randomised terrain is precisely that. SETUP.md §1.2: decide, record, move | Yes — reverts if a qualifying GPU is obtained |
| D16 | 6 Sep 2026 | **`elevation_mapping_cupy` restored as the terrain layer; D12 reversed** | The RTX 3050 provides CUDA, which the previously audited Intel Iris Xe did not. The CPU `grid_map` fallback is no longer forced — it returns to being a fallback | Yes |
| D17 | 6 Sep 2026 | **Build the whole stack offline first; defer every install** | The team has no machine set up yet and will rent AWS GPU later. Writing assets ahead of the toolchain is only safe if the claim "this works" is never made on their behalf, so each phase records what was statically checked and what was not, and every uncompiled file carries an UNVERIFIED banner. The risk accepted is a batch of API-level fixes at first build, concentrated in the ROS wrappers | Yes |
| D18 | 7 Sep 2026 | **A frozen camera is not covered by SPEC.md §9** | `t_camera_stale` catches a camera that goes silent, but not one that keeps republishing the same image with a fresh timestamp. Nothing in the supervisor notices that frame content has stopped changing, so a frozen camera reads as healthy. Recorded as a known gap rather than designed around; `faults.py` includes the scenario so it fails visibly when it is run | **CLOSED 7 Sep 2026 by D19** |
| D19 | 7 Sep 2026 | **Frozen-camera detection added: `rgb_static_for` on `/perception/health`, `t_frame_static` in SPEC.md §9.3, and a new `CAMERA_FROZEN` reason code** | Closes D18. Liveness and freshness are different questions and `t_camera_stale` only answers the first. Perception fingerprints each frame; the supervisor stops when content stops changing, independently of age. Absence of the signal is not treated as a freeze, so an older perception build stays driveable | **No** — safety condition |
| D20 | 7 Sep 2026 | **The randomised suite needs ~1470 missions per side to detect a 2-point regression, not the ~1000 TASK.md Phase 6 plans for** | Fell out of implementing the Phase 7 regression gate. At a 95% baseline the minimum detectable drop is 7.7 points at n=100, 5.4 at n=200, 2.4 at n=1000 and 2.0 at n=1470. Below that, "no significant regression" is not evidence of no regression, so the gate reports UNDERPOWERED and refuses to pass. Phase 6's target should rise, or the effect we agree to care about should | Open — a planning decision, not a code change |
| D21 | 8 Oct 2026 | **Isaac Sim becomes the primary simulator again; D15 is reversed. Gazebo Harmonic is retained as the fallback and as the CI simulator** | Decided by the project owner. Recorded against a known conflict: NVIDIA's published minimum for Isaac Sim (an RTX 4080-class GPU with RT cores and 16 GB VRAM, 32 GB RAM, 50 GB SSD; unchanged since the 6 Sep check and present in the 5.1 documentation too) is not met by the machine in hand (RTX 3050 laptop, 4–6 GB VRAM), and hosted CI runners have no GPU. So `drishti_sim_isaac` is written against the Isaac Sim 6.1 API and checked offline, but has never been run and nothing in CI exercises it. The SPEC.md §4 interface contract is unchanged; the Gazebo path stays the only one verified end to end; the first run of the Isaac backend needs a qualifying GPU (CLOUD_SETUP.md §10). Numbers measured in one simulator are not evidence for the other | Yes — the simulator is behind the §4 contract |

---

## Assumptions in force

These are working assumptions, not verified facts. Each one is a place where
being wrong costs rework — revisit them when the corresponding question closes.

| # | Assumption | Falsified by | Related |
|---|---|---|---|
| A1 | A stereo or RGB-D camera is acceptable; monocular-only is not mandated at evaluation | Requirements clarification | Q4 |
| ~~A2~~ | ~~The workstation GPU meets the Isaac Sim floor~~ — **FALSIFIED 5 Sep 2026** by the hardware audit. No NVIDIA GPU present. See B3. | *(falsified)* | Q2, B3 |
| A3 | No physical UGV within the evaluation window; Phase 7 is contingent | Stakeholders | Q3 |
| A4 | The semantic taxonomy in SPEC.md §5.2 is sufficient; no domain-specific classes required | Domain-expert review | Q5 |
| A5 | Licence positions in REFERENCES.md still hold | Re-checking upstream | — |

---

## Pinned versions

Fill during Phase 0 and update on every deliberate upgrade. Empty rows are the
honest state, not an oversight.

**Still entirely empty as of 6 Sep 2026** — nothing is installed, because of
B3. These rows get filled at the first successful `colcon build`, not before.

### Development machine, as audited 5 Sep 2026

| | |
|---|---|
| CPU | 12th Gen Intel Core i5-1235U — 10 cores / 12 threads |
| RAM | 7.7 GB |
| GPU | Intel Iris Xe (integrated), ~3.8 GB shared — **no NVIDIA GPU** |
| GPU driver | Intel 32.0.101.5542 |
| OS | Windows 11 Home Single Language, build 26200 |
| WSL | not installed; hypervisor present, so WSL2 is available |
| Disk free | 8.7 GB on C:, 56 GB on D: |

Recorded so the constraint is not re-litigated from memory. See **B3**.

### Toolchain

| Component | Version | Pinned on |
|---|---|---|
| Ubuntu | | |
| NVIDIA driver | | |
| ROS 2 | | |
| Gazebo Harmonic | | |
| Isaac Sim | | |
| Isaac Sim ROS 2 bridge extension (`isaacsim.ros2.bridge`) | | |
| CUDA | | |
| CuPy | | |
| Nav2 | | |
| RTAB-Map | | |
| `elevation_mapping_cupy` | | |
| Nav2 velocity message type (`Twist` / `TwistStamped`) | | |

---

## Results log

One row per mission-suite run. A row without a seed and a parameter set is an
anecdote, not a result (EVALUATION.md §7.2).

| Date | Commit | Suite | Runs | Collision-free | Goal completion | Drift | Notes |
|---|---|---|---|---|---|---|---|
| — | — | — | — | — | — | — | No runs yet |

---
