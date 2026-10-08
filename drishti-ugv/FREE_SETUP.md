# FREE_SETUP — what can be run without paid compute

For when [CLOUD_SETUP.md](CLOUD_SETUP.md)'s rented-GPU path is not an option. A
cloud "free tier" still needs a card on file and can bill on a mistake (wrong
instance type, forgetting to terminate, data-transfer overage), so this document
does not use it. Everything below runs on platforms with a hard free quota and no
payment method attached.

> Treat the commands here as a starting point, not a guarantee. What has actually
> been exercised is listed in [STATUS.md](STATUS.md) under *Verified in CI*;
> anything not listed there has not been run.

> **Isaac Sim does not run on this path.** It needs an RTX-class GPU that meets
> NVIDIA's minimum; hosted runners have none and a notebook T4 is below it. This
> guide uses the Gazebo backend throughout, selected with `sim:=gazebo`. See
> STATUS.md D21.

---

## 1. Only one part of the stack needs a GPU

`elevation_mapping_cupy` (Phase 3) needs CUDA, because CuPy is CUDA. Nav2,
RTAB-Map (CPU/ORB features), the safety supervisor and CPU perception all run
without one — slower per frame, which does not matter for a short mission.

| Need | Platform | Cost | Billing risk |
|---|---|---|---|
| Everything except Phase 3 | GitHub Codespaces / Actions | Free quota | None — no card needed for the free quota |
| The CUDA-specific check (Phase 3) | Kaggle Notebooks, T4, via RoboStack | Free, 30 GPU-hrs/week | None — no card at all; it stops at quota |

## 2. Why GitHub

- A **public** repository gets unlimited free Actions minutes. A **private** one
  gets 2,000 free minutes a month on the same no-card basis; it stops running when
  exhausted and does not bill unless a payment method has been added **and** the
  spending limit raised above $0. Check that none is on file for a hard guarantee.
- The `ubuntu-24.04` hosted runner (and the Codespaces devcontainer, pinned to the
  same image) is the one Linux version ROS 2 Jazzy ships binaries for. Jazzy does
  **not** install from apt on Ubuntu 22.04, which is why notebook platforms whose
  base image is 22.04 are unsuitable for the apt route.
- Codespaces gives an interactive shell in that environment for debugging; once
  something builds, the Actions workflow repeats it on every push and saves the
  logs as downloadable artifacts.

Files in the repository for this:

```
.devcontainer/devcontainer.json          Codespaces: Ubuntu 24.04, no GPU
.devcontainer/setup.sh                   installs ROS 2 Jazzy, Gazebo Harmonic bridge,
                                         Nav2, RTAB-Map (no CUDA / CuPy)
.github/workflows/free-tier-verify.yml   the same steps, as CI
```

A passing mark on that workflow only means the job did not time out: several steps
deliberately do not fail the job, so read the uploaded logs.

## 3. Phase 1 needs a stub

`bringup.launch.py` does not wire in Phase 4 perception yet (its own comment says
so), and the supervisor's rule is that `/perception/health` never arriving is
itself a stop condition ([CONTRIBUTING.md](CONTRIBUTING.md), rule 2). Without
something publishing on that topic, Nav2 can plan but the supervisor holds
`/cmd_vel` at zero indefinitely.

`ugv_ws/tools/phase1_stub_perception_health.py` is a test-only node that publishes
a constantly-healthy `PerceptionHealth` so Phase 1 can be exercised before Phase 4
exists. Remove it the day real perception is wired into the launch file, and do not
record a Phase 5 or Phase 6 result while it is running.

## 4. What can be exercised

### 4.1 The supervisor, standalone

`safety.launch.py` is designed to run with nothing feeding it: no simulator, no
Nav2, no perception. It must publish `ACTION_STOP` with zero velocity from the
first tick (SPEC.md §9.4.2).

```bash
ros2 launch drishti_bringup safety.launch.py
ros2 topic echo /safety/state
ros2 topic info /cmd_vel --verbose      # exactly one publisher: safety_supervisor
```

### 4.2 Offline checks

```bash
cd drishti-ugv/ugv_ws && python tools/run_checks.py
```

### 4.3 Nav2 bring-up (needs the stub)

```bash
python3 ugv_ws/tools/phase1_stub_perception_health.py &
ros2 launch drishti_bringup bringup.launch.py sim:=gazebo world:=easy.sdf headless:=true
```

Known open issue: Nav2's lifecycle manager aborts because `collision_monitor`
(included unconditionally by nav2_bringup) has no usable configuration in
`nav2.yaml`. Empty lists are rejected by its parameter loader, so the fix is a
non-empty, inert definition. Do **not** copy Nav2's shipped example verbatim: it
sets `cmd_vel_out_topic: "cmd_vel"`, which would make the node a second publisher
on `/cmd_vel` and void the invariant in 4.1.

### 4.4 Not yet exercised

- **Frozen-camera fault injection** (`fault_injector`, scenario
  `T16_camera_freeze`). The injector expects the bridge to publish under a `raw_`
  prefix so it can sit between sensor and stack; that wiring is not in
  `safety.launch.py`. The scenario is a ROS parameter, not a flag:
  `ros2 run drishti_eval fault_injector --ros-args -p scenario:=T16_camera_freeze`.
- **SLAM, terrain/CuPy and the mission suite** (Phases 2, 3, 6): they need a GPU
  (Phase 3) or hours of runtime (Phase 6).

## 5. The GPU-only phase (Kaggle + RoboStack)

Kaggle's notebook image is not Ubuntu 24.04, so `apt-get install ros-jazzy-desktop`
does not work there. RoboStack (`robostack.github.io`) ships ROS 2 as conda-forge
packages and installs independently of the host OS version. RTAB-Map is not in
RoboStack's main Jazzy channel (a community port exists), and this project has not
tested RoboStack against the full combination — scope it narrowly: show that
`elevation_mapping_cupy` initialises and produces a cost grid on the T4, and no more.

## 6. Step by step

1. Open a Codespace on the branch (**Code → Codespaces → Create codespace**).
   Confirm no payment method is attached, or that the spending limit is $0.
2. Let `postCreateCommand` run (`.devcontainer/setup.sh`); apt failures at this
   stage are package-name problems.
3. `cd drishti-ugv/ugv_ws && rosdep install --from-paths src --ignore-src -y &&
   colcon build --symlink-install`.
4. Run 4.1, then 4.2, then 4.3.
5. Push; let the workflow reproduce it and read the uploaded logs.
6. Record versions and what actually passed in STATUS.md (Pinned versions and
   Results log). A run that is not recorded did not happen.
