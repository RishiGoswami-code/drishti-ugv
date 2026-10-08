# CLOUD_SETUP — DRISHTI-UGV

How to stand up a cloud GPU machine and run the stack end to end, phase by
phase, using the assets already written offline (STATUS.md D17).

> **!! UNVERIFIED !!** Nothing in this document has been executed against a
> real cloud instance. It follows directly from [SETUP.md](SETUP.md) (the
> requirements baseline), [STATUS.md](STATUS.md) (the decisions already taken)
> and `docker/Dockerfile` (written, also never built — STATUS.md D17). Treat
> every command here as the documented intent, not a proven result, and update
> this file with what actually happened the first time it runs.
>
> As of **7 September 2026**. Re-check instance pricing and AMI names before
> spending money — both drift.

---

## 1. Why this is a cheaper machine than SETUP.md §1 implies

> **Update, 8 October 2026 (D21).** This section and §2 size the machine for the
> **Gazebo** backend. Isaac Sim is now the primary simulator and needs a larger
> one: see §10.

SETUP.md §1 is written against the **Isaac Sim** floor (RTX 4080-class, RT
cores required, A100/H100 explicitly unsupported). That floor does not apply
here: [D15](STATUS.md) already settled the simulator on **Gazebo Harmonic**,
which has no RT-core requirement at all.

The only reason this machine needs a GPU is:

- **CUDA, for `elevation_mapping_cupy`** (D16) — the terrain layer's GPU path.
- Optional local YOLO inference / light fine-tuning for perception.

Neither needs a datacentre-class card. A **T4-class GPU is enough**, which is
also one of the cheapest GPU tiers cloud providers sell — the opposite of what
SETUP.md §1's table implies if read in isolation.

---

## 2. Instance choice

| | Recommendation | Why |
|---|---|---|
| Instance | **AWS `g4dn.xlarge`** (1× T4, 16 GB VRAM, 4 vCPU, 16 GB RAM) | Comfortably above the RTX 3050's 4–6 GB the team already validated against (STATUS.md, *The development machine*) |
| Scale-up option | `g5.xlarge` (1× A10G, 24 GB VRAM) | Headroom for Phase 6 running many mission-suite scenarios; use if `g4dn` chokes on suite throughput, not pre-emptively |
| AMI | **AWS Deep Learning Base AMI, Ubuntu 24.04** | NVIDIA driver and CUDA preinstalled — skips the step that most often burns a day (SETUP.md §4's dependency-hell warning applies here first) |
| Storage | 100 GB gp3 minimum | SETUP.md §1's 50 GB Isaac Sim figure doesn't apply; budget for ROS 2 desktop + Gazebo + rosbag2 recordings, which add up fast |
| Region | wherever the team is, for latency on the Foxglove link (§4) | — |

Confirm on launch that the chosen AMI's driver version is compatible with the
CUDA 12.4 base the Dockerfile pins — if not, either update the Dockerfile's
`FROM` line or pick an AMI with a matching driver. This is exactly the
CUDA/CuPy pairing STATUS.md flags as the most likely first-build failure.

**Cost control.** GPU instances bill while running, not while used. Stop (not
just disconnect from) the instance between sessions, and terminate it once a
phase's numbers are recorded in STATUS.md — the machine is disposable, the
repository and STATUS.md's Results Log are not.

---

## 3. Seeing the run without a display

The cloud box has no monitor, and Gazebo/RViz assume one. Do not fight X11
forwarding over SSH — it is slow and it is not what any teammate will actually
use to watch a run. In order of how much setup they cost:

1. **Headless first.** `gz sim -s` (server-only, no rendering) runs the
   physics and sensors without a display at all. This is sufficient for every
   automated check and every Phase 6 suite run — nothing in `drishti_eval`
   needs a GPU-rendered view.
2. **Foxglove Studio**, connected from a laptop to the instance's exposed
   websocket port, for actually watching a run live or replaying a bag. This
   needs `ros-jazzy-foxglove-bridge`, which **is not yet in `docker/Dockerfile`**
   — add it to the `apt-get install` list alongside the other `ros-${ROS_DISTRO}-*`
   packages before relying on this.
3. **rosbag2 record, review later.** SETUP.md §5's checklist already requires
   a bag can be recorded and replayed — do this by default on every run, since
   a bag is cheap and a re-run of a cloud GPU instance is not.

Do not open the instance's ports to the public internet for Foxglove — tunnel
over SSH (`ssh -L 8765:localhost:8765 ...`) or restrict the security group to
the team's IPs.

---

## 4. Bring-up

```bash
# on the fresh instance
sudo apt-get update && sudo apt-get install -y docker.io nvidia-container-toolkit
sudo systemctl restart docker

git clone <repo-url> && cd drishti-ugv/drishti-ugv

docker build -t drishti:dev -f docker/Dockerfile .

docker run --rm -it --gpus all \
    -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -p 8765:8765 \
    -v "$PWD/ugv_ws:/ws/src/drishti" \
    drishti:dev
```

`-p 8765:8765` is the Foxglove bridge port from §3, once that package is added
to the image. The workspace is bind-mounted (Dockerfile's own comment: "an
edit on the host is an edit in the container") — do not `git clone` a second
copy inside the container.

---

## 5. First build

This is, per STATUS.md, **the first time any of this has ever been compiled**:

```bash
cd /ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Expected friction, already flagged rather than discovered fresh:

- **`rclcpp` API details in `safety_supervisor_node.cpp`** — the node was
  written against a remembered API, never compiled (STATUS.md, *Next actions*).
- **CuPy/CUDA version mismatch** — the Dockerfile comment calls this out as
  the most likely failure in the `elevation_mapping_cupy` path.
- **`gz` topic names** — `drishti_sim/config/bridge.yaml` was authored without
  a running Gazebo to check names against; confirm with `gz topic -l` before
  assuming a silent topic means a dead sensor rather than a typo (SETUP.md §5).

Do not treat a clean `colcon build` as validation of anything beyond
compilation — the acceptance criteria below are separate and unmet until
checked individually.

---

## 6. Phase 0 verification — do this before anything else

Directly from [SETUP.md §5](SETUP.md), in order:

```bash
gz topic -l                                     # names match bridge.yaml?
ros2 topic echo /camera/rgb/image_raw --once    # sane values, real sensor timestamp?
ros2 run tf2_tools view_frames                  # tree matches SPEC.md §3.1, no duplicate publishers
ros2 topic info /cmd_vel --verbose               # exactly ONE publisher: safety_supervisor
ros2 param get <any_node> use_sim_time            # true, for every node
```

The `/cmd_vel` check is the one the whole safety architecture rests on (D5,
CONTRIBUTING.md non-negotiable #1). If Nav2 or anything else shows up as a second
publisher, stop here — nothing downstream is safe to run until that's fixed.

Then, still with no simulator dependency:

```bash
cd /ws && python tools/run_checks.py
```

This should reproduce the ~5,000 offline assertions STATUS.md already claims
pass, on this machine, as a sanity check that the checkout is intact before
spending GPU time on anything else.

---

## 7. Running the framework — phase by phase

Follow TASK.md's phase gates; do not skip ahead on a green build. Launch files
already exist for each stage (`ugv_ws/src/*/launch/`):

| Phase | Bring-up | What to confirm |
|---|---|---|
| 1 — Sim navigation | `ros2 launch drishti_sim sim.launch.py`, `ros2 launch drishti_description description.launch.py`, `ros2 launch drishti_bringup bringup.launch.py` | UGV reaches a teleop'd, then a commanded, goal; TF stays clean while driving |
| 2 — Visual SLAM | `ros2 launch drishti_bringup slam.launch.py` (RTAB-Map, `config/rtabmap.yaml`) | loop closure on a revisited route; then `ros2 run drishti_eval evaluate_trajectory` against the logged ground-truth bag for ATE/RPE |
| 3 — Traversability | `ros2 launch drishti_bringup terrain.launch.py` on the Hard world (the ditch) | costmap refuses the ditch on step height; this is also the first real exercise of the CuPy path |
| 4 — Perception | `ros2 launch drishti_bringup perception.launch.py` | taxonomy and health topics populate; **do not install `ultralytics` into the image casually** — it's AGPL-3.0 with a separate commercial licence (REFERENCES.md §3), install it deliberately once the licence question for this use is settled |
| 5 — Safety | `ros2 launch drishti_bringup safety.launch.py`, then `ros2 run drishti_eval fault_injector` | each fault (`camera_silence`, `depth_silence`, `slam_loss`, `camera_freeze`) reports its own reason, not another sensor's |
| 6 — Suite | `python -m drishti_eval.plan_suite --count 1470 --base-seed 0 --json plan.json`, then execute the plan against the running stack | D20: **1470 missions per side**, not the ~1000 TASK.md originally planned, to detect a 2-point regression at the 95% baseline |
| 7 — Gates | `drishti_eval`'s budget/regression checks against the profile Phase 6 produced | meaningless before Phase 6 has run — this is why it's last |

`plan_suite` itself needs no ROS (it's pure Python, seeded, deterministic by
design — see its own module docstring); only *executing* the plan against the
sim does. Generate the plan file locally if convenient, then move only the
JSON to the cloud box.

`hardware.launch.py` in `drishti_bringup/launch/` is for the eventual physical
transfer (SETUP.md §6) — do not run it against this cloud instance, it targets
`config/hardware.yaml`'s edge-device contract, not a GPU box.

---

## 8. Record what happened

Per CONTRIBUTING.md's Definition of Done: a run that isn't recorded didn't happen,
for the purposes of any reported result. After each phase:

- Fill the empty rows in STATUS.md's **Pinned versions** table (Ubuntu, driver,
  ROS 2, Gazebo, CUDA, CuPy, Nav2, RTAB-Map, `elevation_mapping_cupy`, the
  `Twist`/`TwistStamped` choice) — these are "still entirely empty" as of the
  last STATUS.md update, and this is the machine that fills them.
- Add a row to STATUS.md's **Results log** for every mission-suite run, with
  its seed and parameter set — EVALUATION.md §7.2: "without the seed and the
  parameter set, a result is an anecdote."
- If a phase's acceptance criteria in TASK.md are met, check its `[ ]` boxes
  and flip the phase's STATUS.md table row from *assets written* to *run*.

---

## 9. Shut down

```bash
docker container prune          # inside the instance, once bags/results are copied out
# then, from the AWS console or CLI:
aws ec2 stop-instances --instance-ids <id>      # between sessions
aws ec2 terminate-instances --instance-ids <id> # once the phase's numbers are recorded
```

Copy rosbag2 recordings and any suite output off the instance before stopping
it — an EBS-backed stopped instance keeps its disk, but don't rely on that as
the archive; STATUS.md and the repository are.

---

## 10. Isaac Sim backend (D21)

Isaac Sim is the primary simulator. The sections above size the machine for
Gazebo; Isaac Sim needs a different one.

| | Gazebo backend (§1–9) | Isaac Sim backend |
|---|---|---|
| GPU | any CUDA GPU; a T4 is enough | RT cores required; NVIDIA's minimum is an RTX 4080-class GPU with 16 GB VRAM |
| RAM / disk | 16 GB / 100 GB | 32 GB minimum / 50 GB SSD for Isaac Sim alone; more for the stack and recordings |
| AWS instance (verify before booking) | `g4dn.xlarge` | a GPU instance with RT cores and at least 16 GB VRAM, for example `g5` (A10G, 24 GB) or `g6e` (L40S, 48 GB); run NVIDIA's Compatibility Checker before relying on one. A100 and H100 are **not** supported: they have no RT cores |
| Cost | low | substantially higher; the stop-don't-just-disconnect rule matters more |

Isaac Sim is available as a container (`nvcr.io/nvidia/isaac-sim`; NVIDIA's own
installer script defaults to tag 6.0.1, so check NGC for a newer one), as a
standalone build and as a Python package. The ROS 2 stack runs separately, in the
existing `docker/Dockerfile`; the two talk over DDS, so run both with
`--network host` and the same `ROS_DOMAIN_ID`.

```bash
export ISAAC_SIM_PATH=/isaac-sim        # the install directory, containing python.sh
ros2 launch drishti_bringup bringup.launch.py sim:=isaac headless:=true world:=easy.sdf
```

ROS 2 Jazzy must be sourced in that shell before launch: Isaac Sim reads its
library path when the process starts and cannot be repaired afterwards. The launch
file expands the xacro and starts Isaac Sim with its own `python.sh`.

`headless:=true` removes the window, not the GPU: the cameras still render.
Watch the run through Foxglove as in §3.

None of this has been run (STATUS.md D21). `drishti_sim_isaac/README.md` lists
what to check first.
