# SETUP — DRISHTI-UGV

Machine requirements and the install order.

> **Version caveat.** The figures below are as recorded in the research
> baseline on **4 September 2026**. NVIDIA revises Isaac Sim requirements
> between releases. Re-check the official requirements page for the release you
> actually install before buying or committing hardware.

---

## 1. Development workstation

NVIDIA's published configuration tiers for Isaac Sim 6.0.1 on x86_64:

| Tier | CPU | RAM | GPU | VRAM | Storage |
|---|---|---|---|---|---|
| Minimum | 4 cores | 32 GB | GeForce RTX 4080-class | 16 GB | 50 GB SSD |
| Good | 8 cores | 64 GB | RTX 5080-class | 16 GB+ | 500 GB SSD |
| Ideal | 16 cores | 64 GB | RTX PRO 6000 Blackwell-class | 48 GB+ | 1 TB NVMe |

Our practical reading for this project:

| Tier | Use case |
|---|---|
| Minimum | basic Isaac Sim, limited sensors and scene complexity |
| Recommended (8+ cores, 64 GB, 16 GB+ VRAM) | full project development |
| Comfortable (12–16 cores, 64–128 GB, 24–48 GB VRAM) | large worlds, many sensors, AI plus simulation |
| Team server (16+ cores, 128 GB+, 48 GB+ VRAM) | parallel experiments, CI simulation |

### 1.1 Hard constraints

- **GPUs without RT cores are not supported** by Isaac Sim. A100 and H100 will
  not work. This surprises people who assume a datacentre GPU is strictly
  better.
- More RAM and VRAM are needed for advanced scenes and many simultaneous
  sensors — the minimum tier will not carry the Phase 6 mission suite.
- OS: Ubuntu 22.04 or 24.04, or Windows 11. **We target Ubuntu 24.04** for ROS 2
  Jazzy.

### 1.2 The Phase 0 decision

If the available GPU does not meet the Isaac Sim floor, switch the simulator to
**Gazebo Harmonic** and record the decision in STATUS.md. Everything above the
simulator — the ROS 2 contract, SLAM, traversability, Nav2, the supervisor — is
unchanged by that choice. This is the point of the interface contract in
SPEC.md §4.

Do not spend days fighting a marginal GPU. Decide, record, move.

### 1.3 Decision taken — 6 September 2026

**Simulator: Gazebo Harmonic.** The development machine is a **Lenovo LOQ with
an RTX 3050 laptop GPU** (4–6 GB VRAM). Isaac Sim's published minimum is an RTX
4080 with 16 GB VRAM and 32 GB RAM, re-verified against NVIDIA's live
requirements page on this date. The rule in §1.2 applies without argument.

The RTX 3050 does clear Isaac Sim's one *hard* gate — it has RT cores — so
Isaac Sim would start. That is precisely the trap §1.2 warns about: it fails on
scene complexity and sensor count rather than at launch, so the wasted time is
discovered late. Isaac Sim stays available for offline synthetic-data
generation on small scenes; it is not the development loop.

The machine does have **CUDA**, which the previously audited laptop did not.
`elevation_mapping_cupy` is therefore back on the primary path (STATUS.md D16),
and local YOLO inference and light fine-tuning are feasible within 4–6 GB.

Recorded as D15 and D16 in STATUS.md.

---

## 2. Software

| Component | Version | Note |
|---|---|---|
| Ubuntu | 24.04 | for ROS 2 Jazzy |
| NVIDIA driver | matching the chosen Isaac Sim release | check the release notes, not habit |
| ROS 2 | Jazzy | one distribution across the whole project |
| Isaac Sim | 6.x | verify with the NVIDIA Compatibility Checker |
| CUDA toolkit | as required by `elevation_mapping_cupy` and the AI stack | this is the usual conflict point |
| Python | as required by the ROS 2 / Isaac Sim environment | do not mix with a system Python |
| Build tools | `git`, `cmake`, `colcon`, `rosdep` | |
| Containers | Docker + NVIDIA Container Toolkit | only if native dependencies conflict |

---

## 3. Install order

Follow this sequence. Each step is verifiable; do not proceed past a failure.

1. Install Ubuntu 24.04 and the NVIDIA driver.
2. Install ROS 2 Jazzy. *Verify:* talker/listener pair communicates.
3. Install Isaac Sim 6.x. *Verify:* Compatibility Checker passes.
4. Install/clone the Isaac Sim ROS 2 workspace. *Verify:* the ROS 2 bridge loads.
5. Create a simple simulated UGV with a stereo camera and IMU.
6. Verify the ROS 2 image, depth, IMU and TF topics arrive with sane timestamps.
7. Install Nav2 and drive the virtual UGV to a goal.
8. Integrate RTAB-Map with stereo/RGB-D.
9. Add `elevation_mapping_cupy`.
10. Build the traversability cost layer.
11. Add lightweight perception.
12. Add semantic/geometry fusion.
13. Add the safety supervisor.
14. Automate scenario generation and metrics.

This mirrors the phase structure in [TASK.md](TASK.md) — steps 1–4 are Phase 0,
5–7 are Phase 1, and so on.

---

## 4. Avoiding dependency hell

This is a real and expensive failure mode on this stack. CUDA, Python and ROS
version constraints from Isaac Sim, `elevation_mapping_cupy` and the AI models
do not automatically agree.

- **Do not install every repository at once.** Add one component, verify it,
  then add the next.
- **Pin working versions** as soon as a baseline works, and record them in
  STATUS.md.
- **Prefer official ROS packages** over building from source where a binary
  exists for Jazzy.
- **Use one ROS distribution** across the entire project. No mixed Humble/Jazzy.
- **Containerise components with conflicting dependencies** rather than
  fighting a single global environment.
- **Keep each major subsystem independently launchable** so a broken dependency
  in one does not block work on another.

---

## 5. Verification checklist

Before declaring Phase 0 complete:

- [ ] `colcon build` succeeds on a clean clone
- [ ] Isaac Sim (or Gazebo) publishes ROS 2 topics that `ros2 topic echo` receives
- [ ] `ros2 run tf2_tools view_frames` shows the SPEC.md §3.1 tree
- [ ] No TF edge has two publishers
- [ ] Every node reports `use_sim_time: true`
- [ ] Sensor messages carry real sensor timestamps, not publish time
- [ ] Exact versions of ROS 2, simulator, CUDA and driver recorded in STATUS.md
- [ ] A rosbag2 recording can be made and replayed

The TF and clock checks are the two that people skip and then pay for during
Phase 3, when the elevation map "floats" and the cause is three weeks upstream.

---

## 6. Future edge computer

For a physical UGV, a Jetson-class NVIDIA edge computer is the logical target,
but **the exact module is selected after profiling, not before**. Benchmark on
the development workstation first (Phase 7), then measure on the intended
device. Choosing a module from a spec sheet before the stack is profiled is how
teams end up with hardware that cannot run their own code.
