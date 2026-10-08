# REFERENCES — DRISHTI-UGV

Upstream repositories, licences and official documentation.

> **Licence caveat — read before relying on this file.**
> Licence positions below are **as recorded in the research baseline on
> 4 September 2026**. Licences, model weights and bundled dependencies change,
> and a repository's headline licence does not always cover everything inside
> it. **Re-verify upstream before integrating, redistributing or deploying
> commercially.** This is an engineering screen, not legal advice; commercial
> deployment needs a formal legal review.

---

## 1. Core dependencies

Components we intend to run.

| Repository | Role | Licence (as of 4 Sep 2026) | Constraint |
|---|---|---|---|
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | Navigation framework: planners, costmaps, behaviour trees, MPPI, collision monitor | Open-source ROS ecosystem — verify per package | Needs robot-specific configuration |
| [introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros) | ROS 2 visual SLAM, stereo/RGB-D, Nav2 integration examples | BSD-3-Clause | Needs calibrated sensors and a correct TF setup |
| [leggedrobotics/elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) | GPU elevation mapping, semantic layers, traversability filtering | MIT | GPU/CUDA compatibility must be validated |
| [leggedrobotics/elevation_mapping_cupy_core](https://github.com/leggedrobotics/elevation_mapping_cupy_core) | Standalone ROS-free GPU elevation-map core | MIT | — |
| [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | YOLO detection and segmentation | **Varies by distribution and use** | Obtain the appropriate licence for the intended use |
| [isl-org/Open3D](https://github.com/isl-org/Open3D) | 3D processing, registration, visualisation | MIT | General-purpose utility, not a navigation stack |
| [isaac-sim/IsaacSim-ros_workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces) | ROS 2 workspaces for Isaac Sim | Apache-2.0 | — |
| NVIDIA Isaac Sim | Simulator | **NVIDIA proprietary / EULA** | Review the EULA and deployment rights |
| [isaac-sim/IsaacSim](https://github.com/isaac-sim/IsaacSim) | Isaac Sim source, standalone examples and the ROS 2 node definitions `drishti_sim_isaac` was written against (read at tag v6.1.0) | Apache-2.0 for the source; binaries under the NVIDIA EULA | Verify |

## 2. Reference and fallback

Components we read, benchmark against, or hold in reserve.

| Repository | Role | Licence (as of 4 Sep 2026) | Why not primary |
|---|---|---|---|
| [LARIAD/Offroad-Nav](https://github.com/LARIAD/Offroad-Nav) | Closest end-to-end off-road reference stack: Isaac Sim, elevation mapping, monocular depth, navigation | MIT | Its main branch targets **ROS 1 Noetic, Isaac Sim 4.5, CUDA 11.6**; a ROS 2 port is in progress. Treat as architecture inspiration, not a drop-in |
| [sacrover/traversability-nav2](https://github.com/sacrover/traversability-nav2) | Outdoor traversability costmap plugin for Nav2 | verify | Reference pipeline is LiDAR-centric; we feed camera-derived elevation instead |
| [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | Visual, visual-inertial and multi-map SLAM | **GPLv3** | Excluded from the shipped build. Offline accuracy benchmark only |
| [DepthAnything/Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2) | Monocular depth estimation | **Small: Apache-2.0; Base/Large/Giant: CC-BY-NC-4.0** | Monocular fallback only. Do not assume commercial use is permitted for the larger variants |
| [clearpathrobotics/clearpath_simulator](https://github.com/clearpathrobotics/clearpath_simulator) | ROS 2 Jazzy + Gazebo Harmonic robot simulation | verify | Gazebo fallback path if the GPU does not meet the Isaac Sim floor |
| [husky/husky](https://github.com/husky/husky) | Clearpath Husky description, control, simulation | verify | Useful UGV model reference |

---

## 3. Licence policy

The rule that decides whether a dependency may be added:

| Verdict | Licences | Action |
|---|---|---|
| **Cleared** | MIT, BSD-2/3-Clause, Apache-2.0 | May be integrated. Preserve notices and attribution |
| **Review first** | Anything ambiguous, dual-licensed, or use-dependent (e.g. Ultralytics) | Do not integrate until the terms for our specific use are confirmed and recorded in STATUS.md |
| **Excluded from the build** | GPLv3 (e.g. ORB-SLAM3) | Offline benchmarking and comparison only. Never linked, vendored or shipped |
| **Non-commercial weights** | CC-BY-NC-4.0 (Depth Anything V2 Base/Large/Giant) | Not usable for a commercial deployment path. Small variant only |
| **Proprietary tooling** | NVIDIA Isaac Sim EULA | Development tool. Confirm it is not a runtime dependency of the delivered module |

### 3.1 Practical checks

- Model **weights** and model **code** can carry different licences. Check both.
- A repository's headline licence does not always cover bundled third-party
  assets, datasets or submodules.
- Isaac Sim is a *development* dependency. The delivered software module must
  not require it at runtime — that separation is what keeps the deliverable
  redistributable.
- Preserve every third-party notice in the delivered repository (NFR-06).

---

## 4. Official documentation

| Resource | URL |
|---|---|
| Isaac Sim — ROS 2 installation and bridge | https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html |
| Isaac Sim 6.0.1 — system requirements | https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/requirements.html |
| ROS 2 Jazzy documentation | https://docs.ros.org/en/jazzy/ |
| Nav2 documentation | https://docs.nav2.org/ |
| RTAB-Map | https://introlab.github.io/rtabmap/ |
