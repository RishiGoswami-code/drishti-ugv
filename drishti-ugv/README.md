# DRISHTI-UGV — documentation index

Vision-first autonomous navigation for an outdoor unmanned ground vehicle in
GPS-denied environments. The top-level [README](../README.md) gives the abstract,
contributions and current status; this directory holds the detailed documents and
the ROS 2 workspace.

## Core principle

> The neural network only answers *"what am I looking at?"*.
> Coordinate transforms, mapping, planning, control and the stop decision stay
> in deterministic, auditable code — so a perception failure degrades into a
> safe halt, never a collision.

## Documents

Read them in this order.

| File | What it answers |
|---|---|
| [PRD.md](PRD.md) | The problem, requirements, success criteria and risks |
| [SPEC.md](SPEC.md) | How it is built: architecture, interfaces, algorithms, budgets |
| [SETUP.md](SETUP.md) | Machine requirements and the install order |
| [CLOUD_SETUP.md](CLOUD_SETUP.md) | Running the stack on a rented GPU machine, phase by phase (needs a budget) |
| [FREE_SETUP.md](FREE_SETUP.md) | The no-budget path: devcontainer and CI instead of a rented machine |
| [TASK.md](TASK.md) | The phased backlog, with acceptance criteria |
| [EVALUATION.md](EVALUATION.md) | Metrics, the test scenario catalog, how results are scored |
| [REFERENCES.md](REFERENCES.md) | Upstream repositories, licences and documentation |
| [STATUS.md](STATUS.md) | Living state of the project and the decision log |
| [CONTRIBUTING.md](CONTRIBUTING.md) | Design invariants and working conventions |

## Stack

| Layer | Choice | Fallback |
|---|---|---|
| OS | Ubuntu 24.04 | — |
| Middleware | ROS 2 Jazzy | ROS 2 Humble |
| Simulator | Isaac Sim 6.x | Gazebo Harmonic (also the CI simulator) |
| Localisation | RTAB-Map (stereo + IMU) | ORB-SLAM3 (offline benchmark only) |
| Terrain | `elevation_mapping_cupy` | custom grid map |
| Navigation | Nav2 + MPPI controller | Nav2 + RPP / DWB |
| Perception | pretrained detection + segmentation, stereo/RGB-D depth | Depth Anything V2 Small |
| Safety | custom deterministic supervisor | — |

Languages: **Python** (perception, tooling, evaluation) and **C++** (real-time
nodes, costmap layers, safety supervisor).

Software versions, compatibility and licence terms move. Anything version-specific
in these documents carries an as-of date and must be re-verified against upstream
before it is relied on.
