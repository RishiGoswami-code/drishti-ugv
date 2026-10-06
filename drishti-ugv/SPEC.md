# SPEC — DRISHTI-UGV

Technical specification. This is the contract every node in the system is
written against. If an implementation and this document disagree, one of them
is a bug — decide which, then fix it here first.

Last updated: 7 September 2026

---

## 1. System overview

```
                        ┌──────────────────────────┐
                        │     GAZEBO HARMONIC      │
                        │  UGV + terrain + sensors │
                        │  RGB / depth / IMU / odom│
                        └────────────┬─────────────┘
                                     │  ROS 2
            ┌────────────────────────┼────────────────────────┐
            ▼                        ▼                        ▼
      RGB image                 depth / cloud                IMU
            │                        │                        │
            ▼                        │                        │
  ┌───────────────────┐              │                        │
  │ PERCEPTION        │              │                        │
  │ detect + segment  │              │                        │
  └─────────┬─────────┘              │                        │
            │  semantic mask         │                        │
            └───────────┬────────────┘                        │
                        ▼                                     ▼
          ┌─────────────────────────────┐        ┌─────────────────────┐
          │ ELEVATION + TRAVERSABILITY  │        │ RTAB-MAP            │
          │ height, slope, roughness,   │◄──pose─┤ visual SLAM         │
          │ variance, semantic layers   │        │ map + loop closure  │
          └──────────────┬──────────────┘        └──────────┬──────────┘
                         │ traversability cost              │ pose / map
                         └───────────────┬──────────────────┘
                                         ▼
                              ┌─────────────────────┐
                              │ NAV2                │
                              │ costmaps, planner,  │
                              │ MPPI controller,    │
                              │ recovery behaviours │
                              └──────────┬──────────┘
                                         │ velocity command
                                         ▼
                              ┌─────────────────────┐
                              │ SAFETY SUPERVISOR   │  ◄── sensor health,
                              │ deterministic gate  │      pose health,
                              │ pass / slow / stop  │      obstacle distance
                              └──────────┬──────────┘
                                         ▼
                                     UGV base
```

The supervisor sits **downstream of Nav2 and outside the perception model**. It
is the last thing between a planner command and the wheels.

### 1.1 Separation of concerns

| Concern | Owner | Why |
|---|---|---|
| "What am I looking at?" | Neural networks | Genuinely uncertain, learned well |
| "Where am I?" | Visual SLAM (RTAB-Map) | Established geometric estimation |
| "What is drivable?" | Deterministic fusion of geometry + semantics | Must be auditable and tunable |
| Transforms, planning, control | Deterministic robotics (TF2, Nav2) | Correctness is provable, not probabilistic |
| "Should I move at all?" | Safety supervisor | Must never depend on model confidence |

---

## 2. Component selection

| Layer | Primary | Fallback | Decision rule |
|---|---|---|---|
| Simulator | **Gazebo Harmonic** | Isaac Sim 6.x, if a qualifying GPU is obtained | Decided 6 Sep 2026 by the SETUP.md §1.2 rule — see D15 |
| Middleware | ROS 2 Jazzy | ROS 2 Humble | Jazzy on Ubuntu 24.04 |
| SLAM | RTAB-Map | ORB-SLAM3 (offline benchmark only) | One SLAM system in the runtime graph, never two |
| Depth | Stereo / RGB-D | Depth Anything V2 Small | Prefer real metric depth |
| Detection | YOLO-family | any lightweight detector | Small class vocabulary first |
| Segmentation | YOLO-seg or dedicated segmenter | classical segmentation | Only the classes navigation needs |
| Elevation | `elevation_mapping_cupy` | custom grid map | Reuse the GPU implementation |
| Traversability | our fusion layer over the elevation map | `traversability-nav2` | Thin custom layer; see §6 |
| Global planning | Nav2 planner | custom A* | Nav2 |
| Local control | Nav2 MPPI | RPP / DWB | MPPI where compute allows |
| Packaging | Docker | native install | Containerise after the baseline works |

**Rule:** exactly one localisation stack runs at a time. Running RTAB-Map and
ORB-SLAM3 together adds integration and debugging cost without guaranteeing
better navigation.

> **Simulator decision, 6 September 2026 (D15).** The project's development GPU
> is an RTX 3050 laptop part (4–6 GB VRAM) in a Lenovo LOQ. NVIDIA's published
> Isaac Sim minimum — re-verified against the live requirements page on 6 Sep
> 2026 — is a **GeForce RTX 4080 with 16 GB VRAM and 32 GB system RAM**, and the
> same page warns that workloads "leveraging a large number of sensors are
> particularly affected" below that line. A stereo pair plus depth plus IMU over
> randomised outdoor terrain is exactly that workload. SETUP.md §1.2 says not to
> fight a marginal GPU, so **Gazebo Harmonic is now the primary simulator.**
>
> Nothing above the simulator changes. SLAM, traversability, Nav2, the
> supervisor and the whole §4 interface contract are untouched — which is what
> the contract is for.
>
> The RTX 3050 does clear the one *hard* gate: it has RT cores, unlike the
> integrated GPU audited on 5 Sep. Isaac Sim would launch. It is held as a
> contingent option for offline synthetic-data generation on small scenes, not
> as the development loop.

---

## 3. Coordinate frames and time

### 3.1 TF tree

```
map
 └── odom
      └── base_link
           ├── camera_link
           │    ├── camera_left_optical
           │    └── camera_right_optical
           └── imu_link
```

- `map → odom` is published by RTAB-Map (correction).
- `odom → base_link` is published by the odometry source (simulator or wheel/visual odometry).
- Everything below `base_link` is static, from the robot description.

### 3.2 Non-negotiable rules

1. **Nothing** publishes a transform it does not own. Two publishers on one
   edge is the single most expensive bug in this system.
2. Optical frames follow the ROS convention (z forward, x right, y down) and
   are distinct from the mounting frames.
3. Every sensor message carries a real sensor timestamp, never
   `now()` at publish time.
4. `use_sim_time` is `true` for **every** node when running against the
   simulator. A single node with the wrong clock corrupts the map.
5. TF and clock health are validated in Phase 0, before any AI work begins.

---

## 4. ROS 2 interface contract

This table is the stable boundary between subsystems. The simulator and a real
sensor driver publish the same topics with the same types and frames, which is
what makes the hardware transfer a driver swap.

### 4.1 Inputs

| Topic | Type | Frame | Publisher | Consumers |
|---|---|---|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/msg/Image` | `camera_left_optical` | sim / driver | perception |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | `camera_left_optical` | sim / driver | elevation, obstacle pipeline |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_left_optical` | sim / driver | perception, SLAM |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | `camera_left_optical` | depth → cloud | elevation mapping |
| `/imu/data` | `sensor_msgs/msg/Imu` | `imu_link` | sim / driver | SLAM, odometry |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` → `base_link` | sim / base | SLAM, Nav2 |

### 4.2 Internal

| Topic | Type | Publisher | Consumers |
|---|---|---|---|
| `/perception/detections` | `vision_msgs/msg/Detection2DArray` | perception | traversability fusion, supervisor |
| `/perception/semantic_mask` | `sensor_msgs/msg/Image` (mono8, class ids) | perception | traversability fusion |
| `/perception/health` | `drishti_msgs/msg/PerceptionHealth` | perception | supervisor |
| `/perception/nearest_obstacle` | `std_msgs/msg/Float32` (metres; NaN = nothing in range) | perception | supervisor |
| `/rtabmap/odom` | `nav_msgs/msg/Odometry` | RTAB-Map | Nav2, diagnostics |
| `/rtabmap/localization_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RTAB-Map | supervisor |
| `/map` | `nav_msgs/msg/OccupancyGrid` | RTAB-Map | Nav2, RViz2 |
| `/elevation_map` | `grid_map_msgs/msg/GridMap` | elevation mapping | traversability fusion |
| `/traversability` | `grid_map_msgs/msg/GridMap` | traversability fusion | Nav2 costmap layer |

### 4.3 Navigation and actuation

| Topic | Type | Publisher | Consumers |
|---|---|---|---|
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 | planner, RViz2 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 (+ our layer) | controller, RViz2 |
| `/plan` | `nav_msgs/msg/Path` | Nav2 | controller, RViz2 |
| `/cmd_vel_nav` | velocity command (see note) | Nav2 controller | **safety supervisor only** |
| `/cmd_vel` | velocity command (see note) | **safety supervisor** | UGV base |
| `/safety/state` | `drishti_msgs/msg/SafetyState` | supervisor | operator, dashboard, logs |
| `/safety/stop` | `std_msgs/msg/Bool` | supervisor | base, diagnostics |

> **Velocity message type.** Nav2 supports both `geometry_msgs/msg/Twist` and
> `geometry_msgs/msg/TwistStamped` depending on configuration. Pin the choice
> once in the shared params file and keep it identical across Nav2, the
> supervisor and the base. Verify against the Nav2 version actually installed —
> do not assume.

> **Critical wiring.** Nav2 must publish to `/cmd_vel_nav`, never directly to
> `/cmd_vel`. The supervisor is the only publisher on `/cmd_vel`. If Nav2 can
> reach the base directly, the safety design is void.

---

## 5. Perception

### 5.1 Sensor strategy

Stereo or RGB-D is preferred over monocular: metric depth dramatically
simplifies elevation mapping, 3D geometry and collision reasoning. Depth
Anything V2 Small is the monocular fallback, to be evaluated as an ablation —
not assumed equivalent.

### 5.2 Semantic taxonomy

Deliberately small. Every class must change a navigation decision; if it does
not, it does not belong here.

| Tier | Classes | Effect on cost |
|---|---|---|
| Traversable | compact dirt, road, validated grass | baseline |
| Caution | loose gravel, uneven grass, shallow rough terrain | moderate increase |
| High cost | mud, water, deep vegetation, steep slope | large increase |
| Lethal | ditch, cliff edge, large rock, tree trunk, wall | non-traversable |
| Dynamic | person, vehicle, animal | lethal + clearance margin, fast decay |

### 5.3 Outputs

| Output | Consumer |
|---|---|
| RGB frame (possibly downscaled) | detector / segmenter |
| Depth map, metres per pixel | point cloud → elevation pipeline |
| Semantic mask (class ids) | traversability fusion |
| Detections: class, confidence, box/mask | local costmap, supervisor |
| Per-frame confidence | speed policy |
| Camera health: timestamp, frame age | supervisor |

### 5.4 Model progression

Do **not** start by collecting thousands of images. Establish the full
navigation loop with pretrained perception, then let failure analysis justify
each model upgrade.

| Stage | Model | Purpose |
|---|---|---|
| MVP | none | prove the ROS 2 / SLAM / Nav2 loop |
| V1 | lightweight YOLO detector | rock, tree, person, vehicle |
| V2 | YOLO-seg or dedicated segmenter | terrain classes |
| V3 | stereo / RGB-D depth | metric geometry |
| V4 (optional) | Depth Anything V2 Small | monocular fallback / ablation |
| V5 | fine-tuned custom model | **only** if a benchmark shows a clear gap |

---

## 6. Terrain representation and traversability

Outdoor navigation differs from indoor navigation because free space is not
binary. A surface can be geometrically open and physically unsafe.

```
point cloud
    │
    ▼
elevation map ── height
    │         ├─ variance
    │         ├─ slope / gradient
    │         ├─ roughness
    │         ├─ visibility
    │         └─ semantic layers
    ▼
traversability cost
    │
    ▼
Nav2 costmap layer
```

### 6.1 Cost function

```
C(x) =  w_s · slope(x)
      + w_r · roughness(x)
      + w_h · height_variance(x)
      + w_o · obstacle(x)
      + w_m · semantic(x)
      + w_u · uncertainty(x)
```

Each term is normalised to `[0, 1]` before weighting. `C(x)` is mapped to the
Nav2 costmap range, with any lethal term saturating the cell.

**Weights are determined by controlled experiment, not by taste.** Record every
weight set and the mission-suite result it produced (EVALUATION.md §5).

### 6.2 The safety philosophy of unknown terrain

> Unknown terrain is expensive, never free.

A cell with no observation, low visibility or low perception confidence gets a
high `uncertainty` cost. The planner is then free to route around ignorance
instead of driving into it. This single rule prevents a large class of
ditch-and-water failures.

### 6.3 Adaptation note

`traversability-nav2` is a useful reference implementation but its pipeline is
LiDAR-centric. The intended adaptation is to feed it (or our equivalent layer)
a **camera-derived** point cloud / elevation representation — not to rewrite
the costmap concept.

---

## 7. Navigation, control and dynamic obstacles

Nav2 is the navigation backbone: planners, costmaps, behaviour-tree navigation,
collision monitoring, MPPI and regulated pure pursuit.

**Why MPPI:** it evaluates future control trajectories against the local
environment rather than following a geometric line — which matters for an
outdoor UGV where turning radius, speed and obstacle proximity interact.

**Why not a custom A* and controller first:** the goal is autonomous
navigation, not a reimplementation of textbook algorithms. Effort belongs in
terrain cost construction, sensor robustness, integration and validation.

### 7.1 Dynamic obstacle policy

Treat dynamic obstacles as a *local replanning* problem — the global route
usually stays valid.

1. Keep global planning relatively stable.
2. Update local obstacle costs rapidly.
3. Let the controller reject trajectories entering lethal cost.
4. If the path is blocked, trigger a local replan.
5. If obstacle distance falls below the emergency threshold, **bypass normal
   planning and stop.**

---

## 8. Performance budgets

| Quantity | Prototype | Stretch |
|---|---|---|
| Perception latency (frame arrival → output) | ≤ 100 ms | ≤ 60 ms |
| Control loop | ≥ 20 Hz | ≥ 30 Hz |
| Planner update | ≥ 5 Hz | ≥ 10 Hz |
| Emergency-stop software path | < 200 ms | < 100 ms |
| Localisation drift | < 2% of distance | < 1–2% |

Budgets are measured, not assumed — see EVALUATION.md §7.

---

## 9. Safety supervisor

The most important custom module. It must be **small, deterministic and easy to
audit**. It contains no learned components and makes no probabilistic
decisions.

### 9.1 Decision logic

```
if localization_lost:                    STOP
elif depth_stale:                        STOP
elif camera_stale:                       STOP
elif frames_static > t_frame_static:     STOP
elif obstacle_distance < d_emergency:    STOP
elif path_is_invalid:                    STOP
elif command_not_finite:                 STOP
elif perception_confidence < c_critical: SLOW_DOWN
else:                                    PASS THROUGH Nav2 COMMAND
```

Evaluation order is fixed and the first match wins. Every branch publishes a
reason code on `/safety/state`.

> **Corrected 5 September 2026 (D13).** An earlier revision of this block put
> the low-confidence `SLOW_DOWN` branch *above* `path_is_invalid`. Under "first
> match wins" that forwarded a command whenever confidence was low, even with
> no usable path — low confidence masked a harder fault. **Every STOP condition
> is now evaluated before the single SLOW branch**, which can only ever turn a
> SLOW into a STOP and never the reverse. `command_not_finite` was also hoisted
> above the forwarding paths, because clamping `NaN` yields `NaN`.
>
> Two consequences worth keeping in mind:
> - The reason-code *numbers* in `drishti_msgs/msg/SafetyState` still follow the
>   original listing order, so they no longer match the evaluation order. Do not
>   infer precedence from them.
> - A non-finite `nearest_obstacle` is read as "nothing found in range", not as
>   a fault. That is only sound because depth staleness is ruled out first —
>   the ordering is load-bearing, not cosmetic.

### 9.2 Conditions

| Condition | Action | Reason |
|---|---|---|
| No camera frames within `t_camera_stale` | STOP | perception cannot be trusted |
| Depth timestamp older than `t_depth_stale` | STOP | obstacle distance may be invalid |
| Localisation lost or covariance above threshold | STOP | pose-dependent navigation is unsafe |
| Lethal obstacle inside `d_emergency` | STOP | avoid collision |
| RGB content unchanged for `t_frame_static` | STOP | the camera is frozen; see below |
| Perception confidence below `c_critical` | SLOW | reduce risk while gathering information |
| No valid path | STOP / RECOVER | do not drive blindly |
| Planner command invalid (NaN, out of range) | STOP | protect the motor interface |

### 9.3 Parameters

All thresholds live in one params file and are logged at startup with the run.

| Parameter | Meaning |
|---|---|
| `t_camera_stale` | max age of an RGB frame |
| `t_depth_stale` | max age of a depth frame |
| `d_emergency` | emergency obstacle distance |
| `c_critical` | perception confidence floor for full speed |
| `v_max`, `v_slow` | normal and reduced speed limits |
| `cov_max` | pose covariance ceiling before "lost" |
| `watchdog_period` | supervisor tick; bounds stop latency |
| `t_frame_static` | max seconds of unchanging RGB content before STOP |
| `t_pose_stale` | max age of a localisation pose |
| `t_plan_stale` | max age of a `/plan` before it is unusable |
| `t_cmd_stale` | max age of a Nav2 command before STOP |

> **Liveness is not freshness (added 7 September 2026, D19).** `t_camera_stale`
> catches a camera that goes *silent*. It cannot catch one that *freezes* --
> republishing a single image with a fresh timestamp -- because the age never
> grows. That failure presents as a perfectly healthy camera while the vehicle
> drives on a view of the world that is minutes old, and it is the more
> dangerous of the two.
>
> Perception therefore reports `rgb_static_for` on `/perception/health`, from a
> per-frame content signature, and the supervisor stops on it independently of
> `rgb_age`. `t_frame_static` is deliberately far longer than `t_camera_stale`:
> a motionless vehicle looking at a static scene through a noiseless simulated
> camera legitimately repeats frames. A spurious trip produces a STOP, which is
> the safe direction to be wrong in.
>
> Absence of the signal is **not** treated as a freeze. A perception build too
> old to publish it leaves the value at zero, and silence is already covered by
> the health message itself going stale.

`t_pose_stale`, `t_plan_stale` and `t_cmd_stale` were added on 5 September
2026 (D13). The original list gave
staleness limits for the camera and depth streams only; the supervisor also
depends on the pose, the plan and the command stream, and without limits on
those a silent planner or a dead Nav2 would leave it forwarding a stale command
indefinitely — a direct breach of invariant 9.4.2.

### 9.4 Invariants

1. The supervisor is the **only** publisher on `/cmd_vel`.
2. Loss of input is a stop condition, never a pass-through. Absence of evidence
   is not evidence of safety.
3. The supervisor never *increases* a commanded velocity.
4. It ticks on its own timer, independent of whether Nav2 is publishing.
5. Its behaviour is unit-testable without ROS running.

---

## 10. Simulation

The simulator is a **test laboratory**, not a visualisation tool.

### 10.1 Escalation ladder

Do these in order. Do not skip.

1. Teleoperate a virtual UGV.
2. Send a fixed goal through Nav2.
3. Navigate using simulated odometry.
4. Remove GPS/ground-truth pose; use visual SLAM.
5. Add depth and elevation mapping.
6. Add semantic perception.
7. Add dynamic obstacles.
8. Randomise lighting, terrain and obstacle placement.
9. Inject sensor failures.
10. Run hundreds to thousands of missions automatically.

### 10.2 Worlds

| World | Purpose | Contents |
|---|---|---|
| Easy | baseline navigation | flat dirt, sparse rocks |
| Medium | terrain reasoning | grass, uneven ground, slopes, trees |
| Hard | unstructured outdoor | ditches, rocks, dense vegetation, narrow corridors |
| Dynamic | collision avoidance | moving humans and vehicles, sudden obstacles |
| Adversarial | robustness | strong shadow, glare, low light, sensor noise |
| Failure | safety | frozen camera, stale depth, SLAM loss, invalid commands |

### 10.3 Domain randomisation

Sun angle and intensity · cloud cover and shadow · camera exposure and noise ·
terrain texture · obstacle size and placement · start and goal pose · vehicle
speed · dynamic-object trajectories · depth noise and dropout.

### 10.4 Ground truth

The simulator gives exact vehicle pose and object geometry. Store ground truth
**alongside** every rosbag2 recording so localisation error, obstacle distance
and path deviation are computed objectively rather than eyeballed.

---

## 11. Failure modes

| Failure | Likely cause | Mitigation |
|---|---|---|
| Robot spins or drives wrongly | TF / base-frame error | validate TF tree and wheel geometry |
| SLAM drifts | poor features, motion blur, exposure | stereo + IMU, better camera, speed limits |
| Terrain map "floats" | pose/depth desynchronisation | timestamp checks, calibration |
| Rock missed | model confidence, occlusion | depth + semantic fusion, safety margin |
| Ditch classified as ground | semantic/geometry mismatch | slope and height-variance risk, conservative cost |
| Dynamic obstacle collision | slow local update | fast costmap, MPPI, emergency threshold |
| Simulation runs slowly | GPU/VRAM overload | lower resolution, fewer sensors, simpler scene |
| AI latency too high | model too large | smaller model, TensorRT, profiling |
| Unknown terrain | out-of-distribution perception | increase cost, slow or stop |
| Camera failure | hardware or stream fault | safety supervisor |
| Localisation loss | no visual features | stop, then re-localise |
| Planner finds no path | terrain genuinely blocked | recover or stop; never force motion |

---

## 12. Hardware transfer

```
SIMULATION
Gazebo → ROS 2 → RTAB-Map → traversability → Nav2 → supervisor
                       │
                       ▼
              hundreds/thousands of missions
                       │
                       ▼
              hardware interface swap
                       │
                       ▼
REAL UGV
stereo/RGB-D + IMU → ROS 2 drivers → (identical stack)
```

Only the **source of sensor messages** and the **low-level motor interface**
change. Everything between is untouched.

### 12.1 Bring-up sequence

1. Bench-test camera and IMU streams.
2. Verify calibration and timestamps.
3. Verify TF frames while stationary.
4. Drive manually at very low speed.
5. Validate odometry.
6. Run visual SLAM with no autonomous control.
7. Run perception and mapping while teleoperated.
8. Enable Nav2 at low speed **with an external emergency stop**.
9. Increase speed only after repeated collision-free runs.

Steps are strictly ordered. An external hardware e-stop is mandatory from step
8 onward, independent of the software supervisor.
