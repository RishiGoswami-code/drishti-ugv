# drishti_sim_isaac

The Isaac Sim backend. It builds the vehicle and the Easy/Medium/Hard worlds in
USD and publishes the SPEC.md §4.1 topics through Isaac Sim's ROS 2 bridge, so the
rest of the stack cannot tell it from the Gazebo backend.

> **UNVERIFIED.** Written against the Isaac Sim 6.1 API and never run: no machine
> on the project meets NVIDIA's minimum (STATUS.md D21). What is checked is
> offline and limited to consistency: `tools/check_isaac_assets.py`. Treat the
> first run as the first build of anything, and expect API-level fixes.

## Run

```bash
export ISAAC_SIM_PATH=/path/to/isaac-sim        # contains python.sh
source /opt/ros/jazzy/setup.bash                # before launch; see the Isaac Sim ROS notes
ros2 launch drishti_sim_isaac isaac_sim.launch.py world:=easy.sdf
# or the whole stack:
ros2 launch drishti_bringup bringup.launch.py sim:=isaac world:=easy.sdf
```

`hard.sdf` needs `z:=0.60`: the vehicle drives on a 0.45 m platform.

## How it is built

| File | Role |
|---|---|
| `scripts/scene_model.py` | Pure Python: parses the expanded URDF and the SDF worlds, composes frames, camera orientation, drive kinematics. Checked offline. |
| `scripts/usd_builder.py` | The only `pxr` code: USD geometry, UsdPhysics bodies and wheel joints, cameras, lights. |
| `scripts/run_isaac_sim.py` | Standalone Isaac Sim app: builds the scene, creates the OmniGraph ROS 2 graphs, runs the drive loop. |
| `launch/isaac_sim.launch.py` | Expands the xacro, starts `python.sh run_isaac_sim.py`, starts `robot_state_publisher`. |
| `config/isaac_sim.json` | The topic seam, step rate, wheel drive, lighting. |

Decisions worth knowing:

- **One source of truth.** The robot comes from `drishti.urdf.xacro` and the
  worlds from `drishti_sim/worlds/*.sdf`. Nothing is copied into this package, so
  the two simulators cannot disagree about a dimension, a mass or a rock.
- **No URDF importer.** Isaac's importer output layout changed between releases; the
  runtime needs known prim paths, so the vehicle is built from UsdPhysics directly.
- **`/cmd_vel` is subscribed to, never published.** The safety supervisor is the
  only publisher (SPEC.md 4.3, 9.4.1). The drive loop reads the twist from the
  `ROS2SubscribeTwist` node and sets wheel velocity targets, with the same
  linear-acceleration limit as the Gazebo DiffDrive plugin.
- **One TF edge.** Only `odom -> base_link` is published here.
  `robot_state_publisher` owns `base_link -> sensors`.
- **`/ground_truth/pose` is evaluation-only** and nothing at run time may
  subscribe to it.

## First-run checklist

Each item is an assumption nobody has been able to check. Do them in order.

1. **Startup.** `python.sh` finds `isaacsim`; `enable_extension("isaacsim.ros2.bridge")`
   works; ROS 2 Jazzy is picked up (look for `internal_lib_fallback` in the log).
2. **Topics.** `ros2 topic list` shows every entry of `config/isaac_sim.json`, and
   `ros2 topic info /cmd_vel --verbose` lists exactly one publisher
   (`safety_supervisor`), as with Gazebo.
3. **Clock and stamps.** `/clock` advances; sensor header stamps are sim time.
4. **TF.** `odom -> base_link` has exactly one publisher; `view_frames` shows the
   SPEC.md 3.1 tree with no duplicate edge.
5. **Cameras.** Images are the right way up and look along +x; `/camera/right/camera_info`
   has a sensible `Tx` (about `-fx * 0.12`); depth is in metres; `/camera/points`
   is in the left optical frame; the rates are about 30 Hz.
6. **IMU.** `/imu/data` has gravity on +z at rest and its orientation is sane. This
   uses the experimental `IMU` prim with the `IsaacReadIMU` node, a pairing taken
   from NVIDIA's tests, not from a working example.
7. **Drive.** The vehicle holds still, drives straight on a forward command, and
   turns the right way on a positive yaw rate. If the wheels spin but the vehicle
   does not move, check the drive gains in `config/isaac_sim.json` first (USD angular
   drive units are per degree).
8. **Odometry.** `/odom` starts at the origin; `compose_ground_truth` assumes the
   node reports start-relative position and `R * R0^-1` orientation, as its
   source does. Compare `/ground_truth/pose` with the spawn pose.
9. **Contact.** The vehicle settles without jitter on the Easy world, then on
   `hard.sdf` with `z:=0.60`.

## Known differences from the Gazebo backend

Not yet characterised; see SPEC.md §10.5. Camera noise is not modelled here.
Depth uses the left camera's 0.05–60 m clip. IMU and odometry publish at the 60 Hz
step rate, where the description asks for 100 Hz. The physics step is 1/60 s, not
1 ms. Do not compare numbers across simulators.
