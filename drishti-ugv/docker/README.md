# Container

`Dockerfile` builds a CUDA + ROS 2 Jazzy image with Nav2, RTAB-Map, grid_map
and the `ros_gz` bridge. TASK.md Phase 7 asks for the validated stack to be
containerised; this exists so that the environment is one command when a
machine appears.

**Never built.** No machine on the project has Docker or ROS 2 (STATUS.md D17).

```bash
docker build -t drishti:dev -f docker/Dockerfile .

docker run --rm -it --gpus all \
    -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD:/ws/src/drishti" drishti:dev
```

Then, inside:

```bash
cd /ws && colcon build --symlink-install && source install/setup.bash
```

## Two deliberate choices

**CUDA base, ROS added on top** — not `ros:jazzy` with CUDA retrofitted. The
terrain layer needs CuPy (D16), and adding CUDA to a ROS image is the harder
direction.

**`ultralytics` is not installed.** It is AGPL-3.0 with a separate commercial
licence (REFERENCES.md §3). Baking it in would make every container carry that
obligation, including ones used only for the tested, model-free half of the
stack. Install it explicitly once the licence question is settled.

## Expected first-build friction

The CuPy/CUDA pairing is the most likely thing to need adjusting —
`cupy-cuda12x` must match the base image's CUDA major version, and
`elevation_mapping_cupy` has its own expectations on top of that.
