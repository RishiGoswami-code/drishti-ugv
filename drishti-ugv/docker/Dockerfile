# DRISHTI-UGV — reproducible development and evaluation image.
#
# TASK.md Phase 7: "containerise the validated stack". Written ahead of the
# validation, so that when a machine appears the environment is one command
# rather than an afternoon of apt.
#
# !! UNVERIFIED !! Never built. No machine on the project has Docker or ROS 2
# (STATUS.md D17). Expect the usual first-build friction, most likely around
# the elevation_mapping_cupy CUDA/CuPy pairing.
#
#   docker build -t drishti:dev -f docker/Dockerfile .
#   docker run --rm -it --gpus all \
#       -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#       -v "$PWD:/ws/src/drishti" drishti:dev
#
# The CUDA base is chosen deliberately over a plain ros:jazzy image: the
# terrain layer needs CuPy (STATUS.md D16), and retrofitting CUDA onto a ROS
# image is more painful than adding ROS to a CUDA one.
FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=jazzy

# ---------------------------------------------------------------- base OS
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release software-properties-common locales \
        build-essential cmake git python3-pip python3-venv \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------- ROS 2 Jazzy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-rtabmap-ros \
        ros-${ROS_DISTRO}-grid-map \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-sim \
        python3-colcon-common-extensions python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------- Python
# CuPy must match the CUDA major version of the base image.
RUN pip3 install --no-cache-dir --break-system-packages \
        "cupy-cuda12x" numpy pytest

# NOT installed here: ultralytics. It is AGPL-3.0 with a separate commercial
# licence (REFERENCES.md §3), and baking it into the image would make every
# container carry that obligation. Install it deliberately, once the licence
# question is settled:
#     pip3 install --break-system-packages ultralytics

RUN rosdep init 2>/dev/null || true

WORKDIR /ws
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" \
        >> /root/.bashrc

# Nothing is copied in: the workspace is bind-mounted so an edit on the host is
# an edit in the container. Baking the source into the image would make the
# fastest loop in the project the slowest.
CMD ["/bin/bash"]
