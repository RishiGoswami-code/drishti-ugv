#!/usr/bin/env bash
# DRISHTI-UGV -- Codespaces / devcontainer bring-up, CPU only, no GPU.
#
# !! UNVERIFIED !! Never executed. This is the corrected, no-billing-risk
# counterpart to docker/Dockerfile (which targets a CUDA machine this project
# doesn't currently have funding for). It installs the same ROS 2 Jazzy /
# Gazebo Harmonic / Nav2 / RTAB-Map stack, on the Ubuntu 24.04 that ROS 2
# Jazzy actually ships binaries for -- see FREE_SETUP.md for why that
# distinction matters. It does NOT install CUDA or CuPy: there is no GPU here,
# so elevation_mapping_cupy (Phase 3) cannot run in this environment. That is
# a known, accepted scope cut, not an oversight -- see FREE_SETUP.md.
set -euo pipefail

sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common locales \
    build-essential cmake git python3-pip python3-venv
sudo locale-gen en_US.UTF-8

# Keyring method, not apt-key (deprecated / often nonfunctional on current
# Ubuntu) -- same approach docker/Dockerfile already uses correctly.
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-grid-map \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    python3-colcon-common-extensions python3-rosdep python3-vcstool

sudo rosdep init 2>/dev/null || true
rosdep update

pip install --break-system-packages --quiet numpy pytest pyyaml

{
  echo "source /opt/ros/jazzy/setup.bash"
  echo "[ -f /workspaces/*/drishti-ugv/ugv_ws/install/setup.bash ] && source /workspaces/*/drishti-ugv/ugv_ws/install/setup.bash"
} >> ~/.bashrc

echo ""
echo "Setup complete. Next:"
echo "  cd drishti-ugv/ugv_ws"
echo "  rosdep install --from-paths src --ignore-src -y"
echo "  colcon build --symlink-install"
echo ""
echo "Expect the first build to fail on rclcpp API details -- STATUS.md"
echo "already names this as the accepted risk of building offline-first."
