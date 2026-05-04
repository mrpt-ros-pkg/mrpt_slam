#!/usr/bin/env bash
# Replicates the GitHub CI workflow locally inside a Docker container.
# Usage:
#   ./scripts/ci_local.sh [rolling|jazzy|humble]   (default: rolling)
#
# The script mounts the current repo read-only, mirrors the CI steps exactly,
# and exits with the same code the workflow would produce.

set -eo pipefail

DISTRO=${1:-rolling}

case "$DISTRO" in
  humble)  IMAGE="ubuntu:jammy"  ;;
  jazzy)   IMAGE="ubuntu:noble"  ;;
  rolling) IMAGE="ubuntu:noble"  ;;
  *) echo "Unknown distro: $DISTRO"; exit 1 ;;
esac

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== CI local: ros_distribution=$DISTRO image=$IMAGE ==="
echo "=== repo: $REPO_ROOT ==="

docker run --rm \
  -v "$REPO_ROOT:/src/mrpt_slam:ro" \
  -e DEBIAN_FRONTEND=noninteractive \
  "$IMAGE" \
  bash -c "
set -eo pipefail

# ── Setup ROS (mirrors ros-tooling/setup-ros@v0.7) ──────────────────────────
apt-get update -qq
apt-get install -y -qq curl gnupg2 lsb-release ca-certificates rsync
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" \
  > /etc/apt/sources.list.d/ros2.list
apt-get update -qq
apt-get install -y -qq \
  build-essential \
  ros-$DISTRO-ros-base \
  python3-colcon-common-extensions \
  python3-rosdep

# ── Install rosdep dependencies ──────────────────────────────────────────────
. /opt/ros/$DISTRO/setup.sh
rsync -a --exclude='build/' --exclude='install/' --exclude='log/' /src/mrpt_slam/ /workspace/
cd /workspace
rosdep init || true
rosdep update
rosdep install --from-paths . --ignore-src -r -y --as-root "apt:false"

# ── Install MRPT ROS packages (rosdep gap workaround) ───────────────────────
apt-get install -y -qq \
  xvfb \
  ros-${DISTRO}-mrpt-msgs \
  ros-${DISTRO}-mrpt-msgs-bridge \
  ros-${DISTRO}-mrpt-libgui \
  ros-${DISTRO}-mrpt-libslam \
  ros-${DISTRO}-mrpt-libros-bridge \
  ros-${DISTRO}-mrpt-libapps || true

# ── Build ────────────────────────────────────────────────────────────────────
. /opt/ros/$DISTRO/setup.sh
MAKEFLAGS='-j2' colcon build \
  --symlink-install \
  --parallel-workers 2 \
  --event-handlers console_direct+

# ── Test ─────────────────────────────────────────────────────────────────────
. /opt/ros/$DISTRO/setup.sh
. install/setup.sh
Xvfb :99 -screen 0 1024x768x24 &
export DISPLAY=:99
colcon test \
  --packages-select \
    mrpt_ekf_slam_2d mrpt_ekf_slam_3d \
    mrpt_icp_slam_2d mrpt_rbpf_slam mrpt_graphslam_2d \
  --event-handlers console_direct+
colcon test-result --verbose
"
