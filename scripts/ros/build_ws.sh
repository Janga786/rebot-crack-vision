#!/usr/bin/env bash
# scripts/ros/build_ws.sh — colcon-build the crackvision ROS 2 overlay (MOT-02).
#
# Builds ros2_ws/src on top of the ~/rebot_ws/install underlay (read-only) and ROS Humble.
# build/, install/ and log/ under ros2_ws are git-ignored — see .gitignore.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh"

cd "$REPO_ROOT/ros2_ws"
exec colcon build --symlink-install --event-handlers console_direct+
