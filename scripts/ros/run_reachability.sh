#!/usr/bin/env bash
# scripts/ros/run_reachability.sh — headless reachability sweep runner (MOT-04.4).
#
# Launches crackvision_motion's mock_planning.launch.py (robot_state_publisher,
# ros2_control_node with mock_components/GenericSystem, controller spawners, move_group — no
# RViz), runs reachability_sweep against it, then shuts the whole launch process group down and
# exits with the sweep's exit code. reachability_sweep itself never commands a trajectory (see
# its module docstring and docs/motion/ROS_WORKSPACE.md); this script only adds
# launch/teardown around it.
#
# Usage:
#   scripts/ros/run_reachability.sh [--reachability-config P] [--out P] [--require-all-reachable]
#                                    [--max-duration-s N] [extra args forwarded to reachability_sweep]
#
# `set -m` (job control) makes bash put the backgrounded launch in its own process group instead
# of sharing this script's own pgid, exactly as test_mock_plan.sh does — required by
# _mock_stack.sh's start_mock_stack/stop_mock_stack.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# `source FILE` with no explicit args leaves the *caller's* positional parameters visible as
# env_ros.sh's own "$@" -- which env_ros.sh's own tail (`[ "$#" -gt 0 ] && exec "$@"`) then treats
# as "run this one command in the scrubbed env" and tries to exec our own --flags. Clear the
# positional parameters before sourcing it and restore them after.
SWEEP_ARGS=("$@")
set --
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
set -- "${SWEEP_ARGS[@]}"

OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "run_reachability.sh: $OVERLAY_SETUP not found — run scripts/ros/build_ws.sh first" >&2
    exit 3
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"

LOG_DIR="$REPO_ROOT/ros2_ws/log"
mkdir -p "$LOG_DIR"
LAUNCH_LOG="$LOG_DIR/reachability_launch.log"

if ! start_mock_stack "$LAUNCH_LOG"; then
    exit 1
fi

echo "run_reachability.sh: running reachability_sweep $*"
ros2 run crackvision_motion reachability_sweep "$@"
SWEEP_EXIT=$?
if [ "$SWEEP_EXIT" -ne 0 ]; then
    echo "run_reachability.sh: reachability_sweep exit=$SWEEP_EXIT; launch log tail follows:" >&2
    tail -n 60 "$LAUNCH_LOG" >&2
fi

echo "run_reachability.sh: reachability_sweep exit=$SWEEP_EXIT, shutting down mock stack"
exit "$SWEEP_EXIT"
