#!/usr/bin/env bash
# scripts/ros/test_mock_plan.sh — headless MoveIt mock-planning smoke test (MOT-02).
#
# Launches crackvision_motion's mock_planning.launch.py (robot_state_publisher,
# ros2_control_node with mock_components/GenericSystem, controller spawners, move_group
# — no RViz), waits for the /move_action action server, runs plan_joint_goal (plan-only,
# never executes), then shuts the whole launch process group down and exits with
# plan_joint_goal's exit code. No robot motion is ever commanded.
# Job control (`set -m`) makes bash put each backgrounded pipeline in its own process
# group instead of sharing this script's own pgid — without it, signalling "the launch's
# process group" for cleanup would also signal this script itself.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1

OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_mock_plan.sh: $OVERLAY_SETUP not found — run scripts/ros/build_ws.sh first" >&2
    exit 1
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

LOG_DIR="$REPO_ROOT/ros2_ws/log"
mkdir -p "$LOG_DIR"
LAUNCH_LOG="$LOG_DIR/mock_plan_launch.log"

LAUNCH_PID=""
LAUNCH_PGID=""

cleanup() {
    if [ -n "$LAUNCH_PGID" ]; then
        kill -INT -- "-$LAUNCH_PGID" 2>/dev/null || true
        for _ in $(seq 1 10); do
            if ! kill -0 -- "-$LAUNCH_PGID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        if kill -0 -- "-$LAUNCH_PGID" 2>/dev/null; then
            echo "test_mock_plan.sh: process group $LAUNCH_PGID still alive after SIGINT, sending SIGKILL" >&2
            kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null || true
            sleep 1
        fi
    fi
}
trap cleanup EXIT INT TERM

ros2 launch crackvision_motion mock_planning.launch.py > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
sleep 1
LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')
if [ -z "$LAUNCH_PGID" ]; then
    echo "test_mock_plan.sh: launch process $LAUNCH_PID exited immediately; log follows:" >&2
    cat "$LAUNCH_LOG" >&2
    exit 1
fi

sleep 3
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "test_mock_plan.sh: launch process died on startup; log follows:" >&2
    cat "$LAUNCH_LOG" >&2
    exit 1
fi

# plan_joint_goal itself waits (up to SERVER_WAIT_TIMEOUT_S) for the /move_action action
# server via rclpy's own graph discovery, so there is no separate CLI-side readiness poll
# here (the `ros2 action list`/daemon path is a slower, less reliable way to ask the same
# question).
echo "test_mock_plan.sh: launch pid=$LAUNCH_PID pgid=$LAUNCH_PGID up, running plan_joint_goal (plan-only, no execution)"
ros2 run crackvision_motion plan_joint_goal
PLAN_EXIT=$?
if [ "$PLAN_EXIT" -ne 0 ]; then
    echo "test_mock_plan.sh: plan_joint_goal failed; launch log follows:" >&2
    cat "$LAUNCH_LOG" >&2
fi

echo "test_mock_plan.sh: plan_joint_goal exit=$PLAN_EXIT, shutting down mock stack"
exit "$PLAN_EXIT"
