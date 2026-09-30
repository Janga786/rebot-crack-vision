#!/usr/bin/env bash
# scripts/ros/test_end_effector.sh — end-of-arm model check (ADR-014 / GEOM-10).
#
# 1. Offline: validates config/robot/end_effector.yaml and renders the overlay URDF/SRDF with xacro
#    (ros2_ws/src/crackvision_description/test/test_end_effector.py).
# 2. Live: brings up the headless mock MoveIt stack (which now loads the overlay), runs
#    `check_end_effector` (read-only: /compute_fk, /compute_ik, /check_state_validity), tears the stack
#    down and asserts no process from the launch's process group survives.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1

python3 -m pytest "$REPO_ROOT/ros2_ws/src/crackvision_description/test/test_end_effector.py" -q -p no:cacheprovider || exit 1

OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
[ -f "$OVERLAY_SETUP" ] || { echo "test_end_effector.sh: run scripts/ros/build_ws.sh first" >&2; exit 3; }
set +u
# shellcheck disable=SC1090
source "$OVERLAY_SETUP"
set -u

# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"
mkdir -p "$REPO_ROOT/ros2_ws/log"
LAUNCH_LOG="$REPO_ROOT/ros2_ws/log/test_end_effector_launch.log"
start_mock_stack "$LAUNCH_LOG" || exit 1
PGID="$_MOCK_STACK_PGID"

ros2 run crackvision_motion check_end_effector --root "$REPO_ROOT"
CHECK_EXIT=$?

stop_mock_stack
trap - EXIT INT TERM
sleep 1
if [ -n "$PGID" ] && ps -eo pgid= | awk -v pg="$PGID" '$1 == pg {found=1} END {exit !found}'; then
    echo "test_end_effector.sh: stray process(es) remain in launch pgid $PGID" >&2
    exit 1
fi

if [ "$CHECK_EXIT" -ne 0 ]; then
    echo "test_end_effector.sh: check_end_effector exited $CHECK_EXIT; launch log tail:" >&2
    tail -n 40 "$LAUNCH_LOG" >&2
    exit "$CHECK_EXIT"
fi
echo "test_end_effector.sh: OK"
exit 0
