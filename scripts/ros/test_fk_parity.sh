#!/usr/bin/env bash
# scripts/ros/test_fk_parity.sh — FK parity: crackvision.kinematics vs live MoveIt /compute_fk
# (gripper_link, tool_tip, camera_link) on the mock stack (GEOM-08.3).
#
# Brings up the headless mock MoveIt stack, dumps MoveIt's own /compute_fk (read-only:
# fk_parity_dump.py never calls execute or trajectory actions and never touches
# controllers) for the all-zero joint state plus 20 seeded random states, tears the
# stack down and asserts no process from the launch's process group survives, then
# compares the dump against crackvision.kinematics + end_effector.yaml.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1

OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
[ -f "$OVERLAY_SETUP" ] || { echo "test_fk_parity.sh: run scripts/ros/build_ws.sh first" >&2; exit 3; }
set +u
# shellcheck disable=SC1090
source "$OVERLAY_SETUP"
set -u

# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"
mkdir -p "$REPO_ROOT/ros2_ws/log"
LAUNCH_LOG="$REPO_ROOT/ros2_ws/log/test_fk_parity_launch.log"
DUMP_JSON="$REPO_ROOT/ros2_ws/log/fk_parity.json"
start_mock_stack "$LAUNCH_LOG" || exit 1
PGID="$_MOCK_STACK_PGID"

python3 "$SCRIPT_DIR/fk_parity_dump.py" --out "$DUMP_JSON" --seed 0 --samples 20 \
    --limits "$REPO_ROOT/config/robot/b601_dm_limits.yaml"
DUMP_EXIT=$?

stop_mock_stack
trap - EXIT INT TERM
sleep 1
if [ -n "$PGID" ] && ps -eo pgid= | awk -v pg="$PGID" '$1 == pg {found=1} END {exit !found}'; then
    echo "test_fk_parity.sh: stray process(es) remain in launch pgid $PGID" >&2
    exit 1
fi

if [ "$DUMP_EXIT" -ne 0 ]; then
    echo "test_fk_parity.sh: fk_parity_dump.py exited $DUMP_EXIT; launch log tail:" >&2
    tail -n 40 "$LAUNCH_LOG" >&2
    exit "$DUMP_EXIT"
fi

"$REPO_ROOT/env.sh" python "$REPO_ROOT/scripts/geometry/compare_fk_parity.py" "$DUMP_JSON"
COMPARE_EXIT=$?

if [ "$COMPARE_EXIT" -ne 0 ]; then
    echo "test_fk_parity.sh: compare_fk_parity.py exited $COMPARE_EXIT" >&2
    exit "$COMPARE_EXIT"
fi
echo "test_fk_parity.sh: OK"
exit 0
