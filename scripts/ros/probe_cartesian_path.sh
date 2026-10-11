#!/usr/bin/env bash
# scripts/ros/probe_cartesian_path.sh — live probe of /compute_cartesian_path and
# /plan_kinematic_path on the headless mock stack (MOT-06.1).
#
# Brings up the mock MoveIt stack (scripts/ros/mock_planning.launch.py), runs
# probe_cartesian_path.py (read-only except for a probe-local /apply_planning_scene; no action
# client, no execution), tears the stack down and asserts no process from the launch's process
# group survives, then exits with the probe's own exit code.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1

OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
[ -f "$OVERLAY_SETUP" ] || { echo "probe_cartesian_path.sh: run scripts/ros/build_ws.sh first" >&2; exit 3; }
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"
mkdir -p "$REPO_ROOT/ros2_ws/log" "$REPO_ROOT/logs"
LAUNCH_LOG="$REPO_ROOT/ros2_ws/log/probe_cartesian_path_launch.log"
OUT_JSON="$REPO_ROOT/logs/cartesian_probe.json"

start_mock_stack "$LAUNCH_LOG" || exit 1
PGID="$_MOCK_STACK_PGID"

python3 "$SCRIPT_DIR/probe_cartesian_path.py" --out "$OUT_JSON"
PROBE_EXIT=$?

stop_mock_stack
trap - EXIT INT TERM
sleep 1
if [ -n "$PGID" ] && ps -eo pgid= | awk -v pg="$PGID" '$1 == pg {found=1} END {exit !found}'; then
    echo "probe_cartesian_path.sh: stray process(es) remain in launch pgid $PGID" >&2
    exit 1
fi

if [ "$PROBE_EXIT" -ne 0 ]; then
    echo "probe_cartesian_path.sh: probe_cartesian_path.py exited $PROBE_EXIT; launch log tail:" >&2
    tail -n 40 "$LAUNCH_LOG" >&2
    exit "$PROBE_EXIT"
fi

echo "probe_cartesian_path.sh: OK, wrote $OUT_JSON"
exit 0
