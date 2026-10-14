#!/usr/bin/env bash
# scripts/operator/plan.sh -- OPS-01.7 `plan` stage: path3d -> plan_crack_path -> retime_plan ->
# trajectory_check for exactly one case (docs/INTERFACES.md §19.1 row 5).
#
# A sequencing wrapper only: reuses the exact tools/flags already proven by
# scripts/ros/run_cartesian_synthetic.sh (path3d, plan_crack_path) and
# scripts/ros/test_retime_crosscheck.sh (retime_plan's ROS-free invocation, trajectory_check), and
# resolves every output at each tool's own default path under --root (data/paths3d/, data/plans/,
# data/trajectories/, and trajectory_check's own <trajectory>.check.json sidecar) -- no new paths,
# no execution step, no preview/approval step.
#
# Usage: scripts/operator/plan.sh --case-id CASE_ID [--root PATH] [--force]
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CASE_ID=""
ROOT="$REPO_ROOT"
FORCE=0

usage() {
    echo "usage: $0 --case-id CASE_ID [--root PATH] [--force]" >&2
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --case-id) CASE_ID="$2"; shift 2 ;;
        --root) ROOT="$2"; shift 2 ;;
        --force) FORCE=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "plan.sh: unknown argument: $1" >&2; usage; exit 2 ;;
    esac
done

if [ -z "$CASE_ID" ]; then
    echo "plan.sh: --case-id is required" >&2
    usage
    exit 2
fi

ROOT="$(cd "$ROOT" && pwd)"

fail() {
    echo "plan.sh: FAIL: $*" >&2
    exit 1
}

PATHS3D_JSON="$ROOT/data/paths3d/${CASE_ID}_paths3d.json"
CARTESIAN_PLAN="$ROOT/data/plans/${CASE_ID}_cartesian_plan.json"
TRAJECTORY="$ROOT/data/trajectories/${CASE_ID}_trajectory.json"
RETIME_REPORT="$TRAJECTORY.retime.json"
CHECK_REPORT="$TRAJECTORY.check.json"

# -- 0. Resumability + dependency precondition (§19.1/§19.3): `plan` depends on `capture` and
#    `paths` both being done and fresh for this case_id; never attempt a doomed run.
STATE_CHECK=$("$REPO_ROOT/env.sh" python - "$ROOT" "$CASE_ID" <<'PYEOF'
import sys
from crackvision import operator_state

root, case_id = sys.argv[1], sys.argv[2]
capture_fresh = operator_state.stage_is_fresh(case_id, "capture", root)
paths_fresh = operator_state.stage_is_fresh(case_id, "paths", root)
plan_fresh = operator_state.stage_is_fresh(case_id, "plan", root)
print(f"capture_fresh={capture_fresh}")
print(f"paths_fresh={paths_fresh}")
print(f"plan_fresh={plan_fresh}")
PYEOF
) || fail "could not evaluate operator_state freshness for case $CASE_ID"

CAPTURE_FRESH=$(echo "$STATE_CHECK" | grep '^capture_fresh=' | cut -d= -f2)
PATHS_FRESH=$(echo "$STATE_CHECK" | grep '^paths_fresh=' | cut -d= -f2)
PLAN_FRESH=$(echo "$STATE_CHECK" | grep '^plan_fresh=' | cut -d= -f2)

if [ "$CAPTURE_FRESH" != "True" ] || [ "$PATHS_FRESH" != "True" ]; then
    echo "plan.sh: precondition not met for case $CASE_ID: capture_fresh=$CAPTURE_FRESH paths_fresh=$PATHS_FRESH (run capture/paths first)" >&2
    exit 3
fi

if [ "$FORCE" -ne 1 ] && [ "$PLAN_FRESH" = "True" ]; then
    echo "plan.sh: skip -- run-state for case $CASE_ID stage plan is already fresh (use --force to re-run)"
    exit 0
fi

# -- 1. path3d (pure ./env.sh python, §19.5).
echo "plan.sh: path3d for case $CASE_ID"
"$REPO_ROOT/env.sh" python -m crackvision.path3d --root "$ROOT" --cases "$CASE_ID"
rc=$?
if [ "$rc" -ne 0 ]; then
    "$REPO_ROOT/env.sh" python -m crackvision.operator_state --root "$ROOT" reset --case "$CASE_ID" --stage plan >/dev/null 2>&1
    fail "crackvision.path3d exit=$rc (expected 0)"
fi
[ -f "$PATHS3D_JSON" ] || fail "path3d exit 0 but $PATHS3D_JSON not written"

# -- 2. plan_crack_path (ROS-sourced, §19.5) -- assumes a MoveIt graph (mock or real) is already
#    up; plan.sh never starts/stops a planning stack itself.
echo "plan.sh: plan_crack_path for case $CASE_ID"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../ros/env_ros.sh" || fail "env_ros.sh setup failed"
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
[ -f "$OVERLAY_SETUP" ] || fail "$OVERLAY_SETUP not found -- run scripts/ros/build_ws.sh first"
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

ros2 run crackvision_motion plan_crack_path --root "$ROOT" --paths3d "$PATHS3D_JSON"
rc=$?
if [ "$rc" -ne 0 ]; then
    "$REPO_ROOT/env.sh" python -m crackvision.operator_state --root "$ROOT" reset --case "$CASE_ID" --stage plan >/dev/null 2>&1
    fail "plan_crack_path exit=$rc (expected 0)"
fi
[ -f "$CARTESIAN_PLAN" ] || fail "plan_crack_path exit 0 but $CARTESIAN_PLAN not written"

# -- 3. retime_plan -- imports no rclpy/*_msgs type (§16.6), so it is run ROS-free: via the
#    console script if the overlay is installed, else the system python3 fallback against the
#    package sources directly (mirrors scripts/ros/test_retime_crosscheck.sh).
echo "plan.sh: retime_plan for case $CASE_ID"
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix crackvision_motion >/dev/null 2>&1; then
    ros2 run crackvision_motion retime_plan --plan "$CARTESIAN_PLAN" --root "$ROOT" --purpose crack_task
else
    PYTHONPATH="$REPO_ROOT/ros2_ws/src/crackvision_motion:${PYTHONPATH:-}" python3 \
        -m crackvision_motion.retime_plan --plan "$CARTESIAN_PLAN" --root "$ROOT" --purpose crack_task
fi
rc=$?
if [ "$rc" -ne 0 ]; then
    "$REPO_ROOT/env.sh" python -m crackvision.operator_state --root "$ROOT" reset --case "$CASE_ID" --stage plan >/dev/null 2>&1
    fail "retime_plan exit=$rc (expected 0)"
fi
[ -f "$TRAJECTORY" ] || fail "retime_plan exit 0 but $TRAJECTORY not written"
[ -f "$RETIME_REPORT" ] || fail "retime_plan exit 0 but $RETIME_REPORT not written"

# -- 4. trajectory_check (pure ./env.sh python, §19.5).
echo "plan.sh: trajectory_check for case $CASE_ID"
"$REPO_ROOT/env.sh" python -m crackvision.trajectory_check \
    --root "$ROOT" --trajectory "$TRAJECTORY" --plan "$CARTESIAN_PLAN" --report "$RETIME_REPORT"
rc=$?
if [ "$rc" -ne 0 ]; then
    "$REPO_ROOT/env.sh" python -m crackvision.operator_state --root "$ROOT" reset --case "$CASE_ID" --stage plan >/dev/null 2>&1
    fail "trajectory_check exit=$rc (expected 0)"
fi
[ -f "$CHECK_REPORT" ] || fail "trajectory_check exit 0 but $CHECK_REPORT not written"

# -- 5. Record the `plan` stage done, naming all four output files + their sha256s (§19.2).
"$REPO_ROOT/env.sh" python - "$ROOT" "$CASE_ID" "$CARTESIAN_PLAN" "$TRAJECTORY" "$RETIME_REPORT" "$CHECK_REPORT" <<'PYEOF' \
    || fail "could not record run-state for case $CASE_ID"
import sys
from crackvision import operator_state

root, case_id, cartesian_plan, trajectory, retime_report, check_report = sys.argv[1:7]
operator_state.mark_stage(
    case_id, "plan", "done",
    [cartesian_plan, trajectory, retime_report, check_report],
    root,
)
PYEOF

echo "plan.sh: OK -- case $CASE_ID plan stage done"
exit 0
