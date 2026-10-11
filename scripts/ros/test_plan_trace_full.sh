#!/usr/bin/env bash
# scripts/ros/test_plan_trace_full.sh — live full-trace smoke test for plan_crack_path (MOT-06.11,
# debugger D0010).
#
# Launches crackvision_motion's mock_planning.launch.py (via _mock_stack.sh, same pattern as
# test_reachability_task_frames.sh/run_reachability.sh) and plans the committed test-only fixture
# under ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full/: a single straight
# boresight-down segment of 0.06 m with 31 trace waypoints at 0.002 m, centred at the recommended
# placement (0.26, 0.0) on a surface at z 0.010 m, with a matching test-only scene. Planning uses
# the committed config/motion/cartesian_planning.yaml with no override. Asserts, from the written
# plan file, that the segment is `full` with trace_waypoints planned == total, every leg's recheck
# passes, and that the planner finished within 300 s. Tears the stack down on every exit path.
#
# `set -m` (job control): _mock_stack.sh's start_mock_stack/stop_mock_stack need the backgrounded
# launch in its own process group, separate from this script's own.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1

OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_plan_trace_full.sh: $OVERLAY_SETUP not found — run scripts/ros/build_ws.sh first" >&2
    exit 1
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"

FIX="$REPO_ROOT/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$LOG_DIR"
LAUNCH_LOG="$LOG_DIR/test_plan_trace_full_launch.log"
PLAN_OUT="$LOG_DIR/test_plan_trace_full_plan.json"
rm -f "$PLAN_OUT"

MAX_DURATION_S=300

if ! start_mock_stack "$LAUNCH_LOG"; then
    exit 1
fi

START_S=$(date +%s)
ros2 run crackvision_motion plan_crack_path \
    --paths3d "$FIX/paths3d.json" \
    --scene-config "$FIX/scene.yaml" \
    --out "$PLAN_OUT" \
    --root "$REPO_ROOT"
PLAN_EXIT=$?
END_S=$(date +%s)
DURATION_S=$((END_S - START_S))

if [ "$PLAN_EXIT" -ne 0 ]; then
    echo "test_plan_trace_full.sh: plan_crack_path exit=$PLAN_EXIT after ${DURATION_S}s; launch log tail follows:" >&2
    tail -n 60 "$LAUNCH_LOG" >&2
    exit "$PLAN_EXIT"
fi

echo "test_plan_trace_full.sh: plan_crack_path finished in ${DURATION_S}s (bound: ${MAX_DURATION_S}s)"
if [ "$DURATION_S" -gt "$MAX_DURATION_S" ]; then
    echo "test_plan_trace_full.sh: ${DURATION_S}s exceeds the ${MAX_DURATION_S}s bound" >&2
    exit 1
fi

python3 -I - "$PLAN_OUT" "$DURATION_S" "$MAX_DURATION_S" <<'PYEOF'
import json
import sys

plan_path, duration_s, max_duration_s = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
plan = json.loads(open(plan_path, encoding="utf-8").read())

assert duration_s <= max_duration_s, f"duration {duration_s}s exceeds bound {max_duration_s}s"

segments = plan["segments"]
assert len(segments) == 1, f"expected exactly one segment report, got {len(segments)}"
seg = segments[0]
assert seg["status"] == "full", f"segment status {seg['status']!r} (reasons={seg['reasons']}), expected 'full'"
assert seg["trace_waypoints"]["planned"] == seg["trace_waypoints"]["total"] == 31, seg["trace_waypoints"]
assert seg["reasons"] == [], seg["reasons"]

cfg = plan["parameters"]["recheck"]
tol_m, tol_deg_rad = cfg["fk_tolerance_m"], __import__("math").radians(cfg["fk_tolerance_deg"])
max_step = cfg["max_joint_step_rad"]
assert plan["legs"], "plan has no legs[]"
for leg in plan["legs"]:
    r = leg["recheck"]
    assert r["max_pos_err_m"] <= tol_m, (leg["kind"], r)
    assert r["max_ang_err_rad"] <= tol_deg_rad, (leg["kind"], r)
    assert r["all_valid"] is True, (leg["kind"], r)
    assert r["within_limits"] is True, (leg["kind"], r)
    assert r["max_joint_step_rad"] <= max_step, (leg["kind"], r)

cov = plan["coverage"]
assert cov["complete"] is True, cov
assert cov["trace_waypoints_planned"] == cov["trace_waypoints_total"] == 31, cov

print(f"test_plan_trace_full.sh: plan OK -- segment full, {len(plan['legs'])} legs, all recheck passed")
PYEOF
ASSERT_EXIT=$?
if [ "$ASSERT_EXIT" -ne 0 ]; then
    echo "test_plan_trace_full.sh: plan assertions failed" >&2
    exit 1
fi

echo "test_plan_trace_full.sh: OK"
exit 0
