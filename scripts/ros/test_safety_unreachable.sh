#!/usr/bin/env bash
# scripts/ros/test_safety_unreachable.sh — INT-03.3: unreachable-path failure-injection test
# (mock-stack planning).
#
# Reuses the synthetic-case + moveit_mock-profile bring-up pattern from
# run_cartesian_synthetic.sh/run_integration_e2e.sh (tools/synth_scene3d.py -->
# crackvision.paths --> crackvision.path3d, a scratch scene_config/1 built from the
# generator's own ground truth, the headless mock MoveIt stack via _mock_stack.sh). Unlike
# those scripts, the generated specimen/crack is then translated +5.0 m along base_link's X
# axis -- far outside docs/motion/REACHABILITY.md's documented envelope (its own sweep only
# covers x in [0.11, 0.50] m, and its own `reach_bound_from_urdf` prefilter bound is 0.962 m
# from base_link, §2/§8.4) -- so every IK candidate for every segment must fail.
#
# `ros2 run crackvision_motion plan_view` and `plan_crack_path` are then both run against
# this translated case and are asserted to refuse cleanly: exit `EXIT_PRECONDITION` (3, the
# code's own contract for "no candidate selected" / "zero trace waypoints planned",
# cli_common.py's PreconditionError -> EXIT_PRECONDITION mapping, read directly from
# plan_view.py/plan_crack_path.py/cli_common.py before writing this script), no
# view_plan.json/cartesian_plan.json written, no unhandled exception (run_cli's own
# catch-all logs any unexpected exception and maps it to exit 1 -- it never lets one escape
# as a raw traceback -- so this script additionally fails itself if either tool exits 1,
# since that would mean something besides the expected "no candidate" precondition went
# wrong), and bounded by the planner's own documented budget computed straight from
# config/motion/cartesian_planning.yaml (service_timeout_s, candidate count, max splits) --
# never an arbitrary wrapper timeout.
#
# No hardware, no real mode: only the headless moveit_mock stack (scripts/ros/_mock_stack.sh)
# is ever started. Scratch space only: everything lives under
# logs/safety_unreachable/<stamp>/; nothing under data/ or config/ in the repo tree is
# written or modified.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CASE_ID="synth_unreachable"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ROOT="$REPO_ROOT/logs/safety_unreachable/$STAMP"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$ROOT" "$LOG_DIR"

PLANNING_CFG="$REPO_ROOT/config/motion/cartesian_planning.yaml"
SCENE_CFG="$ROOT/scene.yaml"
PATHS3D_JSON="$ROOT/data/paths3d/${CASE_ID}_paths3d.json"
VIEW_PLAN_OUT="$ROOT/view_plan.json"
CARTESIAN_PLAN_OUT="$ROOT/cartesian_plan.json"
LAUNCH_LOG="$LOG_DIR/test_safety_unreachable_launch.log"

# Translation applied to every base_link position this script writes (specimen/table centre,
# every trace waypoint's p_base_m): the full sweep's own grid tops out at x=0.50 m and its own
# reach_bound_from_urdf prefilter is 0.962 m (docs/motion/REACHABILITY.md §2/§8), so +5.0 m
# puts every point ~4.5x past even that generous prefilter bound.
DX_M=5.0

fail() {
    echo "test_safety_unreachable.sh: FAIL: $*" >&2
    exit 1
}

echo "test_safety_unreachable.sh: scratch root $ROOT (translation dx=+${DX_M}m)"

# 1. Synthetic scene + the real crackvision.paths/path3d CLIs (same as run_cartesian_synthetic.sh).
"$REPO_ROOT/env.sh" python "$REPO_ROOT/tools/synth_scene3d.py" --out "$ROOT" --seed 0 --case-id "$CASE_ID" \
    || fail "tools/synth_scene3d.py failed"
"$REPO_ROOT/env.sh" python -m crackvision.paths --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.paths failed"
"$REPO_ROOT/env.sh" python -m crackvision.path3d --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.path3d failed"
[ -f "$PATHS3D_JSON" ] || fail "expected paths3d output missing: $PATHS3D_JSON"

# 2. Fixup (component_id int->string, same workaround as run_cartesian_synthetic.sh) AND the
#    +DX_M translation of every trace waypoint's p_base_m plus the scratch scene's
#    table/specimen centre, built from the generator's own ground truth shifted by the same
#    delta so scene and crack stay geometrically consistent (this test must fail on
#    unreachability, not on an unrelated surface/scene mismatch).
"$REPO_ROOT/env.sh" python - "$PATHS3D_JSON" "$ROOT/data/synthetic/${CASE_ID}_gt.json" "$SCENE_CFG" "$DX_M" <<'PYEOF' \
    || fail "scratch scene/paths3d fixup+translation failed"
import json
import sys

paths3d_path, gt_path, scene_path, dx_s = sys.argv[1:5]
dx = float(dx_s)

doc = json.load(open(paths3d_path, encoding="utf-8"))
for comp in doc["components"]:
    comp["component_id"] = str(comp["component_id"])
    for polyline in comp["polylines"]:
        for pt in polyline["points"]:
            if pt.get("p_base_m") is not None:
                p = pt["p_base_m"]
                pt["p_base_m"] = [p[0] + dx, p[1], p[2]]
        # segment_tasks() (cartesian_core.py) reads position_m/surface_point_m off
        # segments[].waypoints[], a separately-derived structure from polylines[].points[] --
        # both must be translated or the planner would still target the original,
        # reachable pose.
        for segment in polyline["segments"]:
            for wp in segment["waypoints"]:
                pm = wp["position_m"]
                wp["position_m"] = [pm[0] + dx, pm[1], pm[2]]
                sp = wp["surface_point_m"]
                wp["surface_point_m"] = [sp[0] + dx, sp[1], sp[2]]
json.dump(doc, open(paths3d_path, "w", encoding="utf-8"), indent=2)

gt = json.load(open(gt_path, encoding="utf-8"))
center = gt["center_base_m"]
normal = gt["normal_base"]
if abs(normal[0]) > 1e-6 or abs(normal[1]) > 1e-6 or abs(normal[2] - 1.0) > 1e-6:
    sys.exit(
        f"test_safety_unreachable.sh: generator surface normal {normal} is not the flat "
        "+Z this script's axis-aligned scratch scene assumes (default --tilt-deg 0 expected)"
    )

cx, cy, cz = center
cx += dx
specimen_thickness_m = 0.01
table_thickness_m = 0.05
specimen_top_z = cz
specimen_center_z = specimen_top_z - specimen_thickness_m / 2.0
table_top_z = specimen_center_z - specimen_thickness_m / 2.0
table_center_z = table_top_z - table_thickness_m / 2.0

scene_doc = {
    "schema": "crackvision.scene_config/1",
    "frame": "base_link",
    "objects": [
        {
            "id": "table",
            "shape": "box",
            "dimensions_m": [1.20, 1.00, table_thickness_m],
            "pose": {"frame": "base_link", "position_m": [cx, cy, table_center_z], "rpy_rad": [0.0, 0.0, 0.0]},
            "value_status": "nominal",
            "source": (
                "test_safety_unreachable.sh (INT-03.3) scratch table: positioned so its top face "
                f"sits exactly at the translated synthetic specimen's bottom face "
                f"(z={table_top_z:.6f} m), translated +{dx} m in x so it sits outside the "
                "reachability envelope, not a real workcell measurement."
            ),
        },
        {
            "id": "specimen",
            "shape": "box",
            "dimensions_m": [0.20, 0.20, specimen_thickness_m],
            "pose": {"frame": "base_link", "position_m": [cx, cy, specimen_center_z], "rpy_rad": [0.0, 0.0, 0.0]},
            "value_status": "nominal",
            "source": (
                "test_safety_unreachable.sh (INT-03.3) scratch specimen: centre/top face taken "
                f"from tools/synth_scene3d.py's own ground truth ({gt_path}) center_base_m/"
                f"normal_base, translated +{dx} m in x so it sits well outside "
                "docs/motion/REACHABILITY.md's documented reachability envelope."
            ),
        },
    ],
    "allowed_collisions": [
        {
            "link_a": "table", "link_b": "base_link",
            "reason": "the robot base is bolted to the table, so the table top and the base necessarily touch/overlap",
            "value_status": "nominal", "source": "Nominal mounting assumption, mirrors config/scene/scene.yaml.",
        },
        {
            "link_a": "specimen", "link_b": "table",
            "reason": "the specimen rests directly on the table top",
            "value_status": "nominal", "source": "Nominal placement assumption, mirrors config/scene/scene.yaml.",
        },
    ],
}
json.dump(scene_doc, open(scene_path, "w", encoding="utf-8"), indent=2)
print(f"test_safety_unreachable.sh: translated scratch scene centre=({cx}, {cy}) specimen_top_z={specimen_top_z}")
PYEOF

# 3. Scratch config/robot copies (planners resolve these relative to --root).
mkdir -p "$ROOT/config/robot"
cp "$REPO_ROOT/config/robot/end_effector.yaml" "$ROOT/config/robot/end_effector.yaml"
cp "$REPO_ROOT/config/robot/b601_dm_limits.yaml" "$ROOT/config/robot/b601_dm_limits.yaml"

# 4. ROS environment + mock stack.
set --
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_safety_unreachable.sh: $OVERLAY_SETUP not found — run scripts/ros/build_ws.sh first" >&2
    exit 3
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"

start_mock_stack "$LAUNCH_LOG" || exit 1
python3 - <<'PYEOF' || fail "MoveIt services did not come up; see $LAUNCH_LOG"
import sys, rclpy
from moveit_msgs.srv import (GetCartesianPath, GetMotionPlan, GetPlanningScene, GetPositionFK,
                             GetPositionIK, GetStateValidity)
rclpy.init()
node = rclpy.create_node("test_safety_unreachable_wait")
ok = all(node.create_client(t, n).wait_for_service(timeout_sec=120.0) for t, n in (
    (GetPositionFK, "/compute_fk"), (GetPositionIK, "/compute_ik"),
    (GetCartesianPath, "/compute_cartesian_path"), (GetMotionPlan, "/plan_kinematic_path"),
    (GetStateValidity, "/check_state_validity"), (GetPlanningScene, "/get_planning_scene")))
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PYEOF

# 5. Compute the planner's own documented time budget from config/motion/cartesian_planning.yaml
#    (never an arbitrary wrapper timeout): service_timeout_s is the per-call wait on every
#    MoveIt service call (ik_solver_timeout_s is sub-budget of ik's own service_timeout_s
#    wait, not additional). plan_view tries 1 view candidate set (tilt x yaw); plan_crack_path
#    tries up to (max_splits_per_segment + 1) rounds of the full (direction x roll) candidate
#    set per segment, each candidate able to issue a handful of service calls (ik, fk,
#    cartesian, validity x N waypoints, motion_plan) -- bounding by
#    "calls_per_candidate x service_timeout_s" per candidate is already generous since a
#    failing candidate normally returns in well under one service_timeout_s.
BUDGET_S="$(python3 -c '
import yaml
cfg = yaml.safe_load(open("'"$PLANNING_CFG"'", encoding="utf-8"))
svc = float(cfg["service_timeout_s"])
n_roll = len(cfg["candidates"]["roll_offsets_deg"])
n_cand = n_roll * 2  # 2 directions, cartesian_core.candidates()
n_tilt = len(cfg["view"]["tilt_candidates_deg"])
n_yaw = len(cfg["view"]["yaw_candidates_deg"])
calls_per_candidate = 6
rounds = cfg["splitting"]["max_splits_per_segment"] + 1
view_budget = n_tilt * n_yaw * calls_per_candidate * svc
cartesian_budget = rounds * n_cand * calls_per_candidate * svc
print(f"{view_budget:.1f} {cartesian_budget:.1f}")
')"
VIEW_BUDGET_S="$(echo "$BUDGET_S" | cut -d" " -f1)"
CARTESIAN_BUDGET_S="$(echo "$BUDGET_S" | cut -d" " -f2)"
echo "test_safety_unreachable.sh: documented budgets: plan_view<=${VIEW_BUDGET_S}s plan_crack_path<=${CARTESIAN_BUDGET_S}s"

# 6. plan_view against the translated scene -- expect the documented "no candidate selected"
#    refusal (PreconditionError -> EXIT_PRECONDITION=3, read directly from plan_view.py's
#    `_run`: `if chosen is None: raise PreconditionError(...)` before `write_view_plan` is
#    ever called), within its documented budget, no view_plan.json written.
echo "test_safety_unreachable.sh: == plan_view (expect refusal) =="
VIEW_LOG="$ROOT/plan_view.log"
T0=$(date +%s)
ros2 run crackvision_motion plan_view --root "$ROOT" --scene-config "$SCENE_CFG" \
    --planning-config "$PLANNING_CFG" --out "$VIEW_PLAN_OUT" --center "5.26" "0.0" >"$VIEW_LOG" 2>&1
rc=$?
T1=$(date +%s)
cat "$VIEW_LOG"
VIEW_ELAPSED=$((T1 - T0))
echo "test_safety_unreachable.sh: plan_view exit=$rc elapsed=${VIEW_ELAPSED}s"
[ "$rc" -eq 3 ] || fail "plan_view exit=$rc, expected 3 (EXIT_PRECONDITION, cli_common.py's PreconditionError mapping)"
[ ! -f "$VIEW_PLAN_OUT" ] || fail "plan_view exit 3 but $VIEW_PLAN_OUT was written -- a precondition refusal must never leave a plan file behind"
grep -q "no view candidate selected" "$VIEW_LOG" || fail "plan_view log does not mention the expected §13.14 Selection refusal; see $VIEW_LOG"
python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) <= float(sys.argv[2]) else 1)" "$VIEW_ELAPSED" "$VIEW_BUDGET_S" \
    || fail "plan_view took ${VIEW_ELAPSED}s, exceeding its own documented budget of ${VIEW_BUDGET_S}s"

# 7. plan_crack_path against the translated paths3d/scene -- expect the documented "zero
#    trace waypoints planned" refusal (same PreconditionError -> EXIT_PRECONDITION=3,
#    plan_crack_path.py's `_run`: the check happens after `finalize(...)`, strictly before
#    `write_plan` is ever called), within its documented budget, no cartesian_plan.json
#    written.
echo "test_safety_unreachable.sh: == plan_crack_path (expect refusal) =="
CARTESIAN_LOG="$ROOT/plan_crack_path.log"
T2=$(date +%s)
ros2 run crackvision_motion plan_crack_path --root "$ROOT" --paths3d "$PATHS3D_JSON" \
    --scene-config "$SCENE_CFG" --planning-config "$PLANNING_CFG" --out "$CARTESIAN_PLAN_OUT" \
    --require-full-coverage >"$CARTESIAN_LOG" 2>&1
rc=$?
T3=$(date +%s)
cat "$CARTESIAN_LOG"
CARTESIAN_ELAPSED=$((T3 - T2))
echo "test_safety_unreachable.sh: plan_crack_path exit=$rc elapsed=${CARTESIAN_ELAPSED}s"
[ "$rc" -eq 3 ] || fail "plan_crack_path exit=$rc, expected 3 (EXIT_PRECONDITION, cli_common.py's PreconditionError mapping)"
[ ! -f "$CARTESIAN_PLAN_OUT" ] || fail "plan_crack_path exit 3 but $CARTESIAN_PLAN_OUT was written -- a precondition refusal must never leave a plan file behind"
grep -q "zero trace waypoints planned" "$CARTESIAN_LOG" || fail "plan_crack_path log does not mention the expected zero-trace-waypoints refusal; see $CARTESIAN_LOG"
python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) <= float(sys.argv[2]) else 1)" "$CARTESIAN_ELAPSED" "$CARTESIAN_BUDGET_S" \
    || fail "plan_crack_path took ${CARTESIAN_ELAPSED}s, exceeding its own documented budget of ${CARTESIAN_BUDGET_S}s"

stop_mock_stack

if grep -qi "unhandled exception\|Traceback (most recent call last)" "$VIEW_LOG" "$CARTESIAN_LOG"; then
    fail "an unhandled exception/traceback was logged; see $VIEW_LOG / $CARTESIAN_LOG"
fi

echo "test_safety_unreachable.sh: OK ($ROOT)"
exit 0
