#!/usr/bin/env bash
# scripts/ros/run_cartesian_synthetic.sh — synthetic end-to-end Cartesian planning run (MOT-06.10).
#
# tools/synth_scene3d.py --> crackvision.paths --> crackvision.path3d --> plan_view +
# plan_crack_path on the headless mock stack, all under one scratch root
# logs/cartesian_synthetic/<stamp>/ (never data/ in the repo tree, never committed). Perception/
# geometry steps run through ./env.sh python; ROS steps run under scripts/ros/env_ros.sh. A
# scratch scene_config/1 is built from the generator's own ground truth (data/synthetic/
# {case}_gt.json) so the specimen top face/centre are exactly consistent with the synthetic
# crack surface. See docs/motion/CARTESIAN_PLANNING.md for the measured results and caveats.
#
# KNOWN CROSS-SYSTEM MISMATCH (see docs/motion/CARTESIAN_PLANNING.md "Caveats"): the real
# crackvision.paths3d/1 writer emits `component_id` as an int (docs/INTERFACES.md §3.13's own
# normative example: `"component_id":0`), but
# ros2_ws/.../crackvision_motion/cartesian_core.py's `_validate_paths3d` requires it to be a
# non-empty *string* — every existing cartesian_core test/fixture hand-writes string ids, so
# nothing previously exercised the real writer's output against the real loader. This script
# works around it by rewriting the generated paths3d file's `component_id` fields to strings
# in place before planning; it changes no contract and touches only this run's own scratch
# copy. A proper fix belongs to whichever of crackvision.path3d/cartesian_core turns out to be
# wrong, not to this script.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CASE_ID="synth_mot06"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ROOT="$REPO_ROOT/logs/cartesian_synthetic/$STAMP"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$ROOT" "$LOG_DIR"

PLANNING_CFG="$REPO_ROOT/config/motion/cartesian_planning.yaml"
SCENE_CFG="$ROOT/scene.yaml"
PATHS3D_JSON="$ROOT/data/paths3d/${CASE_ID}_paths3d.json"
VIEW_PLAN_OUT="$ROOT/view_plan.json"
CARTESIAN_PLAN_OUT="$ROOT/cartesian_plan.json"
LAUNCH_LOG="$LOG_DIR/run_cartesian_synthetic_launch.log"

fail() {
    echo "run_cartesian_synthetic.sh: FAIL: $*" >&2
    exit 1
}

echo "run_cartesian_synthetic.sh: scratch root $ROOT"

# 1. Synthetic scene (GEOM-08.8 generator) + the real crackvision.paths/path3d CLIs, same way
#    tests/test_path3d_synthetic.py exercises them.
T0=$(date +%s)
"$REPO_ROOT/env.sh" python "$REPO_ROOT/tools/synth_scene3d.py" --out "$ROOT" --seed 0 --case-id "$CASE_ID" \
    || fail "tools/synth_scene3d.py failed"
"$REPO_ROOT/env.sh" python -m crackvision.paths --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.paths failed"
"$REPO_ROOT/env.sh" python -m crackvision.path3d --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.path3d failed"
T1=$(date +%s)
[ -f "$PATHS3D_JSON" ] || fail "expected paths3d output missing: $PATHS3D_JSON"

# 2. Workaround for the component_id type mismatch described above (this run's own scratch
#    copy only), then a scratch scene_config/1 (table + specimen) built from the generator's
#    own ground truth so the specimen top face/centre match the synthetic surface exactly.
"$REPO_ROOT/env.sh" python - "$PATHS3D_JSON" "$ROOT/data/synthetic/${CASE_ID}_gt.json" "$SCENE_CFG" <<'PYEOF' \
    || fail "scratch scene/paths3d fixup failed"
import json
import sys

paths3d_path, gt_path, scene_path = sys.argv[1:4]

doc = json.load(open(paths3d_path, encoding="utf-8"))
for comp in doc["components"]:
    comp["component_id"] = str(comp["component_id"])
json.dump(doc, open(paths3d_path, "w", encoding="utf-8"), indent=2)

gt = json.load(open(gt_path, encoding="utf-8"))
center = gt["center_base_m"]
normal = gt["normal_base"]
if abs(normal[0]) > 1e-6 or abs(normal[1]) > 1e-6 or abs(normal[2] - 1.0) > 1e-6:
    sys.exit(
        f"run_cartesian_synthetic.sh: generator surface normal {normal} is not the flat "
        "+Z this script's axis-aligned scratch scene assumes (default --tilt-deg 0 expected)"
    )

cx, cy, cz = center
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
                "run_cartesian_synthetic.sh (MOT-06.10) scratch table: positioned so its top face "
                f"sits exactly at the synthetic specimen's bottom face (z={table_top_z:.6f} m), "
                "not a real workcell measurement."
            ),
        },
        {
            "id": "specimen",
            "shape": "box",
            "dimensions_m": [0.20, 0.20, specimen_thickness_m],
            "pose": {"frame": "base_link", "position_m": [cx, cy, specimen_center_z], "rpy_rad": [0.0, 0.0, 0.0]},
            "value_status": "nominal",
            "source": (
                "run_cartesian_synthetic.sh (MOT-06.10) scratch specimen: centre/top face taken "
                f"directly from tools/synth_scene3d.py's own ground truth ({gt_path}) "
                "center_base_m/normal_base, so this scene is consistent with the synthetic crack "
                "surface by construction, not a real measurement."
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
print(f"run_cartesian_synthetic.sh: scratch scene centre=({cx}, {cy}) specimen_top_z={specimen_top_z}")
PYEOF

# 3. The ROS planners resolve config/robot/end_effector.yaml and config/robot/b601_dm_limits.yaml
#    relative to --root; give the scratch root its own read-only copies (the production
#    config/scene/scene.yaml and config/motion/cartesian_planning.yaml are passed by absolute
#    path instead and are never copied/modified).
mkdir -p "$ROOT/config/robot"
cp "$REPO_ROOT/config/robot/end_effector.yaml" "$ROOT/config/robot/end_effector.yaml"
cp "$REPO_ROOT/config/robot/b601_dm_limits.yaml" "$ROOT/config/robot/b601_dm_limits.yaml"

# 4. ROS environment + mock stack.
set --
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "run_cartesian_synthetic.sh: $OVERLAY_SETUP not found — run scripts/ros/build_ws.sh first" >&2
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
node = rclpy.create_node("run_cartesian_synthetic_wait")
ok = all(node.create_client(t, n).wait_for_service(timeout_sec=120.0) for t, n in (
    (GetPositionFK, "/compute_fk"), (GetPositionIK, "/compute_ik"),
    (GetCartesianPath, "/compute_cartesian_path"), (GetMotionPlan, "/plan_kinematic_path"),
    (GetStateValidity, "/check_state_validity"), (GetPlanningScene, "/get_planning_scene")))
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PYEOF

# 5. plan_view, then plan_crack_path (--require-full-coverage: refuses rather than silently
#    reporting a partial trace as the headline result).
echo "run_cartesian_synthetic.sh: plan_view"
T2=$(date +%s)
ros2 run crackvision_motion plan_view --root "$ROOT" --scene-config "$SCENE_CFG" \
    --planning-config "$PLANNING_CFG" --out "$VIEW_PLAN_OUT"
rc=$?
[ "$rc" -eq 0 ] || fail "plan_view exit=$rc (expected 0)"
[ -f "$VIEW_PLAN_OUT" ] || fail "plan_view exit 0 but $VIEW_PLAN_OUT not written"
T3=$(date +%s)

echo "run_cartesian_synthetic.sh: plan_crack_path"
ros2 run crackvision_motion plan_crack_path --root "$ROOT" --paths3d "$PATHS3D_JSON" \
    --scene-config "$SCENE_CFG" --planning-config "$PLANNING_CFG" --out "$CARTESIAN_PLAN_OUT" \
    --require-full-coverage
rc=$?
if [ "$rc" -ne 0 ]; then
    fail "plan_crack_path exit=$rc (expected 0): the synthetic crack could not be fully covered " \
        "(--require-full-coverage refused); see $CARTESIAN_PLAN_OUT.log / above output for the reported reasons"
fi
[ -f "$CARTESIAN_PLAN_OUT" ] || fail "plan_crack_path exit 0 but $CARTESIAN_PLAN_OUT not written"
T4=$(date +%s)

stop_mock_stack

# 6. Validate both outputs with the real loaders (load_view_plan/load_plan), and assert the
#    card's required facts independently of the planner's own log lines.
echo "run_cartesian_synthetic.sh: validating outputs"
python3 - "$REPO_ROOT" "$ROOT" "$VIEW_PLAN_OUT" "$CARTESIAN_PLAN_OUT" "$PATHS3D_JSON" <<'PYEOF' || fail "output validation failed"
import json
import sys

sys.path.insert(0, f"{sys.argv[1]}/ros2_ws/src/crackvision_motion")
from crackvision_motion.cartesian_plan import load_plan, load_view_plan  # noqa: E402

repo_root, root, view_plan_path, plan_path, paths3d_path = sys.argv[1:6]
limits_path = f"{repo_root}/config/robot/b601_dm_limits.yaml"

view_plan = load_view_plan(view_plan_path, limits_path)
assert view_plan["chosen"] is not None, "view_plan: no candidate chosen"

plan = load_plan(plan_path, limits_path)
cov = plan["coverage"]
assert cov["complete"] is True, f"coverage.complete is not true: {cov}"
assert plan["sources"]["paths3d"]["execution_eligible"] is False, "sources.paths3d.execution_eligible is not false"

paths3d_doc = json.load(open(paths3d_path, encoding="utf-8"))
assert paths3d_doc["execution_eligible"] is False, "paths3d execution_eligible is not false"
for reason in ("wrist_camera_nominal", "tool_nominal", "optical_frames_nominal", "synthetic_capture"):
    assert reason in paths3d_doc["ineligible_reasons"], f"missing expected ineligible reason {reason}"

seg = plan["segments"][0]
print(
    f"run_cartesian_synthetic.sh: view candidates_tried={len(view_plan['candidates'])} "
    f"chosen_tilt_deg={view_plan['chosen']['tilt_deg']} chosen_yaw_deg={view_plan['chosen']['yaw_deg']}"
)
print(
    f"run_cartesian_synthetic.sh: cartesian candidates_tried={seg['candidates_tried']} "
    f"chosen={seg['chosen']} coverage={cov['trace_waypoints_planned']}/{cov['trace_waypoints_total']} "
    f"fraction_by_length={cov['fraction_by_length']:.6f}"
)
PYEOF

echo "run_cartesian_synthetic.sh: timing (s): synth+paths+path3d=$((T1 - T0)) plan_view=$((T3 - T2)) plan_crack_path=$((T4 - T3))"
echo "run_cartesian_synthetic.sh: OK ($ROOT)"
exit 0
