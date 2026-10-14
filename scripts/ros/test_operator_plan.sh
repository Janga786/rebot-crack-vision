#!/usr/bin/env bash
# scripts/ros/test_operator_plan.sh — OPS-01.7 headless test of `scripts/operator/plan.sh` against
# the mock planning stack, reusing MOT-06.10's synthetic scratch fixtures
# (tools/synth_scene3d.py) exactly as scripts/ros/run_cartesian_synthetic.sh does for a case's
# capture record + paths, but driven through plan.sh (path3d -> plan_crack_path -> retime_plan ->
# trajectory_check) instead of calling those four tools by hand.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CASE_ID="synth_opsplan"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ROOT="$REPO_ROOT/logs/operator_plan_synthetic/$STAMP"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$ROOT" "$LOG_DIR"

PLANNING_CFG="$REPO_ROOT/config/motion/cartesian_planning.yaml"
LAUNCH_LOG="$LOG_DIR/test_operator_plan_launch.log"

PATHS3D_JSON="$ROOT/data/paths3d/${CASE_ID}_paths3d.json"
PATHS_JSON="$ROOT/data/paths/${CASE_ID}_paths.json"
CAPTURE_RECORD="$ROOT/data/captures/${CASE_ID}_capture.json"
MASK_PNG="$ROOT/data/overlays/${CASE_ID}_mask.png"
CARTESIAN_PLAN="$ROOT/data/plans/${CASE_ID}_cartesian_plan.json"
TRAJECTORY="$ROOT/data/trajectories/${CASE_ID}_trajectory.json"
RETIME_REPORT="$TRAJECTORY.retime.json"
CHECK_REPORT="$TRAJECTORY.check.json"

fail() {
    echo "test_operator_plan.sh: FAIL: $*" >&2
    exit 1
}

echo "test_operator_plan.sh: scratch root $ROOT"

# 1. Synthetic scene (GEOM-08.8 generator, MOT-06.10) + the real crackvision.paths/path3d CLIs --
#    same fixture setup as scripts/ros/run_cartesian_synthetic.sh.
"$REPO_ROOT/env.sh" python "$REPO_ROOT/tools/synth_scene3d.py" --out "$ROOT" --seed 0 --case-id "$CASE_ID" \
    || fail "tools/synth_scene3d.py failed"
"$REPO_ROOT/env.sh" python -m crackvision.paths --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.paths failed"
[ -f "$CAPTURE_RECORD" ] || fail "expected synthetic capture record missing: $CAPTURE_RECORD"
[ -f "$PATHS_JSON" ] || fail "expected paths output missing: $PATHS_JSON"

# KNOWN CROSS-SYSTEM MISMATCH (see docs/motion/CARTESIAN_PLANNING.md "Caveats", also worked around
# by scripts/ros/run_cartesian_synthetic.sh): crackvision.paths writes `component_id` as an int,
# but crackvision_motion.cartesian_core's `_validate_paths3d` requires it to be a non-empty
# *string*; crackvision.path3d (which plan.sh calls internally) simply copies whatever type
# paths.json carries straight through to paths3d.json. Fixed up here, before plan.sh ever runs --
# this script's own scratch fixture copy only, not a change to path3d/plan_crack_path/cartesian_core.
"$REPO_ROOT/env.sh" python - "$PATHS_JSON" <<'PYEOF' || fail "paths.json component_id fixup failed"
import json
import sys

path = sys.argv[1]
doc = json.load(open(path, encoding="utf-8"))
for comp in doc["components"]:
    comp["component_id"] = str(comp["component_id"])
json.dump(doc, open(path, "w", encoding="utf-8"), indent=2)
PYEOF

# 2. A scratch scene_config/1 (table + specimen) built from the generator's own ground truth, at
#    the real production path under --root (config/scene/scene.yaml, plan_crack_path's own
#    default) so plan.sh -- which never accepts a --scene-config override of its own -- resolves
#    it without any new flag. Also give the scratch root its own copies of the rest of
#    config/motion + config/robot (real production files, read-only to this run) plus a symlink to
#    the (large) presentation/sim URDF tree, exactly scripts/ros/run_cartesian_synthetic.sh's own
#    scratch-root config copying for b601_dm_limits.yaml/end_effector.yaml.
mkdir -p "$ROOT/config/scene" "$ROOT/config/motion" "$ROOT/config/robot"
cp "$REPO_ROOT/config/motion/cartesian_planning.yaml" "$ROOT/config/motion/cartesian_planning.yaml"
cp "$REPO_ROOT/config/motion/retime.yaml" "$ROOT/config/motion/retime.yaml"
cp "$REPO_ROOT/config/motion/execution.yaml" "$ROOT/config/motion/execution.yaml"
cp "$REPO_ROOT/config/robot/end_effector.yaml" "$ROOT/config/robot/end_effector.yaml"
cp "$REPO_ROOT/config/robot/b601_dm_limits.yaml" "$ROOT/config/robot/b601_dm_limits.yaml"
[ -e "$ROOT/presentation" ] || ln -s "$REPO_ROOT/presentation" "$ROOT/presentation"

"$REPO_ROOT/env.sh" python - "$ROOT" "$CASE_ID" <<'PYEOF' \
    || fail "scratch scene build failed"
import json
import sys
from pathlib import Path

root, case_id = sys.argv[1:3]

gt_path = Path(root) / "data" / "synthetic" / f"{case_id}_gt.json"
gt = json.load(open(gt_path, encoding="utf-8"))
center = gt["center_base_m"]
normal = gt["normal_base"]
if abs(normal[0]) > 1e-6 or abs(normal[1]) > 1e-6 or abs(normal[2] - 1.0) > 1e-6:
    sys.exit(f"synthetic surface normal {normal} is not the flat +Z this scratch scene assumes")

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
            "id": "table", "shape": "box",
            "dimensions_m": [1.20, 1.00, table_thickness_m],
            "pose": {"frame": "base_link", "position_m": [cx, cy, table_center_z], "rpy_rad": [0.0, 0.0, 0.0]},
            "value_status": "nominal",
            "source": "test_operator_plan.sh (OPS-01.7) scratch table, not a real workcell measurement.",
        },
        {
            "id": "specimen", "shape": "box",
            "dimensions_m": [0.20, 0.20, specimen_thickness_m],
            "pose": {"frame": "base_link", "position_m": [cx, cy, specimen_center_z], "rpy_rad": [0.0, 0.0, 0.0]},
            "value_status": "nominal",
            "source": "test_operator_plan.sh (OPS-01.7) scratch specimen, centre/top face from the synthetic generator's own ground truth.",
        },
    ],
    "allowed_collisions": [
        {"link_a": "table", "link_b": "base_link",
         "reason": "the robot base is bolted to the table",
         "value_status": "nominal", "source": "Nominal mounting assumption, mirrors config/scene/scene.yaml."},
        {"link_a": "specimen", "link_b": "table",
         "reason": "the specimen rests directly on the table top",
         "value_status": "nominal", "source": "Nominal placement assumption, mirrors config/scene/scene.yaml."},
    ],
}
scene_path = Path(root) / "config" / "scene" / "scene.yaml"
json.dump(scene_doc, open(scene_path, "w", encoding="utf-8"), indent=2)
print(f"test_operator_plan.sh: scratch scene centre=({cx}, {cy}) specimen_top_z={specimen_top_z}")
PYEOF

# 3. Mark `capture`/`paths` done in crackvision.operator_state for this case, so plan.sh's own
#    precondition check (§19.1/§19.3) is satisfied without re-running the full segment/skeleton
#    pipeline this test does not need.
"$REPO_ROOT/env.sh" python - "$ROOT" "$CASE_ID" "$CAPTURE_RECORD" "$MASK_PNG" "$PATHS_JSON" <<'PYEOF' \
    || fail "could not seed operator_state capture/segment/paths stages"
import sys
from crackvision import operator_state

root, case_id, capture_record, mask_png, paths_json = sys.argv[1:6]
operator_state.mark_stage(case_id, "capture", "done", [capture_record], root)
operator_state.mark_stage(case_id, "segment", "done", [mask_png], root)
operator_state.mark_stage(case_id, "paths", "done", [paths_json], root)
PYEOF

# 4. ROS environment + mock stack (plan.sh assumes one is already up; it never starts/stops one
#    itself).
set --
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_operator_plan.sh: $OVERLAY_SETUP not found — run scripts/ros/build_ws.sh first" >&2
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
node = rclpy.create_node("test_operator_plan_wait")
ok = all(node.create_client(t, n).wait_for_service(timeout_sec=120.0) for t, n in (
    (GetPositionFK, "/compute_fk"), (GetPositionIK, "/compute_ik"),
    (GetCartesianPath, "/compute_cartesian_path"), (GetMotionPlan, "/plan_kinematic_path"),
    (GetStateValidity, "/check_state_validity"), (GetPlanningScene, "/get_planning_scene")))
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PYEOF

# 5. The real sequencer under test: scripts/operator/plan.sh. This leaves the ROS/overlay sourced
#    in this shell (harmless: plan.sh's own ./env.sh subprocess calls scrub their own environment
#    regardless of what the caller's shell has sourced).
echo "test_operator_plan.sh: == plan.sh --case-id $CASE_ID =="
"$REPO_ROOT/scripts/operator/plan.sh" --case-id "$CASE_ID" --root "$ROOT"
rc=$?

stop_mock_stack

[ "$rc" -eq 0 ] || fail "plan.sh exit=$rc (expected 0)"

for f in "$CARTESIAN_PLAN" "$TRAJECTORY" "$RETIME_REPORT" "$CHECK_REPORT"; do
    [ -f "$f" ] || fail "plan.sh exit 0 but expected output missing: $f"
done

# 6. Assert crackvision.operator_state reports `plan` as done, with all four output files
#    sha-bound, and that the final trajectory_check report has every check pass or skip.
"$REPO_ROOT/env.sh" python - "$ROOT" "$CASE_ID" "$CHECK_REPORT" <<'PYEOF' || fail "operator_state/trajectory_check assertion failed"
import json
import sys
from crackvision import operator_state

root, case_id, check_report_path = sys.argv[1:4]

assert operator_state.stage_is_fresh(case_id, "plan", root), "plan stage is not reported fresh"
state = operator_state.load(case_id, root)
entry = state["stages"]["plan"]
assert entry["status"] == "done", f"plan stage status={entry['status']!r}, expected done"
outputs = {o["path"] for o in entry["outputs"]}
expected_suffixes = (
    f"data/plans/{case_id}_cartesian_plan.json",
    f"data/trajectories/{case_id}_trajectory.json",
    f"data/trajectories/{case_id}_trajectory.json.retime.json",
    f"data/trajectories/{case_id}_trajectory.json.check.json",
)
for suffix in expected_suffixes:
    assert any(o.endswith(suffix) for o in outputs), f"missing recorded output ending {suffix}: {outputs}"
for o in entry["outputs"]:
    assert len(o["sha256"]) == 64, f"not sha-bound: {o}"

report = json.load(open(check_report_path, encoding="utf-8"))
for c in report["checks"]:
    outcome = c["outcome"]
    print(f"test_operator_plan.sh: check {c['check']}={outcome}")
    if outcome not in ("pass", "skip"):
        sys.exit(f"check {c['check']} outcome={outcome!r}, expected pass or skip")
print("test_operator_plan.sh: operator_state plan stage done, all outputs sha-bound, trajectory_check all pass/skip")
PYEOF

echo "test_operator_plan.sh: OK ($ROOT)"
exit 0
