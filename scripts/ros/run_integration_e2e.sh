#!/usr/bin/env bash
# scripts/ros/run_integration_e2e.sh — INT-01.2: one command chaining the real
# synthetic-perception chain (INT-01.1, scripts/perc/run_synthetic_perception.sh) into
# MoveIt-mock crack planning, retiming and a limits-checked mock execution.
#
# synth_scene3d.py --> visualize --> skeleton --> paths --> visualize_paths --> path3d
# (all via run_synthetic_perception.sh, unmodified) --> plan_view --> plan_crack_path
# (--require-full-coverage) --> retime_plan --> crackvision.trajectory_check -->
# execute_trajectory --mode dry --> execute_trajectory --mode mock, all on the headless
# moveit_mock stack. Never the GT-skeleton-bypass front end run_cartesian_synthetic.sh
# (MOT-06.10) builds for its own purposes -- this script reuses only the downstream
# scene/retime/execute patterns from run_cartesian_synthetic.sh and test_retime_execute.sh
# (MOT-07.9), copied verbatim rather than modifying either file.
#
# No hardware, no vendor driver, no real mode, CRACKVISION_ARM_REAL never set: only the
# headless ros2_control mock hardware stack (scripts/ros/_mock_stack.sh) is ever started,
# which installs its own `trap stop_mock_stack EXIT INT TERM`, so the stack is torn down on
# every exit path, including a failure partway through.
#
# Scratch space only: everything this script writes lives under
# logs/integration_e2e/<stamp>/ plus the logs/synthetic_perception/<stamp>/ scratch root
# produced by run_synthetic_perception.sh; nothing under data/ or config/ in the repo tree
# is ever written or modified.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
WORK="$REPO_ROOT/logs/integration_e2e/$STAMP"
RECORD_DIR="$WORK/execution_records"
mkdir -p "$WORK" "$RECORD_DIR"

fail() {
    echo "run_integration_e2e.sh: FAIL: $*" >&2
    exit 1
}

# -- guard: never run anywhere near a real vendor driver process (§0's truncated-comm
# pattern, mirroring test_execution.sh / test_retime_execute.sh).
if pgrep -x reBotArmControl >/dev/null 2>&1; then
    echo "run_integration_e2e.sh: a real vendor driver process (reBotArmControl) is running -- refusing to run" >&2
    exit 3
fi

# --------------------------------------------------------------------------------------
# 1. Front end: the real segmentation-to-paths-to-3D chain (INT-01.1), unmodified.
# --------------------------------------------------------------------------------------

echo "run_integration_e2e.sh: == step 1: run_synthetic_perception.sh =="
PERC_LOG="$WORK/run_synthetic_perception.log"
bash "$REPO_ROOT/scripts/perc/run_synthetic_perception.sh" >"$PERC_LOG" 2>&1
rc=$?
cat "$PERC_LOG"
[ "$rc" -eq 0 ] || fail "run_synthetic_perception.sh exit=$rc (expected 0); see $PERC_LOG"

PERC_ROOT="$(sed -n 's/^run_synthetic_perception.sh: scratch root //p' "$PERC_LOG" | head -n1)"
[ -n "$PERC_ROOT" ] && [ -d "$PERC_ROOT" ] || fail "could not recover the scratch root from $PERC_LOG"

PATHS3D_SRC="$(ls "$PERC_ROOT"/data/paths3d/*_paths3d.json 2>/dev/null | head -n1)"
[ -n "$PATHS3D_SRC" ] || fail "no paths3d output found under $PERC_ROOT/data/paths3d"
CASE_ID="$(basename "$PATHS3D_SRC" _paths3d.json)"
GT_JSON="$PERC_ROOT/data/synthetic/${CASE_ID}_gt.json"
[ -f "$GT_JSON" ] || fail "expected ground-truth file missing: $GT_JSON"
echo "run_integration_e2e.sh: perception scratch root $PERC_ROOT (case=$CASE_ID)"

# --------------------------------------------------------------------------------------
# 2. Scratch scene_config/1 (table + specimen) from the generator's own ground truth, same
#    pattern as run_cartesian_synthetic.sh step 2 -- config/scene/scene.yaml is never read.
#    The paths3d output is copied into this script's own work dir first (never mutating
#    run_synthetic_perception.sh's own scratch root), then the same component_id
#    int->string workaround is applied to that copy only.
# --------------------------------------------------------------------------------------

SCENE_CFG="$WORK/scene.yaml"
PATHS3D_JSON="$WORK/data/paths3d/${CASE_ID}_paths3d.json"
mkdir -p "$(dirname "$PATHS3D_JSON")"

# plan_crack_path resolves the paths3d doc's sources.capture_record relative to --root (WORK)
# and sha-checks it (§13.1/§13.15); copy that one file verbatim to the same relative path so
# it resolves under WORK exactly as it does under PERC_ROOT, without altering its bytes/sha.
CAPTURE_REL="$(python3 -c '
import json, sys
print(json.load(open(sys.argv[1], encoding="utf-8"))["sources"]["capture_record"])
' "$PATHS3D_SRC")"
mkdir -p "$WORK/$(dirname "$CAPTURE_REL")"
cp "$PERC_ROOT/$CAPTURE_REL" "$WORK/$CAPTURE_REL"

"$REPO_ROOT/env.sh" python - "$PATHS3D_SRC" "$PATHS3D_JSON" "$GT_JSON" "$SCENE_CFG" <<'PYEOF' \
    || fail "scratch scene/paths3d fixup failed"
import json
import sys

paths3d_src, paths3d_dst, gt_path, scene_path = sys.argv[1:5]

doc = json.load(open(paths3d_src, encoding="utf-8"))
for comp in doc["components"]:
    comp["component_id"] = str(comp["component_id"])
json.dump(doc, open(paths3d_dst, "w", encoding="utf-8"), indent=2)

gt = json.load(open(gt_path, encoding="utf-8"))
center = gt["center_base_m"]
normal = gt["normal_base"]
if abs(normal[0]) > 1e-6 or abs(normal[1]) > 1e-6 or abs(normal[2] - 1.0) > 1e-6:
    sys.exit(
        f"run_integration_e2e.sh: generator surface normal {normal} is not the flat "
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
                "run_integration_e2e.sh (INT-01.2) scratch table: positioned so its top face "
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
                "run_integration_e2e.sh (INT-01.2) scratch specimen: centre/top face taken "
                f"directly from run_synthetic_perception.sh's own generator ground truth "
                f"({gt_path}) center_base_m/normal_base, so this scene is consistent with the "
                "synthetic crack surface by construction, not a real measurement."
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
print(f"run_integration_e2e.sh: scratch scene centre=({cx}, {cy}) specimen_top_z={specimen_top_z}")
PYEOF

# --------------------------------------------------------------------------------------
# 3. Scratch config/robot copies (the ROS planners resolve these relative to --root; the
#    production config/scene/scene.yaml and config/motion/cartesian_planning.yaml are
#    passed by absolute path instead and are never copied/modified).
# --------------------------------------------------------------------------------------

mkdir -p "$WORK/config/robot"
cp "$REPO_ROOT/config/robot/end_effector.yaml" "$WORK/config/robot/end_effector.yaml"
cp "$REPO_ROOT/config/robot/b601_dm_limits.yaml" "$WORK/config/robot/b601_dm_limits.yaml"
SCRATCH_EE="$WORK/config/robot/end_effector.yaml"
SCRATCH_LIMITS="$WORK/config/robot/b601_dm_limits.yaml"

PLANNING_CFG="$REPO_ROOT/config/motion/cartesian_planning.yaml"
VIEW_PLAN_OUT="$WORK/view_plan.json"
CARTESIAN_PLAN_OUT="$WORK/cartesian_plan.json"

# --------------------------------------------------------------------------------------
# 4. ROS environment + mock stack (planning leg).
# --------------------------------------------------------------------------------------

set --
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "run_integration_e2e.sh: $OVERLAY_SETUP not found — run scripts/ros/build_ws.sh first" >&2
    exit 3
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"

PLAN_LAUNCH_LOG="$WORK/mock_planning_launch.log"
start_mock_stack "$PLAN_LAUNCH_LOG" || exit 1
python3 - <<'PYEOF' || fail "MoveIt services did not come up; see $PLAN_LAUNCH_LOG"
import sys, rclpy
from moveit_msgs.srv import (GetCartesianPath, GetMotionPlan, GetPlanningScene, GetPositionFK,
                             GetPositionIK, GetStateValidity)
rclpy.init()
node = rclpy.create_node("run_integration_e2e_wait")
ok = all(node.create_client(t, n).wait_for_service(timeout_sec=120.0) for t, n in (
    (GetPositionFK, "/compute_fk"), (GetPositionIK, "/compute_ik"),
    (GetCartesianPath, "/compute_cartesian_path"), (GetMotionPlan, "/plan_kinematic_path"),
    (GetStateValidity, "/check_state_validity"), (GetPlanningScene, "/get_planning_scene")))
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PYEOF

# --------------------------------------------------------------------------------------
# 5. plan_view, then plan_crack_path (--require-full-coverage).
# --------------------------------------------------------------------------------------

echo "run_integration_e2e.sh: == step 5: plan_view =="
ros2 run crackvision_motion plan_view --root "$WORK" --scene-config "$SCENE_CFG" \
    --planning-config "$PLANNING_CFG" --out "$VIEW_PLAN_OUT"
rc=$?
[ "$rc" -eq 0 ] || fail "plan_view exit=$rc (expected 0)"
[ -f "$VIEW_PLAN_OUT" ] || fail "plan_view exit 0 but $VIEW_PLAN_OUT not written"

echo "run_integration_e2e.sh: == step 5: plan_crack_path =="
ros2 run crackvision_motion plan_crack_path --root "$WORK" --paths3d "$PATHS3D_JSON" \
    --scene-config "$SCENE_CFG" --planning-config "$PLANNING_CFG" --out "$CARTESIAN_PLAN_OUT" \
    --require-full-coverage
rc=$?
if [ "$rc" -ne 0 ]; then
    fail "plan_crack_path exit=$rc (expected 0): the synthetic crack could not be fully covered " \
        "(--require-full-coverage refused); see $CARTESIAN_PLAN_OUT.log / above output for the reported reasons"
fi
[ -f "$CARTESIAN_PLAN_OUT" ] || fail "plan_crack_path exit 0 but $CARTESIAN_PLAN_OUT not written"

stop_mock_stack

# --------------------------------------------------------------------------------------
# 6. Validate both outputs with the real loaders (load_view_plan/load_plan).
# --------------------------------------------------------------------------------------

echo "run_integration_e2e.sh: == step 6: validating planning outputs =="
python3 - "$REPO_ROOT" "$VIEW_PLAN_OUT" "$CARTESIAN_PLAN_OUT" "$PATHS3D_JSON" "$SCRATCH_LIMITS" <<'PYEOF' \
    || fail "planning output validation failed"
import json
import sys

sys.path.insert(0, f"{sys.argv[1]}/ros2_ws/src/crackvision_motion")
from crackvision_motion.cartesian_plan import load_plan, load_view_plan  # noqa: E402

repo_root, view_plan_path, plan_path, paths3d_path, limits_path = sys.argv[1:6]

view_plan = load_view_plan(view_plan_path, limits_path)
assert view_plan["chosen"] is not None, "view_plan: no candidate chosen"

plan = load_plan(plan_path, limits_path)
cov = plan["coverage"]
assert cov["complete"] is True, f"coverage.complete is not true: {cov}"
assert plan["sources"]["paths3d"]["execution_eligible"] is False, "sources.paths3d.execution_eligible is not false"

paths3d_doc = json.load(open(paths3d_path, encoding="utf-8"))
assert paths3d_doc["execution_eligible"] is False, "paths3d execution_eligible is not false"

seg = plan["segments"][0]
print(
    f"run_integration_e2e.sh: view candidates_tried={len(view_plan['candidates'])} "
    f"chosen_tilt_deg={view_plan['chosen']['tilt_deg']} chosen_yaw_deg={view_plan['chosen']['yaw_deg']}"
)
print(
    f"run_integration_e2e.sh: cartesian candidates_tried={seg['candidates_tried']} "
    f"chosen={seg['chosen']} coverage={cov['trace_waypoints_planned']}/{cov['trace_waypoints_total']} "
    f"fraction_by_length={cov['fraction_by_length']:.6f}"
)
PYEOF

# --------------------------------------------------------------------------------------
# 7. retime_plan.
# --------------------------------------------------------------------------------------

echo "run_integration_e2e.sh: == step 7: retime_plan =="
RETIME_CONFIG="$REPO_ROOT/config/motion/retime.yaml"
EXECUTION_CONFIG="$REPO_ROOT/config/motion/execution.yaml"
URDF="$REPO_ROOT/presentation/sim/reBot_B601_DM_with_gripper.urdf"
TRAJECTORY="$WORK/trajectory.json"
REPORT="$TRAJECTORY.retime.json"
for f in "$RETIME_CONFIG" "$EXECUTION_CONFIG" "$URDF"; do
    [ -f "$f" ] || fail "required input missing: $f"
done

RETIME_LOG="$WORK/retime_plan.log"
ros2 run crackvision_motion retime_plan --plan "$CARTESIAN_PLAN_OUT" --root "$WORK" \
    --limits "$SCRATCH_LIMITS" --end-effector-config "$SCRATCH_EE" \
    --execution-config "$EXECUTION_CONFIG" --retime-config "$RETIME_CONFIG" --urdf "$URDF" \
    --out "$TRAJECTORY" --purpose crack_task >"$RETIME_LOG" 2>&1
rc=$?
cat "$RETIME_LOG"
[ "$rc" -eq 0 ] || fail "retime_plan exit=$rc (expected 0); see $RETIME_LOG"
[ -f "$TRAJECTORY" ] || fail "retime_plan exit 0 but $TRAJECTORY not written"
[ -f "$REPORT" ] || fail "retime_plan exit 0 but $REPORT (retime_report sidecar) not written"
echo "run_integration_e2e.sh: retime_plan OK -> $TRAJECTORY"

# --------------------------------------------------------------------------------------
# 8. trajectory_check -- the independent checker (conda env, never crackvision_motion).
# --------------------------------------------------------------------------------------

echo "run_integration_e2e.sh: == step 8: trajectory_check =="
CHECK_OUT="$WORK/trajectory_check.json"
"$REPO_ROOT/env.sh" python -m crackvision.trajectory_check \
    --trajectory "$TRAJECTORY" --plan "$CARTESIAN_PLAN_OUT" --report "$REPORT" \
    --limits "$SCRATCH_LIMITS" --end-effector-config "$SCRATCH_EE" \
    --execution-config "$EXECUTION_CONFIG" --retime-config "$RETIME_CONFIG" \
    --out "$CHECK_OUT" -q
rc=$?
[ "$rc" -eq 0 ] || fail "trajectory_check exit=$rc (expected 0); see $CHECK_OUT"
python3 -c '
import json, sys
report = json.load(open(sys.argv[1], encoding="utf-8"))
for c in report["checks"]:
    check, outcome = c["check"], c["outcome"]
    print(f"run_integration_e2e.sh: check {check}={outcome}")
    if outcome not in ("pass", "skip"):
        sys.exit(f"check {check} outcome={outcome!r}, expected pass/skip")
' "$CHECK_OUT" || fail "trajectory_check: not every check outcome is pass/skip"
echo "run_integration_e2e.sh: trajectory_check OK"

# --------------------------------------------------------------------------------------
# 9. mock stack (execution leg): pre-position the mock arm to the plan's start state, then
#    dry/mock the retimed trajectory itself.
# --------------------------------------------------------------------------------------

echo "run_integration_e2e.sh: == step 9: mock stack (execution) =="
EXEC_LAUNCH_LOG="$WORK/mock_execution_launch.log"
start_mock_stack "$EXEC_LAUNCH_LOG" || exit 1
sleep 2

ros2 run crackvision_motion apply_scene --root "$REPO_ROOT" --config "$SCENE_CFG" >/dev/null \
    || fail "apply_scene (scratch scene) failed"

read_joint_state() {
    python3 - "$1" "$2" <<'PYEOF'
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

topic, timeout_s = sys.argv[1], float(sys.argv[2])
rclpy.init()
node = Node("run_integration_e2e_js_reader")
box = {}
node.create_subscription(JointState, topic, lambda m: box.setdefault("m", m), qos_profile_sensor_data)
deadline = time.monotonic() + timeout_s
while "m" not in box and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if "m" not in box:
    print(f"read_joint_state: no message on {topic} within {timeout_s}s", file=sys.stderr)
    sys.exit(1)
m = box["m"]
print(json.dumps(dict(zip(m.name, m.position))))
PYEOF
}

max_delta_to_point() {
    python3 -c '
import json, sys
a = json.loads(sys.argv[1])
point = [float(v) for v in sys.argv[2].split(",")]
names = [f"joint{i}" for i in range(1, 7)]
print(max(abs(a[n] - p) for n, p in zip(names, point)))
' "$1" "$2"
}

assert_lt() {
    python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) < float(sys.argv[2]) else 1)" "$1" "$2"
}

# -- 9a. a hand-written purpose:"test" move, home(all-zero, the mock hardware's own startup
#    state) -> the plan's start_state, 3 points ramped like test_execution.sh's
#    trajectory_smoke fixture, scaled so no joint exceeds ~0.3 rad/s (well under
#    b601_dm_limits.yaml's 1.0 rad/s, 1.0 rad/s^2) -- this never touches any real-mode gate
#    and carries no limits/scene/end-effector sha (purpose "test", warn-only outside real).
START_POINT_CSV="$(python3 -c '
import json
traj = json.load(open("'"$TRAJECTORY"'", encoding="utf-8"))
print(",".join(str(v) for v in traj["points"][0]["positions"]))
')"
END_POINT_CSV="$(python3 -c '
import json
traj = json.load(open("'"$TRAJECTORY"'", encoding="utf-8"))
print(",".join(str(v) for v in traj["points"][-1]["positions"]))
')"

TO_START="$WORK/to_start.json"
python3 - "$START_POINT_CSV" "$TO_START" <<'PYEOF' || fail "could not build to_start.json"
import json
import sys

start_csv, out_path = sys.argv[1], sys.argv[2]
start = [float(v) for v in start_csv.split(",")]
max_abs = max(abs(v) for v in start) or 1.0
duration_s = max(3.0, max_abs / 0.3)
mid = [v / 2.0 for v in start]
doc = {
    "schema": "crackvision.joint_trajectory/1",
    "created_utc": "2026-01-01T00:00:00Z",
    "producer": "run_integration_e2e.sh (INT-01.2)",
    "purpose": "test",
    "planning_frame": "base_link",
    "joint_names": [f"joint{i}" for i in range(1, 7)],
    "notes": "home(all-zero) -> the retimed plan's start_state, so the mock stack's own joint_states matches the real trajectory's first point before it is dry/mock executed.",
    "points": [
        {"t_s": 0.0, "positions": [0.0] * 6},
        {"t_s": duration_s / 2.0, "positions": mid},
        {"t_s": duration_s, "positions": start},
    ],
    "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": None},
    "end_effector_config_sha256": None,
    "scene_config_sha256": None,
    "source": {"paths3d": None},
}
json.dump(doc, open(out_path, "w", encoding="utf-8"), indent=2)
PYEOF

echo "-- mock mode (purpose test): home -> plan start_state --"
TOSTART_LOG="$WORK/execute_to_start.log"
ros2 run crackvision_motion execute_trajectory --mode mock --profile moveit_mock \
    --trajectory "$TO_START" --scene-config "$SCENE_CFG" --execution-config "$EXECUTION_CONFIG" \
    --record-dir "$RECORD_DIR" >"$TOSTART_LOG" 2>&1
rc=$?
cat "$TOSTART_LOG"
[ "$rc" -eq 0 ] || fail "to_start move exit=$rc (expected 0); see $TOSTART_LOG"
TOSTART_REC="$(ls -t "$RECORD_DIR"/*.json | head -n1)"
python3 -c '
import json, sys
record = json.load(open(sys.argv[1]))
outcome = record["outcome"]
if outcome != "completed":
    sys.exit(f"to_start outcome={outcome!r}, expected completed")
' "$TOSTART_REC" || fail "to_start move did not report outcome=completed"
echo "run_integration_e2e.sh: to_start move OK"

# -- 9b. --mode dry on the real retimed trajectory: must exit 0, with would_refuse_in_real
#    containing at least G-ELIGIBLE (the synthetic paths3d is ineligible, §10/§16.2) and
#    G-APPROVAL (no --approval given).
echo "-- dry mode: retimed trajectory --"
DRY_LOG="$WORK/execute_dry.log"
ros2 run crackvision_motion execute_trajectory --mode dry --profile moveit_mock \
    --trajectory "$TRAJECTORY" --root "$WORK" --scene-config "$SCENE_CFG" \
    --execution-config "$EXECUTION_CONFIG" --record-dir "$RECORD_DIR" >"$DRY_LOG" 2>&1
rc=$?
cat "$DRY_LOG"
[ "$rc" -eq 0 ] || fail "dry mode exit=$rc (expected 0); see $DRY_LOG"
DRY_REC="$(ls -t "$RECORD_DIR"/*.json | head -n1)"
python3 -c '
import json, sys
record = json.load(open(sys.argv[1]))
required = {"G-ELIGIBLE", "G-APPROVAL"}
got = set(record.get("would_refuse_in_real", []))
missing = required - got
if missing:
    sys.exit(f"would_refuse_in_real {sorted(got)} is missing required id(s) {sorted(missing)}")
for c in record["gate_report"]:
    gate, outcome, detail = c["gate"], c["outcome"], c["detail"]
    print(f"  {gate}={outcome}: {detail}")
' "$DRY_REC" || fail "dry mode would_refuse_in_real assertion failed"
echo "run_integration_e2e.sh: dry mode OK, would_refuse_in_real includes G-ELIGIBLE and G-APPROVAL"

# -- 9c. --mode mock on the real retimed trajectory: must exit 0, outcome completed, arrival
#    verified, with G-LIMITS/G-STALE-CONFIG/G-COLLISION all passing (the scratch
#    scene/config are passed explicitly, so every sha the trajectory recorded is consistent
#    with what is actually read here -- never a production config edit).
echo "-- mock mode: retimed trajectory --"
MOCK_LOG="$WORK/execute_mock.log"
ros2 run crackvision_motion execute_trajectory --mode mock --profile moveit_mock \
    --trajectory "$TRAJECTORY" --root "$WORK" --scene-config "$SCENE_CFG" \
    --execution-config "$EXECUTION_CONFIG" --record-dir "$RECORD_DIR" >"$MOCK_LOG" 2>&1
rc=$?
cat "$MOCK_LOG"
[ "$rc" -eq 0 ] || fail "mock mode exit=$rc (expected 0); see $MOCK_LOG"
MOCK_REC="$(ls -t "$RECORD_DIR"/*.json | head -n1)"
python3 -c '
import json, sys
record = json.load(open(sys.argv[1]))
outcome = record["outcome"]
if outcome != "completed":
    sys.exit(f"outcome={outcome!r}, expected completed")
outcomes = {c["gate"]: c["outcome"] for c in record["gate_report"]}
for gate in ("G-LIMITS", "G-STALE-CONFIG", "G-COLLISION"):
    got = outcomes.get(gate)
    if got != "pass":
        sys.exit(f"{gate} outcome={got!r}, expected pass: {record['gate_report']}")
' "$MOCK_REC" || fail "mock mode outcome/gate assertion failed"

LIVE="$(read_joint_state /joint_states 5)" || fail "mock mode: could not read joint_states after completion"
DELTA="$(max_delta_to_point "$LIVE" "$END_POINT_CSV")"
assert_lt "$DELTA" "0.06" || fail "mock mode: did not arrive at the trajectory's final point (max|dq|=$DELTA)"
echo "run_integration_e2e.sh: mock mode OK, outcome=completed, arrival verified (max|dq|=$DELTA)"

stop_mock_stack

sleep 1
STRAY=$(ps -eo pid=,comm= | grep -E "move_group|ros2_control_node|robot_state_publisher" || true)
if [ -n "$STRAY" ]; then
    echo "run_integration_e2e.sh: stray process(es) remain after teardown:" >&2
    echo "$STRAY" >&2
    exit 1
fi

echo "run_integration_e2e.sh: OK ($WORK)"
exit 0
