#!/usr/bin/env bash
# scripts/ros/test_view_retime_execute.sh — MOT-07.10.5 headless mock run of a view-move
# trajectory (ADR-019's MOT-07.10 addendum, docs/INTERFACES.md §17): run_cartesian_synthetic.sh's
# own scratch view_plan (MOT-06.10) -> retime_plan --view-plan (§17.3) -> trajectory_check
# (independent re-check, §17.4) -> execute_trajectory --mode dry/mock on the moveit_mock profile
# only, plus the real-only refusal and a tampered-sha G-ELIGIBLE refusal, modelled directly on
# scripts/ros/test_retime_execute.sh's MOT-07.9 pattern.
#
# No hardware, no vendor driver, no real mode, CRACKVISION_ARM_REAL never set: this script only
# ever starts the headless ros2_control mock hardware stack (scripts/ros/_mock_stack.sh,
# start_mock_stack/stop_mock_stack), which installs its own `trap stop_mock_stack EXIT INT TERM`,
# so the stack is torn down on every exit path, including a failure partway through.
#
# The view-move trajectory's own first point is the view plan's transit_leg.positions[0], which
# (per the card) is the SRDF all-zero home — the same state the mock hardware starts up in — so
# no pre-positioning move is needed (unlike test_retime_execute.sh's crack_task plan, whose
# start_state is a synthetic capture_record joint state). This is verified below rather than
# assumed: if G-START-STATE still refuses, the script fails loudly rather than silently
# pre-positioning.
#
# Scratch space only: everything this script writes lives under logs/view_retime_execute/<stamp>/
# and the run_cartesian_synthetic.sh scratch root it reuses (logs/cartesian_synthetic/<stamp>/);
# nothing under data/ or config/ is ever written or modified.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
WORK="$REPO_ROOT/logs/view_retime_execute/$STAMP"
RECORD_DIR="$WORK/execution_records"
mkdir -p "$WORK" "$RECORD_DIR"

fail() {
    echo "test_view_retime_execute.sh: FAIL: $*" >&2
    exit 1
}

# -- guard: never run anywhere near a real vendor driver process (§0's truncated-comm pattern,
# mirroring test_execution.sh/test_retime_execute.sh).
if pgrep -x reBotArmControl >/dev/null 2>&1; then
    echo "test_view_retime_execute.sh: a real vendor driver process (reBotArmControl) is running -- refusing to run" >&2
    exit 3
fi

# --------------------------------------------------------------------------------------
# 1. A fresh scratch root (MOT-06.10's own synthetic view_plan + cartesian_plan).
# --------------------------------------------------------------------------------------

echo "test_view_retime_execute.sh: == step 1: run_cartesian_synthetic.sh (fresh scratch plan) =="
SYNTH_LOG="$WORK/run_cartesian_synthetic.log"
bash "$SCRIPT_DIR/run_cartesian_synthetic.sh" >"$SYNTH_LOG" 2>&1
rc=$?
cat "$SYNTH_LOG"
[ "$rc" -eq 0 ] || fail "run_cartesian_synthetic.sh exit=$rc (expected 0); see $SYNTH_LOG"

ROOT="$(sed -n 's/^run_cartesian_synthetic.sh: scratch root //p' "$SYNTH_LOG" | head -n1)"
[ -n "$ROOT" ] && [ -d "$ROOT" ] || fail "could not recover the scratch root from $SYNTH_LOG"
VIEW_PLAN="$ROOT/view_plan.json"
SCENE_CONFIG="$ROOT/scene.yaml"
SCRATCH_LIMITS="$ROOT/config/robot/b601_dm_limits.yaml"
SCRATCH_EE="$ROOT/config/robot/end_effector.yaml"
for f in "$VIEW_PLAN" "$SCENE_CONFIG" "$SCRATCH_LIMITS" "$SCRATCH_EE"; do
    [ -f "$f" ] || fail "expected scratch root file missing: $f"
done
echo "test_view_retime_execute.sh: scratch root $ROOT"

# --------------------------------------------------------------------------------------
# 2. retime_plan --view-plan -- the plan's scratch limits/end-effector copies, but the repo's
#    real retime.yaml/execution.yaml and URDF (never a loosened policy). --root must be the
#    scratch root throughout so source.view_plan.path resolves consistently everywhere.
# --------------------------------------------------------------------------------------

echo "test_view_retime_execute.sh: == step 2: retime_plan --view-plan =="
RETIME_CONFIG="$REPO_ROOT/config/motion/retime.yaml"
EXECUTION_CONFIG="$REPO_ROOT/config/motion/execution.yaml"
URDF="$REPO_ROOT/presentation/sim/reBot_B601_DM_with_gripper.urdf"
TRAJECTORY="$ROOT/view_trajectory.json"
REPORT="$TRAJECTORY.retime.json"
for f in "$RETIME_CONFIG" "$EXECUTION_CONFIG" "$URDF"; do
    [ -f "$f" ] || fail "required input missing: $f"
done

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_view_retime_execute.sh: $OVERLAY_SETUP not found -- run scripts/ros/build_ws.sh first" >&2
    exit 3
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

RETIME_LOG="$WORK/retime_plan.log"
ros2 run crackvision_motion retime_plan --view-plan "$VIEW_PLAN" --root "$ROOT" \
    --limits "$SCRATCH_LIMITS" --end-effector-config "$SCRATCH_EE" \
    --execution-config "$EXECUTION_CONFIG" --retime-config "$RETIME_CONFIG" --urdf "$URDF" \
    --out "$TRAJECTORY" >"$RETIME_LOG" 2>&1
rc=$?
cat "$RETIME_LOG"
[ "$rc" -eq 0 ] || fail "retime_plan --view-plan exit=$rc (expected 0); see $RETIME_LOG"
[ -f "$TRAJECTORY" ] || fail "retime_plan exit 0 but $TRAJECTORY not written"
[ -f "$REPORT" ] || fail "retime_plan exit 0 but $REPORT (retime_report sidecar) not written"
echo "test_view_retime_execute.sh: retime_plan --view-plan OK -> $TRAJECTORY"

HEADLINE_LOG="$WORK/headline.log"
python3 - "$REPORT" >"$HEADLINE_LOG" <<'PYEOF' || fail "could not read retime_report headline numbers"
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
totals = report["totals"]
print(
    f"duration_s={totals['duration_s']:.3f} n_points={totals['n_points']} "
    f"peak_velocity_fraction_max={max(totals['peak_velocity_fraction']):.3f} "
    f"peak_acceleration_fraction_max={max(totals['peak_acceleration_fraction']):.3f}"
)
for leg in report["legs"]:
    print(f"leg {leg['leg_index']} ({leg['kind']}): point_range={leg['point_range']} duration_s={leg['duration_s']:.3f}")
PYEOF
cat "$HEADLINE_LOG"

# --------------------------------------------------------------------------------------
# 3. trajectory_check --plan <view_plan> -- the independent checker (conda env, never
#    crackvision_motion), §17.4.
# --------------------------------------------------------------------------------------

echo "test_view_retime_execute.sh: == step 3: trajectory_check =="
CHECK_OUT="$WORK/trajectory_check.json"
"$REPO_ROOT/env.sh" python -m crackvision.trajectory_check \
    --trajectory "$TRAJECTORY" --plan "$VIEW_PLAN" --report "$REPORT" \
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
    print(f"test_view_retime_execute.sh: check {check}={outcome}")
    if outcome not in ("pass", "skip"):
        sys.exit(f"check {check} outcome={outcome!r}, expected pass or skip")
' "$CHECK_OUT" || fail "trajectory_check reported a non-pass/skip check outcome"
echo "test_view_retime_execute.sh: trajectory_check OK"

# --------------------------------------------------------------------------------------
# 4. headless mock stack: dry/mock execute the view-move trajectory directly -- no
#    pre-positioning move, since the transit leg starts at the mock hardware's own all-zero
#    startup state (verified below via G-START-STATE's offline outcome, not assumed).
# --------------------------------------------------------------------------------------

echo "test_view_retime_execute.sh: == step 4: mock stack =="
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"
LAUNCH_LOG="$WORK/mock_planning_launch.log"
start_mock_stack "$LAUNCH_LOG" || exit 1
sleep 2

ros2 run crackvision_motion apply_scene --root "$REPO_ROOT" --config "$SCENE_CONFIG" >/dev/null \
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
node = Node("test_view_retime_execute_js_reader")
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

END_POINT_CSV="$(python3 -c '
import json
traj = json.load(open("'"$TRAJECTORY"'", encoding="utf-8"))
print(",".join(str(v) for v in traj["points"][-1]["positions"]))
')"

# -- 4a. --mode dry: must exit 0, with would_refuse_in_real containing at least G-APPROVAL
#    (no --approval given) -- G-START-STATE must *not* be among the real-preview refusals, since
#    the transit leg starts at the mock hardware's own all-zero startup state.
echo "-- dry mode: view-move trajectory --"
DRY_LOG="$WORK/execute_dry.log"
ros2 run crackvision_motion execute_trajectory --mode dry --profile moveit_mock \
    --trajectory "$TRAJECTORY" --root "$ROOT" --scene-config "$SCENE_CONFIG" \
    --execution-config "$EXECUTION_CONFIG" --record-dir "$RECORD_DIR" >"$DRY_LOG" 2>&1
rc=$?
cat "$DRY_LOG"
[ "$rc" -eq 0 ] || fail "dry mode exit=$rc (expected 0); see $DRY_LOG"
DRY_REC="$(ls -t "$RECORD_DIR"/*.json | head -n1)"
python3 -c '
import json, sys
record = json.load(open(sys.argv[1]))
outcomes = {c["gate"]: c["outcome"] for c in record["gate_report"]}
for gate in ("G-TRAJ", "G-ELIGIBLE"):
    got = outcomes.get(gate)
    if got != "pass":
        sys.exit(f"{gate} outcome={got!r}, expected pass: {record['gate_report']}")
required = {"G-APPROVAL"}
got = set(record.get("would_refuse_in_real", []))
missing = required - got
if missing:
    sys.exit(f"would_refuse_in_real {sorted(got)} is missing required id(s) {sorted(missing)}")
for c in record["gate_report"]:
    gate, outcome, detail = c["gate"], c["outcome"], c["detail"]
    print(f"  {gate}={outcome}: {detail}")
' "$DRY_REC" || fail "dry mode gate/would_refuse_in_real assertion failed"
echo "test_view_retime_execute.sh: dry mode OK, G-TRAJ/G-ELIGIBLE pass, would_refuse_in_real includes G-APPROVAL"

# -- 4b. --mode mock: must exit 0, outcome completed, final joint_states within tolerance of the
#    transit leg's own last configuration.
echo "-- mock mode: view-move trajectory --"
MOCK_LOG="$WORK/execute_mock.log"
ros2 run crackvision_motion execute_trajectory --mode mock --profile moveit_mock \
    --trajectory "$TRAJECTORY" --root "$ROOT" --scene-config "$SCENE_CONFIG" \
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
for gate in ("G-LIMITS", "G-STALE-CONFIG", "G-ELIGIBLE"):
    got = outcomes.get(gate)
    if got != "pass":
        sys.exit(f"{gate} outcome={got!r}, expected pass: {record['gate_report']}")
' "$MOCK_REC" || fail "mock mode outcome/gate assertion failed"

LIVE="$(read_joint_state /joint_states 5)" || fail "mock mode: could not read joint_states after completion"
DELTA="$(max_delta_to_point "$LIVE" "$END_POINT_CSV")"
assert_lt "$DELTA" "0.06" || fail "mock mode: did not arrive at the trajectory's final point (max|dq|=$DELTA)"
echo "test_view_retime_execute.sh: mock mode OK, outcome=completed, arrival verified (max|dq|=$DELTA)"

# -- 4c. --mode real without CRACKVISION_ARM_REAL set: must refuse, exit 3 (G-ARM, §11.7).
#    This script never sets CRACKVISION_ARM_REAL anywhere -- this is the unavoidable refusal
#    any real-mode attempt gets, not a bypass of it.
echo "-- real mode (no CRACKVISION_ARM_REAL): must refuse --"
REAL_LOG="$WORK/execute_real.log"
env -u CRACKVISION_ARM_REAL ros2 run crackvision_motion execute_trajectory --mode real \
    --trajectory "$TRAJECTORY" --root "$ROOT" --scene-config "$SCENE_CONFIG" \
    --execution-config "$EXECUTION_CONFIG" --record-dir "$RECORD_DIR" >"$REAL_LOG" 2>&1
rc=$?
cat "$REAL_LOG"
[ "$rc" -eq 3 ] || fail "real mode exit=$rc (expected 3, G-ARM refusal); see $REAL_LOG"
echo "test_view_retime_execute.sh: real mode correctly refused (exit 3)"

# --------------------------------------------------------------------------------------
# 5. tampered copy: mutate source.view_plan.sha256 in a copy of the trajectory, expect
#    G-ELIGIBLE to refuse (exit 3) in mock, nothing sent to the mock arm.
# --------------------------------------------------------------------------------------

echo "test_view_retime_execute.sh: == step 5: tampered trajectory (G-ELIGIBLE must refuse) =="
TAMPERED="$WORK/view_trajectory_tampered.json"
python3 -c '
import json, sys
doc = json.load(open(sys.argv[1], encoding="utf-8"))
sha = doc["source"]["view_plan"]["sha256"]
tampered = ("0" if sha[0] != "0" else "1") + sha[1:]
doc["source"]["view_plan"]["sha256"] = tampered
json.dump(doc, open(sys.argv[2], "w", encoding="utf-8"), indent=2)
' "$TRAJECTORY" "$TAMPERED" || fail "could not build tampered trajectory copy"

TAMPER_LOG="$WORK/execute_tampered.log"
ros2 run crackvision_motion execute_trajectory --mode mock --profile moveit_mock \
    --trajectory "$TAMPERED" --root "$ROOT" --scene-config "$SCENE_CONFIG" \
    --execution-config "$EXECUTION_CONFIG" --record-dir "$RECORD_DIR" >"$TAMPER_LOG" 2>&1
rc=$?
cat "$TAMPER_LOG"
[ "$rc" -eq 3 ] || fail "tampered trajectory exit=$rc (expected 3, G-ELIGIBLE refusal); see $TAMPER_LOG"
TAMPER_REC="$(ls -t "$RECORD_DIR"/*.json | head -n1)"
python3 -c '
import json, sys
record = json.load(open(sys.argv[1]))
if record["outcome"] != "refused":
    sys.exit(f"outcome={record['outcome']!r}, expected refused")
outcomes = {c["gate"]: c["outcome"] for c in record["gate_report"]}
if outcomes.get("G-ELIGIBLE") not in ("fail", "error"):
    sys.exit(f"G-ELIGIBLE outcome={outcomes.get('G-ELIGIBLE')!r}, expected fail/error: {record['gate_report']}")
if record["joint_trace"]:
    sys.exit("joint_trace is non-empty -- something was sent to the mock arm")
' "$TAMPER_REC" || fail "tampered trajectory gate assertion failed"
echo "test_view_retime_execute.sh: tampered trajectory correctly refused at G-ELIGIBLE (exit 3), nothing sent"

stop_mock_stack

sleep 1
STRAY=$(ps -eo pid=,comm= | grep -E "move_group|ros2_control_node|robot_state_publisher" || true)
if [ -n "$STRAY" ]; then
    echo "test_view_retime_execute.sh: stray process(es) remain after teardown:" >&2
    echo "$STRAY" >&2
    exit 1
fi

echo "test_view_retime_execute.sh: PASS ($WORK)"
exit 0
