#!/usr/bin/env bash
# scripts/ros/test_retime_execute.sh — MOT-07.9 headless mock execution of a retimed synthetic
# crack plan (ADR-019, docs/INTERFACES.md §16 + §11): run_cartesian_synthetic.sh's own scratch
# plan (MOT-06.10) -> retime_plan (MOT-07) -> trajectory_check (independent re-check, §16.9) ->
# execute_trajectory --mode dry and --mode mock on the moveit_mock profile only.
#
# No hardware, no vendor driver, no real mode, CRACKVISION_ARM_REAL never set: this script only
# ever starts the headless ros2_control mock hardware stack (`scripts/ros/_mock_stack.sh`,
# `start_mock_stack`/`stop_mock_stack`), which installs its own `trap stop_mock_stack EXIT INT
# TERM`, so the stack is torn down on every exit path, including a failure partway through.
#
# The retimed trajectory's own start point (the plan's `start_state`, a synthetic
# `capture_record` joint state, not all-zero) does not match the mock stack's own all-zero
# startup state, so G-START-STATE would refuse immediately. Per the card, this script first runs
# a small, hand-written `purpose: "test"` trajectory (same home(0)->target pattern as
# `test_execution.sh`'s `trajectory_smoke.json`) in `--mode mock` to put the mock arm's live
# `/joint_states` at the plan's start state before touching the real retimed trajectory at all —
# this is motion, but only ever against the mock stack, under the same gated `execute_trajectory`
# path everything else in this script uses, never a bypass of any gate.
#
# Scratch space only: everything this script writes lives under logs/retime_execute/<stamp>/ and
# the run_cartesian_synthetic.sh scratch root it reuses (logs/cartesian_synthetic/<stamp>/);
# nothing under data/ or config/ is ever written or modified.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
WORK="$REPO_ROOT/logs/retime_execute/$STAMP"
RECORD_DIR="$WORK/execution_records"
mkdir -p "$WORK" "$RECORD_DIR"

fail() {
    echo "test_retime_execute.sh: FAIL: $*" >&2
    exit 1
}

# -- guard: never run anywhere near a real vendor driver process (§0's truncated-comm pattern,
# mirroring test_execution.sh).
if pgrep -x reBotArmControl >/dev/null 2>&1; then
    echo "test_retime_execute.sh: a real vendor driver process (reBotArmControl) is running -- refusing to run" >&2
    exit 3
fi

# --------------------------------------------------------------------------------------
# 1. A fresh scratch root (MOT-06.10's own synthetic plan, untimed).
# --------------------------------------------------------------------------------------

echo "test_retime_execute.sh: == step 1: run_cartesian_synthetic.sh (fresh scratch plan) =="
SYNTH_LOG="$WORK/run_cartesian_synthetic.log"
bash "$SCRIPT_DIR/run_cartesian_synthetic.sh" >"$SYNTH_LOG" 2>&1
rc=$?
cat "$SYNTH_LOG"
[ "$rc" -eq 0 ] || fail "run_cartesian_synthetic.sh exit=$rc (expected 0); see $SYNTH_LOG"

ROOT="$(sed -n 's/^run_cartesian_synthetic.sh: scratch root //p' "$SYNTH_LOG" | head -n1)"
[ -n "$ROOT" ] && [ -d "$ROOT" ] || fail "could not recover the scratch root from $SYNTH_LOG"
PLAN="$ROOT/cartesian_plan.json"
SCENE_CONFIG="$ROOT/scene.yaml"
SCRATCH_LIMITS="$ROOT/config/robot/b601_dm_limits.yaml"
SCRATCH_EE="$ROOT/config/robot/end_effector.yaml"
for f in "$PLAN" "$SCENE_CONFIG" "$SCRATCH_LIMITS" "$SCRATCH_EE"; do
    [ -f "$f" ] || fail "expected scratch root file missing: $f"
done
echo "test_retime_execute.sh: scratch root $ROOT"

# --------------------------------------------------------------------------------------
# 2. retime_plan -- the plan's scratch limits/end-effector copies, but the repo's real
#    retime.yaml/execution.yaml and URDF (never a loosened policy).
# --------------------------------------------------------------------------------------

echo "test_retime_execute.sh: == step 2: retime_plan =="
RETIME_CONFIG="$REPO_ROOT/config/motion/retime.yaml"
EXECUTION_CONFIG="$REPO_ROOT/config/motion/execution.yaml"
URDF="$REPO_ROOT/presentation/sim/reBot_B601_DM_with_gripper.urdf"
TRAJECTORY="$ROOT/trajectory.json"
REPORT="$TRAJECTORY.retime.json"
for f in "$RETIME_CONFIG" "$EXECUTION_CONFIG" "$URDF"; do
    [ -f "$f" ] || fail "required input missing: $f"
done

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_retime_execute.sh: $OVERLAY_SETUP not found -- run scripts/ros/build_ws.sh first" >&2
    exit 3
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

RETIME_LOG="$WORK/retime_plan.log"
ros2 run crackvision_motion retime_plan --plan "$PLAN" --root "$ROOT" \
    --limits "$SCRATCH_LIMITS" --end-effector-config "$SCRATCH_EE" \
    --execution-config "$EXECUTION_CONFIG" --retime-config "$RETIME_CONFIG" --urdf "$URDF" \
    --out "$TRAJECTORY" --purpose crack_task >"$RETIME_LOG" 2>&1
rc=$?
cat "$RETIME_LOG"
[ "$rc" -eq 0 ] || fail "retime_plan exit=$rc (expected 0); see $RETIME_LOG"
[ -f "$TRAJECTORY" ] || fail "retime_plan exit 0 but $TRAJECTORY not written"
[ -f "$REPORT" ] || fail "retime_plan exit 0 but $REPORT (retime_report sidecar) not written"
echo "test_retime_execute.sh: retime_plan OK -> $TRAJECTORY"

# Headline numbers (duration, points, per-trace-leg v_c/ramps, peak utilisation) straight from the
# report, independent of retime_plan's own log lines -- also feeds docs/motion/TIME_PARAMETERIZATION.md.
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
    if leg["kind"] == "trace":
        ts = leg["tool_speed"]
        print(
            f"trace leg {leg['leg_index']}: v_c={ts['constant']:.6f} m/s target={ts['target']:.6f} m/s "
            f"min={ts['min']:.6f} max={ts['max']:.6f} ramp_in_m={ts['ramp_in_m']:.6f} ramp_out_m={ts['ramp_out_m']:.6f}"
        )
PYEOF
cat "$HEADLINE_LOG"

# --------------------------------------------------------------------------------------
# 3. trajectory_check -- the independent checker (conda env, never crackvision_motion).
# --------------------------------------------------------------------------------------

echo "test_retime_execute.sh: == step 3: trajectory_check =="
CHECK_OUT="$WORK/trajectory_check.json"
"$REPO_ROOT/env.sh" python -m crackvision.trajectory_check \
    --trajectory "$TRAJECTORY" --plan "$PLAN" --report "$REPORT" \
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
    print(f"test_retime_execute.sh: check {check}={outcome}")
' "$CHECK_OUT"
echo "test_retime_execute.sh: trajectory_check OK"

# --------------------------------------------------------------------------------------
# 4. headless mock stack: move the mock arm to the plan's start state, then dry/mock the
#    retimed trajectory itself.
# --------------------------------------------------------------------------------------

echo "test_retime_execute.sh: == step 4: mock stack =="
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
node = Node("test_retime_execute_js_reader")
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

# -- 4a. a hand-written purpose:"test" move, home(all-zero, the mock hardware's own startup
#    state) -> the plan's start_state, 3 points ramped like test_execution.sh's trajectory_smoke
#    fixture, scaled so no joint exceeds ~0.3 rad/s (well under b601_dm_limits.yaml's 1.0 rad/s,
#    1.0 rad/s^2) -- this never touches any real-mode gate and carries no limits/scene/end-effector
#    sha (purpose "test", warn-only outside real).
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
    "producer": "test_retime_execute.sh (MOT-07.9)",
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
    --trajectory "$TO_START" --scene-config "$SCENE_CONFIG" --execution-config "$EXECUTION_CONFIG" \
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
echo "test_retime_execute.sh: to_start move OK"

# -- 4b. --mode dry on the real retimed trajectory: must exit 0, with would_refuse_in_real
#    containing at least G-ELIGIBLE (the MOT-06.10 synthetic paths3d is ineligible, §10/§16.2)
#    and G-APPROVAL (no --approval given).
echo "-- dry mode: retimed trajectory --"
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
required = {"G-ELIGIBLE", "G-APPROVAL"}
got = set(record.get("would_refuse_in_real", []))
missing = required - got
if missing:
    sys.exit(f"would_refuse_in_real {sorted(got)} is missing required id(s) {sorted(missing)}")
for c in record["gate_report"]:
    gate, outcome, detail = c["gate"], c["outcome"], c["detail"]
    print(f"  {gate}={outcome}: {detail}")
' "$DRY_REC" || fail "dry mode would_refuse_in_real assertion failed"
echo "test_retime_execute.sh: dry mode OK, would_refuse_in_real includes G-ELIGIBLE and G-APPROVAL"

# -- 4c. --mode mock on the real retimed trajectory: must exit 0, outcome completed, arrival
#    verified, with G-LIMITS/G-STALE-CONFIG/G-COLLISION all passing (the scratch scene/config
#    are passed explicitly, so every sha the trajectory recorded is consistent with what is
#    actually read here -- never a production config edit).
echo "-- mock mode: retimed trajectory --"
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
for gate in ("G-LIMITS", "G-STALE-CONFIG", "G-COLLISION"):
    got = outcomes.get(gate)
    if got != "pass":
        sys.exit(f"{gate} outcome={got!r}, expected pass: {record['gate_report']}")
' "$MOCK_REC" || fail "mock mode outcome/gate assertion failed"

LIVE="$(read_joint_state /joint_states 5)" || fail "mock mode: could not read joint_states after completion"
DELTA="$(max_delta_to_point "$LIVE" "$END_POINT_CSV")"
assert_lt "$DELTA" "0.06" || fail "mock mode: did not arrive at the trajectory's final point (max|dq|=$DELTA)"
echo "test_retime_execute.sh: mock mode OK, outcome=completed, arrival verified (max|dq|=$DELTA)"

stop_mock_stack

sleep 1
STRAY=$(ps -eo pid=,comm= | grep -E "move_group|ros2_control_node|robot_state_publisher" || true)
if [ -n "$STRAY" ]; then
    echo "test_retime_execute.sh: stray process(es) remain after teardown:" >&2
    echo "$STRAY" >&2
    exit 1
fi

echo "test_retime_execute.sh: OK ($WORK)"
exit 0
