#!/usr/bin/env bash
# scripts/ros/test_retime_crosscheck.sh — MOT-07.8 offline cross-validation: retimer vs
# independent checker vs execution_gate's offline G-LIMITS, on a committed MOT-06.10 synthetic
# plan (docs/INTERFACES.md §16, ADR-019), with fault injection. No ROS graph, no robot motion:
# `retime_plan` and `execution_gate` both run ROS-free under the system python3
# (scripts/ros/env_ros.sh, no colcon build required since both modules import no rclpy/*_msgs
# type), `crackvision.trajectory_check` runs under the conda interpreter (./env.sh python).
#
# Fixture: ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/ (verbatim
# plan_crack_path output from scripts/ros/run_cartesian_synthetic.sh, MOT-06.10, plus the
# end_effector.yaml/b601_dm_limits.yaml copies whose shas it records -- see that directory's
# README.md for provenance).
#
# Scratch space only: everything this script writes lives under logs/retime_crosscheck/<stamp>/,
# never under data/.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

FIXTURE_DIR="$REPO_ROOT/ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan"
PLAN="$FIXTURE_DIR/cartesian_plan.json"
LIMITS="$FIXTURE_DIR/b601_dm_limits.yaml"
END_EFFECTOR="$FIXTURE_DIR/end_effector.yaml"

EXECUTION_CONFIG="$REPO_ROOT/config/motion/execution.yaml"
RETIME_CONFIG="$REPO_ROOT/config/motion/retime.yaml"
URDF="$REPO_ROOT/presentation/sim/reBot_B601_DM_with_gripper.urdf"
SCENE_CONFIG="$REPO_ROOT/config/scene/scene.yaml"
COMMISSIONING="$REPO_ROOT/config/robot/commissioning.yaml"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
WORK="$REPO_ROOT/logs/retime_crosscheck/$STAMP"
TRAJECTORY="$WORK/trajectory.json"
REPORT="$TRAJECTORY.retime.json"
FAULTS_DIR="$WORK/faults"
mkdir -p "$WORK" "$FAULTS_DIR"

fail() {
    echo "test_retime_crosscheck.sh: FAIL: $*" >&2
    exit 1
}

for f in "$PLAN" "$LIMITS" "$END_EFFECTOR" "$EXECUTION_CONFIG" "$RETIME_CONFIG" "$URDF" "$SCENE_CONFIG" "$COMMISSIONING"; do
    [ -f "$f" ] || fail "required input missing: $f"
done

# -- 1. retime_plan (system python3, no colcon build needed -- retime_plan imports no rclpy/*_msgs
#    type, §16.6). Explicit --limits/--end-effector-config point at the fixture's own copies (the
#    ones the plan's own sources.* shas were recorded against); --retime-config/--execution-config
#    use the real production files at their default paths, so this is a test of the real policy,
#    not a loosened one.
echo "test_retime_crosscheck.sh: retime_plan"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
set +u
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ -f "$OVERLAY_SETUP" ]; then
    source "$OVERLAY_SETUP"
fi
set -u

RETIME_LOG="$WORK/retime_plan.log"
if [ -f "$OVERLAY_SETUP" ] && command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix crackvision_motion >/dev/null 2>&1; then
    ros2 run crackvision_motion retime_plan --plan "$PLAN" \
        --limits "$LIMITS" --end-effector-config "$END_EFFECTOR" \
        --execution-config "$EXECUTION_CONFIG" --retime-config "$RETIME_CONFIG" --urdf "$URDF" \
        --out "$TRAJECTORY" --purpose test >"$RETIME_LOG" 2>&1
else
    PYTHONPATH="$REPO_ROOT/ros2_ws/src/crackvision_motion:${PYTHONPATH:-}" python3 \
        -m crackvision_motion.retime_plan --plan "$PLAN" \
        --limits "$LIMITS" --end-effector-config "$END_EFFECTOR" \
        --execution-config "$EXECUTION_CONFIG" --retime-config "$RETIME_CONFIG" --urdf "$URDF" \
        --out "$TRAJECTORY" --purpose test >"$RETIME_LOG" 2>&1
fi
rc=$?
cat "$RETIME_LOG" >&2
if [ "$rc" -ne 0 ]; then
    fail "retime_plan exit=$rc (expected 0) on the committed fixture plan; see $RETIME_LOG. " \
        "This is a refusal in the producer itself (§16.3/§16.2), not a disagreement between the " \
        "retimer and the checker -- the fixture's trace leg could not be retimed at the real " \
        "config/motion/retime.yaml policy."
fi
[ -f "$TRAJECTORY" ] || fail "retime_plan exit 0 but $TRAJECTORY not written"
[ -f "$REPORT" ] || fail "retime_plan exit 0 but $REPORT (retime_report sidecar) not written"
echo "test_retime_crosscheck.sh: retime_plan OK -> $TRAJECTORY"

# -- 2. Headline numbers from the retime_report (duration, points, per-trace-leg v_c, peak
#    utilisation), independent of the retimer's own log lines.
python3 - "$REPORT" <<'PYEOF' || fail "could not read retime_report headline numbers"
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
totals = report["totals"]
print(
    f"test_retime_crosscheck.sh: duration_s={totals['duration_s']:.3f} n_points={totals['n_points']} "
    f"peak_velocity_fraction_max={max(totals['peak_velocity_fraction']):.3f} "
    f"peak_acceleration_fraction_max={max(totals['peak_acceleration_fraction']):.3f}"
)
for leg in report["legs"]:
    if leg["kind"] == "trace":
        ts = leg["tool_speed"]
        print(
            f"test_retime_crosscheck.sh: trace leg {leg['leg_index']} v_c={ts['constant']:.6f} m/s "
            f"target={ts['target']:.6f} m/s min={ts['min']:.6f} max={ts['max']:.6f} "
            f"ramp_in_m={ts['ramp_in_m']:.6f} ramp_out_m={ts['ramp_out_m']:.6f}"
        )
PYEOF

# -- 3. Independent checker (conda interpreter, never crackvision_motion). Clean path: every
#    check must pass.
echo "test_retime_crosscheck.sh: trajectory_check (clean)"
CHECK_OUT="$WORK/trajectory_check.json"
"$REPO_ROOT/env.sh" python -m crackvision.trajectory_check \
    --trajectory "$TRAJECTORY" --plan "$PLAN" --report "$REPORT" \
    --limits "$LIMITS" --end-effector-config "$END_EFFECTOR" \
    --execution-config "$EXECUTION_CONFIG" --retime-config "$RETIME_CONFIG" \
    --out "$CHECK_OUT" -q
rc=$?
if [ "$rc" -ne 0 ]; then
    fail "trajectory_check exit=$rc (expected 0) on the clean retimed trajectory; see $CHECK_OUT"
fi
python3 - "$CHECK_OUT" <<'PYEOF' || fail "clean trajectory_check report has a non-pass check"
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
bad = [c for c in report["checks"] if c["outcome"] not in ("pass", "skip")]
if bad:
    for c in bad:
        print(f"test_retime_crosscheck.sh: unexpected outcome {c['check']}={c['outcome']}: {c['detail']}", file=sys.stderr)
    sys.exit(1)
for c in report["checks"]:
    print(f"test_retime_crosscheck.sh: check {c['check']}={c['outcome']}")
PYEOF
[ $? -eq 0 ] || exit 1
echo "test_retime_crosscheck.sh: trajectory_check (clean) OK"

# -- 4. execution_gate's offline evaluation (system python3, mock mode, MOT-05.3's own call
#    convention): G-TRAJ and G-LIMITS must both pass.
evaluate_gate() {
    # evaluate_gate TRAJECTORY_PATH -- prints "<gate>=<outcome>" for every gate on stdout.
    PYTHONPATH="$REPO_ROOT/ros2_ws/src/crackvision_motion:$REPO_ROOT/ros2_ws/src/crackvision_description:${PYTHONPATH:-}" \
        python3 - "$1" "$REPO_ROOT" "$EXECUTION_CONFIG" "$COMMISSIONING" "$LIMITS" "$SCENE_CONFIG" "$END_EFFECTOR" <<'PYEOF'
import sys
from pathlib import Path

traj_path, root, exec_cfg, commissioning, limits, scene_cfg, end_effector = sys.argv[1:8]
from crackvision_motion.execution_gate import evaluate_offline

report = evaluate_offline(
    "mock", traj_path,
    root=Path(root),
    execution_config_path=exec_cfg,
    commissioning_path=commissioning,
    limits_path=limits,
    scene_config_path=scene_cfg,
    end_effector_config_path=end_effector,
    approval_path=None, speed_scale=None, env={},
)
for c in report.checks:
    print(f"{c.id}={c.outcome}:{c.message}")
PYEOF
}

echo "test_retime_crosscheck.sh: execution_gate offline evaluation (clean)"
GATE_OUT="$WORK/execution_gate_clean.txt"
evaluate_gate "$TRAJECTORY" >"$GATE_OUT" || fail "execution_gate evaluation crashed; see $GATE_OUT"
cat "$GATE_OUT"
for gate in G-TRAJ G-LIMITS; do
    line=$(grep "^$gate=" "$GATE_OUT") || fail "$gate missing from execution_gate report"
    outcome="${line#*=}"; outcome="${outcome%%:*}"
    case "$outcome" in
        pass|warn) ;;
        *) fail "$gate reported '$outcome' on the clean trajectory (expected pass, warn allowed outside real)" ;;
    esac
done
echo "test_retime_crosscheck.sh: execution_gate (clean) OK: G-TRAJ and G-LIMITS pass"

# -- 5. Fault injection (stdlib-only retime_fault_inject.py): one mutated trajectory per fault.
echo "test_retime_crosscheck.sh: fault injection"
python3 "$SCRIPT_DIR/retime_fault_inject.py" \
    --trajectory "$TRAJECTORY" --plan "$PLAN" --limits "$LIMITS" --out-dir "$FAULTS_DIR"
rc=$?
[ "$rc" -eq 0 ] || fail "retime_fault_inject.py exit=$rc (expected 0)"

# Faults that execution_gate's G-LIMITS must independently flag too (velocity/acceleration
# faults, per the card's acceptance criteria).
declare -A GATE_MUST_FLAG=(
    [segment_overspeed]=1
    [accel_spike]=1
)

FAULT_NAMES=(segment_overspeed accel_spike dropped_config corner_cut toolspeed_dip wrong_limits_sha t_s_gap)
UNCAUGHT=()

for name in "${FAULT_NAMES[@]}"; do
    fpath="$FAULTS_DIR/fault_$name.json"
    [ -f "$fpath" ] || { UNCAUGHT+=("$name: fault file was not written"); continue; }

    fout="$FAULTS_DIR/fault_$name.check.json"
    "$REPO_ROOT/env.sh" python -m crackvision.trajectory_check \
        --trajectory "$fpath" --plan "$PLAN" \
        --limits "$LIMITS" --end-effector-config "$END_EFFECTOR" \
        --execution-config "$EXECUTION_CONFIG" --retime-config "$RETIME_CONFIG" \
        --out "$fout" -q
    check_rc=$?
    if [ "$check_rc" -ne 3 ]; then
        UNCAUGHT+=("$name: trajectory_check exit=$check_rc, expected 3 (see $fout)")
        continue
    fi
    echo "test_retime_crosscheck.sh: fault '$name' caught by trajectory_check (exit 3)"

    if [ "${GATE_MUST_FLAG[$name]:-0}" -eq 1 ]; then
        gout="$FAULTS_DIR/fault_$name.gate.txt"
        evaluate_gate "$fpath" >"$gout" || { UNCAUGHT+=("$name: execution_gate crashed, see $gout"); continue; }
        line=$(grep "^G-LIMITS=" "$gout")
        outcome="${line#*=}"; outcome="${outcome%%:*}"
        if [ "$outcome" != "fail" ]; then
            UNCAUGHT+=("$name: G-LIMITS reported '$outcome', expected fail (see $gout)")
            continue
        fi
        echo "test_retime_crosscheck.sh: fault '$name' independently flagged by G-LIMITS"
    fi
done

if [ "${#UNCAUGHT[@]}" -gt 0 ]; then
    for msg in "${UNCAUGHT[@]}"; do
        echo "test_retime_crosscheck.sh: UNCAUGHT FAULT: $msg" >&2
    done
    fail "${#UNCAUGHT[@]} injected fault(s) went uncaught"
fi

echo "test_retime_crosscheck.sh: OK ($WORK)"
exit 0
