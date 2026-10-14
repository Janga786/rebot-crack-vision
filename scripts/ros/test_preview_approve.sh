#!/usr/bin/env bash
# scripts/ros/test_preview_approve.sh — MOT-08.8 headless mock run of the view->preview->approve->
# execute workflow (docs/INTERFACES.md §18, amending §11.5/§11.7 G-APPROVAL; §17 for the view-move
# trajectory itself): run_cartesian_synthetic.sh's own scratch view_plan (MOT-06.10) -> retime_plan
# --view-plan (§17.3) -> trajectory_check (independent re-check, §17.4) -> trajectory_preview
# (§18.1, deterministic headless render + manifest) -> approve_trajectory (§18.2, typed
# /dev/tty-direct APPROVE <sha8> via a pseudo-tty) -> execute_trajectory --mode dry --approval
# <file> on the moveit_mock profile, asserting G-APPROVAL pass in the execution record's
# gate_report. It then exercises the tamper/refusal paths §18.2/§18.3 define: a wrong approval
# phrase, a byte-tampered preview artefact, an approval bound to a different trajectory, and the
# unavoidable real-mode-without-commissioning refusal, modelled directly on
# scripts/ros/test_view_retime_execute.sh's MOT-07.10.5 pattern.
#
# The pseudo-tty is built with util-linux `script -qec ... /dev/null`, piping the typed phrase into
# its stdin: this is a TEST-ONLY way of driving approve_trajectory's /dev/tty-direct prompt from a
# non-interactive CI shell. It is not a bypass of the typed-confirmation rule (§18.2/ADR-020
# Decision §5) -- a human still has to type the exact phrase somewhere upstream of this piped
# input in any real operator workflow; this script only automates that keystroke for headless
# regression testing.
#
# No hardware, no vendor driver, no real mode, CRACKVISION_ARM_REAL never set: this script only
# ever starts the headless ros2_control mock hardware stack (scripts/ros/_mock_stack.sh,
# start_mock_stack/stop_mock_stack), which installs its own `trap stop_mock_stack EXIT INT TERM`,
# so the stack is torn down on every exit path, including a failure partway through.
#
# Scratch space only: everything this script writes lives under logs/preview_approve/<stamp>/ and
# the run_cartesian_synthetic.sh scratch root it reuses (logs/cartesian_synthetic/<stamp>/);
# nothing under data/ or config/ is ever written or modified.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
WORK="$REPO_ROOT/logs/preview_approve/$STAMP"
RECORD_DIR="$WORK/execution_records"
mkdir -p "$WORK" "$RECORD_DIR"

fail() {
    echo "test_preview_approve.sh: FAIL: $*" >&2
    exit 1
}

# -- guard: never run anywhere near a real vendor driver process (§0's truncated-comm pattern,
# mirroring test_execution.sh/test_retime_execute.sh/test_view_retime_execute.sh).
if pgrep -x reBotArmControl >/dev/null 2>&1; then
    echo "test_preview_approve.sh: a real vendor driver process (reBotArmControl) is running -- refusing to run" >&2
    exit 3
fi

# --------------------------------------------------------------------------------------
# 1. A fresh scratch root (MOT-06.10's own synthetic view_plan + cartesian_plan).
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 1: run_cartesian_synthetic.sh (fresh scratch plan) =="
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
echo "test_preview_approve.sh: scratch root $ROOT"

# --root is REPO_ROOT for every tool invocation below (conda-side and ROS-side alike), so every
# repo-relative path recorded in a plan/trajectory/preview/approval resolves consistently against
# the same root everywhere it is read back -- per the card body and docs/INTERFACES.md §11.2/§18.

# --------------------------------------------------------------------------------------
# 2. retime_plan --view-plan.
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 2: retime_plan --view-plan =="
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
    echo "test_preview_approve.sh: $OVERLAY_SETUP not found -- run scripts/ros/build_ws.sh first" >&2
    exit 3
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

RETIME_LOG="$WORK/retime_plan.log"
ros2 run crackvision_motion retime_plan --view-plan "$VIEW_PLAN" --root "$REPO_ROOT" \
    --limits "$SCRATCH_LIMITS" --end-effector-config "$SCRATCH_EE" \
    --execution-config "$EXECUTION_CONFIG" --retime-config "$RETIME_CONFIG" --urdf "$URDF" \
    --out "$TRAJECTORY" >"$RETIME_LOG" 2>&1
rc=$?
cat "$RETIME_LOG"
[ "$rc" -eq 0 ] || fail "retime_plan --view-plan exit=$rc (expected 0); see $RETIME_LOG"
[ -f "$TRAJECTORY" ] || fail "retime_plan exit 0 but $TRAJECTORY not written"
[ -f "$REPORT" ] || fail "retime_plan exit 0 but $REPORT (retime_report sidecar) not written"
echo "test_preview_approve.sh: retime_plan --view-plan OK -> $TRAJECTORY"

# --------------------------------------------------------------------------------------
# 3. trajectory_check (independent re-check, §17.4).
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 3: trajectory_check =="
CHECK_OUT="$WORK/trajectory_check.json"
"$REPO_ROOT/env.sh" python -m crackvision.trajectory_check \
    --trajectory "$TRAJECTORY" --plan "$VIEW_PLAN" --report "$REPORT" --root "$REPO_ROOT" \
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
    print(f"test_preview_approve.sh: check {check}={outcome}")
    if outcome not in ("pass", "skip"):
        sys.exit(f"check {check} outcome={outcome!r}, expected pass or skip")
' "$CHECK_OUT" || fail "trajectory_check reported a non-pass/skip check outcome"
echo "test_preview_approve.sh: trajectory_check OK"

# --------------------------------------------------------------------------------------
# 4. trajectory_preview (§18.1): deterministic headless render + manifest, written only under
#    $WORK (never data/previews/).
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 4: trajectory_preview =="
PREVIEW_DIR="$WORK/preview"
"$REPO_ROOT/env.sh" python -m crackvision.trajectory_preview \
    --trajectory "$TRAJECTORY" --plan "$VIEW_PLAN" --retime-report "$REPORT" --check-report "$CHECK_OUT" \
    --root "$REPO_ROOT" --limits "$SCRATCH_LIMITS" --end-effector-config "$SCRATCH_EE" \
    --scene-config "$SCENE_CONFIG" --execution-config "$EXECUTION_CONFIG" --urdf "$URDF" \
    --out-dir "$PREVIEW_DIR" -q
rc=$?
[ "$rc" -eq 0 ] || fail "trajectory_preview exit=$rc (expected 0)"
PREVIEW_MANIFEST="$PREVIEW_DIR/preview.json"
for f in "$PREVIEW_MANIFEST" "$PREVIEW_DIR/joints.png" "$PREVIEW_DIR/toolpath.png" "$PREVIEW_DIR/toolspeed.png"; do
    [ -f "$f" ] || fail "trajectory_preview exit 0 but $f not written"
done
SHA8="$(python3 -c '
import json
m = json.load(open("'"$PREVIEW_MANIFEST"'", encoding="utf-8"))
if m["schema"] != "crackvision.trajectory_preview/1":
    raise SystemExit("manifest schema mismatch: " + m["schema"])
if m["check_report"]["all_pass"] is not True:
    raise SystemExit("check_report.all_pass is not true: " + repr(m["check_report"]))
print(m["trajectory"]["sha256"][:8])
')" || fail "preview manifest invalid"
echo "test_preview_approve.sh: trajectory_preview OK -> $PREVIEW_DIR (sha8=$SHA8)"

# --------------------------------------------------------------------------------------
# 5. approve_trajectory (§18.2): typed APPROVE <sha8> via a pseudo-tty. A wrong phrase refuses
#    (exit 3, no file); the correct phrase writes the approval under $WORK (never data/approvals/).
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 5: approve_trajectory (wrong phrase must refuse) =="
WRONG_APPROVAL="$WORK/approval_wrongphrase.json"
APPROVE_CMD_BASE="\"$REPO_ROOT/env.sh\" python -m crackvision.approve_trajectory --preview \"$PREVIEW_MANIFEST\" --operator ci-mock --root \"$REPO_ROOT\" --limits \"$SCRATCH_LIMITS\" --end-effector-config \"$SCRATCH_EE\" --scene-config \"$SCENE_CONFIG\" --execution-config \"$EXECUTION_CONFIG\""
WRONG_LOG="$WORK/approve_wrongphrase.log"
printf 'APPROVE deadbeef\n' | script -qec "$APPROVE_CMD_BASE --out \"$WRONG_APPROVAL\"" /dev/null >"$WRONG_LOG" 2>&1
rc=$?
cat "$WRONG_LOG"
[ "$rc" -eq 3 ] || fail "wrong-phrase approve_trajectory exit=$rc (expected 3)"
[ -f "$WRONG_APPROVAL" ] && fail "wrong-phrase approve_trajectory wrote $WRONG_APPROVAL (expected no file)"
echo "test_preview_approve.sh: wrong phrase correctly refused (exit 3), no file written"

echo "test_preview_approve.sh: == step 5b: approve_trajectory (correct phrase) =="
APPROVAL="$WORK/approval.json"
APPROVE_LOG="$WORK/approve.log"
printf 'APPROVE %s\n' "$SHA8" | script -qec "$APPROVE_CMD_BASE --out \"$APPROVAL\"" /dev/null >"$APPROVE_LOG" 2>&1
rc=$?
cat "$APPROVE_LOG"
[ "$rc" -eq 0 ] || fail "approve_trajectory exit=$rc (expected 0); see $APPROVE_LOG"
[ -f "$APPROVAL" ] || fail "approve_trajectory exit 0 but $APPROVAL not written"
python3 -c '
import json
a = json.load(open("'"$APPROVAL"'", encoding="utf-8"))
assert a["schema"] == "crackvision.execution_approval/1", a["schema"]
assert len(a["preview_artifacts"]) == 4, a["preview_artifacts"]
' || fail "approval file malformed"
echo "test_preview_approve.sh: approve_trajectory OK -> $APPROVAL"

# --------------------------------------------------------------------------------------
# 6. headless mock stack + execute_trajectory --mode dry --approval <file>: exit 0, G-APPROVAL
#    pass in gate_report.
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 6: mock stack =="
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"
LAUNCH_LOG="$WORK/mock_planning_launch.log"
start_mock_stack "$LAUNCH_LOG" || exit 1
sleep 2

ros2 run crackvision_motion apply_scene --root "$REPO_ROOT" --config "$SCENE_CONFIG" >/dev/null \
    || fail "apply_scene (scratch scene) failed"

gate_outcome() {
    # $1 = record json path, $2 = gate id
    python3 -c '
import json, sys
record = json.load(open(sys.argv[1]))
outcomes = {c["gate"]: c["outcome"] for c in record["gate_report"]}
print(outcomes.get(sys.argv[2], ""))
' "$1" "$2"
}

echo "-- dry mode, genuine approval: must exit 0, G-APPROVAL pass --"
DRY_LOG="$WORK/execute_dry.log"
ros2 run crackvision_motion execute_trajectory --mode dry --profile moveit_mock \
    --trajectory "$TRAJECTORY" --root "$REPO_ROOT" --limits "$SCRATCH_LIMITS" \
    --end-effector-config "$SCRATCH_EE" --scene-config "$SCENE_CONFIG" \
    --execution-config "$EXECUTION_CONFIG" --approval "$APPROVAL" \
    --record-dir "$RECORD_DIR" >"$DRY_LOG" 2>&1
rc=$?
cat "$DRY_LOG"
[ "$rc" -eq 0 ] || fail "dry mode exit=$rc (expected 0); see $DRY_LOG"
DRY_REC="$(ls -t "$RECORD_DIR"/*.json | head -n1)"
GOT="$(gate_outcome "$DRY_REC" G-APPROVAL)"
[ "$GOT" = "pass" ] || fail "G-APPROVAL outcome=$GOT, expected pass (genuine approval, dry mode): $DRY_REC"
echo "test_preview_approve.sh: dry mode OK, G-APPROVAL pass"

# --------------------------------------------------------------------------------------
# 7. Tamper case A: byte-modified copy of toolpath.png in a copied preview dir -> G-APPROVAL warn.
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 7: tampered preview artefact (G-APPROVAL must warn) =="
PREVIEW_TAMPERED="$WORK/preview_tampered"
cp -r "$PREVIEW_DIR" "$PREVIEW_TAMPERED"
python3 -c '
f = "'"$PREVIEW_TAMPERED"'/toolpath.png"
with open(f, "r+b") as fh:
    fh.seek(100)
    b = fh.read(1)
    fh.seek(100)
    fh.write(bytes([b[0] ^ 0xFF]))
' || fail "could not tamper toolpath.png"

APPROVAL_TAMPERED_PNG="$WORK/approval_tampered_png.json"
python3 -c '
import json, os, sys
approval_path, orig_dir, tampered_dir, out_path, repo_root = sys.argv[1:6]
doc = json.load(open(approval_path, encoding="utf-8"))
tampered_rel = os.path.relpath(tampered_dir, repo_root)
for entry in doc["preview_artifacts"]:
    name = os.path.basename(entry["path"])
    entry["path"] = f"{tampered_rel}/{name}"
json.dump(doc, open(out_path, "w", encoding="utf-8"), indent=2)
' "$APPROVAL" "$PREVIEW_DIR" "$PREVIEW_TAMPERED" "$APPROVAL_TAMPERED_PNG" "$REPO_ROOT" \
    || fail "could not build tampered-artefact approval copy"

TAMPER_PNG_LOG="$WORK/execute_tampered_png.log"
ros2 run crackvision_motion execute_trajectory --mode dry --profile moveit_mock \
    --trajectory "$TRAJECTORY" --root "$REPO_ROOT" --limits "$SCRATCH_LIMITS" \
    --end-effector-config "$SCRATCH_EE" --scene-config "$SCENE_CONFIG" \
    --execution-config "$EXECUTION_CONFIG" --approval "$APPROVAL_TAMPERED_PNG" \
    --record-dir "$RECORD_DIR" >"$TAMPER_PNG_LOG" 2>&1
rc=$?
cat "$TAMPER_PNG_LOG"
[ "$rc" -eq 0 ] || fail "tampered-artefact dry mode exit=$rc (expected 0); see $TAMPER_PNG_LOG"
TAMPER_PNG_REC="$(ls -t "$RECORD_DIR"/*.json | head -n1)"
GOT="$(gate_outcome "$TAMPER_PNG_REC" G-APPROVAL)"
[ "$GOT" = "warn" ] || fail "G-APPROVAL outcome=$GOT, expected warn (tampered artefact, dry mode): $TAMPER_PNG_REC"
echo "test_preview_approve.sh: tampered artefact correctly warned at G-APPROVAL (dry mode)"

# --------------------------------------------------------------------------------------
# 8. Tamper case B: an approval for a different trajectory -> G-APPROVAL warn.
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 8: approval bound to a different trajectory (G-APPROVAL must warn) =="
DIFFERENT_TRAJECTORY="$WORK/different_trajectory.json"
python3 -c '
import json
doc = json.load(open("'"$TRAJECTORY"'", encoding="utf-8"))
doc["created_utc"] = "2000-01-01T00:00:00Z" if doc["created_utc"] != "2000-01-01T00:00:00Z" else "2000-01-01T00:00:01Z"
json.dump(doc, open("'"$DIFFERENT_TRAJECTORY"'", "w", encoding="utf-8"), indent=2)
' || fail "could not build a different-trajectory copy"

DIFF_LOG="$WORK/execute_diff_trajectory.log"
ros2 run crackvision_motion execute_trajectory --mode dry --profile moveit_mock \
    --trajectory "$DIFFERENT_TRAJECTORY" --root "$REPO_ROOT" --limits "$SCRATCH_LIMITS" \
    --end-effector-config "$SCRATCH_EE" --scene-config "$SCENE_CONFIG" \
    --execution-config "$EXECUTION_CONFIG" --approval "$APPROVAL" \
    --record-dir "$RECORD_DIR" >"$DIFF_LOG" 2>&1
rc=$?
cat "$DIFF_LOG"
[ "$rc" -eq 0 ] || fail "different-trajectory dry mode exit=$rc (expected 0); see $DIFF_LOG"
DIFF_REC="$(ls -t "$RECORD_DIR"/*.json | head -n1)"
GOT="$(gate_outcome "$DIFF_REC" G-APPROVAL)"
[ "$GOT" = "warn" ] || fail "G-APPROVAL outcome=$GOT, expected warn (different trajectory, dry mode): $DIFF_REC"
echo "test_preview_approve.sh: approval for a different trajectory correctly warned at G-APPROVAL"

# --------------------------------------------------------------------------------------
# 9. --dry-run --mode real with the genuine approval: offline-only, writes nothing, exits 0
#    regardless, G-APPROVAL pass.
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 9: --dry-run --mode real (genuine approval) =="
DRYRUN_REAL_LOG="$WORK/execute_dryrun_real.log"
ros2 run crackvision_motion execute_trajectory --dry-run --mode real \
    --trajectory "$TRAJECTORY" --root "$REPO_ROOT" --limits "$SCRATCH_LIMITS" \
    --end-effector-config "$SCRATCH_EE" --scene-config "$SCENE_CONFIG" \
    --execution-config "$EXECUTION_CONFIG" --approval "$APPROVAL" >"$DRYRUN_REAL_LOG" 2>&1
rc=$?
cat "$DRYRUN_REAL_LOG"
[ "$rc" -eq 0 ] || fail "--dry-run --mode real exit=$rc (expected 0); see $DRYRUN_REAL_LOG"
GOT="$(python3 -c '
import json
report = json.load(open("'"$DRYRUN_REAL_LOG"'", encoding="utf-8"))
outcomes = {c["gate"]: c["outcome"] for c in report["gate_report"]}
print(outcomes.get("G-APPROVAL", ""))
')"
[ "$GOT" = "pass" ] || fail "--dry-run --mode real: G-APPROVAL outcome=$GOT, expected pass"
echo "test_preview_approve.sh: --dry-run --mode real OK, G-APPROVAL pass"

# --------------------------------------------------------------------------------------
# 10. --mode real without CRACKVISION_ARM_REAL set: must refuse, exit 3 (G-ARM, §11.7). This
#     script never sets CRACKVISION_ARM_REAL anywhere -- this is the unavoidable refusal any
#     real-mode attempt gets, not a bypass of it.
# --------------------------------------------------------------------------------------

echo "test_preview_approve.sh: == step 10: real mode (no CRACKVISION_ARM_REAL): must refuse =="
REAL_LOG="$WORK/execute_real.log"
env -u CRACKVISION_ARM_REAL ros2 run crackvision_motion execute_trajectory --mode real \
    --trajectory "$TRAJECTORY" --root "$REPO_ROOT" --limits "$SCRATCH_LIMITS" \
    --end-effector-config "$SCRATCH_EE" --scene-config "$SCENE_CONFIG" \
    --execution-config "$EXECUTION_CONFIG" --approval "$APPROVAL" \
    --record-dir "$RECORD_DIR" >"$REAL_LOG" 2>&1
rc=$?
cat "$REAL_LOG"
[ "$rc" -eq 3 ] || fail "real mode exit=$rc (expected 3, G-ARM refusal); see $REAL_LOG"
echo "test_preview_approve.sh: real mode correctly refused (exit 3)"

stop_mock_stack

sleep 1
STRAY=$(ps -eo pid=,comm= | grep -E "move_group|ros2_control_node|robot_state_publisher" || true)
if [ -n "$STRAY" ]; then
    echo "test_preview_approve.sh: stray process(es) remain after teardown:" >&2
    echo "$STRAY" >&2
    exit 1
fi

echo "test_preview_approve.sh: PASS ($WORK)"
exit 0
