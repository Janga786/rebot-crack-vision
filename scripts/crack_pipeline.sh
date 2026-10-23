#!/usr/bin/env bash
# scripts/crack_pipeline.sh — camera sees a crack -> the arm traces it (reBot B601-DM + wrist D405).
#
#   scan-plan [crack_trace scan-plan args]   the scan pose (camera 25 cm above the target, default table x=0.30)
#                                           -> data/crack_trace/scan.json, rehearsed in simulation with arm_guard
#   scan CASE [slow]          ARM + CAMERA: run-trajectory.sh moves the arm to the scan pose and holds it; meanwhile
#                             crack_trace capture waits for the arm to settle, captures, and runs the crack model.
#                             You press Enter at the fold like any run-trajectory.sh run. The arm returns to the fold.
#   plan CASE [plan args]     path3d -> the trace trajectory + preview image, rehearsed in simulation (x1 and x2)
#   trace CASE [slow]         ARM: run-trajectory.sh plays data/crack_trace/CASE_trace.json (approach, trace, fold)
#   demo [IMAGE]              no hardware: scan-plan, a rendered replay of IMAGE printed 20 cm wide on the table,
#                             a simulated scan run (demo_move --simulate --guard-sim publishing the arm state) with
#                             the capture running against it, plan, rehearse. Default IMAGE: crack test print #1.
#
# Every ARM step is the operator's: it goes through ~/rebot_setup/run-trajectory.sh (guard pre-flight, hold at the
# fold, Enter to go, stop/off). Extra runner flags via DEMO_EXTRA, e.g. DEMO_EXTRA="--gains j2=100/4,j3=80/4".
# Camera mount and tool tip are nominal (CAD prior) until GEOM-05/07: expect a few cm of offset; the planner hovers.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${REBOT_RUNNER:-$HOME/rebot_setup}"
PY="$HOME/miniconda3/envs/rebot/bin/python"
CT=("$REPO/env.sh" python -m crackvision.crack_trace)
OUT="$REPO/data/crack_trace"
SCAN="$OUT/scan.json"
STATE="$RUNNER/runs/arm_state.json"

die() { echo "crack_pipeline: $*" >&2; exit 1; }
rehearse() {   # TRAJ SLOW
  local out rc
  out=$("$RUNNER/run-trajectory.sh" --rehearse "$1" "$2" 2>&1); rc=$?
  echo "$out" | grep -E "^(pre-checks|arm_guard pre-flight|outcome)|REFUS|refus|GUARD FAULT" | sed 's/^/  /'
  echo "$out" | grep -q "^outcome: completed" && [ "$rc" -eq 0 ] || { echo "  rehearsal x$2 FAILED (exit $rc)"; return 1; }
}

cmd=${1:-}; shift || true
case "$cmd" in
  scan-plan)
    "${CT[@]}" scan-plan --out "$SCAN" "$@" || die "scan-plan failed"
    echo "rehearsal (simulation, real guard config):"; rehearse "$SCAN" 1 || exit 1 ;;

  scan)
    CASE=${1:?usage: scan CASE [slow]}; SLOW=${2:-1}
    [ -f "$SCAN" ] || die "no $SCAN: run scan-plan first"
    mkdir -p "$RUNNER/runs" "$OUT"
    "${CT[@]}" -q capture --case "$CASE" --scan "$SCAN" --state-file "$STATE" > "$OUT/${CASE}_capture.log" 2>&1 &
    cap=$!
    echo "capture waiting in the background (log $OUT/${CASE}_capture.log)"
    DEMO_EXTRA="--state-file $STATE ${DEMO_EXTRA:-}" "$RUNNER/run-trajectory.sh" "$SCAN" "$SLOW"
    rrc=$?
    if kill -0 "$cap" 2>/dev/null; then sleep 2; kill "$cap" 2>/dev/null; fi
    wait "$cap"; crc=$?
    cat "$OUT/${CASE}_capture.log" | grep -vE "^\s*$" | tail -5
    [ "$rrc" -eq 0 ] || echo "the arm run ended with exit $rrc"
    [ "$crc" -eq 0 ] && echo "next: $0 plan $CASE" || { echo "capture exit $crc (see the log)"; exit "$crc"; } ;;

  plan)
    CASE=${1:?usage: plan CASE [plan args]}; shift
    "${CT[@]}" plan --case "$CASE" "$@" || die "plan failed"
    echo "rehearsal (simulation, real guard config):"
    rehearse "$OUT/${CASE}_trace.json" 1 && rehearse "$OUT/${CASE}_trace.json" 2 || exit 1
    echo "look at the preview, then: $0 trace $CASE 2" ;;

  trace)
    CASE=${1:?usage: trace CASE [slow]}; SLOW=${2:-1}
    T="$OUT/${CASE}_trace.json"; [ -f "$T" ] || die "no $T: run plan first"
    "$RUNNER/run-trajectory.sh" "$T" "$SLOW" ;;

  demo)
    IMAGE=${1:-$REPO/data/crack_targets/images/01_crack.jpg}
    LIKE=$(ls -d "$REPO"/data/recordings/*/ 2>/dev/null | head -1)
    [ -f "$IMAGE" ] || die "no image $IMAGE"; [ -n "$LIKE" ] || die "no D405 recording under data/recordings (intrinsics)"
    CASE="demo_$(date -u +%Y%m%dT%H%M%SZ)"; W="$OUT/$CASE"; mkdir -p "$W"
    echo "== 1. scan pose"; "${CT[@]}" scan-plan --out "$W/scan.json" || exit 1
    echo "== 2. rendered camera view of $(basename "$IMAGE") on the table"
    "${CT[@]}" make-replay --image "$IMAGE" --scan "$W/scan.json" --like "$LIKE" --out "$W/replay" || exit 1
    echo "== 3. simulated scan run (real guard config) + capture against its published arm state"
    "${CT[@]}" -q capture --case "$CASE" --scan "$W/scan.json" --state-file "$W/arm_state.json" --replay "$W/replay" \
        --timeout-s 120 > "$W/capture.log" 2>&1 &
    cap=$!
    (cd "$RUNNER" && env -u PYTHONPATH "$PY" -u demo_move.py --simulate --guard-sim --operator-go "crack_pipeline demo" \
        --trajectory "$W/scan.json" --wait-go "$W/go" --sim-auto-go 1 --state-file "$W/arm_state.json") \
        | grep -E "^outcome|GUARD|refus" | sed 's/^/  runner: /'
    wait "$cap"; crc=$?; grep -vE "^\s*$" "$W/capture.log" | tail -3 | sed 's/^/  /'
    [ "$crc" -eq 0 ] || die "capture failed (exit $crc): $W/capture.log"
    echo "== 4. plan + rehearse"; "$0" plan "$CASE" || exit 1
    echo "demo OK: trajectory $OUT/${CASE}_trace.json, preview $OUT/${CASE}_trace_preview.png" ;;

  *) sed -n '2,21p' "$0"; exit 2 ;;
esac
