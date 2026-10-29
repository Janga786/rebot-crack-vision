#!/usr/bin/env bash
# scripts/crack_pipeline.sh — camera sees a crack -> the arm traces it (reBot B601-DM + wrist D405).
#
#   search-plan               the search sweep: the camera, looking straight down, visits 6 stops over the table
#                             (x 0.26/0.36, y -0.15/0/0.15, 25 cm up); rehearsed in simulation with arm_guard
#   search CASE [slow]        ARM + CAMERA: run-trajectory.sh plays the sweep; crack_trace search-capture runs the crack
#                             model at every stop and locates the crack (data/crack_trace/CASE_search/)
#   scan-plan --from-search CASE   the scan pose centred on the crack the search found
#   scan-plan [crack_trace scan-plan args]   the scan pose (camera 25 cm above the target, default table x=0.30)
#                                           -> data/crack_trace/scan.json, rehearsed in simulation with arm_guard
#   scan CASE [slow]          ARM + CAMERA: run-trajectory.sh moves the arm to the scan pose and holds it; meanwhile
#                             crack_trace capture waits for the arm to settle, captures, and runs the crack model.
#                             You press Enter at the fold like any run-trajectory.sh run. The arm returns to the fold.
#   plan CASE [plan args]     path3d -> the trace trajectory (the best accepted crack; --all-paths: every crack) +
#                             preview image, rehearsed in simulation (x1 and x2)
#   trace CASE [slow]         ARM: run-trajectory.sh plays data/crack_trace/CASE_trace.json (approach, trace, fold)
#   demo [IMAGE [X Y]]        no hardware: the whole chain with IMAGE printed 20 cm wide at (X, Y) on the table (default
#                             crack test print #1 at 0.34 -0.12): search-plan, rendered camera views, the sweep with
#                             the arm state replayed in real time (tools/replay_arm_state.py, no motor) and the model
#                             run at every stop, scan-plan --from-search, scan + capture, plan, guarded rehearsals.
#
# The first run on the powered arm (docs/motion/FIRST_CRACK_RUN.md; logic in src/crackvision/first_run.py):
#   preflight                 go/no-go at the desk: env, model, guard approval + hooks, runner sha256, camera free +
#                             D405 USB 3 + a validated capture, stale files cleaned, disk, search.json/scan.json
#                             regenerated and rehearsed x1 and x2, print PDF, crack gate on -> data/crack_trace/
#                             preflight_<UTC>.json; exit 0 only with no FAIL. Nothing moves.
#   bench [CASE]              CAMERA, ARM OFF: the print under the camera now, the crack gate, a plain-words component
#                             table + overlay + verdict (data/crack_trace/bench/CASE/). bench --live: the filtered
#                             live view (http://127.0.0.1:8090, Ctrl-C to stop) for aiming.
#   session CASE [slow]       search -> scan-plan --from-search -> scan -> plan -> preview -> trace, slow 2 and the kd 4
#                             override by default (DEMO_EXTRA wins if set), a typed yes before every step, stops on any
#                             failure. Calls only the subcommands above; every arm step is run-trajectory.sh's.
#   archive CASE              copies the case, the runner's run files, the preflight and the guard events into
#                             evidence/operator/first-crack-run-<date>/ with sha256s and a README to fill in.
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
  search-plan)
    "${CT[@]}" search-plan --out "$OUT/search.json" "$@" || die "search-plan failed"
    echo "rehearsal (simulation, real guard config):"
    rehearse "$OUT/search.json" 1 && rehearse "$OUT/search.json" 2 || exit 1 ;;

  search)
    CASE=${1:?usage: search CASE [slow]}; SLOW=${2:-1}
    [ -f "$OUT/search.json" ] || die "no $OUT/search.json: run search-plan first"
    mkdir -p "$RUNNER/runs" "$OUT"; rm -f "$STATE"
    "${CT[@]}" -q search-capture --case "$CASE" --search "$OUT/search.json" --state-file "$STATE" \
        > "$OUT/${CASE}_search.log" 2>&1 &
    cap=$!
    echo "search-capture waiting in the background (log $OUT/${CASE}_search.log)"
    DEMO_EXTRA="--state-file $STATE ${DEMO_EXTRA:-}" "$RUNNER/run-trajectory.sh" "$OUT/search.json" "$SLOW"
    rrc=$?
    if kill -0 "$cap" 2>/dev/null; then sleep 3; kill "$cap" 2>/dev/null; fi
    wait "$cap"; crc=$?
    grep -E "^stop|^search" "$OUT/${CASE}_search.log"
    [ "$rrc" -eq 0 ] || echo "the arm run ended with exit $rrc"
    [ "$crc" -eq 0 ] && echo "next: $0 scan-plan --from-search $CASE" || { echo "search exit $crc (see the log)"; exit "$crc"; } ;;

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
    PRINT_AT=(${2:-0.34} ${3:--0.12} 0.0)       # where the print lies on the table (off-centre on purpose)
    LIKE=$(ls -d "$REPO"/data/recordings/*/ 2>/dev/null | head -1)
    [ -f "$IMAGE" ] || die "no image $IMAGE"; [ -n "$LIKE" ] || die "no D405 recording under data/recordings (intrinsics)"
    CASE="demo_$(date -u +%Y%m%dT%H%M%SZ)"; W="$OUT/$CASE"; mkdir -p "$W"
    REPLAY=("$REPO/env.sh" python "$REPO/tools/replay_arm_state.py")
    echo "== 1. search sweep, print hidden at ${PRINT_AT[*]}"
    "${CT[@]}" search-plan --out "$W/search.json" || exit 1
    rehearse "$W/search.json" 1 || exit 1
    "${CT[@]}" make-replay --image "$IMAGE" --search "$W/search.json" --print-center "${PRINT_AT[@]}" --like "$LIKE" \
        --out "$W/search_replay" | tail -1
    echo "== 2. the sweep, played in real time (arm state replayed, no motor) while search-capture runs the model"
    "${CT[@]}" -q search-capture --case "${CASE}_s" --search "$W/search.json" --state-file "$W/arm_state.json" \
        --replay "$W/search_replay" --timeout-s 120 > "$W/search.log" 2>&1 &
    cap=$!; sleep 14
    "${REPLAY[@]}" "$W/search.json" "$W/arm_state.json" --lead-s 1
    wait "$cap"; crc=$?; grep -E "^stop|^search" "$W/search.log" | sed 's/^/  /'
    [ "$crc" -eq 0 ] || die "search failed (exit $crc): $W/search.log"
    echo "== 3. scan pose centred on what the search found"
    "${CT[@]}" scan-plan --from-search "${CASE}_s" --out "$W/scan.json" || exit 1
    rehearse "$W/scan.json" 1 || exit 1
    "${CT[@]}" make-replay --image "$IMAGE" --scan "$W/scan.json" --print-center "${PRINT_AT[@]}" --like "$LIKE" \
        --out "$W/replay" | tail -1
    echo "== 4. the scan, played in real time + capture + crack model"
    rm -f "$W/arm_state.json"
    "${CT[@]}" -q capture --case "$CASE" --scan "$W/scan.json" --state-file "$W/arm_state.json" --replay "$W/replay" \
        --timeout-s 120 > "$W/capture.log" 2>&1 &
    cap=$!; sleep 12
    "${REPLAY[@]}" "$W/scan.json" "$W/arm_state.json" --lead-s 1
    wait "$cap"; crc=$?; grep -vE "^\s*$" "$W/capture.log" | tail -3 | sed 's/^/  /'
    [ "$crc" -eq 0 ] || die "capture failed (exit $crc): $W/capture.log"
    echo "== 5. plan + rehearse"; "$0" plan "$CASE" || exit 1
    echo "demo OK: trajectory $OUT/${CASE}_trace.json, preview $OUT/${CASE}_trace_preview.png" ;;

  preflight|bench|session|archive)
    "$REPO/env.sh" python -m crackvision.first_run "$cmd" "$@" ;;

  *) sed -n '2,38p' "$0"; exit 2 ;;
esac
