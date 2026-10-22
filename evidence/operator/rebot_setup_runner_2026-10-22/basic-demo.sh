#!/usr/bin/env bash
# basic-demo.sh [speed]   (speed 1..2, default 2)
#
# 1. enables all 7 motors, moves every joint to zero (the fold, J1 forward) and HOLDS
# 2. press Enter (or the Claude session writes runs/DEMO_GO) -> the "show" demo runs, ends folded facing the
#    audience, and every motor is switched off
#    type "stop" + Enter, or Ctrl-C (once), at any time -> the arm parks back to the fold and switches off
#    no go within 5 minutes -> switched off at the fold
#
# J1 forgets its turn count when the 24 V is cycled. The J1 reference is kept in j1_reference.json; if J1 reads
# neither forward nor the audience direction relative to it, the script asks you to turn the base to FORWARD by
# hand and confirm before anything is enabled.
#
# The 24 V switch is the e-stop: cutting it while the arm is raised DROPS the arm.
set -u
cd "$(dirname "$(readlink -f "$0")")"
SPEED=${1:-2}
PY=$HOME/miniconda3/envs/rebot/bin/python
REF=${DEMO_REF:-$PWD/j1_reference.json}
GO=$PWD/runs/DEMO_GO
LOG=$PWD/runs/demo_live.log
mkdir -p runs

echo "basic-demo: show at ${SPEED}x. Base clamped? Area clear? Hand near the 24 V switch."

start_demo() {   # extra args -> demo_move.py; sets $pid
  : > "$LOG"
  env -u PYTHONPATH "$PY" -u demo_move.py --operator-go "operator ran basic-demo.sh" --choreo show \
      --speed "$SPEED" --wait-go "$GO" --j1-ref-file "$REF" ${DEMO_EXTRA:-} "$@" > "$LOG" 2>&1 &
  pid=$!
}

trap 'echo; echo "(Ctrl-C passed to the demo: it parks to the fold and switches off)"' INT

start_demo
tail -n +1 -f "$LOG" --pid="$pid" 2>/dev/null | grep --line-buffered -vE "^  (segment|j[0-9]:)" &
tailpid=$!

# wait until it holds at zero, or exits (refusal)
until grep -q "HOLDING; waiting" "$LOG" 2>/dev/null || ! kill -0 "$pid" 2>/dev/null; do sleep 0.2; done

if ! kill -0 "$pid" 2>/dev/null; then
  wait "$pid"; rc=$?
  if [ "$rc" -eq 4 ]; then
    kill "$tailpid" 2>/dev/null
    echo
    echo "J1 reference unknown (after a power cycle J1 loses its turn count)."
    read -r -p "Turn the base by hand to face FORWARD (the original zero direction). Facing forward now? type yes: " ans
    [ "$ans" = "yes" ] || { echo "not confirmed; nothing was enabled."; exit 4; }
    start_demo --j1-here-is-zero
    tail -n +1 -f "$LOG" --pid="$pid" 2>/dev/null | grep --line-buffered -vE "^  (segment|j[0-9]:)" &
    tailpid=$!
    until grep -q "HOLDING; waiting" "$LOG" 2>/dev/null || ! kill -0 "$pid" 2>/dev/null; do sleep 0.2; done
  fi
fi

if kill -0 "$pid" 2>/dev/null; then
  echo
  echo ">>> Holding at zero. Press Enter to run the show, or type stop + Enter to switch off."
  while kill -0 "$pid" 2>/dev/null && ! grep -q "second go received" "$LOG"; do
    if read -r -t 1 line; then
      if [ "$line" = "stop" ]; then kill -INT "$pid"; break; fi
      echo "Enter (basic-demo.sh)" > "$GO"
      break
    fi
  done
fi
wait "$pid" 2>/dev/null; rc=$?
sleep 0.3; kill "$tailpid" 2>/dev/null
echo
grep -E "^outcome|DISABLED \(confirmed\)|WARNING" "$LOG" || true
exit "$rc"
