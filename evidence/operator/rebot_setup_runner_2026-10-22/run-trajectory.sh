#!/usr/bin/env bash
# run-trajectory.sh TRAJECTORY.json [slow]             run a joint trajectory on the arm, arm_guard in front of it
# run-trajectory.sh --check TRAJECTORY.json [slow]     read-only: bus pre-checks + path check + arm_guard pre-flight;
#                                                      enables nothing (it does open the bus and read registers)
# run-trajectory.sh --rehearse TRAJECTORY.json [slow]  the whole run in simulation, real guard config, no hardware
#
# TRAJECTORY.json = a crackvision.joint_trajectory/1 file (trajectories/*.json, or the repo planner's output).
# slow = 1 (the file's own timing) or more (that many times slower). Default 1.
#
# 1. enables all 7 motors, moves every joint to zero (the fold) and HOLDS
# 2. press Enter -> approach from the fold, the trajectory, return to the fold, every motor switched off
#    type stop + Enter, or Ctrl-C (once), at any time -> the arm parks back to the fold and switches off
#    no go within 5 minutes -> switched off at the fold
# 3. if arm_guard stops the arm, it HOLDS where it is (nothing is switched off) and waits for you:
#    stop + Enter (or Ctrl-C) -> the guard is reset (logged) and the arm parks slowly to the fold, then switches off
#    stop + Enter again during that park -> switched off immediately: the arm DROPS
#    off + Enter, at any time -> switched off immediately (also when every park attempt faults again): support the
#    arm first, it DROPS
#
# J1 forgets its turn count when the 24 V is cycled. The J1 reference is kept in j1_reference.json (shared with
# basic-demo.sh); if J1 reads neither forward nor the audience direction relative to it, you are asked to turn the
# base to FORWARD by hand and confirm before anything is enabled.
#
# The 24 V switch is the e-stop: cutting it while the arm is raised DROPS the arm.
set -u
cd "$(dirname "$(readlink -f "$0")")"
MODE=run
case "${1:-}" in
  --check) MODE=check; shift ;;
  --rehearse) MODE=rehearse; shift ;;
esac
TRAJ=${1:-}
SLOW=${2:-1}
if [ -z "$TRAJ" ] || [ ! -f "$TRAJ" ]; then
  echo "usage: $0 [--check|--rehearse] TRAJECTORY.json [slow]   (see the header of this script)"; exit 2
fi
TRAJ=$(readlink -f "$TRAJ")
PY=$HOME/miniconda3/envs/rebot/bin/python
REF=${DEMO_REF:-$PWD/j1_reference.json}
GO=$PWD/runs/DEMO_GO
OFF=$PWD/runs/DEMO_OFF
LOG=$PWD/runs/trajectory_live.log
mkdir -p runs

if [ "$MODE" = rehearse ]; then
  SIMGO=$(mktemp -u /tmp/rehearse_go.XXXXXX)
  env -u PYTHONPATH "$PY" -u demo_move.py --simulate --guard-sim --operator-go "rehearsal (run-trajectory.sh)" \
      --trajectory "$TRAJ" --slow "$SLOW" --wait-go "$SIMGO" --sim-auto-go 2 | grep -vE "^  j[0-9]:"
  rc=${PIPESTATUS[0]}; rm -f "$SIMGO"; exit "$rc"
fi

if [ "$MODE" = check ]; then
  env -u PYTHONPATH "$PY" -u demo_move.py --check-only --trajectory "$TRAJ" --slow "$SLOW" --wait-go "$GO" \
      --j1-ref-file "$REF" ${DEMO_EXTRA:-}
  rc=$?
  [ "$rc" -eq 4 ] && echo "J1 reference unknown: turn the base to FORWARD by hand, then run without --check (it asks)."
  exit "$rc"
fi

echo "run-trajectory: $(basename "$TRAJ") at 1/${SLOW} speed. Base clamped? Area clear? Hand near the 24 V switch."

start_run() {   # extra args -> demo_move.py; sets $pid
  : > "$LOG"
  env -u PYTHONPATH "$PY" -u demo_move.py --operator-go "operator ran run-trajectory.sh" \
      --trajectory "$TRAJ" --slow "$SLOW" --wait-go "$GO" --j1-ref-file "$REF" ${DEMO_EXTRA:-} "$@" > "$LOG" 2>&1 &
  pid=$!
}

trap 'echo; echo "(Ctrl-C passed on: park to the fold and switch off; after a guard fault: reset + park)"' INT

start_run
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
    start_run --j1-here-is-zero
    tail -n +1 -f "$LOG" --pid="$pid" 2>/dev/null | grep --line-buffered -vE "^  (segment|j[0-9]:)" &
    tailpid=$!
    until grep -q "HOLDING; waiting" "$LOG" 2>/dev/null || ! kill -0 "$pid" 2>/dev/null; do sleep 0.2; done
  fi
fi

if kill -0 "$pid" 2>/dev/null; then
  echo
  echo ">>> Holding at zero. Press Enter to run the trajectory, or type stop + Enter to switch off."
  while kill -0 "$pid" 2>/dev/null && ! grep -q "second go received" "$LOG"; do
    if read -r -t 1 line; then
      if [ "$line" = "stop" ]; then kill -INT "$pid"; break; fi
      echo "Enter (run-trajectory.sh)" > "$GO"
      break
    fi
  done
  # while it runs (or holds after a guard fault): stop + Enter = Ctrl-C for the run, off + Enter = switch off now
  echo ">>> stop + Enter = park to the fold and switch off.  off + Enter = switch off NOW (support the arm: it drops)."
  while kill -0 "$pid" 2>/dev/null; do
    if read -r -t 1 line; then
      case "$line" in
        stop) kill -INT "$pid" 2>/dev/null && echo "(stop sent)" ;;
        off) echo "off (run-trajectory.sh)" > "$OFF" && echo "(off sent)" ;;
      esac
    fi
  done
fi
wait "$pid" 2>/dev/null; rc=$?
sleep 0.3; kill "$tailpid" 2>/dev/null
echo
grep -E "^outcome|DISABLED \(confirmed\)|WARNING|GUARD FAULT" "$LOG" || true
exit "$rc"
