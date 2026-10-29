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
# 2. press Enter (an EMPTY line) at the prompt -> approach from the fold, the trajectory, return to the fold, every
#    motor switched off. Only an empty line typed AFTER the prompt starts it: anything else that is not stop/off is
#    ignored ("type Enter to run, stop, or off"). Lines typed before the prompt (while it moves to zero): stop or
#    off is acted on as soon as it holds at zero (stop = park + off, off = off at the fold); an Enter or anything
#    else is discarded.
#    type stop + Enter, or Ctrl-C (once), at any time -> the arm parks back to the fold and switches off
#    (Ctrl-C before anything is enabled -> refused, nothing enabled; Ctrl-C at the J1 "type yes" question -> not
#    confirmed, exit 4, nothing enabled)
#    type off + Enter at the hold -> switched off at the fold now
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
      --trajectory "$TRAJ" --slow "$SLOW" --wait-go "$SIMGO" --sim-auto-go 2 ${DEMO_EXTRA:-} | grep -vE "^  j[0-9]:"
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

INTD=0     # a Ctrl-C was pressed (the J1 question checks it; demo_move.py gets the SIGINT itself)
on_int() {
  INTD=1
  echo
  [ "${PHASE:-run}" = j1 ] && return
  echo "(Ctrl-C passed on: before enabling = refused, nothing enabled; moving/holding = park to the fold and switch off;"
  echo " after a guard fault = reset + park)"
}
trap on_int INT

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
    # nothing runs now: a Ctrl-C here must end this question (read in 1 s slices so the trap's flag is seen)
    PHASE=j1
    printf "Turn the base by hand to face FORWARD (the original zero direction). Facing forward now? type yes: "
    ans=""
    while :; do
      if [ "$INTD" -eq 1 ]; then echo "Ctrl-C: not confirmed; nothing was enabled."; exit 4; fi
      read -r -t 1 ans; rrc=$?
      [ "$rrc" -eq 0 ] && break
      [ "$rrc" -gt 128 ] || { ans=""; break; }     # > 128 = the 1 s slice ran out (or a signal): ask on; else EOF
    done
    PHASE=run
    [ "$ans" = "yes" ] || { echo "not confirmed; nothing was enabled."; exit 4; }
    start_run --j1-here-is-zero
    tail -n +1 -f "$LOG" --pid="$pid" 2>/dev/null | grep --line-buffered -vE "^  (segment|j[0-9]:)" &
    tailpid=$!
    until grep -q "HOLDING; waiting" "$LOG" 2>/dev/null || ! kill -0 "$pid" 2>/dev/null; do sleep 0.2; done
  fi
fi

if kill -0 "$pid" 2>/dev/null; then
  echo
  # lines typed before the prompt (while the arm moved to zero): a stray Enter (or anything else) must not start
  # the run and is dropped; stop / off typed early is acted on now
  early=""
  while read -r -t 0.05 line; do
    case "$line" in
      stop|off) early=$line; break ;;
    esac
  done
  case "$early" in
    stop) kill -INT "$pid" 2>/dev/null && echo "(stop typed while moving to zero: sent; parking to the fold)"
          while read -r -t 0.05 _; do :; done ;;   # a second early line must not become a second Ctrl-C (= off now)
    off) echo "off (run-trajectory.sh)" > "$OFF" && echo "(off typed while moving to zero: sent)"
          while read -r -t 0.05 _; do :; done ;;
    *)
      echo ">>> Holding at zero. Press Enter to run the trajectory, or type stop + Enter to switch off."
      while kill -0 "$pid" 2>/dev/null && ! grep -q "second go received" "$LOG"; do
        if read -r -t 1 line; then
          case "$line" in
            "") echo "Enter (run-trajectory.sh)" > "$GO"; break ;;
            stop) kill -INT "$pid"; break ;;
            off) echo "off (run-trajectory.sh)" > "$OFF" && echo "(off sent)"; break ;;
            *) echo "type Enter to run, stop, or off" ;;
          esac
        fi
      done ;;
  esac
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
