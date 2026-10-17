#!/usr/bin/env bash
# estop_poll.sh: MOT-09 B5 (unloaded e-stop test) helper. READ-ONLY.
#
# Repeats `motorbridge-cli scan` over motor IDs 1-7 and logs how many motors answer, with UTC timestamps. The scan
# reads only the ESC_ID/MST_ID registers; nothing is enabled, commanded or written. It stops once it has seen the
# motors drop out (0 answer: the switch cut motor power) and come back (7/7), or at the timeout.
#
# Usage: estop_poll.sh [timeout_s]     default 90; log in runs/estop_poll_<UTC>.log
# Exit:  0 = drop-out and return observed, 1 = not observed before the timeout
# MB_CLI overrides the CLI path (offline tests only).
set -u
here=$(cd "$(dirname "$0")" && pwd)
timeout_s=${1:-90}
cli=${MB_CLI:-$HOME/miniconda3/envs/rebot/bin/motorbridge-cli}
mkdir -p "$here/runs"
log="$here/runs/estop_poll_$(date -u +%Y%m%dT%H%M%SZ).log"
phase=on            # on -> back (after a 0/7 scan) -> done (after a 7/7 scan)
t0=$(date +%s)
echo "# estop_poll.sh: read-only scans of IDs 1-7 until the motors drop out and return (timeout ${timeout_s} s)" | tee "$log"
while (( $(date +%s) - t0 < timeout_s )); do
  out=$(env -u PYTHONPATH "$cli" scan --vendor damiao --transport dm-serial --serial-port /dev/ttyACM0 \
          --serial-baud 921600 --start-id 1 --end-id 7 2>&1)
  n=$(grep -c '^\[hit\]' <<<"$out")
  other=$(grep -v -E '^(\[hit\]|\[\.\. \]|\[scan:|command=)' <<<"$out" | head -2 | tr '\n' ' ')
  echo "$(date -u +%FT%T.%3NZ) answered=$n/7 phase=$phase${other:+ cli: $other}" | tee -a "$log"
  if [[ $phase == on ]] && (( n == 0 )); then
    phase=back
    echo "# power cut observed: no motor answers" | tee -a "$log"
  elif [[ $phase == back ]] && (( n == 7 )); then
    echo "# power restored: 7/7 answer" | tee -a "$log"
    echo "B5 OBSERVED" | tee -a "$log"
    exit 0
  fi
done
echo "# timeout: B5 NOT observed (phase=$phase)" | tee -a "$log"
exit 1
