#!/usr/bin/env bash
# scripts/safety/run_safety_case.sh — INT-03.5: unified failure-injection runner.
#
# Runs the full mock/dry safety-case suite in one command, in order:
#   1. scripts/perc/test_safety_empty_mask.sh      (empty mask)
#   2. scripts/perc/test_safety_invalid_depth.sh   (invalid depth)
#   3. scripts/ros/test_safety_unreachable.sh      (unreachable path)
#   4. scripts/ros/test_execution.sh               (stale calibration/config + limit violations)
#
# Stops at the first failing stage and reports which one failed. Never sets
# CRACKVISION_ARM_REAL and never touches real hardware (see docs/integration/SAFETY_CASE.md).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

unset CRACKVISION_ARM_REAL

if [ ! -d "$REPO_ROOT/ros2_ws/install" ]; then
    echo "run_safety_case.sh: ros2_ws/install missing, building workspace first"
    bash "$REPO_ROOT/scripts/ros/build_ws.sh" || {
        echo "run_safety_case.sh: FAIL: scripts/ros/build_ws.sh failed" >&2
        exit 1
    }
fi

declare -a LABELS=(
    "empty_mask"
    "invalid_depth"
    "unreachable_path"
    "stale_calibration_and_limits"
)
declare -a SCRIPTS=(
    "scripts/perc/test_safety_empty_mask.sh"
    "scripts/perc/test_safety_invalid_depth.sh"
    "scripts/ros/test_safety_unreachable.sh"
    "scripts/ros/test_execution.sh"
)
declare -a RESULTS=()

N=${#SCRIPTS[@]}
for ((i = 0; i < N; i++)); do
    label="${LABELS[$i]}"
    rel="${SCRIPTS[$i]}"
    echo "=============================================================================="
    echo "run_safety_case.sh: stage $((i + 1))/$N: $label ($rel)"
    echo "=============================================================================="
    bash "$REPO_ROOT/$rel"
    rc=$?
    if [ "$rc" -ne 0 ]; then
        echo "run_safety_case.sh: FAIL: stage $label ($rel) exited $rc" >&2
        echo "" >&2
        echo "Safety case summary:" >&2
        for ((j = 0; j < i; j++)); do
            echo "  PASS  ${LABELS[$j]}" >&2
        done
        echo "  FAIL  $label" >&2
        for ((j = i + 1; j < N; j++)); do
            echo "  SKIP  ${LABELS[$j]}" >&2
        done
        exit 1
    fi
    RESULTS+=("$label")
    echo "run_safety_case.sh: stage $label: PASS"
done

echo "=============================================================================="
echo "run_safety_case.sh: safety case summary"
echo "=============================================================================="
for label in "${RESULTS[@]}"; do
    echo "PASS  $label"
done
echo "run_safety_case.sh: all $N fault-class stages PASS"
exit 0
