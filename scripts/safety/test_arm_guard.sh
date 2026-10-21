#!/usr/bin/env bash
# scripts/safety/test_arm_guard.sh — arm_guard, the last-line motor-command filter (docs/motion/ARM_GUARD.md).
#
# 1. the guard config loads and matches its approval record
# 2. the full test suite: rules, fake-bus runtime, closed-loop fuzz against a hostile planner, replay of every
#    real run logged on the arm, the interpreter-start hook, and the installed motorbridge's method allowlist
# 3. the hook is installed and active in every env that can reach the motors (rebot, lerobot)
# Never opens a bus and never touches hardware.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PKG="$REPO_ROOT/packages/arm_guard"

run() { (cd "$PKG/src" && "$REPO_ROOT/env.sh" python -m arm_guard "$@"); }

echo "test_arm_guard.sh: [1/3] config + approval"
run check || { echo "test_arm_guard.sh: FAIL: config check" >&2; exit 1; }

echo "test_arm_guard.sh: [2/3] test suite"
(cd "$PKG" && "$REPO_ROOT/env.sh" python -m pytest -q -p no:cacheprovider tests) || {
    echo "test_arm_guard.sh: FAIL: arm_guard tests" >&2; exit 1; }

echo "test_arm_guard.sh: [3/3] hook installed in the motor envs"
run hook-status --python "$HOME/miniconda3/envs/rebot/bin/python" "$HOME/miniconda3/envs/lerobot/bin/python" || {
    echo "test_arm_guard.sh: FAIL: hook missing or inactive (python -m arm_guard install-hook)" >&2; exit 1; }
echo "test_arm_guard.sh: PASS"
