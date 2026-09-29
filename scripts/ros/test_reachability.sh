#!/usr/bin/env bash
# scripts/ros/test_reachability.sh — reachability sweep smoke test (MOT-04.4).
#
# Runs run_reachability.sh against the tiny reachability_smoke.yaml fixture (2 x values x 3 y
# values x 1 surface_z x 1 standoff = 6 targets; the near x=0.20 targets were probed live against
# move_group and found reachable, the far x=0.95 targets are inside reach_bound_from_urdf's
# conservative sphere so they reach a real IK call and come back genuinely unreachable rather
# than prefiltered — see docs/motion/ROS_WORKSPACE.md), then validates the resulting map with the
# system-python3 map loader (which itself enforces every reachable joint vector is within
# config/robot/b601_dm_limits.yaml — docs/INTERFACES.md §7.2), and finally asserts that no
# process from the launch's process group survived teardown.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1

FIXTURE="$REPO_ROOT/ros2_ws/src/crackvision_motion/test/fixtures/reachability_smoke.yaml"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$LOG_DIR"
OUT="$LOG_DIR/test_reachability_map.json"
RUN_LOG="$LOG_DIR/test_reachability_run.log"
rm -f "$OUT"

"$SCRIPT_DIR/run_reachability.sh" --reachability-config "$FIXTURE" --out "$OUT" 2>&1 | tee "$RUN_LOG"
SWEEP_EXIT=${PIPESTATUS[0]}

LAUNCH_PGID=$(grep -oE 'pgid=[0-9]+' "$RUN_LOG" | head -n1 | cut -d= -f2 || true)

if [ "$SWEEP_EXIT" -ne 0 ]; then
    echo "test_reachability.sh: run_reachability.sh exited $SWEEP_EXIT" >&2
    exit 1
fi

if [ ! -f "$OUT" ]; then
    echo "test_reachability.sh: expected map $OUT was not written" >&2
    exit 1
fi

# validate_map (called by load_map) already refuses any reachable target whose joints fall
# outside config/robot/b601_dm_limits.yaml, so a clean load_map here already proves that check.
python3 - "$REPO_ROOT" "$OUT" <<'PYEOF'
import sys

root, out_path = sys.argv[1], sys.argv[2]
sys.path.insert(0, root + "/ros2_ws/src/crackvision_motion")
from crackvision_motion.reachability_map import load_map

map_dict = load_map(out_path, limits_path=root + "/config/robot/b601_dm_limits.yaml")

assert map_dict["complete"] is True, "map is not complete"
counts = map_dict["summary"]["counts_by_status"]
assert counts.get("reachable", 0) >= 1, f"expected >=1 reachable target, got {counts}"
assert counts.get("fk_mismatch", 0) == 0, f"expected 0 fk_mismatch targets, got {counts}"
assert counts.get("error", 0) == 0, f"expected 0 error targets, got {counts}"

for t in map_dict["targets"]:
    if t["x"] >= 0.9:
        assert t["status"] in ("prefiltered", "unreachable"), (
            f"target {t['target_id']} at x={t['x']} is status {t['status']!r}, "
            "expected prefiltered or unreachable"
        )

print(f"test_reachability.sh: map assertions OK -- counts={counts}")
PYEOF
CHECK_EXIT=$?

if [ "$CHECK_EXIT" -ne 0 ]; then
    echo "test_reachability.sh: map assertions failed" >&2
    exit 1
fi

if [ -n "${LAUNCH_PGID:-}" ]; then
    sleep 1
    SURVIVORS=$(ps -eo pid=,pgid=,comm= | awk -v pg="$LAUNCH_PGID" '$2 == pg')
    if [ -n "$SURVIVORS" ]; then
        echo "test_reachability.sh: stray process(es) remain in launch pgid $LAUNCH_PGID:" >&2
        echo "$SURVIVORS" >&2
        exit 1
    fi
    echo "test_reachability.sh: no stray processes in launch pgid $LAUNCH_PGID"
else
    echo "test_reachability.sh: warning: could not determine launch pgid from $RUN_LOG" >&2
fi

echo "test_reachability.sh: OK"
exit 0
