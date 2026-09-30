#!/usr/bin/env bash
# scripts/ros/test_reachability_task_frames.sh — ADR-014 reachability smoke test.
#
# Two real sweeps against the mock stack (via run_reachability.sh, which tears the stack down itself):
#   1. tool_tip at 0.01/0.04 m clearance with the specimen_block proxy + table environment: the near
#      targets must be reachable at BOTH clearances (the MOT-04.5 frame error made this impossible), the
#      far ones must not be, and the map must record the task frame, collision model and environment;
#   2. camera_link 0.25 m above a surface point (view phase): must be reachable.
# Both maps are validated with the system-python3 map loader (joint limits, schema). Exit 0 only if all hold.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1

FIX="$REPO_ROOT/ros2_ws/src/crackvision_motion/test/fixtures"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$LOG_DIR"
TOOL_MAP="$LOG_DIR/test_task_frames_tool_map.json"
VIEW_MAP="$LOG_DIR/test_task_frames_view_map.json"
rm -f "$TOOL_MAP" "$VIEW_MAP"

"$SCRIPT_DIR/run_reachability.sh" --reachability-config "$FIX/reachability_task_frames_smoke.yaml" --out "$TOOL_MAP" \
    || { echo "test_reachability_task_frames.sh: tool sweep failed" >&2; exit 1; }
"$SCRIPT_DIR/run_reachability.sh" --reachability-config "$FIX/reachability_view_smoke.yaml" --out "$VIEW_MAP" \
    --require-all-reachable || { echo "test_reachability_task_frames.sh: camera view sweep failed" >&2; exit 1; }

python3 - "$REPO_ROOT" "$TOOL_MAP" "$VIEW_MAP" <<'PYEOF'
import sys

root, tool_path, view_path = sys.argv[1:4]
sys.path.insert(0, root + "/ros2_ws/src/crackvision_motion")
from crackvision_motion.reachability_map import load_map

limits = root + "/config/robot/b601_dm_limits.yaml"
tool = load_map(tool_path, limits_path=limits)
assert tool["complete"] is True
assert tool["robot"]["ik_link"] == "tool_tip", tool["robot"]
assert tool["grid"]["surface_collision"]["model"] == "specimen_block"
assert tool["grid"]["environment"]["objects"] == ["table"], tool["grid"].get("environment")
counts = tool["summary"]["counts_by_status"]
assert counts.get("fk_mismatch", 0) == 0 and counts.get("error", 0) == 0, counts
near = [t for t in tool["targets"] if t["x"] < 0.5]
far = [t for t in tool["targets"] if t["x"] > 0.9]
assert len(near) == 6 and all(t["status"] == "reachable" for t in near), [(t["target_id"], t["status"]) for t in near]
assert far and all(t["status"] in ("unreachable", "prefiltered") for t in far), [(t["target_id"], t["status"]) for t in far]

view = load_map(view_path, limits_path=limits)
assert view["complete"] is True and view["robot"]["ik_link"] == "camera_link"
assert [t["status"] for t in view["targets"]] == ["reachable"], view["targets"]
print(f"test_reachability_task_frames.sh: tool counts={counts}; camera view reachable "
      f"(tilt {view['targets'][0]['tilt_deg']} deg)")
PYEOF
[ $? -eq 0 ] || { echo "test_reachability_task_frames.sh: map assertions failed" >&2; exit 1; }
echo "test_reachability_task_frames.sh: OK"
exit 0
