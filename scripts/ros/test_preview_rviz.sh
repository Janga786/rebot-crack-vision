#!/usr/bin/env bash
# scripts/ros/test_preview_rviz.sh — MOT-08.7 headless proof that preview_rviz.launch.py's own
# node graph (mock_planning.launch.py + preview_trajectory, §18.4) actually publishes a preview
# without any motion, on a host with no DISPLAY and no Xvfb (rviz2 itself cannot be exercised
# here, so this script never starts it).
#
# Manual step for an operator with a display (not run by this script):
#   source scripts/ros/env_ros.sh; set +u; source ros2_ws/install/setup.bash
#   ros2 launch crackvision_motion preview_rviz.launch.py trajectory:=PATH
#
# This script only ever starts the headless ros2_control mock hardware stack
# (scripts/ros/_mock_stack.sh), never CRACKVISION_ARM_REAL, and refuses outright if a real
# vendor driver process is already running (§0's truncated-comm pattern, mirroring
# test_view_retime_execute.sh). Everything it writes lives under logs/preview_rviz/<stamp>/.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
WORK="$REPO_ROOT/logs/preview_rviz/$STAMP"
mkdir -p "$WORK"

fail() {
    echo "test_preview_rviz.sh: FAIL: $*" >&2
    exit 1
}

# -- guard: never run anywhere near a real vendor driver process.
if pgrep -x reBotArmControl >/dev/null 2>&1; then
    echo "test_preview_rviz.sh: a real vendor driver process (reBotArmControl) is running -- refusing to run" >&2
    exit 3
fi

TRAJECTORY="$REPO_ROOT/ros2_ws/src/crackvision_motion/test/fixtures/trajectory_smoke.json"
[ -f "$TRAJECTORY" ] || fail "fixture trajectory not found: $TRAJECTORY"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_preview_rviz.sh: $OVERLAY_SETUP not found -- run scripts/ros/build_ws.sh first" >&2
    exit 3
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

# --------------------------------------------------------------------------------------
# 1. ros2 launch ... --show-args (headless, no process started).
# --------------------------------------------------------------------------------------

echo "test_preview_rviz.sh: == step 1: ros2 launch --show-args =="
SHOW_ARGS_LOG="$WORK/show_args.log"
ros2 launch crackvision_motion preview_rviz.launch.py --show-args >"$SHOW_ARGS_LOG" 2>&1
rc=$?
cat "$SHOW_ARGS_LOG"
[ "$rc" -eq 0 ] || fail "ros2 launch --show-args exit=$rc (expected 0); see $SHOW_ARGS_LOG"
grep -q "'trajectory'" "$SHOW_ARGS_LOG" || fail "--show-args did not list the 'trajectory' argument"

# --------------------------------------------------------------------------------------
# 2. headless mock stack + preview_trajectory --once (no rviz2 -- this host has no display).
# --------------------------------------------------------------------------------------

echo "test_preview_rviz.sh: == step 2: mock stack + preview_trajectory --once =="
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"
LAUNCH_LOG="$WORK/mock_planning_launch.log"
start_mock_stack "$LAUNCH_LOG" || exit 1
sleep 2

read_joint_state_positions() {
    python3 - "$1" "$2" <<'PYEOF'
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

topic, timeout_s = sys.argv[1], float(sys.argv[2])
rclpy.init()
node = Node("test_preview_rviz_js_reader")
box = {}
node.create_subscription(JointState, topic, lambda m: box.setdefault("m", m), qos_profile_sensor_data)
deadline = time.monotonic() + timeout_s
while "m" not in box and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if "m" not in box:
    print(f"read_joint_state_positions: no message on {topic} within {timeout_s}s", file=sys.stderr)
    sys.exit(1)
m = box["m"]
print(json.dumps(dict(zip(m.name, m.position))))
PYEOF
}

BEFORE_JS="$(read_joint_state_positions /joint_states 5)" || fail "could not read /joint_states before preview_trajectory"

# The verifier must already be discovered (TRANSIENT_LOCAL durability is cached at the writer,
# not externally) when preview_trajectory publishes and then exits on --once, so it is started
# in the background first and preview_trajectory --once is run while it is still subscribing.
echo "test_preview_rviz.sh: == step 3: verify latched /display_planned_path + /crackvision/preview_markers =="
VERIFY_LOG="$WORK/verify.log"
python3 - "$TRAJECTORY" >"$VERIFY_LOG" 2>&1 <<'PYEOF' &
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from moveit_msgs.msg import DisplayTrajectory
from visualization_msgs.msg import MarkerArray

trajectory_path = sys.argv[1]
traj = json.load(open(trajectory_path, encoding="utf-8"))
expected_n_points = len(traj["points"])
expected_joint_names = list(traj["joint_names"])

qos = QoSProfile(depth=1)
qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
qos.reliability = QoSReliabilityPolicy.RELIABLE

rclpy.init()
node = Node("test_preview_rviz_verifier")
box = {}
node.create_subscription(
    DisplayTrajectory, "/display_planned_path", lambda m: box.setdefault("display", m), qos
)
node.create_subscription(
    MarkerArray, "/crackvision/preview_markers", lambda m: box.setdefault("markers", m), qos
)

deadline = time.monotonic() + 10.0
while ("display" not in box or "markers" not in box) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()

if "display" not in box:
    sys.exit("no message received on /display_planned_path")
if "markers" not in box:
    sys.exit("no message received on /crackvision/preview_markers")

display = box["display"]
if len(display.trajectory) != 1:
    sys.exit(f"expected exactly 1 RobotTrajectory, got {len(display.trajectory)}")
points = display.trajectory[0].joint_trajectory.points
if len(points) != expected_n_points:
    sys.exit(f"DisplayTrajectory point count {len(points)} != fixture {expected_n_points}")
joint_names = list(display.trajectory[0].joint_trajectory.joint_names)
if joint_names != expected_joint_names:
    sys.exit(f"joint_names {joint_names} != fixture {expected_joint_names}")

markers = box["markers"]
if len(markers.markers) == 0:
    sys.exit("MarkerArray is empty")

print(f"DisplayTrajectory: n_points={len(points)} joint_names={joint_names}")
print(f"MarkerArray: n_markers={len(markers.markers)}")
PYEOF
VERIFY_PID=$!
sleep 2  # let the verifier's subscriptions be discovered before preview_trajectory publishes and exits

PREVIEW_LOG="$WORK/preview_trajectory.log"
ros2 run crackvision_motion preview_trajectory --trajectory "$TRAJECTORY" --once >"$PREVIEW_LOG" 2>&1
rc=$?
cat "$PREVIEW_LOG"
[ "$rc" -eq 0 ] || fail "preview_trajectory --once exit=$rc (expected 0); see $PREVIEW_LOG"

AFTER_JS="$(read_joint_state_positions /joint_states 5)" || fail "could not read /joint_states after preview_trajectory"

wait "$VERIFY_PID"
rc=$?
cat "$VERIFY_LOG"
[ "$rc" -eq 0 ] || fail "late-joining subscriber verification failed (exit=$rc); see $VERIFY_LOG"

# --------------------------------------------------------------------------------------
# 4. /joint_states unchanged across the run (no motion).
# --------------------------------------------------------------------------------------

echo "test_preview_rviz.sh: == step 4: /joint_states unchanged (no motion) =="
DELTA="$(python3 -c '
import json, sys
a = json.loads(sys.argv[1])
b = json.loads(sys.argv[2])
if set(a) != set(b):
    sys.exit(f"joint name sets differ: {sorted(a)} vs {sorted(b)}")
print(max(abs(a[n] - b[n]) for n in a))
' "$BEFORE_JS" "$AFTER_JS")" || fail "could not compare before/after joint states"
python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) < 1e-6 else 1)" "$DELTA" \
    || fail "/joint_states changed across the run (max|dq|=$DELTA) -- preview_trajectory moved the arm"
echo "test_preview_rviz.sh: /joint_states unchanged (max|dq|=$DELTA)"

stop_mock_stack

sleep 1
STRAY=$(ps -eo pid=,comm= | grep -E "move_group|ros2_control_node|robot_state_publisher" || true)
if [ -n "$STRAY" ]; then
    echo "test_preview_rviz.sh: stray process(es) remain after teardown:" >&2
    echo "$STRAY" >&2
    exit 1
fi

echo "test_preview_rviz.sh: PASS ($WORK)"
exit 0
