#!/usr/bin/env bash
# scripts/ros/test_joint_states_qos.sh — QoS-live smoke test for MOT-05.7.
#
# Proves execute_trajectory's non-latched joint_states subscription (used by both
# `_wait_for_message`/G-START-STATE and `execute()`'s `js_sub`) actually receives
# /rebotarm/joint_states from a BEST_EFFORT publisher, and that an explicit RELIABLE
# subscription to the same topic -- the bug this card fixes -- receives nothing in the same
# window. Starts only `ros2 run rebot_motion mock_driver` from the read-only ~/rebot_ws
# underlay; never touches the real driver. Modelled on test_reachability.sh: own process
# group (`set -m`), trap teardown, no stray processes after exit.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

if pgrep -x reBotArmControl >/dev/null 2>&1; then
    echo "test_joint_states_qos.sh: refusing -- a real reBotArmController process is running" >&2
    exit 3
fi

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1

OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_joint_states_qos.sh: $OVERLAY_SETUP not found -- run scripts/ros/build_ws.sh first" >&2
    exit 1
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

LOG_DIR="$REPO_ROOT/ros2_ws/log"
mkdir -p "$LOG_DIR"
DRIVER_LOG="$LOG_DIR/test_joint_states_qos_driver.log"

DRIVER_PID=""
DRIVER_PGID=""

cleanup() {
    if [ -n "$DRIVER_PGID" ]; then
        kill -INT -- "-$DRIVER_PGID" 2>/dev/null || true
        for _ in $(seq 1 10); do
            if ! kill -0 -- "-$DRIVER_PGID" 2>/dev/null; then
                DRIVER_PGID=""
                return 0
            fi
            sleep 1
        done
        if kill -0 -- "-$DRIVER_PGID" 2>/dev/null; then
            echo "test_joint_states_qos.sh: process group $DRIVER_PGID still alive after SIGINT, sending SIGKILL" >&2
            kill -KILL -- "-$DRIVER_PGID" 2>/dev/null || true
            sleep 1
        fi
        DRIVER_PGID=""
    fi
}
trap cleanup EXIT INT TERM

ros2 run rebot_motion mock_driver > "$DRIVER_LOG" 2>&1 &
DRIVER_PID=$!
sleep 1
DRIVER_PGID=$(ps -o pgid= -p "$DRIVER_PID" 2>/dev/null | tr -d ' ')
if [ -z "$DRIVER_PGID" ]; then
    echo "test_joint_states_qos.sh: mock_driver process $DRIVER_PID exited immediately; log follows:" >&2
    cat "$DRIVER_LOG" >&2
    exit 1
fi

sleep 2
if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
    echo "test_joint_states_qos.sh: mock_driver died on startup; log follows:" >&2
    cat "$DRIVER_LOG" >&2
    exit 1
fi
echo "test_joint_states_qos.sh: mock_driver pid=$DRIVER_PID pgid=$DRIVER_PGID up"

python3 - "$REPO_ROOT" <<'PYEOF'
import sys
import time

root = sys.argv[1]
sys.path.insert(0, root + "/ros2_ws/src/crackvision_motion")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import crackvision_motion.execute_trajectory as et

DISCOVERY_BOUND_S = 5.0
TOPIC = "/rebotarm/joint_states"

rclpy.init(args=None)
node = Node("test_joint_states_qos_probe")
try:
    msg = et._wait_for_message(node, TOPIC, JointState, DISCOVERY_BOUND_S)
    if msg is None:
        print(f"test_joint_states_qos.sh: FAIL -- the executor's own (BEST_EFFORT) joint_states "
              f"subscription received nothing on {TOPIC} within {DISCOVERY_BOUND_S}s", file=sys.stderr)
        sys.exit(1)
    print(f"test_joint_states_qos.sh: BEST_EFFORT subscription received joint_states on {TOPIC}")

    box = {}
    reliable_sub = node.create_subscription(JointState, TOPIC, lambda m: box.setdefault("msg", m), 10)
    deadline = time.monotonic() + DISCOVERY_BOUND_S
    while "msg" not in box and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(reliable_sub)
    if "msg" in box:
        print("test_joint_states_qos.sh: FAIL -- an explicitly RELIABLE subscription unexpectedly "
              "received a message from the BEST_EFFORT mock_driver publisher", file=sys.stderr)
        sys.exit(1)
    print(f"test_joint_states_qos.sh: RELIABLE subscription received nothing on {TOPIC} within "
          f"{DISCOVERY_BOUND_S}s (documents the QoS-incompatibility bug MOT-05.7 fixes)")
finally:
    node.destroy_node()
    rclpy.shutdown()

print("test_joint_states_qos.sh: OK")
PYEOF
PROBE_EXIT=$?

if [ "$PROBE_EXIT" -ne 0 ]; then
    echo "test_joint_states_qos.sh: probe failed; mock_driver log follows:" >&2
    cat "$DRIVER_LOG" >&2
fi

exit "$PROBE_EXIT"
