#!/usr/bin/env bash
# scripts/ros/test_execution.sh — MOT-05.5 headless mock end-to-end verification of
# execute_trajectory (ADR-016, docs/INTERFACES.md §11). No hardware, no real vendor driver,
# no robot motion outside a mock stack: moveit_mock (ros2_control mock hardware under
# mock_planning.launch.py) and rebot_motion's vendor-interface mock_driver.
#
# Design choice (per the card): G-GRAPH/G-START-STATE/G-COLLISION are all evaluated every run
# regardless of order (§11.7) -- an earlier gate's refusal never skips a later one. G-COLLISION
# always needs the MoveIt mock stack as its validity oracle (§11.11), for every driver profile,
# so "assert the refusal happens at G-GRAPH before any service wait" would still pay for ~90
# densified-waypoint validity calls against an absent oracle (each blocking for
# --service-timeout-s) before reporting. This script instead keeps a *second*, freshly started
# MoveIt mock stack instance running alongside mock_driver for the whole vendor-interface phase
# (phase 3): the first instance (phase 2, moveit_mock profile) is torn down first so
# mock_driver's own /rebotarm/joint_states is never started while the first instance's
# /joint_states could still be mistaken for it, then a fresh instance is brought back up
# purely as the §11.11 oracle -- its own mock arm never being addressed by any
# vendor-profile goal (G-GRAPH only ever resolves /rebotarm/follow_joint_trajectory for those
# profiles).
#
# Known limitation observed while writing this script, not worked around here: execute_trajectory
# subscribes to every `joint_states_topic` (`_wait_for_message` and the `execute()` monitoring
# loop's `js_sub`, crackvision_motion/execute_trajectory.py) with the default RELIABLE QoS
# (depth 10). Both rebot_motion's mock_driver and the real reBotArmController publish
# joint_states on `rclpy.qos.qos_profile_sensor_data` (BEST_EFFORT) -- a RELIABLE subscriber is
# never compatible with a BEST_EFFORT publisher, so G-START-STATE can never receive a message for
# the `vendor_mock`/`vendor` profiles ("no joint_states received within Xs"), and the execute()
# monitoring loop could never see feedback either. This blocks "mock mode with profile
# vendor_mock reaches the goal" below until execute_trajectory's joint_states subscriptions use a
# BEST_EFFORT-compatible QoS; it is not in this card's scope to fix (execute_trajectory.py is not
# in scope.write), so the corresponding check below is left asserting the required behaviour and
# will fail loudly against today's code rather than being softened to match the bug.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
FIXTURES="$REPO_ROOT/ros2_ws/src/crackvision_motion/test/fixtures"
GATE_FIXTURES="$FIXTURES/execution_gate"

# -- guard: never run anywhere near a real vendor driver process. `ps`/`pgrep -x` match against
# /proc/<pid>/comm, which the kernel truncates to 15 bytes -- "reBotArmController" (18 chars)
# truncates to exactly "reBotArmControl".
if pgrep -x reBotArmControl >/dev/null 2>&1; then
    echo "test_execution.sh: a real vendor driver process (reBotArmControl) is running -- refusing to run" >&2
    exit 3
fi

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1

OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_execution.sh: $OVERLAY_SETUP not found -- run scripts/ros/build_ws.sh first" >&2
    exit 3
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u

# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"

LOG_DIR="$REPO_ROOT/ros2_ws/log"
mkdir -p "$LOG_DIR"
RECORD_DIR="$REPO_ROOT/logs/test_execution_records"
rm -rf "$RECORD_DIR"
mkdir -p "$RECORD_DIR"

MOCKDRIVER_PID=""
MOCKDRIVER_PGID=""

stop_mock_driver() {
    if [ -n "$MOCKDRIVER_PGID" ]; then
        kill -INT -- "-$MOCKDRIVER_PGID" 2>/dev/null || true
        for _ in $(seq 1 10); do
            if ! kill -0 -- "-$MOCKDRIVER_PGID" 2>/dev/null; then
                MOCKDRIVER_PGID=""
                return 0
            fi
            sleep 1
        done
        if kill -0 -- "-$MOCKDRIVER_PGID" 2>/dev/null; then
            echo "test_execution.sh: mock_driver process group $MOCKDRIVER_PGID still alive after SIGINT, sending SIGKILL" >&2
            kill -KILL -- "-$MOCKDRIVER_PGID" 2>/dev/null || true
            sleep 1
        fi
        MOCKDRIVER_PGID=""
    fi
}

fail() {
    echo "test_execution.sh: FAIL: $*" >&2
    exit 1
}

# run_exec CMD... -- runs an execute_trajectory invocation detached from the controlling
# terminal with /dev/null stdin, so even an unexpected gate pass in real mode could never reach
# a /dev/tty confirmation.
run_exec() {
    setsid -w "$@" </dev/null
}

latest_record() {
    ls -t "$RECORD_DIR"/*.json 2>/dev/null | head -n1
}

# read_joint_state TOPIC TIMEOUT_S -- the current {"jointN": position, ...} on TOPIC, read fresh
# via a short-lived rclpy subscriber (never by parsing `ros2 topic echo` text). Subscribes with
# BEST_EFFORT QoS: compatible with both the RELIABLE ros2_control mock hardware (/joint_states)
# and rebot_motion's mock_driver / the real driver (/rebotarm/joint_states, BEST_EFFORT) -- see
# the header note above.
read_joint_state() {
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
node = Node("test_execution_js_reader")
box = {}
node.create_subscription(JointState, topic, lambda m: box.setdefault("m", m), qos_profile_sensor_data)
deadline = time.monotonic() + timeout_s
while "m" not in box and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if "m" not in box:
    print(f"read_joint_state: no message on {topic} within {timeout_s}s", file=sys.stderr)
    sys.exit(1)
m = box["m"]
print(json.dumps(dict(zip(m.name, m.position))))
PYEOF
}

# max_delta JSON1 JSON2 -- max |delta q| over joint1..joint6 between two {"jointN": pos} blobs.
max_delta() {
    python3 -c '
import json, sys
a, b = json.loads(sys.argv[1]), json.loads(sys.argv[2])
names = [f"joint{i}" for i in range(1, 7)]
print(max(abs(a[n] - b[n]) for n in names))
' "$1" "$2"
}

# max_delta_to_point JSON "j1,j2,j3,j4,j5,j6" -- max |delta q| between a {"jointN": pos} blob and
# an explicit joint1..joint6 point.
max_delta_to_point() {
    python3 -c '
import json, sys
a = json.loads(sys.argv[1])
point = [float(v) for v in sys.argv[2].split(",")]
names = [f"joint{i}" for i in range(1, 7)]
print(max(abs(a[n] - p) for n, p in zip(names, point)))
' "$1" "$2"
}

assert_lt() {
    python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) < float(sys.argv[2]) else 1)" "$1" "$2"
}

assert_gt() {
    python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) > float(sys.argv[2]) else 1)" "$1" "$2"
}

# assert_outcome RECORD_JSON OUTCOME
assert_outcome() {
    python3 -c '
import json, sys
record = json.load(open(sys.argv[1]))
got, expected = record["outcome"], sys.argv[2]
if got != expected:
    sys.exit("outcome=%r, expected %r" % (got, expected))
' "$1" "$2"
}

# assert_gate_outcome RECORD_JSON GATE_ID EXPECTED_OUTCOME
assert_gate_outcome() {
    python3 -c '
import json, sys
record = json.load(open(sys.argv[1]))
gate, expected = sys.argv[2], sys.argv[3]
hits = [c for c in record["gate_report"] if c["gate"] == gate]
if not hits:
    sys.exit("%s not found in gate_report" % gate)
got = hits[0]["outcome"]
if got != expected:
    sys.exit("%s outcome=%r, expected %r: %s" % (gate, got, expected, hits[0]["detail"]))
' "$1" "$2" "$3"
}

# assert_refusal_ids RECORD_JSON "ID1,ID2,..." -- exact set of fail/error gate ids.
assert_refusal_ids() {
    python3 -c '
import json, sys
record = json.load(open(sys.argv[1]))
expected = set(sys.argv[2].split(","))
got = {c["gate"] for c in record["gate_report"] if c["outcome"] in ("fail", "error")}
if got != expected:
    sys.exit("refusal ids %r != expected %r" % (sorted(got), sorted(expected)))
' "$1" "$2"
}

# --------------------------------------------------------------------------------------
# phase 1: fully offline refusals -- no ROS graph is up at all
# --------------------------------------------------------------------------------------

echo "== phase 1: offline refusals, no ROS graph =="

echo "-- real mode, repo defaults: must refuse offline before any rclpy.init --"
run_exec ros2 run crackvision_motion execute_trajectory \
    --mode real --trajectory "$FIXTURES/trajectory_smoke.json" --record-dir "$RECORD_DIR"
RC=$?
[ "$RC" -eq 3 ] || fail "real-mode-defaults: expected exit 3, got $RC"
REC="$(latest_record)"
[ -n "$REC" ] || fail "real-mode-defaults: no execution record written"
assert_outcome "$REC" refused || fail "real-mode-defaults: outcome assertion failed"
assert_refusal_ids "$REC" "G-ARM,G-TRAJ,G-LIMITS-HASH,G-SCENE,G-EE,G-COMMISSIONING,G-ESTOP,G-APPROVAL,G-STALE-CONFIG" \
    || fail "real-mode-defaults: refusal id set assertion failed"
echo "   OK: exit 3, refusal ids match: $(python3 -c 'import json,sys; print(sorted({c["gate"] for c in json.load(open(sys.argv[1]))["gate_report"] if c["outcome"] in ("fail","error")}))' "$REC")"

echo "-- mock mode, overspeed fixture: must refuse (G-LIMITS only) with no ROS contact --"
run_exec ros2 run crackvision_motion execute_trajectory \
    --mode mock --trajectory "$FIXTURES/trajectory_overspeed.json" --record-dir "$RECORD_DIR"
RC=$?
[ "$RC" -eq 3 ] || fail "overspeed: expected exit 3, got $RC"
REC="$(latest_record)"
assert_outcome "$REC" refused || fail "overspeed: outcome assertion failed"
assert_refusal_ids "$REC" "G-LIMITS" || fail "overspeed: refusal id set assertion failed"
echo "   OK: exit 3, G-LIMITS only"

# --------------------------------------------------------------------------------------
# phase 2: MoveIt mock stack (moveit_mock profile)
# --------------------------------------------------------------------------------------

echo "== phase 2: MoveIt mock stack (moveit_mock profile) =="
start_mock_stack "$LOG_DIR/test_execution_moveit_mock_launch.log" || exit 1
sleep 2

ros2 run crackvision_motion apply_scene --root "$REPO_ROOT" --config "$REPO_ROOT/config/scene/scene.yaml" >/dev/null \
    || fail "apply_scene (production scene) failed"

echo "-- dry mode: joint_states must be unchanged (max|dq| < 1e-4) --"
BEFORE="$(read_joint_state /joint_states 5)" || fail "dry: could not read joint_states before"
sleep 1
run_exec ros2 run crackvision_motion execute_trajectory \
    --mode dry --trajectory "$FIXTURES/trajectory_smoke.json" --record-dir "$RECORD_DIR"
RC=$?
[ "$RC" -eq 0 ] || fail "dry: expected exit 0, got $RC"
AFTER="$(read_joint_state /joint_states 5)" || fail "dry: could not read joint_states after"
DELTA="$(max_delta "$BEFORE" "$AFTER")"
assert_lt "$DELTA" "1e-4" || fail "dry: arm moved (max|dq|=$DELTA)"
echo "   OK: exit 0, max|dq|=$DELTA"

echo "-- mock mode: trajectory_smoke (home->P) must complete and arrive --"
sleep 1
run_exec ros2 run crackvision_motion execute_trajectory \
    --mode mock --trajectory "$FIXTURES/trajectory_smoke.json" --record-dir "$RECORD_DIR"
RC=$?
[ "$RC" -eq 0 ] || fail "mock smoke: expected exit 0, got $RC"
REC="$(latest_record)"
assert_outcome "$REC" completed || fail "mock smoke: outcome assertion failed"
LIVE="$(read_joint_state /joint_states 5)" || fail "mock smoke: could not read joint_states"
DELTA="$(max_delta_to_point "$LIVE" "0.1,-0.6,-0.9,0.1,0.1,0.1")"
assert_lt "$DELTA" "0.06" || fail "mock smoke: did not arrive at P (max|dq|=$DELTA)"
echo "   OK: exit 0, arrived at P (max|dq|=$DELTA)"

echo "-- mock mode: rerunning the same trajectory refuses on start-state mismatch, no motion --"
BEFORE="$LIVE"
sleep 1
run_exec ros2 run crackvision_motion execute_trajectory \
    --mode mock --trajectory "$FIXTURES/trajectory_smoke.json" --record-dir "$RECORD_DIR"
RC=$?
[ "$RC" -eq 3 ] || fail "start mismatch: expected exit 3, got $RC"
REC="$(latest_record)"
assert_gate_outcome "$REC" G-START-STATE fail || fail "start mismatch: G-START-STATE assertion failed"
AFTER="$(read_joint_state /joint_states 5)" || fail "start mismatch: could not read joint_states"
DELTA="$(max_delta "$BEFORE" "$AFTER")"
assert_lt "$DELTA" "1e-4" || fail "start mismatch: arm moved (max|dq|=$DELTA)"
echo "   OK: exit 3, G-START-STATE fail, no motion"

echo "-- mock mode: trajectory_estop (P->Q, slow) stops on an injected /crackvision/estop --"
# A brief settle delay: the previous invocation's node/subscriptions need a moment to fully
# release before a fresh node's 0.5s joint_states discovery window (timeouts.joint_state_s) is
# reliably long enough to see the existing publisher.
sleep 1
timeout 20 ros2 run crackvision_motion execute_trajectory \
    --mode mock --trajectory "$FIXTURES/trajectory_estop.json" --record-dir "$RECORD_DIR" </dev/null &
EXEC_PID=$!
sleep 2
# Bounded: `ros2 topic pub --once` waits for a matched subscriber before publishing and would
# hang forever if the executor already exited (e.g. an unexpected refusal) before this runs.
timeout 5 ros2 topic pub /crackvision/estop std_msgs/msg/Bool "{data: true}" --once >/dev/null \
    || echo "test_execution.sh: WARN: publishing /crackvision/estop did not complete within 5s (the executor may have already exited)" >&2
wait "$EXEC_PID"
RC=$?
[ "$RC" -eq 1 ] || fail "estop: expected exit 1, got $RC"
REC="$(latest_record)"
assert_outcome "$REC" estopped || fail "estop: outcome assertion failed"
LIVE="$(read_joint_state /joint_states 5)" || fail "estop: could not read joint_states"
DELTA="$(max_delta_to_point "$LIVE" "0.2,-0.5,-0.8,0.0,0.0,0.0")"
assert_gt "$DELTA" "0.06" || fail "estop: arrived at Q despite the e-stop (max|dq to Q|=$DELTA)"
echo "   OK: exit 1, stopped short of Q (max|dq to Q|=$DELTA)"

echo "-- mock mode: a short trajectory from the live e-stopped state is refused by G-COLLISION once an oversized collider is applied --"
COLLISION_TRAJ="$(mktemp -t mot055_collision_traj.XXXXXX.json)"
python3 -c '
import json, sys
live = json.loads(sys.argv[1])
names = [f"joint{i}" for i in range(1, 7)]
p0 = [live[n] for n in names]
p1 = [v + 0.02 for v in p0]
print(json.dumps({
    "schema": "crackvision.joint_trajectory/1",
    "created_utc": "2026-01-01T00:00:00Z",
    "producer": "test_execution.sh (MOT-05.5)",
    "purpose": "test",
    "planning_frame": "base_link",
    "joint_names": names,
    "notes": "generated at test-run time from the live /joint_states after the e-stop sub-case",
    "points": [
        {"t_s": 0.0, "positions": p0},
        {"t_s": 1.0, "positions": p1},
    ],
    "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": None},
    "end_effector_config_sha256": None,
    "scene_config_sha256": None,
    "source": {"paths3d": None},
}, indent=2))
' "$LIVE" > "$COLLISION_TRAJ"

ros2 run crackvision_motion apply_scene --root "$REPO_ROOT" \
    --config "$FIXTURES/scene_collision_smoke.yaml" >/dev/null \
    || fail "apply_scene (collision fixture) failed"

BEFORE="$LIVE"
sleep 1
run_exec ros2 run crackvision_motion execute_trajectory \
    --mode mock --trajectory "$COLLISION_TRAJ" --record-dir "$RECORD_DIR"
RC=$?
[ "$RC" -eq 3 ] || fail "collision: expected exit 3, got $RC"
REC="$(latest_record)"
assert_gate_outcome "$REC" G-COLLISION fail || fail "collision: G-COLLISION assertion failed"
AFTER="$(read_joint_state /joint_states 5)" || fail "collision: could not read joint_states"
DELTA="$(max_delta "$BEFORE" "$AFTER")"
assert_lt "$DELTA" "1e-4" || fail "collision: arm moved (max|dq|=$DELTA)"
rm -f "$COLLISION_TRAJ"
echo "   OK: exit 3, G-COLLISION fail, no motion"

echo "== tearing down the first MoveIt mock stack instance before starting mock_driver =="
stop_mock_stack

# --------------------------------------------------------------------------------------
# phase 3: vendor-interface mock (rebot_motion mock_driver) + a fresh MoveIt mock stack
# instance as the §11.11 collision/scene oracle (never addressed as a driver here)
# --------------------------------------------------------------------------------------

echo "== phase 3: vendor-interface mock (rebot_motion mock_driver) =="
start_mock_stack "$LOG_DIR/test_execution_oracle_launch.log" || exit 1
sleep 2
ros2 run crackvision_motion apply_scene --root "$REPO_ROOT" --config "$REPO_ROOT/config/scene/scene.yaml" >/dev/null \
    || fail "apply_scene (production scene, phase 3 oracle) failed"

ros2 run rebot_motion mock_driver --ros-args -p start_at_home:=false \
    > "$LOG_DIR/test_execution_mock_driver.log" 2>&1 &
MOCKDRIVER_PID=$!
sleep 2
MOCKDRIVER_PGID=$(ps -o pgid= -p "$MOCKDRIVER_PID" 2>/dev/null | tr -d ' ')
if [ -z "$MOCKDRIVER_PGID" ] || ! kill -0 "$MOCKDRIVER_PID" 2>/dev/null; then
    echo "test_execution.sh: mock_driver died on startup; log follows:" >&2
    cat "$LOG_DIR/test_execution_mock_driver.log" >&2
    exit 1
fi
# `trap` replaces, never stacks, a handler -- start_mock_stack's own internal
# `trap stop_mock_stack EXIT INT TERM` must not be the only one left registered, so this single
# combined handler calls both (each is a no-op once its own resource is already gone).
cleanup_phase3() {
    stop_mock_driver
    stop_mock_stack
}
trap cleanup_phase3 EXIT INT TERM
echo "test_execution.sh: mock_driver pid=$MOCKDRIVER_PID pgid=$MOCKDRIVER_PGID up"

# mock_driver executes a goal whether enabled or not (it only warns), but the real driver
# requires /enable first; call it here on the mock so the executor itself never has to.
ros2 service call /rebotarm/enable std_srvs/srv/Trigger "{}" >/dev/null \
    || fail "could not call /rebotarm/enable on mock_driver"

echo "-- mock mode (vendor_mock profile): trajectory_smoke (home->P) against mock_driver --"
sleep 1
run_exec ros2 run crackvision_motion execute_trajectory \
    --mode mock --profile vendor_mock --trajectory "$FIXTURES/trajectory_smoke.json" \
    --record-dir "$RECORD_DIR" --service-timeout-s 10
RC=$?
[ "$RC" -eq 0 ] || fail "vendor_mock smoke: expected exit 0, got $RC (see header note: execute_trajectory's joint_states subscriptions use RELIABLE QoS, incompatible with mock_driver's BEST_EFFORT joint_states -- a known bug outside this card's scope)"
REC="$(latest_record)"
assert_outcome "$REC" completed || fail "vendor_mock smoke: outcome assertion failed"
LIVE="$(read_joint_state /rebotarm/joint_states 5)" || fail "vendor_mock smoke: could not read /rebotarm/joint_states"
DELTA="$(max_delta_to_point "$LIVE" "0.1,-0.6,-0.9,0.1,0.1,0.1")"
assert_lt "$DELTA" "0.06" || fail "vendor_mock smoke: did not arrive at P (max|dq|=$DELTA)"
echo "   OK: exit 0, arrived at P (max|dq|=$DELTA)"

echo "-- real mode (vendor profile), armed, measured fixture set: refused by G-GRAPH (mock driver visible), no motion --"
VENDOR_REAL_DIR="$(mktemp -d -t mot055_vendor_real.XXXXXX)"
cp "$GATE_FIXTURES/paths3d_eligible.json" "$VENDOR_REAL_DIR/paths3d_eligible.json"
TRAJ_SHA="$(python3 -c '
import json
from crackvision_motion.joint_trajectory import file_sha256

root = "'"$REPO_ROOT"'"
gate_fixtures = "'"$GATE_FIXTURES"'"
out_dir = "'"$VENDOR_REAL_DIR"'"

traj = {
    "schema": "crackvision.joint_trajectory/1",
    "created_utc": "2026-01-01T00:00:00Z",
    "producer": "test_execution.sh (MOT-05.5)",
    "purpose": "crack_task",
    "planning_frame": "base_link",
    "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "points": [
        {"t_s": 0.0, "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        {"t_s": 1.5, "positions": [0.05, -0.3, -0.45, 0.05, 0.05, 0.05]},
        {"t_s": 3.0, "positions": [0.1, -0.6, -0.9, 0.1, 0.1, 0.1]},
    ],
    "limits_file": {
        "path": "config/robot/b601_dm_limits.yaml",
        "sha256": file_sha256(f"{root}/config/robot/b601_dm_limits.yaml"),
    },
    "end_effector_config_sha256": file_sha256(f"{gate_fixtures}/end_effector_measured.yaml"),
    "scene_config_sha256": file_sha256(f"{gate_fixtures}/scene_measured.yaml"),
    "source": {
        "paths3d": {
            "path": "paths3d_eligible.json",
            "sha256": file_sha256(f"{gate_fixtures}/paths3d_eligible.json"),
            "execution_eligible": True,
        }
    },
}
traj_path = f"{out_dir}/trajectory.json"
with open(traj_path, "w") as fh:
    json.dump(traj, fh, indent=2)
print(file_sha256(traj_path))
')"

python3 -c '
import json
import sys
from datetime import datetime, timezone

out_dir, traj_sha = sys.argv[1], sys.argv[2]
approval = {
    "schema": "crackvision.execution_approval/1",
    "trajectory_sha256": traj_sha,
    "approved_by": "test_execution.sh",
    "approved_utc": datetime.now(timezone.utc).isoformat(),
    "preview_artifacts": [],
}
with open(f"{out_dir}/approval.json", "w") as fh:
    json.dump(approval, fh, indent=2)
' "$VENDOR_REAL_DIR" "$TRAJ_SHA"

BEFORE="$(read_joint_state /rebotarm/joint_states 5)" || fail "vendor real-mode: could not read /rebotarm/joint_states before"
sleep 1
CRACKVISION_ARM_REAL=1 run_exec ros2 run crackvision_motion execute_trajectory \
    --mode real --profile vendor \
    --root "$VENDOR_REAL_DIR" \
    --trajectory "$VENDOR_REAL_DIR/trajectory.json" \
    --execution-config "$REPO_ROOT/config/motion/execution.yaml" \
    --commissioning "$GATE_FIXTURES/commissioning_ready.yaml" \
    --limits "$REPO_ROOT/config/robot/b601_dm_limits.yaml" \
    --scene-config "$GATE_FIXTURES/scene_measured.yaml" \
    --end-effector-config "$GATE_FIXTURES/end_effector_measured.yaml" \
    --approval "$VENDOR_REAL_DIR/approval.json" \
    --record-dir "$RECORD_DIR" --service-timeout-s 10
RC=$?
[ "$RC" -eq 3 ] || fail "vendor real-mode: expected exit 3, got $RC"
REC="$(latest_record)"
assert_outcome "$REC" refused || fail "vendor real-mode: outcome assertion failed"
assert_gate_outcome "$REC" G-GRAPH fail || fail "vendor real-mode: G-GRAPH assertion failed"
AFTER="$(read_joint_state /rebotarm/joint_states 5)" || fail "vendor real-mode: could not read /rebotarm/joint_states after"
DELTA="$(max_delta "$BEFORE" "$AFTER")"
assert_lt "$DELTA" "1e-4" || fail "vendor real-mode: arm moved (max|dq|=$DELTA)"
rm -rf "$VENDOR_REAL_DIR"
echo "   OK: exit 3, G-GRAPH fail, no motion"

echo "== tearing down phase 3 (mock_driver + second MoveIt mock stack instance) =="
stop_mock_driver
stop_mock_stack

# --------------------------------------------------------------------------------------
# final check: no process from any launched process group survived teardown
# --------------------------------------------------------------------------------------

sleep 1
STRAY=$(ps -eo pid=,comm= | grep -E "move_group|ros2_control_node|mock_driver|robot_state_publisher" || true)
if [ -n "$STRAY" ]; then
    echo "test_execution.sh: stray process(es) remain after teardown:" >&2
    echo "$STRAY" >&2
    exit 1
fi
echo "test_execution.sh: no stray processes after teardown"

echo "test_execution.sh: OK"
exit 0
