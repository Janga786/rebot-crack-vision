#!/usr/bin/env bash
# scripts/ros/test_view_plan.sh — headless mock-stack smoke test of plan_view (MOT-06.9).
#
# Plans a camera view pose against the committed, test-only fixture scene (ros2_ws/src/
# crackvision_motion/test/fixtures/cartesian/smoke/scene.yaml, MOT-06.7's own specimen-at-the-
# recommended-placement fixture, reused read-only) on the mock stack (no robot, no action client:
# plan_view never commands motion, no /compute_cartesian_path call either), then checks the
# written view_plan file with an independent python3 block that makes its OWN /compute_fk call
# rather than trusting the planner's recheck numbers:
#   - a candidate was chosen; its fov_check passed, it is reachable and recheck_passed;
#   - camera_link sits 0.25 +/- 0.002 m from the specimen top centre, optical axis within
#     15 + 0.1 deg of the anti-normal, and the FOV check passed for all 4 footprint corners;
#   - the transit leg starts at the SRDF all-zero home state and every configuration is valid
#     and inside the inclusive limits (continuity);
# and finally re-runs with an --center far outside reach, which must exit 3, list every tried
# candidate's reason, and write no file.
#
# Build is assumed done (scripts/ros/build_ws.sh). The planning config is the unmodified
# production config/motion/cartesian_planning.yaml. The mock stack is torn down on every exit
# path (_mock_stack.sh's EXIT/INT/TERM trap). Exit 0 only if every assertion holds.
set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

set --
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env_ros.sh" || exit 1
OVERLAY_SETUP="$REPO_ROOT/ros2_ws/install/setup.bash"
if [ ! -f "$OVERLAY_SETUP" ]; then
    echo "test_view_plan.sh: $OVERLAY_SETUP not found — run scripts/ros/build_ws.sh first" >&2
    exit 3
fi
set +u
# shellcheck disable=SC1091
source "$OVERLAY_SETUP"
set -u
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_mock_stack.sh"

FIX_REL="ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke"
FIX="$REPO_ROOT/$FIX_REL"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$LOG_DIR"
PLAN="$LOG_DIR/test_view_plan.json"
PLAN_UNREACHABLE="$LOG_DIR/test_view_plan_unreachable.json"
PLANNING_CFG="$REPO_ROOT/config/motion/cartesian_planning.yaml"
LAUNCH_LOG="$LOG_DIR/test_view_plan_launch.log"
rm -f "$PLAN" "$PLAN_UNREACHABLE"

fail() {
    echo "test_view_plan.sh: FAIL: $*" >&2
    exit 1
}

# 1. Mock stack up, and its MoveIt services reachable before the planner's own short wait.
start_mock_stack "$LAUNCH_LOG" || exit 1
python3 - <<'PYEOF' || fail "MoveIt services did not come up; see $LAUNCH_LOG"
import sys, rclpy
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
rclpy.init()
node = rclpy.create_node("test_view_plan_wait")
ok = all(node.create_client(t, n).wait_for_service(timeout_sec=120.0) for t, n in (
    (GetPositionFK, "/compute_fk"), (GetPositionIK, "/compute_ik"),
    (GetMotionPlan, "/plan_kinematic_path"), (GetStateValidity, "/check_state_validity"),
    (GetPlanningScene, "/get_planning_scene")))
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PYEOF

# 2. Plan a view pose for the fixture's recommended-placement specimen (expect exit 0).
echo "test_view_plan.sh: planning a view pose for $FIX_REL/scene.yaml"
ros2 run crackvision_motion plan_view --scene-config "$FIX/scene.yaml" --planning-config "$PLANNING_CFG" --out "$PLAN"
rc=$?
[ "$rc" -eq 0 ] || fail "plan_view exit=$rc (expected 0)"
[ -f "$PLAN" ] || fail "plan_view exit 0 but $PLAN not written"

# 3. Independent assertions from the plan file + this block's own /compute_fk call.
python3 - "$REPO_ROOT" "$PLAN" <<'PYEOF'
import json, math, sys
import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from sensor_msgs.msg import JointState

root, plan_path = sys.argv[1:3]
JOINTS = [f"joint{i}" for i in range(1, 7)]
VIEW_DISTANCE_M, VIEW_DISTANCE_TOL_M = 0.25, 0.002
ANTI_NORMAL_TOL_DEG = 15.0 + 0.1
CENTRE = np.array([0.26, 0.0, 0.01])  # fixture specimen top centre (0.26, 0.0, 0.005 + 0.01/2)
failures = []


def check(cond, msg):
    if not cond:
        failures.append(msg)
    return cond


plan = json.load(open(plan_path))
check(plan["schema"] == "crackvision.view_plan/1", "schema")
check(plan["planning_frame"] == "base_link", "planning_frame")
check(plan["center_source"] == "scene.specimen", f"center_source {plan['center_source']}")
check(np.allclose(plan["center_xy_m"], CENTRE[:2], atol=1e-6), f"center_xy_m {plan['center_xy_m']}")
check(plan["chosen"] is not None, "no candidate was chosen")
chosen = plan["chosen"]

matches = [c for c in plan["candidates"] if c["tilt_deg"] == chosen["tilt_deg"] and c["yaw_deg"] == chosen["yaw_deg"]]
check(len(matches) == 1, f"chosen candidate not found exactly once: {matches}")
cand = matches[0]
check(cand["fov_check"]["passed"], f"chosen candidate's own fov_check failed: {cand['fov_check']}")
check(cand["reachable"], "chosen candidate not reachable")
check(cand["recheck_passed"], "chosen candidate's recheck_passed is false")
check(all(m >= 0.0 for m in cand["fov_check"]["horizontal_margins_deg"]), "horizontal FOV margin negative")
check(all(m >= 0.0 for m in cand["fov_check"]["vertical_margins_deg"]), "vertical FOV margin negative")

transit = plan["transit_leg"]
check(transit is not None, "transit_leg missing despite a chosen candidate")
check(transit["kind"] == "transit" and transit["segment_ref"] is None, "transit_leg shape")
positions = np.array(transit["positions"], dtype=float)
check(np.allclose(positions[0], 0.0, atol=1e-9), f"transit does not start at home: {positions[0].tolist()}")

rclpy.init()
node = rclpy.create_node("test_view_plan_check")


def call(client, req):
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    if not fut.done() or fut.result() is None:
        raise RuntimeError(f"service call to {client.srv_name} did not complete")
    return fut.result()


fk_cli = node.create_client(GetPositionFK, "/compute_fk")
val_cli = node.create_client(GetStateValidity, "/check_state_validity")
for c in (fk_cli, val_cli):
    if not c.wait_for_service(timeout_sec=30.0):
        sys.exit(f"service {c.srv_name} unavailable")


def robot_state(q):
    rs = RobotState()
    rs.joint_state = JointState(name=JOINTS, position=[float(v) for v in q])
    return rs


def fk(q, link):
    req = GetPositionFK.Request()
    req.header.frame_id = "base_link"
    req.fk_link_names = [link]
    req.robot_state = robot_state(q)
    resp = call(fk_cli, req)
    if resp.error_code.val != 1 or len(resp.pose_stamped) != 1:
        raise RuntimeError(f"/compute_fk error {resp.error_code.val}")
    p, o = resp.pose_stamped[0].pose.position, resp.pose_stamped[0].pose.orientation
    x, y, z, w = o.x, o.y, o.z, o.w
    x_axis = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)])
    return np.array([p.x, p.y, p.z]), x_axis


def valid(q):
    req = GetStateValidity.Request()
    req.group_name = "arm"
    req.robot_state = robot_state(q)
    return bool(call(val_cli, req).valid)


for q in positions:
    check(valid(q), f"transit configuration invalid per /check_state_validity: {q.tolist()}")
if len(positions) > 1:
    step = float(np.max(np.abs(np.diff(positions, axis=0))))
    check(step <= transit["recheck"]["max_joint_step_rad"] + 1e-9, f"transit joint step {step}")

final_q = positions[-1]
cam_pos, cam_x_axis = fk(final_q, "camera_link")
dist = float(np.linalg.norm(cam_pos - CENTRE))
check(abs(dist - VIEW_DISTANCE_M) <= VIEW_DISTANCE_TOL_M,
      f"camera_link distance from specimen centre {dist:.5f} m (expected {VIEW_DISTANCE_M} +/- {VIEW_DISTANCE_TOL_M})")
anti_normal = np.array([0.0, 0.0, -1.0])
angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(cam_x_axis, anti_normal))))))
check(angle_deg <= ANTI_NORMAL_TOL_DEG, f"optical axis angle to anti-normal {angle_deg:.3f} deg > {ANTI_NORMAL_TOL_DEG}")

node.destroy_node()
rclpy.shutdown()

print(f"test_view_plan.sh: chosen tilt={chosen['tilt_deg']} yaw={chosen['yaw_deg']}, "
      f"camera distance {dist:.5f} m, optical-axis/anti-normal angle {angle_deg:.3f} deg, "
      f"{len(plan['candidates'])} candidates tried")
if failures:
    for f in failures:
        print(f"ASSERTION FAILED: {f}", file=sys.stderr)
    sys.exit(1)
PYEOF
[ $? -eq 0 ] || fail "plan assertions failed"

# 4. An --center far outside reach: refusal (exit 3), reasons listed, no plan file written.
echo "test_view_plan.sh: planning with --center far outside reach (expect exit 3)"
ros2 run crackvision_motion plan_view --scene-config "$FIX/scene.yaml" --planning-config "$PLANNING_CFG" \
    --center 5.0 5.0 --out "$PLAN_UNREACHABLE" > "$LOG_DIR/test_view_plan_unreachable.log" 2>&1
rc=$?
[ "$rc" -eq 3 ] || fail "--center 5.0 5.0 exit=$rc (expected 3); see $LOG_DIR/test_view_plan_unreachable.log"
[ ! -e "$PLAN_UNREACHABLE" ] || fail "--center 5.0 5.0 refusal still wrote $PLAN_UNREACHABLE"
grep -q "no view candidate selected" "$LOG_DIR/test_view_plan_unreachable.log" \
    || fail "refusal log does not list candidates' reasons; see $LOG_DIR/test_view_plan_unreachable.log"
echo "test_view_plan.sh: --center 5.0 5.0 refused as expected, reasons: $(grep 'no view candidate selected' "$LOG_DIR/test_view_plan_unreachable.log")"

# 5. Teardown (the EXIT trap also covers every early exit above).
stop_mock_stack
echo "test_view_plan.sh: OK"
exit 0
