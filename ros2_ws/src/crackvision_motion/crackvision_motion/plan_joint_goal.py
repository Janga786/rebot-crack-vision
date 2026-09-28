"""Headless MoveIt joint-space planning smoke test for the B601-DM mock stack (MOT-02).

Sends a single plan-only MoveGroup action goal for the "arm" group and exits 0 if OMPL
reports MoveItErrorCodes.SUCCESS, 1 otherwise. No trajectory is ever executed and no
hardware is touched -- this only exercises the planning pipeline against
mock_components/GenericSystem, per the no-robot-motion-from-this-loop rule.
"""

import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node

# Within config/robot/b601_dm_limits.yaml bounds and collision-free for the canonical
# B601-DM model (verified manually against rebotarm_moveit_config's default collision
# scene before being hard-coded here).
DEFAULT_JOINT_GOAL = {
    "joint1": 0.3,
    "joint2": -0.5,
    "joint3": -0.4,
    "joint4": 0.1,
    "joint5": -0.15,
    "joint6": 0.2,
}

GROUP_NAME = "arm"
SERVER_WAIT_TIMEOUT_S = 90.0
GOAL_ACCEPT_TIMEOUT_S = 15.0
RESULT_TIMEOUT_S = 20.0


def build_goal(joint_goal: dict) -> MoveGroup.Goal:
    goal = MoveGroup.Goal()
    request = MotionPlanRequest()
    request.group_name = GROUP_NAME
    request.allowed_planning_time = 5.0
    request.num_planning_attempts = 5
    constraints = Constraints()
    for joint_name, position in joint_goal.items():
        jc = JointConstraint()
        jc.joint_name = joint_name
        jc.position = position
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    request.goal_constraints.append(constraints)
    goal.request = request
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = True
    return goal


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = Node("plan_joint_goal")
    client = ActionClient(node, MoveGroup, "/move_action")
    try:
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_S):
            node.get_logger().error("move_group action server '/move_action' did not appear")
            return 1

        send_future = client.send_goal_async(build_goal(DEFAULT_JOINT_GOAL))
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=GOAL_ACCEPT_TIMEOUT_S)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            node.get_logger().error("MoveGroup goal was rejected")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=RESULT_TIMEOUT_S)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            node.get_logger().error("timed out waiting for MoveGroup plan result")
            return 1

        error_code = wrapped_result.result.error_code.val
        if error_code == MoveItErrorCodes.SUCCESS:
            node.get_logger().info(f"plan succeeded for group '{GROUP_NAME}' (error_code={error_code})")
            return 0
        node.get_logger().error(f"plan failed for group '{GROUP_NAME}' (error_code={error_code})")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
