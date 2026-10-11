class ArmActions:
    def __init__(self, node, hardware, namespace):
        action = f"/{namespace}/follow_joint_trajectory"

    def execute_gripper_command(self, goal_handle):
        goal = goal_handle.request.command
        self._hardware.set_gripper_target(goal.position)
