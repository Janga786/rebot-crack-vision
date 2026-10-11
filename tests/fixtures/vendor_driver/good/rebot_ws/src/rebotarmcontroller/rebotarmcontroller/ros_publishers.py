_GRIPPER_OPEN_MOTOR_POSITION = -5.0
_GRIPPER_CLOSED_MOTOR_POSITION = 0.0
_GRIPPER_MAX_WIDTH = 0.143


def _gripper_motor_to_joint_position(position: float) -> float:
    return position


def publish_joint_states(namespace):
    topic = f"/{namespace}/joint_states"
    return topic


def publish_arm_status(namespace):
    topic = f"/{namespace}/arm_status"
    return topic
