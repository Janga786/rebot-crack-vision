from rclpy.qos import qos_profile_sensor_data


def subscribe_joint_states(node, topic, cb):
    qos = qos_profile_sensor_data
    return node.create_subscription(JointState, topic, cb, qos)
