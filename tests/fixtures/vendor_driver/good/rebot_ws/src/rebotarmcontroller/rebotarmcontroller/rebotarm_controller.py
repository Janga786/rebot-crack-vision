from rclpy.node import Node


class reBotArmController(Node):
    def __init__(self) -> None:
        self.declare_parameter("arm_namespace", "rebotarm")
