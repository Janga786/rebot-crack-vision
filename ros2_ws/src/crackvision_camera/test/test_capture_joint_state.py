"""Unit tests for capture_joint_state (CAM-05.2). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_camera/test/test_capture_joint_state.py -q -p no:cacheprovider
"""

import json
import sys
import threading
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "crackvision_motion"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "crackvision_description"))

from crackvision_camera.capture_joint_state import _wait_for_joint_state, main  # noqa: E402
from crackvision_motion.check_end_effector import ARM_JOINTS  # noqa: E402
from crackvision_motion.cli_common import PreconditionError  # noqa: E402

SCRAMBLED_NAMES = ["gripper_joint1", "joint3", "joint1", "gripper_joint2", "joint6", "joint2", "joint5", "joint4"]
POSITIONS_BY_NAME = {
    "gripper_joint1": 0.5, "gripper_joint2": 0.6,
    "joint1": 0.1, "joint2": 0.2, "joint3": 0.3, "joint4": 0.4, "joint5": 0.7, "joint6": 0.8,
}


def _scrambled_joint_state(*, with_header_stamp: bool) -> JointState:
    msg = JointState()
    msg.name = list(SCRAMBLED_NAMES)
    msg.position = [POSITIONS_BY_NAME[n] for n in SCRAMBLED_NAMES]
    if with_header_stamp:
        msg.header.stamp.sec = 123
        msg.header.stamp.nanosec = 456
    return msg


@pytest.fixture
def ros_context():
    rclpy.init(args=None)
    yield
    rclpy.shutdown()


def _run_main_with_background_publisher(argv, msg: JointState) -> int:
    """Run `main(argv)` (its own default rclpy context) while a *separate* context's node
    repeatedly publishes `msg` on /joint_states in a background thread."""
    ctx = rclpy.Context()
    rclpy.init(args=None, context=ctx)
    node = Node("test_capture_joint_state_bg_pub", context=ctx)
    pub = node.create_publisher(JointState, "/joint_states", 10)
    stop = threading.Event()

    def _spin():
        while not stop.is_set():
            pub.publish(msg)
            time.sleep(0.05)

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    try:
        return main(argv)
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown(context=ctx)


def test_wait_for_joint_state_raises_precondition_on_timeout_with_no_publisher(ros_context):
    node = Node("test_capture_joint_state_nopub")
    try:
        with pytest.raises(PreconditionError):
            _wait_for_joint_state(node, timeout_s=0.3)
    finally:
        node.destroy_node()


def test_wait_for_joint_state_ignores_messages_missing_required_joints(ros_context):
    node = Node("test_capture_joint_state_missing")
    pub = node.create_publisher(JointState, "/joint_states", 10)
    msg = JointState()
    msg.name = ["joint1", "joint2", "gripper_joint1"]  # missing joint3..joint6
    msg.position = [0.1, 0.2, 0.5]
    timer = node.create_timer(0.05, lambda: pub.publish(msg))
    try:
        with pytest.raises(PreconditionError):
            _wait_for_joint_state(node, timeout_s=0.5)
    finally:
        node.destroy_timer(timer)
        node.destroy_node()


def test_main_without_a_publisher_exits_3(tmp_path):
    code = main(["--root", str(tmp_path), "--timeout-s", "0.3"])
    assert code == 3


def test_main_reorders_filters_extras_and_uses_header_stamp(tmp_path):
    output = tmp_path / "js.json"
    code = _run_main_with_background_publisher(
        ["--root", str(tmp_path), "--timeout-s", "5.0", "--output", str(output)],
        _scrambled_joint_state(with_header_stamp=True),
    )
    assert code == 0

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["joint_names"] == list(ARM_JOINTS)
    assert payload["positions_rad"] == [POSITIONS_BY_NAME[j] for j in ARM_JOINTS]
    assert payload["stamp_ns"] == 123 * 1_000_000_000 + 456
    assert payload["stamp_source"] == "joint_states_header"


def test_main_falls_back_to_wall_clock_when_header_stamp_is_zero(tmp_path):
    output = tmp_path / "js.json"
    code = _run_main_with_background_publisher(
        ["--root", str(tmp_path), "--timeout-s", "5.0", "--output", str(output)],
        _scrambled_joint_state(with_header_stamp=False),
    )
    assert code == 0

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["stamp_source"] == "wall_clock_at_receipt"
    assert payload["stamp_ns"] > 0
