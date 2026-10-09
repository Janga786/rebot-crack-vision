"""Unit tests for capture_optical_tf (CAM-05.2). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_camera/test/test_capture_optical_tf.py -q -p no:cacheprovider
"""

import json
import sys
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "crackvision_motion"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "crackvision_description"))

from crackvision_camera.capture_optical_tf import _lookup, _payload, main  # noqa: E402
from crackvision_motion.cli_common import PreconditionError  # noqa: E402

XYZ = (0.01, -0.02, 0.03)
QUAT_XYZW = (0.0, 0.0, 0.70710678, 0.70710678)


def _publish_known_transform(node: Node) -> StaticTransformBroadcaster:
    broadcaster = StaticTransformBroadcaster(node)
    msg = TransformStamped()
    msg.header.frame_id = "camera_link"
    msg.child_frame_id = "camera_color_optical_frame"
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = XYZ
    (msg.transform.rotation.x, msg.transform.rotation.y,
     msg.transform.rotation.z, msg.transform.rotation.w) = QUAT_XYZW
    broadcaster.sendTransform(msg)
    return broadcaster


@pytest.fixture
def ros_context():
    rclpy.init(args=None)
    yield
    rclpy.shutdown()


def test_lookup_round_trips_a_known_published_transform(ros_context):
    node = Node("test_capture_optical_tf_pub")
    broadcaster = _publish_known_transform(node)
    try:
        result = _lookup(node, timeout_s=5.0)
    finally:
        node.destroy_node()

    t = result.transform.translation
    q = result.transform.rotation
    assert (t.x, t.y, t.z) == pytest.approx(XYZ, abs=1e-6)
    assert (q.x, q.y, q.z, q.w) == pytest.approx(QUAT_XYZW, abs=1e-6)
    del broadcaster


def test_lookup_without_a_publisher_raises_precondition_error(ros_context):
    node = Node("test_capture_optical_tf_nopub")
    try:
        with pytest.raises(PreconditionError):
            _lookup(node, timeout_s=0.3)
    finally:
        node.destroy_node()


def test_main_without_a_publisher_exits_3(tmp_path):
    code = main(["--root", str(tmp_path), "--timeout-s", "0.3"])
    assert code == 3


def test_main_dry_run_without_a_publisher_exits_3_and_writes_no_output(tmp_path):
    output = tmp_path / "data" / "calibration" / "camera_optical_tf.json"
    code = main(["--root", str(tmp_path), "--timeout-s", "0.3", "--dry-run", "--output", str(output)])
    assert code == 3
    assert not output.exists()


def test_payload_matches_the_tf_json_shape_within_tolerance(ros_context):
    node = Node("test_capture_optical_tf_payload")
    broadcaster = _publish_known_transform(node)
    try:
        transform = _lookup(node, timeout_s=5.0)
    finally:
        node.destroy_node()
    del broadcaster

    payload = _payload(transform)
    assert payload["xyz_m"] == pytest.approx(list(XYZ), abs=1e-6)
    assert payload["quat_xyzw"] == pytest.approx(list(QUAT_XYZW), abs=1e-6)
    assert payload["target_frame"] == "camera_link"
    assert payload["source_frame"] == "camera_color_optical_frame"
    assert "captured_at_utc" in payload
    # must round-trip through json.dumps exactly as the tool writes it
    reparsed = json.loads(json.dumps(payload))
    assert reparsed == payload
