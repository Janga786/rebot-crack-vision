"""capture_joint_state -- read /joint_states and write GEOM-08.4's --joint-state JSON (CAM-05.2).

console_script: `ros2 run crackvision_camera capture_joint_state`. One-shot: subscribe once, wait for a
message naming all six arm joints, write it, exit.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING, Dict

if TYPE_CHECKING:
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

from crackvision_motion.cli_common import PreconditionError, RunSummary, find_root, run_cli, utc_stamp

TOOL = "capture_joint_state"
TOPIC = "/joint_states"
DEFAULT_TIMEOUT_S = 10.0
_POLL_S = 0.1


def _wait_for_joint_state(node: "Node", timeout_s: float) -> "JointState":
    """Spin `node` until a message on `TOPIC` names all of ARM_JOINTS, else raise PreconditionError.

    rclpy/sensor_msgs/ARM_JOINTS are imported here (not at module scope) so that argparse-only
    invocations (e.g. --help) work with just crackvision_camera/crackvision_motion on PYTHONPATH.
    """
    import rclpy
    from sensor_msgs.msg import JointState

    from crackvision_motion.check_end_effector import ARM_JOINTS

    box: Dict[str, JointState] = {}

    def _cb(msg: JointState) -> None:
        if "msg" not in box and set(ARM_JOINTS).issubset(msg.name):
            box["msg"] = msg

    sub = node.create_subscription(JointState, TOPIC, _cb, 10)
    deadline = time.monotonic() + timeout_s
    try:
        while "msg" not in box:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise PreconditionError(
                    f"no message on '{TOPIC}' containing all of {ARM_JOINTS} within {timeout_s}s"
                )
            rclpy.spin_once(node, timeout_sec=min(_POLL_S, remaining))
    finally:
        node.destroy_subscription(sub)
    return box["msg"]


def _run(args: argparse.Namespace, log, summary: RunSummary) -> Dict[str, str]:
    import rclpy
    from rclpy.node import Node

    from crackvision_motion.check_end_effector import ARM_JOINTS

    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    output = (Path(args.output).resolve() if args.output
              else root / "data" / "calibration" / f"joint_state_{utc_stamp()}.json")

    rclpy.init(args=None)
    node = Node(TOOL)
    try:
        msg = _wait_for_joint_state(node, args.timeout_s)
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            stamp_source = "joint_states_header"
        else:
            stamp_ns = node.get_clock().now().nanoseconds
            stamp_source = "wall_clock_at_receipt"
    finally:
        node.destroy_node()
        rclpy.shutdown()

    positions = [float(msg.position[msg.name.index(j)]) for j in ARM_JOINTS]
    payload = {
        "joint_names": list(ARM_JOINTS),
        "positions_rad": positions,
        "stamp_ns": int(stamp_ns),
        "stamp_source": stamp_source,
    }
    summary.increment("joint_states_captured")
    log.info("captured joint state: %s", payload)

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    log.info("wrote %s", output)
    return {"status": "ok"}


def _extra_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--timeout-s", type=float, default=DEFAULT_TIMEOUT_S,
                        help="seconds to wait for /joint_states before exit 3")
    parser.add_argument("--output", type=Path, default=None,
                        help="output JSON path (default: data/calibration/joint_state_<UTC>.json under the repo root)")


def main(argv=None) -> int:
    argv = list(argv) if argv is not None else sys.argv[1:]
    return run_cli(TOOL, _run, argv, extra_args=_extra_args)


if __name__ == "__main__":
    sys.exit(main())
