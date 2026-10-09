"""capture_optical_tf -- read camera_link -> camera_color_optical_frame off realsense2_camera's TF (CAM-05.2).

console_script: `ros2 run crackvision_camera capture_optical_tf`. One-shot: look up the transform once,
write GEOM-08.4's --optical-tf JSON, exit. ADR-012 §6.2: this is the only way this transform may be
obtained -- never hand-build it from extrinsics metadata. Requires realsense2_camera (or an equivalent
TF publisher) to already be running.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Dict

if TYPE_CHECKING:
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node

from crackvision_motion.cli_common import (
    EXIT_OK,
    EXIT_PRECONDITION,
    PreconditionError,
    RunSummary,
    add_common_args,
    find_root,
    run_cli,
    setup_logging,
)

TOOL = "capture_optical_tf"
TARGET_FRAME = "camera_link"
SOURCE_FRAME = "camera_color_optical_frame"
DEFAULT_OUTPUT = "data/calibration/camera_optical_tf.json"
DEFAULT_TIMEOUT_S = 10.0
_POLL_S = 0.1


def _lookup(node: "Node", timeout_s: float) -> "TransformStamped":
    """Poll (via spin_once) until `SOURCE_FRAME` -> `TARGET_FRAME` is available, else raise PreconditionError.

    rclpy/tf2_ros are imported here (not at module scope) so that argparse-only invocations
    (e.g. --help) work with just crackvision_camera/crackvision_motion on PYTHONPATH.
    """
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    buffer = Buffer()
    TransformListener(buffer, node)
    deadline = time.monotonic() + timeout_s
    while True:
        if buffer.can_transform(TARGET_FRAME, SOURCE_FRAME, Time()):
            return buffer.lookup_transform(TARGET_FRAME, SOURCE_FRAME, Time())
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise PreconditionError(
                f"no TF from '{SOURCE_FRAME}' to '{TARGET_FRAME}' within {timeout_s}s -- "
                "is realsense2_camera running? "
                "ros2 launch realsense2_camera rs_launch.py ..."
            )
        rclpy.spin_once(node, timeout_sec=min(_POLL_S, remaining))


def _payload(transform: "TransformStamped") -> Dict:
    t = transform.transform.translation
    q = transform.transform.rotation
    return {
        "xyz_m": [t.x, t.y, t.z],
        "quat_xyzw": [q.x, q.y, q.z, q.w],
        "target_frame": TARGET_FRAME,
        "source_frame": SOURCE_FRAME,
        "captured_at_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }


def _run(args: argparse.Namespace, log, summary: RunSummary) -> Dict[str, str]:
    import rclpy
    from rclpy.node import Node

    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    output = Path(args.output).resolve() if args.output else root / DEFAULT_OUTPUT

    rclpy.init(args=None)
    node = Node(TOOL)
    try:
        transform = _lookup(node, args.timeout_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    payload = _payload(transform)
    summary.increment("transforms_captured")
    log.info("captured %s -> %s: xyz_m=%s quat_xyzw=%s", SOURCE_FRAME, TARGET_FRAME,
              payload["xyz_m"], payload["quat_xyzw"])

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    log.info("wrote %s", output)
    return {"status": "ok"}


def _dry_run(args: argparse.Namespace) -> int:
    """--dry-run performs the TF lookup (to prove it would succeed) but writes no file."""
    import rclpy
    from rclpy.node import Node

    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    log_dir = root / "logs"
    logger = setup_logging(TOOL, log_dir, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()
    status, exit_code = "ok", EXIT_OK
    try:
        rclpy.init(args=None)
        node = Node(TOOL)
        try:
            transform = _lookup(node, args.timeout_s)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        payload = _payload(transform)
        logger.info("--dry-run: TF lookup succeeded xyz_m=%s quat_xyzw=%s; writing nothing",
                    payload["xyz_m"], payload["quat_xyzw"])
    except PreconditionError as exc:
        logger.error("%s", exc)
        summary.add_error(str(exc))
        status, exit_code = "precondition", EXIT_PRECONDITION
    summary.write(log_dir, TOOL, status, exit_code)
    return exit_code


def _extra_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--timeout-s", type=float, default=DEFAULT_TIMEOUT_S,
                        help="seconds to wait for the TF before exit 3")
    parser.add_argument("--output", type=Path, default=None,
                        help=f"output JSON path (default: {DEFAULT_OUTPUT} under the repo root)")


def main(argv=None) -> int:
    argv = list(argv) if argv is not None else sys.argv[1:]
    parser = argparse.ArgumentParser(prog=TOOL)
    add_common_args(parser)
    _extra_args(parser)
    args = parser.parse_args(argv)
    if args.dry_run:
        return _dry_run(args)
    return run_cli(TOOL, _run, argv, extra_args=_extra_args)


if __name__ == "__main__":
    sys.exit(main())
