#!/usr/bin/env python3
"""scripts/ros/fk_parity_dump.py — dump MoveIt's /compute_fk for GEOM-08.3.

Read-only: it calls only the /compute_fk service on the running mock MoveIt stack
(scripts/ros/mock_planning.launch.py via scripts/ros/test_fk_parity.sh). It never calls
execute or trajectory actions, and never touches controllers.

Builds the all-zero joint state plus `--samples` seeded uniform draws (numpy
`default_rng(seed)`) within each arm joint's [lower, upper] from `--limits`, sets the
gripper joints to 0 (they do not affect gripper_link/tool_tip/camera_link), and asks
MoveIt for FK of gripper_link, tool_tip and camera_link in base_link for every sample.
Writes the result as JSON to `--out` for scripts/geometry/compare_fk_parity.py.

Usage:
    scripts/ros/fk_parity_dump.py --out PATH [--seed N] [--samples 20]
                                   [--limits config/robot/b601_dm_limits.yaml]

Exit codes (docs/INTERFACES.md §0.2): 0 success, 1 /compute_fk returned a non-SUCCESS
error code, 2 usage/config error (malformed limits file), 3 /compute_fk not available
within 30 s.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node

EXIT_OK = 0
EXIT_RUNTIME = 1
EXIT_USAGE = 2
EXIT_PRECONDITION = 3

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
GRIPPER_JOINTS = ("gripper_joint1", "gripper_joint2")
FK_LINKS = ("gripper_link", "tool_tip", "camera_link")
SERVICE_WAIT_S = 30.0


def _load_limits(path: Path) -> dict:
    with open(path, "r") as f:
        cfg = yaml.safe_load(f)
    joints = cfg["joints"]
    out = {}
    for name in ARM_JOINTS:
        spec = joints[name]
        out[name] = (float(spec["lower"]), float(spec["upper"]))
    return out


def _joint_sets(limits: dict, seed: int, samples: int) -> list[dict]:
    rng = np.random.default_rng(seed)
    sets = [{j: 0.0 for j in ARM_JOINTS}]
    for _ in range(samples):
        q = {j: float(rng.uniform(limits[j][0], limits[j][1])) for j in ARM_JOINTS}
        sets.append(q)
    return sets


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True, help="output JSON path")
    p.add_argument("--seed", type=int, default=0, help="numpy default_rng seed")
    p.add_argument("--samples", type=int, default=20, help="number of random joint sets")
    p.add_argument(
        "--limits", type=Path,
        default=Path(__file__).resolve().parents[2] / "config" / "robot" / "b601_dm_limits.yaml",
        help="joint limits YAML",
    )
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        limits = _load_limits(args.limits)
    except (OSError, KeyError, ValueError, yaml.YAMLError) as exc:
        print(f"fk_parity_dump: malformed limits file {args.limits}: {exc}", file=sys.stderr)
        return EXIT_USAGE

    joint_sets = _joint_sets(limits, args.seed, args.samples)

    rclpy.init(args=None)
    node = Node("fk_parity_dump")
    try:
        client = node.create_client(GetPositionFK, "/compute_fk")
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            print("fk_parity_dump: /compute_fk not available within 30s", file=sys.stderr)
            return EXIT_PRECONDITION

        samples_out = []
        for q in joint_sets:
            state = {**q, **{j: 0.0 for j in GRIPPER_JOINTS}}
            req = GetPositionFK.Request()
            req.header.frame_id = "base_link"
            req.fk_link_names = list(FK_LINKS)
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(state)
            req.robot_state.joint_state.position = [float(v) for v in state.values()]

            fut = client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            if not fut.done() or fut.result() is None:
                print("fk_parity_dump: /compute_fk call failed or timed out", file=sys.stderr)
                return EXIT_RUNTIME
            resp = fut.result()
            if resp.error_code.val != MoveItErrorCodes.SUCCESS:
                print(f"fk_parity_dump: /compute_fk returned error code {resp.error_code.val}",
                      file=sys.stderr)
                return EXIT_RUNTIME

            links = {}
            for name, ps in zip(resp.fk_link_names, resp.pose_stamped):
                pos = ps.pose.position
                ori = ps.pose.orientation
                links[name] = {
                    "xyz": [pos.x, pos.y, pos.z],
                    "quat_xyzw": [ori.x, ori.y, ori.z, ori.w],
                }
            samples_out.append({"q": q, "links": links})
    finally:
        node.destroy_node()
        rclpy.shutdown()

    out_doc = {
        "seed": args.seed,
        "joint_names": list(ARM_JOINTS),
        "samples": samples_out,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out_doc, indent=2), encoding="utf-8")
    print(f"fk_parity_dump: wrote {len(samples_out)} samples to {args.out}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
