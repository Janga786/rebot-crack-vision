#!/usr/bin/env python3
"""scripts/geometry/compare_fk_parity.py — FK parity: crackvision.kinematics vs MoveIt (GEOM-08.3).

Loads a dump produced by scripts/ros/fk_parity_dump.py (MoveIt's own /compute_fk, run
against the headless mock stack) and compares each sample's gripper_link, tool_tip and
camera_link pose against the pure-numpy `crackvision.kinematics` FK chain composed with
the repo's `config/robot/end_effector.yaml` transforms. Independent evidence that 3D
paths lifted through crackvision.kinematics land in the same base_link MoveIt plans in.

Usage:
    ./env.sh python scripts/geometry/compare_fk_parity.py DUMP.json [--root PATH]
                                                           [--tol-pos-m 1e-6] [--tol-rot-rad 1e-6]

Exit codes (docs/INTERFACES.md §0.2): 0 all links within tolerance, 1 any link exceeds
tolerance on any sample, 2 malformed dump.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from crackvision.kinematics import KinematicsError, load_chain, load_end_effector, transform_from_xyz_quat

EXIT_OK = 0
EXIT_FAIL = 1
EXIT_MALFORMED = 2

TOL_POS_M = 1e-6
TOL_ROT_RAD = 1e-6


def _rotation_angle(r_a: np.ndarray, r_b: np.ndarray) -> float:
    rel = r_a.T @ r_b
    cos_theta = np.clip((np.trace(rel) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cos_theta))


def _ours_poses(chain, end_effector, q: dict[str, float]) -> dict[str, np.ndarray]:
    t_base_gripper = chain.fk(q)
    return {
        "gripper_link": t_base_gripper,
        "tool_tip": t_base_gripper @ end_effector.T_gripper_link_tool_tip,
        "camera_link": t_base_gripper @ end_effector.T_gripper_link_camera_link,
    }


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("dump", type=Path, help="JSON dump from scripts/ros/fk_parity_dump.py")
    p.add_argument("--root", type=Path, default=None, help="project root override (unused, §0.3 parity)")
    p.add_argument("--tol-pos-m", type=float, default=TOL_POS_M)
    p.add_argument("--tol-rot-rad", type=float, default=TOL_ROT_RAD)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        doc = json.loads(args.dump.read_text(encoding="utf-8"))
        joint_names = list(doc["joint_names"])
        samples = doc["samples"]
        if not samples:
            raise ValueError("dump has zero samples")
        for s in samples:
            if set(s["q"]) != set(joint_names) or set(s["links"]) != {"gripper_link", "tool_tip", "camera_link"}:
                raise ValueError("sample missing expected joints/links")
    except (OSError, json.JSONDecodeError, KeyError, ValueError, TypeError) as exc:
        print(f"compare_fk_parity: malformed dump {args.dump}: {exc}", file=sys.stderr)
        return EXIT_MALFORMED

    try:
        chain = load_chain()
        end_effector = load_end_effector()
    except (KinematicsError, OSError) as exc:
        print(f"compare_fk_parity: {exc}", file=sys.stderr)
        return EXIT_MALFORMED

    max_pos_err = {link: 0.0 for link in ("gripper_link", "tool_tip", "camera_link")}
    max_rot_err = {link: 0.0 for link in ("gripper_link", "tool_tip", "camera_link")}
    ok = True

    for sample in samples:
        q = {name: float(sample["q"][name]) for name in joint_names}
        ours = _ours_poses(chain, end_effector, q)
        for link, t_ours in ours.items():
            moveit_link = sample["links"][link]
            t_moveit = transform_from_xyz_quat(moveit_link["xyz"], moveit_link["quat_xyzw"])
            pos_err = float(np.linalg.norm(t_ours[:3, 3] - t_moveit[:3, 3]))
            rot_err = _rotation_angle(t_moveit[:3, :3], t_ours[:3, :3])
            max_pos_err[link] = max(max_pos_err[link], pos_err)
            max_rot_err[link] = max(max_rot_err[link], rot_err)
            if pos_err > args.tol_pos_m or rot_err > args.tol_rot_rad:
                ok = False

    print(f"compare_fk_parity: {len(samples)} samples, seed={doc.get('seed')}")
    print(f"{'link':<14}{'max |dp| (m)':<16}{'max |dtheta| (rad)':<20}")
    for link in ("gripper_link", "tool_tip", "camera_link"):
        print(f"{link:<14}{max_pos_err[link]:<16.3e}{max_rot_err[link]:<20.3e}")

    if not ok:
        print(f"compare_fk_parity: FAIL — exceeded tol-pos-m={args.tol_pos_m} or "
              f"tol-rot-rad={args.tol_rot_rad}", file=sys.stderr)
        return EXIT_FAIL

    print("compare_fk_parity: OK")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
