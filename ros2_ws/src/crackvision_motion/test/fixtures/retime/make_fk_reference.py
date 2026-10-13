#!/usr/bin/env python3
"""Reference-fixture generator/checker for `crackvision_motion.tool_fk` parity (MOT-07.2).

Runs under `./env.sh python` (needs the conda-side `crackvision` package; `tool_fk` itself never
imports it). Computes base_link->tool_tip poses for the all-zero joint state plus `--n-configs`
seeded configurations within `config/robot/b601_dm_limits.yaml`'s joint1..6 limits, via
`crackvision.kinematics.load_chain(...).fk(q) @ load_end_effector(...).T_gripper_link_tool_tip` --
the exact composition `test_tool_fk.py` checks `crackvision_motion.tool_fk` against.

Usage:
  ./env.sh python ros2_ws/src/crackvision_motion/test/fixtures/retime/make_fk_reference.py
      regenerate fk_reference.json next to this script

  ./env.sh python ros2_ws/src/crackvision_motion/test/fixtures/retime/make_fk_reference.py --check
      recompute and byte-compare against the committed fixture; writes nothing; exits non-zero on
      any numeric difference or if the URDF/end_effector.yaml sha256 the fixture recorded has
      drifted (fixture freshness)
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from crackvision.config import find_root
from crackvision.kinematics import file_sha256, load_chain, load_end_effector, transform_to_xyz_quat

HERE = Path(__file__).resolve().parent
SEED = 20260101
N_CONFIGS_DEFAULT = 24
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


def _joint_limits() -> dict:
    import yaml

    limits_path = find_root() / "config" / "robot" / "b601_dm_limits.yaml"
    cfg = yaml.safe_load(limits_path.read_text())
    return {name: (cfg["joints"][name]["lower"], cfg["joints"][name]["upper"]) for name in JOINT_NAMES}


def _seeded_configs(n_configs: int) -> list:
    limits = _joint_limits()
    rng = np.random.default_rng(SEED)
    lowers = np.array([limits[name][0] for name in JOINT_NAMES])
    uppers = np.array([limits[name][1] for name in JOINT_NAMES])
    configs = [[0.0] * 6]
    for _ in range(n_configs):
        q = rng.uniform(lowers, uppers)
        configs.append([round(float(v), 12) for v in q])
    return configs


def _build(n_configs: int) -> dict:
    urdf_path = find_root() / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf"
    end_effector_path = find_root() / "config" / "robot" / "end_effector.yaml"

    chain = load_chain(urdf_path)
    end_effector = load_end_effector(end_effector_path)

    configs = _seeded_configs(n_configs)
    poses = []
    for q in configs:
        T = chain.fk(q) @ end_effector.T_gripper_link_tool_tip
        xyz, quat = transform_to_xyz_quat(T)
        poses.append({
            "position_m": [round(v, 12) for v in xyz],
            "quat_xyzw": [round(v, 12) for v in quat],
        })

    return {
        "schema": "crackvision_motion.tool_fk_reference/1",
        "seed": SEED,
        "joint_names": JOINT_NAMES,
        "urdf": {"path": "presentation/sim/reBot_B601_DM_with_gripper.urdf", "sha256": file_sha256(urdf_path)},
        "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": file_sha256(end_effector_path)},
        "configs": configs,
        "poses": poses,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="recompute and compare, writing nothing")
    parser.add_argument("--n-configs", type=int, default=N_CONFIGS_DEFAULT,
                        help=f"number of seeded configurations in addition to all-zero (default {N_CONFIGS_DEFAULT})")
    args = parser.parse_args()

    fixture_path = HERE / "fk_reference.json"
    fresh = _build(args.n_configs)

    if not args.check:
        fixture_path.write_text(json.dumps(fresh, indent=1) + "\n")
        print(f"wrote {fixture_path}")
        return 0

    if not fixture_path.is_file():
        print(f"missing fixture: {fixture_path}", flush=True)
        return 1
    committed = json.loads(fixture_path.read_text())
    if committed != fresh:
        print(f"fixture is stale: {fixture_path} no longer matches a fresh regeneration", flush=True)
        return 1
    print(f"fixture is fresh: {fixture_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
