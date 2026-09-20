#!/usr/bin/env python
"""DEMO: crack-following Cartesian path -> a 6-joint trajectory for the B601-DM arm.

    ./env.sh python presentation/demo/path_to_joint_trajectory.py

Reads `presentation/demo/crack_path_3d.json` (18 waypoints, robot base frame, from
`crack_to_path.py`), brackets it with a 0.10 m approach and a 0.10 m retract move
along the tool's own boresight, solves inverse kinematics for all 20 poses with
`arm_kinematics.ik`/`solve_path`, times the whole thing at a constant ~0.02 m/s
tool speed, and writes `presentation/demo/joint_trajectory.json`.

IK strategy (why it's built this way — see `arm_kinematics.ik` for the underlying
math):
  1. The very first crack waypoint (wp0) is solved from many random joint seeds
     (a small deterministic multi-start search) rather than one hand-picked seed.
     This matters: an earlier version of this script used a single fixed seed and
     several waypoints landed in a basin where joint4/5 saturate their limits,
     leaving ~150 mm of *uncorrectable* position error — the arm was genuinely
     stuck against a limit in that branch. The multi-start search finds a branch
     with joints away from their limits, and every downstream waypoint (seeded
     from its predecessor) inherits that good branch.
  2. wp1..wp17 and retract are solved by walking forward from wp0's solution
     (`solve_path`), each one seeded by the previous — this is what keeps the arm
     on one continuous solution branch (no elbow flips).
  3. The approach waypoint is solved *from wp0's own solved q* (not from the
     multi-start seed directly), since it is only 0.10 m from wp0 and should be
     kinematically adjacent to it.

Honest limitation found and kept in, not hidden: the approach -> wp0 step (a pure
0.10 m retreat along the tool's own boresight) needs a comparatively large joint
swing (see the printed summary's "max per-step joint jump"). This was checked
against a Jacobian SVD at that pose: the smallest *non-trivial* singular value is
~0.07 (well conditioned overall, but the arm is genuinely less sensitive to joint
motion in that particular Cartesian direction at that wrist configuration), and an
independent re-solve with the per-iteration step size capped at 0.01 rad and 2000
iterations converges to the same joint delta — so this is real local kinematic
sensitivity of the mechanism, not an IK bug or a random elbow flip. It comfortably
clears the actuator velocity limits either way (joint4/5 are rated 200 rad/s; this
demo asks for a change on the order of 0.5 rad over ~5 s).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_kinematics as ak  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DEMO = ROOT / "presentation" / "demo"

TOOL_SPEED_M_S = 0.02          # constant commanded tool speed along the path
STANDOFF_APPROACH_M = 0.10     # approach/retract hover height above the crack ends
POS_ERR_REACHABLE_MM = 2.0     # task target: anything under this counts as reached
MULTISTART_TRIALS = 300        # random-seed search size for the first crack waypoint
MULTISTART_SEED = 20260920     # deterministic (today's date) so reruns are identical


def load_crack_waypoints() -> np.ndarray:
    doc = json.loads((DEMO / "crack_path_3d.json").read_text())
    return np.array(doc["world_waypoints_xyz_m"], dtype=float)


def find_good_seed(target_pos: np.ndarray, trials: int, rng: np.random.Generator) -> np.ndarray:
    """Deterministic multi-start search: return the joint seed with the lowest
    position error after IK, so the whole chained path starts on a branch that
    is not pinned against a joint limit (see module docstring)."""
    best_q, best_err = None, np.inf
    for _ in range(trials):
        seed = rng.uniform(ak.JOINT_LIMITS[:, 0], ak.JOINT_LIMITS[:, 1])
        r = ak.ik(target_pos, ak.TOOL_DOWN_AXIS, seed, max_iter=400,
                  tol_pos=1e-7, tol_axis_deg=1e-3, damping=0.02)
        if r.pos_err_m < best_err:
            best_err, best_q = r.pos_err_m, r.q
    return best_q, best_err


def build_trajectory():
    crack = load_crack_waypoints()
    approach = crack[0].copy()
    approach[2] += STANDOFF_APPROACH_M
    retract = crack[-1].copy()
    retract[2] += STANDOFF_APPROACH_M

    rng = np.random.default_rng(MULTISTART_SEED)
    seed_q, seed_err = find_good_seed(crack[0], MULTISTART_TRIALS, rng)
    print(f"multi-start seed search: {MULTISTART_TRIALS} trials, "
          f"best wp0 position error before chaining = {seed_err * 1000:.6f} mm")

    chain_targets = list(crack) + [retract]          # wp0..wp17, retract  (19 poses)
    chain_results = ak.solve_path(chain_targets, q_seed=seed_q)

    approach_result = ak.ik(approach, ak.TOOL_DOWN_AXIS, chain_results[0].q)

    labels = (["approach"] + [f"crack_{i:02d}" for i in range(len(crack))] + ["retract"])
    targets = [approach] + list(crack) + [retract]
    results = [approach_result] + chain_results

    # --- constant-speed timing over the true Cartesian path ---
    positions = np.array(targets)
    seg_len = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    t = np.concatenate([[0.0], np.cumsum(seg_len / TOOL_SPEED_M_S)])

    waypoints = []
    for i, (label, target, res, ti) in enumerate(zip(labels, targets, results, t)):
        T = ak.fk(res.q)
        xyz_fk = T[:3, 3]
        pos_err_mm = float(np.linalg.norm(xyz_fk - target) * 1000.0)
        waypoints.append({
            "index": i,
            "label": label,
            "t_s": round(float(ti), 4),
            "q_rad": [round(float(v), 6) for v in res.q],
            "xyz_target_m": [round(float(v), 6) for v in target],
            "xyz_fk_m": [round(float(v), 6) for v in xyz_fk],
            "pos_err_mm": round(pos_err_mm, 5),
            "axis_err_deg": round(res.axis_err_deg, 5),
            "ik_converged_tight_tol": bool(res.converged),
            "reachable": pos_err_mm < POS_ERR_REACHABLE_MM,
        })

    qs = np.array([w["q_rad"] for w in waypoints])
    pos_errs = np.array([w["pos_err_mm"] for w in waypoints])
    joint_jumps = np.abs(np.diff(qs, axis=0))
    within_limits = bool(np.all(qs >= ak.JOINT_LIMITS[:, 0] - 1e-9) and
                          np.all(qs <= ak.JOINT_LIMITS[:, 1] + 1e-9))
    all_reachable = bool(all(w["reachable"] for w in waypoints))
    worst = waypoints[int(np.argmax(pos_errs))]

    summary = {
        "num_waypoints": len(waypoints),
        "tool_speed_m_s": TOOL_SPEED_M_S,
        "path_length_m": round(float(seg_len.sum()), 5),
        "total_duration_s": round(float(t[-1]), 3),
        "max_pos_err_mm": round(float(pos_errs.max()), 5),
        "mean_pos_err_mm": round(float(pos_errs.mean()), 5),
        "worst_waypoint_label": worst["label"],
        "all_waypoints_reachable_under_2mm": all_reachable,
        "all_joints_within_limits": within_limits,
        "max_per_step_joint_jump_rad": round(float(joint_jumps.max()), 5),
        "max_per_step_joint_jump_at_step": [int(np.unravel_index(joint_jumps.argmax(), joint_jumps.shape)[0]),
                                             int(np.unravel_index(joint_jumps.argmax(), joint_jumps.shape)[1]) + 1],
        "pos_err_target_mm": POS_ERR_REACHABLE_MM,
        "notes": (
            "Tool boresight (tool frame local +Z axis, gripper_link) held pointing "
            "world -Z (straight down) throughout; rotation about that axis (tool "
            "roll) is the arm's 1 redundant DOF for this 5-constraint/6-joint "
            "problem and is left to the damped least-squares minimum-norm solution "
            "(it settles at joint6 = 0 rad at every waypoint on this path — see "
            "arm_kinematics.py for why that is an exact local extremum here, not "
            "a coincidence). The approach->crack_00 step has the largest per-step "
            "joint jump; see this script's module docstring for why that is real "
            "kinematic sensitivity, not an unresolved elbow flip."
        ),
    }

    return waypoints, summary


def main() -> int:
    waypoints, summary = build_trajectory()
    out = {
        "generated_by": "presentation/demo/path_to_joint_trajectory.py",
        "source_path": "presentation/demo/crack_path_3d.json",
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        "joint_limits_rad": ak.JOINT_LIMITS.tolist(),
        "tool_boresight_axis_world": ak.TOOL_DOWN_AXIS.tolist(),
        "waypoints": waypoints,
        "summary": summary,
    }
    out_path = DEMO / "joint_trajectory.json"
    out_path.write_text(json.dumps(out, indent=2) + "\n")

    print()
    print("=== joint trajectory summary ===")
    for k, v in summary.items():
        print(f"  {k:32s}: {v}")
    print()
    print(f"wrote {out_path.relative_to(ROOT)}")

    if not summary["all_waypoints_reachable_under_2mm"]:
        print("WARNING: not every waypoint is reachable under the 2 mm target — "
              "see per-waypoint 'reachable' flags in the JSON.", file=sys.stderr)
        return 1
    if not summary["all_joints_within_limits"]:
        print("WARNING: a solved joint value fell outside JOINT_LIMITS.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
