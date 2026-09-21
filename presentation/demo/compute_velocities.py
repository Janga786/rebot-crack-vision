#!/usr/bin/env python
"""DEMO: linear/angular tool velocity and joint velocity along the solved
inspection trajectory.

    ./env.sh python presentation/demo/compute_velocities.py

Reads presentation/demo/joint_trajectory.json (20 poses, already IK-solved by
path_to_joint_trajectory.py) and derives, PER SEGMENT between consecutive
poses:
  - joint angular velocity  qdot   = (q[i+1]-q[i]) / dt        (rad/s, 6-vector)
  - tool linear velocity    v_lin  = (p[i+1]-p[i]) / dt         (m/s, 3-vector)
  - tool angular velocity   omega  = axis-angle(R[i+1] R[i]^T) / dt  (rad/s, 3-vector)

These are SEGMENT-AVERAGE finite differences between the 20 already-solved
keyframes, not a finer re-sampling of the path -- an honest simplification,
named as such in the output. `ak.fk(q)` (the same forward kinematics used to
solve the trajectory) supplies both position and full orientation for the
angular-velocity axis-angle computation; joint_trajectory.json only stored
position (`xyz_fk_m`) and a scalar boresight-axis error, not the rotation
matrix, so it is recomputed here rather than reused.

Everything here is presentation scaffolding (see module docstrings in
crack_to_path.py / arm_kinematics.py) -- nothing in src/ imports it.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEMO = ROOT / "presentation" / "demo"
sys.path.insert(0, str(DEMO))
import arm_kinematics as ak  # noqa: E402

JOINT_VEL_LIMIT_RAD_S = {
    "joint1": 50.0, "joint2": 50.0, "joint3": 50.0,
    "joint4": 200.0, "joint5": 200.0, "joint6": 200.0,
}


def rotation_axis_angle(R: np.ndarray) -> tuple[np.ndarray, float]:
    """A 3x3 rotation matrix -> (unit axis, angle in radians), via the
    standard Rodrigues log map. Returns a zero axis for a (near-)identity
    rotation rather than dividing by ~0."""
    cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = float(np.arccos(cos_theta))
    if theta < 1e-9:
        return np.zeros(3), 0.0
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2.0 * np.sin(theta))
    return axis, theta


def main() -> int:
    doc = json.load(open(DEMO / "joint_trajectory.json"))
    wps = doc["waypoints"]
    names = doc["joint_names"]

    ts = np.array([w["t_s"] for w in wps])
    qs = np.array([w["q_rad"] for w in wps])
    Rs = [ak.fk(q)[:3, :3] for q in qs]
    ps = np.array([ak.fk(q)[:3, 3] for q in qs])  # recomputed FK position, cross-check vs xyz_fk_m
    stored_ps = np.array([w["xyz_fk_m"] for w in wps])
    max_fk_cross_check_mm = float(np.max(np.linalg.norm(ps - stored_ps, axis=1)) * 1000.0)

    segments = []
    max_lin_speed = 0.0
    max_ang_speed = 0.0
    max_qdot_ratio = 0.0  # fraction of that joint's own velocity limit
    worst_joint = None

    for i in range(len(wps) - 1):
        dt = float(ts[i + 1] - ts[i])
        if dt <= 0:
            continue
        dp = ps[i + 1] - ps[i]
        v_lin = dp / dt
        speed = float(np.linalg.norm(v_lin))

        R_rel = Rs[i + 1] @ Rs[i].T
        axis, theta = rotation_axis_angle(R_rel)
        omega = axis * theta / dt
        ang_speed = float(np.linalg.norm(omega))

        dq = qs[i + 1] - qs[i]
        qdot = dq / dt
        for jn, qd in zip(names, qdot):
            ratio = abs(qd) / JOINT_VEL_LIMIT_RAD_S[jn]
            if ratio > max_qdot_ratio:
                max_qdot_ratio = ratio
                worst_joint = jn

        max_lin_speed = max(max_lin_speed, speed)
        max_ang_speed = max(max_ang_speed, ang_speed)

        segments.append({
            "from": wps[i]["label"], "to": wps[i + 1]["label"],
            "t_mid_s": float((ts[i] + ts[i + 1]) / 2.0), "dt_s": dt,
            "midpoint_xyz_m": [round(v, 5) for v in ((ps[i] + ps[i + 1]) / 2.0).tolist()],
            "v_lin_m_s": [round(v, 5) for v in v_lin.tolist()],
            "speed_m_s": round(speed, 5),
            "omega_rad_s": [round(v, 5) for v in omega.tolist()],
            "angular_speed_rad_s": round(ang_speed, 5),
            "qdot_rad_s": [round(v, 5) for v in qdot.tolist()],
        })

    out = {
        "generated_by": "presentation/demo/compute_velocities.py",
        "status": "DEMO - segment-average finite differences over the 20 solved keyframes",
        "method": "v_lin = dp/dt; omega = axis-angle(R_i+1 R_i^T)/dt; qdot = dq/dt",
        "fk_cross_check_vs_stored_xyz_fk_mm_max": round(max_fk_cross_check_mm, 5),
        "joint_velocity_limits_rad_s": JOINT_VEL_LIMIT_RAD_S,
        "summary": {
            "max_linear_speed_m_s": round(max_lin_speed, 5),
            "max_angular_speed_rad_s": round(max_ang_speed, 5),
            "max_angular_speed_deg_s": round(np.degrees(max_ang_speed), 3),
            "designed_constant_tool_speed_m_s": 0.02,
            "max_joint_velocity_fraction_of_limit": round(max_qdot_ratio, 5),
            "worst_joint": worst_joint,
            "note": "tool angular velocity stays low across the 18-crack-waypoint inspection pass "
                    "itself, because the IK holds the boresight straight down at every one of those "
                    "keyframes (see axis_err_deg in joint_trajectory.json) -- the wrist does not "
                    "wobble even though joints 1-3 sweep substantially (see fig04). It is NOT low "
                    "at the retract transition: that waypoint is one of two (with 'approach') that "
                    "did not fully converge (see joint_trajectory.json's per-waypoint 'reachable' "
                    "flag and docs/TECHNICAL_APPROACH.md sec 2.5), so it exits on a different, "
                    "farther solution branch and the resulting joint6 swing shows up here as this "
                    "run's single largest angular-speed segment. Segment-average, not a finer "
                    "resampling: a transient between keyframes could differ slightly.",
        },
        "segments": segments,
    }
    out_path = DEMO / "velocity_trajectory.json"
    out_path.write_text(json.dumps(out, indent=2) + "\n")

    print(f"FK cross-check vs stored xyz_fk_m: max {max_fk_cross_check_mm:.5f} mm")
    print(f"max linear speed:   {max_lin_speed:.4f} m/s  (designed constant: 0.02 m/s)")
    print(f"max angular speed:  {max_ang_speed:.5f} rad/s ({np.degrees(max_ang_speed):.3f} deg/s)")
    print(f"worst joint velocity: {worst_joint} at {max_qdot_ratio*100:.2f}% of its limit")
    print(f"wrote {out_path.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
