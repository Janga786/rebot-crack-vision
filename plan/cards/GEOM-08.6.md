# GEOM-08.6 — tool_tip trace/approach/retract waypoints from lifted segments (crackvision.tool_waypoints)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-08.1",
    "GEOM-08.5"
  ],
  "requirements": [
    "REQ-GEOM-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/tool_waypoints.py",
      "tests/test_tool_waypoints.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#8.2",
    "docs/INTERFACES.md#10",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "src/crackvision/lift3d.py",
    "src/crackvision/kinematics.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_tool_waypoints.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Every waypoint's rotated +X equals −n_out to 1e-9, quaternions are unit and in xyzw order, and position − surface_point = clearance·n_out.",
      "Resampling includes both segment ends, and consecutive trace waypoints are ≤ spacing + 1e-9 apart along the 3D polyline.",
      "The roll follows the §10 parallel-transport rule, including both fallbacks. Along a smoothly curving surface, the angle between consecutive +Z axes is ≤ the angle between consecutive normals + 1e-9 (no roll flips).",
      "Exactly one approach waypoint precedes and one retract waypoint follows each segment, at 0.04 m with the neighbour's orientation.",
      "σ_along_normal = sqrt(nᵀΣn + σ_tool²), and within_budget ⇔ 3σ ≤ clearance. Both are tested at the boundary."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 2,
    "consequence": 4,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-08.6",
  "parent": "GEOM-08",
  "title": "tool_tip trace/approach/retract waypoints from lifted segments (crackvision.tool_waypoints)",
  "outcome": "`crackvision.tool_waypoints.segment_waypoints` converts one lifted segment into §10 tool_tip poses in base_link. Positions sit at the 0.01 m trace clearance along the outward normal, resampled every 2 mm of arc length. +X runs along the anti-normal with a deterministic parallel-transport roll. Approach and retract poses sit at 0.04 m. Each waypoint carries σ_along_normal and a within_budget flag."
}
```

Create src/crackvision/tool_waypoints.py (numpy and scipy Rotation; no file I/O).

API:
- `@dataclass WaypointParams`: trace_clearance_m=0.01, approach_clearance_m=0.04, waypoint_spacing_m=0.002 (ADR-014 §2 nominal values; always recorded in the output).
- `@dataclass Waypoint`: phase, position (3), quat_xyzw (4), surface_point (3), normal (3), clearance_m, sigma_along_normal_m, sigma_max_m, interpolated, within_budget.
- `segment_waypoints(lifted: lift3d.LiftedPolyline, seg: (start, end), capture_tool_z_base: (3,), tool_sigma_pos_m: float, params) -> list[Waypoint]`.

Steps:
1. Take the segment's points in order (valid and interpolated). Compute the cumulative 3D arc length of p_base.
2. Sample s = 0, spacing, …, plus the final s = L (without duplicating it when L is a multiple of the spacing). At each sample, linearly interpolate the surface point, nlerp the normal, and take the covariance and `interpolated` flag of the nearer source point (interpolated if either neighbour is).
   - A zero-length segment (a single distinct point) yields one trace waypoint.
3. Orientation:
   - X = −n.
   - Z0 = capture_tool_z_base − (·X)X. If ‖Z0‖ < 1e-6, use base (0, 0, 1), then (1, 0, 0).
   - Z_{k+1} = Z_k − (Z_k·X_{k+1})X_{k+1}, normalised; if degenerate, use the same fallbacks.
   - Y = Z × X; R = [X Y Z] columns → quat_xyzw via scipy.
4. position = surface + clearance·n.
5. σ_along_normal = sqrt(nᵀΣn + tool_sigma²); sigma_max = sqrt(λ_max(Σ) + tool_sigma²); within_budget = 3·σ_along_normal ≤ clearance.
6. Prepend an approach waypoint (the first trace waypoint shifted to approach clearance, same R) and append a retract waypoint (the last one, likewise).

Also export `capture_tool_z(chain, end_effector, q) -> (3,)` = the Z column of FK(q)·T_gripper_link_tool_tip.

Tests:
- a planar straight crack;
- an arc on a cylinder of radius 0.1 m (the roll-continuity bound);
- a degenerate Z0 (capture tool Z parallel to the normal) exercising the fallback;
- spacing and endpoint inclusion;
- the approach/retract geometry;
- the σ/within_budget boundary.
