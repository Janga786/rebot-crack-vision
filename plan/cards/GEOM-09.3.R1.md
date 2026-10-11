# GEOM-09.3.R1 — Repair GEOM-09.3: uncertainty_mc.py reimplements deprojection/waypoint geometry instead

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-GEOM-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/uncertainty_mc.py",
      "tests/test_uncertainty_mc.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#14. Uncertainty budget",
    "src/crackvision/uncertainty.py",
    "src/crackvision/lift3d.py",
    "src/crackvision/tool_waypoints.py",
    "src/crackvision/kinematics.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_uncertainty_mc.py -q",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "uncertainty_mc.py imports and calls `crackvision.lift3d.lift_polyline` (or an equivalent single-point entry point into the real surface-depth/normal/covariance logic) for the point case, and `crackvision.tool_waypoints`'s real waypoint-construction function(s) for the waypoint case, per perturbed sample, rather than reimplementing deprojection/position-offset arithmetic inline.",
      "All acceptance criteria of GEOM-09.3 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 2,
    "task_class": "numerical_library",
    "local_ok": false
  },
  "track": "calibration",
  "priority": 99,
  "repairs": "GEOM-09.3",
  "findings": [
    "d26d11bf029f1fb0"
  ],
  "id": "GEOM-09.3.R1",
  "title": "Repair GEOM-09.3: uncertainty_mc.py reimplements deprojection/waypoint geometry instead",
  "parent": "GEOM-09",
  "outcome": "Resolve the review findings on GEOM-09.3 while every acceptance criterion of GEOM-09.3 still holds."
}
```

Repair work generated deterministically from review attempt `00744-GEOM-09.3-integration` of `GEOM-09.3`.
Original card: `plan/cards/GEOM-09.3.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] uncertainty_mc.py reimplements deprojection/waypoint geometry instead of calling the real lift3d/tool_waypoints functions
- Evidence: src/crackvision/uncertainty_mc.py:32 imports only `geometry, kinematics` (no `lift3d`, no `tool_waypoints`). `_draw_perturbed_p_base` calls `geometry.deproject_pixels` directly rather than `lift3d.lift_polyline` (which implements the real annulus surface-depth sampler, plane-fit normal, and §10.3 gap/interpolation/segment-drop policy per docs/INTERFACES.md §10.3-10.4). `mc_waypoint_sigma` computes `tool_tip_sample = p_base + clearance_m * n + d_tool` inline rather than calling `tool_waypoints._waypoint`/`segment_waypoints`, which also derive the real roll-axis/quaternion frame and `sigma_max_m`/`within_budget`.
- Consequence: The MC cross-check validates a hand-derived linear(ish) approximation of the geometry pipeline against `crackvision.uncertainty`'s analytic linearization, rather than cross-checking the analytic decomposition against the actual non-linear lift3d/tool_waypoints behavior (annulus depth aggregation, gap interpolation, plane-fit normal noise, real roll-axis construction). This defeats the purpose of a Monte-Carlo cross-check and could mask real disagreement between the analytic model and the true pipeline that only manifests through lift3d's surface-depth/normal-fit logic or tool_waypoints' frame construction.
- Affected: src/crackvision/uncertainty_mc.py, tests/test_uncertainty_mc.py
- Acceptance condition: uncertainty_mc.py imports and calls `crackvision.lift3d.lift_polyline` (or an equivalent single-point entry point into the real surface-depth/normal/covariance logic) for the point case, and `crackvision.tool_waypoints`'s real waypoint-construction function(s) for the waypoint case, per perturbed sample, rather than reimplementing deprojection/position-offset arithmetic inline.
