# GEOM-09.2 — Analytic per-term uncertainty decomposition library (crackvision.uncertainty)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-09.1"
  ],
  "requirements": [
    "REQ-GEOM-1"
  ],
  "spec_state": "draft",
  "refine_after": [
    "GEOM-09.1"
  ],
  "scope": {
    "write": [
      "src/crackvision/uncertainty.py",
      "tests/test_uncertainty.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#14. Uncertainty budget",
    "docs/INTERFACES.md#10. Robot-frame 3D crack paths and tool waypoints",
    "src/crackvision/lift3d.py",
    "src/crackvision/tool_waypoints.py",
    "src/crackvision/kinematics.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_uncertainty.py -q",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "For a given lifted point/waypoint, the sum of the returned per-term covariance contributions equals (within float tolerance) the total covariance that GEOM-08.5/.6 already compute via §10.4, i.e. this module decomposes rather than recomputes a different total.",
      "Each term (pixel/annulus noise, depth-sensor noise, hand-eye sigma_calib rot+pos, TCP/pivot sigma) is individually attributable and unit-tested by zeroing out all other inputs and checking the remaining term matches a hand-derived or closed-form expectation.",
      "The module is pure (no file I/O, no CLI) per the existing lift3d/tool_waypoints convention; it imports but does not modify crackvision.lift3d, crackvision.tool_waypoints or crackvision.kinematics.",
      "Invalid or missing per-point covariance inputs (e.g. invalid=True points) are refused or explicitly skipped rather than silently producing a zero/garbage contribution."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 2,
    "task_class": "numerical_library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-09.2",
  "title": "Analytic per-term uncertainty decomposition library (crackvision.uncertainty)",
  "parent": "GEOM-09",
  "outcome": "crackvision/uncertainty.py computes, for a lifted 3D point or tool waypoint, the named per-term covariance contributions (pixel noise, depth noise, hand-eye calibration, TCP calibration) defined by §14, consistent with the existing §10.4 total."
}
```

Implement `crackvision/uncertainty.py` per the §14 contract GEOM-09.1 produces.

GEOM-08.5's `lift3d` already builds a first-order total covariance per lifted point from pixel/annulus sampling noise, a plane-fit normal and `CalibSigmas` (hand-eye `cam_pos_m`/`cam_rot_rad`). GEOM-08.6's `tool_waypoints` carries that forward plus TCP/pivot calibration (GEOM-06) into tool-pose waypoints. This card adds a decomposition layer: reusing the same Jacobian pieces already present in those modules (do not duplicate the geometry, import and reuse the helper functions/Jacobians), compute each named term's contribution to the final covariance separately, so the sum reproduces the existing total.

Expose a small dataclass (e.g. `UncertaintyBreakdown`) with one field per §14-named term plus the total, and a function that takes a lifted point (or waypoint) plus the same `LiftParams`/`CalibSigmas` inputs lift3d already takes, and returns the breakdown. Unit tests: breakdown sums to the existing total on real lift3d/tool_waypoints output, and each term vanishes when its corresponding input sigma is set to zero.
