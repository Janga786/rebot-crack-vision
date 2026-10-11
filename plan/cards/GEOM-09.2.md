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
      "Invalid or missing per-point covariance inputs (e.g. invalid=True points) are refused or explicitly skipped rather than silently producing a zero/garbage contribution.",
      "The module also computes the §14.2 `fk` term per ADR-018 §3: Σ_base^(fk) = Σ_j (∂p_base/∂q_j · σ_joint_rad)², where ∂p_base/∂q is a central finite-difference Jacobian of the base_link point through crackvision.kinematics FK at the capture's joint positions. It is unit-tested by checking that it is exactly zero for σ_joint_rad = 0, that it scales with σ_joint_rad², and that it is positive semi-definite.",
      "The pixel+depth+handeye point terms sum (within float tolerance) to the source point's existing §10.4 cov_base. The full analytic total including fk equals pixel+depth+handeye+fk (INTERFACES §14.4). At waypoint level, nᵀΣ^(term)n for pixel/depth/handeye/fk plus σ_tool_pos² sums exactly to the new sigma_along_normal_m² (§14.5). Without fk, that sum equals the source sigma_along_normal_m²."
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

## Debugger note (D0004, 2026-10-11T04:49:51Z)
Debugger D0004. Decompose attempt 1 was rejected only because the single child's header omitted scope.write, acceptance, routing and track. Every child header MUST carry the full required shape: outcome, scope.write, acceptance.checks + criteria, routing, track, spec_state ready, motion false. This card is small enough for ONE child that copies scope.write [src/crackvision/uncertainty.py, tests/test_uncertainty.py] and the check `./env.sh pytest tests/test_uncertainty.py -q`. The previous child body was sound and can be reused. Contract gap: INTERFACES §14.2 fixes FIVE terms (pixel, depth, handeye, tcp, fk), and ADR-018 §3 defines the analytic fk term. GEOM-09.3 (MC) and GEOM-09.4 (CLI) do not implement it, so this library must expose the fk contribution as well (FK Jacobian via crackvision.kinematics, σ_joint_rad as a plain float argument). Interpolated points: per §14.5 the CLI uses the nearest source point's terms (§10.3 gap-fill). The library may refuse invalid points and refuse to decompose an interpolated point from its own p_optical, but it should make the nearest-source-point lookup possible for GEOM-09.4.
