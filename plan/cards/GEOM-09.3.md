# GEOM-09.3 — Monte-Carlo cross-check of the pixel-to-tool-pose uncertainty chain (crackvision.uncertainty_mc)

```json card
{
  "kind": "branch",
  "depends_on": [
    "GEOM-09.1",
    "GEOM-09.2"
  ],
  "requirements": [
    "REQ-GEOM-1"
  ],
  "refine_after": [
    "GEOM-09.1"
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
      "The sampler perturbs exactly the per-term distributions §14 names (pixel detection, depth noise, hand-eye sigma_calib, TCP sigma), reruns the real lift3d/tool_waypoints/kinematics code paths (not a re-derived approximation) per sample, and accumulates an empirical covariance.",
      "A deterministic seed makes the test reproducible; the sample count is bounded so the test suite runs in a reasonable time (document the bound in the test).",
      "On a case where the analytic linearization is known-good (small sigmas, near-linear regime), the empirical MC covariance agrees with crackvision.uncertainty's analytic breakdown within the §14 tolerance, and the test asserts this agreement.",
      "A case engineered to break the linear approximation (e.g. large rotation sigma) is shown to produce a disagreement the module's agreement statistic actually flags, proving the check is not vacuous."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 2,
    "task_class": "numerical_library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-09.3",
  "title": "Monte-Carlo cross-check of the pixel-to-tool-pose uncertainty chain (crackvision.uncertainty_mc)",
  "parent": "GEOM-09",
  "outcome": "crackvision/uncertainty_mc.py resamples the pixel->base_link->tool_tip chain per §14's perturbation distributions and reports empirical covariance plus an agreement statistic against crackvision.uncertainty's analytic result."
}
```

Implement `crackvision/uncertainty_mc.py`: a Monte-Carlo cross-check of GEOM-09.2's analytic decomposition, per the §14 contract.

For a given lifted point or tool waypoint and its input sigmas, draw N samples perturbing pixel location, depth, hand-eye extrinsic (rotation+translation) and TCP offset per §14's documented distributions; for each sample, recompute the point/waypoint by calling the real `lift3d`/`tool_waypoints`/`kinematics` functions with the perturbed inputs (not a simplified model). Accumulate the empirical covariance of the resulting 3D points/poses across samples, and compare it to `crackvision.uncertainty`'s analytic total via the §14 agreement statistic (e.g. eigenvalue or trace ratio).

Keep this pure (no CLI, no file I/O) — GEOM-09.4 wires it into the CLI/report. Bound N so `tests/test_uncertainty_mc.py` runs quickly; use a fixed seed. Include both an agreeing case and a deliberately non-linear case that the statistic flags, so the check has discriminating power.

## Decomposed 2026-10-11T14:41:44Z by L-GEOM (claude-sonnet-5, high)
Children: GEOM-09.3.1
