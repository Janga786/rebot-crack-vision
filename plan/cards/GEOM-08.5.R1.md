# GEOM-08.5.R1 — Repair GEOM-08.5: Interpolated (valid=True) points carry NaN depth_m/p_optical/normal_rm

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/lift3d.py",
      "tests/test_lift3d.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0.5",
    "docs/INTERFACES.md#3.13",
    "docs/INTERFACES.md#6",
    "docs/INTERFACES.md#9",
    "docs/INTERFACES.md#10",
    "src/crackvision/geometry.py",
    "src/crackvision/kinematics.py",
    "src/crackvision/capture_record.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_lift3d.py -q -p no:cacheprovider",
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
      "For every point with valid=True, including interpolated ones, depth_m, p_optical, p_base, n_base, normal_rms_m, sigma_normal_rad and cov_base are all finite. For example: p_optical = T⁻¹·p_base, depth_m = p_optical z, and normal_rms_m/sigma_normal_rad taken from the conservative neighbour. A test must assert np.isfinite on all of these over lifted.valid in a fixture that contains interpolated points.",
      "The test asserts that p_base[k] for an interpolated k equals (1-t)·p_base[left] + t·p_base[right], that n_base[k] is the normalised lerp and has unit norm, and that cov_base[k] equals the bounding neighbour's covariance with the larger trace. The neighbours must have different traces, for example through differing depths, so the test can tell the two choices apart.",
      "The FD Jacobian comparison runs at the pixel (row=ppy, col=ppx), for example by setting ppx/ppy to the centre of the image or by using a full-size image, and also at a genuinely off-axis pixel. Both comparisons pass.",
      "All acceptance criteria of GEOM-08.5 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "priority": 95,
  "repairs": "GEOM-08.5",
  "findings": [
    "b1c388bdd9dd9340",
    "11e98bd20f36243f",
    "d5c7454e7695e7b5"
  ],
  "id": "GEOM-08.5.R1",
  "title": "Repair GEOM-08.5: Interpolated (valid=True) points carry NaN depth_m/p_optical/normal_rm",
  "parent": "GEOM-08",
  "outcome": "Resolve the review findings on GEOM-08.5 while every acceptance criterion of GEOM-08.5 still holds."
}
```

Repair work generated deterministically from review attempt `00186-GEOM-08.5-review` of `GEOM-08.5`.
Original card: `plan/cards/GEOM-08.5.md` — its acceptance criteria must still hold.

## Finding 1 [major] Interpolated (valid=True) points carry NaN depth_m/p_optical/normal_rms_m/sigma_normal_rad
- Evidence: In lift3d.py _apply_gap_policy, interpolation writes only p_base, n_base, cov_base, reason, interpolated and valid. depth_m, p_optical, normal_rms_m and sigma_normal_rad stay at their NaN initial values. Probe output for indices 7-9: 'True True interpolated:no_valid_depth_in_annulus ... nan [nan nan nan] nan nan'. Summary: 'p_optical NaN in valid: 3', 'sigma_normal_rad NaN in valid: 3'.
- Consequence: This breaks the criterion 'No NaN leaks into valid points'. The §10.2 serializer would emit null geometry fields on points it counts in counts.valid ('points with non-null geometry'), or emit NaN, which is invalid JSON. Any consumer that indexes these fields over valid points gets NaN.
- Affected: src/crackvision/lift3d.py, tests/test_lift3d.py
- Acceptance condition: For every point with valid=True, including interpolated ones, depth_m, p_optical, p_base, n_base, normal_rms_m, sigma_normal_rad and cov_base are all finite. For example: p_optical = T⁻¹·p_base, depth_m = p_optical z, and normal_rms_m/sigma_normal_rad taken from the conservative neighbour. A test must assert np.isfinite on all of these over lifted.valid in a fixture that contains interpolated points.

## Finding 2 [minor] Gap test does not verify interpolated values or covariance inheritance
- Evidence: In test_gap_policy_interpolate_split_trim_and_drop, the 3-px gap checks only valid, interpolated, the reason prefix and `not np.any(np.isnan(lifted.p_base[i]))`. Nothing compares p_base[k] with the lerp of its neighbours, n_base with the nlerp, or cov_base with the larger-trace neighbour's covariance.
- Consequence: The criterion asks for tests of gap interpolation matching §10 exactly. A regression in the lerp parameter, the nlerp, or the choice of conservative covariance would go undetected.
- Affected: tests/test_lift3d.py
- Acceptance condition: The test asserts that p_base[k] for an interpolated k equals (1-t)·p_base[left] + t·p_base[right], that n_base[k] is the normalised lerp and has unit norm, and that cov_base[k] equals the bounding neighbour's covariance with the larger trace. The neighbours must have different traces, for example through differing depths, so the test can tell the two choices apart.

## Finding 3 [minor] Covariance FD test has no centre (principal-point) pixel
- Evidence: test_covariance_matches_finite_difference_jacobian uses make_intrinsics() with ppx=320.0 and ppy=240.0 on a 100×100 depth image, and tests pixels (50,50) and (80,20). The code comment calls (50,50) the 'centre pixel', but it is roughly 300 px off-axis.
- Consequence: The acceptance criterion's centre-pixel case, where x=y=0 and the J off-diagonal z-terms vanish, is never tested. The test's own comment misdescribes what it checks.
- Affected: tests/test_lift3d.py
- Acceptance condition: The FD Jacobian comparison runs at the pixel (row=ppy, col=ppx), for example by setting ppx/ppy to the centre of the image or by using a full-size image, and also at a genuinely off-axis pixel. Both comparisons pass.
