# GEOM-08.8.R1 — Repair GEOM-08.8: 15° view tilt (optical axis vs anti-normal) is never exercised; both s

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2",
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "docs/COMPLETION_LOG.md",
      "docs/geometry/PATH3D_VERIFICATION.md",
      "tests/test_path3d_synthetic.py",
      "tools/synth_scene3d.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#3.13",
    "docs/INTERFACES.md#9",
    "docs/INTERFACES.md#10",
    "src/crackvision/path3d.py",
    "src/crackvision/paths.py",
    "src/crackvision/kinematics.py",
    "src/crackvision/geometry.py",
    "src/crackvision/capture_record.py",
    "config/robot/end_effector.yaml",
    "config/motion/specimen_placement.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "synthetic-e2e",
        "cmd": "./env.sh pytest tests/test_path3d_synthetic.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      },
      {
        "id": "evidence-doc",
        "cmd": "test -s docs/geometry/PATH3D_VERIFICATION.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "A scene in which the angle between the camera optical axis and the plane anti-normal is 15° (asserted in the test, e.g. the arccos of the optical-frame normal's z equals 15°±0.1°). The surface may also be tilted independently. On that scene, all noise-free tolerances (median ≤0.5 mm, max ≤1.5 mm, normal ≤1°, +X·n_true ≤ −cos1°, clearance ≤0.5 mm) pass, the doc reports the measured numbers, and the 'projectively equivalent' claim is removed.",
      "All acceptance criteria of GEOM-08.8 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 4,
    "consequence": 4,
    "task_class": "verification-synthetic",
    "local_ok": false
  },
  "track": "calibration",
  "priority": 95,
  "repairs": "GEOM-08.8",
  "findings": [
    "e35e5ab7fc2c9686"
  ],
  "id": "GEOM-08.8.R1",
  "title": "Repair GEOM-08.8: 15° view tilt (optical axis vs anti-normal) is never exercised; both s",
  "parent": "GEOM-08",
  "outcome": "Resolve the review findings on GEOM-08.8 while every acceptance criterion of GEOM-08.8 still holds."
}
```

Repair work generated deterministically from review attempt `00203-GEOM-08.8-review` of `GEOM-08.8`.
Original card: `plan/cards/GEOM-08.8.md` — its acceptance criteria must still hold.

## Finding 1 [major] 15° view tilt (optical axis vs anti-normal) is never exercised; both scenes are fronto-parallel in the camera frame
- Evidence: tools/synth_scene3d.py camera_target_pose sets `z_axis = -normal`, which puts the boresight exactly on the anti-normal for every --tilt-deg. tests/test_path3d_synthetic.py::test_capture_distance_and_anti_normal_angle asserts `cos_angle > 1.0 - 1e-9`. ADR-014 line 71 defines the view phase as 'optical axis within 15° of the surface anti-normal'. My script (generate_case at tilt 0/15) printed the normal in the optical frame as [~0, ~0, -1] with a view angle of 1.2e-06° and 1.5e-06°. The doc's 'Interpretation of --tilt-deg' paragraph claims this is 'projectively equivalent' to a 15° off-axis view; that is false, because a camera rigidly rotated together with the plane sees an identical image.
- Consequence: The oblique-view geometry is never tested: annulus depth gradient, plane-fit normal off the optical axis, and covariance rotation relative to the normal. The 0.000° normal error is trivially guaranteed. Downstream cards (GEOM-09 budget, MOT) would rely on accuracy numbers at the 15° band edge that were never measured.
- Affected: tools/synth_scene3d.py, tests/test_path3d_synthetic.py, docs/geometry/PATH3D_VERIFICATION.md
- Acceptance condition: A scene in which the angle between the camera optical axis and the plane anti-normal is 15° (asserted in the test, e.g. the arccos of the optical-frame normal's z equals 15°±0.1°). The surface may also be tilted independently. On that scene, all noise-free tolerances (median ≤0.5 mm, max ≤1.5 mm, normal ≤1°, +X·n_true ≤ −cos1°, clearance ≤0.5 mm) pass, the doc reports the measured numbers, and the 'projectively equivalent' claim is removed.
