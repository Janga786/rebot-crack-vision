# GEOM-08.8 — Synthetic ground-truth verification of pixel→base_link 3D paths and tool waypoints (+ evidence note)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-08.7"
  ],
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2",
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "tools/synth_scene3d.py",
      "tests/test_path3d_synthetic.py",
      "docs/geometry/PATH3D_VERIFICATION.md",
      "docs/COMPLETION_LOG.md"
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
      "Ground truth is generated independently of lift3d. Rendering uses ray/plane intersection in base_link from the chain FK(q)·T_gl_cl·T_cl_opt, with pixels produced by forward projection (model 'none'). lift3d code is not reused to make the truth.",
      "Noise-free tilted plane (0° and 15° view tilt at 0.25 m, the ADR-014 view pose, placed over the specimen_placement centre): median 3D error ≤ 0.5 mm and max ≤ 1.5 mm vs the GT curve, normals ≤ 1°, trace-waypoint +X·n_true ≤ −cos(1°), clearance error ≤ 0.5 mm.",
      "Crack-cavity depth (≥ 3 mm deeper under the mask) does not bias the surface points beyond the noise-free bounds.",
      "With Gaussian depth noise per the GEOM-02 stereo model, ≥ 90% of valid points lie within 3·sigma_base along each axis, computed with the calibration term disabled via a zero-sigma measured end_effector fixture so that only sensor terms are tested.",
      "Injected invalid-depth holes produce the §10 interpolation and split behaviour and reasons. A record without a robot block → exit 3. A nominal yaml → execution_eligible false.",
      "The evidence doc reports the measured numbers, seeds and commands. It states plainly that this verifies software consistency under the nominal chain, not physical accuracy, which is GEOM-05/07 and INT-04/05's job."
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
  "id": "GEOM-08.8",
  "parent": "GEOM-08",
  "title": "Synthetic ground-truth verification of pixel→base_link 3D paths and tool waypoints (+ evidence note)",
  "outcome": "A deterministic synthetic scene generator renders a known specimen surface under the nominal eye-in-hand chain, and an end-to-end test drives skeleton → crackvision.paths → crackvision.path3d. The test shows recovered 3D paths, normals and tool poses match ground truth within stated tolerances, with reported uncertainty consistent with injected noise. The measured numbers are recorded in docs/geometry/PATH3D_VERIFICATION.md."
}
```

1. `tools/synth_scene3d.py`: an importable library and a small CLI (`./env.sh python tools/synth_scene3d.py --out ROOT --seed N [--tilt-deg 0|15] [--depth-noise] [--holes]`).
   - Surface: a plane in base_link through the specimen_placement centre (read config/motion/specimen_placement.yaml; fall back to the ADR-014 (0.29, 0, table z) if keys differ, and say so), with a known normal (tilt about base Y by 0° or 10°).
   - Capture pose: choose q by a simple numeric IK on `kinematics` FK (scipy least_squares) so that camera_link is 0.25 m from the centre with the optical axis at the requested tilt to the anti-normal. Assert that the residual is < 1e-6 and that q is within limits.
   - Intrinsics: 848×480, fx = fy ≈ 430, model 'none', depth_scale 1e-4.
   - Depth: per pixel, intersect the pixel ray (base_link, via the chain) with the plane → optical z → uint16. Inside the crack mask, add a cavity of +3 mm. Optionally add Gaussian noise σ = depth_uncertainty_m(z) and zero-depth holes (one 3-px and one 10-px along the crack).
   - Crack: a smooth 3D curve on the plane (a sine arc ≈ 60 mm long), forward-projected to pixels, rasterised 1-px 8-connected, and dilated to a ~5-px-wide mask. Write the skeleton PNG, mask, case_map entry, colour placeholder, depth PNG, and a §9 record (`synthetic: true`, `optical_source: nominal_d405`) via `capture_record.build_record`. Also write the GT JSON (3D curve samples, normal).
2. `tests/test_path3d_synthetic.py`: in a tmp root, generate → run `crackvision.paths` main() on the skeleton → run `crackvision.path3d` main() → compare against GT.
   - For each lifted point, take the nearest-GT-curve distance. Check normal angles and waypoint geometry, using the tolerances in the criteria.
   - Noise-consistency test with a fixed seed.
   - The hole and refusal cases.
   - Keep the runtime under 10 minutes on CPU; no GPU needed.
3. `docs/geometry/PATH3D_VERIFICATION.md`: the measured numbers table, seeds and commands, plus the limitations: nominal chain, pinhole-only intrinsics, no FK/encoder or depth-bias error, synthetic surfaces. These feed GEOM-09's budget. Append to COMPLETION_LOG.md.

If a tolerance fails, report the numbers and the suspected cause. Do not loosen tolerances or edit the lift3d, tool_waypoints or path3d modules (file a repair instead).
