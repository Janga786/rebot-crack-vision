# GEOM-04.5 — Full synthetic ground-truth verification of the hand-eye pipeline (+ evidence note)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-04.4"
  ],
  "requirements": [
    "REQ-GEOM-2",
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "tools/synth_handeye_scene.py",
      "tests/test_handeye_synthetic.py",
      "docs/calibration/HANDEYE_VERIFICATION.md",
      "docs/COMPLETION_LOG.md"
    ]
  },
  "inputs": [
    "docs/calibration/PLAN.md#4",
    "docs/calibration/PLAN.md#6",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "config/robot/end_effector.yaml",
    "src/crackvision/calibration/charuco.py",
    "src/crackvision/calibration/handeye.py",
    "src/crackvision/calibration/handeye_capture.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "synthetic-e2e",
        "cmd": "./env.sh pytest tests/test_handeye_synthetic.py -q -p no:cacheprovider",
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
        "cmd": "test -s docs/calibration/HANDEYE_VERIFICATION.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "tools/synth_handeye_scene.py writes a poses directory in exactly the format handeye_capture.py consumes, for a chosen ground-truth T_gripper_link_camera_link and a fixed T_base_target, using charuco.render_synthetic_view for every pose's image (real rendered images through the real detector, not bypassed).",
      "Two scenarios are covered: (a) the actual ADR-014 CAD-prior pose read from config/robot/end_effector.yaml's wrist_camera block (15 deg pitch, -Z mount side); (b) a deliberately mirrored/flipped mount-side variant of that same pose.",
      "Pose generation covers PLAN §2's diversity requirement (>=3 tilt bands, >=4 azimuth headings) and produces >=15 solve + >=4 held-out poses per scenario, at a configurable seed.",
      "The end-to-end test runs handeye_capture.run_solve (or the CLI) on scenario (a) and asserts: layer-1/2/3 all pass under an injected noise level stated explicitly in the test and the evidence doc (pixel-detection noise + a plausible FK-pose noise), recovered T_gripper_link_camera_link matches ground truth within a tolerance tighter than the layer-3 threshold, and check_against_cad_prior does NOT flag it.",
      "The same test on scenario (b) asserts check_against_cad_prior DOES flag it (>10 mm or >5 deg per ADR-014 §5) — this is the concrete regression PLAN.md/ADR-014 ask for ('Synthetic tests must include the 15 deg stock pitch and the -Z mount side').",
      "sigma_calib_from_held_out's reported numbers for scenario (a) are sanity-checked (order of magnitude, not exact) against the injected noise level.",
      "docs/calibration/HANDEYE_VERIFICATION.md records the measured numbers for both scenarios, the injected noise levels, seeds and exact commands to reproduce, and states plainly that this verifies algorithmic/software correctness under a simulated nominal chain, not physical accuracy on the real rig — that is GEOM-05's job.",
      "If a threshold fails, report the numbers and suspected cause; do not loosen PLAN §4's thresholds or edit charuco.py/handeye.py/handeye_capture.py to force a pass — file a repair card instead."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4,
    "task_class": "verification-synthetic",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-04.5",
  "parent": "GEOM-04",
  "title": "Full synthetic ground-truth verification of the hand-eye pipeline (+ evidence note)",
  "outcome": "A deterministic synthetic hand-eye dataset generator plus an end-to-end test drives the full charuco -> handeye -> handeye_capture pipeline against a known T_gripper_link_camera_link (both the ADR-014 CAD-prior 15 deg / -Z-mount pose and a deliberately wrong-mount-side pose), confirming the docs/calibration/PLAN.md §4 thresholds are met under injected noise and that the CAD-prior disagreement flag correctly fires for the wrong-mount case. This is the evidence PLAN.md §6 requires from GEOM-04."
}
```

1. `tools/synth_handeye_scene.py`: an importable library plus a small CLI (`./env.sh python tools/synth_handeye_scene.py --out DIR --scenario cad_prior|wrong_mount_side --seed N [--pixel-noise-px P] [--pose-noise-m M --pose-noise-deg D]`). Reads `config/robot/end_effector.yaml`'s `wrist_camera` block for the CAD-prior scenario's ground-truth `T_gripper_link_camera_link`; for `wrong_mount_side`, mirror it about the mount-side axis (document exactly which reflection, matching ADR-014's own description of what a wrong-side mount would look like). Fabricate a plausible `T_base_gripper_i` pose set spanning PLAN §2's tilt/azimuth bands (these do not need to come from real robot kinematics — GEOM-08.2's FK module is a different branch and not a dependency here; any diverse, physically-plausible-looking pose set is fine as long as it is documented as synthetic) and a fixed `T_base_target`. For each pose, render the board image via `charuco.render_synthetic_view` with the requested pixel noise, and perturb the recorded `T_base_gripper_i` by the requested pose noise before writing it (so the *recorded* pose has FK-like noise while the *rendered* image is generated from the true pose — this is what actually exercises the solver's noise tolerance, not a synthetic image bypass).
2. `tests/test_handeye_synthetic.py`: generate both scenarios in a tmp dir, run `handeye_capture.run_solve`, assert per the criteria above. Keep total runtime under ~10 minutes on CPU (no GPU needed); if rendering is slow, reduce image resolution rather than pose/held-out counts (PLAN's floors must stay honoured).
3. `docs/calibration/HANDEYE_VERIFICATION.md`: the measured numbers table (both scenarios, all three layers, recovered pose vs ground truth, CAD-prior check result), noise levels and seeds, exact commands, and the accuracy-vs-hardware disclaimer. Append one entry to `docs/COMPLETION_LOG.md`.
