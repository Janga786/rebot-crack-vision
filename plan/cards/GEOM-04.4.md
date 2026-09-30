# GEOM-04.4 — Capture-pose assembly + hand-eye solve CLI (crackvision.calibration.handeye_capture)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-04.2",
    "GEOM-04.3"
  ],
  "requirements": [
    "REQ-GEOM-2",
    "REQ-CAM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/calibration/handeye_capture.py",
      "tests/test_handeye_capture.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/calibration/PLAN.md#2",
    "docs/calibration/PLAN.md#6",
    "src/crackvision/calibration/charuco.py",
    "src/crackvision/calibration/handeye.py",
    "src/crackvision/config.py",
    "src/crackvision/logging_setup.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_handeye_capture.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "cli-help",
        "cmd": "./env.sh python -m crackvision.calibration.handeye_capture --help",
        "timeout_s": 60,
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
      "The per-pose record format (documented in the module docstring, since neither PLAN.md nor docs/INTERFACES.md fixes one — same precedent as GEOM-06/07 leaving the on-disk pose format to the implementing card) captures exactly what PLAN §2 requires per pose: the FK-reported T_base_link_gripper_link (quat_xyzw + translation_m), a reference to the raw colour frame, the held-out flag, and the measured square/marker length used for that session.",
      "The CLI follows docs/INTERFACES.md §0 exactly: common flags, exit codes 0/1/2/3, --dry-run writes nothing, logs/handeye_capture_*.json + logs/handeye_capture_latest.json summaries.",
      "Enforces PLAN §2's real minimums at the CLI layer (distinct from handeye.py's algorithmic MIN_POSES=3 floor): exit 3 if fewer than 15 solve poses or fewer than 4 held-out poses are present after the layer-1 gate drops any failing pose.",
      "Runs charuco.detect on every listed image, drops (and reports, never silently discards without a count) any pose failing the layer-1 gate, then calls handeye.solve_hand_eye + cross_method_agreement + held_out_residuals + check_against_cad_prior + sigma_calib_from_held_out on the remainder.",
      "Exit code reflects the worst layer result: 0 only if layer-1 (no forced pose loss beyond what's reported), layer-2 and layer-3 all pass; 1 if the CLI ran to completion but any layer failed (the evidence JSON still records every number, pass or fail — never suppressed).",
      "Evidence JSON (data/calibration/handeye_evidence_<timestamp>.json, never committed) records: per-pose layer-1 detection results, the full layer-2 cross-method pairwise table, the layer-3 held-out numbers, every method's recovered T_gripper_link_camera_link, the CAD-prior comparison, and sigma_calib — enough for GEOM-05 to copy a measured wrist_camera block from it.",
      "Tests build a tmp poses directory from crackvision.calibration.charuco.render_synthetic_view (real rendered images, not bypassed detection) plus a known ground-truth X, and cover: a full passing run; the exit-3 too-few-poses path; --dry-run writes nothing; bad args -> exit 2; a missing/corrupt image -> the pose is reported and dropped, not a crash."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-04.4",
  "parent": "GEOM-04",
  "title": "Capture-pose assembly + hand-eye solve CLI (crackvision.calibration.handeye_capture)",
  "outcome": "crackvision.calibration.handeye_capture defines the on-disk per-pose record format for a hand-eye capture session and a CLI that loads a directory of them, runs ChArUco detection, enforces PLAN's pose-count/held-out floors, solves via crackvision.calibration.handeye, and writes a single evidence JSON with every layer-1/2/3 number plus the recovered T_gripper_link_camera_link and sigma_calib — the tool GEOM-05 will point at a real capture session."
}
```

Implement `src/crackvision/calibration/handeye_capture.py`.

Record format (document in the module docstring): a directory of per-pose JSON files (or one manifest JSON listing entries — pick whichever is simpler to validate incrementally and say why), each `{quat_xyzw_gripper[4], translation_m_gripper[3], image: <relative path>, held_out: bool, square_length_m, marker_length_m}`. This is deliberately minimal — it is glue between whatever GEOM-05's real capture session writes and the solver, not a new normative INTERFACES.md contract (docs/INTERFACES.md's own capture-record work, §8.4/GEOM-08, is about lifting crack paths post-calibration, a different consumer; do not conflate the two or touch docs/INTERFACES.md here).

- `POSE_CAPTURE_MIN_SOLVE_POSES = 15`, `POSE_CAPTURE_MIN_HELD_OUT_POSES = 4` (PLAN §2's real floors, distinct from `handeye.MIN_POSES`).
- `load_pose_captures(poses_dir) -> list[RawPoseCapture]`: parse every record, load its image (`PIL.Image` or `cv2.imread`), skip and report (never silently drop without counting) any record whose image is missing/unreadable.
- `assemble(poses_dir, board_spec: charuco.BoardSpec, intrinsics: geometry.Intrinsics) -> tuple[list[handeye.HandEyePoseSample], list[dict]]`: run `charuco.detect` on each, split solve/held_out by the `held_out` flag, return the usable `HandEyePoseSample` list plus a per-pose report dict (image, ok, num_corners, reprojection_rms_px, held_out) for the evidence file.
- `run_solve(poses_dir, *, methods=('tsai','park','daniilidis'), board_spec, intrinsics) -> EvidenceReport` (a frozen dataclass covering everything the criteria list requires) — enforces the two `POSE_CAPTURE_MIN_*` floors after the layer-1 drop, raising a typed error the CLI turns into exit 3.
- `write_evidence(report, path)` — plain JSON dump, `data/calibration/handeye_evidence_<UTCstamp>.json`.

CLI: `./env.sh python -m crackvision.calibration.handeye_capture --poses-dir DIR --square-length-m X --marker-length-m Y [--methods tsai,park,daniilidis] [common flags]`. Follow the §0.4 log-artefact convention exactly as `crackvision.paths`/`crackvision.capture_record` already do — read one of those two files first for the boilerplate (`argparse` setup via `crackvision.config.add_common_args`, logging via `crackvision.logging_setup`) rather than re-deriving it.

Tests build fixtures with `charuco.render_synthetic_view` at a known `T_gripper_camera` and known gripper poses (reuse the same pose-generation approach as `handeye.py`'s own tests, but now rendering real images through the real detector end-to-end).
