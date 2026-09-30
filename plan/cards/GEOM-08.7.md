# GEOM-08.7 — crackvision.path3d CLI: paths.json + mask + capture record → data/paths3d/{case}_paths3d.json with eligibility

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-08.4",
    "GEOM-08.5",
    "GEOM-08.6"
  ],
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/path3d.py",
      "tests/test_path3d_cli.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#1.2",
    "docs/INTERFACES.md#2",
    "docs/INTERFACES.md#3.13",
    "docs/INTERFACES.md#9",
    "docs/INTERFACES.md#10",
    "src/crackvision/paths.py",
    "src/crackvision/naming.py",
    "src/crackvision/config.py",
    "src/crackvision/logging_setup.py",
    "src/crackvision/capture_record.py",
    "src/crackvision/lift3d.py",
    "src/crackvision/tool_waypoints.py",
    "src/crackvision/kinematics.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_path3d_cli.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "cli-help",
        "cmd": "./env.sh python -m crackvision.path3d --help",
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
      "The output validates field-by-field against §10. A test asserts the full key set at every level and that component and polyline order equals paths.json.",
      "Refusals are covered by tests: a missing capture record, a record without a robot block, a downscaled case, and an image-shape mismatch between paths, mask, depth and record each produce exit 3 (single case) or partial/1 (mixed), with a per-case reason in the logs JSON.",
      "With today's nominal end_effector.yaml, execution_eligible is false and the reasons include wrist_camera_nominal and tool_nominal. With a synthetic record it also includes synthetic_capture.",
      "--accept-end-effector-change is the only way past a sha mismatch, and it adds end_effector_changed_since_capture.",
      "§0 flags work (--dry-run writes nothing, --skip-existing, --root/--config), logs are written, and the CLI mirrors crackvision.paths' structure."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "impl-cli",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-08.7",
  "parent": "GEOM-08",
  "title": "crackvision.path3d CLI: paths.json + mask + capture record → data/paths3d/{case}_paths3d.json with eligibility",
  "outcome": "`./env.sh python -m crackvision.path3d` lifts every main path and branch of each case into the exact §10 JSON (points, segments, waypoints, calibration, uncertainty_model, execution_eligible + reasons, source sha256s). It follows INTERFACES §0 and refuses cases without a valid §9 capture record."
}
```

Create src/crackvision/path3d.py, mirroring crackvision.paths: build_parser / add_common_args / main / summary logging, and exit codes 0/1/2/3.

Usage: `./env.sh python -m crackvision.path3d [--cases A B …] [--trace-clearance-m 0.01] [--approach-clearance-m 0.04] [--waypoint-spacing-m 0.002] [--annulus-inner-px 3] [--annulus-outer-px 8] [--min-fraction-valid 0.3] [--max-gap-px 5] [--pixel-sigma-px 1.0] [--max-skew-s 0.1] [--accept-end-effector-change] [common flags]`.

Per case, reading data/case_map.json:
- If the case is downscaled → refuse with reason `downscaled_case`.
- Load data/paths/{case}_paths.json, the mask via `naming.mask_path` (>0 ⇒ crack), and data/captures/{case}_capture.json via `capture_record.load_capture_record`.
- Load depth. Check that the (h, w) of paths, mask, depth and record are equal (§0.5).
- T = `capture_record.T_base_link_optical`.
- For each component, and each polyline (main, then branches in paths.json order), run `lift3d.lift_polyline` on its DENSE points, then `tool_waypoints.segment_waypoints` per segment. Take capture_tool_z from q.
- Assemble exactly the §10 JSON (NaN → null) and compute the eligibility reasons per §10.
- Write data/paths3d/{case}_paths3d.json atomically (tmp + rename).

Status: all cases ok → 0. Some cases fail → status partial, 1. Every requested case refused for a precondition (missing or invalid record, shape mismatch) → 3. Bad args → 2.

Tests use a tmp root fixture built in the test: a fronto-parallel plane at 0.25 m in optical, a straight skeleton line with a 3-px depth hole and one 10-px hole, a mask, and a record assembled with `capture_record.build_record` at a valid q. Cover:
- the full key set;
- positions within 1 mm of analytic;
- the segment split;
- eligibility reasons;
- each refusal;
- --dry-run, --skip-existing and the logs JSON.

No robot motion and no ROS imports. Do not wire this into run_test.sh (OPS-01's job).
