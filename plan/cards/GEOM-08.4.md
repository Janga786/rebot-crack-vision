# GEOM-08.4 — Capture record library + assemble/validate CLI (crackvision.capture_record, INTERFACES §9)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-08.1",
    "GEOM-08.2"
  ],
  "requirements": [
    "REQ-GEOM-2",
    "REQ-CAM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/capture_record.py",
      "tests/test_capture_record.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#3.10",
    "docs/INTERFACES.md#8.4",
    "docs/INTERFACES.md#9",
    "src/crackvision/kinematics.py",
    "src/crackvision/geometry.py",
    "src/crackvision/config.py",
    "src/crackvision/logging_setup.py",
    "src/crackvision/paths.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_capture_record.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "cli-help",
        "cmd": "./env.sh python -m crackvision.capture_record --help",
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
      "Every §9 refusal rule has a test that triggers it and checks the reason text.",
      "A record without a robot block raises a CaptureRecordError whose message says it is valid for 2D only (§8.4).",
      "The chain composition is FK(q)·T_gripper_link_camera_link·T_camera_link_optical, verified against a hand-composed product.",
      "The CLI follows §0 exactly (common flags, dry-run writes nothing, exit codes 0/1/2/3, logs/capture_record_*.json summaries), mirroring crackvision.paths.",
      "depth_scale is copied from the metadata and never defaulted. A synthetic metadata input (`synthetic: true`) yields a record with `synthetic: true`."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-08.4",
  "parent": "GEOM-08",
  "title": "Capture record library + assemble/validate CLI (crackvision.capture_record, INTERFACES §9)",
  "outcome": "`crackvision.capture_record` loads and validates §9 records, with explicit refusal reasons, and builds T_base_link_camera_color_optical_frame from a record. Its CLI assembles a record from a §3.10 D405 metadata JSON plus a joint-state JSON, and validates existing records. Any capture without joint state or the end_effector sha256 is refused for 3D."
}
```

Library:
- `SCHEMA = 'crackvision.capture_3d/1'`; `class CaptureRecordError(Exception)`; `capture_record_path(root, case) -> root/data/captures/{case}_capture.json`.
- `@dataclass CaptureRecord` covers the §9 fields: intrinsics as a `geometry.Intrinsics` (build it from the flat §9 dict, not from_stream_dict's nested form), depth_file, depth_scale, q (np.array(6)), stamps, end_effector sha and statuses, T_camera_link_optical (4×4), optical_source, synthetic, image_hw, kinematic_model path and sha.
- `load_capture_record(path, *, root, end_effector: kinematics.EndEffector, max_skew_s=0.1, accept_end_effector_change=False) -> CaptureRecord`.
  - Applies every §9 refusal rule, including a limit check via `kinematics.load_chain().fk(q)` and a check that the URDF sha matches `kinematic_model.sha256`.
  - Sets `end_effector_changed: bool` when overridden.
- `load_depth(record, root) -> uint16 array`: shape must equal image_hw.
- `T_base_link_optical(record, chain, end_effector) -> 4×4` = FK(q)·T_gl_cl·T_cl_opt.
- `build_record(...)` → dict and `write_record(dict, path)`: pure assembly.

CLI `./env.sh python -m crackvision.capture_record {assemble,validate} [common flags]`:
- `assemble --case C --d405-metadata META.json --joint-state JS.json [--optical-source nominal_d405|driver_tf --optical-tf TF.json]`.
  - JS.json = {joint_names, positions_rad, stamp_ns, stamp_source}.
  - capture_stamp_ns comes from the metadata's `timestamp_utc`.
  - `driver_tf` requires TF.json {xyz_m, quat_xyzw}, read from the realsense2_camera static TF (ADR-012: never hand-built from extrinsics_depth_to_color). The default is `nominal_d405` with a WARNING.
  - Writes data/captures/{C}_capture.json with the current end_effector.yaml sha256 and statuses.
- `validate --cases A B …`: loads each record, prints OK or the refusal reason. Exit 3 if any is refused.
- v1 assembles from §3.10 metadata only. `source.kind: recording` is reserved for CAM-05/INT-02 writers, and the loader accepts it.

Tests use tmp roots with synthetic metadata/depth fixtures written in the test. Cover:
- each refusal (missing robot, bad joint names, out-of-limit q, missing sha, sha mismatch, and the mismatch with override, skew, shape mismatch);
- the transform composition;
- the CLI: assemble → validate round trip; --dry-run writes nothing; bad args → 2; missing record → 3.
