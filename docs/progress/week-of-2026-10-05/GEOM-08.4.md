# GEOM-08.4 — Capture record library + assemble/validate CLI (crackvision.capture_record, INTERFACES §9)

**Accepted:** 2026-10-09 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`55c8ea6`](https://github.com/Janga786/rebot-crack-vision/commit/55c8ea6c8acb0df784e33fb3a578af29ef2447f2)

## What was done

Implemented crackvision.capture_record: §9 CaptureRecord dataclass, load_capture_record with all listed §9.4 refusal rules (missing robot block, bad joint names, out-of-limit q via FK, missing/mismatched end_effector sha with override, capture/robot stamp skew, case_map downscaled check), load_depth with shape validation, T_base_link_optical chain composition, build_record/write_record pure assembly, and an assemble/validate CLI following §0 conventions (common flags, dry-run, logs/capture_record_*.json summaries, exit codes 0/2/3).

After the first audit, a repair round (GEOM-08.4.R1) fixed the reported issues: Fixed a gap in the eye-in-hand capture record validator so it now actually refuses a capture whose declared image size disagrees with the depth image, colour image, crack mask, or path-extraction file on disk — previously this corruption case would have silently passed validation. Added 9 new automated tests covering each of these mismatch cases plus the kinematic-model-hash and downscaled-case refusal rules that had no test coverage before; the full test suite (309 tests) still passes.

Files changed:
- [src/crackvision/capture_record.py](https://github.com/Janga786/rebot-crack-vision/blob/55c8ea6c8acb0df784e33fb3a578af29ef2447f2/src/crackvision/capture_record.py) (+601 / −1)
- [tests/test_capture_record.py](https://github.com/Janga786/rebot-crack-vision/blob/55c8ea6c8acb0df784e33fb3a578af29ef2447f2/tests/test_capture_record.py) (+463 / −0)

Commits: [`e1d0892`](https://github.com/Janga786/rebot-crack-vision/commit/e1d0892bc8b9f8bf3e020373d189fc8d95a75ae5), [`55c8ea6`](https://github.com/Janga786/rebot-crack-vision/commit/55c8ea6c8acb0df784e33fb3a578af29ef2447f2)

## Why it was done

`crackvision.capture_record` loads and validates §9 records, with explicit refusal reasons, and builds T_base_link_camera_color_optical_frame from a record. Its CLI assembles a record from a §3.10 D405 metadata JSON plus a joint-state JSON, and validates existing records. Any capture without joint state or the end_effector sha256 is refused for 3D.

- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals
- Requirement **REQ-CAM-2**: Aligned colour/depth capture with timestamps, intrinsics, depth scale, extrinsics and metadata

## How it moves the project forward

- Geometry & calibration: **9/24** tasks accepted; whole project: **42/92**.
- REQ-GEOM-2: 6/18 contributing tasks done
- REQ-CAM-2: 3/8 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, cli-help ✔, suite ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Both review findings from GEOM-08.4 are resolved: §9.4 image-dimension consistency is now enforced in load_capture_record against the aligned depth PNG, the d405 colour frame (when source.kind == d405_metadata), the mask, and paths.json, each with a distinguishable reason text; and the previously-untested downscaled/kinematic_model.sha256 refusal rules now have dedicated tests. All three scheduler acceptance checks and the full 309-test suite pass, independently re-verified.”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task GEOM-08.4.R1): §9.4 image-dimension consistency refusal rule is never enforced; case_map.json downscaled and kinematic_model.sha256-mismatch refusal rules ship with zero tests.

## What it unlocks next

- **Ready to start:** GEOM-08.5 — Lift ordered pixel polylines to base_link surface points with normals, gap policy and per-point covariance (crackvision.lift3d)
- Closer: GEOM-08.7 — crackvision.path3d CLI: paths.json + mask + capture record → data/paths3d/{case}_paths3d.json with eligibility (still needs GEOM-08.5, GEOM-08.6)
