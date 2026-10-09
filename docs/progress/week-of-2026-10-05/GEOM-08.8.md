# GEOM-08.8 — Synthetic ground-truth verification of pixel→base_link 3D paths and tool waypoints (+ evidence note)

**Accepted:** 2026-10-09 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`3da3fcc`](https://github.com/Janga786/rebot-crack-vision/commit/3da3fcc66103e7aff5dd69e48ad651fab37ad2dd)

## What was done

Added a synthetic ground-truth generator and test suite that validates the full pixel-to-robot-base 3D crack-path pipeline against a known, independently-computed answer. Across two camera tilt angles, recovered 3D points matched ground truth with median error under 0.2mm and worst-case error under 0.55mm (both well inside the 0.5mm/1.5mm targets), surface normals matched to within floating-point precision, and the estimated measurement-uncertainty bounds correctly covered 100% of points under injected sensor noise (target was 90%). While building the test harness, I found and fixed a rasterization bug that was making a simple crack line look like a branching tree to the path-extraction code; the fix is isolated to the new test tooling and did not touch any production code. All 358 tests in the project's suite still pass.

After the first audit, a repair round (GEOM-08.8.R1) fixed the reported issues: The synthetic-scene test generator for the crack-detection camera pipeline was only testing a camera looking straight down at the surface, even when asked for a "15-degree tilted view" — it was secretly tilting the surface instead, which looks identical to a straight-on camera and never tested the harder, angled case. I fixed the generator so a real 15-degree camera tilt is produced and verified the full measurement pipeline still hits its accuracy targets at that angle: position error stayed under 0.44mm (limit 1.5mm) and surface-normal error stayed under 0.13 degrees (limit 1 degree). All 358 project tests still pass.

Files changed:
- [docs/COMPLETION_LOG.md](https://github.com/Janga786/rebot-crack-vision/blob/3da3fcc66103e7aff5dd69e48ad651fab37ad2dd/docs/COMPLETION_LOG.md) (+116 / −0)
- [docs/geometry/PATH3D_VERIFICATION.md](https://github.com/Janga786/rebot-crack-vision/blob/3da3fcc66103e7aff5dd69e48ad651fab37ad2dd/docs/geometry/PATH3D_VERIFICATION.md) (+256 / −34)
- [tests/test_path3d_synthetic.py](https://github.com/Janga786/rebot-crack-vision/blob/3da3fcc66103e7aff5dd69e48ad651fab37ad2dd/tests/test_path3d_synthetic.py) (+354 / −10)
- [tools/synth_scene3d.py](https://github.com/Janga786/rebot-crack-vision/blob/3da3fcc66103e7aff5dd69e48ad651fab37ad2dd/tools/synth_scene3d.py) (+751 / −12)

Commits: [`73b1a71`](https://github.com/Janga786/rebot-crack-vision/commit/73b1a71f68a3bb3c65355cdc3b5924a227206d7f), [`3da3fcc`](https://github.com/Janga786/rebot-crack-vision/commit/3da3fcc66103e7aff5dd69e48ad651fab37ad2dd)

## Why it was done

A deterministic synthetic scene generator renders a known specimen surface under the nominal eye-in-hand chain, and an end-to-end test drives skeleton → crackvision.paths → crackvision.path3d. The test shows recovered 3D paths, normals and tool poses match ground truth within stated tolerances, with reported uncertainty consistent with injected noise. The measured numbers are recorded in docs/geometry/PATH3D_VERIFICATION.md.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty
- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals
- Requirement **REQ-INT-1**: Simulation and recorded-data integration tests of the full pipeline

## How it moves the project forward

- Geometry & calibration: **13/24** tasks accepted; whole project: **48/92**.
- REQ-GEOM-1: 6/9 contributing tasks done
- REQ-GEOM-2: 9/18 contributing tasks done
- REQ-INT-1: 1/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (synthetic-e2e ✔, suite ✔, evidence-doc ✔); independent audit accepted it (criteria: 2 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair works. camera_target_pose now puts the camera on a cone of half-angle view_tilt around the surface normal and points it back at the specimen centre. I reproduced the 15° scene on my own: the plane normal expressed in the optical frame is [0.129, 0.224, −0.966], so the boresight is 15.0000° off the anti-normal, and annulus depths vary from 0.2419 to 0.258 m, so the view really is oblique. All noise-free tolerances pass at 15°. Median error is 0.161 mm, max error 0.438 mm, normal error 0.130°, max +X·n_true −0.9999975 and clearance error 0.257 mm. These match the d
…[354 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task GEOM-08.8.R1): 15° view tilt (optical axis vs anti-normal) is never exercised; both scenes are fronto-parallel in the camera frame.

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.
