# GEOM-08.1 — Contracts: eye-in-hand capture record (§9) + robot-frame 3D path / tool-waypoint file with uncertainty and execution-eligibility policy (§10)

**Accepted:** 2026-09-30 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`d6445fd`](https://github.com/Janga786/rebot-crack-vision/commit/d6445fdf32a1ce40d05deee9c63ebb958033877a)

## What was done

Wrote the full normative spec for two new pipeline contracts into docs/INTERFACES.md: the eye-in-hand capture record format (§9) that pairs a colour/depth image with the robot's joint state and calibration file hash, and the robot-frame 3D crack-path/tool-waypoint file (§10) with its uncertainty model and rules for when a path is safe to actually run on the arm. All three required checks pass: the new sections are present and correctly numbered, the existing §0–§8 content is untouched (append-only, verified by diff), and the existing contract test suite still passes (5/5).

Files changed:
- [docs/COMPLETION_LOG.md](https://github.com/Janga786/rebot-crack-vision/blob/d6445fdf32a1ce40d05deee9c63ebb958033877a/docs/COMPLETION_LOG.md) (+62 / −0)
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/d6445fdf32a1ce40d05deee9c63ebb958033877a/docs/INTERFACES.md) (+292 / −0)

Commits: [`d6445fd`](https://github.com/Janga786/rebot-crack-vision/commit/d6445fdf32a1ce40d05deee9c63ebb958033877a)

## Why it was done

docs/INTERFACES.md gains two appended normative sections: §9 the `crackvision.capture_3d/1` capture record (implements §8.4) and §10 the `crackvision.paths3d/1` file, its lifting/gap policy, first-order uncertainty model, tool-waypoint policy and execution-eligibility rules. Every later GEOM-08.x card implements against these sections.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty
- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals

## How it moves the project forward

- Geometry & calibration: **4/24** tasks accepted; whole project: **32/92**.
- REQ-GEOM-1: 2/9 contributing tasks done
- REQ-GEOM-2: 2/18 contributing tasks done
- Verification: automated checks run by the pipeline itself (sections ✔, sections-0-8-unchanged ✔, contracts ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “GEOM-08.1 passes review. The change only appends to docs/INTERFACES.md: the §9 capture_3d/1 section and the §10 paths3d/1 section. It also adds one entry to COMPLETION_LOG.md. I found no removed lines in the diff, and the contract tests pass (5 passed). Every geometry API the new sections reference exists in src/crackvision/geometry.py: Intrinsics, deproject_pixels, depth_uncertainty_m, AnnulusSample, sample_surface_depth_annulus, PlaneFit, fit_plane and DEFAULT_VALID_DEPTH_RANGE_M = (0.07, 0.50). The documented fit_plane sign pitfall matches the code. The nominal_d405 quat
…[399 chars clipped]”

## What it unlocks next

- Closer: GEOM-08.2 — Pure-numpy B601-DM forward kinematics + end-effector transforms (crackvision.kinematics) (still needs GEOM-10)
- Closer: GEOM-08.4 — Capture record library + assemble/validate CLI (crackvision.capture_record, INTERFACES §9) (still needs GEOM-08.2)
- Closer: GEOM-08.5 — Lift ordered pixel polylines to base_link surface points with normals, gap policy and per-point covariance (crackvision.lift3d) (still needs GEOM-08.2, GEOM-08.4)
- Closer: GEOM-08.6 — tool_tip trace/approach/retract waypoints from lifted segments (crackvision.tool_waypoints) (still needs GEOM-08.5)
