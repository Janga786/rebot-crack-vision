# MOT-04.3 — Specimen placement scorer + recommend_placement CLI (+ verification-grid emitter)

**Accepted:** 2026-09-29 · **Work package:** Motion planning (L-MOTION) · **Track:** software · **Verified commit:** [`57bdca7`](https://github.com/Janga786/rebot-crack-vision/commit/57bdca7d5d6705fc518964cc802d0e291a4d7c24)

## What was done

Finished the specimen-placement scorer and its recommend_placement CLI for the robot motion-planning reachability pipeline: given a completed reachability sweep, it now picks a deterministic best specimen placement (or honestly reports infeasibility with the largest square that would fit) and writes a companion verification grid for independent re-checking. All 17 unit tests pass, including hand-verified known-optimum and tie-break scenarios, and the CLI runs successfully end-to-end against the real production sweep configuration, correctly finding and validating a feasible placement.

After the first audit, a repair round (MOT-04.3.R1) fixed the reported issues: Repaired the specimen-placement CLI test suite after an upstream change increased the robot-arm reachability sweep's grid spacing from 25mm to 30mm, which had broken the placement recommender's consistency check between its test data and the real configuration. Rebuilt the synthetic test map by proportionally rescaling its geometry to the new grid spacing, re-verified all 17 unit tests plus the end-to-end command-line workflow (which now correctly recommends placing the test specimen at position (0.33m, 0.0m)) and the mock motion-planning regression, all passing.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/57bdca7d5d6705fc518964cc802d0e291a4d7c24/docs/INTERFACES.md) (+28 / −8)
- [ros2_ws/src/crackvision_motion/crackvision_motion/placement.py](https://github.com/Janga786/rebot-crack-vision/blob/57bdca7d5d6705fc518964cc802d0e291a4d7c24/ros2_ws/src/crackvision_motion/crackvision_motion/placement.py) (+289 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py](https://github.com/Janga786/rebot-crack-vision/blob/57bdca7d5d6705fc518964cc802d0e291a4d7c24/ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py) (+147 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/57bdca7d5d6705fc518964cc802d0e291a4d7c24/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json](https://github.com/Janga786/rebot-crack-vision/blob/57bdca7d5d6705fc518964cc802d0e291a4d7c24/ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json) (+2 / −1)
- [ros2_ws/src/crackvision_motion/test/test_placement.py](https://github.com/Janga786/rebot-crack-vision/blob/57bdca7d5d6705fc518964cc802d0e291a4d7c24/ros2_ws/src/crackvision_motion/test/test_placement.py) (+541 / −26)

Commits: [`364b2f9`](https://github.com/Janga786/rebot-crack-vision/commit/364b2f9402b2998141da94be81506fda367b79da), [`57bdca7`](https://github.com/Janga786/rebot-crack-vision/commit/57bdca7d5d6705fc518964cc802d0e291a4d7c24)

## Why it was done

`ros2 run crackvision_motion recommend_placement` turns a complete reachability map into a deterministic, nominal specimen placement (or an honest 'infeasible' with the largest feasible square). It also emits a half-step verification grid config for independent re-checking and validates placement files.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **5/15** tasks accepted; whole project: **29/74**.
- REQ-MOT-2: 4/9 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, cli-smoke ✔, mock-plan-regression ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair correctly fixes the cli-smoke regression by regenerating the synthetic map fixture and test config at the new 0.03 m grid step (matching upstream MOT-04.5's config change), via a uniform 1.2x scale of the hand-computable annulus/patch geometry, plus bumping tolerance_m defaults to satisfy load_config's tolerance_m >= step/2 constraint. All four acceptance checks pass on re-run, the diff is scoped to the two allowed test files, and production-config-relative values (footprint_m/tolerance_m used against the real reachability.yaml) were correctly left unscaled since
…[41 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-04.3.R1): check cli-smoke fails after upstream change.

## What it unlocks next

- **Ready to start:** MOT-04.5 — Run the full reachability sweep, recommend + independently verify the specimen placement, document
