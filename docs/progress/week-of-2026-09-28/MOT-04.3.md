# MOT-04.3 — Specimen placement scorer + recommend_placement CLI (+ verification-grid emitter)

**Accepted:** 2026-09-29 · **Work package:** Motion planning (L-MOTION) · **Track:** software · **Verified commit:** [`364b2f9`](https://github.com/Janga786/rebot-crack-vision/commit/364b2f9402b2998141da94be81506fda367b79da)

## What was done

Finished the specimen-placement scorer and its recommend_placement CLI for the robot motion-planning reachability pipeline: given a completed reachability sweep, it now picks a deterministic best specimen placement (or honestly reports infeasibility with the largest square that would fit) and writes a companion verification grid for independent re-checking. All 17 unit tests pass, including hand-verified known-optimum and tie-break scenarios, and the CLI runs successfully end-to-end against the real production sweep configuration, correctly finding and validating a feasible placement.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/364b2f9402b2998141da94be81506fda367b79da/docs/INTERFACES.md) (+28 / −8)
- [ros2_ws/src/crackvision_motion/crackvision_motion/placement.py](https://github.com/Janga786/rebot-crack-vision/blob/364b2f9402b2998141da94be81506fda367b79da/ros2_ws/src/crackvision_motion/crackvision_motion/placement.py) (+289 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py](https://github.com/Janga786/rebot-crack-vision/blob/364b2f9402b2998141da94be81506fda367b79da/ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py) (+147 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/364b2f9402b2998141da94be81506fda367b79da/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json](https://github.com/Janga786/rebot-crack-vision/blob/364b2f9402b2998141da94be81506fda367b79da/ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/test_placement.py](https://github.com/Janga786/rebot-crack-vision/blob/364b2f9402b2998141da94be81506fda367b79da/ros2_ws/src/crackvision_motion/test/test_placement.py) (+499 / −0)

Commits: [`364b2f9`](https://github.com/Janga786/rebot-crack-vision/commit/364b2f9402b2998141da94be81506fda367b79da)

## Why it was done

`ros2 run crackvision_motion recommend_placement` turns a complete reachability map into a deterministic, nominal specimen placement (or an honest 'infeasible' with the largest feasible square). It also emits a half-step verification grid config for independent re-checking and validates placement files.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **2/15** tasks accepted; whole project: **23/74**.
- REQ-MOT-2: 1/9 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, cli-smoke ✔, mock-plan-regression ✔); independent audit accepted it (criteria: 7 passed, 1 failed, 0 not verifiable without hardware).
- Audit summary: “Independently re-ran all four scheduler acceptance checks (build, unit tests, CLI smoke, mock-plan regression) and all pass. Reviewed placement.py's scoring/ranking/verification-grid logic line-by-line against the normative INTERFACES §7.3 rule and confirmed exact correspondence (footprint dilation, out-of-grid corner test, 1e-9 inclusive sampling, tie-break tuple, alternatives dedup, max_feasible_square search). Exit-code wiring in cli_common.py correctly maps ConfigError/MapError/PlacementError→2, PreconditionError→3, generic failures→1, matching §7.4. The INTERFACES.md d
…[427 chars clipped]”

## What it unlocks next

- Closer: MOT-04.5 — Run the full reachability sweep, recommend + independently verify the specimen placement, document (still needs MOT-04.4)
