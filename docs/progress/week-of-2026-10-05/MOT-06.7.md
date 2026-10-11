# MOT-06.7 — Headless mock-stack smoke test of plan_crack_path with consistent scene + paths3d fixtures

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`ddad51f`](https://github.com/Janga786/rebot-crack-vision/commit/ddad51f2b36ec5cb1404df0637cc2092bdd9af07)

## What was done

A headless smoke test (scripts/ros/test_cartesian_plan.sh) now plans a test-only crack-path fixture with four segments against a matching fixture scene on the MoveIt mock stack. It checks the result with its own FK and collision-validity calls. Both reachable segments, a 0.06 m straight line (31 waypoints) and a 120° arc (33 waypoints), are planned in full: the worst tool-tip deviation from the 10 mm standoff path is 0.195 mm, and the boresight error is under 0.1°. The out-of-reach segment is reported failed, and the segment sunk 5 mm into the specimen is reported as a surface/scene mismatch. Coverage is 71.07% by length, as expected, and requiring full coverage makes the planner exit with code 3.

Files changed:
- [ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/capture_stub.json](https://github.com/Janga786/rebot-crack-vision/blob/ddad51f2b36ec5cb1404df0637cc2092bdd9af07/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/capture_stub.json) (+26 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/expected.json](https://github.com/Janga786/rebot-crack-vision/blob/ddad51f2b36ec5cb1404df0637cc2092bdd9af07/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/expected.json) (+78 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/make_fixture.py](https://github.com/Janga786/rebot-crack-vision/blob/ddad51f2b36ec5cb1404df0637cc2092bdd9af07/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/make_fixture.py) (+311 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/paths3d.json](https://github.com/Janga786/rebot-crack-vision/blob/ddad51f2b36ec5cb1404df0637cc2092bdd9af07/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/paths3d.json) (+7029 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/scene.yaml](https://github.com/Janga786/rebot-crack-vision/blob/ddad51f2b36ec5cb1404df0637cc2092bdd9af07/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/scene.yaml) (+35 / −0)
- [scripts/ros/test_cartesian_plan.sh](https://github.com/Janga786/rebot-crack-vision/blob/ddad51f2b36ec5cb1404df0637cc2092bdd9af07/scripts/ros/test_cartesian_plan.sh) (+377 / −0)

Commits: [`ddad51f`](https://github.com/Janga786/rebot-crack-vision/commit/ddad51f2b36ec5cb1404df0637cc2092bdd9af07)

## Why it was done

scripts/ros/test_cartesian_plan.sh plans a committed fixture paths3d (reachable straight and curved segments at the recommended placement, a branch, and a deliberately unreachable segment) against a consistent fixture scene on the mock stack and asserts full coverage of the reachable segments, tool_tip clearance and boresight via FK, collision validity, limits, continuity, and honest failure reporting.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-INT-1**: Simulation and recorded-data integration tests of the full pipeline

## How it moves the project forward

- Motion planning: **28/41** tasks accepted; whole project: **82/125**.
- REQ-MOT-1: 24/34 contributing tasks done
- REQ-MOT-2: 15/22 contributing tasks done
- REQ-INT-1: 2/6 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, smoke ✔, regression ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-06.7 adds a headless smoke test for plan_crack_path's four-segment fixture scenario. All claims were independently verified: the committed fixtures regenerate byte-identical from make_fixture.py, the full test_cartesian_plan.sh smoke script was re-run end-to-end (~11.7 min) and reproduced the scheduler's exact results (two full segments with sub-millimeter trace accuracy, one ik_failed, one surface_scene_mismatch, correct coverage fraction, and a clean exit 3 under --require-full-coverage with no stray plan file or processes), and the regression suite passed (496/496). 
…[487 chars clipped]”

## What it unlocks next

- Closer: MOT-06.10 — Synthetic end-to-end planning run (GEOM-08 synthetic case → paths3d → view + crack plan) and CARTESIAN_PLANNING.md (still needs MOT-06.9)
