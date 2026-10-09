# GEOM-08.3 — FK parity: crackvision.kinematics vs live MoveIt /compute_fk (gripper_link, tool_tip, camera_link) on the mock stack

**Accepted:** 2026-10-09 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** simulation · **Verified commit:** [`b71fe27`](https://github.com/Janga786/rebot-crack-vision/commit/b71fe2753c03d82c13c67f3b778b0dfb093ba7a5)

## What was done

Built an automated check that compares the project's own fast forward-kinematics math against MoveIt's official kinematics for the robot's gripper, tool tip, and wrist camera. Across 21 different arm poses (one at rest plus 20 random ones within the robot's joint limits), the two methods agreed to within about 4e-16 meters in position and 5.6e-8 radians in orientation — both far tighter than the 1e-6 threshold required. This gives independent confidence that 3D crack paths computed offline will line up correctly with where the real robot's motion planner thinks those points are.

Files changed:
- [scripts/geometry/compare_fk_parity.py](https://github.com/Janga786/rebot-crack-vision/blob/b71fe2753c03d82c13c67f3b778b0dfb093ba7a5/scripts/geometry/compare_fk_parity.py) (+116 / −0)
- [scripts/ros/fk_parity_dump.py](https://github.com/Janga786/rebot-crack-vision/blob/b71fe2753c03d82c13c67f3b778b0dfb093ba7a5/scripts/ros/fk_parity_dump.py) (+146 / −0)
- [scripts/ros/test_fk_parity.sh](https://github.com/Janga786/rebot-crack-vision/blob/b71fe2753c03d82c13c67f3b778b0dfb093ba7a5/scripts/ros/test_fk_parity.sh) (+60 / −0)

Commits: [`b71fe27`](https://github.com/Janga786/rebot-crack-vision/commit/b71fe2753c03d82c13c67f3b778b0dfb093ba7a5)

## Why it was done

A headless script shows the pure-numpy FK chain (plus the end_effector.yaml transforms) agrees with MoveIt's own FK of the planning model on 21 joint states, to ≤ 1e-6 m and ≤ 1e-6 rad, for gripper_link, tool_tip and camera_link. This is independent evidence that 3D paths are placed in the same base_link MoveIt plans in.

- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals

## How it moves the project forward

- Geometry & calibration: **13/24** tasks accepted; whole project: **48/92**.
- REQ-GEOM-2: 9/18 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, fk-parity ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “GEOM-08.3 implements exactly the scoped files, reproduces FK parity to ~4e-16 m / ~5.6e-8 rad across 21 joint states (well inside 1e-6/1e-6), and the dump/compare/test scripts behave correctly under direct re-verification (malformed dump → exit 2, tampered link → exit 1 with correct per-link table, read-only /compute_fk-only RPC, no stray processes after teardown).”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.
