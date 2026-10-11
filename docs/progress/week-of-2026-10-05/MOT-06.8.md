# MOT-06.8 — View-phase core: camera_link view-pose candidates and D405 FOV coverage check (pure Python)

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`3b512c4`](https://github.com/Janga786/rebot-crack-vision/commit/3b512c44c1002d4a746651ef950ed48d3492b294)

## What was done

Added the pure-Python geometry core for the robot's camera "view phase": given the specimen's position (from the scene file or a manual center), it generates candidate camera poses at 0° and 15° tilt that always look exactly at the specimen's center, and checks whether the camera's real field of view would actually see every corner of the specimen from each pose. All 13 new unit tests pass, along with the full 481-test suite for this package, confirming the tilt angles, pass-through-center geometry, and FOV pass/fail logic are numerically correct.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/view_core.py](https://github.com/Janga786/rebot-crack-vision/blob/3b512c44c1002d4a746651ef950ed48d3492b294/ros2_ws/src/crackvision_motion/crackvision_motion/view_core.py) (+209 / −0)
- [ros2_ws/src/crackvision_motion/test/test_view_core.py](https://github.com/Janga786/rebot-crack-vision/blob/3b512c44c1002d4a746651ef950ed48d3492b294/ros2_ws/src/crackvision_motion/test/test_view_core.py) (+253 / −0)

Commits: [`3b512c4`](https://github.com/Janga786/rebot-crack-vision/commit/3b512c44c1002d4a746651ef950ed48d3492b294)

## Why it was done

crackvision_motion.view_core generates the §13 camera_link view-pose candidates (0.25 m standoff, tilt 0/15°, yaw set) over the specimen centre from the scene or an explicit centre, and checks that every specimen footprint corner lies inside the nominal D405 FOV with margin and within the view band — ROS-free and unit-tested.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **26/40** tasks accepted; whole project: **80/124**.
- REQ-MOT-2: 14/21 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, regression ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “view_core.py implements specimen_frame/view_candidates/fov_check exactly as the card body and acceptance criteria specify (sphere-rotation geometry reused from reachability_core/placement.py's --emit-view-config path, so the optical axis always passes through the specimen centre at both 0° and 15° tilt). Independently reran both acceptance commands (13/13 and 481/481 passed), confirmed scope is limited to the two allowed files, confirmed no ROS imports, and checked the numerical claims (tilt-0 axis = -n with standoff along +n, tilt-15 angle exactly 15° and still through cen
…[120 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-06.9 — plan_view node/CLI + headless smoke: reachable, collision-free camera view pose with the specimen in FOV
