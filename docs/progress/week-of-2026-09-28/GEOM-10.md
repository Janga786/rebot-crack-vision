# GEOM-10 — End-of-arm model: tool_tip + wrist D405 (nominal CAD prior) in the planning model

**Accepted:** 2026-09-30 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** simulation · **Verified commit:** [`d1cf7dc`](https://github.com/Janga786/rebot-crack-vision/commit/d1cf7dc1ae08a66c1889026985190dcec006d5eb)

## What was done

Implemented interactively by the technical-lead recovery session (Claude Opus 5.5, operator-authorised, 2026-09-30); commit baed324. Operator facts: D405 eye-in-hand on Seeed stock D405_305_Mount (~15 deg), extrinsic unmeasured (CAD prior only). Please re-derive the tool_tip and camera prior claims (see card body) and run the checks.

_This step was a physical procedure performed by the operator; the evidence files below were recorded during it and then independently audited._

Files changed:
- [README.md](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/README.md) (+13 / −0)
- [config/robot/end_effector.yaml](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/config/robot/end_effector.yaml) (+94 / −0)
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/docs/INTERFACES.md) (+77 / −0)
- [docs/TECHNICAL_APPROACH.md](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/docs/TECHNICAL_APPROACH.md) (+6 / −0)
- [docs/adr/012-frames-and-conventions.md](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/docs/adr/012-frames-and-conventions.md) (+10 / −1)
- [docs/adr/014-end-effector-frames-and-task-phases.md](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/docs/adr/014-end-effector-frames-and-task-phases.md) (+145 / −0)
- [docs/motion/ROBOT_MODEL.md](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/docs/motion/ROBOT_MODEL.md) (+6 / −0)
- [docs/motion/ROS_WORKSPACE.md](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/docs/motion/ROS_WORKSPACE.md) (+25 / −0)
- [ros2_ws/src/crackvision_description/crackvision_description/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_description/crackvision_description/__init__.py) (+0 / −0)
- [ros2_ws/src/crackvision_description/crackvision_description/end_effector.py](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_description/crackvision_description/end_effector.py) (+206 / −0)
- [ros2_ws/src/crackvision_description/package.xml](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_description/package.xml) (+21 / −0)
- [ros2_ws/src/crackvision_description/resource/crackvision_description](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_description/resource/crackvision_description) (+0 / −0)
- [ros2_ws/src/crackvision_description/setup.cfg](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_description/setup.cfg) (+4 / −0)
- [ros2_ws/src/crackvision_description/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_description/setup.py) (+28 / −0)
- [ros2_ws/src/crackvision_description/srdf/b601_dm_end_effector.srdf.xacro](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_description/srdf/b601_dm_end_effector.srdf.xacro) (+23 / −0)
- [ros2_ws/src/crackvision_description/test/test_end_effector.py](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_description/test/test_end_effector.py) (+177 / −0)
- [ros2_ws/src/crackvision_description/urdf/b601_dm_end_effector.urdf.xacro](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_description/urdf/b601_dm_end_effector.urdf.xacro) (+104 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/check_end_effector.py](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_motion/crackvision_motion/check_end_effector.py) (+275 / −0)
- [ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py) (+39 / −2)
- [ros2_ws/src/crackvision_motion/package.xml](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_motion/package.xml) (+1 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [scripts/ros/test_end_effector.sh](https://github.com/Janga786/rebot-crack-vision/blob/d1cf7dc1ae08a66c1889026985190dcec006d5eb/scripts/ros/test_end_effector.sh) (+51 / −0)

Commits: [`baed324`](https://github.com/Janga786/rebot-crack-vision/commit/baed3241389968b7839b8438b4c9583eee53251f)

## Why it was done

config/robot/end_effector.yaml is the single, provenance-tagged source of the tool tip and the eye-in-hand D405 pose on gripper_link; the mock MoveIt stack plans with those frames and padded camera/mount collision proxies; ADR-014 records why.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals
- Requirement **REQ-GEOM-3**: TCP calibration and physical boresight verification

## How it moves the project forward

- Geometry & calibration: **2/11** tasks accepted; whole project: **24/77**.
- REQ-MOT-1: 1/13 contributing tasks done
- REQ-GEOM-2: 1/6 contributing tasks done
- REQ-GEOM-3: 2/5 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, end-effector ✔, mock-plan-regression ✔, sweep-regression ✔, vendor-untouched ✔); independent audit accepted it (criteria: 8 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “GEOM-10 (commit baed324, reviewed at d1cf7dc) does what it claims. I recomputed the numbers from the original sources: - **Tool tip:** in the canonical finger STLs placed by `gripper_joint1`/`gripper_joint2` at q=0, the farthest point along x is 0.0000 m (9.8e-8), at y≈0, with the tip edge spanning z ±1.33 mm. - **Camera prior:** bowenszhu `simulate_fov.py`@ca379b9a constants, mapped by Rx(pi), give xyz (-0.07828, -0.00900, -0.06398) and a rotation 0.0° from rpy (pi, -15°, 0). That matches the YAML to 0.02 mm, which is rounding. - **Mount box:** the bounds of the mount STL 
…[623 chars clipped]”

## What it unlocks next

- **Ready to start:** GEOM-11 — TCP/pivot calibration uses the tool_tip prior (ADR-014), not the grasp-centre gripper_tcp
- Closer: GEOM-04 — Hand-eye calibration software (synthetic-verified) (still needs GEOM-02, GEOM-03, GEOM-03)
- Closer: GEOM-05 — Execute hand-eye calibration on hardware (operator) (still needs GEOM-04, CAM-01, MOT-09, GEOM-04)
- Closer: GEOM-08 — Pixel paths → robot-frame 3D paths with uncertainty (still needs GEOM-02, GEOM-03, PERC-03, GEOM-03, PERC-03)
- Closer: MOT-03 — Planning scene from config (nominal until measured) (still needs MOT-02)
- Closer: MOT-04.5 — Run the full reachability sweep, recommend + independently verify the specimen placement, document (still needs MOT-04.3, MOT-04.4, MOT-04.6, MOT-03)
- Closer: MOT-04.6 — Reachability on task frames (tool_tip / camera_link) with a workcell-faithful specimen proxy + camera view check (still needs MOT-04.3, MOT-04.4)
- Closer: MOT-05 — Commissioning-gated execution interface (mock/dry/real) (still needs MOT-03, MOT-01, MOT-02)
- Closer: MOT-06 — Cartesian path planning along crack paths (still needs MOT-03, MOT-04, GEOM-08, MOT-02, GEOM-08)
