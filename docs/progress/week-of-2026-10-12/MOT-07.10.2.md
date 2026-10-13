# MOT-07.10.2 — Accept view_move in joint_trajectory and execution_gate (G-TRAJ, G-ELIGIBLE) with gate regression

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`e51fdc3`](https://github.com/Janga786/rebot-crack-vision/commit/e51fdc36bfc68608bad60dd0472defb05c3a5837)

## What was done

Joint trajectories can now declare purpose \"view_move\" (binding a view plan instead of a paths3d file), and the execution gate's offline checks validate that binding end-to-end: it confirms the view-plan file matches its declared hash, loads and parses correctly, has a non-null chosen view with a transit path, and that the trajectory's start/end match that path to within 1e-9 rad. A separate check then verifies the view plan's recorded config hashes agree with the trajectory's, warning outside real mode and refusing in real mode on any mismatch. All 194 motion unit tests pass, the full existing gate-regression suite (scripts/ros/test_execution.sh) still passes unchanged, and real-mode execution is still refused without the arm-armed flag.

After the first audit, a repair round (MOT-07.10.2.R2) fixed the reported issues: The view-move execution gate's robustness fix for bad/missing limits files and non-UTF-8 view-plan files was already in place from earlier work, and I re-verified it still works correctly. All required tests (203 total) pass, including the specific parametrized cases for corrupted limits YAML, missing limits files, and non-UTF-8 view-plan files across mock, dry-run, and real execution modes, and the full integration test script and real-mode safety refusal check both pass with exit code 0.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py](https://github.com/Janga786/rebot-crack-vision/blob/e51fdc36bfc68608bad60dd0472defb05c3a5837/ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py) (+64 / −2)
- [ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/e51fdc36bfc68608bad60dd0472defb05c3a5837/ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py) (+39 / −21)
- [ros2_ws/src/crackvision_motion/test/test_execution_gate.py](https://github.com/Janga786/rebot-crack-vision/blob/e51fdc36bfc68608bad60dd0472defb05c3a5837/ros2_ws/src/crackvision_motion/test/test_execution_gate.py) (+238 / −5)
- [ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/e51fdc36bfc68608bad60dd0472defb05c3a5837/ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py) (+77 / −0)

Commits: [`cbdd286`](https://github.com/Janga786/rebot-crack-vision/commit/cbdd28686c745254683a8eff82700d13a03151b2), [`24ac771`](https://github.com/Janga786/rebot-crack-vision/commit/24ac771462c1952744ced9890417a38a1370e4bf)

## Why it was done

joint_trajectory.load_trajectory accepts §17.1 view_move files and rejects malformed source shapes. execution_gate implements §17.2's G-TRAJ/G-ELIGIBLE view_move rules. The MOT-05 gate regression suite still passes unchanged.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **45/57** tasks accepted; whole project: **107/144**.
- REQ-MOT-2: 31/38 contributing tasks done
- REQ-OPS-1: 3/11 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, gate-regression ✔, real-refusal ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “This card adds no commits. The fix for finding bb9fa426d003590e is already on main at e51fdc3. In execution_gate.py:737-745, `_evaluate_eligible_view_move` wraps `load_view_plan`. It catches CartesianPlanError, then UnicodeDecodeError (reported as fail), then OSError, KeyError, TypeError, ValueError and yaml.YAMLError (reported as error). An independent probe ran each case in mock, dry and real. A non-UTF-8 view plan, a missing limits file, malformed limits YAML, a non-UTF-8 limits file, list-shaped limits, empty limits and a directory as limits_path all return a GateReport
…[357 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-07.10.2.R2): View_move G-ELIGIBLE lets UnicodeDecodeError, OSError and YAML errors escape evaluate_offline; G-ELIGIBLE view_move branch crashes the whole offline gate on non-CartesianPlanError load failures.

## What it unlocks next

- **Ready to start:** MOT-07.10.3 — retime_plan --view-plan: retime the view_plan transit leg into a view_move trajectory
- Closer: MOT-07.10.5 — Headless mock run: view_plan → retime_plan --view-plan → trajectory_check → execute_trajectory dry/mock (still needs MOT-07.10.3, MOT-07.10.4)
