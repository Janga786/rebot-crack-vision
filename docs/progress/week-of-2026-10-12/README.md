# Progress — week of Mon Oct 12 – Sun Oct 18, 2026

**18 task(s) completed and independently audited this week** · project total **95/156** accepted

## Completed this week

- **MOT-07.1 — Decision + contract: time parameterization and independent trajectory checker (ADR-019, INTERFACES §16)** ([details](MOT-07.11.md)): Decided and documented how MOT-07 will turn an untimed Cartesian crack-path plan into a timed, executable robot trajectory, and how a second, independent program will check that result. I verified han
…[684 chars clipped]
- **MOT-07.2 — Pure-numpy tool_tip FK in crackvision_motion (tool_fk) with parity against crackvision.kinematics** ([details](MOT-07.2.md)): Added a pure-numpy, ROS-free forward-kinematics module (tool_fk) that computes the robot's tool-tip position/orientation from joint angles, matching the existing reference implementation to better tha
…[249 chars clipped]
- **MOT-07.3 — Retime core: retime config loader, plan polyline assembly, refinement and limit-constrained discrete time profile (pure Python)** ([details](MOT-07.3.md)): Built the core time-parameterization engine that turns an untimed robot-arm Cartesian plan into a timed joint trajectory respecting the arm's speed and acceleration limits, plus its config file and a 
…[392 chars clipped]
- **MOT-07.4 — Retimer: constant tool speed on trace legs + §11.2 trajectory and crackvision.retime_report/1 assembly with self-check** ([details](MOT-07.4.md)): Built the MOT-07.4 retimer: it takes a geometric crack-trace plan and produces a timed robot trajectory that moves at one steady, configurable speed along each trace while respecting every joint's spe
…[398 chars clipped]
- **MOT-07.6 — Independent trajectory checker (conda side), part 1: schema, provenance, FD limits, §11.6 margin, rest and sampling + CLI** ([details](MOT-07.6.md)): Built the first half of the independent trajectory safety-checker for the robot arm's motion pipeline: a standalone tool (crackvision.trajectory_check) that re-derives, from the written specification 
…[442 chars clipped]
- **MOT-07.5 — retime_plan CLI (ROS-free console script): plan → data/trajectories/{case}_trajectory.json + .retime.json** ([details](MOT-07.5.md)): Added the retime_plan command-line tool that turns a planned cartesian path into a timed robot trajectory file, following the project's §16.6 specification exactly. It is registered as both a ROS cons
…[255 chars clipped]
- **MOT-07.7 — Independent trajectory checker, part 2: path fidelity, constant tool speed (crackvision.kinematics FK) and retime-report consistency** ([details](MOT-07.7.md)): Finished the second half of the independent trajectory checker: it now also verifies that a planned robot path stays on its intended route (within 1e-6 rad), that crack-tracing moves hold a steady too
…[367 chars clipped]
- **MOT-07.11 — Retimer G-TOOLSPEED is infeasible on real plan_crack_path trace legs: tool-speed dips at joint-space vertices** ([details](MOT-07.11.md)): The retimer now handles a real crack-tracing plan that it used to refuse. It accepts short tool-speed dips where the joint path turns sharply (1–18° on the test plan), as long as each dip is at most 5
…[343 chars clipped]
- **MOT-07.8 — Offline cross-validation: retimer vs independent checker vs G-LIMITS on a committed synthetic plan, with fault injection** ([details](MOT-07.8.md)): The offline cross-validation test for the motion retimer now runs cleanly end-to-end: it retimes a committed synthetic robot path, checks it independently, and confirms the safety limit gate agrees. I
…[155 chars clipped]
- **MOT-07.9 — Headless mock execution of a retimed synthetic crack plan + TIME_PARAMETERIZATION.md** ([details](MOT-07.9.md)): Added scripts/ros/test_retime_execute.sh (run_cartesian_synthetic.sh -> retime_plan -> trajectory_check -> execute_trajectory --mode dry/mock, moveit_mock profile only) and docs/motion/TIME_PARAMETERI
…[111 chars clipped]
- **MOT-07.10.1 — Decide and append the view_move purpose contract (ADR-019 addendum + INTERFACES §17)** ([details](MOT-07.10.1.md)): Settled the long-standing "view_move purpose" design gap for the robot motion stack by writing a formal decision (ADR-019 addendum) and a detailed specification (INTERFACES.md §17) for how a camera-po
…[461 chars clipped]
- **MOT-05.6 — Operator execution runbook (mock → dry → real) + ROS_WORKSPACE.md entry** ([details](MOT-05.6-v2.md)): Added the operator runbook (docs/motion/EXECUTION.md) that tells anyone running the robot executor exactly how to go from a safe mock test to a real-hardware run, including why real mode is currently 
…[336 chars clipped]
- **MOT-07.10.2 — Accept view_move in joint_trajectory and execution_gate (G-TRAJ, G-ELIGIBLE) with gate regression** ([details](MOT-07.10.2.md)): Joint trajectories can now declare purpose \"view_move\" (binding a view plan instead of a paths3d file), and the execution gate's offline checks validate that binding end-to-end: it confirms the view
…[550 chars clipped]
- **MOT-05.3 — Offline execution gate: commissioning/approval/execution-config loaders + full gate report (pure Python)** ([details](MOT-05.3-v2.md)): Built the offline safety gate that must pass before the robot arm's trajectory executor is allowed to run in real mode: it checks trajectory validity, joint-limit compliance, speed caps, scene/end-eff
…[521 chars clipped]
- **MOT-07.10.3 — retime_plan --view-plan: retime the view_plan transit leg into a view_move trajectory** ([details](MOT-07.10.3.md)): Implemented retime_plan --view-plan (§17.3): factored retime.retime_plan_doc's shared §16.2-16.4 body into a private _retime_legs_doc helper, added retime_view_plan_doc on top of it, and updated retim
…[506 chars clipped]
- **MOT-07.10.4 — trajectory_check: accept a view_plan and check view_move trajectories independently** ([details](MOT-07.10.4.md)): trajectory_check now also validates view-move trajectories against a view_plan file, running the same path/limits/report checks used for crack-task trajectories but over the single transit-leg polylin
…[328 chars clipped]
- **MOT-07.10.5 — Headless mock run: view_plan → retime_plan --view-plan → trajectory_check → execute_trajectory dry/mock** ([details](MOT-07.10.5.md)): Wrote scripts/ros/test_view_retime_execute.sh (modelled on test_retime_execute.sh) proving the headless mock chain view_plan -> retime_plan --view-plan -> trajectory_check -> execute_trajectory (dry, 
…[146 chars clipped]
- **MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves** ([details](MOT-07.10.md)): All sub-tasks of this work package were completed and audited together: MOT-07.10.1, MOT-07.10.1.R1, MOT-07.10.2, MOT-07.10.2.R1, MOT-07.10.2.R2, MOT-07.10.3, MOT-07.10.4, MOT-07.10.5.

## Quality loop

- Implementation/review sessions run: 29 / 292
- Audits that rejected work with evidence-backed findings: 10 (repairs created: 11)
- Re-verified after an upstream change: CAM-02, GEOM-01, GEOM-01.R1, GEOM-08.1, GEOM-09.1, GEOM-09.1.R1, GEOM-10, MOT-02, MOT-03, MOT-04.1, MOT-04.1.R1, MOT-04.2, MOT-04.3, MOT-04.3.R1, MOT-04.4, MOT-04.4.R1, MOT-04.5, MOT-04.6, MOT-04.6.R1, MOT-05.1, MOT-05.1.R1, MOT-05.2, MOT-05.3, MOT-05.3.R1, MOT-05.3.R2, MOT-05.4, MOT-05.4.R1, MOT-05.5, MOT-05.6, MOT-05.6.R1, MOT-05.7, MOT-05.7.R1, MOT-06.10, MOT-06.11, MOT-06.11.R1, MOT-06.2, MOT-06.2.R1, MOT-06.4, MOT-06.4.R1, MOT-06.6, MOT-06.6.R1, MOT-06.9, MOT-07.1, MOT-07.1.R1, MOT-07.10.1.R1, MOT-07.10.2.R1, MOT-07.10.2.R2, MOT-07.11, MOT-07.2, MOT-07.3, MOT-07.4, MOT-07.4.R1, MOT-07.5, MOT-07.5.R1, MOT-07.6, MOT-07.6.R1, MOT-07.7, MOT-07.7.R1, MOT-07.8, MOT-07.8.R1, MOT-07.9, MOT-10.1, MOT-10.1.R1, MOT-10.1.R2, MOT-10.3, PERC-03, PERC-03.R1

## Planning changes

- decomposed: MOT-07.10 → MOT-07.10.1, MOT-07.10.2, MOT-07.10.3, MOT-07.10.4, MOT-07.10.5
- decomposed: INT-01 → INT-01.1, INT-01.2, INT-01.3
- decomposed: MOT-08 → MOT-08.1, MOT-08.2, MOT-08.3, MOT-08.4, MOT-08.5, MOT-08.6, MOT-08.7, MOT-08.8, MOT-08.9

## Progress by work package

| Work package | Accepted | Total |
|---|---|---|
| Host platform & reproducibility (L-HOST) | 10 | 10 |
| Local coding model (L-LLM) | 5 | 8 |
| D405 capture (L-CAM) | 7 | 9 |
| Perception (L-PERC) | 13 | 16 |
| Geometry & calibration (L-GEOM) | 20 | 32 |
| Motion planning (L-MOTION) | 37 | 66 |
| Integration & commissioning (L-INT) | 1 | 9 |
| Operator workflow & handoff (L-OPS) | 1 | 4 |
| Imitation learning (later phase) (L-IL) | 0 | 1 |

## Waiting on the operator / hardware (snapshot)

- CAM-01: hardware — operator must declare: d405_connected
- GEOM-04.1: privilege_denied — Denied. This request fails criteria 1, 2, 3 and 5.

(1) Root isn't needed. The target is /home/boosterk1/miniconda3/envs
…[2288 chars clipped]
- LLM-06.1: dependency — The backlog currently has zero cards that are simultaneously unaccepted/ready, non-hardware, motion!=true, routing.conse
…[181 chars clipped]
- MOT-09: hardware — operator must declare: arm_powered_estop_verified, workspace_clear
