# MOT-05.1 — Decision + contracts: commissioning-gated execution (ADR-016 + INTERFACES §11)

```json card
{
  "kind": "decision",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "docs/adr/016-commissioning-gated-execution.md",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/adr/000-template.md",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#8",
    "docs/INTERFACES.md#10",
    "config/robot/b601_dm_limits.yaml",
    "config/robot/end_effector.yaml",
    "config/scene/scene.yaml",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py",
    "ros2_ws/src/crackvision_description/crackvision_description/end_effector.py",
    "ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py",
    "docs/motion/ROBOT_MODEL.md",
    "docs/motion/SCENE.md",
    "plan/cards/MOT-07.md",
    "plan/cards/MOT-08.md",
    "plan/cards/MOT-09.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "interfaces-append-only",
        "cmd": "bash -c 'n=$(git show c6c90ba:docs/INTERFACES.md | wc -l); diff <(git show c6c90ba:docs/INTERFACES.md) <(head -n \"$n\" docs/INTERFACES.md)'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "section-11-schemas",
        "cmd": "bash -c 'grep -q \"^## 11\\. \" docs/INTERFACES.md || exit 1; for s in crackvision.joint_trajectory/1 crackvision.execution_config/1 crackvision.commissioning/1 crackvision.execution_approval/1 crackvision.execution_record/1 execute_trajectory; do grep -qF \"$s\" docs/INTERFACES.md || { echo \"missing $s\"; exit 1; }; done'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "adr-present",
        "cmd": "bash -c 'f=docs/adr/016-commissioning-gated-execution.md; test -f $f && grep -q \"ADR-014\" $f && grep -q \"assert_commissioning_ready\" $f && grep -qi \"e-stop\" $f'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§11 is appended after §10. §0–§10 are byte-identical to c6c90ba.",
      "Every gate check has a stable id and a mock/dry/real applicability matrix, and the doc separates offline checks, online read-only checks and confirmation. Real mode passes only if every applicable check passes, and every failing check is reported, not just the first.",
      "Real mode calls both scene_core.assert_commissioning_ready and end_effector.assert_commissioning_ready, with no exception path. The ADR records that GEOM-05, GEOM-07 and MOT-09 seem to need the arm moved before calibration, and how they proceed without bypassing the gate. Any 'commissioning tier' exception is written up as an escalated alternative that needs technical-lead approval, not as the decision.",
      "The speed policy states the real speed_scale default and cap of ≤0.10 until the commissioning record raises it with operator evidence, and defines retiming as uniform time scaling (v·s, a·s²).",
      "Limits hash: in real mode the commissioning record's limits_file_sha256 must equal the current b601_dm_limits.yaml. The trajectory's limits sha is checked in every mode when non-null, and null is accepted only in mock and dry.",
      "The position-margin policy is explicit and handles the all-zero SRDF home, which lies exactly on joint2/joint3 upper=0.0.",
      "The vendor real-driver node name, its action and status interfaces, and what /rebotarm/disable physically does (torque drop and gravity fall?) are stated with file:line citations into ~/rebot_ws/src (read-only). The e-stop behaviour (cancel/hold/disable) is decided from that evidence. The hardware e-stop is the safety function; the software e-stop topics are secondary.",
      "The executor never calls vendor motion-producing services (enable, safe_home, park, move_to_pose_ik, gravity_compensation/*, set_zero, set_mode, gripper/*), and the ADR says who performs each of these steps.",
      "Confirmation requires both the CRACKVISION_ARM_REAL=1 env and a typed phrase bound to the trajectory sha read from a controlling /dev/tty. There is no flag, env or stdin bypass.",
      "Approval (crackvision.execution_approval/1) is bound to the trajectory sha256 and has a max age. The paths3d execution_eligible rule from §10.6 is enforced. Stale end_effector/scene sha bindings are refused in real mode.",
      "Real-mode collision validity has a stated topology, i.e. which move_group answers /check_state_validity while the vendor driver runs, and mock mode is proven unable to address the real driver (and real mode unable to address a mock)."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 4,
    "context": 4,
    "consequence": 5,
    "task_class": "safety-architecture-decision",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-05.1",
  "parent": "MOT-05",
  "title": "Decision + contracts: commissioning-gated execution (ADR-016 + INTERFACES §11)",
  "outcome": "ADR-016 fixes the safety design of the mock/dry/real executor, and docs/INTERFACES.md §11 (appended) defines its normative contracts: modes and driver profiles, the joint-trajectory file, execution config, commissioning record, approval record, gate-check catalogue with mode matrix, confirmation, e-stop/monitoring, execution record and the execute_trajectory CLI. MOT-05.2–.6 implement it verbatim."
}
```

Write ADR-016 (follow docs/adr/000-template.md: context, decision, alternatives, consequences) and append INTERFACES §11. Do not implement code. Read ~/rebot_ws read-only (rebotarmcontroller/ros_actions.py, ros_services.py and the node that publishes arm status; rebot_motion/mock_driver.py and motion_runner.py; deploy/run_real_demo.sh and preflight.sh) and cite file:line for every vendor fact.

Verified facts to build on (lead survey, 2026-10-08):
- Real driver: ActionServer control_msgs/FollowJointTrajectory at `/{ns}/follow_joint_trajectory` (ns default `rebotarm`). It requires joint_names == hardware joint_names and honours point time_from_start. Trigger services at /{ns}/: enable, disable, safe_home, park, gravity_compensation/start|stop, set_zero, set_mode, move_to_pose_ik, gripper/set|open|close. It publishes /{ns}/joint_states. Per MOT-01, the real bringup publishes a gripper-less URDF.
- rebot_motion mock_driver: node `mock_rebotarm_driver`, same interface as the real driver (/{ns}/follow_joint_trajectory, /{ns}/joint_states at 50 Hz, Trigger services). It is in ~/rebot_ws/install.
- The MOT-02 mock stack (mock_planning.launch.py) uses ros2_control mock hardware, action `/rebotarm_controller/follow_joint_trajectory` and `/joint_states`, and move_group provides /check_state_validity, /get_planning_scene, /apply_planning_scene. Initial positions and the SRDF `home` are all 0.0. joint2/joint3 limits are [-3.14, 0.0], so home sits on their upper limit.
- Deploy pattern being reused: two independent axes (driver mock|real × plan dry|go), REAL=1 env plus a typed 'yes' on /dev/tty, refusal without a tty, teardown on any exit, an e-stop topic (/rebot_motion/estop std_msgs/Bool), and a velocity cap.

Recommended decisions. Adopt them unless vendor evidence contradicts; if you deviate, justify it in the ADR.
1. Modes: `mock` (sends goals only to a mock), `dry` (offline gate plus all online read-only checks, never constructs an ActionClient; usable against the live vendor driver as a rehearsal, and also prints what real mode would refuse) and `real`. The CLI default mode is `dry`.
2. Driver profiles in config/motion/execution.yaml: `moveit_mock` (action /rebotarm_controller/follow_joint_trajectory, joint_states /joint_states, modes mock|dry), `vendor_mock` (/rebotarm/…, expected node mock_rebotarm_driver, modes mock|dry) and `vendor` (/rebotarm/…, expected node = the real driver's node name, modes dry|real). Graph discrimination: mock mode refuses if the real driver node is visible; real mode refuses unless the expected real node is the server and no mock driver node is visible.
3. Gate ids (suggested): G-ARM (real: env CRACKVISION_ARM_REAL=1), G-TRAJ (schema valid), G-LIMITS-HASH, G-LIMITS (positions, velocities and accelerations after scaling, explicit and finite-difference), G-SPEED (scale ≤ cap), G-SCENE (scene_core gate), G-EE (end_effector gate), G-COMMISSIONING (record commissioned, limits sha), G-ESTOP (hardware e-stop verified in the record), G-ELIGIBLE (paths3d execution_eligible true and its sha matches), G-APPROVAL, G-STALE-CONFIG (end_effector/scene sha bindings), then online G-GRAPH, G-START-STATE (fresh joint_states, tolerance) and G-COLLISION (densified waypoints via /check_state_validity with the production scene objects present), then G-CONFIRM (real). In mock and dry, real-only checks are reported as `skip`, never `pass`.
4. ADR-014: strict for every real execution. Record the ordering problem and recommend that GEOM-05/GEOM-07 capture hand-guided poses (vendor gravity compensation, a separate operator procedure outside this executor) and that MOT-09 does readback-only dry rehearsals until calibration lands. Present a 'commissioning purpose tier' only as an escalated alternative.
5. Trajectory file crackvision.joint_trajectory/1 (JSON): schema, created_utc, producer, purpose (`crack_task` | `test`; real refuses `test`), planning_frame, joint_names exactly [joint1..joint6], points[{t_s (first 0, strictly increasing), positions[6], velocities[6]?, accelerations[6]?}], limits_file{path, sha256|null}, end_effector_config_sha256|null, scene_config_sha256|null, source{paths3d{path, sha256, execution_eligible}}|null. MOT-07 must emit this format.
6. Commissioning record config/robot/commissioning.yaml (crackvision.commissioning/1): commissioned bool, limits_file_sha256, speed_scale_cap (default 0.10), estop{kind: hardware, verified, verified_utc, operator, evidence}, joint_ranges_verified, evidence[], source. It ships uncommissioned, and MOT-09 fills it.
7. Approval crackvision.execution_approval/1: trajectory_sha256, approved_by, approved_utc, preview_artifacts[{path, sha256}]. Max age comes from execution config. MOT-08 produces it.
8. Confirmation phrase: `EXECUTE <first 8 hex of trajectory sha256>`, typed on /dev/tty, exact match after strip, with the e-stop location and speed scale shown first.
9. Monitoring: e-stop topics /crackvision/estop and /rebot_motion/estop (Bool true); SIGINT/SIGTERM counts as an e-stop; joint_states staleness > timeout or tracking error > tolerance cancels the goal. Decide disable-after-cancel from the vendor code.
10. Record crackvision.execution_record/1, written in every mode, including refusals, to logs/execution/<stamp>_<mode>_<sha8>.json. It holds the gate report, all input shas, scale, outcome and a sampled joint trace. The §0.4 logs are written as well.
11. Exit codes: 0 ok; 1 runtime (aborted, e-stop, tracking, action failure); 2 usage; 3 precondition (any gate refusal, missing services). `--dry-run` (§0.3) = offline gate report only: no ROS, nothing written, exit 0. Distinguish it clearly from `--mode dry`.
12. Real-mode collision topology: recommend running the MOT-02 mock_planning stack purely as a validity oracle, with production scene applied. In real mode its mock controller is never addressed, which the profile guarantees. Validity requests carry an explicit robot_state, so it does not matter that the oracle's monitored state is the mock state. State this caveat.

Also list in §11 the default numeric values (start tolerance, tracking tolerance, joint_state timeout, densify step, position margin, approval max age) with a rationale for each.
