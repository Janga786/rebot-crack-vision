# MOT-05.1.R1 — Repair MOT-05.1: G-LIMITS checks only after speed scaling, so the 0.10 real cap does no

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "docs/adr/016-commissioning-gated-execution.md"
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
      "§11.7 G-LIMITS (and ADR §4) require the unscaled trajectory (scale 1.0) to be within b601_dm_limits.yaml, in addition to (or instead of) the scaled check. Alternatively, they state that in real mode the scaled trajectory must be within speed_scale_cap × the limits. Either way, the 10%-of-limits claim becomes a checked property.",
      "§11.6 states a sign-correct rule for both bounds (e.g. for the upper bound, v ≤ 0 unless the point is within the inclusive bound and decelerating to it). That rule must explicitly accept a trajectory that ends at, or passes through, joint2/joint3 = 0.0 within limits, illustrated with a home-approach example.",
      "§11.7 states that in real mode a crack_task trajectory with null source.paths3d is refused. It also states that G-ELIGIBLE reads execution_eligible (and ineligible_reasons) from the paths3d file itself, after verifying its sha256, rather than trusting the copy in the trajectory.",
      "§11.7 and ADR §10 consistently state that real mode refuses null end_effector_config_sha256/scene_config_sha256, and they state the mock/dry behaviour on a mismatch.",
      "§11.7/§11.10 define, per mode, which `check` gate failures refuse (exit 3), which are only reported, and dry mode's exit code when it reports a refusal that only real mode would make.",
      "§11 states the real-mode default speed_scale (≤0.10) and the --speed-scale flag. It picks one of clamp or refuse, consistently in the ADR and §11.7, and uses one set of execution.yaml key names.",
      "ADR §7 and §11.8 either (a) choose cancel-and-hold, with disable left to the operator or hardware e-stop until MOT-09 confirms a disable brings the arm to a safe rest, or (b) explicitly justify why an automatic torque drop at an arbitrary pose is acceptable given the possible-fall assumption, and say which triggers (if any) disable.",
      "ADR Context and §8 list /rebotarm/move_to_pose, /rebotarm/joints/*/cmd/* and /rebotarm/arm_status with file:line citations. §8's never-call/owner table covers move_to_pose and the passthrough topics.",
      "§11.10 gives the invocation as the crackvision_motion ROS entry point (e.g. `ros2 run crackvision_motion execute_trajectory`, after sourcing scripts/ros/env_ros.sh and the overlay), consistent with MOT-05.4, and says where --dry-run runs without ROS.",
      "All acceptance criteria of MOT-05.1 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 5,
    "ambiguity": 4,
    "context": 4,
    "consequence": 5,
    "task_class": "safety-architecture-decision",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 99,
  "repairs": "MOT-05.1",
  "findings": [
    "f473aa6c4c25642c",
    "ec6197744e98dcc9",
    "9f2226de63c863fe",
    "534cf636e3341d27",
    "ec6d56a2d0eb17aa",
    "dbb7862e58e08e89",
    "161f9795e07ee81b",
    "105a4a50e1be7842",
    "b990916b0edaebf2"
  ],
  "id": "MOT-05.1.R1",
  "title": "Repair MOT-05.1: G-LIMITS checks only after speed scaling, so the 0.10 real cap does no",
  "parent": "MOT-05",
  "outcome": "Resolve the review findings on MOT-05.1 while every acceptance criterion of MOT-05.1 still holds."
}
```

Repair work generated deterministically from review attempt `00228-MOT-05.1-review` of `MOT-05.1`.
Original card: `plan/cards/MOT-05.1.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] G-LIMITS checks only after speed scaling, so the 0.10 real cap does not bound speed relative to the limits
- Evidence: INTERFACES.md:1444 G-LIMITS: 'every point's positions/velocities/accelerations, after `speed_scale` retiming, within `b601_dm_limits.yaml`'. ADR:157-158: '`G-LIMITS` is evaluated **after** this scaling is applied, never before.' ADR:146-148 claims 'real execution is capped at 10% of the b601_dm_limits.yaml velocity/acceleration limits', and ADR:153 only *assumes* the input 'already respect[s] the limits at scale = 1.0'.
- Consequence: Take a trajectory file whose velocities are 10× the limits (10 rad/s on joint1) and accelerations 100× the limits. Retimed at 0.10 it lands exactly at 100% of the limits and passes G-LIMITS. The real arm then runs at full planning-limit speed, not 10% of it, and the commissioning cap meant to keep it slow is defeated. MOT-05.2/.3 implement this verbatim.
- Affected: docs/INTERFACES.md, docs/adr/016-commissioning-gated-execution.md
- Acceptance condition: §11.7 G-LIMITS (and ADR §4) require the unscaled trajectory (scale 1.0) to be within b601_dm_limits.yaml, in addition to (or instead of) the scaled check. Alternatively, they state that in real mode the scaled trajectory must be within speed_scale_cap × the limits. Either way, the 10%-of-limits claim becomes a checked property.

## Finding 2 [major] Position-margin rule (b) rejects every trajectory that approaches home, and its sign is wrong for lower bounds
- Evidence: INTERFACES.md:1424-1427: '(b) requires that any `near_limit` point's velocity in that joint be `<= 0` moving toward the bound it is near'. ADR:192-196 repeats this. b601_dm_limits.yaml: joint2/joint3 upper: 0.0. position_margin_rad default 0.03 (§11.12).
- Consequence: Take any trajectory ending at the all-zero home, e.g. joint2 going from -0.5 to 0.0. Its last points, at -0.02 and -0.01, are near_limit and have positive velocity (explicit or finite-difference) toward upper=0.0, so G-LIMITS fails. Returning to home, the canonical rest pose, is therefore unplannable. And for a near-lower-bound point, '<= 0' would require moving *toward* the lower bound, the opposite of the intent. MOT-05.2's acceptance pins 'the margin policy matches §11, including the all-zero home'.
- Affected: docs/INTERFACES.md, docs/adr/016-commissioning-gated-execution.md
- Acceptance condition: §11.6 states a sign-correct rule for both bounds (e.g. for the upper bound, v ≤ 0 unless the point is within the inclusive bound and decelerating to it). That rule must explicitly accept a trajectory that ends at, or passes through, joint2/joint3 = 0.0 within limits, illustrated with a home-approach example.

## Finding 3 [major] G-ELIGIBLE can be bypassed in real mode: null source.paths3d passes, and it trusts the recorded bool
- Evidence: INTERFACES.md:1450 G-ELIGIBLE: 'when `source.paths3d` is present: its recorded `execution_eligible == true` and its `sha256` matches'. §11.2 allows the whole source.paths3d object to be null. §10.6: 'MOT-05 must refuse real execution of any path whose file reports execution_eligible: false.'
- Consequence: Take a purpose: crack_task trajectory with source.paths3d: null, or one whose recorded execution_eligible is hand-set to true while the referenced paths3d file says false (same sha, since the field is copied, not read). Either passes G-ELIGIBLE in real mode. This lets crack paths from nominal calibration or a synthetic capture run on the real arm, against the §10.6 rule the card says must be enforced.
- Affected: docs/INTERFACES.md, docs/adr/016-commissioning-gated-execution.md
- Acceptance condition: §11.7 states that in real mode a crack_task trajectory with null source.paths3d is refused. It also states that G-ELIGIBLE reads execution_eligible (and ineligible_reasons) from the paths3d file itself, after verifying its sha256, rather than trusting the copy in the trajectory.

## Finding 4 [major] G-STALE-CONFIG accepts null end_effector/scene bindings in real mode, and the ADR and table disagree on which modes it refuses
- Evidence: INTERFACES.md:1452: 'trajectory's `end_effector_config_sha256`/`scene_config_sha256`, when non-null, match the *current* config files'. Check is marked for all modes. ADR:255: '`G-STALE-CONFIG` independently refuses `real` if…'. Contrast G-LIMITS-HASH, where 'real refuses a null sha unconditionally' (ADR:168-170).
- Consequence: In real mode, a trajectory with null EE/scene shas cannot be checked for staleness, yet it passes. A trajectory planned against an old end-effector or scene would run against today's configs. Implementers also get contradictory guidance on whether a sha mismatch refuses mock or dry.
- Affected: docs/INTERFACES.md, docs/adr/016-commissioning-gated-execution.md
- Acceptance condition: §11.7 and ADR §10 consistently state that real mode refuses null end_effector_config_sha256/scene_config_sha256, and they state the mock/dry behaviour on a mismatch.

## Finding 5 [major] §11 never says when mock and dry runs refuse or what they exit with
- Evidence: INTERFACES.md:1437 defines only: '`real` passes only if every gate marked `check` for `real` reports `pass`.' Yet G-APPROVAL, G-ELIGIBLE and G-START-STATE are `check` in mock/dry (lines 1450-1455). §11.10 exit 3 = 'any `G-*` gate refusal'. MOT-05.5 expects 'dry mode exits 0' against the production configs, and expects mock to exit 3 on a start mismatch.
- Consequence: The contract gives MOT-05.3/.4/.5 no answer to: does mock refuse without an approval file, or with an ineligible paths3d (which is every paths3d today, since calibration is nominal)? Does dry exit 3 when gates fail, or report and exit 0? Implementations and tests will diverge, or will force approvals and eligible inputs for mock smoke tests.
- Affected: docs/INTERFACES.md
- Acceptance condition: §11.7/§11.10 define, per mode, which `check` gate failures refuse (exit 3), which are only reported, and dry mode's exit code when it reports a refusal that only real mode would make.

## Finding 6 [major] Speed policy is incomplete and self-contradictory: no real default, clamp vs fail, no CLI flag, mismatched key names
- Evidence: ADR:146 'effective `speed_scale` is `min(requested_scale, commissioning.speed_scale_cap)`' vs ADR:151 '`G-SPEED` fails if the requested scale exceeds the mode-appropriate cap'. INTERFACES.md:1360 `speed_scale.default` is 'for `mock`/`dry` runs' only; no real default is stated anywhere. The §11.10 CLI synopsis has no --speed-scale flag, though MOT-05.4 lists one. ADR:150 uses the names `default_speed_scale`/`speed_scale_cap`, but §11.1 uses `speed_scale.default`/`speed_scale.cap`.
- Consequence: The card criterion ('states the real speed_scale default') is unmet. Under min(), G-SPEED can never fail; under fail-on-request, there is no clamp. And because §11 'wins' over the MOT-05.4 body, the CLI has no way to pass a scale.
- Affected: docs/INTERFACES.md, docs/adr/016-commissioning-gated-execution.md
- Acceptance condition: §11 states the real-mode default speed_scale (≤0.10) and the --speed-scale flag. It picks one of clamp or refuse, consistently in the ADR and §11.7, and uses one set of execution.yaml key names.

## Finding 7 [major] E-stop response auto-disables an arm the ADR itself assumes may fall under gravity
- Evidence: ADR:52-53: '**This ADR treats `disable` as torque-off with a possible uncontrolled fall under gravity**'. ADR:206-208 and INTERFACES.md:1465-1469 nevertheless trigger cancel-then-disable on any e-stop source, including joint_states staleness and tracking error. The vendor evidence the ADR cites shows cancel already holds position (ros_actions.py:175-176 hold_current_position). The vendor's own shutdown safe-homes *before* disabling (rebotarm_controller.py:104-109; hardware_manager.py:199-216).
- Consequence: A transient tracking error or a joint_states hiccup with the tool 1 cm from the specimen (§8.2 trace clearance) drops torque at an arbitrary pose. By the ADR's own stated assumption, the arm then falls onto the specimen or operator. The decision is not derived from the evidence; the rationale (ADR:211-213) is only about not leaving the arm 'silently enabled'.
- Affected: docs/adr/016-commissioning-gated-execution.md, docs/INTERFACES.md
- Acceptance condition: ADR §7 and §11.8 either (a) choose cancel-and-hold, with disable left to the operator or hardware e-stop until MOT-09 confirms a disable brings the arm to a safe rest, or (b) explicitly justify why an automatic torque drop at an arbitrary pose is acceptable given the possible-fall assumption, and say which triggers (if any) disable.

## Finding 8 [major] Vendor surface enumeration misses a motion-producing action, the passthrough command topics and the arm_status status topic
- Evidence: ros_actions.py:19-26 creates the MoveToPose ActionServer at f"/{namespace}/move_to_pose". motor_passthrough.py:19-80 subscribes /{ns}/joints/<joint>/cmd/{mit,pos_vel,vel} and drives motors. ros_publishers.py:42-47 publishes ArmStatus on /{ns}/arm_status (latched). ADR:34-38 lists only services plus gripper/command, and ADR:216-218 claims to cover 'every vendor service/action… except follow_joint_trajectory and the readback-only joint_states'.
- Consequence: The card asks for the driver's 'action and status interfaces' to be stated. The status topic (arm_status, which also reports enabled/state machine and could back an 'arm enabled' precondition) is absent. The executor's never-call contract does not forbid the move_to_pose action or the raw motor command topics, both of which produce motion.
- Affected: docs/adr/016-commissioning-gated-execution.md
- Acceptance condition: ADR Context and §8 list /rebotarm/move_to_pose, /rebotarm/joints/*/cmd/* and /rebotarm/arm_status with file:line citations. §8's never-call/owner table covers move_to_pose and the passthrough topics.

## Finding 9 [major] CLI invocation contradicts the rclpy interpreter convention and the downstream cards
- Evidence: INTERFACES.md:1494: './env.sh python -m crackvision.execute_trajectory …'. Project convention: 'rclpy nodes use /usr/bin/python3'. The executor is an rclpy node (§11.7 online gates, ActionClient). MOT-05.4/05.6 bodies use `ros2 run crackvision_motion execute_trajectory`.
- Consequence: The §11 contract, which 'wins' over card bodies, puts the only motion-capable node under the conda interpreter that env.sh scrubs ROS from, and under a non-existent crackvision.* module path. Downstream implementers must either break §11 or break the ROS convention.
- Affected: docs/INTERFACES.md
- Acceptance condition: §11.10 gives the invocation as the crackvision_motion ROS entry point (e.g. `ros2 run crackvision_motion execute_trajectory`, after sourcing scripts/ros/env_ros.sh and the overlay), consistent with MOT-05.4, and says where --dry-run runs without ROS.
