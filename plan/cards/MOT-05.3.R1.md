# MOT-05.3.R1 — Repair MOT-05.3: evaluate_offline raises on bad inputs and out-of-range speed_scale ins

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "config/motion/execution.yaml",
      "config/robot/commissioning.yaml",
      "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/**",
      "ros2_ws/src/crackvision_motion/test/test_execution_gate.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#10",
    "docs/adr/016-commissioning-gated-execution.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py",
    "ros2_ws/src/crackvision_description/crackvision_description/end_effector.py",
    "config/scene/scene.yaml",
    "config/robot/end_effector.yaml",
    "config/robot/b601_dm_limits.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_execution_gate.py ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "upstream-gates-regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_scene_core.py ros2_ws/src/crackvision_description/test/test_end_effector.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "shipped-uncommissioned",
        "cmd": "bash -c 'grep -qE \"^commissioned: *false\" config/robot/commissioning.yaml && grep -q \"crackvision.commissioning/1\" config/robot/commissioning.yaml && grep -q \"crackvision.execution_config/1\" config/motion/execution.yaml'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "pure-python",
        "cmd": "bash -c '! grep -nE \"^\\s*(import|from) +(rclpy|control_msgs|moveit_msgs|sensor_msgs)\" ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "For each of the cases above, evaluate_offline returns a GateReport and does not raise. The affected gate shows fail (or §11 error): G-TRAJ for a missing trajectory; G-SCENE/G-EE (and G-STALE-CONFIG) for missing or malformed scene/end-effector files; G-SPEED for speed_scale 0, -1 or 2.0, with G-LIMITS reporting not-evaluable rather than raising. load_execution_config and load_commissioning raise GateConfigError on malformed YAML. Tests pin each case.",
      "GateCheck can carry §11.7's outcomes: warn in mock/dry for every 'warn' cell, and error for precondition or unreadable-input cases. warn never appears in real. GateReport.passed and refusals treat fail and error as refusing and warn as non-refusing. to_dict() emits gate_report entries with keys gate, category, outcome, detail, where category is offline or confirmation per §11.7. Tests assert warn for G-APPROVAL, G-SCENE and G-EE in mock/dry with the repo configs, error for G-LIMITS when G-TRAJ cannot load, and the §11.9 key set.",
      "All acceptance criteria of MOT-05.3 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "python-pure-lib",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-05.3",
  "findings": [
    "07f73b3be28b9c43",
    "ed70cd8f42944e41"
  ],
  "id": "MOT-05.3.R1",
  "title": "Repair MOT-05.3: evaluate_offline raises on bad inputs and out-of-range speed_scale ins",
  "parent": "MOT-05",
  "outcome": "Resolve the review findings on MOT-05.3 while every acceptance criterion of MOT-05.3 still holds."
}
```

Repair work generated deterministically from review attempt `00298-MOT-05.3-review` of `MOT-05.3`.
Original card: `plan/cards/MOT-05.3.md` — its acceptance criteria must still hold.

## Finding 1 [major] evaluate_offline raises on bad inputs and out-of-range speed_scale instead of returning failed checks
- Evidence: I called evaluate_offline from a scratch script with the repo configs and swapped in one bad input per case. Output: 'RAISE missing trajectory: FileNotFoundError', 'RAISE missing scene: FileNotFoundError', 'RAISE bad yaml scene: ParserError', 'RAISE bad yaml ee: ParserError', 'RAISE missing ee: FileNotFoundError', 'RAISE speed 0: TrajectoryError: speed_scale must be in (0, 1], got 0.0', 'RAISE speed 2.0: TrajectoryError ...', 'RAISE speed -1 (real): TrajectoryError ...', 'RAISE bad yaml commissioning: ParserError', 'RAISE bad yaml exec cfg: ParserError'. Causes: (a) load_trajectory calls Path.read_text() before its own try (joint_trajectory.py:84), and the G-TRAJ block catches only TrajectoryError. (b) The G-SCENE and G-EE blocks catch only SceneError and EndEffectorError, but scene_core.load_config opens the file directly and both loaders call yaml.safe_load unguarded. (c) _evaluate_stale_config calls _sha256_of(scene/ee path) with no handler. (d) The G-LIMITS block calls scale_time(traj, effective_scale), which raises TrajectoryError for s outside (0,1], before G-SPEED is evaluated. (e) The _load_yaml_mapping helper (used by load_execution_config and load_commissioning) lets yaml.YAMLError escape instead of raising GateConfigError.
- Consequence: This breaks §11.7 ('Every gate listed for the active mode is evaluated and reported') and the card's 'nothing propagates' criterion. MOT-05.4 would see a crash, so its --dry-run and execution record would be missing or exit 1 instead of a structured refusal with exit 3. A speed_scale above 1.0 or at or below 0, the exact case G-SPEED must refuse, never reaches G-SPEED. A malformed execution.yaml or commissioning.yaml escapes as a yaml error rather than the GateConfigError the strict loaders promise.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/test/test_execution_gate.py
- Acceptance condition: For each of the cases above, evaluate_offline returns a GateReport and does not raise. The affected gate shows fail (or §11 error): G-TRAJ for a missing trajectory; G-SCENE/G-EE (and G-STALE-CONFIG) for missing or malformed scene/end-effector files; G-SPEED for speed_scale 0, -1 or 2.0, with G-LIMITS reporting not-evaluable rather than raising. load_execution_config and load_commissioning raise GateConfigError on malformed YAML. Tests pin each case.

## Finding 2 [major] §11.7 warn/error outcomes folded into pass/fail; to_dict is not the §11.9 gate_report shape
- Evidence: The card says 'If this body and §11 disagree, §11 wins.' The implementer's note says the opposite: 'per the card body (which wins over §11 on conflict), I collapsed §11.7's 5-state outcome model'. The module docstring repeats this. §11.7 defines warn ('Only reported, never refuses') and error ('could not be evaluated ... Refuses like fail'), and the matrix marks G-APPROVAL 'warn' in mock and dry. In the code, the G-SCENE block reports GateCheck(G_SCENE, "pass", f"loads; not commissioning-ready (warn in {mode}) ..."), and _evaluate_approval sets fail_status = "fail" if mode == "real" else "pass". GateReport.to_dict emits {"id","status","message"} per check. §11.9 requires gate_report[] list[{gate, category, outcome, detail}] with outcome ∈ pass|fail|warn|skip|error. The dry report from my probe shows G-SCENE 'status': 'pass' for the nominal production scene.
- Consequence: The criterion 'GateReport.to_dict() ... is the gate block of crackvision.execution_record/1' does not hold, and the §11.7 matrix (its warn cells) does not match exactly. Mock and dry records would label a nominal scene or end-effector, a null sha and a missing approval as 'pass'. That misrepresents safety evidence, and MOT-05.4 cannot derive would_refuse_in_real from gate_report as §11.7 specifies. Precondition failures are indistinguishable from evaluated fails. Downstream MOT-05.4 would build on a non-normative shape.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/test/test_execution_gate.py
- Acceptance condition: GateCheck can carry §11.7's outcomes: warn in mock/dry for every 'warn' cell, and error for precondition or unreadable-input cases. warn never appears in real. GateReport.passed and refusals treat fail and error as refusing and warn as non-refusing. to_dict() emits gate_report entries with keys gate, category, outcome, detail, where category is offline or confirmation per §11.7. Tests assert warn for G-APPROVAL, G-SCENE and G-EE in mock/dry with the repo configs, error for G-LIMITS when G-TRAJ cannot load, and the §11.9 key set.
