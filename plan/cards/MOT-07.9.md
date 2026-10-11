# MOT-07.9 — Headless mock execution of a retimed synthetic crack plan + TIME_PARAMETERIZATION.md

```json card
{
  "kind": "integration",
  "depends_on": [
    "MOT-07.8"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2",
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "scripts/ros/test_retime_execute.sh",
      "docs/motion/TIME_PARAMETERIZATION.md",
      "docs/motion/ROS_WORKSPACE.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "docs/motion/EXECUTION.md",
    "docs/motion/CARTESIAN_PLANNING.md",
    "scripts/ros/run_cartesian_synthetic.sh",
    "scripts/ros/test_execution.sh",
    "scripts/ros/_mock_stack.sh",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "mock-exec",
        "cmd": "bash scripts/ros/test_retime_execute.sh",
        "timeout_s": 2400,
        "expect_exit": 0
      },
      {
        "id": "docs",
        "cmd": "test -s docs/motion/TIME_PARAMETERIZATION.md && grep -q 'retime_plan' docs/motion/ROS_WORKSPACE.md && grep -q 'trajectory_check' docs/motion/TIME_PARAMETERIZATION.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Only mock/dry modes are used, with the moveit_mock profile only. The script never sets CRACKVISION_ARM_REAL, never uses the vendor profile or the real mode, and tears down the mock stack on every exit path (trap).",
      "--mode dry exits 0 with would_refuse_in_real containing at least G-ELIGIBLE (the synthetic paths3d is ineligible) and G-APPROVAL. --mode mock exits 0 with execution record outcome completed and arrival verified. The G-LIMITS, G-STALE-CONFIG and G-COLLISION gate entries are pass, using the scratch scene/config passed explicitly so shas are consistent, never by editing production config.",
      "TIME_PARAMETERIZATION.md covers: the method and why not TOTG/Ruckig (from ADR-019); the measured results of this run (duration, points, per-trace-leg v_c and ramps, peak utilisation, execution record path); effective tool speed = speed_scale × v_c (5 mm/s at the real 0.10 default for a 0.05 m/s file); the caveats (nominal geometry, ineligible input, the view-plan gap); and the exact operator commands.",
      "ROS_WORKSPACE.md gains a retime_plan entry consistent with its existing CLI entries; no other content is changed."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "integration-test",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.9",
  "title": "Headless mock execution of a retimed synthetic crack plan + TIME_PARAMETERIZATION.md",
  "parent": "MOT-07",
  "outcome": "scripts/ros/test_retime_execute.sh runs synthetic plan → retime_plan → trajectory_check → execute_trajectory --mode dry and --mode mock (moveit_mock profile) on the headless mock stack, with mock completing and arrival verified; docs/motion/TIME_PARAMETERIZATION.md documents the method, results, caveats and operator commands, and ROS_WORKSPACE.md lists retime_plan."
}
```

`test_retime_execute.sh` follows `test_execution.sh` / `run_cartesian_synthetic.sh` patterns (`set -m`, `_mock_stack.sh`, a trap for teardown):
1. Run `scripts/ros/run_cartesian_synthetic.sh`, or reuse its helper steps, to get a fresh scratch root.
2. Run `retime_plan` with `--root` = the scratch root. The limits/end_effector copies already sit in the scratch root's `config/robot/`; pass `retime.yaml`/`execution.yaml` explicitly.
3. Run `trajectory_check` and require exit 0.
4. Start the mock stack, apply the scratch scene, and run `execute_trajectory --mode dry --profile moveit_mock`, then `--mode mock --profile moveit_mock`. Pass `--root` (scratch, so G-ELIGIBLE can resolve paths3d), `--scene-config` (scratch scene) and `--execution-config` (repo).
5. Assert the exit codes and the record fields with a small inline `python3 -c`.

If G-COLLISION or start-state gating needs the mock robot at the plan's start state first, use the existing MOT-02/MOT-05.5 mechanism (e.g. `plan_joint_goal` or a mock-mode `test`-purpose move to the start). Document whichever you use. Never relax a gate. Report honestly any gate that fails for a real reason (e.g. the joint_states QoS or start-state tolerance) rather than working around it.
