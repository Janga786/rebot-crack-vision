# MOT-07.8 — Offline cross-validation: retimer vs independent checker vs G-LIMITS on a committed synthetic plan, with fault injection

```json card
{
  "kind": "integration",
  "depends_on": [
    "MOT-07.5",
    "MOT-07.7"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "scripts/ros/test_retime_crosscheck.sh",
      "scripts/ros/retime_fault_inject.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/**"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#16",
    "docs/motion/CARTESIAN_PLANNING.md",
    "scripts/ros/run_cartesian_synthetic.sh",
    "ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
    "src/crackvision/trajectory_check.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "crosscheck",
        "cmd": "bash scripts/ros/test_retime_crosscheck.sh",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The fixture is a verbatim copy of a real plan_crack_path output from scripts/ros/run_cartesian_synthetic.sh, plus copies of the end_effector.yaml and b601_dm_limits.yaml whose shas the plan records. Provenance is in a README in the fixture dir. Nothing is hand-edited, and the total fixture size is under 200 KB.",
      "Clean path: retime_plan exits 0, trajectory_check exits 0 with every check pass, and execution_gate's offline evaluation in mock mode reports G-TRAJ and G-LIMITS pass. Headline numbers are printed: duration, points, per-trace-leg v_c, peak utilisation.",
      "Fault injection writes mutated copies of the trajectory, one fault each: one segment 20% too fast, an acceleration spike, a dropped plan configuration, a corner cut, a mid-trace tool-speed dip, a wrong limits sha, a t_s gap > max_period_s. The checker exits 3 on every one, and G-LIMITS (joint_trajectory.check_limits) independently flags the velocity/acceleration faults. A fault that goes uncaught fails the script.",
      "The script uses scratch space under logs/, writes nothing under data/, needs no running ROS graph, and exits non-zero with a clear message on any failure."
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
  "id": "MOT-07.8",
  "title": "Offline cross-validation: retimer vs independent checker vs G-LIMITS on a committed synthetic plan, with fault injection",
  "parent": "MOT-07",
  "outcome": "scripts/ros/test_retime_crosscheck.sh retimes a committed MOT-06.10 synthetic plan (system python), passes it through the independent checker (conda) and execution_gate's offline gates, and shows that injected faults are caught by the checker and, for limit faults, also by G-LIMITS — all offline, with no ROS graph."
}
```

1. Produce the fixture: run `bash scripts/ros/run_cartesian_synthetic.sh` (headless mock stack, about 3 min) and copy its `cartesian_plan.json` into `ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/`. Also copy `config/robot/end_effector.yaml` and `config/robot/b601_dm_limits.yaml`, but only if their shas equal `plan.sources.*.sha256`; otherwise regenerate the plan. Write a README recording the run stamp and the shas.
2. `test_retime_crosscheck.sh`:
   - run `retime_plan` under `scripts/ros/env_ros.sh` with explicit `--limits`/`--end-effector-config` pointing at the fixture copies and `--out` under `logs/retime_crosscheck/<stamp>/`;
   - run `./env.sh python -m crackvision.trajectory_check` with the same files;
   - run the execution_gate offline evaluation (system python3, as MOT-05.3 tests do) for G-TRAJ/G-LIMITS;
   - then for each fault, `retime_fault_inject.py` (stdlib only) writes a mutated copy, and the script asserts the expected detections.
3. Do not modify retimer or checker code here. A disagreement between them is a finding: report it and fail the script.
