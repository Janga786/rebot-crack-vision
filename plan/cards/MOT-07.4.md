# MOT-07.4 — Retimer: constant tool speed on trace legs + §11.2 trajectory and crackvision.retime_report/1 assembly with self-check

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07.2",
    "MOT-07.3"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime.py",
      "ros2_ws/src/crackvision_motion/test/test_retime.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#16",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/adr/019-time-parameterization.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/retime_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/tool_fk.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "config/motion/retime.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_retime.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Per trace leg, v_c is the largest value ≤ trace.tool_speed_m_s (bisection to a stated tolerance) such that tool speed is never above v_c(1+tol), every segment outside the ramps is within ±tol of v_c, and each ramp's tool arc length is ≤ max_ramp_m, all measured with the §16 chord definition using tool_fk. A leg with v_c < min_tool_speed_m_s is a refusal naming the leg.",
      "The output passes joint_trajectory.load_trajectory's schema validation and check_limits at full limits with execution.yaml's margin (zero violations); otherwise it refuses. There is no clamping and no hidden uniform slowdown. Explicit velocities/accelerations are absent.",
      "The §16.3 mapping holds: producer, purpose, limits_file {path, sha256}, end_effector_config_sha256, scene_config_sha256 and source.paths3d copied verbatim from the plan. Refusal is tested for a limits-file sha ≠ plan.sources.limits.sha256 and for an end_effector file sha ≠ plan.sources.end_effector.sha256.",
      "Every plan configuration appears bit-exact, in order, among the output points; the retime report's per-leg point ranges, durations, tool-speed stats and totals match the trajectory (tested); and the report validates against §16.4 with a strict loader in this module.",
      "Tests use synthetic plans built with cartesian_plan builders (transit/approach/trace/retract with a straight and a curved Cartesian trace whose configurations come from tool_fk-consistent joint paths), not production config shas."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 5,
    "task_class": "motion-algorithm",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.4",
  "title": "Retimer: constant tool speed on trace legs + §11.2 trajectory and crackvision.retime_report/1 assembly with self-check",
  "parent": "MOT-07",
  "outcome": "crackvision_motion.retime turns a validated cartesian plan into a §11.2 joint_trajectory document plus a §16.4 retime report, holding each trace leg at a single constant tool_tip speed (with bounded ramps) within all joint limits, refusing (typed error) on sha mismatch, too-slow traces or any check_limits violation."
}
```

Implement `retime.py` on top of `retime_core` and `tool_fk`.

Suggested API:
- `RetimeError`;
- `retime_plan_doc(plan, *, limits_path, end_effector_path, urdf_path, retime_cfg, execution_cfg_path, purpose, plan_path) -> (trajectory_doc, report_doc)`;
- `validate_retime_report(doc)`;
- `write_outputs(traj, report, out_path)`: atomic temp + rename, report at `<out>.retime.json`.

Steps:
1. Verify the limits and end_effector shas against `plan.sources`.
2. Build and refine the polyline (`retime_core`).
3. Per trace leg, compute tool positions at the refined points and the per-segment chord length.
4. Bisect `v_c` per trace leg. Each trial runs the full `time_profile` with caps `u_i ≤ v_c·h_i/chord_i` (no cap when chord < 1e-9 m) on that leg's segments, keeping the other trace legs at their current `v_c`, and checks the band/ramp rules. Process trace legs in order, then run one final profile with every leg's chosen `v_c`.
5. Run the `check_limits` self-check.
6. Assemble the §11.2 document: `planning_frame` base_link, `joint_names` joint1..6, points `{t_s, positions}`, `created_utc`.
7. Assemble the §16.4 report.

State the effective-speed note (`speed_scale` multiplies tool speed) in the report's notes if §16 provides a field for it.
