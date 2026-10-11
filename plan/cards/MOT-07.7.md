# MOT-07.7 — Independent trajectory checker, part 2: path fidelity, constant tool speed (crackvision.kinematics FK) and retime-report consistency

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07.6"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/trajectory_check.py",
      "tests/test_trajectory_check_path.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "src/crackvision/trajectory_check.py",
    "src/crackvision/kinematics.py",
    "config/robot/end_effector.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_trajectory_check_path.py tests/test_trajectory_check.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "independent",
        "cmd": "test -f src/crackvision/trajectory_check.py && ! grep -nE '^\\s*(from|import)\\s+(crackvision_motion|rclpy)' src/crackvision/trajectory_check.py",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "kinematics-regression",
        "cmd": "./env.sh pytest tests/test_kinematics.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "C-PATH: each output point is projected onto the plan polyline built from the concatenated legs with junction duplicates dropped. The point's distance must be ≤ the §16 path tolerance; arc-length progress must be non-decreasing; every plan configuration must be matched by an output point. A shortcut that cuts a corner by more than the tolerance is caught (tested), as is a skipped plan configuration.",
      "C-TOOLSPEED: leg membership comes from the projection, never from the report. Per trace leg, tool speed is |Δp_tool|/Δt with p_tool from crackvision.kinematics (load_chain + load_end_effector tool_tip). Each leg must satisfy: never above the declared v_c(1+tol); within ±tol outside ramps; ramps ≤ max_ramp_m; v_c ≤ the configured target. Tests catch a mid-trace speed dip, an overspeed and an over-long ramp.",
      "C-REPORT: the report's per-leg point ranges, durations and v_c must agree with the recomputed values within stated tolerances. A missing report reports C-REPORT as skip (or as §16.5 defines) while the other checks still run.",
      "The checker's FK file shas are recorded in its report, and a mismatch with the trajectory's end_effector_config_sha256 fails C-PROVENANCE."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "verification",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.7",
  "title": "Independent trajectory checker, part 2: path fidelity, constant tool speed (crackvision.kinematics FK) and retime-report consistency",
  "parent": "MOT-07",
  "outcome": "crackvision.trajectory_check additionally verifies C-PATH (every point on the plan polyline, monotone, all plan configurations present), C-TOOLSPEED (trace legs located geometrically, tool_tip speed from crackvision.kinematics within the §16 band/ramp rules) and C-REPORT (the retime report agrees with the recomputed values)."
}
```

Extend `trajectory_check.py` with C-PATH, C-TOOLSPEED and C-REPORT per §16.5, keeping its independence from crackvision_motion.

- **Projection.** For each output point, find the nearest polyline segment and its parameter. To keep this O(N), walk segments monotonically with a window, and fall back to a global search when the window fails.
- **Tests** build plans and trajectories by hand: joint paths whose tool_tip trace is a straight and a curved line, using `crackvision.kinematics`'s own FK/IK-free construction such as joint-space interpolation. Time them with simple constant-speed profiles, then inject the faults listed in the criteria.
