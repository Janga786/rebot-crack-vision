# MOT-07.3 — Retime core: retime config loader, plan polyline assembly, refinement and limit-constrained discrete time profile (pure Python)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07.1"
  ],
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime_core.py",
      "ros2_ws/src/crackvision_motion/test/test_retime_core.py",
      "config/motion/retime.yaml"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#16",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/adr/019-time-parameterization.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_retime_core.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "config-loads",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'cd ros2_ws/src/crackvision_motion && python3 -c \"from crackvision_motion.retime_core import load_retime_config; load_retime_config(\\\"../../../config/motion/retime.yaml\\\")\"'",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "retime.yaml ships the §16.1 defaults with a value_status and sources rationale for every block. The loader is strict and refuses (typed ConfigError) unknown or missing keys and out-of-range values (fractions in (0,1], positive steps/periods, min speed ≤ target, tolerance in (0,1)).",
      "Polyline assembly drops only junction duplicates within CONTINUITY_TOL_RAD and keeps every plan configuration bit-exact. It returns per-leg index spans in the refined polyline. Refinement inserts only linear interpolants and never exceeds max_joint_step_rad.",
      "The profile honours (V), (A) using exactly §11.7's FD acceleration, the (R) rest boundary, the (M) §11.6 FD margin rule, and max_period_s via subdivide-and-reprofile with a bounded iteration count. It also accepts per-segment external speed caps (used by MOT-07.4 for tool speed).",
      "Tests pass joint_trajectory.check_limits (full limits, execution.yaml margin) with zero violations on: a single straight move (duration within 10% of the analytic trapezoid/triangle at f_v/f_a); a polyline with sharp corners; a path approaching and leaving joint2's upper bound 0.0 (home approach); a coarse 0.1 rad transit-like path; and a path whose caps bind. All sample-period, step and rest bounds hold.",
      "Infeasible inputs (e.g. zero-length plan, NaN) raise typed errors. No ROS imports; deterministic output (same input gives an identical t_s)."
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
  "id": "MOT-07.3",
  "title": "Retime core: retime config loader, plan polyline assembly, refinement and limit-constrained discrete time profile (pure Python)",
  "parent": "MOT-07",
  "outcome": "crackvision_motion.retime_core loads config/motion/retime.yaml strictly, turns a validated cartesian plan into the exact refined joint polyline with leg spans, and computes segment durations satisfying §16's FD velocity/acceleration fractions, rest boundary, §11.6 margin rule and sample-period/step bounds — verified against joint_trajectory.check_limits on synthetic paths."
}
```

Implement `retime_core.py` (stdlib + numpy + PyYAML; no rclpy, no *_msgs) and ship `config/motion/retime.yaml` with §16.1's defaults.

Suggested API (refine to §16 names):
- `RetimeConfigError`, `load_retime_config(path) -> dict`;
- `plan_polyline(plan) -> (configs: ndarray[N,6], leg_spans: list[(leg_index, kind, first, last)])`;
- `refine(configs, leg_spans, max_step) -> (configs, leg_spans)`;
- `time_profile(configs, limits, *, f_v, f_a, margin_rad, max_period_s, caps=None) -> (configs, t_s, leg_spans)`. It may subdivide further for the period bound; spans are updated accordingly.

Algorithm, per ADR-019:
- Segment path speeds `u_i` with `h_i = ‖Δq_i‖` (state the norm used), direction `d_i = Δq_i/h_i` and `τ_i = h_i/u_i`.
- **Forward pass:** `u_0` from (V) + (R); then each `u_k` = the largest value ≤ cap_k feasible for (A) at point k and (M), given `u_(k−1)`.
- **Backward pass:** the mirror image from the end with (R), lowering `u_(k−1)` to the largest feasible value given `u_k`.
- Small `u` is always feasible for (A). Find the largest feasible value robustly: coarse scan + bisection, then re-verify the pair.
- After both passes, verify every constraint directly. If any residual violation remains, scale the caps at the offending points by 0.9 and repeat, at most a stated number of times; otherwise raise. Never return an unverified profile.
- Then subdivide every segment with `τ > max_period_s` and re-profile, bounded.

Tests construct inputs directly or with `cartesian_plan` builders. Do not depend on production config shas.
