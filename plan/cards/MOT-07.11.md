# MOT-07.11 — Retimer G-TOOLSPEED is infeasible on real plan_crack_path trace legs: tool-speed dips at joint-space vertices

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07.5",
    "MOT-07.7"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime_core.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime.py",
      "ros2_ws/src/crackvision_motion/test/test_retime.py",
      "ros2_ws/src/crackvision_motion/test/test_retime_core.py",
      "config/motion/retime.yaml",
      "docs/adr/019-time-parameterization.md",
      "docs/INTERFACES.md",
      "src/crackvision/trajectory_check.py",
      "tests/test_trajectory_check.py",
      "tests/test_trajectory_check_path.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/cartesian_plan.json",
    "ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/README.md",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "fixture-retimes",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_retime.py -q -p no:cacheprovider -k real_synthetic_fixture",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "motion-regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      },
      {
        "id": "checker",
        "cmd": "./env.sh pytest tests/test_trajectory_check_path.py tests/test_trajectory_check.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "test_retime.py has a test whose name contains 'real_synthetic_fixture'. It calls retime_plan_doc on ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/cartesian_plan.json with that directory's limits/end_effector copies and the committed config/motion/retime.yaml and config/motion/execution.yaml. It asserts success, a trace v_c ≥ trace.min_tool_speed_m_s, and that the output passes joint_trajectory.check_limits at full limits with execution.yaml's margin (zero violations). The fixture files are not modified.",
      "config/robot/b601_dm_limits.yaml, retime.yaml's limit_fraction.* and sampling.*, G-LIMITS and G-SELFCHECK are unchanged. The fix is never clamping, never a hidden uniform slowdown, and never moving a plan configuration: every plan configuration still appears bit-exact, in order, in the output.",
      "The root cause is fixed by one of these, each documented: (a) handling joint-space vertices in the retimer/time_profile without leaving the plan polyline; (b) a G-TOOLSPEED rule amendment that tolerates bounded, localised dips at interior plan vertices with an explicit bound; or (c) a §16.7 parameter change with a measured justification. Raising speed_tolerance is a last resort and needs a stated rationale for why the result is still a meaningful 'constant tool speed' claim. The choice and the measured evidence (dip ratio vs v_c, joint-space turn angles at trace vertices) are recorded in an ADR-019 addendum and in INTERFACES §16, appended or amended per project convention.",
      "Whatever G-TOOLSPEED now accepts, the independent checker's C-TOOLSPEED (src/crackvision/trajectory_check.py) accepts exactly the same set and is tested on the same dip case. The retimer and checker must not disagree on the fixture's retimed trajectory.",
      "A unit test with a synthetic piecewise-linear joint path that has sharp interior joint-space turns (≥15°) shows the new behaviour. G-TOOLSPEED still refuses a genuine mid-trace speed dip that is not tied to a vertex, and still refuses a segment above the band."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 4,
    "context": 4,
    "consequence": 5,
    "task_class": "motion-algorithm",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.11",
  "title": "Retimer G-TOOLSPEED is infeasible on real plan_crack_path trace legs: tool-speed dips at joint-space vertices",
  "parent": "MOT-07",
  "outcome": "retime_plan, using the committed config/motion/retime.yaml, retimes the committed MOT-07.8 fixture (a verbatim run_cartesian_synthetic.sh plan) with exit 0 and some trace v_c ≥ trace.min_tool_speed_m_s. Joint limits, limit fractions and sampling stay as they are. Any change to G-TOOLSPEED semantics or §16.7 values is recorded in ADR-019 and INTERFACES §16 and mirrored in the independent checker's C-TOOLSPEED.",
  "source": "debugger D0012"
}
```

MOT-07.8's integration test showed that the retimer refuses a real `plan_crack_path` output: the committed fixture `ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/cartesian_plan.json`, a verbatim copy from `run_cartesian_synthetic.sh`. It fails with `trace leg 2: no constant tool speed in [0.005, 0.05] m/s satisfies G-TOOLSPEED` (exit 3). Earlier tests only used smooth synthetic joint paths.

Measured diagnosis (debugger D0012, reproducible with the fixture plus throwaway configs under /tmp):
- The trace leg's joint-step/tool-chord ratio is uniform (5.4–5.7 rad/m), so uneven chord lengths are NOT the cause. The caps in retime.py already normalise them.
- The joint-space polyline turns 1–18° at nearly every interior trace vertex, roughly every 3.5 mm (IK/Cartesian-interpolation jitter).
- The acceleration-limited `time_profile` (1.0 rad/s² × 0.95) slows down at those vertices. The dips grow with v_c:
  - v_c=0.002: no mid-leg violation;
  - v_c=0.005: 2-sample dips to 0.90·v_c (samples 479–480 and 1391–1392 of the trace leg's output range);
  - v_c=0.01: plateaus at 0.90;
  - v_c=0.02: plateaus at 0.59.
- Rule 2 in `_check_toolspeed` (retime.py:162-201) rejects any mid-leg below-band sample. So the only feasible v_c lies below `min_tool_speed_m_s` = 0.005, and the bisection refuses.
- With throwaway configs, speed_tolerance 0.09 and 0.10 still fail; 0.12 passes.

What to do:
1. Pick and justify a fix that keeps the safety envelope: joint limits, limit fractions and sampling are untouched, and there is no clamping. Preferred: handle vertex transitions properly in the profile, or amend G-TOOLSPEED to tolerate bounded, vertex-localised dips with an explicit bound. Loosening speed_tolerance alone is the last resort.
2. Record the decision and the measured evidence in an ADR-019 addendum and amend INTERFACES §16 (G-TOOLSPEED, §16.7 table and C-TOOLSPEED) consistently.
3. Mirror the semantics in `src/crackvision/trajectory_check.py` C-TOOLSPEED, keeping it independent (no crackvision_motion import).
4. Add the `real_synthetic_fixture` test and the sharp-turn synthetic tests named in the criteria.

Do not edit the MOT-07.8 fixture or its scripts (`scripts/ros/test_retime_crosscheck.sh`, `scripts/ros/retime_fault_inject.py`). MOT-07.8 resumes once this card is accepted.
