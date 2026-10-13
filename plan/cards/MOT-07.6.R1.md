# MOT-07.6.R1 — Repair MOT-07.6: C-PROVENANCE does not check trajectory start/end against the plan's fi

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/trajectory_check.py",
      "tests/test_trajectory_check.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml",
    "src/crackvision/logging_setup.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_trajectory_check.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "independent",
        "cmd": "test -f src/crackvision/trajectory_check.py && ! grep -nE '^\\s*(from|import)\\s+(crackvision_motion|rclpy)' src/crackvision/trajectory_check.py",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "cli-help",
        "cmd": "./env.sh python -m crackvision.trajectory_check --help",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "C-PROVENANCE fails, with detail naming the mismatch, when the trajectory's points[0].positions differs from the first configuration of the plan's lowest-leg_index leg, or its points[-1].positions differs from the plan's last leg's last configuration. A test builds such a trajectory and asserts C-PROVENANCE 'fail', and the clean case still passes.",
      "validate_trajectory_schema (and the plan leg positions check) reject non-finite t_s, positions, velocities and accelerations (math.isfinite), so C-SCHEMA is 'fail' and the CLI exits 3. A test with a NaN position asserts that outcome.",
      "All acceptance criteria of MOT-07.6 still hold at the new revision"
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
  "priority": 95,
  "repairs": "MOT-07.6",
  "findings": [
    "0ca556318b2c06b3",
    "0e3b3c43261a55b3"
  ],
  "id": "MOT-07.6.R1",
  "title": "Repair MOT-07.6: C-PROVENANCE does not check trajectory start/end against the plan's fi",
  "parent": "MOT-07",
  "outcome": "Resolve the review findings on MOT-07.6 while every acceptance criterion of MOT-07.6 still holds."
}
```

Repair work generated deterministically from review attempt `00865-MOT-07.6-review` of `MOT-07.6`.
Original card: `plan/cards/MOT-07.6.md` — its acceptance criteria must still hold.

## Finding 1 [major] C-PROVENANCE does not check trajectory start/end against the plan's first/last configuration
- Evidence: check_provenance (src/crackvision/trajectory_check.py:242-306) only compares limits_file.sha256, end_effector_config_sha256, scene_config_sha256 and source.paths3d. grep finds no comparison of points[0]/points[-1] positions with plan legs[].positions. Probe /tmp/mot076probe/probe.py: with joint1 offset by +0.5 rad on every point, while the plan's first and last configurations are [0]*6, the result was C-PROVENANCE 'pass' and every check 'pass'.
- Consequence: The card's C-PROVENANCE acceptance criterion is not met. A trajectory that does not start at the plan's start state, or does not end at its end state, is certified as consistent with the plan until MOT-07.7's C-PATH exists. Even then, this card's criterion stays unmet.
- Affected: src/crackvision/trajectory_check.py, tests/test_trajectory_check.py
- Acceptance condition: C-PROVENANCE fails, with detail naming the mismatch, when the trajectory's points[0].positions differs from the first configuration of the plan's lowest-leg_index leg, or its points[-1].positions differs from the plan's last leg's last configuration. A test builds such a trajectory and asserts C-PROVENANCE 'fail', and the clean case still passes.

## Finding 2 [major] Non-finite positions (NaN/Infinity) pass C-SCHEMA and every other check; CLI exits 0
- Evidence: _is_number accepts float('nan'), and every limit comparison (abs(vi) > V, abs(ai) > A, step > max_step, abs(v0) > bound0) is False for NaN. Probe: with points[2].positions[0] = NaN, all six checks returned 'pass'. With the real repo configs, `./env.sh python -m crackvision.trajectory_check --trajectory /tmp/mot076probe/nan.json --plan /tmp/mot076probe/plan.json -q` printed 'nan exit=0', and logs/trajectory_check_latest.json shows status ok with 6 passes.
- Consequence: The independent safety verifier certifies a trajectory whose commanded joint position is undefined. NaN is not a valid JSON number (RFC 8259), so it cannot be a valid §11.2 positions[6] entry. Downstream gating (MOT-08, execution) would rely on a false pass.
- Affected: src/crackvision/trajectory_check.py, tests/test_trajectory_check.py
- Acceptance condition: validate_trajectory_schema (and the plan leg positions check) reject non-finite t_s, positions, velocities and accelerations (math.isfinite), so C-SCHEMA is 'fail' and the CLI exits 3. A test with a NaN position asserts that outcome.
