# MOT-07.8.R1 — Repair MOT-07.8: toolspeed_dip fault is not isolated: it also breaks C-SAMPLING and C-L

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-2",
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/**",
      "scripts/ros/retime_fault_inject.py",
      "scripts/ros/test_retime_crosscheck.sh"
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
      "On the committed fixture, fault_toolspeed_dip.json's trajectory_check report has C-TOOLSPEED=fail while C-LIMITS, C-SAMPLING and C-PATH pass. The dip is placed outside any MOT-07.11 vertex-tied-dip allowance (e.g. a smooth multi-segment slowdown whose below-band run exceeds trace.max_ramp_m or 3b's share). test_retime_crosscheck.sh asserts the expected failing check id(s) for each fault, at least C-TOOLSPEED for toolspeed_dip, C-SAMPLING for t_s_gap, C-PROVENANCE for wrong_limits_sha, C-PATH for dropped_config/corner_cut and C-LIMITS for the limit faults, and fails if the expected check did not fail.",
      "All acceptance criteria of MOT-07.8 still hold at the new revision"
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
  "priority": 95,
  "repairs": "MOT-07.8",
  "findings": [
    "fc26a0c71103bef7"
  ],
  "id": "MOT-07.8.R1",
  "title": "Repair MOT-07.8: toolspeed_dip fault is not isolated: it also breaks C-SAMPLING and C-L",
  "parent": "MOT-07",
  "outcome": "Resolve the review findings on MOT-07.8 while every acceptance criterion of MOT-07.8 still holds."
}
```

Repair work generated deterministically from review attempt `00943-MOT-07.8-review` of `MOT-07.8`.
Original card: `plan/cards/MOT-07.8.md` — its acceptance criteria must still hold.

## Finding 1 [major] toolspeed_dip fault is not isolated: it also breaks C-SAMPLING and C-LIMITS, so it never shows that C-TOOLSPEED alone catches a dip
- Evidence: scripts/ros/retime_fault_inject.py fault_toolspeed_dip: `_shift_tail(points, mid + 1, 19.0 * tau)` (one segment ×20). fault_t_s_gap uses `seg = len(points) // 2`, the same segment 6886. The run log at logs/retime_crosscheck/20261013T104522Z/faults/fault_toolspeed_dip.check.json shows C-LIMITS fail ('joint3 point 6886 FD acceleration -1.720301 exceeds limit 1.0'), C-SAMPLING fail ('segment 6886 duration 0.050362s exceeds max_period_s 0.02') and C-TOOLSPEED fail. test_retime_crosscheck.sh only checks `if [ "$check_rc" -ne 3 ]`. Probe: a smooth dip (durations ×(1+0.5·sin²) over 600 segments centred on 6886, max tau 0.0153 s) gives C-LIMITS/C-SAMPLING/C-PATH pass and only C-TOOLSPEED fail, so an isolated dip is feasible.
- Consequence: The card asks for one fault per copy, and the debugger note requires the mid-trace dip to be caught under MOT-07.11's C-TOOLSPEED semantics. As written, a regression that disabled or loosened C-TOOLSPEED (or G-TOOLSPEED rule 2/3) would leave the script green: the dip copy is still rejected for its period gap and acceleration spike. The cross-check gives false assurance for exactly the rule MOT-07.11 just changed.
- Affected: scripts/ros/retime_fault_inject.py, scripts/ros/test_retime_crosscheck.sh
- Acceptance condition: On the committed fixture, fault_toolspeed_dip.json's trajectory_check report has C-TOOLSPEED=fail while C-LIMITS, C-SAMPLING and C-PATH pass. The dip is placed outside any MOT-07.11 vertex-tied-dip allowance (e.g. a smooth multi-segment slowdown whose below-band run exceeds trace.max_ramp_m or 3b's share). test_retime_crosscheck.sh asserts the expected failing check id(s) for each fault, at least C-TOOLSPEED for toolspeed_dip, C-SAMPLING for t_s_gap, C-PROVENANCE for wrong_limits_sha, C-PATH for dropped_config/corner_cut and C-LIMITS for the limit faults, and fails if the expected check did not fail.
