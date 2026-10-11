# MOT-06.1.R1 — Repair MOT-06.1: Doc's FK error figure contradicts its own cited JSON value by ~300x

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "docs/motion/CARTESIAN_PROBE.md",
      "scripts/ros/probe_cartesian_path.py",
      "scripts/ros/probe_cartesian_path.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#8",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_apply.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py",
    "scripts/ros/_mock_stack.sh",
    "scripts/ros/env_ros.sh",
    "config/robot/end_effector.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      },
      {
        "id": "probe",
        "cmd": "bash scripts/ros/probe_cartesian_path.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "note",
        "cmd": "test -s docs/motion/CARTESIAN_PROBE.md && grep -q 'tool_tip' docs/motion/CARTESIAN_PROBE.md && grep -qi 'fraction' docs/motion/CARTESIAN_PROBE.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Either the script logs a per-point (or first-excluded) max error field that actually equals the claimed ~1e-4 m figure, or the doc states the true JSON value (0.03 m) together with the ramp-segment explanation, so the documented number is reproducible from the JSON.",
      "probe_cartesian_path.py logs the multi-seed / avoid_collisions-false IK attempts (or an equivalent systematic check) to logs/cartesian_probe.json, and the doc's seed/collision-independence claim cites that JSON key.",
      "The fallback-IK-fails path returns a non-zero exit code (e.g. EXIT_RUNTIME) rather than EXIT_OK, or the acceptance-check wording is relaxed to allow documented partial completion.",
      "All acceptance criteria of MOT-06.1 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 3,
    "task_class": "ros-live-probe",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-06.1",
  "findings": [
    "5d48873ff1771bff",
    "c7697d2a04c5275a",
    "caf77e68be19e558"
  ],
  "id": "MOT-06.1.R1",
  "title": "Repair MOT-06.1: Doc's FK error figure contradicts its own cited JSON value by ~300x",
  "parent": "MOT-06",
  "outcome": "Resolve the review findings on MOT-06.1 while every acceptance criterion of MOT-06.1 still holds."
}
```

Repair work generated deterministically from review attempt `00452-MOT-06.1-review` of `MOT-06.1`.
Original card: `plan/cards/MOT-06.1.md` — its acceptance criteria must still hold.

## Finding 1 [major] Doc's FK error figure contradicts its own cited JSON value by ~300x
- Evidence: docs/motion/CARTESIAN_PROBE.md facts table: 'max position error ≈ 1e-4 m (after correcting for the resampling artifact below...)', cited key `queries.fk_check_line_no_roll`. Rerunning the probe: `fk_check_line_no_roll.max_pos_error_m` = 0.030000102703766834 (not ~1e-4). No JSON field contains any value near 1e-4; the smaller number is a hand-derived 'correction' that isn't logged anywhere.
- Consequence: MOT-06.2 would read a table claiming sub-millimeter FK tracking accuracy when the only machine-checkable number in the artifact is 3 cm, which could lead to under-specified tolerance/validation logic for trajectory tracking in the production planner.
- Affected: docs/motion/CARTESIAN_PROBE.md, scripts/ros/probe_cartesian_path.py
- Acceptance condition: Either the script logs a per-point (or first-excluded) max error field that actually equals the claimed ~1e-4 m figure, or the doc states the true JSON value (0.03 m) together with the ramp-segment explanation, so the documented number is reproducible from the JSON.

## Finding 2 [major] Singularity 'seed/collision-independent' claim is not backed by the committed, re-runnable artifact
- Evidence: docs/motion/CARTESIAN_PROBE.md: '...NO_IK_SOLUTION for every seed tried (home-zero plus 10 additional random seeds tried interactively while debugging this card)... reproduces with avoid_collisions: false too.' probe_cartesian_path.py only ever calls compute_ik for this pose with the single HOME_SEED and avoid_collisions defaulting to True (grep confirms no second call with varied seeds or avoid_collisions=False). logs/cartesian_probe.json has no key for any of this extra testing. I independently re-verified the underlying claim is true (13/13 trials across random seeds and avoid_collisions true/false all returned -31), but the delivered artifact itself does not establish it.
- Consequence: The card's single most consequential finding (the literal MOT-06 pose being unreachable due to a genuine kinematic singularity, not an artifact) is not reproducible by re-running the committed probe — a future re-run after an environment change would have no way to detect if this conclusion still holds.
- Affected: docs/motion/CARTESIAN_PROBE.md, scripts/ros/probe_cartesian_path.py
- Acceptance condition: probe_cartesian_path.py logs the multi-seed / avoid_collisions-false IK attempts (or an equivalent systematic check) to logs/cartesian_probe.json, and the doc's seed/collision-independence claim cites that JSON key.

## Finding 3 [minor] Probe can exit 0 on an incomplete run (fallback-IK-fails branch)
- Evidence: probe_cartesian_path.py main(): when the 2° roll fallback IK also fails, the code does `_write(args.out, result); return EXIT_OK` despite steps 3-7 never having run — contradicting the card's own stated contract 'exit 0 iff every probe query completed and the JSON was written.' Not exercised in the actual runs here (fallback succeeded both times), confirmed by inspecting logs/cartesian_probe.json which has all 9 expected query keys.
- Consequence: On a different seed/environment where the fallback also lands on a singularity, the scheduler's acceptance check would pass (exit 0) on a silently truncated probe, masking an incomplete investigation.
- Affected: scripts/ros/probe_cartesian_path.py
- Acceptance condition: The fallback-IK-fails path returns a non-zero exit code (e.g. EXIT_RUNTIME) rather than EXIT_OK, or the acceptance-check wording is relaxed to allow documented partial completion.
