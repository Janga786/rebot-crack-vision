# MOT-06.11.R1 — Repair MOT-06.11: §15's 'nothing changes §13.9–§13.14' disclaimer contradicts its own ov

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "config/motion/cartesian_planning.yaml",
      "docs/INTERFACES.md",
      "docs/adr/017-cartesian-crack-path-planning.md",
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full/**",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_core.py",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py",
      "ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py",
      "scripts/ros/test_plan_trace_full.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "docs/motion/CARTESIAN_PROBE.md",
    "logs/cartesian_probe.json",
    "ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
    "config/motion/cartesian_planning.yaml",
    "scripts/ros/_mock_stack.sh",
    "config/motion/specimen_placement.yaml"
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
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py ros2_ws/src/crackvision_motion/test/test_cartesian_core.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "live-trace",
        "cmd": "bash scripts/ros/test_plan_trace_full.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§15's intro disclaimer is corrected to list §13.11(.2) as amended for the fraction==1.0 case (or §13.11.2's own text is cross-referenced to §15.2), so the two statements about scope no longer contradict each other.",
      "All acceptance criteria of MOT-06.11 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4,
    "task_class": "ros-node",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-06.11",
  "findings": [
    "1d9f25e8381aed90"
  ],
  "id": "MOT-06.11.R1",
  "title": "Repair MOT-06.11: §15's 'nothing changes §13.9–§13.14' disclaimer contradicts its own ov",
  "parent": "MOT-06",
  "outcome": "Resolve the review findings on MOT-06.11 while every acceptance criterion of MOT-06.11 still holds."
}
```

Repair work generated deterministically from review attempt `00642-MOT-06.2-rereview` of `MOT-06.11`.
Original card: `plan/cards/MOT-06.11.md` — its acceptance criteria must still hold.

## Finding 1 [major] §15's 'nothing changes §13.9–§13.14' disclaimer contradicts its own override of §13.11.2's waypoint_index_map rule
- Evidence: docs/INTERFACES.md:2348-2350 states 'Nothing here changes §13.1–§13.4, §13.6, §13.9–§13.14 or §13.17.' §13.11.2 (line 2069) defines `waypoint_index_map` as matched via '§13.7's matching rule', with `null` for an unmatched configuration. §15.2's final paragraph (line 2388) then states: for a fraction==1.0 trace leg, `waypoint_index_map` is instead each configuration's nearest requested-waypoint index under the §15.2 projection rule and is 'never null' — directly overriding the §13.11.2-defined semantics that the disclaimer two paragraphs earlier claims is untouched.
- Consequence: A reader who trusts the disclaimer and consults §13.11.2 in isolation (e.g. to implement or validate the plan-file schema) will apply the original §13.7-pointer-walk/null-bearing rule even to fraction==1.0 trace legs, producing output that disagrees with §15.2's explicit, more specific rule — the two normative statements in the same document directly conflict about which sections §15 amends.
- Affected: docs/INTERFACES.md
- Acceptance condition: §15's intro disclaimer is corrected to list §13.11(.2) as amended for the fraction==1.0 case (or §13.11.2's own text is cross-referenced to §15.2), so the two statements about scope no longer contradict each other.
