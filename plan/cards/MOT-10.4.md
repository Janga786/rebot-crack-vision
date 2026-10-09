# MOT-10.4 — Operator: place the specimen at the recommended pose and perform the workcell survey (no motion)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "MOT-10.1",
    "MOT-10.3",
    "MOT-04.5"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "config/scene/workcell_survey.yaml"
    ]
  },
  "inputs": [
    "docs/motion/WORKCELL_SURVEY.md",
    "docs/INTERFACES.md#12",
    "config/scene/workcell_survey.template.yaml",
    "config/motion/specimen_placement.yaml",
    "docs/motion/REACHABILITY.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "survey-valid",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion survey_to_scene --validate-survey config/scene/workcell_survey.yaml'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "not-the-fixture",
        "cmd": "bash -c '! cmp -s config/scene/workcell_survey.yaml ros2_ws/src/crackvision_motion/test/fixtures/workcell_survey_example.yaml'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The readings were taken by the operator on the real workcell following WORKCELL_SURVEY.md. Operator name/initials and the date are recorded. Photo evidence exists at the recorded data/workcell_survey/ paths, and those photos are not committed.",
      "The specimen was placed at config/motion/specimen_placement.yaml's recommended centre/yaw (MOT-04.5) before measuring. The survey records the AS-PLACED pose, not the target. Any deliberate deviation is explained in notes.",
      "The operator confirmed that the physical base reference faces match the ones WORKCELL_SURVEY.md derives from base_link.STL, or recorded the discrepancy.",
      "The arm stayed unpowered or at rest throughout. Nothing was jogged and no robot software was driven.",
      "Clearance obstacles inside the survey radius were either all recorded or explicitly declared absent."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 4,
    "task_class": "operator-measurement",
    "local_ok": false
  },
  "hardware": [
    "workcell_measured"
  ],
  "track": "physical",
  "id": "MOT-10.4",
  "parent": "MOT-10",
  "title": "Operator: place the specimen at the recommended pose and perform the workcell survey (no motion)",
  "outcome": "config/scene/workcell_survey.yaml holds the operator's real raw measurements (table, base mounting, as-placed specimen, clearance obstacles, ACM observations), with uncertainties, operator, date, and photo evidence kept under data/workcell_survey/. The survey validates with survey_to_scene."
}
```

Operator task. No code changes.

1. Read docs/motion/WORKCELL_SURVEY.md end to end. Tools: steel rule/tape, engineer's square, calipers, spirit level, phone camera.
2. With the arm powered off or at rest, place the specimen at the centre/yaw recommended in config/motion/specimen_placement.yaml (see docs/motion/REACHABILITY.md's handoff section). Mark or fix it so it cannot shift.
3. Copy config/scene/workcell_survey.template.yaml to config/scene/workcell_survey.yaml and fill in every reading with its uncertainty: base mounting / adapter plate, table extents and level, the four specimen top corners from the base reference faces, specimen thickness at ≥3 places, clearance obstacles, ACM observations, operator, date and notes. Take the photos listed in the procedure and save them under data/workcell_survey/<date>/. Do not commit them.
4. Run the survey-valid check command. If it refuses, fix the reading it names (re-measure rather than adjust numbers), then re-run.
5. Commit only config/scene/workcell_survey.yaml. If an agent transcribes a paper sheet for the operator, the operator must confirm the transcription before commit.

The camera/mount pose is not measured here. It comes from GEOM-05.
