# MOT-10.2 — Workcell survey core: validate a crackvision.workcell_survey/1 record and derive measured scene objects (pure Python)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-10.1"
  ],
  "requirements": [
    "REQ-MOT-1"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/survey_core.py",
      "ros2_ws/src/crackvision_motion/test/test_survey_core.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/workcell_survey_example.yaml"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#12",
    "docs/motion/WORKCELL_SURVEY.md",
    "config/scene/workcell_survey.template.yaml",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py",
    "docs/motion/SCENE.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_survey_core.py -q -p no:cacheprovider",
        "timeout_s": 180,
        "expect_exit": 0
      },
      {
        "id": "no-ros-import",
        "cmd": "bash -c '! grep -nE \"^\\s*(import|from) (rclpy|moveit_msgs|geometry_msgs)\" ros2_ws/src/crackvision_motion/crackvision_motion/survey_core.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "scene-core-still-green",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_scene_core.py -q -p no:cacheprovider",
        "timeout_s": 180,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "survey_core is pure Python, needs nothing beyond yaml/stdlib/numpy, and imports nothing from ROS. Like scene_core it uses exact key sets, never invents a value, and raises a typed SurveyError that names the offending field.",
      "Derivations follow INTERFACES §12 exactly. Corners → centre/yaw/footprint, with the rectangularity check. Thickness → specimen box, bottom on the table top. Base mounting → table top z (0 when bolted directly). Obstacles → boxes. Each derived quantity also carries a propagated 1-sigma uncertainty.",
      "Output objects/ACM entries pass scene_core.load_config's validation when written to YAML. Every one is value_status measured, and its source cites the survey file's sha256 and the specific readings used.",
      "Tests cover: the worked fixture gives hand-computed expected poses within 1e-9; each §12 refusal rule (non-rectangular corners, specimen off the table, obstacle in the base keep-out, missing uncertainty, unknown key, wrong schema); a yawed specimen; adapter-plate offset; and that the template file (all nulls) is refused as incomplete.",
      "The fixture is clearly labelled synthetic/example and is never mistaken for a real survey."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "python-lib+tests",
    "local_ok": false
  },
  "id": "MOT-10.2",
  "parent": "MOT-10",
  "title": "Workcell survey core: validate a crackvision.workcell_survey/1 record and derive measured scene objects (pure Python)",
  "outcome": "crackvision_motion/survey_core.py loads and validates a survey record, applies §12's derivation and refusal rules, and returns crackvision.scene_config/1 objects plus ACM entries, all tagged measured with survey-sha sources. Unit tests cover it against a worked fixture."
}
```

Implement §12 from MOT-10.1 as a pure-Python module next to scene_core.py, in the same style: module docstring naming the contract, _check_keys/_require helpers, typed errors.

API (names may be refined, but document them in the module docstring):
- `load_survey(path) -> dict`: validated record.
- `survey_sha256(path) -> str`.
- `derive_scene(survey, survey_sha) -> {'objects': [...], 'allowed_collisions': [...], 'derived': {...uncertainties, specimen centre/yaw/top_z...}}`.
- `SurveyError`.

The fixture is a complete, plausible example survey. Use a specimen near the current nominal (0.30, 0.0), slightly yawed, with one obstacle box. Hand-compute the expected outputs in test comments.

Do not modify scene_core.py or the production scene. Run the tests with the system python3 via scripts/ros/env_ros.sh, the same way scripts/ros/test_scene.sh runs test_scene_core.py.
