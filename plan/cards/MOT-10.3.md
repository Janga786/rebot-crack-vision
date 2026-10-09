# MOT-10.3 — survey_to_scene CLI: survey → scene.yaml (+ verify configs at the measured pose), drift check; decouple scene tests from the production scene's nominal state

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-10.2",
    "MOT-04.3"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-OPS-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/survey_to_scene.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/test_scene_core.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/scene_nominal_example.yaml",
      "docs/motion/SCENE.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#7",
    "docs/INTERFACES.md#12",
    "docs/motion/WORKCELL_SURVEY.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/survey_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/placement.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py",
    "ros2_ws/src/crackvision_motion/test/fixtures/workcell_survey_example.yaml",
    "config/motion/reachability.yaml"
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
        "id": "validate-fixture",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion survey_to_scene --validate-survey ros2_ws/src/crackvision_motion/test/fixtures/workcell_survey_example.yaml'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "template-refused",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion survey_to_scene --validate-survey config/scene/workcell_survey.template.yaml'",
        "timeout_s": 120,
        "expect_exit": 2
      },
      {
        "id": "convert-and-check",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && f=ros2_ws/src/crackvision_motion/test/fixtures/workcell_survey_example.yaml && ros2 run crackvision_motion survey_to_scene --survey $f --out logs/mot10_3_scene.yaml --emit-verify-config logs/mot10_3_pv.yaml --emit-view-config logs/mot10_3_vv.yaml && ros2 run crackvision_motion survey_to_scene --survey $f --check-scene logs/mot10_3_scene.yaml && python3 -c \"import sys; sys.path.insert(0,\\\"ros2_ws/src/crackvision_motion\\\"); from crackvision_motion.scene_core import load_config, assert_commissioning_ready; assert_commissioning_ready(load_config(\\\"logs/mot10_3_scene.yaml\\\"))\"'",
        "timeout_s": 180,
        "expect_exit": 0
      },
      {
        "id": "drift-detected",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion survey_to_scene --survey ros2_ws/src/crackvision_motion/test/fixtures/workcell_survey_example.yaml --check-scene config/scene/scene.yaml'",
        "timeout_s": 120,
        "expect_exit": 1
      },
      {
        "id": "dry-run-writes-nothing",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && rm -f logs/mot10_3_dry.yaml && ros2 run crackvision_motion survey_to_scene --dry-run --survey ros2_ws/src/crackvision_motion/test/fixtures/workcell_survey_example.yaml --out logs/mot10_3_dry.yaml && test ! -e logs/mot10_3_dry.yaml'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_scene_core.py ros2_ws/src/crackvision_motion/test/test_survey_core.py -q -p no:cacheprovider",
        "timeout_s": 180,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The CLI follows INTERFACES §0 through cli_common (common flags, exit codes 0/1/2/3, logs). Modes: --validate-survey (0 valid / 2 invalid); convert (--survey --out); --check-scene PATH (0 identical to regeneration / 1 drift, with a printed diff); --dry-run writes nothing. It never overwrites config/scene/scene.yaml unless that path is given explicitly as --out.",
      "The generated scene file has a header comment naming the survey path, its sha256 and the generator. Generation is byte-deterministic, so --check-scene is meaningful. The scene's frame/schema are unchanged, and it carries the measured table, specimen, obstacle objects and ACM entries.",
      "--emit-verify-config / --emit-view-config build placement-shaped inputs from the MEASURED specimen pose and top z, and reuse placement.verification_config / view_verification_config. The verification logic is not reimplemented. The emitted files are valid reachability configs that keep reachability.yaml's ik_link, standoffs, orientation, IK tolerance and collision settings.",
      "test_scene_core.py: the commissioning-refusal test now uses a committed nominal fixture instead of the production scene. The production-table test asserts that the top face is at z = -(adapter plate) within the §12 tolerance rather than exactly 0, or else reads the expected value from the survey contract. Every existing assertion about schema validation, the absence of a world camera object, and single-straggler refusal is kept.",
      "SCENE.md gains a section that documents survey_to_scene and the measured-scene workflow (WORKCELL_SURVEY.md → survey file → survey_to_scene → scene.yaml → test_scene.sh). The §4 'today's values' table is left alone until MOT-10.5.",
      "Nothing here touches action servers or execution. The tool is file-in/file-out only."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 4,
    "consequence": 3,
    "task_class": "ros-pkg-cli+tests",
    "local_ok": false
  },
  "id": "MOT-10.3",
  "parent": "MOT-10",
  "title": "survey_to_scene CLI: survey → scene.yaml (+ verify configs at the measured pose), drift check; decouple scene tests from the production scene's nominal state",
  "outcome": "`ros2 run crackvision_motion survey_to_scene` validates a survey and writes a measured crackvision.scene_config/1 file. It can also emit placement/view verification reachability configs centred on the measured specimen pose, and it can check that a committed scene.yaml still matches its survey. test_scene_core.py no longer depends on the production scene being nominal."
}
```

Add the console script `survey_to_scene = crackvision_motion.survey_to_scene:main` to setup.py. Structure it like recommend_placement.py: argparse plus cli_common.add_common_args, and run_cli for exit-code mapping. The module must stay importable without rclpy, since it is file-in/file-out.

Verify configs: look at how recommend_placement calls placement.verification_config(placement, cfg) and view_verification_config(...). Build the `placement` mapping (center_xy_m, yaw_rad, footprint_m, tolerance_m, surface z) from survey_core.derive_scene's measured specimen. Take tolerance_m from the measured corner uncertainty or from reachability.yaml's placement tolerance, whichever §12 specifies. cfg = config/motion/reachability.yaml (or --reachability-config).

Tests: move the 'production scene is nominal → refuse' expectation onto a new fixture, ros2_ws/src/crackvision_motion/test/fixtures/scene_nominal_example.yaml, which is a copy of today's nominal scene. This lets MOT-10.5 flip the production scene to measured without breaking scripts/ros/test_scene.sh.

logs/ outputs are scratch. Never commit them.
