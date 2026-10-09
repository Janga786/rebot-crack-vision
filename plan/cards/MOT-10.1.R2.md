# MOT-10.1.R2 — Repair MOT-10.1: WORKCELL_SURVEY.md's documented validate/convert commands do not match

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "config/scene/workcell_survey.template.yaml",
      "docs/INTERFACES.md",
      "docs/motion/WORKCELL_SURVEY.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#7",
    "docs/INTERFACES.md#8",
    "docs/INTERFACES.md#11",
    "docs/motion/SCENE.md",
    "docs/motion/ROBOT_MODEL.md",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "config/scene/scene.yaml",
    "config/motion/reachability.yaml",
    "config/motion/specimen_placement.yaml",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "sections",
        "cmd": "bash -c 'test $(grep -c \"^## 11\\. \" docs/INTERFACES.md) -eq 1 && test $(grep -c \"^## 12\\. \" docs/INTERFACES.md) -eq 1 && grep -qF crackvision.workcell_survey/1 docs/INTERFACES.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "doc",
        "cmd": "bash -c 'test -s docs/motion/WORKCELL_SURVEY.md && grep -qF crackvision.workcell_survey/1 docs/motion/WORKCELL_SURVEY.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "template-yaml",
        "cmd": "./env.sh python -c \"import yaml,sys; d=yaml.safe_load(open('config/scene/workcell_survey.template.yaml')); sys.exit(0 if d.get('schema')=='crackvision.workcell_survey/1' else 1)\"",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "docs/motion/WORKCELL_SURVEY.md §4 is updated to give the actual survey_to_scene invocation: `ros2 run crackvision_motion survey_to_scene --validate-survey PATH` (via scripts/ros/env_ros.sh) for validate-only, and `--survey PATH --out config/scene/scene.yaml` for conversion, matching docs/motion/SCENE.md §8 and the tool's real flags.",
      "All acceptance criteria of MOT-10.1 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4,
    "task_class": "decision+contract",
    "local_ok": false
  },
  "priority": 95,
  "repairs": "MOT-10.1",
  "findings": [
    "69e47917431c5906"
  ],
  "id": "MOT-10.1.R2",
  "title": "Repair MOT-10.1: WORKCELL_SURVEY.md's documented validate/convert commands do not match",
  "parent": "MOT-10",
  "outcome": "Resolve the review findings on MOT-10.1 while every acceptance criterion of MOT-10.1 still holds."
}
```

Repair work generated deterministically from review attempt `00347-MOT-10.1-rereview` of `MOT-10.1`.
Original card: `plan/cards/MOT-10.1.md` — its acceptance criteria must still hold.

## Finding 1 [major] WORKCELL_SURVEY.md's documented validate/convert commands do not match the real survey_to_scene CLI
- Evidence: docs/motion/WORKCELL_SURVEY.md §4 says: `./env.sh python -m crackvision.survey_to_scene --survey data/workcell_survey/survey_<date>.yaml --dry-run` to validate and the same command without `--dry-run` to convert. I confirmed `./env.sh python -c "import crackvision.survey_to_scene"` raises `ModuleNotFoundError: No module named 'crackvision.survey_to_scene'`. The actual tool, per the now-updated docs/motion/SCENE.md §8 (committed by MOT-10.3, the change driving this re-review) and ros2_ws/src/crackvision_motion/crackvision_motion/survey_to_scene.py, is `ros2 run crackvision_motion survey_to_scene`, invoked via `scripts/ros/env_ros.sh` (not `./env.sh`), with `--validate-survey PATH` for validate-only and `--survey PATH --out PATH` for conversion (`--out` has no default and is required). Separately, cli_common.py:192-193 shows `--dry-run` for this tool skips calling `_run` entirely and just logs 'skipping' with exit 0 -- it performs no validation at all, directly contradicting WORKCELL_SURVEY.md's claim that `--survey PATH --dry-run` 'validates the file against crackvision.workcell_survey/1 and reports what it would refuse'.
- Consequence: An operator following WORKCELL_SURVEY.md's §4 instructions verbatim gets a ModuleNotFoundError trying to validate or convert their survey, or -- if they otherwise invoke the real tool with `--dry-run` instead of `--validate-survey` believing it checks their file -- gets a silent exit 0 that validated nothing, creating false confidence that a possibly-invalid survey (bad rectangularity, missing readings, etc.) is fine before it feeds the safety-relevant collision scene.
- Affected: docs/motion/WORKCELL_SURVEY.md
- Acceptance condition: docs/motion/WORKCELL_SURVEY.md §4 is updated to give the actual survey_to_scene invocation: `ros2 run crackvision_motion survey_to_scene --validate-survey PATH` (via scripts/ros/env_ros.sh) for validate-only, and `--survey PATH --out config/scene/scene.yaml` for conversion, matching docs/motion/SCENE.md §8 and the tool's real flags.
