# MOT-07.5 — retime_plan CLI (ROS-free console script): plan → data/trajectories/{case}_trajectory.json + .retime.json

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07.4"
  ],
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/retime.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "ros2_ws/src/crackvision_motion/setup.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "entry-point",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; ros2 run crackvision_motion retime_plan --help'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "ros-free",
        "cmd": "test -f ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py && ! grep -nE '^\\s*(from|import)\\s+(rclpy|[a-z_]+_msgs)' ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py ros2_ws/src/crackvision_motion/crackvision_motion/retime.py ros2_ws/src/crackvision_motion/crackvision_motion/retime_core.py",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Flags, defaults and output path are exactly §16.6. Config defaults resolve under --root: retime.yaml, b601_dm_limits.yaml, end_effector.yaml, execution.yaml and the presentation/sim URDF. Output defaults to data/trajectories/{case_id}_trajectory.json plus the .retime.json sidecar.",
      "Exit codes per §0.2/§16.6, each tested via main(argv): 0 written; 2 usage/config error (unknown retime.yaml key, bad flag); 3 refusal (invalid plan, sha mismatch, trace below min speed, self-check violation); 1 unexpected runtime error. Refusals write no trajectory file.",
      "--dry-run validates the inputs and prints the per-leg summary (kind, configurations, refined points, provisional duration) without writing any file, and exits 0 regardless.",
      "The §0.4 logs (logs/retime_plan_*.log/.json) are written as other crackvision_motion CLIs do via cli_common; setup.py gains only the retime_plan console_script; the existing test suite still passes."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 4,
    "task_class": "python-cli",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.5",
  "title": "retime_plan CLI (ROS-free console script): plan → data/trajectories/{case}_trajectory.json + .retime.json",
  "parent": "MOT-07",
  "outcome": "`ros2 run crackvision_motion retime_plan` (and `python3 -m crackvision_motion.retime_plan`) retimes a cartesian plan per §16.6 with §0 flags, default paths under --root, --dry-run, atomic writes and exit codes 0/1/2/3, without importing rclpy."
}
```

Add `retime_plan.py` with `main(argv=None)` and register `retime_plan = crackvision_motion.retime_plan:main` in `setup.py` console_scripts. Do not reorder the existing entries.

- Reuse `cli_common` for the §0.3 flags, logging and summary JSON. Never import rclpy: this is an offline file transform even though it ships in the ROS package.
- `--root` defaults to `$CRACKVISION_ROOT` (exported by env_ros.sh) or the repo root, using the same resolution as the other CLIs.
- Tests call `main()` with a synthetic plan written to tmp_path, plus tmp copies of the limits/end_effector/URDF files whose shas the synthetic plan records.
- Rebuild with `scripts/ros/build_ws.sh`.
