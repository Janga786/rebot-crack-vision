# MOT-04.1.R1 — Repair MOT-04.1: check unit fails after upstream change

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "config/motion/reachability.yaml",
      "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
      "ros2_ws/src/crackvision_motion/test/test_reachability_core.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#6",
    "docs/adr/012-frames-and-conventions.md",
    "docs/motion/ROBOT_MODEL.md#4",
    "config/robot/b601_dm_limits.yaml",
    "docs/TECHNICAL_APPROACH.md#2.5",
    "docs/motion/ROS_WORKSPACE.md",
    "ros2_ws/src/crackvision_motion/setup.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_reachability_core.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "config-valid",
        "cmd": "bash scripts/ros/env_ros.sh python3 -c \"import sys; sys.path.insert(0,'ros2_ws/src/crackvision_motion'); from crackvision_motion.reachability_core import load_config; load_config('config/motion/reachability.yaml')\"",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "no-ros-import",
        "cmd": "bash scripts/ros/env_ros.sh python3 -c \"import sys; sys.path.insert(0,'ros2_ws/src/crackvision_motion'); import crackvision_motion.reachability_core; assert not any(m.split('.')[0] in ('rclpy','moveit_msgs','geometry_msgs') for m in sys.modules)\"",
        "timeout_s": 120,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "`bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_reachability_core.py -q -p no:cacheprovider` exits 0",
      "All acceptance criteria of MOT-04.1 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 2,
    "consequence": 3,
    "task_class": "python-geometry-lib",
    "local_ok": false
  },
  "priority": 99,
  "repairs": "MOT-04.1",
  "findings": [
    "610d6dba73918410"
  ],
  "id": "MOT-04.1.R1",
  "title": "Repair MOT-04.1: check unit fails after upstream change",
  "parent": "MOT-04",
  "outcome": "Resolve the review findings on MOT-04.1 while every acceptance criterion of MOT-04.1 still holds."
}
```

Repair work generated deterministically from review attempt `recheck-MOT-04.1` of `MOT-04.1`.
Original card: `plan/cards/MOT-04.1.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] check unit fails after upstream change
- Evidence: `bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_reachability_core.py -q -p no:cacheprovider` exit 1: .........................F                                               [100%]
=================================== FAILURES ===================================
__________________________ test_load_committed_config __________________________

    def test_load_committed_config():
        cfg = load_config(REPO_ROOT / "config" / "motion" / "reachability.yaml")
        assert cfg["frame"] == "base_link"
        assert cfg["group"] == "arm"
        assert cfg["ik_link"] == "gripper_tcp"
        assert cfg["value_status"] == "nominal"
        assert cfg["boresight"]["provenance"] == "prior_evidence"
>       assert len(axis_values(cfg["grid"]["x_m"])) == 19
E       AssertionError: assert 16 == 19
E        +  where 16 = len([0.05, 0.08, 0.11, 0.14, 0.16999999999999998, 0.2, .
…[426 chars clipped]
- Consequence: previously accepted behaviour regressed
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py, ros2_ws/src/crackvision_motion/test/test_reachability_core.py, config/motion/reachability.yaml
- Acceptance condition: `bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_reachability_core.py -q -p no:cacheprovider` exits 0
