# MOT-04.3.R1 — Repair MOT-04.3: check cli-smoke fails after upstream change

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "ros2_ws/src/crackvision_motion/crackvision_motion/placement.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json",
      "ros2_ws/src/crackvision_motion/test/test_placement.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#7",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_map.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "config/motion/reachability.yaml",
    "ros2_ws/src/crackvision_motion/setup.py"
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
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_placement.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "cli-smoke",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion recommend_placement --map ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json --reachability-config config/motion/reachability.yaml --out logs/mot04_check_placement.yaml --emit-verify-config logs/mot04_check_verify.yaml && ros2 run crackvision_motion recommend_placement --validate-placement logs/mot04_check_placement.yaml'",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "mock-plan-regression",
        "cmd": "bash scripts/ros/test_mock_plan.sh",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "`bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion recommend_placement --map ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json --reachability-config config/motion/reachability.yaml --out logs/mot04_check_placement.yaml --emit-verify-config logs/mot04_check_verify.yaml && ros2 run crackvision_motion recommend_placement --validate-placement logs/mot04_check_placement.yaml'` exits 0",
      "All acceptance criteria of MOT-04.3 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "python-algorithm",
    "local_ok": false
  },
  "priority": 99,
  "repairs": "MOT-04.3",
  "findings": [
    "72c2e769bc2b27fa"
  ],
  "id": "MOT-04.3.R1",
  "title": "Repair MOT-04.3: check cli-smoke fails after upstream change",
  "parent": "MOT-04",
  "outcome": "Resolve the review findings on MOT-04.3 while every acceptance criterion of MOT-04.3 still holds."
}
```

Repair work generated deterministically from review attempt `recheck-MOT-04.3` of `MOT-04.3`.
Original card: `plan/cards/MOT-04.3.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] check cli-smoke fails after upstream change
- Evidence: `bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion recommend_placement --map ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json --reachability-config config/motion/reachability.yaml --out logs/mot04_check_placement.yaml --emit-verify-config logs/mot04_check_verify.yaml && ros2 run crackvision_motion recommend_placement --validate-placement logs/mot04_check_placement.yaml'` exit 2: 2026-09-29 13:57:52,111 ERROR   crackvision_motion.recommend_placement: reachability config grid.x_m.step (0.03) does not match the map's grid.x_m.step (0.025); recommend_placement requires the same sweep resolution in both
[ros2run]: Process exited with failure 2

- Consequence: previously accepted behaviour regressed
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/placement.py, ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py, ros2_ws/src/crackvision_motion/test/test_placement.py, ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json, ros2_ws/src/crackvision_motion/setup.py
- Acceptance condition: `bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion recommend_placement --map ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json --reachability-config config/motion/reachability.yaml --out logs/mot04_check_placement.yaml --emit-verify-config logs/mot04_check_verify.yaml && ros2 run crackvision_motion recommend_placement --validate-placement logs/mot04_check_placement.yaml'` exits 0
