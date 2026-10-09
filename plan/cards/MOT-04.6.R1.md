# MOT-04.6.R1 — Repair MOT-04.6: check unit fails after upstream change

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "config/motion/reachability.yaml",
      "docs/INTERFACES.md",
      "ros2_ws/src/crackvision_motion/crackvision_motion/placement.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/reachability_task_frames_smoke.yaml",
      "ros2_ws/src/crackvision_motion/test/fixtures/reachability_view_smoke.yaml",
      "ros2_ws/src/crackvision_motion/test/fixtures/scene_table.yaml",
      "ros2_ws/src/crackvision_motion/test/test_placement.py",
      "ros2_ws/src/crackvision_motion/test/test_reachability_core.py",
      "scripts/ros/test_reachability_task_frames.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#7",
    "docs/INTERFACES.md#8",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
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
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "task-frames-smoke",
        "cmd": "bash scripts/ros/test_reachability_task_frames.sh",
        "timeout_s": 1200,
        "expect_exit": 0
      },
      {
        "id": "legacy-sweep-smoke",
        "cmd": "bash scripts/ros/test_reachability.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "cli-smoke",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion recommend_placement --map ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json --reachability-config config/motion/reachability.yaml --out logs/mot046_check_placement.yaml --emit-verify-config logs/mot046_check_verify.yaml && ros2 run crackvision_motion recommend_placement --validate-placement logs/mot046_check_placement.yaml'",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "`bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider` exits 0",
      "All acceptance criteria of MOT-04.6 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 5,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4
  },
  "track": "simulation",
  "priority": 99,
  "repairs": "MOT-04.6",
  "findings": [
    "22c0467a55a38a48"
  ],
  "id": "MOT-04.6.R1",
  "title": "Repair MOT-04.6: check unit fails after upstream change",
  "parent": "MOT-04",
  "outcome": "Resolve the review findings on MOT-04.6 while every acceptance criterion of MOT-04.6 still holds."
}
```

Repair work generated deterministically from review attempt `recheck-MOT-04.6` of `MOT-04.6`.
Original card: `plan/cards/MOT-04.6.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] check unit fails after upstream change
- Evidence: `bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider` exit 2: , kwargs)
/usr/lib/python3/dist-packages/pluggy/manager.py:92: in _hookexec
    return self._inner_hookexec(hook, methods, kwargs)
/usr/lib/python3/dist-packages/pluggy/manager.py:83: in <lambda>
    self._inner_hookexec = lambda hook, methods, kwargs: hook.multicall(
/opt/ros/humble/lib/python3.10/site-packages/launch_testing/pytest/hooks.py:193: in pytest_pycollect_makemodule
    entrypoint = find_launch_test_entrypoint(path)
/opt/ros/humble/lib/python3.10/site-packages/launch_testing/pytest/hooks.py:186: in find_launch_test_entrypoint
    module = path.pyimport()
/usr/lib/python3/dist-packages/py/_path/local.py:704: in pyimport
    __import__(modname)
/usr/lib/python3/dist-packages/_pytest/assertion/rewrite.py:170: in exec_module
    exec(co, module.__dict__)
ros2_ws
…[720 chars clipped]
- Consequence: previously accepted behaviour regressed
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py, ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py, ros2_ws/src/crackvision_motion/crackvision_motion/placement.py, ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py, ros2_ws/src/crackvision_motion/test/test_reachability_core.py
- Acceptance condition: `bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider` exits 0
