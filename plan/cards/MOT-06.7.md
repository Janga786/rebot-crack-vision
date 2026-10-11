# MOT-06.7 — Headless mock-stack smoke test of plan_crack_path with consistent scene + paths3d fixtures

```json card
{
  "kind": "integration",
  "depends_on": [
    "MOT-06.6"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2",
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "scripts/ros/test_cartesian_plan.sh",
      "ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/**"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#10",
    "ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "scripts/ros/_mock_stack.sh",
    "scripts/ros/test_reachability_task_frames.sh",
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
        "id": "smoke",
        "cmd": "bash scripts/ros/test_cartesian_plan.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The fixtures are committed and deterministic: a small generator script plus its committed output. The fixture scene's specimen top face lies exactly at the fixture surface height, so 0.01 m tool clearance is collision-free by construction; the fixture is explicitly test-only and config/scene/scene.yaml is untouched.",
      "The script asserts, from the written plan file and an independent /compute_fk pass made by the script itself rather than the planner's own recheck numbers: every reachable segment is `full`; trace configurations put tool_tip within 1 mm of surface_point + 0.01·n and its +X within 1° of −n; approach/retract at 0.04 m; all configurations are valid per /check_state_validity and within inclusive limits; legs are continuous; coverage.fraction_by_length matches the expected value.",
      "The unreachable segment is reported `failed` (or `partial`) with reasons, is included in the coverage denominator, and makes coverage.complete false. With --require-full-coverage the same input exits 3.",
      "A fixture segment that penetrates the specimen is reported surface_scene_mismatch and not planned.",
      "The mock stack is torn down on every exit path; the script exits 0 only if every assertion holds and never weakens an assertion to match observed behaviour."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "ros-smoke",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-06.7",
  "title": "Headless mock-stack smoke test of plan_crack_path with consistent scene + paths3d fixtures",
  "parent": "MOT-06",
  "outcome": "scripts/ros/test_cartesian_plan.sh plans a committed fixture paths3d (reachable straight and curved segments at the recommended placement, a branch, and a deliberately unreachable segment) against a consistent fixture scene on the mock stack and asserts full coverage of the reachable segments, tool_tip clearance and boresight via FK, collision validity, limits, continuity, and honest failure reporting."
}
```

Fixture generator: `test/fixtures/cartesian/smoke/make_fixture.py`, using system python3 and numpy. It writes a minimal valid §10 paths3d document and a matching `scene_config/1` fixture.

Surface: flat, at z_s (e.g. 0.010), centred at the recommended placement (0.26, 0.0). The fixture scene's specimen box top is at z_s (0.20×0.20 m), plus the table.

Segments, waypoints built per §10.5 (2 mm spacing, +X = −n, parallel-transported +Z, approach/retract at 0.04, trace at 0.01):
- (1) a straight 0.06 m main segment;
- (2) a curved arc branch;
- (3) a segment at x ≈ 0.62 m (outside reach) that must fail;
- (4) a short segment whose surface points sit 5 mm below the specimen top (surface_scene_mismatch).

Set `execution_eligible: false` with the nominal reasons, as real synthetic data would.

`test_cartesian_plan.sh`: build is assumed done. Steps:
1. `set -m`, source env_ros and the overlay, `start_mock_stack`.
2. Run plan_crack_path with the fixture scene and `--out logs/test_cartesian_plan.json`.
3. Run the independent assertions in an embedded python3 block that calls /compute_fk and /check_state_validity itself.
4. Re-run with --require-full-coverage and expect exit 3.
5. `stop_mock_stack`.
