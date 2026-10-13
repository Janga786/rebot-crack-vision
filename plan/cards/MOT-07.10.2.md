# MOT-07.10.2 — Accept view_move in joint_trajectory and execution_gate (G-TRAJ, G-ELIGIBLE) with gate regression

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07.10.1"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py",
      "ros2_ws/src/crackvision_motion/test/test_execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/view_move/**"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#17",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/adr/019-time-parameterization.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py"
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
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; python3 -m pytest ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py ros2_ws/src/crackvision_motion/test/test_execution_gate.py ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py -q -p no:cacheprovider'",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "gate-regression",
        "cmd": "bash scripts/ros/test_execution.sh",
        "timeout_s": 1800,
        "expect_exit": 0
      },
      {
        "id": "real-refusal",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; unset CRACKVISION_ARM_REAL; timeout 60 ros2 run crackvision_motion execute_trajectory --mode real --trajectory ros2_ws/src/crackvision_motion/test/fixtures/trajectory_smoke.json --root \"$PWD\" </dev/null; rc=$?; echo rc=$rc; test $rc -eq 3'",
        "timeout_s": 120,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The purpose/source rules match §17.1 exactly. Existing crack_task/test files (all committed fixtures) load and gate exactly as before.",
      "G-ELIGIBLE for view_move separates integrity (fail in every mode) from eligibility (warn in mock/dry, fail in real) exactly as §17.2 states. Eligibility is evaluated only after integrity passes, and the view-plan file is always read rather than trusting the trajectory.",
      "Tests cover, for view_move: a valid binding (pass in mock/dry/real for G-TRAJ and G-ELIGIBLE); a missing file; a sha mismatch; a non-view_plan file; null chosen/transit_leg; an endpoint mismatch; each of the three sha-binding mismatches (warn in mock/dry, fail in real); and the schema refusals (view_move+paths3d, test+view_plan).",
      "No other gate's behaviour changes. In particular, `test` is still refused in real and G-ELIGIBLE's crack_task null-paths3d failure is unchanged."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "execution-gate",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.10.2",
  "title": "Accept view_move in joint_trajectory and execution_gate (G-TRAJ, G-ELIGIBLE) with gate regression",
  "parent": "MOT-07.10",
  "outcome": "joint_trajectory.load_trajectory accepts §17.1 view_move files and rejects malformed source shapes. execution_gate implements §17.2's G-TRAJ/G-ELIGIBLE view_move rules. The MOT-05 gate regression suite still passes unchanged."
}
```

Implement §17.1 and §17.2. MOT-05's code is accepted, so keep the diff minimal.

- joint_trajectory.py:
  - Add "view_move" to `_PURPOSE_VALUES`.
  - Source validation: if purpose == view_move, `set(source) == {"paths3d","view_plan"}`, paths3d is None, and view_plan is a dict with exactly {path: non-empty str, sha256: 64-hex (lower-cased)}. Otherwise keep the existing exact `{"paths3d"}` rule, so a `view_plan` key on crack_task/test raises TrajectoryError.
  - Return the normalised source dict.
- execution_gate.py:
  - G-TRAJ: view_move → pass.
  - G-ELIGIBLE: add a view_move branch before the existing `source.paths3d is None` handling.
    - Integrity (fail in every mode): resolve the path against root, sha256 the file, call cartesian_plan.load_view_plan(path, limits_path) (CartesianPlanError → fail), require non-null chosen/transit_leg, and require the trajectory's first/last positions within 1e-9 rad max-abs of transit_leg positions[0]/[-1].
    - Eligibility (via the existing `_eligible_violation(mode, ...)`, so warn in mock/dry and fail in real): the view plan's `sources.limits/end_effector/scene.sha256` equal the trajectory's limits_file.sha256 / end_effector_config_sha256 / scene_config_sha256. A null trajectory sha counts as a mismatch.
    - Put all failing reasons in the detail.
- Tests: build view plans in tmp_path from `cartesian_plan.new_view_plan`, or a minimal valid doc using the committed fixtures' limits. Put any committed fixture under fixtures/execution_gate/view_move/.
- Do not touch execute_trajectory.py. Run `scripts/ros/test_execution.sh` as the regression; it must pass unchanged.
