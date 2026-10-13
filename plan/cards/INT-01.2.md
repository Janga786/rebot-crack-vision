# INT-01.2 — One command: synthetic perception through MoveIt-mock crack planning, retiming and a limits-checked mock execution

```json card
{
  "kind": "integration",
  "depends_on": [
    "INT-01.1"
  ],
  "requirements": [
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "scripts/ros/run_integration_e2e.sh"
    ]
  },
  "inputs": [
    "scripts/perc/run_synthetic_perception.sh",
    "scripts/ros/run_cartesian_synthetic.sh",
    "scripts/ros/test_retime_execute.sh",
    "scripts/ros/_mock_stack.sh",
    "scripts/ros/env_ros.sh",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#16",
    "docs/motion/CARTESIAN_PLANNING.md",
    "docs/motion/TIME_PARAMETERIZATION.md"
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
        "id": "e2e",
        "cmd": "bash scripts/ros/run_integration_e2e.sh",
        "timeout_s": 2400,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Step 1 runs scripts/perc/run_synthetic_perception.sh (INT-01.1) unmodified to get a scratch root with real paths3d output derived through the real visualize/skeleton/paths/visualize_paths chain -- never the GT-skeleton-bypass scratch root scripts/ros/run_cartesian_synthetic.sh builds for MOT-06.10's own purposes.",
      "A scratch scene_config/1 (table + specimen) is built from the generator's own ground truth (data/synthetic/{case}_gt.json's center_base_m/normal_base), same pattern as run_cartesian_synthetic.sh step 2, so the planning scene is geometrically consistent with the synthetic surface by construction; config/scene/scene.yaml is never read or modified.",
      "plan_view then plan_crack_path --require-full-coverage both exit 0 on the headless mock stack; coverage.complete is asserted true from the loaded plan.",
      "retime_plan --purpose crack_task exits 0 and writes both the trajectory and its .retime.json report; crackvision.trajectory_check (conda env, -m crackvision.trajectory_check) exits 0 and every check's outcome is pass or skip, never fail/error.",
      "execute_trajectory --mode dry exits 0 with would_refuse_in_real containing at least G-ELIGIBLE and G-APPROVAL (matching MOT-07.9's own documented result for this same nominal/ineligible input).",
      "The mock arm is pre-positioned to the plan's start_state with a small hand-written purpose:\"test\" trajectory before --mode mock is run on the real retimed trajectory (same pattern as scripts/ros/test_retime_execute.sh), then execute_trajectory --mode mock exits 0 with outcome completed and G-LIMITS, G-STALE-CONFIG and G-COLLISION all pass, and final /joint_states arrival within the retimed trajectory's last point verified independently of the execution record's own claim.",
      "Only the moveit_mock profile and --mode dry/mock are used; CRACKVISION_ARM_REAL is never set; the mock stack is torn down on every exit path (trap), and no move_group/ros2_control_node/robot_state_publisher process remains afterward.",
      "Everything is written under a fresh logs/integration_e2e/<stamp>/ (or reused scratch roots under logs/synthetic_perception/, logs/cartesian_synthetic-equivalent scratch dirs this script itself creates) -- nothing under data/ or config/ in the repo tree."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 4,
    "consequence": 4,
    "task_class": "integration-test",
    "local_ok": false
  },
  "track": "simulation",
  "id": "INT-01.2",
  "title": "One command: synthetic perception through MoveIt-mock crack planning, retiming and a limits-checked mock execution",
  "parent": "INT-01",
  "outcome": "scripts/ros/run_integration_e2e.sh chains INT-01.1's script into plan_view -> plan_crack_path -> retime_plan -> crackvision.trajectory_check -> execute_trajectory --mode dry and --mode mock on the headless moveit_mock stack, as one command, with full coverage, all limits/retime checks passing and arrival verified."
}
```

## Why

This is INT-01's headline deliverable: the full REQ-INT-1 chain, synthetic RGB-D case to a limits-checked mock execution, as one command. `scripts/ros/test_retime_execute.sh` (MOT-07.9) already proves every downstream step (retime_plan -> trajectory_check -> execute_trajectory dry/mock) against `run_cartesian_synthetic.sh`'s (MOT-06.10) GT-skeleton-bypass scratch plan; this card swaps that front end for INT-01.1's real segmentation-to-paths-to-3D script and assembles the result into one new script, reusing every downstream pattern from `test_retime_execute.sh` nearly verbatim.

## What to build

`scripts/ros/run_integration_e2e.sh` (new file). Model it directly on `scripts/ros/test_retime_execute.sh` (same `fail()` helper, `set -uo pipefail; set -m`, the `reBotArmControl` real-driver guard, `logs/integration_e2e/<stamp>/` scratch work dir, `_mock_stack.sh`/`env_ros.sh` sourcing, trap-based teardown and the stray-process check at the end):

1. **Front end (replaces `run_cartesian_synthetic.sh`'s role):** run `bash "$REPO_ROOT/scripts/perc/run_synthetic_perception.sh"`, capture its `scratch root <path>` stdout line the same way `test_retime_execute.sh` recovers `run_cartesian_synthetic.sh`'s root, call it `PERC_ROOT`. Confirm `PERC_ROOT/data/paths3d/<case>_paths3d.json` exists.
2. **Scratch scene:** build a `scene.yaml` under a new work dir (`logs/integration_e2e/<stamp>/`) from `PERC_ROOT/data/synthetic/<case>_gt.json`'s `center_base_m`/`normal_base`, same table+specimen construction `run_cartesian_synthetic.sh` step 2 uses (reuse that logic directly -- copy the heredoc, adapted to read from `PERC_ROOT` instead of regenerating the ground truth). Fix up `component_id` int->str in the paths3d copy as `run_cartesian_synthetic.sh` does (same documented cross-system workaround; do not touch `cartesian_core.py` or `crackvision.path3d`).
3. Copy `config/robot/{end_effector,b601_dm_limits}.yaml` into the work dir's `config/robot/`.
4. Source `env_ros.sh`, check the built overlay (`ros2_ws/install/setup.bash`, exit 3 if missing per `docs/INTERFACES.md` §0), start the mock stack, wait for MoveIt services (same `rclpy` wait-loop as `run_cartesian_synthetic.sh`).
5. `plan_view` then `plan_crack_path --require-full-coverage`; fail loudly on non-zero exit.
6. Validate `view_plan.json`/`cartesian_plan.json` with the real `load_view_plan`/`load_plan` loaders (same validation block as `run_cartesian_synthetic.sh` step 6), asserting `coverage.complete is True`.
7. `retime_plan --plan <cartesian_plan.json> --root <work dir> --limits/--end-effector-config <work-dir copies> --execution-config config/motion/execution.yaml --retime-config config/motion/retime.yaml --urdf presentation/sim/reBot_B601_DM_with_gripper.urdf --purpose crack_task --out <trajectory.json>`; assert exit 0, trajectory + `.retime.json` exist.
8. `./env.sh python -m crackvision.trajectory_check --trajectory ... --plan ... --report ... --limits ... --end-effector-config ... --execution-config ... --retime-config ... --out ... -q`; assert exit 0 and every `checks[].outcome` is `pass`/`skip`.
9. `apply_scene` the scratch scene; pre-position the mock arm to the trajectory's first point via a hand-written `purpose:"test"` 3-point trajectory in `--mode mock` (copy `test_retime_execute.sh`'s `to_start.json` construction and `read_joint_state`/`max_delta_to_point`/`assert_lt` helpers verbatim).
10. `execute_trajectory --mode dry --profile moveit_mock ...`; assert exit 0 and `would_refuse_in_real` contains `G-ELIGIBLE` and `G-APPROVAL`.
11. `execute_trajectory --mode mock --profile moveit_mock ...`; assert exit 0, `outcome == "completed"`, `G-LIMITS`/`G-STALE-CONFIG`/`G-COLLISION` all `pass`, and live `/joint_states` within tolerance of the trajectory's last point.
12. `stop_mock_stack`; stray-process check; `echo "run_integration_e2e.sh: OK (...)"`; `exit 0`.

Reuse code from `run_cartesian_synthetic.sh`/`test_retime_execute.sh` by copying the relevant blocks (these are standalone bash scripts, not a shared library) -- do not modify either of those files.

## Out of scope
- The view-plan leg's own retime/execute (already covered by MOT-07.10.5's `test_view_retime_execute.sh`); this card only needs the crack trace leg to satisfy "MoveIt plan (mock) -> limits".
- Real/vendor execution modes, approval workflows, physical hardware.
