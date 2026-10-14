# OPS-01.1 — Decision + contract: operator CLI stage model, run-state schema, resumability (ADR-021, INTERFACES §19)

```json card
{
  "kind": "decision",
  "requirements": [
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "docs/adr/021-operator-cli-workflow.md",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/adr/019-time-parameterization.md",
    "docs/adr/020-trajectory-preview-approval.md",
    "docs/INTERFACES.md#8. End-of-arm frames, task frames and the eye-in-hand capture contract (ADR-014)",
    "docs/INTERFACES.md#9. Eye-in-hand capture record (crackvision.capture_3d/1)",
    "docs/INTERFACES.md#11. Commissioning-gated execution (ADR-016)",
    "docs/INTERFACES.md#17. View-move trajectories",
    "docs/INTERFACES.md#18. Trajectory preview and operator approval",
    "docs/motion/EXECUTION.md",
    "docs/motion/PREVIEW_APPROVAL.md",
    "docs/motion/CARTESIAN_PLANNING.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "section-present",
        "cmd": "grep -q '^## 19\\.' docs/INTERFACES.md",
        "timeout_s": 10,
        "expect_exit": 0
      },
      {
        "id": "adr-present",
        "cmd": "test -f docs/adr/021-operator-cli-workflow.md",
        "timeout_s": 10,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "ADR-021 follows docs/adr/000-template.md's structure (context, decision, consequences) and does not contradict ADR-014/016/019/020.",
      "INTERFACES.md §19 is APPENDED (no existing numbered section is rewritten) and defines: the stage list and fixed order doctor, capture, segment, paths, plan, preview, execute; which stages are case_id-scoped vs. global; the `data/operator_runs/{case_id}_run_state.json` file, schema `crackvision.operator_run/1`, with per-stage status in {pending, done, failed}, each stage's declared output file paths + sha256, and timestamps.",
      "The resumability rule is explicit and mechanical: a stage is skippable (treated as already done) only if its recorded status is done AND every one of its recorded output files still exists with a matching sha256 AND every upstream stage it declares a dependency on is itself still fresh by the same rule (cascade invalidation on a stale upstream output) -- `--force` always reruns a named stage regardless, `--stage NAME` runs exactly one named stage and exits 3 if an undone prerequisite stage exists.",
      "§19 states plainly, and ADR-021's Decision section repeats, that no OPS-01 script may ever call execute_trajectory without having first produced a trajectory_preview manifest and a matching crackvision.execution_approval/1 file for that exact trajectory sha (the same approve_trajectory step), for every mode including mock/dry, and that OPS-01 never sets CRACKVISION_ARM_REAL itself and never defaults --mode to anything but mock.",
      "§19 explicitly names the view-move case (ADR-014/§17): capture's optional move-to-view-pose step is itself a trajectory run subject to the same preview/approve/execute sequence, never a shortcut.",
      "Doctor is defined as case-independent (no case_id), read-only, and its own exit code is restricted to {0, 3} (never 1/2) since it only aggregates other read-only checks."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 3,
    "context": 4,
    "consequence": 3,
    "task_class": "contract-design",
    "local_ok": false
  },
  "id": "OPS-01.1",
  "title": "Decision + contract: operator CLI stage model, run-state schema, resumability (ADR-021, INTERFACES §19)",
  "parent": "OPS-01",
  "outcome": "A written, normative contract (new ADR + INTERFACES.md §19) that fixes the operator workflow's stage list/order, the per-case run-state file schema, the resumability/re-entry rule, and the rule that any trajectory execution (capture's view move or the final crack-trace execute) always goes through retime_plan -> trajectory_check -> trajectory_preview -> approve_trajectory -> execute_trajectory in that order with no OPS-01 code path ever skipping a gate."
}
```

Write `docs/adr/021-operator-cli-workflow.md` (follow `docs/adr/000-template.md`) and append a new `## 19. Operator CLI workflow (crackvision.operator_run/1)` section to `docs/INTERFACES.md` (after §18; do not renumber or edit any existing section, per repo convention).

Cover, concretely:

1. **Stage list and order:** `doctor` (global, no case_id) then, per case_id, `capture -> segment -> paths -> plan -> preview -> execute`, matching OPS-01's title. State which existing contract/CLI each stage delegates to (do not invent new logic where one already exists):
   - `doctor`: `scripts/check_env.py`, `scripts/check_host.py`, `scripts/check_realsense.py` (+ a `ros2_ws/install/setup.bash` presence check).
   - `capture`: §8.4/§9 capture contract -- `crackvision.realsense_capture`, `crackvision_camera capture_joint_state`/`capture_optical_tf`, `crackvision.capture_record assemble`; optionally preceded by a view-move trajectory run (§17) when the camera is not already in position.
   - `segment`: `crackvision.prepare_inputs` + `crackvision.inference`.
   - `paths`: `crackvision.skeleton` + `crackvision.paths` + `crackvision.visualize_paths`.
   - `plan`: `crackvision.path3d` (§10) + `plan_crack_path` (§13) + `retime_plan --plan` (§16) + `trajectory_check` (§16.5).
   - `preview`: `trajectory_preview` (§18.1).
   - `execute`: `approve_trajectory` (§18.2) + `execute_trajectory` (§11).
2. **Run-state file:** `data/operator_runs/{case_id}_run_state.json`, schema `crackvision.operator_run/1`: `{schema, case_id, created_utc, updated_utc, stages: {<stage>: {status, updated_utc, outputs: [{path, sha256}], depends_on: [stage,...], notes}}}`. Never committed (git-ignored, like `data/captures/`).
3. **Freshness/resumability rule** exactly as in the acceptance criteria above -- spell out the cascade-invalidation example (e.g. a re-run after `capture` changes invalidates `plan`/`preview`/`execute`'s cached done status even if their own output files are untouched).
4. **Gate-non-bypass rule:** restate, in OPS-01's own words, that `execute_trajectory`'s G-* gates (§11, §17, §18) are the only authority on whether real motion happens; OPS-01 only sequences the existing CLIs in the order EXECUTION.md/PREVIEW_APPROVAL.md already specify and never adds a flag that skips `trajectory_preview`/`approve_trajectory`, never sets `CRACKVISION_ARM_REAL`, and defaults every `--mode` to `mock`.
5. State which stages require ROS sourced (`capture`'s joint-state/optical-tf read and view move, `plan`'s `plan_crack_path`, `execute`'s `execute_trajectory`) vs. pure `./env.sh` (`segment`, `paths`, `plan`'s `path3d`/`retime_plan`/`trajectory_check`, `preview`).

Keep it short and normative, like ADR-019/020 -- this is the contract every other OPS-01.n card implements against.
