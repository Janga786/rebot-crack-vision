# MOT-07.10.1 — Decide and append the view_move purpose contract (ADR-019 addendum + INTERFACES §17)

```json card
{
  "kind": "decision",
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "docs/adr/019-time-parameterization.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "docs/adr/016-commissioning-gated-execution.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "section17",
        "cmd": "bash -c 'grep -q \"^## 17\\. \" docs/INTERFACES.md && for s in 17.1 17.2 17.3 17.4 17.5; do grep -q \"^### $s \" docs/INTERFACES.md || { echo missing $s; exit 1; }; done && grep -q view_move docs/INTERFACES.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "adr-addendum",
        "cmd": "bash -c 'grep -q \"^## Addendum (MOT-07.10\" docs/adr/019-time-parameterization.md && grep -q view_move docs/adr/019-time-parameterization.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "contracts",
        "cmd": "./env.sh pytest tests/test_contracts.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§17 is appended after §16. The only edits to §1–§16 are a one-line forward pointer at the end of §16.8 (and, if wanted, one in §11.2/§11.7) that reads 'amended by §17'. No existing rule text is rewritten.",
      "§17 states every rule listed in the card body unambiguously. Each rule is precise enough to implement in joint_trajectory, execution_gate, retime/retime_plan and trajectory_check without further interpretation.",
      "crack_task/test files that validate today still validate: `source` stays exactly {paths3d} for them. view_move is the only purpose that carries `source.view_plan`.",
      "No real-mode gate is relaxed. Commissioning, e-stop, approval, start-state, collision, stale-config and confirmation still apply to view_move unchanged. The addendum explains why a view_plan binding is enough provenance for a camera-positioning move.",
      "The ADR addendum records the alternatives that were rejected (reusing `test`; a synthetic paths3d binding; deferring to OPS-01) and the reasons."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 4,
    "consequence": 5,
    "task_class": "motion-contract",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.10.1",
  "title": "Decide and append the view_move purpose contract (ADR-019 addendum + INTERFACES §17)",
  "parent": "MOT-07.10",
  "outcome": "ADR-019 has a MOT-07.10 addendum and docs/INTERFACES.md has a new appended §17. Together they settle §16.8: `purpose: \"view_move\"`, its `source.view_plan` binding, its G-TRAJ/G-ELIGIBLE rules, and how retime_plan and trajectory_check handle a view_plan."
}
```

Settle the gap that ADR-019/§16.8 named, recording this decision. Do NOT defer it to OPS-01: OPS-01 needs an executable capture move.

1. ADR-019: append `## Addendum (MOT-07.10): purpose "view_move" for view-plan transit legs` with Status/Context/Decision/Consequences. Add a one-line pointer at the end of §16.8: "Resolved by §17 (MOT-07.10)."

2. docs/INTERFACES.md: append `## 17. View-move trajectories (purpose "view_move"; amends §11.2, §11.7, §16.3–§16.6, resolves §16.8)` with these subsections:

### 17.1 §11.2 amendment
- `purpose` ∈ {"crack_task", "test", "view_move"}.
- For crack_task/test, `source` stays exactly `{paths3d}` (unchanged). For view_move, `source` must be exactly `{"paths3d": null, "view_plan": {"path": str non-empty, "sha256": 64-hex}}`.
- Schema errors: view_move with non-null paths3d; a missing or malformed view_plan; `source.view_plan` present on crack_task/test.
- `view_plan.path` is repo-relative (POSIX) when the file lies under `--root`, otherwise absolute. Readers resolve it against `--root`, like `source.paths3d.path`.

### 17.2 §11.7 amendment
- G-TRAJ: view_move is schema-valid and passes in every mode. It is not `test`, so it never warns.
- G-ELIGIBLE for view_move:
  - Integrity (refuse in every mode): the file at `source.view_plan.path` exists; its sha256 equals `source.view_plan.sha256`; it loads via `crackvision.view_plan/1` validation (cartesian_plan.load_view_plan with the current limits file); `chosen` and `transit_leg` are non-null; and the trajectory's first/last `positions` are within 1e-9 rad (max-abs) of `transit_leg.positions[0]`/`[-1]`.
  - Eligibility (warn in mock/dry, refuse in real; evaluated only after the integrity checks pass): the view plan's `sources.limits.sha256`, `sources.end_effector.sha256` and `sources.scene.sha256` each equal the trajectory's non-null `limits_file.sha256`, `end_effector_config_sha256` and `scene_config_sha256`. G-LIMITS-HASH and G-STALE-CONFIG then bind those shas to the live files, which closes the chain.
- The existing crack_task/test rules are unchanged. The `purpose 'test' ... nothing to check` pass stays for `test` only.
- §17.2 also lists, explicitly, the gates that are unchanged for view_move: G-ARM, G-COMMISSIONING, G-ESTOP, G-APPROVAL, G-SPEED, G-START-STATE, G-COLLISION, G-CONFIRM.

### 17.3 retime_plan amendment (§16.3/§16.4/§16.6)
- `--plan PATH` and `--view-plan PATH` are mutually exclusive, and exactly one is required. Otherwise exit 2.
- Giving `--purpose` together with `--view-plan` is exit 2. `--view-plan` always writes `purpose: "view_move"`, and `--purpose` keeps its crack_task|test choices for `--plan`.
- The view plan is loaded with load_view_plan (invalid → exit 3; `transit_leg` null → exit 3) and retimed as a one-leg plan `legs = [transit_leg]` through the same §16.2 algorithm. There are no trace legs, so G-TOOLSPEED is vacuous.
- Field mapping: §16.3 rules, with `plan.sources` read from the view plan (limits sha mismatch → refuse; end_effector sha mismatch → refuse; scene copied unchecked). `source = {paths3d: null, view_plan: {path, sha256}}`, where sha256 is the file read.
- Default `--out` is `data/trajectories/view_trajectory.json`.
- Report: `input_shas.plan` = the view-plan sha; `legs[]` has one transit entry carrying the transit_leg's `leg_index`.
- `--dry-run` prints the one-leg summary.

### 17.4 trajectory_check amendment (§16.5)
- `--plan` accepts either `crackvision.cartesian_plan/1` or `crackvision.view_plan/1`, dispatched on `schema`.
- C-SCHEMA: the trajectory parses with the §17.1 rules. A view_plan's minimal structure: `schema`; `sources.{scene,end_effector,limits,config}` each `{path, sha256}`; `chosen` a non-null object; `transit_leg` an object with `kind == "transit"` and non-empty `positions` of 6 finite floats.
- For a view plan, P = `transit_leg.positions` (one leg).
- C-PROVENANCE: a view_move trajectory requires a view_plan, and a crack_task/test trajectory requires a cartesian_plan; a mismatch is fail. For view_move, `source.paths3d` is null and `source.view_plan.sha256` equals the sha256 of the `--plan` file (the path is not compared). Limits/end_effector/scene sha rules are as §16.5, read from `view_plan.sources`.
- C-TOOLSPEED: pass with `numbers {trace_legs: []}` when there are no trace legs.
- C-REPORT: as §16.5 over the single transit leg.

### 17.5 Non-goals and defaults
- The view-plan default paths.
- Statement that real-mode view_move remains blocked by commissioning exactly like crack_task.
- No change to §11.9 records, beyond the record carrying the new purpose value.

Do not change code. Keep §0.5 and every existing contract intact.
