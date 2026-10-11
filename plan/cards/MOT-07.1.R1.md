# MOT-07.1.R1 — Repair MOT-07.1: §16.5 checker contract is underspecified: v_c source for C-TOOLSPEED/C

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "docs/adr/019-time-parameterization.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#15",
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "docs/motion/CARTESIAN_PLANNING.md",
    "config/robot/b601_dm_limits.yaml",
    "config/robot/end_effector.yaml",
    "config/motion/execution.yaml",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "src/crackvision/kinematics.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "adr",
        "cmd": "test -s docs/adr/019-time-parameterization.md && grep -q 'Status:' docs/adr/019-time-parameterization.md",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "section",
        "cmd": "grep -qE '^## 16\\. ' docs/INTERFACES.md && grep -q 'crackvision.retime_config/1' docs/INTERFACES.md && grep -q 'crackvision.retime_report/1' docs/INTERFACES.md && grep -q 'crackvision.trajectory_check/1' docs/INTERFACES.md && grep -nE '^## (15|16)\\. ' docs/INTERFACES.md | awk -F: 'NR==1&&$2~/^## 15/{a=$1} NR==2&&$2~/^## 16/{b=$1} END{exit !(a&&b&&b>a)}'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§16.5 (or a §16.4/§16.6 addition) must state all of the following. (a) How C-TOOLSPEED gets v_c without the report, e.g. 'a v_c ≤ target exists such that rules 1–3 hold', with the inference made explicit. (b) What C-REPORT compares v_c against, with numeric tolerances for v_c, duration_s and the tool_speed extrema. (c) An outcome vocabulary that includes the outcomes for an absent --report and for a failed precondition, consistent with the exit-code rule. (d) A rule separating exit 2 from a C-SCHEMA fail (exit 3) for a schema-invalid trajectory. (e) A single report schema name consistent with the card's crackvision.trajectory_check/1, and a default --out path. (f) The point_range convention at shared leg-boundary points. (g) What scene_config_sha256 is checked against, and how that file is supplied.",
      "All acceptance criteria of MOT-07.1 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 4,
    "context": 4,
    "consequence": 5,
    "task_class": "motion-contract",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-07.1",
  "findings": [
    "49e4da76a512b0ff"
  ],
  "id": "MOT-07.1.R1",
  "title": "Repair MOT-07.1: §16.5 checker contract is underspecified: v_c source for C-TOOLSPEED/C",
  "parent": "MOT-07",
  "outcome": "Resolve the review findings on MOT-07.1 while every acceptance criterion of MOT-07.1 still holds."
}
```

Repair work generated deterministically from review attempt `00778-MOT-07.1-review` of `MOT-07.1`.
Original card: `plan/cards/MOT-07.1.md` — its acceptance criteria must still hold.

## Finding 1 [major] §16.5 checker contract is underspecified: v_c source for C-TOOLSPEED/C-REPORT, outcome vocabulary, exit-code ambiguity, report schema name, point_range convention, scene provenance
- Evidence: 1. INTERFACES.md:2651 C-TOOLSPEED says the band/ramp/never-above rules 'are independently recomputed per trace leg', but those rules are parameterised by v_c. Line 2629 says the checker derives checks 'never from retime_plan's own report, except where C-REPORT', so without --report no v_c is available, and no rule says how to infer one.
2. Line 2652 C-REPORT compares 'the report's declared per-leg v_c ... with what the checker itself recomputes', but the checker never recomputes a v_c. No comparison tolerances are given either; MOT-07.7 expects 'within stated tolerances'.
3. Line 2654 limits outcomes to `pass|fail`. That leaves no outcome for C-REPORT when --report is absent, or for checks that cannot be evaluated after C-SCHEMA fails. MOT-07.6 ('a check whose precondition failed is reported as error ... omit them or report them as not implemented, as §16.5 allows') and MOT-07.7 ('reports C-REPORT as skip (or as §16.5 defines)') both assume vocabulary §16.5 does not have.
4. Lines 2658-2659 make 'unreadable/malformed input file' exit 2, while C-SCHEMA failing ('the trajectory parses as crackvision.joint_trajectory/1') is exit 3. A schema-invalid trajectory matches both.
5. Line 2656 sets `schema: "crackvision.trajectory_check_report/1"`. The card outcome and MOT-07.6 ('writing a crackvision.trajectory_check/1 report') expect `crackvision.trajectory_check/1`. The checker's --out has no default in §16.6, but MOT-07.6 expects `<trajectory>.check.json`.
6. §16.4 `point_range: [first, last]` does not say whether the leg-boundary point (the vertex shared after the duplicate is dropped, since load_plan enforces continuity at 1e-6, cartesian_plan.py:301-310) belongs to both legs. C-REPORT compares point_range anyway.
7. Line 2645 checks scene_config_sha256 against 'the files given on the command line', but §16.5/§16.6 list no scene-config flag, and nothing compares it to plan.sources.scene.sha256.
- Consequence: MOT-07.6/07.7 must build the checker independently from §16 text, and MOT-07.4/07.5 build the retimer separately. Where the contract leaves choices open, the two sides will pick different conventions (v_c inference, point_range overlap, schema string, outcome for an absent report). C-REPORT/C-TOOLSPEED could then fail on a correct trajectory or pass a wrong one, and the 'two independent implementations agree' guarantee in ADR-019 breaks down.
- Affected: docs/INTERFACES.md
- Acceptance condition: §16.5 (or a §16.4/§16.6 addition) must state all of the following. (a) How C-TOOLSPEED gets v_c without the report, e.g. 'a v_c ≤ target exists such that rules 1–3 hold', with the inference made explicit. (b) What C-REPORT compares v_c against, with numeric tolerances for v_c, duration_s and the tool_speed extrema. (c) An outcome vocabulary that includes the outcomes for an absent --report and for a failed precondition, consistent with the exit-code rule. (d) A rule separating exit 2 from a C-SCHEMA fail (exit 3) for a schema-invalid trajectory. (e) A single report schema name consistent with the card's crackvision.trajectory_check/1, and a default --out path. (f) The point_range convention at shared leg-boundary points. (g) What scene_config_sha256 is checked against, and how that file is supplied.
