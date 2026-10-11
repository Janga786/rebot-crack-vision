# MOT-06.2 — Decision + contract: Cartesian crack-path planning (ADR-017, INTERFACES §13)

```json card
{
  "kind": "decision",
  "depends_on": [
    "MOT-06.1",
    "GEOM-08",
    "MOT-05.1"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "docs/adr/017-cartesian-crack-path-planning.md",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/motion/CARTESIAN_PROBE.md",
    "docs/INTERFACES.md#8",
    "docs/INTERFACES.md#10",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#0",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "docs/adr/016-commissioning-gated-execution.md",
    "config/scene/scene.yaml",
    "config/motion/specimen_placement.yaml",
    "config/robot/b601_dm_limits.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "adr",
        "cmd": "test -s docs/adr/017-cartesian-crack-path-planning.md && grep -q 'Status:' docs/adr/017-cartesian-crack-path-planning.md",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "section",
        "cmd": "grep -qE '^## 13\\. .*cartesian' docs/INTERFACES.md && grep -q 'crackvision.cartesian_plan/1' docs/INTERFACES.md && grep -q 'crackvision.cartesian_planning_config/1' docs/INTERFACES.md && grep -qE '^## 12\\. ' docs/INTERFACES.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§13 is appended after §12, and §0–§12 are byte-identical to before; anything that amends earlier behaviour is stated in §13 itself, the way §8 amends §6.",
      "Every algorithmic choice is grounded in a MOT-06.1 fact where one applies (link_name support, fraction semantics, jump-threshold fields, timestamps). Where the probe contradicted the card body, the ADR records the alternative chosen.",
      "The contract fully specifies: input binding (paths3d §10, scene, end_effector, limits shas); segment iteration order; the candidate set (direction × roll offsets about tool_tip +X, applied uniformly per segment) and the selection rule; split/re-entry for partial traces, with a bound; the independent re-check (FK tolerance, /check_state_validity with the scene, inclusive joint limits, max joint step); the coverage metrics; per-segment status and reason vocabularies; the plan and view-plan schemas field by field; the config schema with defaults and sources; and the CLI flags, outputs and exit codes per §0.",
      "Safety invariants are explicit: collision checking is never relaxed, clearances (§8.2/§10) are never reduced to gain coverage, nothing is dropped silently, and the plan copies paths3d execution_eligible in the §11.2 `source.paths3d` shape. Planning ineligible paths is allowed for sim; the plan is untimed geometric output for MOT-07 and is never executable directly.",
      "The view phase (camera_link at 0.25 m, optical axis ≤ 15° from the anti-normal, specimen footprint inside the nominal D405 FOV) is specified as its own CLI and schema section."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 4,
    "context": 4,
    "consequence": 4,
    "task_class": "motion-contract",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-06.2",
  "title": "Decision + contract: Cartesian crack-path planning (ADR-017, INTERFACES §13)",
  "parent": "MOT-06",
  "outcome": "ADR-017 and an appended INTERFACES §13 fix the planning algorithm, roll/direction/split fallbacks, independent re-check, coverage definition, the crackvision.cartesian_planning_config/1 and crackvision.cartesian_plan/1 (+ view plan) schemas, the CLIs and their exit codes, consistent with §8, §10 and §11."
}
```

Write ADR-017 (template docs/adr/000-template.md) and APPEND `## 13. Cartesian crack-path plans (`crackvision.cartesian_plan/1`)` to docs/INTERFACES.md. Do not edit §0–§12. The design below is the lead's recommendation; adopt it unless a MOT-06.1 fact forces a change, and record any such change in the ADR.

**Inputs.** A §10 paths3d file; `config/scene/scene.yaml` (or `--scene-config`); `config/robot/end_effector.yaml`; `config/robot/b601_dm_limits.yaml`; `config/motion/cartesian_planning.yaml` (new; schema `crackvision.cartesian_planning_config/1`, strict loader, unknown key = config error).
- Start state: the §9 capture record's joint state, found through paths3d `sources.capture_record`; override with `--start-joints`.
- End state: config `end_state: start|home|none`, default `start`.

**Segments.** Iterate `components[]` in `order_index` order. Within a component, the main polyline first, then branches by `branch_index`; within a polyline, `segments[]` in order. Each segment yields: approach pose, trace poses, retract pose (tool_tip, base_link, from §10.2 waypoints).

**Candidates.** direction {forward, reverse} × roll offsets about tool_tip +X. Default offsets: 0, ±30, ±60, ±90, ±120, ±150, 180 deg. Each offset is applied uniformly to every pose of the segment; reverse swaps approach and retract.

For each candidate:
1. IK for the approach pose, seeded from the current config.
2. Transit: /plan_kinematic_path with a joint goal from the current config.
3. Cartesian approach → first trace pose.
4. Cartesian trace.
5. Cartesian retract.

Use link_name tool_tip, avoid_collisions true, and `max_step`/jump thresholds from config.

**Selection rule.** Pick the first candidate, in deterministic order, that achieves the whole trace. Among those that do, maximise the minimum joint margin to the limits; break ties on smallest |roll offset|, then forward.

**Partial trace.** If no candidate completes the trace, keep the candidate that reaches the most trace waypoints. Determine the last reached waypoint by FK matching, not by `fraction`. Add a Cartesian retract along that waypoint's normal at `approach_retract_offset_m`. Retry the remainder as a new sub-segment with the full candidate set, up to `max_splits_per_segment` (default 3). Remainders with fewer than 3 waypoints are reported as uncovered.

**Independent re-check, every output configuration.**
- /compute_fk tool_tip against the target pose for configurations that correspond to waypoints (default tolerances 1 mm / 1°).
- /check_state_validity with the scene applied.
- Inclusive limits from b601_dm_limits.yaml.
- Max per-joint step between consecutive configurations (config value).

A failed re-check fails the leg with a reason. It is never ignored.

**Surface/scene consistency (pre-planning, pure).** A segment whose trace tool positions lie inside, or less than the clearance above, any scene collision object is reported as `surface_scene_mismatch`, not planned.

**Plan file** `data/plans/{case}_cartesian_plan.json`, untimed. Fields:
- `schema`, `case_id`, `created_utc`, `producer`, `planning_frame` base_link, `joint_names` joint1..6;
- `sources`: paths3d `{path, sha256, execution_eligible}` in the §11.2 `source.paths3d` shape, plus scene, end_effector, limits and config path+sha256;
- `parameters`;
- `start_state`, `end_state`;
- `legs[]`: `{leg_index, kind: transit|approach|trace|retract, segment_ref {component_id, polyline_kind, branch_index, segment_index, sub_segment}, direction, roll_offset_rad, positions[][6], waypoint_index_map, recheck {max_pos_err_m, max_ang_err_rad, all_valid, within_limits, max_joint_step_rad}}`;
- `segments[]` report: `{segment_ref, status: full|partial|failed|surface_scene_mismatch, reasons[], candidates_tried, chosen {direction, roll_offset_rad}, trace_waypoints {total, planned}, trace_length_m {total, planned}}`;
- `coverage`: `{trace_waypoints_total, trace_waypoints_planned, length_m_total, length_m_planned, fraction_by_length, complete, upstream {points, valid, invalid_by_reason}}`, with upstream copied from paths3d counts;
- `caveats[]`: starts with a nominal-geometry disclaimer.

State that all legs concatenated form one continuous joint path, and that MOT-07 retimes it into §11.2. MoveIt timestamps are discarded.

**Reason vocabulary** (at least): `ik_failed`, `transit_failed`, `cartesian_incomplete`, `collision`, `joint_jump`, `joint_limit`, `fk_mismatch`, `surface_scene_mismatch`, `too_short`.

**View plan** (`crackvision.view_plan/1`, separate subsection):
- camera_link pose at `view_distance_m` (0.25; band 0.20–0.30) above the specimen centre, from the scene `specimen` object or `--center`;
- tilt candidates {0, 15°} and yaw candidates from config;
- FOV check: every specimen footprint corner must project inside the nominal D405 colour FOV with a margin; FOV values from config, with source;
- output: the chosen view config, the transit leg from home, and the FOV/tilt evidence.

**CLIs** (ros2 run crackvision_motion): `plan_crack_path --paths3d PATH [--out] [--start-joints] [--scene-config] [--planning-config] [--require-full-coverage] [§0 flags]` and `plan_view [--out] [--center X Y] [--scene-config] [--planning-config] [§0 flags]`. Exit codes:
- 0: plan written. Partial coverage is allowed and reported, unless `--require-full-coverage` is set.
- 1: runtime failure.
- 2: usage/config error.
- 3: refusal (invalid input, sha mismatch, a required service missing, zero trace waypoints planned, or incomplete coverage with `--require-full-coverage`).

`--dry-run` validates the inputs and prints the segment/candidate plan without ROS.

State that segment ordering is not optimised (future work).
