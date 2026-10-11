# MOT-06.2.R1 — Repair MOT-06.2: Many §13 internal cross-references point to the wrong subsection

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
      "docs/adr/017-cartesian-crack-path-planning.md"
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
      "Every §13.x reference inside §13 names the subsection that actually defines the referenced rule (partial trace=§13.7, re-check=§13.8, surface check=§13.9, reasons=§13.10, plan file/sources/segments=§13.11.x, coverage=§13.12), and §0–§12 stay byte-identical.",
      "The ADR states that the probe's consequence 2 recommends `round(fraction*(n-1))`, records FK matching as the chosen alternative with its own rationale (or adopts the probe's method), and §13.7 no longer cites consequence 2 as forbidding `fraction`. Citations of 'CARTESIAN_PROBE.md fact N' either match a real identifier in that doc or are reworded.",
      "§13.9 still tests trace positions against the surface object, e.g. flagging distance < clearance − a stated, configurable tolerance (or at minimum flagging positions inside it), and ADR-017 §5 gives a correct rationale for whatever tolerance is chosen.",
      "§13.14 and §13.2 define where the footprint and surface z come from when `--center` is used (e.g. a `view.footprint_m`/`view.surface_z_m` config key with a default and source, or a stated refusal), and state precedence when both `--center` and a scene specimen are present.",
      "All acceptance criteria of MOT-06.2 still hold at the new revision"
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
  "priority": 95,
  "repairs": "MOT-06.2",
  "findings": [
    "ea692fa1b6251085",
    "f2661111e77aa954",
    "2167cd37e7f60af7",
    "23e6168bc9b66440"
  ],
  "id": "MOT-06.2.R1",
  "title": "Repair MOT-06.2: Many §13 internal cross-references point to the wrong subsection",
  "parent": "MOT-06",
  "outcome": "Resolve the review findings on MOT-06.2 while every acceptance criterion of MOT-06.2 still holds."
}
```

Repair work generated deterministically from review attempt `00458-MOT-06.2-review` of `MOT-06.2`.
Original card: `plan/cards/MOT-06.2.md` — its acceptance criteria must still hold.

## Finding 1 [major] Many §13 internal cross-references point to the wrong subsection
- Evidence: docs/INTERFACES.md headings: 13.7 Partial trace, 13.8 Independent re-check, 13.9 Surface/scene check, 13.10 Reason vocabulary, 13.11 Plan file. The text says: line 1824 'applied for every online check (§13.9) and the pre-planning surface check (§13.10)'; line 1826 'inclusive position bound (§13.9)'; line 1829 'recorded in the output file's `sources` (§13.12)'; lines 1853-1855 'recheck.* ... §13.9'; lines 1856-1857 'splitting.* ... §13.8'; line 1859 '§13.10's mismatch check'; line 1878 'sub-segment, §13.8'; line 1888 '§13.10's surface/scene check'; line 1913 '`fraction < 1` → see §13.8 (partial trace)'; line 1919 'passes §13.9's independent re-check'; line 1926 'If no candidate fully completes, §13.8 applies'; line 1962 'one `segments[]` entry (§13.12)'; line 1985 '(`fk_mismatch`, ... §13.9 below)'. Every one of these is off by one subsection.
- Consequence: ADR-017 states that §13 is 'implemented verbatim' by MOT-06.3–.6. Implementers following these references land on the wrong rules, e.g. the partial-trace step sends them to the re-check section and the re-check reference sends them to the surface check.
- Affected: docs/INTERFACES.md
- Acceptance condition: Every §13.x reference inside §13 names the subsection that actually defines the referenced rule (partial trace=§13.7, re-check=§13.8, surface check=§13.9, reasons=§13.10, plan file/sources/segments=§13.11.x, coverage=§13.12), and §0–§12 stay byte-identical.

## Finding 2 [major] ADR/§13 misstate CARTESIAN_PROBE consequence 2 as rejecting fraction-based last-waypoint mapping
- Evidence: CARTESIAN_PROBE.md:42-45: 'MOT-06.2 must map `fraction` back to "how far along the crack did we get" via `round(fraction * (len(requested_waypoints) - 1))` against the *requested* waypoints, never by indexing or counting the returned `RobotTrajectory`.' ADR-017 Alternatives: 'Index the returned trajectory by its own point count/`fraction` ... Rejected (fact 2): ... the probe is exactly why `fraction` indexing is unsound here.' INTERFACES §13.7: 'never by `fraction` or by indexing the returned trajectory (CARTESIAN_PROBE.md consequence 2)'.
- Consequence: The criterion requires choices grounded in probe facts, and requires the ADR to record where the probe contradicts the card body. Here the probe recommends fraction mapping and the card body mandates FK matching. The ADR hides that disagreement by attributing the opposite conclusion to the probe, so downstream readers get a false record of the evidence.
- Affected: docs/adr/017-cartesian-crack-path-planning.md, docs/INTERFACES.md
- Acceptance condition: The ADR states that the probe's consequence 2 recommends `round(fraction*(n-1))`, records FK matching as the chosen alternative with its own rationale (or adopts the probe's method), and §13.7 no longer cites consequence 2 as forbidding `fraction`. Citations of 'CARTESIAN_PROBE.md fact N' either match a real identifier in that doc or are reworded.

## Finding 3 [major] Surface/scene pre-check excludes the specimen entirely, losing the 'trace position inside the surface object' case
- Evidence: INTERFACES §13.9: 'compute the signed distance to every scene object ... **except** the one named by `surface_object_id` (default "specimen")'. The card body says: 'A segment whose trace tool positions lie inside, or less than the clearance above, any scene collision object is reported as `surface_scene_mismatch`'. ADR §5 justifies the exclusion with 'Checking against that object unmodified would flag every trace waypoint, always'. But trace waypoints sit at distance == clearance from a consistent surface, and the card's test is strict 'less than', so a consistent scene would not fire, apart from numerical tolerance. config/scene/scene.yaml models only `table` and `specimen`, so after the exclusion only the table is checked.
- Consequence: The main scene/surface inconsistency goes unflagged by the pure pre-check: a measured crack surface lying inside or below the scene's specimen box (e.g. a placeholder specimen pose that is wrong, which scene.yaml itself labels PLACEHOLDER). Such segments are planned and fail later as generic `cartesian_incomplete`/`collision` instead of `surface_scene_mismatch`. The ADR's claim that the check still catches 'a mis-placed specimen' is false under the exclusion.
- Affected: docs/INTERFACES.md, docs/adr/017-cartesian-crack-path-planning.md
- Acceptance condition: §13.9 still tests trace positions against the surface object, e.g. flagging distance < clearance − a stated, configurable tolerance (or at minimum flagging positions inside it), and ADR-017 §5 gives a correct rationale for whatever tolerance is chosen.

## Finding 4 [major] View-phase footprint for --center references a non-existent config key
- Evidence: INTERFACES §13.14: 'Centre (cx, cy) and footprint from the scene's `specimen` object ..., or `--center X Y` with `view.fov` defaults' footprint if no scene specimen exists.' §13.2's `view.*` keys are `view_distance_m`, `tilt_candidates_deg`, `yaw_candidates_deg`, `fov_deg.horizontal/vertical` and `fov_margin_deg`. None of them is a footprint. `z_top` is also undefined when there is no scene specimen. The case where `--center` is given and a scene specimen also exists is not covered either.
- Consequence: The FOV corner check, which this card's acceptance criterion requires, has no defined footprint or surface height on the `--center` path. Implementers must invent one, and plan_view output becomes implementation-defined.
- Affected: docs/INTERFACES.md
- Acceptance condition: §13.14 and §13.2 define where the footprint and surface z come from when `--center` is used (e.g. a `view.footprint_m`/`view.surface_z_m` config key with a default and source, or a stated refusal), and state precedence when both `--center` and a scene specimen are present.
