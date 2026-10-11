# MOT-06.2 — Decision + contract: Cartesian crack-path planning (ADR-017, INTERFACES §13)

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`a998a49`](https://github.com/Janga786/rebot-crack-vision/commit/a998a499d42f12b2f8379927dfa9c66c128b5a83)

## What was done

Wrote ADR-017 (Cartesian crack-path planning: candidate search over direction x uniform roll offset, selection rule, partial-trace FK-matching/splitting, independent re-check with jump_threshold disabled in favor of an explicit max_joint_step_rad, surface/scene mismatch excluding the traced specimen, and the view-phase geometry) and appended INTERFACES.md §13 (crackvision.cartesian_planning_config/1, crackvision.cartesian_plan/1, crackvision.view_plan/1 schemas, CLIs plan_crack_path/plan_view with §0 exit codes). Every algorithmic choice is grounded in a MOT-06.1 probe fact (wrist singularity, fraction semantics, collision-truncation silence, roll-ramp inflation, jump_threshold fields unexercised) or an explicit internal-consistency clarification recorded in the ADR.

After the first audit, a repair round (MOT-06.2.R1) fixed the reported issues: Fixed all four review findings on MOT-06.2: corrected every misdirected §13.x cross-reference, fixed ADR-017's misattribution of CARTESIAN_PROBE.md consequence 2 (it recommends the fraction formula; the card instead chose FK matching, now with an honest rationale), changed the surface/scene pre-check to test the surface object itself against a relaxed (configurable) tolerance instead of excluding it, and defined `view.footprint_m`/`view.surface_z_m` config keys plus `--center` vs scene-specimen precedence for the view phase.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/a998a499d42f12b2f8379927dfa9c66c128b5a83/docs/INTERFACES.md) (+450 / −35)
- [docs/adr/017-cartesian-crack-path-planning.md](https://github.com/Janga786/rebot-crack-vision/blob/a998a499d42f12b2f8379927dfa9c66c128b5a83/docs/adr/017-cartesian-crack-path-planning.md) (+257 / −18)

Commits: [`e6eb36f`](https://github.com/Janga786/rebot-crack-vision/commit/e6eb36f818d0674ff2ad4b3e8e61dff22b3aa54c), [`5d4c5ca`](https://github.com/Janga786/rebot-crack-vision/commit/5d4c5ca66e9f54b1f961b29a990a6019a8f4e969)

## Why it was done

ADR-017 and an appended INTERFACES §13 fix the planning algorithm, roll/direction/split fallbacks, independent re-check, coverage definition, the crackvision.cartesian_planning_config/1 and crackvision.cartesian_plan/1 (+ view plan) schemas, the CLIs and their exit codes, consistent with §8, §10 and §11.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **15/40** tasks accepted; whole project: **62/123**.
- REQ-MOT-1: 14/33 contributing tasks done
- REQ-MOT-2: 5/21 contributing tasks done
- Verification: automated checks run by the pipeline itself (adr ✔, section ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this repair. All four findings are fixed, and the original MOT-06.2 criteria still hold. - **§0–§12 unchanged:** the first 1810 lines of `docs/INTERFACES.md` match base 78e6b7a exactly. - **Cross-references:** every §13.x reference now points at the subsection that defines the rule. - **Probe consequence 2:** ADR-017 §3 now says the probe recommends `round(fraction*(n-1))`. It records FK matching as a deliberate departure with its own rationale, and §13.7 no longer says consequence 2 forbids `fraction`. - **Surface check:** §13.9 now tests the specimen against
…[538 chars clipped]”
- Quality loop: the audit found 4 issue(s) that were fixed before acceptance (repair task MOT-06.2.R1): Surface/scene pre-check excludes the specimen entirely, losing the 'trace position inside the surface object' case; View-phase footprint for --center references a non-existent config key; Many §13 internal cross-references point to the wrong subsection; ADR/§13 misstate CARTESIAN_PROBE consequence 2 as rejecting fraction-based last-waypoint mapping.

## What it unlocks next

- **Ready to start:** MOT-06.3 — Cartesian planning core: config loader, paths3d segment tasks, roll/direction candidates, surface-scene check (pure Python)
- **Ready to start:** MOT-06.5 — MoveIt client layer for Cartesian/transit/IK/FK/validity requests (cartesian_moveit)
