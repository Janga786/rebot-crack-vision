# MOT-05.1 — Decision + contracts: commissioning-gated execution (ADR-016 + INTERFACES §11)

**Accepted:** 2026-10-09 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`4dcfad0`](https://github.com/Janga786/rebot-crack-vision/commit/4dcfad0ddb393b875cacf801686193447598dc59)

## What was done

Finished the design-and-contracts card for MOT-05's safety-gated robot-execution module: ADR-016 plus a new section 11 of INTERFACES.md now fully specify the modes, config schemas, the full real-motion gate checklist, and the CLI before any of that code gets written. Both scheduler acceptance checks (doc structure and append-only diff against the prior commit) pass with exit code 0. The design is grounded in actual vendor source citations from the robot's existing ROS driver code, including the finding that the driver's own shutdown sequence treats 'disable' as something that needs a safe-home first, which supports treating it conservatively as a torque-cut that could let the arm fall under gravity until that's verified by hand.

After the first audit, a repair round (MOT-05.1.R1) fixed the reported issues: The safety contract for the arm executor (ADR-016 and the interface spec §11) was revised to fix nine review findings. The 10%-of-limits speed cap is now a checked property: trajectories must be within the joint limits before slowing down, not only after. Software e-stops now stop the arm and leave it holding under power instead of cutting torque mid-air. All three document checks pass, and the earlier interface sections are byte-identical to the baseline.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/4dcfad0ddb393b875cacf801686193447598dc59/docs/INTERFACES.md) (+370 / −66)
- [docs/adr/016-commissioning-gated-execution.md](https://github.com/Janga786/rebot-crack-vision/blob/4dcfad0ddb393b875cacf801686193447598dc59/docs/adr/016-commissioning-gated-execution.md) (+576 / −138)

Commits: [`4517cc8`](https://github.com/Janga786/rebot-crack-vision/commit/4517cc82704df6e1e7a4c615192356204098ae44), [`4dcfad0`](https://github.com/Janga786/rebot-crack-vision/commit/4dcfad0ddb393b875cacf801686193447598dc59)

## Why it was done

ADR-016 fixes the safety design of the mock/dry/real executor, and docs/INTERFACES.md §11 (appended) defines its normative contracts: modes and driver profiles, the joint-trajectory file, execution config, commissioning record, approval record, gate-check catalogue with mode matrix, confirmation, e-stop/monitoring, execution record and the execute_trajectory CLI. MOT-05.2–.6 implement it verbatim.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **6/27** tasks accepted; whole project: **48/104**.
- REQ-MOT-1: 5/24 contributing tasks done
- REQ-INT-2: 1/13 contributing tasks done
- Verification: automated checks run by the pipeline itself (interfaces-append-only ✔, section-11-schemas ✔, adr-present ✔); independent audit accepted it (criteria: 11 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “All nine review findings on MOT-05.1 are concretely and consistently resolved in both docs/INTERFACES.md §11 and docs/adr/016-commissioning-gated-execution.md: G-LIMITS now checks the unscaled file against limits before the scaled check; the position-margin rule is sign-correct and explicitly passes a home-approach example; G-ELIGIBLE reads eligibility from the paths3d file after a sha check rather than trusting the trajectory's copy, and refuses null source.paths3d on crack_task in real; G-STALE-CONFIG refuses null EE/scene shas in real with consistent mock/dry warn behavi
…[920 chars clipped]”
- Quality loop: the audit found 9 issue(s) that were fixed before acceptance (repair task MOT-05.1.R1): Vendor surface enumeration misses a motion-producing action, the passthrough command topics and the arm_status status topic; E-stop response auto-disables an arm the ADR itself assumes may fall under gravity; G-STALE-CONFIG accepts null end_effector/scene bindings in real mode, and the ADR and table disagree on which modes it refuses; G-ELIGIBLE can be bypassed in real mode: null source.paths3d passes, and it trusts the recorded bool; CLI invocation contradicts the rclpy interpreter convention and the downstream cards.

## What it unlocks next

- **Ready to start:** MOT-05.2 — Joint-trajectory file library: validation, hashing, limit checks, time scaling, densification (pure Python)
- **Ready to start:** MOT-10.1 — Decision + contract: workcell survey procedure and survey record (INTERFACES §12, crackvision.workcell_survey/1)
