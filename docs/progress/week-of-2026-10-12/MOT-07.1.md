# MOT-07.1 — Decision + contract: time parameterization and independent trajectory checker (ADR-019, INTERFACES §16)

**Accepted:** 2026-10-12 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`18154dc`](https://github.com/Janga786/rebot-crack-vision/commit/18154dcdcdbbbf8db02389f65dcb849aa4c61156)

## What was done

Decided and documented how MOT-07 will turn an untimed Cartesian crack-path plan into a timed, executable robot trajectory, and how a second, independent program will check that result. I verified hands-on that the two standard off-the-shelf tools for this (MoveIt's TOTG and the Ruckig library) are not actually usable from Python on this robot's software install, which justified building a custom but simpler, well-specified algorithm instead. The new contract (ADR-019 plus a new section of the interface spec) spells out exactly what the trajectory must guarantee -- it must follow the planned path exactly, stay within the arm's speed/acceleration limits, start and stop at rest, and move the tool at a steady speed along a crack -- so the next cards can implement it and be checked against a precise, agreed specification rather than guesswork.</presentation_summary>
</invoke>

After the first audit, a repair round (MOT-07.1.R1) fixed the reported issues: The independent trajectory checker's contract (INTERFACES.md §16.5) now gives an exact rule for whether a trace leg has an admissible constant tool speed. The rule enumerates the finitely many speed breakpoints, so the checker cannot falsely reject a correct retimed trajectory. The retimer's reported speed is accepted if it is within 1e-3 m/s of that admissible set. Only the document's acceptance checks were run; there is no checker code yet.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/18154dcdcdbbbf8db02389f65dcb849aa4c61156/docs/INTERFACES.md) (+384 / −11)
- [docs/adr/019-time-parameterization.md](https://github.com/Janga786/rebot-crack-vision/blob/18154dcdcdbbbf8db02389f65dcb849aa4c61156/docs/adr/019-time-parameterization.md) (+173 / −0)

Commits: [`98ef98b`](https://github.com/Janga786/rebot-crack-vision/commit/98ef98bf812e50460212b0e39c2389b6e3feb4f2), [`44f64ab`](https://github.com/Janga786/rebot-crack-vision/commit/44f64abfd1ae0b1f9c1568ff331291a3a7a18ade), [`18154dc`](https://github.com/Janga786/rebot-crack-vision/commit/18154dcdcdbbbf8db02389f65dcb849aa4c61156)

## Why it was done

ADR-019 and an appended INTERFACES §16 fix the retiming algorithm and its output guarantees (exact plan-polyline following, finite-difference vel/acc within limit fractions, rest-to-rest, §11.6 margin, sample period/step bounds, constant tool speed on trace legs), the crackvision.retime_config/1, crackvision.retime_report/1 and crackvision.trajectory_check/1 schemas, both CLIs and their exit codes, consistent with §11 and §13/§15.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **26/51** tasks accepted; whole project: **80/138**.
- REQ-MOT-1: 21/40 contributing tasks done
- REQ-MOT-2: 13/32 contributing tasks done
- Verification: automated checks run by the pipeline itself (adr ✔, section ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair correctly replaces the checker's unworkable "bisection for largest v_c" rule with an exact existence-based breakpoint enumeration, matching finding f0068cbec5c6bb1b's acceptance condition precisely (feasible set F, min_tool_speed floor, 1e-3 m/s distance-to-F comparison for C-REPORT). Independently verified the non-monotonicity counterexample by computation; the math in the new §16.5 text is sound. Both scheduler checks pass, the diff is scoped to §16.5 only, and all other MOT-07.1 acceptance criteria remain intact.”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-07.1.R1): §16.5 checker contract is underspecified: v_c source for C-TOOLSPEED/C-REPORT, outcome vocabulary, exit-code ambiguity, report schema name, point_range convention, scene provenance; C-TOOLSPEED mandates a 'largest v_c ≤ target' bisection that cannot find v_c on fixed segment speeds, so correct trajectories fail.

## What it unlocks next

- **Ready to start:** MOT-07.2 — Pure-numpy tool_tip FK in crackvision_motion (tool_fk) with parity against crackvision.kinematics
- **Ready to start:** MOT-07.3 — Retime core: retime config loader, plan polyline assembly, refinement and limit-constrained discrete time profile (pure Python)
- **Ready to start:** MOT-07.6 — Independent trajectory checker (conda side), part 1: schema, provenance, FD limits, §11.6 margin, rest and sampling + CLI
- Closer: MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves (still needs MOT-07.5, MOT-07.7, MOT-07.9)
