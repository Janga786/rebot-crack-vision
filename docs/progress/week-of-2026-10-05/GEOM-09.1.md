# GEOM-09.1 — Decision + contract: pixel-to-tool-pose uncertainty budget (ADR-018, INTERFACES §14)

**Accepted:** 2026-10-11 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`5fefed8`](https://github.com/Janga786/rebot-crack-vision/commit/5fefed8e9fd98096af500471263d5a1ff165e51e)

## What was done

Added ADR-018 and a new INTERFACES.md §14 that define how we'll audit the existing pixel-to-robot-arm position uncertainty calculation: breaking the total error estimate down into named causes (camera pixel noise, depth sensor noise, hand-eye calibration, tool-tip calibration, and a new robot-joint-encoder term) and cross-checking the math against a randomized simulation of the same pipeline. Both required acceptance checks (ADR file exists, INTERFACES section present) pass, and only the two documentation files in this card's scope were touched.

After the first audit, a repair round (GEOM-09.1.R1) fixed the reported issues: Resolved the §14 self-contradiction: sigma_total_m/sigma_along_normal_m in the uncertainty-budget report are now explicitly defined as a NEW total (Σ_base^(total)=pixel+depth+handeye+fk) strictly ≥ the accepted paths3d sigma_base_m/sigma_along_normal_m, with contributions_m2 summing to that new total. The accepted paths3d FK-less value is preserved unchanged in the report (as the new field sigma_along_normal_base_m for waypoints), and within_budget continues to be computed against that unchanged value, never the new total. ADR-018 §2 was rewritten to match: it no longer claims the four partial covariances sum to "the existing Σ_base" — it names the sum Σ_base^(total), states explicitly it is not the accepted cov_base_m2/sigma_along_normal_m, and is never written back into paths3d.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/5fefed8e9fd98096af500471263d5a1ff165e51e/docs/INTERFACES.md) (+123 / −5)
- [docs/adr/018-uncertainty-budget.md](https://github.com/Janga786/rebot-crack-vision/blob/5fefed8e9fd98096af500471263d5a1ff165e51e/docs/adr/018-uncertainty-budget.md) (+181 / −7)

Commits: [`4593ab2`](https://github.com/Janga786/rebot-crack-vision/commit/4593ab20bf63e347773c91fc7fe78bb9d3e6c938), [`5fefed8`](https://github.com/Janga786/rebot-crack-vision/commit/5fefed8e9fd98096af500471263d5a1ff165e51e)

## Why it was done

ADR-018 and a new INTERFACES.md §14 define the uncertainty-budget terms, the analytic propagation method, the Monte-Carlo cross-check method and the `crackvision.uncertainty_budget/1` report file contract.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **12/28** tasks accepted; whole project: **66/123**.
- REQ-GEOM-1: 6/13 contributing tasks done
- Verification: automated checks run by the pipeline itself (adr-exists ✔, interfaces-section ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair cleanly resolves the self-contradiction: §14.4/§14.5 now define sigma_total_m/sigma_along_normal_m as a new, strictly-≥ fk-inclusive total that contributions_m2 sums to exactly, while preserving the unchanged accepted paths3d value (sigma_base_m / the new sigma_along_normal_base_m field) as the sole basis for within_budget. ADR-018 §2 and Consequences were rewritten in lockstep with identical terminology, eliminating the prior 'sums to the unchanged total' contradiction. The diff is scoped exactly to docs/INTERFACES.md and docs/adr/018-uncertainty-budget.md as re
…[143 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task GEOM-09.1.R1): §14 schema is self-contradictory: adding the new FK term cannot both leave totals unchanged and sum to them.

## What it unlocks next

- **Can now be planned in detail:** GEOM-09.2 — Analytic per-term uncertainty decomposition library (crackvision.uncertainty)
- Closer: GEOM-09.3 — Monte-Carlo cross-check of the pixel-to-tool-pose uncertainty chain (crackvision.uncertainty_mc) (still needs GEOM-09.2)
- Closer: GEOM-09.4 — Uncertainty budget CLI + report (crackvision.uncertainty_budget) (still needs GEOM-09.2, GEOM-09.3)
