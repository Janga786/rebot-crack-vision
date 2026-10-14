# INT-03.1 — Empty-mask failure-injection test (perception chain)

**Accepted:** 2026-10-14 · **Work package:** Integration & commissioning (L-INT) · **Track:** simulation · **Verified commit:** [`4a01422`](https://github.com/Janga786/rebot-crack-vision/commit/4a0142222a1a97d0eb422ac08b55ffad5cc672de)

## What was done

Built an automated test that feeds a legitimate "no crack detected" (all-zero) mask through the full perception pipeline (skeleton extraction, path tracing, 3D lifting) to verify it fails safely rather than crashing or inventing a fake crack. The pipeline already handled this correctly end-to-end -- it reports zero skeleton pixels, zero path components, and an explicit "not safe to execute" flag with the correct reason -- so no code changes were required, only the new regression test and a documentation writeup for future integration cards to cite.

Files changed:
- [docs/integration/safety/EMPTY_MASK.md](https://github.com/Janga786/rebot-crack-vision/blob/4a0142222a1a97d0eb422ac08b55ffad5cc672de/docs/integration/safety/EMPTY_MASK.md) (+105 / −0)
- [scripts/perc/test_safety_empty_mask.sh](https://github.com/Janga786/rebot-crack-vision/blob/4a0142222a1a97d0eb422ac08b55ffad5cc672de/scripts/perc/test_safety_empty_mask.sh) (+170 / −0)

Commits: [`4a01422`](https://github.com/Janga786/rebot-crack-vision/commit/4a0142222a1a97d0eb422ac08b55ffad5cc672de)

## Why it was done

A new headless script feeds an all-zero (no-crack) mask through crackvision.skeleton -> crackvision.paths -> crackvision.path3d and proves the chain reaches a clean, schema-valid empty result with no crash and no fabricated crack, documented in docs/integration/safety/EMPTY_MASK.md.

- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Integration & commissioning: **6/14** tasks accepted; whole project: **132/170**.
- REQ-INT-2: 17/26 contributing tasks done
- REQ-OPS-1: 19/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (empty-mask ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “INT-03.1 adds a self-contained failure-injection test that feeds an all-zero prediction mask through crackvision.skeleton/paths/path3d and verifies the chain reaches a clean, schema-valid empty result (skeleton_pixels=0, component_count=0, paths3d components=[] with execution_eligible=false and no_valid_points). I re-ran the script independently and inspected the raw output JSON directly (not just the script's own assertions), confirming all claims: exit codes 0/0/0, no tracebacks, full §10 schema compliance, and no fabricated points/claims. Git diff confirms only the two i
…[295 chars clipped]”

## What it unlocks next

- Closer: INT-03.5 — Assemble the safety case + unified failure-injection runner (still needs INT-03.2, INT-03.3, INT-03.4)
