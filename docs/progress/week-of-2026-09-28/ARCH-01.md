# ARCH-01 — ADR-011: Phase-2 scope lifts the phase-1 deferrals in a controlled order

**Accepted:** 2026-09-28 · **Work package:** Root architect (A0) · **Track:** software · **Verified commit:** [`be686c1`](https://github.com/Janga786/rebot-crack-vision/commit/be686c1f5dcbe079b9d63f8ffd4cb43a9e4a4ef9)

## What was done

Documented the project's move from phase-1 (perception-only) to phase-2 (full pipeline) scope in a new architecture decision record, ADR-011, which spells out the order deferred work resumes in: ordered crack paths, then 3D calibration, then motion planning, then gated real-robot commissioning. The three earlier ADRs that had deferred that work were each marked superseded-in-part, and the project's architecture doc now points to this new record. All required acceptance checks passed.

Files changed:
- [docs/ARCHITECTURE.md](https://github.com/Janga786/rebot-crack-vision/blob/be686c1f5dcbe079b9d63f8ffd4cb43a9e4a4ef9/docs/ARCHITECTURE.md) (+7 / −0)
- [docs/adr/002-rgb-first-depth-later.md](https://github.com/Janga786/rebot-crack-vision/blob/be686c1f5dcbe079b9d63f8ffd4cb43a9e4a4ef9/docs/adr/002-rgb-first-depth-later.md) (+1 / −0)
- [docs/adr/008-mask-to-skeleton-endpoint.md](https://github.com/Janga786/rebot-crack-vision/blob/be686c1f5dcbe079b9d63f8ffd4cb43a9e4a4ef9/docs/adr/008-mask-to-skeleton-endpoint.md) (+1 / −0)
- [docs/adr/009-robot-ros-integration-deferred.md](https://github.com/Janga786/rebot-crack-vision/blob/be686c1f5dcbe079b9d63f8ffd4cb43a9e4a4ef9/docs/adr/009-robot-ros-integration-deferred.md) (+1 / −0)
- [docs/adr/011-phase2-full-pipeline-scope.md](https://github.com/Janga786/rebot-crack-vision/blob/be686c1f5dcbe079b9d63f8ffd4cb43a9e4a4ef9/docs/adr/011-phase2-full-pipeline-scope.md) (+64 / −0)

Commits: [`be686c1`](https://github.com/Janga786/rebot-crack-vision/commit/be686c1f5dcbe079b9d63f8ffd4cb43a9e4a4ef9)

## Why it was done

The repo's own decision record says the full pipeline is now in scope and in which order deferrals lift.

- Requirement **REQ-OPS-2**: Reproducible install/config/tests/evaluation results, handoff docs, requirements→evidence map, honest readiness report

## How it moves the project forward

- Root architect: **1/1** tasks accepted; whole project: **10/69**.
- REQ-OPS-2: 1/3 contributing tasks done
- Verification: automated checks run by the pipeline itself (adr ✔, superseded ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “ADR-011 correctly documents phase-2 scope lift per the template, cites G-001, keeps ADR-010 in force, states the commissioning gate, and ADR-002/008/009 each got exactly one added status line with ARCHITECTURE.md gaining only an appended section. Both scheduler checks pass and no out-of-scope files were touched.”

## What it unlocks next

- **Ready to start:** GEOM-01 — Frames, units and pixel conventions (ADR-012 + geometry contract)
- Closer: MOT-02 — ROS 2 overlay workspace + headless MoveIt mock planning (still needs MOT-01)
- Closer: PERC-02 — Skeleton graph: junctions, endpoints, spur pruning (still needs TC-010)
