# ADR-011: Phase 2 lifts the phase-1 deferrals, in dependency order

**Status:** accepted · **Date:** 2026-09-28 · **Supersedes:** — · **Superseded by:** —

## Context
Phase 1 (cards TC-001…016) deliberately stopped at files on disk: RGB-only segmentation, a 1-px
skeleton raster with no ordered path, and no robot/ROS integration or training. Those deferrals are
recorded in ADR-002 (depth consumed after segmentation only), ADR-008 (skeleton raster is the
perception endpoint, no traversal), ADR-009 (robot control, calibration and ROS 2 are deferred) and
ADR-010 (no training this phase).

Goal **G-001** (`plan/goals.json`) now states the full scope in scope: D405 capture → crack
segmentation (OpenCrack) → ordered crack paths → calibrated 3D robot coordinates → MoveIt trajectory
planning → validated reBot B601-DM execution, with physical execution only through an
operator-enabled commissioning mode. The phase-1 deferrals in ADR-002/008/009 therefore have to lift.
The question this ADR answers is *in what order*, and what stays deferred.

## Decision
Lift the ADR-002/008/009 deferrals in the order the plan's own dependency graph encodes — each stage
consumes the previous stage's on-disk contract, so this is also the only order that is buildable:

1. **Ordered paths** (lifts the ADR-008 deferral): skeleton graph construction, spur pruning,
   branch/endpoint resolution and path ordering on the existing 1-px raster. The frame invariant
   (`INTERFACES.md` §0.5 — mask, skeleton and aligned depth share one pixel grid) is unchanged and
   still binds.
2. **Geometry / calibration** (lifts the ADR-002 deferral): depth→3D backprojection of the ordered
   2D path using the aligned D405 depth already captured under ADR-002, plus camera-to-robot
   hand-eye calibration. Depth still never feeds the segmentation network — only the post-segmentation
   3D lift changes.
3. **Motion planning** (partially lifts the ADR-009 deferral): MoveIt2 trajectory generation from the
   calibrated 3D path, running in simulation/planning-only mode.
4. **Gated commissioning** (the remainder of the ADR-009 deferral): real reBot B601-DM execution,
   available only behind the operator commissioning gate described below.

Constraints that do **not** change:
- OpenCrack remains the segmentation model (ADR-001); this ADR does not reopen model choice.
- The pixel contract in `INTERFACES.md` §0.5 is unchanged by any of these stages.
- **ADR-010 (no training this phase) stays in force.** It is not superseded by this ADR. Whether to
  fine-tune or retrain is decided later, by the real-imagery evaluation (PERC-08 / Level 3 in
  `docs/VALIDATION_PLAN.md`), not by this scope change.
- **Real robot motion happens only through the operator commissioning gate**: an explicit
  commissioning flag plus a typed operator confirmation, per `AGENT_INSTRUCTIONS.md` and the standing
  house rule. Every code path reachable without that gate defaults to simulation/dry-run; no card in
  this phase may command the physical arm outside it.

## Consequences
+ Card planning can now target the full pipeline stage by stage, each stage building strictly on the
  on-disk contract the previous stage already produced — no stage has to guess at an interface that
  doesn't exist yet.
+ ADR-002/008/009 stay as the historical record of *why* phase 1 stopped where it did; only their
  deferral clauses are superseded, not their underlying design decisions (RGB-only input, the frame
  invariant, the 1-px skeleton endpoint's stats-file contract, the on-disk hand-off contract).
+ ADR-010 remains binding, so no card in this phase may fine-tune or retrain OpenCrack without a
  separate decision once PERC-08 reports.
− The 3.10/3.11 interpreter split noted in ADR-009 is not resolved by this ADR; motion-planning and
  ROS-facing cards still need their own environment decision.
− Ordering paths, calibrating and planning motion before Level-3 evaluation has run on real D405
  imagery means some of this work could be discarded if OpenCrack turns out not to segment usably —
  an accepted risk, not a new one (see `docs/RISKS.md` #5).

## Revisit when
PERC-08's real-imagery evaluation reports, which is the trigger for reconsidering ADR-010; or if the
3.10/3.11 interpreter split makes the motion-planning/ROS stages unbuildable as split above, in which
case that stage needs its own ADR.
