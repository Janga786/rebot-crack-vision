# ADR-008: This phase ends at a one-pixel skeleton raster — no graph, no ordered path

**Status:** accepted · **Date:** 2026-09-16
Status: Superseded in part by ADR-011 (2026-09-28)

## Context
The eventual robot pipeline needs an *ordered* 3D path. Getting there from a binary mask means
skeletonize → graph construction → spur pruning → branch resolution → path ordering → smoothing.
Each stage has real design choices (which branch is "the crack"? how long is a spur worth keeping?
how do you order a Y-junction?) that cannot be answered sensibly before we know what the masks
actually look like on D405 imagery.

## Decision
The perception endpoint for this phase is:

```
prediction (0/1) → remove_small_objects(min_size=64) → skeletonize() → 1-px binary raster
```

Ship exactly that. **Do not** implement branch/endpoint detection, graph construction, path ordering,
spur pruning, polyline fitting, spline smoothing, or `skan` integration. Emit a small
`_skeleton_stats.json` (component counts, pixel counts) so the later design has data to reason from.

## Consequences
+ Small, testable, unambiguous card (TC-010) with no open design questions.
+ The stats files become the evidence base for designing the traversal stage properly later.
− A downstream consumer cannot yet get an ordered path — correct and intended for this phase.
− A later agent may be tempted to "helpfully" add traversal. `AGENT_INSTRUCTIONS.md` rule 6 and the
  Out-of-scope section of TC-010 forbid it explicitly.

## Revisit when
Level-3 evaluation has produced real masks and we can see the actual branching statistics. Path
ordering then becomes its own design session, not a bolted-on function.
