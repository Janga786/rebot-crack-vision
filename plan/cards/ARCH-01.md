# ARCH-01 — ADR-011: Phase-2 scope lifts the phase-1 deferrals in a controlled order

```json card
{
  "kind": "decision",
  "requirements": [
    "REQ-OPS-2"
  ],
  "scope": {
    "write": [
      "docs/adr/011-phase2-full-pipeline-scope.md",
      "docs/adr/002-rgb-first-depth-later.md",
      "docs/adr/008-mask-to-skeleton-endpoint.md",
      "docs/adr/009-robot-ros-integration-deferred.md",
      "docs/ARCHITECTURE.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "adr",
        "cmd": "test -f docs/adr/011-phase2-full-pipeline-scope.md"
      },
      {
        "id": "superseded",
        "cmd": "grep -q 'Superseded in part by ADR-011' docs/adr/002-rgb-first-depth-later.md && grep -q 'Superseded in part by ADR-011' docs/adr/008-mask-to-skeleton-endpoint.md && grep -q 'Superseded in part by ADR-011' docs/adr/009-robot-ros-integration-deferred.md"
      }
    ],
    "criteria": [
      "ADR-011 follows docs/adr/000-template.md and cites goal G-001 (plan/goals.json)",
      "Only the deferral clauses of ADR-002/008/009 are superseded; ADR-010 (no training) explicitly stays in force until PERC-08 decides",
      "Each of ADR-002/008/009 gains exactly one status line; no other edits",
      "ARCHITECTURE.md gains an appended 'Phase 2 scope' section pointing at ADR-011 and plan/; nothing else changes",
      "States that physical motion happens only via the operator commissioning gate"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 3
  },
  "priority": 75,
  "id": "ARCH-01",
  "title": "ADR-011: Phase-2 scope lifts the phase-1 deferrals in a controlled order",
  "parent": "A0",
  "outcome": "The repo's own decision record says the full pipeline is now in scope and in which order deferrals lift."
}
```

Write `docs/adr/011-phase2-full-pipeline-scope.md` (format: docs/adr/000-template.md). Context: phase 1 (cards TC-001…016)
deliberately deferred depth→3D (ADR-002), path ordering (ADR-008), calibration/MoveIt/ROS (ADR-009) and training (ADR-010).
Goal G-001 (plan/goals.json) now requires the full pipeline. Decision: lift ADR-002/008/009 deferrals in the order the plan's
dependencies encode (ordered paths → geometry/calibration → motion → gated commissioning); keep ADR-010 until the real-imagery
evaluation (PERC-08) says otherwise; OpenCrack stays the segmentation model; the pixel contract (INTERFACES §0.5) is unchanged;
real robot motion only through the operator commissioning gate. Then add one line `Status: Superseded in part by ADR-011 (2026-..)`
under the status line of ADR-002, ADR-008 and ADR-009, and append a short "## Phase 2 scope" section to docs/ARCHITECTURE.md.
