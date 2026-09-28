# PERC-03.R1 — Repair PERC-03: Dense paths break and drop a skeleton pixel wherever edges are joined 

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-PERC-2"
  ],
  "scope": {
    "write": [
      "config/project.yaml",
      "docs/INTERFACES.md",
      "src/crackvision/paths.py",
      "tests/test_paths.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#3.4",
    "docs/INTERFACES.md#0.5",
    "presentation/demo/crack_to_path.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_paths.py -q"
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q",
        "timeout_s": 1200
      }
    ],
    "criteria": [
      "Across junctions, every main and branch `dense` path is 8-connected: consecutive points have max(|dr|,|dc|) ≤ 1. No pixel of the traversed PERC-02 edges is dropped. When edges end next to the junction instead of on it, the path goes through the node or cluster pixel. A test with a skeletonize-produced crossing or fork fixture asserts both properties.",
      "`grep -n '§3.5' src/crackvision/paths.py` returns nothing, and all contract references read §3.13.",
      "All acceptance criteria of PERC-03 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4
  },
  "priority": 95,
  "repairs": "PERC-03",
  "findings": [
    "deb0db9f5dd86d0c",
    "bb1b54d2151887c6"
  ],
  "id": "PERC-03.R1",
  "title": "Repair PERC-03: Dense paths break and drop a skeleton pixel wherever edges are joined ",
  "parent": "L-PERC",
  "outcome": "Resolve the review findings on PERC-03 while every acceptance criterion of PERC-03 still holds."
}
```

Repair work generated deterministically from review attempt `00034-PERC-03-review` of `PERC-03`.
Original card: `plan/cards/PERC-03.md` — its acceptance criteria must still hold.

## Finding 1 [major] Dense paths break and drop a skeleton pixel wherever edges are joined through a junction
- Evidence: src/crackvision/paths.py `_edge_chain_to_pixels`: `pixels.extend(oriented if not pixels else oriented[1:])` assumes each following edge starts on the previous edge's last pixel. PERC-02 edges can stop next to the junction pixel instead. Fork fixture (bar at row 20, cols 1–38; stem at col 20, rows 10–19; two diagonals forking from (10,20)): edge 3 ends at (20,19) and edge 4 starts at (20,21), both attached to junction node 3 at (19,20). The main dense output is [...,(20,18),(20,19),(20,22),(20,23),...], so (20,21) is dropped and the path jumps 3 px. On 30 random dilated crossing-line masks passed through skimage.skeletonize and build_case_paths(min_spur 5, rdp 1.5), 43/134 polylines had consecutive dense points with Chebyshev distance > 1.
- Consequence: The documented contract says `dense` has one entry per skeleton pixel, but real junctions produce paths that jump and miss pixels. `length_px` is undercounted, RDP runs on a broken chain, and GEOM-0x would deproject a centreline with holes and shortcuts at every crack junction. That is the common case for real cracks.
- Affected: src/crackvision/paths.py, tests/test_paths.py
- Acceptance condition: Across junctions, every main and branch `dense` path is 8-connected: consecutive points have max(|dr|,|dc|) ≤ 1. No pixel of the traversed PERC-02 edges is dropped. When edges end next to the junction instead of on it, the path goes through the node or cluster pixel. A test with a skeletonize-produced crossing or fork fixture asserts both properties.

## Finding 2 [minor] Normative module docstring and CLI help point to the wrong INTERFACES section (§3.5 instead of §3.13)
- Evidence: src/crackvision/paths.py lines 5 and 7 ('Schema ... is docs/INTERFACES.md §3.5', 'Policy (docs/INTERFACES.md §3.5)'), line 380 and lines 425/432 all cite §3.5. §3.5 is `scripts/check_env.py` (TC-003); the new contract is §3.13 (INTERFACES.md line 525). INTERFACES §3.13 says the module docstring is normative.
- Consequence: The normative text points readers to an unrelated contract, which makes the path contract hard to trace.
- Affected: src/crackvision/paths.py
- Acceptance condition: `grep -n '§3.5' src/crackvision/paths.py` returns nothing, and all contract references read §3.13.
