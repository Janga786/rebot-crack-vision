# PERC-02 — Skeleton graph: junctions, endpoints, spur pruning

**Accepted:** 2026-09-28 · **Work package:** Perception (L-PERC) · **Track:** software · **Verified commit:** [`ec8e94d`](https://github.com/Janga786/rebot-crack-vision/commit/ec8e94de9aa9c35bffcec3f73b3e6bfb78ed6ed4)

## What was done

Built and tested a new perception module that turns a 1-pixel-wide crack skeleton into a graph of junctions, endpoints and loops with the full pixel path along each edge, all in original image coordinates. It correctly classifies Y/X/T/loop/spur and multi-component synthetic skeletons, prunes short spur branches by a configurable pixel-length threshold without ever deleting a component's main path, and serialises the graph to deterministic JSON. All 21 new tests pass, and the full project test suite (139 tests) still passes.

Files changed:
- [src/crackvision/skeleton_graph.py](https://github.com/Janga786/rebot-crack-vision/blob/ec8e94de9aa9c35bffcec3f73b3e6bfb78ed6ed4/src/crackvision/skeleton_graph.py) (+274 / −0)
- [tests/test_skeleton_graph.py](https://github.com/Janga786/rebot-crack-vision/blob/ec8e94de9aa9c35bffcec3f73b3e6bfb78ed6ed4/tests/test_skeleton_graph.py) (+366 / −0)

Commits: [`ec8e94d`](https://github.com/Janga786/rebot-crack-vision/commit/ec8e94de9aa9c35bffcec3f73b3e6bfb78ed6ed4)

## Why it was done

A tested pixel-graph representation of the 1-px skeleton with configurable spur pruning, in the original frame.

- Requirement **REQ-PERC-2**: 1-px skeleton, branch handling and ordered crack paths in the original pixel frame

## How it moves the project forward

- Perception: **8/16** tasks accepted; whole project: **20/74**.
- REQ-PERC-2: 1/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (pytest ✔, suite ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “PERC-02 implements an 8-connected pixel graph over a {0,255} skeleton with junction-cluster merging, endpoint/junction/loop/isolated classification, spur pruning, and deterministic JSON serialisation, exactly as scoped (only skeleton_graph.py and its test file touched). I re-ran both scheduler-trusted test commands (21/21 and 139 passed/2 skipped) and additionally fuzz-tested build_graph/prune_spurs against 50+ real skimage.skeletonize outputs and several hand-built adversarial shapes (theta graphs, lollipops, cascading spur chains): all edge paths stay pixel-contiguous, no
…[336 chars clipped]”

## What it unlocks next

- **Ready to start:** PERC-03 — Ordered crack paths + path contract
