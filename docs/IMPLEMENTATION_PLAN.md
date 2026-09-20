# IMPLEMENTATION_PLAN.md — how this gets built

**Planned:** 2026-09-16 (Opus architecture session) · **Implemented by:** Claude Sonnet agents,
one task card per session.

This document is the narrative bridge between `ARCHITECTURE.md` (what) and `task_cards/` (how).
If you are an implementation agent, read `task_cards/AGENT_INSTRUCTIONS.md` first — this file is
orientation, not instruction.

---

## 1. Strategy

The planning effort was spent **once**, deliberately, on a more capable model, so that sixteen
cheaper implementation sessions can *execute* rather than *design*. Concretely, that means every card
already specifies:

- exact file paths, CLI flags and exit codes (`INTERFACES.md`)
- the chosen library for each job and why (Pillow not OpenCV for conversion; `predict_from_modelfolder`
  not `predict`; stdlib `csv` not pandas)
- the upstream facts that would otherwise need re-researching (`ARCHITECTURE.md` §4)
- what **not** to build, and the ADR that explains why (`adr/008`, `adr/009`, `adr/010`)
- verification commands whose output can be pasted into a completion report

A Sonnet agent that finds itself doing architecture research has almost certainly missed a document.

## 2. Phasing

| Phase | Cards | Outcome |
|---|---|---|
| **P0 — Foundation** | TC-001, TC-002 | The tree, the config/logging primitives, the isolated Python 3.11 env, and the `env.sh` scrub that makes this machine usable at all. |
| **P1 — Model online** | TC-003, TC-004, TC-005, TC-006 | Environment provably correct, OpenCrack downloaded and semantically validated, and the model **actually run once** on a synthetic image. This is the first real risk retired. |
| **P2 — The pipeline** | TC-007, TC-008, TC-009, TC-010 | Arbitrary imagery → mask → overlay → skeleton, each component independently testable. |
| **P3 — Usable** | TC-011 | `./run_test.sh`. The project becomes something a human runs repeatedly while looking at pictures. |
| **P4 — Camera** | TC-012, TC-013 | D405 capture, aligned, with lossless depth and full intrinsics — built and tested with **no camera attached**. |
| **P5 — Hardening** | TC-014, TC-015, TC-016 | Integration and hygiene tests, evaluation-manifest tooling, reproducible docs. |

**After P5**, the implementation phase is done and the project's actual question opens: collect
D405 imagery per `D405_TEST_PLAN.md` and run the Level-3 evaluation.

## 3. Critical path

```
TC-001 → TC-002 → TC-004 → TC-005 → TC-006 → TC-008 → TC-011 → TC-014 → TC-016
```

Nine of the sixteen cards. TC-003, TC-007, TC-009, TC-010, TC-012, TC-013 and TC-015 hang off it and
can be parallelised (see `TASK_INDEX.md` §Parallelisation), but sequential execution is recommended:
the ordering cost is small and it eliminates every merge hazard on the shared append-only files.

## 4. The three risks that actually decide this project

1. **`env.sh` must work.** ROS Humble leaks Python 3.10 paths into a Python 3.11 interpreter on this
   machine — verified, not hypothetical. If TC-002's scrub is wrong, every subsequent card fails in a
   confusing way. TC-003 turns the scrub into an automated assertion precisely so this cannot rot.
2. **The nnU-Net input/output contract.** Three facts drive almost every acceptance criterion:
   JPEG is rejected; input must be exactly 3-channel RGB in a single `_0000.png`; output PNGs carry
   the values `{0,1}` and look black. TC-007, TC-008 and TC-009 each enforce one end of this.
3. **Domain fit is the open question, not a bug.** OpenCrack is a pavement/close-up-surface model
   that over-fires on joints and mortar. Whether it works on D405 imagery at 10–20 cm is the thing
   we are here to find out. No card may "improve" the output to make it look better — that would
   destroy the measurement. Hence `adr/010` and the deliberate absence of accuracy assertions in L2.

## 5. What "done" means for this phase

- [ ] `./env.sh python scripts/check_env.py` → all PASS
- [ ] `./env.sh python scripts/smoke_test.py` → `SMOKE TEST PASSED`
- [ ] `./env.sh pytest tests/ -v` → green with no GPU and no camera
- [ ] `./run_test.sh` on a folder of images → comparison PNGs a human can look at
- [ ] `./env.sh python -m crackvision.realsense_capture --synthetic 3` → valid RGB-D + metadata
- [ ] `docs/RUNBOOK.md` reproduces the whole thing from a clean clone
- [ ] every card has an honest entry in `docs/COMPLETION_LOG.md`

**Not** part of done: any accuracy number, any real crack imagery, any 3D, any robot motion.

## 6. Estimated effort

Not wall-clock estimates — relative sizing for sequencing.

| Size | Cards |
|---|---|
| Trivial | — |
| Small | TC-001, TC-005, TC-012 |
| Small–Medium | TC-003, TC-004, TC-010, TC-015, TC-016 |
| Medium | TC-002, TC-006, TC-007, TC-008, TC-009, TC-011, TC-014 |
| Complex | TC-013 |

TC-013 is the only genuinely complex card: it combines lazy imports, hardware absence, binary
fidelity (16-bit depth round-trip) and a metadata schema that a later 3D stage depends on. Give it a
full session. TC-002 is medium but **highest-consequence** — a mistake there poisons everything after.

## 7. What a Sonnet agent should do when the plan is wrong

Adapt to minor upstream drift (a renamed kwarg, a changed function signature) and record it under
`ISSUES:`. **Stop and report BLOCKED** when a *verified fact* in `ARCHITECTURE.md` §4 turns out to be
false — a missing model file, a changed `dataset.json`, a removed CLI entry point. Those are real
findings that invalidate design decisions, and papering over them is worse than stopping.

Never weaken a test, never `sudo`, never kill a GPU process, never touch another project's conda env.
