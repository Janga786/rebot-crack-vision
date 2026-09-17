# TASK_INDEX.md — dependency map and status board

**Statuses:** `READY` · `BLOCKED` (prerequisites unmet) · `IN PROGRESS` · `COMPLETE` · `PARTIAL` · `SKIPPED`

Each agent updates this table at the end of its card: set its own row, then flip any card whose
prerequisites are now all satisfied from `BLOCKED` to `READY`.

---

## Status board

| ID | Task | Depends on | Complexity | Status | Agent |
|---|---|---|---|---|---|
| TC-001 | Project scaffolding and Git hygiene | — | SMALL | COMPLETE | Sonnet |
| TC-002 | Python environment, dependencies, `env.sh` | TC-001 | MEDIUM | COMPLETE | Sonnet |
| TC-003 | Environment verification (Level 1) | TC-002 | SMALL–MED | COMPLETE | Sonnet |
| TC-004 | OpenCrack model acquisition | TC-002 | SMALL–MED | **READY** | Sonnet |
| TC-005 | nnU-Net wiring + semantic validation | TC-004 | SMALL | BLOCKED | Sonnet |
| TC-006 | Synthetic end-to-end smoke test (Level 2) | TC-003, TC-005 | MEDIUM | BLOCKED | Sonnet |
| TC-007 | Input preparation + naming contract | TC-002 | MEDIUM | **READY** | Sonnet |
| TC-008 | Batch inference runner | TC-005, TC-006, TC-007 | MEDIUM | BLOCKED | Sonnet |
| TC-009 | Visualization pipeline | TC-007 | MEDIUM | BLOCKED | Sonnet |
| TC-010 | Skeletonization utility | TC-002 (TC-007 for real data) | SMALL–MED | BLOCKED | Sonnet |
| TC-011 | One-command runner `run_test.sh` | TC-003, TC-007, TC-008, TC-009, TC-010 | MEDIUM | BLOCKED | Sonnet |
| TC-012 | RealSense software validation | TC-002 | SMALL | **READY** | Sonnet |
| TC-013 | D405 capture utility | TC-012 | **COMPLEX** | BLOCKED | Sonnet |
| TC-014 | Integration + contract + hygiene tests | TC-010, TC-011, TC-013 | MEDIUM | BLOCKED | Sonnet |
| TC-015 | Evaluation-manifest tooling | TC-001 (TC-013 for fixtures) | SMALL–MED | BLOCKED | Sonnet |
| TC-016 | Documentation and reproducibility | TC-011, TC-014, TC-015 | SMALL–MED | BLOCKED | Sonnet |

**Complexity key** — `SMALL`: one file, narrow contract. `MEDIUM`: a module plus tests, some judgement.
`COMPLEX`: several interacting concerns (TC-013: lazy imports, hardware absence, binary fidelity,
metadata schema).

---

## Dependency graph

```mermaid
graph TD
    TC001[TC-001 scaffolding] --> TC002[TC-002 env + env.sh]
    TC001 --> TC015[TC-015 manifest tooling]
    TC002 --> TC003[TC-003 check_env]
    TC002 --> TC004[TC-004 fetch model]
    TC002 --> TC007[TC-007 prepare_inputs]
    TC002 --> TC010[TC-010 skeleton]
    TC002 --> TC012[TC-012 realsense check]
    TC004 --> TC005[TC-005 verify model]
    TC003 --> TC006[TC-006 smoke test]
    TC005 --> TC006
    TC005 --> TC008[TC-008 inference]
    TC006 --> TC008
    TC007 --> TC008
    TC007 --> TC009[TC-009 visualize]
    TC003 --> TC011[TC-011 run_test.sh]
    TC008 --> TC011
    TC009 --> TC011
    TC010 --> TC011
    TC012 --> TC013[TC-013 D405 capture]
    TC010 --> TC014[TC-014 integration tests]
    TC011 --> TC014
    TC013 --> TC014
    TC013 -.fixtures.-> TC015
    TC011 --> TC016[TC-016 docs]
    TC014 --> TC016
    TC015 --> TC016
```

No cycles. TC-013→TC-015 is a soft edge (synthetic fixtures are convenient, not required — TC-015
can hand-write fixtures and says so).

---

## Recommended execution order (strictly sequential — the safe default)

```
TC-001 → TC-002 → TC-003 → TC-004 → TC-005 → TC-006 → TC-007 → TC-008
      → TC-009 → TC-010 → TC-011 → TC-012 → TC-013 → TC-014 → TC-015 → TC-016
```

This order is what the cards' `NEXT CARD:` fields point to. Follow it unless you are deliberately
parallelising.

---

## Parallelisation

Once **TC-002** is COMPLETE, six cards become independently runnable — they share no files:

| Wave | Cards | Why they are safe together |
|---|---|---|
| **A** (after TC-002) | **TC-003**, **TC-004**, **TC-007**, **TC-010**, **TC-012**, **TC-015** | disjoint file sets: `scripts/check_env.py` · `scripts/fetch_model.py`+`config/model_manifest.json` · `src/.../naming.py`+`prepare_inputs.py` · `src/.../skeleton.py` · `scripts/check_realsense.py` · `tools/dataset_manifest.py` |
| **B** | **TC-005** (after TC-004), **TC-009** (after TC-007), **TC-013** (after TC-012) | still disjoint |
| **C** | **TC-006** (after TC-003+TC-005) | first GPU use |
| **D** | **TC-008** (after TC-005, TC-006, TC-007) | touches `scripts/smoke_test.py` — must not overlap with TC-006 |
| **E** | **TC-011** (after TC-003, 007, 008, 009, 010) | orchestrator |
| **F** | **TC-014** → **TC-016** | must be last; TC-014 reads every component |

**Shared files that force serialisation:**
- `docs/COMPLETION_LOG.md` and `task_cards/TASK_INDEX.md` — appended by *every* card. Parallel agents
  must append, never rewrite; resolve any conflict by keeping both entries.
- `tests/conftest.py` — created by TC-007, extended by TC-014. Do not run those two concurrently.
- `scripts/smoke_test.py` — created by TC-006, edited by TC-008 (one substitution only).
- `tests/test_realsense_import.py` — created by TC-012, extended by TC-013.
- `README.md` — quickstart section by TC-011, full rewrite by TC-016.

**Recommendation:** run sequentially. The ordering above costs little and removes every merge
hazard. Parallelise only Wave A, and only if you are running agents concurrently on purpose.

---

## File ownership

One card creates each file. Nothing else may create or rewrite it.

| File / directory | Created by | May be modified by |
|---|---|---|
| `.gitignore`, `pyproject.toml`, `config/project.yaml` | TC-001 | — |
| `src/crackvision/config.py`, `logging_setup.py`, `__init__.py` | TC-001 | — |
| `env.sh`, `requirements/**` | TC-002 | TC-012 (realsense pin), TC-016 (pin core) |
| `scripts/check_env.py` | TC-003 | — |
| `scripts/fetch_model.py`, `config/model_manifest.json` | TC-004 | — |
| `scripts/verify_model.py` | TC-005 | — |
| `scripts/smoke_test.py` | TC-006 | TC-008 (swap to `run_inference()` only) |
| `src/crackvision/naming.py`, `prepare_inputs.py` | TC-007 | — |
| `tests/conftest.py` | TC-007 | TC-014 (add fixtures) |
| `tests/test_naming.py`, `test_prepare_inputs.py` | TC-007 | — |
| `src/crackvision/inference.py` | TC-008 | — |
| `src/crackvision/visualize.py`, `tests/test_visualize.py` | TC-009 | — |
| `src/crackvision/skeleton.py`, `tests/test_skeleton.py` | TC-010 | — |
| `run_test.sh`, `scripts/summarize_run.py` | TC-011 | — |
| `scripts/check_realsense.py` | TC-012 | — |
| `tests/test_realsense_import.py` | TC-012 | TC-013 (add tests) |
| `src/crackvision/realsense_capture.py` | TC-013 | — |
| `tests/test_integration_smoke.py`, `test_contracts.py` | TC-014 | — |
| `tools/dataset_manifest.py`, `tests/test_dataset_manifest.py` | TC-015 | — |
| `README.md` | TC-001 (placeholder) | TC-011 (quickstart), TC-016 (full) |
| `docs/COMPLETION_LOG.md` | TC-001 | **every card — append only** |
| `task_cards/TASK_INDEX.md` | planning | every card (status cells); re-synced from canonical state by the harness |
| `docs/ENVIRONMENT_SNAPSHOT.md`, `RUNBOOK.md`, `LICENSE-NOTICES.md` | TC-016 | — |
| `docs/ARCHITECTURE.md`, `INTERFACES.md`, `RISKS.md`, `VALIDATION_PLAN.md`, `D405_TEST_PLAN.md`, `MACHINE_STATE.md`, `IMPLEMENTATION_PLAN.md`, `adr/**` | planning session | **nobody** (TC-016 may *append* an "Implementation notes" section to `ARCHITECTURE.md`) |
| `docs/ORCHESTRATION.md` | harness | **nobody** |
| `.task_orchestrator/**`, `~/.local/bin/cv-*` | harness | **nobody — this is the review gate** |
| `docs/REVIEW_LOG.md`, `.task_orchestrator/reviews/**` | `cv-review` (Opus) | **no implementation agent** |
| `task_cards/TC-*.md`, `AGENT_INSTRUCTIONS.md` | planning session | **nobody** |

---

## Progress

```
TC-001, TC-002 █░░░░░░░░░░░░░░░  2 / 16 complete
```

TC-002 delivered the `crackvision` conda env (Python 3.11), CUDA torch 2.14.0+cu126, nnunetv2 2.8.1,
the pinned imaging stack, and the `env.sh` scrubbing launcher. TC-003, TC-004, TC-007, TC-010, TC-012
and TC-015 are now READY (Wave A, see Parallelisation above).
