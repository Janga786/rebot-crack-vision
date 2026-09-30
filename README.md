# rebot_crack_vision

**Zero-shot crack segmentation for a reBot B601-DM concrete-inspection arm.**

> **Current status (Phase 2, 2026-09-30).**
> - Phase 1 below is complete.
> - Phase 2 ([ADR-011](docs/adr/011-phase2-full-pipeline-scope.md)) extends it to ordered crack paths,
>   depth → 3D, eye-in-hand hand-eye calibration, MoveIt planning and gated commissioning. The work is
>   tracked as cards under [`plan/`](plan) and weekly write-ups in [`docs/progress/`](docs/progress).
> - The end-of-arm model is in [ADR-014](docs/adr/014-end-effector-frames-and-task-phases.md): a wrist D405
>   on Seeed's stock mount, a `tool_tip` distinct from the grasp-centre `gripper_tcp`, and separate
>   view and trace phases.
> - Everything robot-side is simulation/mock only. The camera extrinsic, tool tip and workcell are
>   nominal priors until measured.
> - Physical steps wait for the operator: D405 and arm adapter connection, calibration, and powered
>   commissioning.

This repository builds the Phase-1 perception pipeline that turns an RGB image of a concrete/masonry
surface into a binary crack mask and a one-pixel crack skeleton, using publicly released weights
with no training of our own, and answers one honest question: *can a zero-shot, publicly released
crack-segmentation model do useful work on this hardware and imagery, or does the project need to
fund its own labelling and training effort?* It deliberately stops short of depth projection,
hand-eye calibration, ROS/robot integration and any model fine-tuning — each deferral is recorded as
an ADR in [`docs/adr/`](docs/adr).

<p align="center">
  <img src="presentation/figures/fig01_pipeline_result.png" width="100%"
       alt="Input image, binary mask, overlay and centerline from a real GPU inference run">
</p>

## Quickstart

1. Put images in `data/input_originals/`
2. Run `./run_test.sh`
3. Open `data/comparisons/`

(Never run scripts against a bare `python` — ROS Humble leaks a Python 3.10 `PYTHONPATH` into any
interpreter on this box, including this project's 3.11 env. Always go through `./env.sh` or
`./run_test.sh`, which wrap it for you. See [`docs/RUNBOOK.md`](docs/RUNBOOK.md) for full setup from
a clean clone.)

## What this does

```
   RGB image                binary mask              1-pixel skeleton
data/input_originals/  ->  OpenCrack nnU-Net v2  ->  {0,1} mask  ->  overlay / comparison
                              (2D, fold 0)          data/predictions/    data/overlays/
                                                                          data/comparisons/
                                                            |
                                                            v
                                              remove_small_objects + skeletonize
                                                            |
                                                            v
                                                data/skeletons/ (1-px {0,255} raster)
```

Every stage preserves the frame invariant: a `(row, col)` in the mask is the same `(row, col)` in
the original image — nothing resizes, crops or pads (`docs/INTERFACES.md` §0.5). That invariant is
what later lets a `(row, col)` be looked up in an aligned depth frame to build a 3D robot
trajectory — the next phase, not this one.

## Current scope / Not yet implemented

Phase 1 (`task_cards/TC-001`…`TC-016`, this repository's original deliverable) is RGB-in,
mask/skeleton-out, on files on disk. It deliberately does **not** include:

- **Depth projection or camera calibration** — RGB is consumed first; depth comes later
  (`docs/adr/002-rgb-first-depth-later.md`).
- **Ordered path extraction from the skeleton** — the 1-pixel raster is this phase's perception
  endpoint; turning it into an ordered traversal path is out of scope here
  (`docs/adr/008-mask-to-skeleton-endpoint.md`).
- **Robot control, hand-eye calibration or ROS 2 integration**
  (`docs/adr/009-robot-ros-integration-deferred.md`).
- **Any model training or fine-tuning** — the OpenCrack weights are used exactly as released
  (`docs/adr/010-no-training-this-phase.md`).
- **A quantitative zero-shot accuracy claim on real D405 imagery** — no camera has been attached to
  this development machine; see [`docs/D405_TEST_PLAN.md`](docs/D405_TEST_PLAN.md) for the planned
  evaluation and `tools/dataset_manifest.py` for the manifest tooling that will back it.

A later phase of this same repository (`docs/adr/011-phase2-full-pipeline-scope.md`,
`docs/adr/012-frames-and-conventions.md`) has since started lifting these deferrals in dependency
order (depth backprojection, frame conventions, robot motion planning); that work lives alongside
this pipeline but is outside what Phase 1 / `TC-016` documents. `docs/ARCHITECTURE.md` and
`docs/TECHNICAL_APPROACH.md` are the entry points if you want the fuller picture.

## Setup from scratch

See [`docs/RUNBOOK.md`](docs/RUNBOOK.md) for numbered, copy-pasteable steps from a clean clone to a
first result, plus a troubleshooting table for the failure modes newcomers actually hit.

## Commands

Every CLI accepts the common flags documented in `docs/INTERFACES.md` §0.3
(`--root`, `--config`, `-v/--verbose`, `-q/--quiet`, `--dry-run`, `--skip-existing`) and exits
0/1/2/3 per `docs/INTERFACES.md` §0.2. Always invoke through `./env.sh`.

| Command | Purpose |
|---|---|
| `./run_test.sh` | One-command pipeline: `check_env` → `prepare_inputs` → `inference` → `visualize` → `skeleton` → `summarize_run`. |
| `./env.sh python scripts/check_env.py` | Level-1 environment check (Python/CUDA/nnU-Net/model files). |
| `./env.sh python scripts/fetch_model.py` | Downloads the OpenCrack nnU-Net weights and writes `config/model_manifest.json`. |
| `./env.sh python scripts/verify_model.py` | Verifies the on-disk model tree against the manifest (`--check-hashes` for full sha256 verification). |
| `./env.sh python scripts/smoke_test.py` | Level-2 synthetic end-to-end smoke test — real checkpoint, synthetic images, no user data needed. |
| `./env.sh python -m crackvision.prepare_inputs` | Converts arbitrary imagery in `data/input_originals/` to 3-channel RGB PNGs and writes `data/case_map.json`. |
| `./env.sh python -m crackvision.inference` | Runs `nnUNetv2_predict_from_modelfolder` over `data/nnunet_input/`. |
| `./env.sh python -m crackvision.visualize` | Renders `data/predictions/` into human-viewable masks/overlays/comparisons. |
| `./env.sh python -m crackvision.skeleton` | Cleans and skeletonizes a mask to a 1-pixel centerline in `data/skeletons/`. |
| `./env.sh python scripts/summarize_run.py` | Prints the per-case summary table `run_test.sh` uses at the end of a run. |
| `./env.sh python scripts/check_realsense.py` | Verifies the RealSense software stack; works with no camera attached. |
| `./env.sh python scripts/capture_d405.py` | Captures depth-aligned RGB + raw depth from a D405 (or `--synthetic` with no hardware). |
| `./env.sh python tools/dataset_manifest.py` | `init`/`scan`/`validate`/`summary` for the D405 evaluation manifest (`docs/D405_TEST_PLAN.md`). |
| `./env.sh pytest tests/ -v` | The unit/integration/contract test suite (225 passed, 3 skipped as of this writing). |

## Project layout

```
src/crackvision/       prepare_inputs · inference · visualize · skeleton · naming · config · paths
                        (plus realsense_capture, recording, geometry, skeleton_graph — later phases)
scripts/                check_env · fetch_model · verify_model · smoke_test · check_realsense ·
                        capture_d405 · summarize_run
tools/                  dataset_manifest.py — D405 evaluation-manifest tooling
tests/                  unit, integration, and contract tests (no GPU or camera required)
docs/                   ARCHITECTURE · INTERFACES · RISKS · VALIDATION_PLAN · D405_TEST_PLAN ·
                        TECHNICAL_APPROACH · MACHINE_STATE · RUNBOOK · ENVIRONMENT_SNAPSHOT ·
                        LICENSE-NOTICES · COMPLETION_LOG · REVIEW_LOG · adr/
task_cards/             the 16 Phase-1 specs this pipeline was built from, plus the (frozen) status
                        board — see `docs/ORCHESTRATION.md` for how planning moved on from here
presentation/           slide deck, figures, and the vision → path → arm demo
config/project.yaml     every tunable; no magic numbers in source
config/model_manifest.json   the pinned model download: repo id, revision, per-file sha256
env.sh                  the launcher — scrubs ROS off the interpreter path, then runs your command
run_test.sh             the one-command orchestrator (see Commands, above)
```

## Results interpretation

- **Predictions in `data/predictions/` are `{0,1}`-valued uint8 PNGs and LOOK SOLID BLACK. This is
  correct, not a bug.** `NaturalImage2DIO.write_seg` writes raw label values; at any normal display
  gamma, `{0,1}` is indistinguishable from black. Do not rescale these files.
- Use `data/overlays/` (crack highlighted on the original) and `data/comparisons/`
  (original | mask | overlay, side by side) to actually look at a result.
- The model over-fires on masonry/tile joints, mortar lines and other crack-like texture — see its
  own model card's Limitations section (cached locally at `models/opencrack-nnunet/README.md` after
  `fetch_model.py` runs).

## Attribution

- **OpenCrack nnU-Net** — `fadeevla/opencrack-nnunet`, CC-BY-4.0.
- **nnU-Net v2** — Isensee, F. et al., *nnU-Net: a self-configuring method for deep learning-based
  biomedical image segmentation*, Nature Methods 18, 203–211 (2021). Apache-2.0.

Full third-party notices and citations: [`docs/LICENSE-NOTICES.md`](docs/LICENSE-NOTICES.md). This
project's own licence has not yet been chosen (capstone coursework) — see that file for details.

Crack imagery in `presentation/assets/` is synthetic, generated by this project's own smoke test.

## Documentation index

**Start here:** [`docs/RUNBOOK.md`](docs/RUNBOOK.md) (setup) ·
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) (design) ·
[`docs/TECHNICAL_APPROACH.md`](docs/TECHNICAL_APPROACH.md) (how it works, written for the deck).

**Planning and contracts**
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — component boundaries, pipeline, environment/GPU
  strategy, known risks.
- [`docs/INTERFACES.md`](docs/INTERFACES.md) — normative I/O contracts every CLI/module must satisfy.
- [`docs/RISKS.md`](docs/RISKS.md) — risk register (R-01…, including the `{0,1}` prediction trap).
- [`docs/VALIDATION_PLAN.md`](docs/VALIDATION_PLAN.md) — the three testing levels this project uses.
- [`docs/D405_TEST_PLAN.md`](docs/D405_TEST_PLAN.md) — the planned zero-shot evaluation design.
- [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) — how this got built, card by card.
- [`docs/MACHINE_STATE.md`](docs/MACHINE_STATE.md) — target machine inventory (OS/CPU/GPU) as
  directly probed.
- [`docs/adr/`](docs/adr) — every deferral/design decision, numbered (`000`…`012`).

**Reproducibility**
- [`docs/RUNBOOK.md`](docs/RUNBOOK.md) — from a clean clone to a first result, plus troubleshooting.
- [`docs/ENVIRONMENT_SNAPSHOT.md`](docs/ENVIRONMENT_SNAPSHOT.md) — frozen dependency/version record.
- [`docs/LICENSE-NOTICES.md`](docs/LICENSE-NOTICES.md) — third-party model/library attribution.

**History and process**
- [`docs/COMPLETION_LOG.md`](docs/COMPLETION_LOG.md) — append-only build history (evidence per card).
- [`docs/REVIEW_LOG.md`](docs/REVIEW_LOG.md) — append-only independent-review audit history.
- [`docs/ORCHESTRATION.md`](docs/ORCHESTRATION.md) — how this project's planning/agent process works,
  including how it moved on from the original `task_cards/` board.
- [`task_cards/TASK_INDEX.md`](task_cards/TASK_INDEX.md) — the original Phase-1 dependency map and
  status board (frozen as history; see `docs/ORCHESTRATION.md` for the current one).
- [`task_cards/AGENT_INSTRUCTIONS.md`](task_cards/AGENT_INSTRUCTIONS.md) and `task_cards/TC-001.md`
  through `TC-016.md` — the individual Phase-1 task specs this pipeline was built from.

**Later-phase tracks** (beyond this document's scope; linked for completeness)
- [`docs/host/HOST_MANIFEST.md`](docs/host/HOST_MANIFEST.md) — host version pinning and drift checks.
- [`docs/motion/ROBOT_MODEL.md`](docs/motion/ROBOT_MODEL.md) — reBot B601-DM model reconciliation.
- [`docs/motion/ROS_WORKSPACE.md`](docs/motion/ROS_WORKSPACE.md) — ROS 2 overlay workspace + headless
  MoveIt mock planning.
- [`docs/llm/SELECTION.md`](docs/llm/SELECTION.md) — local coding model shortlist.
- [`docs/progress/README.md`](docs/progress/README.md) — the project's running progress log.

**Presentation**
- [`presentation/index.html`](presentation/index.html) — a self-contained slide deck (arrow keys to
  navigate, `Ctrl-P` to export a PDF); [`presentation/SLIDES.md`](presentation/SLIDES.md) carries the
  same content as speaker notes, and every figure is regenerated from committed artefacts by
  `presentation/tools/make_figures.py`.
