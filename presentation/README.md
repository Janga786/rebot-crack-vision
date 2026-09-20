# Presentation — *Finding the crack, then reaching it*

Everything needed to give (or rebuild) a talk on the crack-vision work.

| File | What it is |
|---|---|
| [`index.html`](index.html) | The deck. Open it in any browser — arrow keys or space to move, `F` for fullscreen, `Ctrl-P` to export a PDF. No internet, no build step, no dependencies. |
| [`SLIDES.md`](SLIDES.md) | The same running order as speaker notes, with every number you'd be asked about and a "questions you should expect" section. Use this to rebuild the deck in PowerPoint or Google Slides. |
| [`deck.pdf`](deck.pdf) | Pre-exported PDF of the deck, for emailing or projecting without a browser. |
| `figures/` | Every figure as a standalone PNG at 170 dpi — drop them straight into any slide tool. |
| `assets/` | The source images the figures are built from (real smoke-test inputs and predictions). |
| `tools/` | The scripts that generate the figures. Figures are **generated, never screenshotted**, so they cannot drift away from what the code actually does. |
| `demo/` | The vision → 3D path → arm-joint-trajectory demo (see the honesty note below). |
| `sim/`, `renders/` | The Isaac Sim scene and its renders — see [`sim/README.md`](sim/README.md) for the environment, runtime and the texture-orientation investigation. |

## Regenerating the figures

```bash
cd ..                                              # project root
./env.sh python presentation/tools/make_figures.py            # fig01, fig02, fig03, fig06
./env.sh python presentation/demo/crack_to_path.py             # crack_path_3d.json
./env.sh python presentation/demo/path_to_joint_trajectory.py  # joint_trajectory.json (needs the above)
./env.sh python presentation/tools/make_trajectory_figures.py  # fig04, fig05 (needs the above)
```

All of that runs on CPU in a few seconds and needs no GPU and no model download — it works from the
committed artefacts in `assets/`. Rebuilding the Isaac Sim renders is separate and needs the
`isaaclab` conda environment; see [`sim/README.md`](sim/README.md).

## What is real and what is scaffolding

This matters more than any figure, and the deck says it on slide 13 ("What is measured, and what is
still declared"):

**Real** — the crack mask, the centerline and every number quoted about them come from an actual
OpenCrack nnU-Net v2 run on the RTX 3090, reproducible with `./env.sh python scripts/smoke_test.py`.
The crack *image* is synthetic (drawn by the project's own smoke test), because no RealSense D405 is
attached yet.

**Scaffolding** — `demo/` and `sim/` follow the vision output into a robot arm so the capstone story
can be shown end to end. They assume a flat surface at a known pose and a declared camera→robot
transform. Depth projection is deferred by [`docs/adr/002`](../docs/adr/002-rgb-first-depth-later.md),
ordered path extraction by [`adr/008`](../docs/adr/008-mask-to-skeleton-endpoint.md), and robot
integration by [`adr/009`](../docs/adr/009-robot-ros-integration-deferred.md). Nothing in `src/`
imports any of it.

## Figure index

| File | Slide | Shows |
|---|---|---|
| `fig06_pipeline_diagram.png` | 4 | the phase-1 data path and the deferred downstream |
| `fig01_pipeline_result.png` | 5 | input → mask → overlay → centerline, from a real GPU run |
| `fig02_negative_control.png` | 6 | the blank frame returning zero crack pixels, beside the positive case |
| `fig03_scoreboard.png` | 8 | all 16 task cards and their review status |
| `fig04_trajectory.png` | 10 | the 18 vision waypoints + the solved 6-joint trajectory (max error 0.089 mm) |
| `fig05_workspace.png` | — (backup) | 3D reach envelope: coupon, crack path, arm at 3 sampled poses — not in the deck, kept as a backup/appendix slide if a technical audience asks to see the workspace |
| `renders/scene_overview.png` | 11 | Isaac Sim: the real B601-DM URDF beside the textured coupon and path |
| `renders/topdown_path.png`, `renders/closeup_coupon.png` | 12 | the alignment proof shot + a close-up; see the honesty note in `sim/README.md` for the small residual visual gap that was measured, not hand-waved |

Regenerate `fig04`/`fig05` with `./env.sh python presentation/tools/make_trajectory_figures.py`
(needs `presentation/demo/joint_trajectory.json`, produced by
`presentation/demo/path_to_joint_trajectory.py`). Rebuild the Isaac Sim renders with
`presentation/sim/build_scene.py` — see that directory's own README for the environment and runtime.
