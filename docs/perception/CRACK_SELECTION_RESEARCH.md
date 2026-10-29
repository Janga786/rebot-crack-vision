# Picking the crack, not the background (research note, 2026-10-28)

Question: how do we make the pipeline trace a real crack and not something crack-like near it? This note reads the
chain as it stands, measures where strays enter, and ranks the fixes. Implemented 2026-10-28: see "Results" at the end.

## How the chain chooses today

| Stage | What decides | What it ignores |
|---|---|---|
| model (`live_predictor`, `inference`) | nnU-Net **argmax** -> {0,1} mask | the softmax probability is thrown away |
| `skeleton` | `remove_small_objects(64 px)` then `skeletonize` | shape, confidence, size in mm |
| `paths` | spur prune (10 px), longest-path decomposition, components ordered | nothing about which component is a crack |
| `crack_trace search` (`locate_crack`) | **centroid of every mask pixel in the frame**, gate `sum >= 150 px`; best stop = most pixels not touching the border | two cracks, or a crack plus a blob, give a target between them |
| `crack_trace plan` | the 3 longest segments >= 10 mm, main paths only | confidence; a stray >= 10 mm counts as a crack |
| `path3d` | annulus depth, `fraction_valid >= 0.3`, `normal_rms_m` recorded | the geometry numbers are reported, not used to reject |
| preview | the human: green on red or do not run | per-component numbers |

## What the model actually emits (measured 2026-10-28)

Softmax maps via `predict_single_npy_array(..., save_or_return_probabilities=True)` (the API already in the env),
components of the argmax mask scored by area, mean probability and elongation (major/minor axis). Prints resized to
848 px wide, which is roughly how the camera sees a 20 cm print at 25 cm.

| image | mask px | components | the crack(s) | strays (< 64 px) | strays that survive the 64 px filter |
|---|---|---|---|---|---|
| 01 | 8172 | 1 | p 0.97, elong 19 | none | none |
| 02 | 22509 | 26 | 3 comps, p 0.90-0.92 | 20, mean p 0.63 | 166/132/91 px at p 0.72-0.88 |
| 03 | 14654 | 32 | 7 comps, p 0.86-0.91 | 21, mean p 0.68 | several 200-900 px, p 0.82-0.88 (crack network) |
| 04 | 28886 | 58 | 23097 px, p 0.95 | 43, mean p 0.63 | 886/647/625 px at p 0.80-0.85 |
| 05 | 24661 | 13 | 18723 px, p 0.94 | 7, mean p 0.58 | none |
| 06 | 4099 | 38 | 2520 px, p 0.90, elong 4.5 | 31, mean p 0.75 | 102-141 px, p 0.85-0.91, elong 26-42 (thin side cracks) |
| **real D405 desk frame** (no crack, 3 m away) | 386 | 2 | - | - | **276 px, p 0.81, 37 px long; 110 px, p 0.69** |

Observations:
- Real cracks come out with mean p 0.87-0.97. Noise blobs come out at 0.58-0.75. The 0.5-0.9 band is the crack's
  fringe plus the strays; keeping the probability is the cheapest separator we have and it costs nothing at runtime.
- The 64 px filter is not a crack test. On 02 and 04 it passes blobs of 100-900 px at p ~0.8 that are texture.
- The crack-free desk frame gives **two components that pass every stage**: both survive the 64 px filter, the
  skeleton is 42 px (about 25-30 mm at that depth, over the 10 mm minimum), and the mask sum (386) is over the
  search gate (150). `search` would call that a crack and `scan-plan` would centre on it. The 276 px blob is the
  dark edge between a ceiling light panel and a tile: a high-contrast step, not a ridge. (CRACK_TRACE_PIPELINE.md's
  "no crack found" on the desk recording was a different frame; it does not hold for every frame.)
- The model's own card: it over-fires on crack-like texture, mortar lines and paving joints (control print #4
  of `tools/make_crack_targets.py`).

## What separates a crack from the background

A crack, as this camera sees it, is simultaneously:
1. **confident**: component mean p >= ~0.8 with a p >= 0.9 core;
2. **thin and long**: skeleton length large relative to area (width of a few px), elongation high;
3. **in the surface**: depth across the mask is continuous with the annulus around it (a crack does not produce a
   depth step; an object edge, a tile edge, a cable, a shadow of something above the surface does);
4. **at working distance**: 7-50 cm, where the model and the D405 are valid. The desk frame is at 3 m;
5. **dark on its own background**: a ridge in intensity, not a one-sided step (the light-panel edge);
6. **stable across frames**: present in several frames of the burst;
7. **tortuous rather than ruler-straight**: joints and seams are straight and of even width; cracks wander.

The pipeline checks none of 1, 2, 4, 5, 6, 7 and records 3 without acting on it.

## Recommended changes, cheapest first

1. **Keep the probabilities** (`live_predictor.predict` returns the crack-class softmax too). Hysteresis mask:
   seed p > 0.9, grow into p > 0.5 (`skimage.filters.apply_hysteresis_threshold`). Drops every blob that has no
   confident core while keeping the crack's fringe attached.
2. **A per-component crack score** in one new module (`crackvision.crack_filter`): for each connected component
   compute area, skeleton length (px and mm via depth), mean/max p, mean width (area / skeleton length), elongation,
   tortuosity (skeleton length / end-to-end distance), border contact. Accept only `mean_p >= 0.8 and
   length_mm >= L and width_mm <= W`; everything else is dropped **before** skeletonising. Replace the 64 px
   `remove_small_objects` default with this (keep the old path behind a flag so Phase-1 results stay reproducible).
   Write the per-component table into the case so the preview can print it.
3. **Per-crack targeting, not per-mask**: `locate_crack` should pick the best-scoring component and lift its
   centroid, and `search` should rank stops by that component's score, not by total mask pixels. `plan` should
   default to the single best crack (`--max-segments 1`) and trace others only on `--all-paths`.
4. **Depth gates in `locate_crack` and `path3d`**: reject when the annulus depth is outside 0.07-0.50 m; reject a
   component whose mask-interior depth differs from the annulus plane by more than a few mm (a step), using the
   `normal_rms_m` / plane residual path3d already computes. This alone removes the desk false positive.
5. **Burst voting in `capture` / `search-capture`**: the burst already holds 10 frames; run the model on 3-5 of them
   (55 ms each) and keep pixels present in a majority. Kills flicker and single-frame specks.
6. **Dark-ridge check**: on the grey image, a crack's skeleton pixels are darker than their local background
   (black-hat or Frangi ridge response). Require most of a component's skeleton pixels to sit on a dark ridge.
   This is the classical cue, and it is what the light-panel edge (a bright/dark step) fails.
7. **Joints and seams** stay the hard case: they satisfy 1-6. Tortuosity and width uniformity flag them, and
   the D405 test plan's negative block is where the threshold gets measured. Do not fine-tune before that
   measurement exists (ADR-010).

## How to verify

- Re-run the probe (`/tmp/.../scratchpad/probe/probe.py` today; make it `tools/probe_components.py`) on the six
  prints, the four control prints and real D405 frames of each at 15/25/40 cm, and report per-component
  precision: how many accepted components lie on an annotated crack.
- Acceptance for the gate: 0 accepted components on the desk recording and on the no-crack controls; the main crack
  accepted on all six prints; the paving-joint control at least flagged.
- Then the existing `crack_pipeline.sh demo` must still find print #1 and plan the same trace.

## Results (implemented 2026-10-28)

Items 1-6 are implemented in `crackvision.crack_filter` and wired into crack_trace `capture`, `search-capture` and
`plan`; item 7 is measured and flagged (`joint_like`), with rejection off by default. Thresholds are
`config/project.yaml` `crack_filter:`; the schema is [INTERFACES.md §3.15](../INTERFACES.md). The probe is
`tools/probe_components.py`. It runs the model in-process without TTA, as the live path does by default.

### Before / after, per input

"Before" is the chain as it was: argmax components, the components that survive the 64 px filter, and whether
`search` would have called it a crack (mask sum >= 150 px). "After" counts the hysteresis candidates, the accepted
components and the best one (highest mean p x length). "Rendered" means the print, 20 cm wide, flat under the camera
at 25 cm, with exact depth and the D405's intrinsics (the view the demo renders). The 15 and 40 cm rows are in the
probe output.

| input | mode | before: comps / after 64 px / search says crack | after: candidates / accepted | best accepted component |
|---|---|---|---|---|
| print 01 | 848 px, no depth | 1 / 1 / yes | 1 / 1 | the crack: 970 px, p 0.97, tortuosity 1.13 |
| print 01 | rendered 25 cm | 1 / 1 / yes | 1 / 1 | the crack: 217 mm, 2.9 mm wide, p 0.91 |
| print 02 | 848 px, no depth | 26 / 6 / yes | 12 / 5 | main crack: 1548 px, p 0.90 |
| print 02 | rendered 25 cm | 13 / 4 / yes | 5 / 3 | main crack: 194 mm, 3.0 mm, p 0.89 |
| print 03 | 848 px, no depth | 32 / 12 / yes | 18 / 13 | main crack: 998 px, p 0.91 |
| print 03 | rendered 25 cm | 14 / 4 / yes | 4 / 3 | main crack: 129 mm, 2.2 mm, p 0.85 |
| print 04 | 848 px, no depth | 58 / 15 / yes | 21 / 12 | the network: 2502 px, p 0.95 |
| print 04 | rendered 25 cm | 23 / 4 / yes | 4 / 3 | the network: 631 mm, 3.5 mm, p 0.90 |
| print 05 | 848 px, no depth | 13 / 6 / yes | 6 / 5 | the star: 993 px, p 0.94 |
| print 05 | rendered 25 cm | 4 / 2 / yes | 2 / 2 | one arm of the star: 138 mm, 2.8 mm, p 0.82 |
| print 06 | 848 px, no depth | 38 / 3 / yes | 27 / 16 | main crack: 853 px, p 0.90 |
| print 06 | rendered 25 cm | 1 / 1 / yes | 1 / 1 | the crack: 186 mm, 2.0 mm, p 0.87 |
| desk frame 15 (real D405) | depth | 2 / 2 / **yes** | 1 / **0** | - (1.87 m away, 34 mm wide, ring plane residual 22 mm) |
| desk, burst of 10 middle frames | depth, vote 2/3 | 2 / 2 / **yes** | 1 / **0** | - |
| desk, all 30 frames | depth | **30 of 30** frames called a crack | **0 of 30** frames with an accepted component | |
| control 16, plain concrete | rendered 25 cm | 8 / 2 / yes | 1 / **1** | the photo's top edge against the table: 163 mm, tortuosity 1.04 (not flagged); 40 cm: flagged |
| control 17, board-formed grooves | all modes | 0-2 / 0-1 / no | 0-1 / 0 | - |
| control 18, paint drips | all modes | 0-2 / 0-1 / no | 0 / 0 | - |
| control 19, paving joints | 848 px, no depth | 29 / 12 / yes | 14 / **8** | a joint, 386 px, tortuosity 1.05, width CV 0.27: **flagged** `joint_like` |
| control 19, paving joints | rendered 25 cm | 10 / 2 / yes | 3 / **2** | a joint, 115 mm, tortuosity 1.07, width CV 0.28: **flagged** |

The four controls are `tools/make_crack_targets.py`'s no-crack prints #16-19, downloaded from Wikimedia Commons for
this measurement (not stored in the repo).

What the numbers say:
- The working-distance and depth-step gates remove the desk false positive. The research note expected the dark-ridge
  check to remove it as well. It does not: the component is the lamp panel's dark metal frame, a thin line darker
  than both of its sides (black-hat ridge fraction 1.00). What tells it apart in the grey image is that its two
  sides differ: flank asymmetry 1.15 against 0.12-0.87 for every best print crack (1.34 for print 05 at 40 cm).
  That is one sample, so `flank_asymmetry` is reported and its gate (`max_flank_asymmetry`) is off.
- The ridge check does not discriminate on these inputs. Every component that reached it was dark on its
  background: the print cracks are high-contrast, and the controls' false positives are dark lines too. It rejects a
  bright/dark step (synthetic test), the case the note described.
- Strays: the hysteresis seed removes 29-64 % of the argmax components (06: 38 -> 27, 02: 26 -> 12, 04: 58 -> 21). The
  confidence gate then rejects 1-7 more per print. Thin side cracks with p 0.82-0.91 (02, 03, 04, 06) are accepted:
  they are real cracks, and `plan` traces only the best component by default.
- Depth gates on the prints: at 15 cm, 1-8 components per print fall under 10 mm; at 25 and 40 cm few strays
  are left for them.
- Print 05 at 15 cm: the model sees only part of the star at that scale (mask 4082 px), and the largest arm comes
  out at mean p < 0.8 and is rejected. The accepted component is a 56 mm arm. At 25 and 40 cm the main crack is
  accepted.
- Joints: with the measured thresholds (tortuosity <= 1.08, width CV <= 0.30, width >= 2 px) the best paving-joint
  component is flagged at 848 px and at 25 cm, and no best print crack is flagged. The margin is thin: print 01's
  tortuosity is 1.09-1.13 and print 02 at 40 cm is 1.07 (width CV 0.38). At 15 cm the joint print's best component is
  a joint crossing (tortuosity 1.89, not flagged; 4 other components there are).

### Acceptance criteria

| criterion | status |
|---|---|
| 0 accepted components on the desk recording | **holds**: 0 of 30 frames (single frame and burst vote) |
| 0 accepted on the no-crack controls | **partly**: grooves (17) and paint drips (18) give 0. Plain concrete (16) gives 0 at 848 px, but in the 25/40 cm renders the photo's dark top strip against the table is accepted. The paving joints (19) are accepted as cracks (see the next row). |
| main crack accepted on all six prints | **holds** at 848 px and at 25 cm (the best component is the main crack on all six); at 15 cm print 05 keeps only one arm |
| paving-joint control at least flagged | **holds** for the best component at 848 px and 25 cm; not at 15 cm |
| `crack_pipeline.sh demo` still finds print #1 and plans the same trace | **holds**: all 6 stops see it (1/1 accepted each, 218-222 mm where it is whole). The best stop is now 5 (score 202) instead of 0, centre (0.336, -0.116) against the true (0.34, -0.12). Trace: 209 mm, hover 13.8 cm, tilt 10 deg, 14.6 s; both rehearsals complete, keep-out margin >= 18 mm; 2 min 44 s |
| per-component precision on annotated real D405 frames at 15/25/40 cm | **not done**: no annotated D405 frames of the prints exist yet (D405 off USB); the renders stand in |

### Reproducibility of the Phase-1 batch path

The flags (`crack_filter.batch`, `inference --save-probabilities`, `skeleton --crack-filter`) are off by default.
Checked 2026-10-28 with `run_test.sh --paths` on the six prints plus the desk frame, HEAD against this change:
- the nnU-Net command and `predict_from_raw_data_args.json` are identical apart from the root path;
- on the same predictions, all 56 downstream files (overlays, comparisons, skeletons, paths) are byte-identical.

Two separate inference runs differ by 0-16 pixels per image (73k-317k crack pixels each). That is GPU run-to-run
noise in nnU-Net, not this change: the command is the same.

### Departures from the plan above

- **Item 4 is in crack_filter, not path3d.** The working-distance, plane-residual and interior-offset gates
  run per component before skeletonising, using the ring and plane-fit method path3d uses. path3d itself is
  unchanged, so `paths3d/1` documents keep their schema.
- **Surface tilt gate added.** A straight step between two parallel surfaces fits one steep plane with no residual.
  The ring plane's angle to the optical axis is therefore gated at 60 deg (`max_surface_tilt_deg`).
- **Flank asymmetry added** (reported, gate off). It is the only grey-image cue that separates the lamp frame.
- **camera.py snap/live: opt-in** (`--filter`, `crack_filter.camera: false`). Their tests pin the old default.
  crack_trace `capture`/`search-capture`/`plan` filter by default.
- **capture now runs the model in-process** (burst vote, no TTA by default; `--tta` for mirroring) instead of the
  batch subprocess with TTA. `--no-filter` restores the batch path exactly.
- **Joint thresholds** are 1.08 / 0.30 / 2 px (measured above), not the 1.05 / 0.25 first written into the config.
