# Speaker notes — *Finding the crack, then reaching it*

Companion to [`index.html`](index.html). Same running order, with the numbers you need on hand and
the things worth saying out loud. Figures are referenced by filename so you can rebuild this deck in
PowerPoint, Google Slides or Keynote without opening the HTML.

**Timing:** ~12 minutes at a comfortable pace, or ~7 if you skip slides 7 and 9.

**The one sentence, if you only get one:** *public crack-segmentation weights run zero-shot on our
hardware, end to end, with every stage pinned by something that fails loudly — and the one question
that actually matters is scheduled, not quietly assumed.*

---

## 1 · Title — Finding the crack, then reaching it

> reBot B601-DM capstone · week of 14–20 September 2026

Say what the project is in one breath: a robot arm that inspects concrete by following a crack, and
this week was the perception half of it.

---

## 2 · The problem — a robot arm can only inspect a crack it can locate

**Figure:** none (table + callout).

- The end goal: a reBot B601-DM arm tracing a crack across a concrete surface with a sensor at the
  tool. Bridge and structure inspection, without a person marking the defect by hand.
- Everything downstream — depth, path, trajectory — is meaningless if the pixels marked "crack" are
  wrong. So perception comes first.
- **The week's question:** can a publicly released model work *zero-shot* on our imagery, or does
  the capstone have to fund labelling and training of its own?

The candidate, for the table:

| | |
|---|---|
| Model | OpenCrack nnU-Net v2, 2D configuration, fold 0 |
| Weights | 268 MB · ≈92.5 M parameters · 256×256 patches |
| Trained on | 49,531 images |
| Reported quality | 0.539 IoU · 0.641 clIoU (τ=4) on the OpenCrack test cohort |
| Licence | CC-BY-4.0 — attribution is a real obligation, and it is in the README |

> If asked "why not train our own?": because a zero-shot answer is a week, and a trained answer is a
> labelling campaign. Establish whether the free option works before spending the expensive one.

---

## 3 · The week in numbers

**Figure:** none (stat tiles).

| Number | What it is |
|---|---|
| **9 of 16** | task cards shipped, each independently reviewed before it counted as done |
| **46** | unit tests passing, zero failures, no GPU required to run them |
| **12** | review gates run — **3** cards were sent back for revision before approval |
| **≈5.8 s/image** | wall-clock at 512², RTX 3090, for a 2-image batch **including nnU-Net process start-up** |
| **10** | architecture decisions recorded as ADRs |
| **20** | risks registered, each with a named mitigation and an owning task card |
| **6** | model files pinned by size, sha256 and upstream revision |
| **0** | crack pixels produced on the blank negative-control frame |

> Be careful with the timing number — it is honest *wall-clock including start-up*, not a throughput
> benchmark. Upstream reports ≈1.45 img/s at 2048² for the same model on an 8 GB card. If someone
> asks about speed, say the pipeline is batch-oriented today and per-image cost has not been
> optimised, because nothing yet depends on it.

---

## 4 · The pipeline, and the contract underneath it

**Figure:** `figures/fig06_pipeline_diagram.png`

```
capture → prepare_inputs → OpenCrack nnU-Net v2 → binary mask → visualize
                                                       ↓
                        (deferred) skeleton → ordered path → depth → hand-eye → arm trajectory
```

- `prepare_inputs` forces every input through EXIF-transpose → 3-channel RGB → lossless PNG. The
  model's reader rejects JPEG outright and asserts 3 or 4 channels; grayscale, RGBA, palette, CMYK
  and 16-bit inputs all break it. One unconditional conversion makes that class of bug impossible.
- **The load-bearing decision:** nothing resizes, crops or pads. A `(row, col)` in the mask is the
  same `(row, col)` in the original frame — and later, in the aligned depth frame.
- Why it matters: break it once and you don't find out here. You find out months later as a
  mis-registered robot trajectory, a long way from the cause.

---

## 5 · Image in, centerline out

**Figure:** `figures/fig01_pipeline_result.png`

Four panels, left to right: input · binary mask · overlay · centerline.

- 512×512 RGB input → **4,577 crack pixels (1.75 % of the frame)** → a **657-pixel** one-pixel-wide
  centerline.
- This is a real inference run on the RTX 3090 through `nnUNetv2_predict_from_modelfolder`, not a
  cached picture — the Level-2 smoke test reproduces it in about 14 seconds.
- Panel 4 is thickened for print. The real skeleton is one pixel wide, which is exactly the point:
  it is what a later stage indexes the depth frame with.

> The crack here is synthetic, drawn by the project's own smoke test. Say so before anyone asks.
> Real D405 imagery is slide 10's honest gap.

---

## 6 · Every run has to fail the blank frame too

**Figure:** `figures/fig02_negative_control.png`

- The same textured surface with **no crack** → **0 crack pixels**. The positive case on the same
  texture → the crack recovered.
- A pipeline stuck in the "on" position looks identical to a working one when you only ever show it
  cracks. The blank frame is what makes the positive case mean anything.
- Both assertions run on **every** smoke test, not once when it was written.

---

## 7 · Three failures worth designing against *(skippable)*

**Figure:** none (three columns).

- **R-01, the environment.** The shell profile sources ROS Humble, putting Python 3.10 packages on a
  3.11 interpreter's path. Pure-Python packages silently shadow the real ones; compiled ones throw
  opaque ABI errors. `env.sh` scrubs `PYTHONPATH`, filters `/opt/ros` out of the loader path, and the
  verifier *fails* if a ROS entry ever reappears. Proven under a deliberately poisoned shell.
- **R-02, the shared GPU.** This workstation runs other robotics projects that park multi-GB servers
  on GPU 0. An out-of-memory prints a five-step degradation ladder as literal next commands rather
  than a traceback. Standing rule: no task may ever kill another project's GPU process.
- **R-03, the silent misread.** Predictions are `{0,1}`-valued PNGs, not `{0,255}` — the file looks
  solid black in any image viewer. The natural reaction ("it found nothing, let me rescale it")
  corrupts the data. Documented, logged once per run, binarised as `> 0` everywhere, and pinned by a
  test.

---

## 8 · Nothing counted as done until someone else checked it

**Figure:** `figures/fig03_scoreboard.png`

- Each of the 16 task cards is a written spec with explicit acceptance criteria and an explicit
  out-of-scope list.
- The loop: spec → implementation → **independent adversarial review that re-runs every acceptance
  command itself** → approve, or send it back.
- 12 gates run; 3 cards came back for a second pass. Reviews re-derive from the filesystem and git
  rather than trusting the implementation report — including a full real-model GPU run, with GPU
  memory checked back to its idle baseline afterwards.

---

## 9 · What review caught *(skippable, but it's the best material)*

**Figure:** none (four callouts).

- **A corrupt model file that downloaded cleanly.** `plans.json` had a second JSON document appended
  to it; every parse threw `Extra data: line 218 column 2`. Semantic validation of the model tree
  caught it at the gate instead of it surfacing as a mystery crash three cards later.
- **A card marked complete that wasn't.** TC-005 scored 7/10 at review and was rejected. Approving it
  would have flipped downstream cards to "ready" against a model tree that could not be loaded.
- **Two upstream traps, found during planning.** The model card names a file that does not exist in
  the repository (`nnUNetPlans.json` vs the actual `plans.json`), and nnU-Net defaults to five folds
  and a `checkpoint_final.pth` this model does not ship. Both are pinned in config and asserted.
- **Verification, not narration.** Approval requires the reviewer's own command output, not the
  implementer's paste.

## Sim 1 · Following the vision output into the arm

**Figure:** `figures/fig04_trajectory.png`

- The 18 ordered centerline waypoints from the real segmentation, fed through a damped
  least-squares inverse-kinematics solve for the actual B601-DM joint chain (built straight from the
  URDF's own joint origins and axes, not a simplified model).
- **Max tool position error 0.089 mm, mean 0.031 mm** — every one of the 20 poses (18 crack points
  plus an approach and a retract waypoint) is reachable well inside a 2 mm target, and every joint
  stays inside its URDF limit for the full 30.3 s pass at a constant 2 cm/s.
- One honest wrinkle, worth having ready if asked: the very first step (approach → first crack point)
  needs an unusually large joint-4 swing. Checked and it's real local kinematic sensitivity at that
  pose (confirmed via the Jacobian's singular values), not an unresolved elbow flip — and it's far
  inside the joint's velocity rating for a 5 s window, so it isn't a practical problem.
- **What's declared, not measured, here:** the coupon's pose and the camera→robot transform. This
  slide is IK math on a real robot model following real vision output — it is not a claim that the
  robot has done this.

## Sim 2 · The same path, in Isaac Sim

**Figure:** `renders/scene_overview.png`

- The actual B601-DM URDF (not a simplified stand-in) imported into Isaac Sim, headless, on the same
  RTX 3090 that ran the segmentation — a concrete coupon carrying the real crack image, the path drawn
  above it in the scene.
- If asked how long this took to build: Isaac Sim's first-ever shader compile on a machine takes 5-15
  minutes; every render after that takes about 15-25 seconds for the whole scene.

## Sim 3 · Does the path actually land on the crack?

**Figure:** `renders/topdown_path.png` + `renders/closeup_coupon.png`

- The orthographic top-down shot is the actual proof: camera dead-centre above the coupon, path
  flattened to zero standoff, so there is no perspective to hide a misalignment behind.
- **Say this plainly if anyone looks closely:** there is a small (few-millimetre) visual gap between
  the rendered path and the painted crack in a few places. This was investigated, not waved away —
  four colour-coded fiducials at the image's four corners land exactly on the coupon's geometric
  corners (a rigid rotation that matches at all four corners is mathematically exact everywhere), and
  a direct pixel comparison between the source image, the model's mask, and the skeleton found zero
  systematic bias. The remaining fuzz is marker size and ordinary texture-filtering blur on a coupon
  this size at this resolution — not a coordinate bug. Full investigation in
  `presentation/sim/README.md`.
- This is the right answer to "why didn't you just nudge it to line up?" — the task rule was that
  waypoint math is never touched to make a picture agree, and that rule held here.

## 10 · What is measured, and what is still declared

**Figure:** none (two columns). **This is the slide that earns the rest of them.**

Measured:
- Real nnU-Net v2 inference on an RTX 3090, timed per image
- Mask values, frame shape and channel handling, asserted by tests
- Zero false positives on the blank control frame
- Model tree integrity by sha256 against a pinned upstream revision
- 46 unit tests across naming, input preparation and visualization

Declared, not yet measured:
- **Zero-shot quality on real D405 imagery** — the headline question. No camera is attached yet, so
  nothing is claimed.
- Depth at the crack — a crack is often narrower than the stereo correlation window, so the camera
  may return the surface depth across it, or a hole.
- Camera→robot extrinsics — the demo transform is chosen, not hand-eye calibrated.
- IoU/Dice against labels — Level-3 evaluation, deliberately outside this phase.

> Say this plainly. A capstone audience can tell the difference between a result and a demo, and
> saying it first is worth more than any extra figure.

---

## 11 · What the remaining seven cards unlock

**Figure:** none (table + callout).

| Card | Unlocks |
|---|---|
| TC-010 | skeletonization as a first-class stage |
| TC-011 | the whole chain behind one command |
| TC-012 / TC-013 | D405 RGB + aligned depth capture |
| TC-014 | integration and contract tests |
| TC-015 / TC-016 | evaluation manifests, reproducibility |

Then the real experiment: a D405 test plan is already written — a distance sweep plus negative
controls chosen to be *hard* (construction seams, stains, aggregate, scratches, marker lines). The
model's own card admits it over-fires on crack-like texture and masonry bonds, so the plan is to
characterise that failure surface deliberately rather than discover it on the robot.

---

## 12 · Takeaway

**Figure:** none (three tiles).

- **Works** — public crack weights run zero-shot, end to end, on our hardware. No training, no
  labelling, no licence problem.
- **Pinned** — environment, model bytes, mask convention and image frame are each asserted by
  something that fails loudly.
- **Honest** — the headline question is scheduled and instrumented, not assumed.

Close on reproducibility: pinned requirements, a launcher that scrubs the environment, a
hash-verified model tree, and a smoke test that proves the GPU path in about fourteen seconds.

---

## Questions you should expect

**"Why not just train your own model?"**
A zero-shot answer costs a week; a trained one costs a labelling campaign. Establish whether the free
option works first. If it doesn't, the D405 evaluation set built in TC-015 is exactly the seed corpus
a fine-tune would need — the work isn't wasted either way.

**"How accurate is it?"**
On the OpenCrack test cohort, upstream reports 0.539 IoU / 0.641 clIoU. **On our imagery, unknown —
and I'm not going to quote a number I haven't measured.** That measurement is TC-015 plus the D405
test plan.

**"Isn't 5.8 seconds an image slow?"**
It's wall-clock including process start-up on a batch of two, and nothing downstream is real-time
yet. Upstream reports ≈1.45 img/s at 2048² on an 8 GB card. Inspection is a stop-and-scan task, so
per-image latency hasn't been optimised and doesn't need to be yet.

**"What happens when the GPU is busy?"**
An OOM prints a five-step degradation ladder ending in a CPU path that always works. Nothing in this
project may kill another project's GPU process.

**"Why 16 task cards instead of just building it?"**
Because each card carries acceptance criteria and an out-of-scope list, an independent reviewer can
re-run them, and three of them failed that check. Without the gate, those three would be in the
codebase right now.
