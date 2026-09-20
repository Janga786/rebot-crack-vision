# ADR-001: OpenCrack nnU-Net v2 is the primary (and only) segmentation model

**Status:** accepted · **Date:** 2026-09-16

## Context
The capstone needs pixel-level crack segmentation from D405 RGB. Candidate released models include
CrackSAM-adapter, OmniCrack30k, Hybrid-Segmentor and OpenCrack nnU-Net. Verified upstream
(2026-09-16): `fadeevla/opencrack-nnunet` is public, ungated, CC-BY-4.0, last modified 2026-06-20, and
reports 0.539 IoU / 0.641 clIoU(τ=4) on the leakage-controlled OpenCrack test cohort — ahead of the
other three by confidence-interval-disjoint margins on all five reported metrics.

## Decision
Use **`fadeevla/opencrack-nnunet`** (Dataset501_OpenCrack, `2d`, `nnUNetTrainer__nnUNetPlans__2d`,
fold 0, `checkpoint_ep0500.pth`) as the sole model for this phase. **Zero-shot only** — no
fine-tuning, no ensembling, no comparison harness.

## Consequences
+ nnU-Net v2 is a mature, self-configuring pipeline with a stable CLI; the HF repo is already laid
  out as an nnU-Net results tree, so setup is placement, not conversion.
+ `pip install nnunetv2` brings the whole inference stack; we write no model code at all.
− We inherit nnU-Net's ergonomics: `_0000` channel suffixes, PNG-only input, and `{0,1}`-valued
  output PNGs that look black.
− Single fold, no ensemble → slightly lower ceiling than a 5-fold run, accepted for a capstone.
− **CC-BY-4.0 requires attribution.** The README and any capstone write-up must cite Fadeev (2026)
  and Isensee et al. (2021).
− The model over-fires on crack-like texture and on masonry bonds/mortar lines, and collapses on wide
  road scenes. Our regime (close-up concrete/bridge deck) is in-domain, but **whether it transfers to
  D405 imagery at 10–20 cm is the open research question** — not a defect to patch.

## Revisit when
Level-3 evaluation on real D405 imagery shows unusable recall or intolerable false positives on the
target surfaces. Then, and only then, consider a second model or fine-tuning — as a new project phase.
