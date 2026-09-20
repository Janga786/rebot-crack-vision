# ADR-010: No training, fine-tuning, or preprocessing of training data

**Status:** accepted · **Date:** 2026-09-16

## Context
nnU-Net ships `nnUNetv2_plan_and_preprocess` and `nnUNetv2_train`, and the OpenCrack repo publishes
`reproducibility/splits_final.json`, so retraining is superficially within reach. The model's known
weaknesses (over-firing on masonry bonds and crack-like texture) are fixable with hard-negative
enrichment — the model card says so. That is an inviting trap.

Fine-tuning needs labelled D405 imagery we do not have, days of GPU on a shared card, and a
quantitative evaluation harness that does not exist yet. Prior experience on this workstation also
records fine-tuning a converged policy making things measurably worse.

## Decision
**Zero-shot only.** No task card may run `nnUNetv2_train`, `nnUNetv2_plan_and_preprocess`, or any
optimiser step. `nnunet/raw/` and `nnunet/preprocessed/` stay empty. No labelling, no augmentation
pipeline, no dataset conversion.

## Consequences
+ The project stays a days-long evaluation, not a weeks-long training effort.
+ The GPU stays free for other projects on this shared machine.
+ We get an honest zero-shot baseline number, which is the thing a fine-tune would have to beat.
− Accuracy is capped at what the released checkpoint gives.
− If zero-shot fails, we will have learned that cheaply, which is the point.

## Revisit when
Level-3 evaluation quantifies a specific, characterised failure mode **and** a labelled D405 dataset
exists to address it. That is a new project phase with its own plan.
