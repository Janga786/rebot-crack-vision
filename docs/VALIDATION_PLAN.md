# VALIDATION_PLAN.md — three levels of testing

| Level | Question it answers | Needs GPU? | Needs camera? | Owning cards |
|---|---|---|---|---|
| **L1 — Environment smoke** | Is the software stack correctly installed and wired? | no (reports it) | no | TC-003 |
| **L2 — Software smoke** | Does the pipeline *execute* end to end and produce well-formed outputs? | preferred, CPU-capable | no | TC-006, TC-014 |
| **L3 — Model-quality evaluation** | Is OpenCrack *good enough* on real D405 crack imagery? | yes | yes | **deferred** — conventions in `D405_TEST_PLAN.md` |

> L1 and L2 say nothing whatsoever about accuracy. Only L3 does. Do not let a green L2 be reported
> as "OpenCrack works".

---

## Level 1 — Environment smoke test

**Command:** `./env.sh python scripts/check_env.py`
**Pass:** exit 0, no `FAIL` rows. **Artefact:** `logs/check_env_latest.json`.

Proves:
1. Python is 3.11.x from the `crackvision` env.
2. **The environment scrub worked** — no `/opt/ros` entry on `PYTHONPATH` or `sys.path`
   (this is the single most important row; see `RISKS.md` R-01).
3. `torch` imports, version `>=2.1.2` and not `2.9.*`, `torch.version.cuda` is set.
4. `torch.cuda.is_available()` and the GPU is the RTX 3090; total and **free** VRAM are reported.
5. NVIDIA driver version parsed from `nvidia-smi`.
6. `nnunetv2` imports and `nnUNetv2_predict_from_modelfolder` is on `PATH`.
7. numpy / scikit-image / Pillow import.
8. `pyrealsense2` presence (WARN-only — it is an optional extra).
9. Project directories exist and are writable; free disk reported (WARN below 20 GB).
10. `nnUNet_raw` / `nnUNet_preprocessed` / `nnUNet_results` are set **and point inside the project**.
11. The model checkpoint is discoverable at the expected path with a plausible size (≈268 MB).

Rows 10–11 `FAIL` before TC-004/TC-005 have run — that is correct and expected. `check_env.py`
distinguishes "not yet fetched" from "broken".

---

## Level 2 — Software smoke test

**Commands:**
```bash
./env.sh python scripts/smoke_test.py          # TC-006, synthetic fixture, real model
./env.sh pytest tests/ -v                      # TC-014, no GPU and no camera required
```

### What L2 proves

| # | Assertion | Where |
|---|---|---|
| 1 | Arbitrary input (JPEG / RGBA / grayscale / palette / 16-bit / EXIF-rotated) converts to 3-channel 8-bit RGB PNG | `test_prepare_inputs.py` |
| 2 | `case_id` derivation matches the table in `INTERFACES.md` §1.1 exactly, including collisions | `test_naming.py` |
| 3 | `case_map.json` validates against its schema and round-trips | `test_prepare_inputs.py` |
| 4 | The nnU-Net command executes and returns 0 | `smoke_test.py` |
| 5 | One prediction PNG exists per input | `smoke_test.py` |
| 6 | Prediction dtype is `uint8` and `set(unique) ⊆ {0,1}` | `smoke_test.py` |
| 7 | **`prediction.shape == input.shape`** (the frame invariant, `RISKS.md` R-14) | `smoke_test.py`, `test_integration_smoke.py` |
| 8 | Mask, overlay and comparison images are generated, non-empty, and correctly sized | `test_visualize.py` |
| 9 | An all-zero mask produces valid outputs and is not an error | `test_visualize.py`, `test_skeleton.py` |
| 10 | Skeleton is 1-px, `skeleton_pixels ≤ mask_pixels`, same shape as the mask | `test_skeleton.py` |
| 11 | `_skeleton_stats.json` matches its schema | `test_skeleton.py` |
| 12 | `crackvision.realsense_capture` imports with **no** `pyrealsense2` installed | `test_realsense_import.py` |
| 13 | A random `uint16` array round-trips byte-identically through the 16-bit depth PNG writer | `test_realsense_import.py` |
| 14 | No literal `/home/` appears anywhere in `src/`, `scripts/`, `tools/`, `tests/` (`RISKS.md` R-15) | `test_integration_smoke.py` |
| 15 | Every CLI's `--help` exits 0 and every CLI honours `--dry-run` | `test_integration_smoke.py` |

### What L2 deliberately does **not** assert

- That the model found the synthetic crack.
- Any IoU, Dice, precision or recall.
- Anything about real-world crack appearance.

The synthetic fixture is a drawn polyline on noise. It is **out of distribution** for a model trained
on real pavement. It may score near zero and **that is not a failure of L2.** Asserting detection on
it would produce a test that fails for the wrong reason and gets "fixed" by weakening the pipeline.
`smoke_test.py` logs the crack-pixel percentage for human interest and asserts nothing about it.

### Test markers

```python
@pytest.mark.gpu      # skipped unless torch.cuda.is_available()
@pytest.mark.camera   # skipped unless a RealSense device is enumerated
@pytest.mark.model    # skipped unless models/opencrack-nnunet/... exists
```

`pytest tests/` on a bare machine (no GPU, no camera, no weights) must exit **0** with those tests
skipped, never errored. This is a named acceptance criterion of TC-014.

---

## Level 3 — Model-quality evaluation (DEFERRED — do not execute this phase)

Data conventions, metadata schema and the shot list are specified now so that collection can start
immediately afterwards without another design session. See **`docs/D405_TEST_PLAN.md`**.

Target: **50–100** D405 images. Dimensions to vary: distance (10/15/20/30/40 cm), crack width
(wide/thin/branching), negatives (no-crack, seams, stains, aggregate, scratches, marker/pencil
lines, shadows), camera angle, lighting.

Qualities to assess, per `D405_TEST_PLAN.md` §5:
- **crack detection** — is a real crack found at all, at each distance?
- **mask continuity** — one connected crack, or a dashed line of fragments? (component count from
  `_skeleton_stats.json` is the proxy)
- **false positives** — how badly does it fire on the negative controls, especially seams and
  mortar lines, which the model card names as its hardest failure mode?
- **distance sensitivity** — at what working distance does performance fall off?
- **lighting sensitivity** — ambient vs ring light vs oblique vs shadowed.
- **viewing-angle sensitivity** — 0° / 15° / 30° / 45° off-normal.
- **surface-texture sensitivity** — smooth concrete vs exposed aggregate vs formwork-marked.

**No images are collected, labelled, or evaluated in this implementation phase.** TC-015 delivers the
metadata tooling and an empty manifest, nothing more.

### Why quantitative metrics are deferred

IoU against hand-drawn labels needs (a) labels we do not have, (b) an annotation protocol for
one-pixel-wide structures where IoU is known to be unfair (the model card notes a perfect thin-crack
prediction can score ≈0.48), and (c) a decision about clIoU tolerance τ. L3 therefore starts
**qualitative and structured** — a scored rubric over the shot list — and only becomes quantitative
if the qualitative pass says the model is worth measuring precisely.
