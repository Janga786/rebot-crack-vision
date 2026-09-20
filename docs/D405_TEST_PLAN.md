# D405_TEST_PLAN.md — evaluation dataset design (planning only)

**Status:** PLAN ONLY. **No images are captured, labelled or evaluated in this implementation phase.**
TC-015 delivers the metadata tooling and a header-only manifest; TC-013 delivers the capture utility.
Collection is a separate, later activity.

**Purpose:** answer, with evidence, *"does OpenCrack give usable zero-shot crack segmentation on
D405 imagery?"* — and, where it fails, characterise **how**.

---

## 1. Hardware assumptions

| | |
|---|---|
| Camera | Intel RealSense **D405** (not attached to this machine at planning time) |
| Depth technology | active stereo, short range; ideal ≈ **7 cm – 50 cm** |
| Colour source | the **left IR imager** — colour and depth share very nearly one optical center |
| Default streams | colour `848×480 @ 30 bgr8`, depth `848×480 @ 30 z16` |
| Alignment | `rs.align(rs.stream.color)` **on** |
| Depth units | read from `get_depth_scale()` — typically **0.0001 m/unit** for D405. Never hard-coded (`RISKS.md` R-09) |
| Connection | USB-3 required; a USB-2 link silently drops to lower resolution/fps — the capture tool records the USB descriptor so this is detectable after the fact |

The planned 10–40 cm working distances sit inside the D405's sweet spot, which is the main reason
this camera was chosen for close-up crack work.

---

## 2. Target: 50–100 images

Suggested allocation (**72 images**, comfortably inside the range):

### 2A. Distance sweep — 25 images
One fixed, representative crack on one surface, shot head-on (0°) under one constant lighting
condition, at:

| Distance | Shots |
|---|---|
| 10 cm | 5 |
| 15 cm | 5 |
| 20 cm | 5 |
| 30 cm | 5 |
| 40 cm | 5 |

This is the **primary sensitivity axis** — it directly determines the arm's working standoff.
Holding everything else fixed is what makes the result interpretable.

### 2B. Crack-type variety — 15 images
At a fixed 20 cm, 0°, constant lighting:

| Crack type | Shots |
|---|---|
| wide (> 3 mm) | 4 |
| thin (< 1 mm) | 4 |
| branching / networked | 4 |
| hairline, barely visible | 3 |

Thin and hairline are where the model card warns IoU is unfair and where the D405's resolution
starts to matter.

### 2C. Negative controls — 20 images
**The most important block.** The model card explicitly names over-firing on crack-like texture and
on masonry bonds/mortar lines as its residual failure mode. A robot that routes a tool along a
control joint is worse than one that finds nothing.

| Negative | Shots |
|---|---|
| plain concrete, no crack | 4 |
| construction / control joint, seam | 4 |
| stain, discolouration, water mark | 3 |
| exposed aggregate, rough texture | 3 |
| surface scratch / scuff (not a crack) | 3 |
| marker or pencil line | 3 |

### 2D. Viewing angle — 6 images
One crack at 20 cm: 0°, 15°, 30°, 45° off-normal (plus 2 repeats of the extremes).

### 2E. Lighting — 6 images
One crack at 20 cm, 0°: ambient office, ring/coaxial light, oblique raking light, harsh direct,
dim/underexposed, hard shadow across the crack. Raking light is included because it is the classic
way to make a hairline crack visible — if it helps, that is an actionable capture recommendation.

---

## 3. Metadata schema — `data/d405/eval_manifest.csv`

One row per captured image. Written/validated by `tools/dataset_manifest.py` (TC-015).
`scan` pre-fills the machine-derived columns from `data/d405/metadata/*.json`; the human fills the
judgement columns.

| Column | Type | Source | Required | Notes |
|---|---|---|---|---|
| `image_id` | string | tool | ✅ | `{session}_{NNNNNN}`; **unique primary key** |
| `timestamp_utc` | ISO-8601 | tool | ✅ | from the frame metadata |
| `camera` | enum | tool | ✅ | `D405` \| `other` |
| `color_path` | path | tool | ✅ | project-relative; validated to exist |
| `depth_path` | path | tool | ✅ | project-relative; validated to exist |
| `metadata_path` | path | tool | ✅ | the per-frame JSON |
| `distance_cm` | float | human | ✅ | measured standoff, lens front to surface |
| `camera_angle_deg` | float | human | ✅ | 0 = normal to surface |
| `surface_type` | enum | human | ✅ | `concrete_smooth` \| `concrete_aggregate` \| `concrete_formed` \| `asphalt` \| `masonry` \| `mortar` \| `other` |
| `lighting_condition` | enum | human | ✅ | `ambient` \| `ring` \| `oblique` \| `direct_harsh` \| `dim` \| `shadowed` |
| `crack_present` | enum | human | ✅ | `yes` \| `no` \| `ambiguous` |
| `crack_type` | enum | human | ✅ | `wide` \| `thin` \| `hairline` \| `branching` \| `none` |
| `negative_class` | enum | human | — | `none` \| `joint` \| `seam` \| `stain` \| `aggregate` \| `scratch` \| `marker` \| `shadow` — **required when `crack_present == no`** |
| `block` | enum | human | ✅ | `distance` \| `crack_type` \| `negative` \| `angle` \| `lighting` |
| `notes` | free text | human | — | anything unusual |
| `depth_valid_fraction` | float | tool | — | fraction of non-zero depth pixels — the R-08 early-warning metric |
| `exposure_us` | int | tool | — | from frame metadata |
| `pred_crack_fraction` | float | tool | — | filled **after** inference |
| `pred_components` | int | tool | — | from `_skeleton_stats.json`, after inference |
| `rating_detection` | int 0–3 | human | — | L3 rubric, §5 |
| `rating_continuity` | int 0–3 | human | — | L3 rubric, §5 |
| `rating_false_positive` | int 0–3 | human | — | L3 rubric, §5 |

`validate` enforces: required columns present; enum membership; `image_id` unique; referenced files
exist; `negative_class` set whenever `crack_present == no`; `distance_cm` in `(0, 200]`;
`camera_angle_deg` in `[0, 90]`.

---

## 4. On-disk layout

```
data/d405/
├── color/      {session}_{NNNNNN}_color.png     8-bit RGB
├── depth/      {session}_{NNNNNN}_depth.png     16-bit single-channel, RAW z16 units
├── metadata/   {session}_{NNNNNN}.json          per-frame: intrinsics, depth scale, extrinsics
│               {session}_session.json           per-session: device, config, tool version
└── eval_manifest.csv
```

Evaluation images are then **copied** (not moved) into `data/input_originals/` for a `./run_test.sh`
pass, so the raw capture set stays pristine. `case_id` derives from the copied filename, and
`image_id` == the `{session}_{NNNNNN}` stem, which keeps the manifest joinable to
`case_map.json` and `_skeleton_stats.json`.

---

## 5. Level-3 scoring rubric (qualitative first — see `VALIDATION_PLAN.md`)

Per image, a human scores three axes 0–3 while looking at `data/comparisons/{case}_comparison.png`:

| Score | `rating_detection` (`crack_present == yes` only) | `rating_continuity` | `rating_false_positive` |
|---|---|---|---|
| 3 | crack fully found, full extent | one continuous run, no gaps | no spurious regions |
| 2 | mostly found, some extent missing | ≤ 2 small gaps | 1 small spurious region |
| 1 | fragment only | dashed / many fragments | several spurious regions |
| 0 | not detected at all | unusable | mask dominated by spurious regions |

For negatives (`crack_present == no`), only `rating_false_positive` applies, and **3 means the model
correctly output nothing.**

Headline numbers to report:
- mean `rating_detection` per `distance_cm` → the working-distance curve
- mean `rating_false_positive` per `negative_class` → the joints/seams question the model card flags
- mean `rating_continuity` per `crack_type` → feeds the deferred path-ordering design (`adr/008`)
- `depth_valid_fraction` at crack pixels vs the whole frame → early evidence for `RISKS.md` R-08

---

## 6. Capture protocol (for the later collection session)

1. Fix the camera on a stand; measure standoff with a ruler or gauge block — do **not** estimate it.
2. `./env.sh python -m crackvision.realsense_capture --session dist10 --frames 5 --interval 1.0`
3. Let auto-exposure settle — the tool discards 30 warm-up frames; do not defeat this.
4. One session name per condition block (`dist10`, `dist15`, `neg_joint`, `light_oblique`, …) so the
   manifest can be grouped by prefix.
5. Do not move the camera within a session.
6. After each block, `tools/dataset_manifest.py scan --merge` and fill the human columns **while the
   conditions are still fresh in mind**. Retrospective metadata is unreliable metadata.
7. Keep a spirit level or an angle gauge for the angle block; `camera_angle_deg` is measured, not guessed.
8. Photograph a ruler in-frame at least once per distance so apparent crack width can be sanity-checked.

---

## 7. Explicitly not in this plan

- Pixel-level ground-truth labelling and any IoU/Dice/clIoU computation (`VALIDATION_PLAN.md` §L3).
- Comparison against CrackSAM-adapter / OmniCrack30k / Hybrid-Segmentor.
- Any fine-tuning or hard-negative enrichment (`adr/010`).
- Real-world crack-width or severity measurement.
- 3D reconstruction of the crack (`adr/002`).
