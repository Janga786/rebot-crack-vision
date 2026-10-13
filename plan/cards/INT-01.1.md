# INT-01.1 — Synthetic RGB-D case through the real segmentation-to-paths-to-3D chain (no ROS)

```json card
{
  "kind": "impl",
  "requirements": [
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "scripts/perc/run_synthetic_perception.sh"
    ]
  },
  "inputs": [
    "tools/synth_scene3d.py",
    "src/crackvision/naming.py",
    "src/crackvision/visualize.py",
    "src/crackvision/skeleton.py",
    "src/crackvision/paths.py",
    "src/crackvision/visualize_paths.py",
    "src/crackvision/path3d.py",
    "docs/INTERFACES.md#2",
    "docs/INTERFACES.md#3.13",
    "docs/INTERFACES.md#10",
    "docs/geometry/PATH3D_VERIFICATION.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "run",
        "cmd": "bash scripts/perc/run_synthetic_perception.sh",
        "timeout_s": 180,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Reuses tools/synth_scene3d.py unmodified (GEOM-08.8's generator is not edited) to write a fresh case under a self-stamped logs/synthetic_perception/<UTC stamp>/ scratch root; nothing under data/ or config/ in the repo tree is written or modified.",
      "The generator's own dilated mask (naming.mask_path) is copied to naming.prediction_path as the one deliberate stand-in for the OpenCrack segmentation step (out of scope per ADR-010/accepted PERC cards); this is explained in a comment, not hidden.",
      "The scratch case_map.json entry for the case is replaced with a full docs/INTERFACES.md #2-compliant entry (case_id, source_path, source_name, width, height, downscaled, sha256_source, sha256_nnunet_input) pointing source_path at the generator's own data/synthetic/{case}_color.png placeholder -- not the minimal {case_id, downscaled} entry the generator itself writes.",
      "crackvision.visualize, crackvision.skeleton, crackvision.paths, crackvision.visualize_paths and crackvision.path3d are each invoked via `./env.sh python -m ...` (never imported/monkeypatched) and must each exit 0; the script fails loudly (prints to stderr, exits 1) on the first non-zero exit or missing expected output file, with a `fail()` helper mirroring scripts/ros/run_cartesian_synthetic.sh's own pattern.",
      "crackvision.skeleton's output (data/skeletons/{case}_skeleton_stats.json) is read back and asserted to report skeleton_pixels > 0 -- this is the one real step scripts/ros/run_cartesian_synthetic.sh (MOT-06.10) never exercises, since that script's generator writes its ground-truth skeleton directly rather than deriving one from a mask through TC-010's real remove_small_objects + skimage.skeletonize.",
      "crackvision.path3d's output (data/paths3d/{case}_paths3d.json) is read back and asserted to have at least one component and execution_eligible == false with ineligible_reasons including the nominal/synthetic-geometry reasons already documented for this generator's output (wrist_camera_nominal, tool_nominal, optical_frames_nominal, synthetic_capture) -- this script never claims eligibility the generator's nominal config doesn't support.",
      "The script never imports rclpy/moveit, never starts any ROS node, and never requires the ros2_ws overlay to be built.",
      "Script prints a `scratch root <path>` line immediately after creating ROOT (same convention as scripts/ros/run_cartesian_synthetic.sh) so a caller script can recover it, and a final `OK (<path>)` line with exit 0 on success."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 2,
    "task_class": "integration-test",
    "local_ok": false
  },
  "track": "simulation",
  "id": "INT-01.1",
  "title": "Synthetic RGB-D case through the real segmentation-to-paths-to-3D chain (no ROS)",
  "parent": "INT-01",
  "outcome": "One command (scripts/perc/run_synthetic_perception.sh) builds a GEOM-08.8 synthetic case and runs it, as a stand-in predicted mask, through the real crackvision.visualize -> crackvision.skeleton -> crackvision.paths -> crackvision.visualize_paths -> crackvision.path3d CLIs, asserting exit 0 and the expected output files/fields at every step, with no ROS and no hardware."
}
```

## Why

INT-01's outcome is "Synthetic RGB-D scene -> paths -> 3D -> MoveIt plan (mock) -> limits, one command." The existing synthetic-planning chain (`scripts/ros/run_cartesian_synthetic.sh`, MOT-06.10) already runs `tools/synth_scene3d.py -> crackvision.paths -> crackvision.path3d -> plan_view/plan_crack_path`, but it never exercises the real `crackvision.visualize`/`crackvision.skeleton` CLIs (TC-010/PERC-02's own mask-cleanup + skeletonize) or `crackvision.visualize_paths` (PERC-04, one of this card's own direct dependencies) -- the generator writes its ground-truth 1-px skeleton directly and `crackvision.paths` consumes that, bypassing the real segmentation-to-skeleton step entirely. This card closes that gap as its own small, ROS-free script; INT-01.2 chains it into the full MoveIt-mock pipeline.

## What to build

`scripts/perc/run_synthetic_perception.sh` (new file, new `scripts/perc/` directory -- mirrors `scripts/geometry/`, `scripts/motion/`). Follow the structure, tone and `fail()`/stamp conventions of `scripts/ros/run_cartesian_synthetic.sh` exactly (top-of-file comment explaining what/why, `set -uo pipefail`, a `fail()` helper, a `STAMP`/`ROOT` under `logs/synthetic_perception/<stamp>/`, an early `echo "...: scratch root $ROOT"` line, a final `OK ($ROOT)` + `exit 0`).

Steps:

1. `CASE_ID=synth_int01`; `ROOT="$REPO_ROOT/logs/synthetic_perception/$STAMP"`.
2. `"$REPO_ROOT/env.sh" python "$REPO_ROOT/tools/synth_scene3d.py" --out "$ROOT" --seed 0 --case-id "$CASE_ID"` (unmodified generator; writes the GT skeleton, the dilated mask at `naming.mask_path`, `data/synthetic/{case}_color.png`/`_depth.png`/`_gt.json`, the capture record and a minimal `case_map.json` entry).
3. A small inline Python block (heredoc, like `run_cartesian_synthetic.sh`'s scene-fixup step) that, using only `crackvision.naming`/stdlib:
   - copies `ROOT/<naming.mask_path(case)>` to `ROOT/<naming.prediction_path(case)>` (the stand-in predicted mask);
   - copies `ROOT/data/synthetic/{case}_color.png` to `ROOT/<naming.nnunet_input_path(case)>` (never fed to nnU-Net -- this file only exists so the case_map's `sha256_nnunet_input` field means something; do not call any `nnUNetv2_*` tool, ADR-010);
   - rewrites `ROOT/data/case_map.json`'s single entry for `case_id` to the full §2 schema (`case_id, source_path (posix-relative to ROOT), source_name, width, height, downscaled: false, sha256_source, sha256_nnunet_input`), `width`/`height` read from the color PNG, both sha256s computed from the files on disk (mirror `prepare_inputs.py`'s own `_sha256`/`_relposix` helpers rather than re-deriving the logic differently).
4. `./env.sh python -m crackvision.visualize --root "$ROOT" --cases "$CASE_ID" -q`; assert exit 0 and that `data/overlays/{case}_mask.png`, `_overlay.png` and `data/comparisons/{case}_comparison.png` exist.
5. `./env.sh python -m crackvision.skeleton --root "$ROOT" --cases "$CASE_ID" -q`; assert exit 0 and that `data/skeletons/{case}_skeleton.png`/`_skeleton_stats.json` exist; read the stats JSON and assert `skeleton_pixels > 0`.
6. `./env.sh python -m crackvision.paths --root "$ROOT" --cases "$CASE_ID" -q`; assert exit 0 and `data/paths/{case}_paths.json` exists.
7. `./env.sh python -m crackvision.visualize_paths --root "$ROOT" --cases "$CASE_ID" -q`; assert exit 0 and its overlay output exists.
8. `./env.sh python -m crackvision.path3d --root "$ROOT" --cases "$CASE_ID" -q`; assert exit 0 and `data/paths3d/{case}_paths3d.json` exists; load it and assert `execution_eligible is False` with `ineligible_reasons` a superset of `{wrist_camera_nominal, tool_nominal, optical_frames_nominal, synthetic_capture}` (same reasons `docs/motion/CARTESIAN_PLANNING.md` already documents for this generator); print the headline component/segment counts.
9. `echo "run_synthetic_perception.sh: OK ($ROOT)"`; `exit 0`.

Use only `./env.sh python -m crackvision.*` invocations for every perception/geometry step (never import these modules directly from the shell script) so the script exercises exactly the CLI surface an operator would use. Do not modify `tools/synth_scene3d.py`, `src/crackvision/**` or any other card's owned files -- this is a pure orchestration script in a new file.

## Out of scope
- Any ROS/MoveIt step (INT-01.2's job).
- Modifying the GEOM-08.8 generator's own ground-truth skeleton/mask semantics.
- Running real OpenCrack/nnU-Net inference.
