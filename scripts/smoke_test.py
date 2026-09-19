"""scripts/smoke_test.py — synthetic end-to-end smoke test (Level 2 core). Built by TC-006.

Generates two deterministic synthetic fixtures, runs the *real* OpenCrack nnU-Net model on them via
the real `nnUNetv2_predict_from_modelfolder` CLI, and asserts the outputs are well-formed. This is
the first time the model actually runs: it proves environment + model folder + CLI flags + GPU work
together end to end, and it pins the {0,1}-valued-prediction convention with a test.

This is an EXECUTION test, not an accuracy test (adr/001, docs/VALIDATION_PLAN.md L2). It asserts
shape, dtype and value-set invariants only — never IoU, Dice, recall, or that any crack pixel was
found. All output lives under data/smoke_test/ and is never written into data/predictions/ or any
other pipeline directory (docs/INTERFACES.md §3.8, task_cards/TC-006-gpu-smoke-test.md).
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFilter
from skimage.morphology import remove_small_objects, skeletonize

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE, RunSummary, setup_logging

SEED = 1234
IMAGE_SIZE = 512
CASES = ("synthetic_crack", "synthetic_blank")
SKELETON_MIN_COMPONENT_SIZE = 64


def _rng() -> np.random.Generator:
    return np.random.default_rng(SEED)


def _noisy_base(rng: np.random.Generator) -> np.ndarray:
    base = np.full((IMAGE_SIZE, IMAGE_SIZE, 3), 128.0)
    noise = rng.normal(loc=0.0, scale=12.0, size=(IMAGE_SIZE, IMAGE_SIZE, 3))
    arr = np.clip(base + noise, 0, 255).astype(np.uint8)
    return arr


def _draw_crack(rng: np.random.Generator) -> Image.Image:
    arr = _noisy_base(rng)
    img = Image.fromarray(arr, mode="RGB")
    draw = ImageDraw.Draw(img)

    # main wandering polyline from ~(60,40) to ~(460,470)
    n_points = 12
    xs = np.linspace(60, 460, n_points) + rng.normal(0, 15, n_points)
    ys = np.linspace(40, 470, n_points) + rng.normal(0, 15, n_points)
    main_points = list(zip(xs.tolist(), ys.tolist()))
    draw.line(main_points, fill=(40, 40, 40), width=3)

    # one branch off the midpoint
    mid_idx = n_points // 2
    branch_start = main_points[mid_idx]
    branch_end = (branch_start[0] + 90, branch_start[1] - 70)
    draw.line([branch_start, branch_end], fill=(40, 40, 40), width=2)

    return img.filter(ImageFilter.GaussianBlur(radius=0.6))


def _draw_blank(rng: np.random.Generator) -> Image.Image:
    arr = _noisy_base(rng)
    return Image.fromarray(arr, mode="RGB")


def generate_fixtures(input_dir: Path, logger: Any) -> dict[str, Path]:
    """Write the two deterministic synthetic fixtures into input_dir. Pure numpy + PIL, no network."""
    input_dir.mkdir(parents=True, exist_ok=True)
    rng = _rng()
    paths: dict[str, Path] = {}

    crack_img = _draw_crack(rng)
    crack_path = input_dir / "synthetic_crack.png"
    crack_img.save(crack_path, format="PNG", compress_level=6)
    paths["synthetic_crack"] = crack_path

    blank_img = _draw_blank(rng)
    blank_path = input_dir / "synthetic_blank.png"
    blank_img.save(blank_path, format="PNG", compress_level=6)
    paths["synthetic_blank"] = blank_path

    logger.info("generated synthetic fixtures: %s, %s", crack_path, blank_path)
    return paths


def convert_to_nnunet_input(fixtures: dict[str, Path], nnunet_input_dir: Path, logger: Any) -> dict[str, Path]:
    """Force every fixture to 3-channel 8-bit RGB PNG named <case>_0000.png (same rules as TC-007)."""
    nnunet_input_dir.mkdir(parents=True, exist_ok=True)
    converted: dict[str, Path] = {}
    for case, src in fixtures.items():
        img = Image.open(src).convert("RGB")
        dst = nnunet_input_dir / f"{case}_0000.png"
        img.save(dst, format="PNG", compress_level=6)
        converted[case] = dst
        logger.info("converted %s -> %s (RGB, PNG)", src, dst)
    return converted


def build_predict_command(cfg: Config, input_dir: Path, output_dir: Path, device: str) -> list[str]:
    model_dir = cfg.paths["models"] / cfg.model["dataset_name"] / cfg.model["trainer_config"]
    return [
        "nnUNetv2_predict_from_modelfolder",
        "-i",
        str(input_dir),
        "-o",
        str(output_dir),
        "-m",
        str(model_dir),
        "-f",
        str(cfg.model["fold"]),
        "-chk",
        cfg.model["checkpoint"],
        "-device",
        device,
        "-npp",
        str(cfg.inference["npp"]),
        "-nps",
        str(cfg.inference["nps"]),
    ]


_OOM_MARKERS = ("out of memory", "cuda out of memory", "outofmemoryerror")


def _looks_like_oom(text: str) -> bool:
    lowered = text.lower()
    return any(marker in lowered for marker in _OOM_MARKERS)


# The released checkpoint's internal metadata (torch.load(..., weights_only=False)["trainer_name"])
# names a custom trainer, "nnUNetTrainerSaveEvery10", used by the OpenCrack authors to control
# checkpoint-saving cadence during their own training run. Only the checkpoint and planner configs
# were published (docs/COMPLETION_LOG.md TC-006) — the trainer subclass's source was not. nnU-Net's
# own inference code (nnunetv2/inference/predict_from_raw_data.py) resolves this name via
# recursive_find_trainer_class_by_name() and, failing that, via the documented `nnUNet_extTrainer`
# environment variable — this is upstream's own supported extension point for exactly this case, not
# an invented workaround. The model card (models/opencrack-nnunet/README.md) confirms the network
# architecture is fully specified by plans.json ("self-configuring... the configuration is the one
# the nnU-Net planner derives") with no mention of a custom architecture, and that training used "no
# early stopping" — consistent with a trainer subclass that only changes checkpoint-saving interval,
# not network construction (which nnUNetTrainer.build_network_architecture already provides and this
# shim does not override). This is inference-only: no training loop is invoked (adr/010).
_EXT_TRAINER_CLASS_NAME = "nnUNetTrainerSaveEvery10"
_EXT_TRAINER_SOURCE = f'''"""Inference-only shim for docs/COMPLETION_LOG.md TC-006 — see scripts/smoke_test.py.

Recreates the class NAME the OpenCrack checkpoint's "trainer_name" metadata field requires, so
nnU-Net's recursive_find_trainer_class_by_name() can resolve it via the nnUNet_extTrainer
environment variable. It inherits nnUNetTrainer unmodified: this file exists to satisfy an
isinstance/name lookup for INFERENCE, never to train (adr/010 — nnUNetv2_train is never invoked).
"""

from nnunetv2.training.nnUNetTrainer.nnUNetTrainer import nnUNetTrainer


class {_EXT_TRAINER_CLASS_NAME}(nnUNetTrainer):
    pass
'''


def prepare_ext_trainer_shim(smoke_root: Path, logger: Any) -> Path:
    """Write the nnUNet_extTrainer shim described above into data/smoke_test/ (never into models/)."""
    ext_dir = smoke_root / "ext_trainer"
    ext_dir.mkdir(parents=True, exist_ok=True)
    shim_path = ext_dir / f"{_EXT_TRAINER_CLASS_NAME}.py"
    shim_path.write_text(_EXT_TRAINER_SOURCE, encoding="utf-8")
    logger.info(
        "wrote nnUNet_extTrainer shim for checkpoint-embedded trainer_name=%r to %s "
        "(inherits nnUNetTrainer unmodified; inference-only, see module docstring)",
        _EXT_TRAINER_CLASS_NAME,
        shim_path,
    )
    return ext_dir


def run_inference(command: list[str], logger: Any, ext_trainer_dir: Path | None = None) -> tuple[int, str]:
    """Run the nnU-Net CLI as a list-form subprocess (never shell=True), streaming output into the log."""
    env = os.environ.copy()
    if ext_trainer_dir is not None:
        env["nnUNet_extTrainer"] = str(ext_trainer_dir)
    logger.info("nnU-Net command: %s", " ".join(command))
    if ext_trainer_dir is not None:
        logger.info("nnUNet_extTrainer=%s", ext_trainer_dir)
    proc = subprocess.run(command, capture_output=True, text=True, env=env)
    combined = proc.stdout + "\n" + proc.stderr
    for line in combined.splitlines():
        if line.strip():
            logger.info("nnunet: %s", line)
    return proc.returncode, combined


def render_visuals(
    case: str,
    pred_path: Path,
    overlays_dir: Path,
    skeletons_dir: Path,
    original_path: Path,
    logger: Any,
) -> dict[str, Any]:
    """Minimal mask + red overlay + skeleton for eyeballing (TC-009/TC-010 do the real versions later)."""
    pred = np.array(Image.open(pred_path))
    binary = pred > 0

    mask_u8 = (binary * 255).astype(np.uint8)
    overlays_dir.mkdir(parents=True, exist_ok=True)
    mask_path = overlays_dir / f"{case}_mask.png"
    Image.fromarray(mask_u8, mode="L").save(mask_path)

    original = np.array(Image.open(original_path).convert("RGB")).astype(np.float64)
    overlay = original.copy()
    color = np.array([255, 0, 0], dtype=np.float64)
    alpha = 0.5
    overlay[binary] = (1 - alpha) * original[binary] + alpha * color
    overlay_path = overlays_dir / f"{case}_overlay.png"
    Image.fromarray(overlay.astype(np.uint8), mode="RGB").save(overlay_path)

    skeletons_dir.mkdir(parents=True, exist_ok=True)
    cleaned = remove_small_objects(binary, min_size=SKELETON_MIN_COMPONENT_SIZE)
    skel = skeletonize(cleaned)
    skel_u8 = (skel * 255).astype(np.uint8)
    skeleton_path = skeletons_dir / f"{case}_skeleton.png"
    Image.fromarray(skel_u8, mode="L").save(skeleton_path)

    mask_pixels = int(binary.sum())
    skeleton_pixels = int(skel.sum())
    crack_pct = 100.0 * mask_pixels / binary.size
    logger.info(
        "case %s: %d crack pixels (%.3f%%), %d skeleton pixels", case, mask_pixels, crack_pct, skeleton_pixels
    )

    return {
        "mask_path": mask_path,
        "overlay_path": overlay_path,
        "skeleton_path": skeleton_path,
        "mask_pixels": mask_pixels,
        "skeleton_pixels": skeleton_pixels,
        "crack_pct": crack_pct,
        "pred_shape": pred.shape,
        "pred_dtype": pred.dtype,
        "pred_unique": sorted(int(v) for v in np.unique(pred)),
    }


def print_oom_ladder(logger: Any) -> None:
    logger.error("CUDA out of memory. Degradation ladder (ARCHITECTURE.md §9), try in order:")
    logger.error("  1. add --not_on_device to the nnU-Net command (moves tile aggregation to CPU RAM)")
    logger.error("  2. lower -npp/-nps (reduces RAM contention, not VRAM, but helps under load)")
    logger.error("  3. add --disable_tta (drops mirroring TTA, ~8x fewer forward passes)")
    logger.error("  4. downscale inputs via an explicit --max-side flag (breaks the frame invariant; not used here)")
    logger.error("  5. re-run with --device cpu (slow but always correct)")
    logger.error("Never kill a GPU process to make room.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="smoke_test.py",
        description="Synthetic end-to-end smoke test: real OpenCrack model, synthetic fixtures (docs/INTERFACES.md §3.8).",
    )
    add_common_args(parser)
    parser.add_argument("--device", choices=["cuda", "cpu"], default="cuda", help="inference device (default: cuda)")
    parser.add_argument("--keep", action="store_true", help="retain fixtures/outputs (default behaviour anyway)")
    parser.add_argument("--clean", action="store_true", help="remove data/smoke_test/** before running")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"smoke_test: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("smoke_test", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    smoke_root = cfg.paths["smoke_test"]
    input_dir = smoke_root / "input"
    nnunet_input_dir = smoke_root / "nnunet_input"
    predictions_dir = smoke_root / "predictions"
    overlays_dir = smoke_root / "overlays"
    skeletons_dir = smoke_root / "skeletons"

    if args.clean:
        import shutil

        for d in (input_dir, nnunet_input_dir, predictions_dir, overlays_dir, skeletons_dir):
            if d.exists():
                shutil.rmtree(d)
        logger.info("--clean: removed prior data/smoke_test/{input,nnunet_input,predictions,overlays,skeletons}")

    if args.dry_run:
        logger.info("dry-run: would generate fixtures, run nnUNetv2_predict_from_modelfolder, render visuals")
        logger.info("dry-run: logs/smoke_test_latest.json not written")
        return EXIT_OK

    if args.device == "cuda":
        import torch

        if not torch.cuda.is_available():
            logger.error("--device cuda requested but torch.cuda.is_available() is False")
            summary.add_error("cuda requested but unavailable")
            summary.write(cfg, "smoke_test", "precondition", EXIT_PRECONDITION)
            return EXIT_PRECONDITION

    timings: dict[str, float] = {}

    t0 = time.monotonic()
    fixtures = generate_fixtures(input_dir, logger)
    timings["generate_fixtures"] = time.monotonic() - t0

    t0 = time.monotonic()
    convert_to_nnunet_input(fixtures, nnunet_input_dir, logger)
    timings["convert_inputs"] = time.monotonic() - t0

    ext_trainer_dir = prepare_ext_trainer_shim(smoke_root, logger)

    command = build_predict_command(cfg, nnunet_input_dir, predictions_dir, args.device)
    t0 = time.monotonic()
    returncode, output = run_inference(command, logger, ext_trainer_dir=ext_trainer_dir)
    timings["nnunet_predict"] = time.monotonic() - t0

    if returncode != 0:
        if _looks_like_oom(output):
            print_oom_ladder(logger)
            summary.add_error("CUDA OOM during nnU-Net predict; see log for degradation ladder")
        else:
            logger.error("nnUNetv2_predict_from_modelfolder exited %d", returncode)
            summary.add_error(f"nnU-Net subprocess exited {returncode}")
        summary.write(cfg, "smoke_test", "failed", EXIT_RUNTIME)
        print("=" * 60)
        print("  SMOKE TEST FAILED   (nnU-Net subprocess did not exit 0)")
        print("=" * 60)
        return EXIT_RUNTIME

    failures: list[str] = []
    per_case: dict[str, dict[str, Any]] = {}

    for case in CASES:
        pred_path = predictions_dir / f"{case}.png"
        if not pred_path.is_file() or pred_path.stat().st_size == 0:
            failures.append(f"{case}: prediction missing or empty at {pred_path}")
            continue

        original_path = fixtures[case]
        t0 = time.monotonic()
        result = render_visuals(case, pred_path, overlays_dir, skeletons_dir, original_path, logger)
        timings[f"render_{case}"] = time.monotonic() - t0
        per_case[case] = result

        if result["pred_dtype"] != np.uint8:
            failures.append(f"{case}: prediction dtype is {result['pred_dtype']}, expected uint8")
        if not set(result["pred_unique"]).issubset({0, 1}):
            failures.append(f"{case}: prediction unique values {result['pred_unique']} not subset of {{0,1}}")
        if result["pred_shape"] != (IMAGE_SIZE, IMAGE_SIZE):
            failures.append(f"{case}: prediction shape {result['pred_shape']} != ({IMAGE_SIZE},{IMAGE_SIZE})")

        for label, path in (("mask", result["mask_path"]), ("overlay", result["overlay_path"]), ("skeleton", result["skeleton_path"])):
            if not path.is_file():
                failures.append(f"{case}: {label} file missing at {path}")
                continue
            with Image.open(path) as im:
                if im.size != (IMAGE_SIZE, IMAGE_SIZE):
                    failures.append(f"{case}: {label} size {im.size} != ({IMAGE_SIZE},{IMAGE_SIZE})")

        if result["skeleton_pixels"] > result["mask_pixels"]:
            failures.append(
                f"{case}: skeleton_pixels ({result['skeleton_pixels']}) > mask_pixels ({result['mask_pixels']})"
            )

    total_duration = sum(timings.values())

    if failures:
        for f in failures:
            logger.error("assertion failed: %s", f)
            summary.add_error(f)
        summary.write(cfg, "smoke_test", "failed", EXIT_RUNTIME)
        print("=" * 60)
        print(f"  SMOKE TEST FAILED   ({len(failures)} assertion(s) failed)")
        for f in failures:
            print(f"    - {f}")
        print("=" * 60)
        return EXIT_RUNTIME

    summary.increment("images", len(per_case))
    summary.write(cfg, "smoke_test", "ok", EXIT_OK)

    print("=" * 60)
    n_assertions_per_image = 6
    print(
        f"  SMOKE TEST PASSED   ({n_assertions_per_image} assertions x {len(per_case)} images, "
        f"{total_duration:.1f} s)"
    )
    print(f"  predictions: {predictions_dir}/")
    pct_str = "   ".join(f"{case} {res['crack_pct']:.2f}%" for case, res in per_case.items())
    print(f"  crack pixels: {pct_str}")
    print("  NOTE: crack-pixel counts are informational only; this test")
    print("        verifies execution, not accuracy.")
    print("=" * 60)
    print("Per-stage timings:")
    for stage, seconds in timings.items():
        print(f"  {stage:<20s} {seconds:.3f}s")
    print(f"nnU-Net command used: {' '.join(command)}")
    for case, result in per_case.items():
        print(f"  {case}: dtype={result['pred_dtype']}, unique={result['pred_unique']}, shape={result['pred_shape']}")

    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
