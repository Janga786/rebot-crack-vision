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
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFilter
from skimage.morphology import remove_small_objects, skeletonize

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.inference import run_inference
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


def _draw_crack(base: np.ndarray, rng: np.random.Generator) -> Image.Image:
    """Draw the wandering polyline + branch onto a copy of `base` (never mutates the caller's array)."""
    img = Image.fromarray(base.copy(), mode="RGB")
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


def generate_fixtures(input_dir: Path, logger: Any) -> dict[str, Path]:
    """Write the two deterministic synthetic fixtures into input_dir. Pure numpy + PIL, no network.

    Both fixtures share exactly one noisy base (a single seeded draw), so synthetic_blank is the
    untouched base and synthetic_crack is that same base with a polyline drawn on a copy of it —
    a matched pair differing only in the crack region (plus the crack image's own blur). The
    polyline geometry is drawn from a second, separately seeded generator so its shape stays
    reproducible regardless of how much of the base generator's stream noise generation consumed.
    """
    input_dir.mkdir(parents=True, exist_ok=True)
    base = _noisy_base(_rng())
    geometry_rng = _rng()
    paths: dict[str, Path] = {}

    crack_img = _draw_crack(base, geometry_rng)
    crack_path = input_dir / "synthetic_crack.png"
    crack_img.save(crack_path, format="PNG", compress_level=6)
    paths["synthetic_crack"] = crack_path

    blank_img = Image.fromarray(base, mode="RGB")
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


def check_model_folder(cfg: Config) -> list[str]:
    """Return the required model paths that are missing; empty if the model folder is intact.

    Mirrors build_predict_command's model_dir derivation. Existence-only: this never downloads or
    modifies anything, it just gives a precise exit-3 error before the subprocess is launched
    (adr/006), instead of letting nnU-Net's own FileNotFoundError surface as an exit-1 traceback.
    """
    model_dir = cfg.paths["models"] / cfg.model["dataset_name"] / cfg.model["trainer_config"]
    required = [
        model_dir / "plans.json",
        model_dir / "dataset.json",
        model_dir / f"fold_{cfg.model['fold']}" / cfg.model["checkpoint"],
    ]
    return [str(p) for p in required if not p.is_file()]


def render_visuals(
    case: str,
    pred: np.ndarray,
    overlays_dir: Path,
    skeletons_dir: Path,
    original_path: Path,
    logger: Any,
) -> dict[str, Any]:
    """Minimal mask + red overlay + skeleton for eyeballing (TC-009/TC-010 do the real versions later).

    Callers must validate `pred`'s dtype/unique-values/shape before calling this (evaluate_case
    does) — a malformed prediction should fail cleanly, not crash inside the overlay blend below.
    """
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
    }


def evaluate_case(
    case: str,
    predictions_dir: Path,
    original_path: Path,
    overlays_dir: Path,
    skeletons_dir: Path,
    logger: Any,
) -> tuple[list[str], dict[str, Any] | None, float, int]:
    """Check one case's prediction against the {0,1}/dtype/frame-invariant contract, then render.

    The dtype/unique-values/shape assertions run BEFORE render_visuals() is ever called, so a
    malformed prediction (wrong shape, wrong dtype) fails this case cleanly instead of crashing
    inside the overlay blend. Returns (failures, result_or_None, render_seconds, assertions_run) —
    result is None when a precondition assertion already failed and rendering was skipped.
    """
    failures: list[str] = []
    checks = 0

    pred_path = predictions_dir / f"{case}.png"
    if not pred_path.is_file() or pred_path.stat().st_size == 0:
        return [f"{case}: prediction missing or empty at {pred_path}"], None, 0.0, checks

    with Image.open(original_path) as orig_im:
        original_shape = (orig_im.height, orig_im.width)

    pred = np.array(Image.open(pred_path))
    pred_dtype = pred.dtype
    pred_unique = sorted(int(v) for v in np.unique(pred))
    pred_shape = pred.shape

    checks += 1
    if pred_dtype != np.uint8:
        failures.append(f"{case}: prediction dtype is {pred_dtype}, expected uint8")
    checks += 1
    if not set(pred_unique).issubset({0, 1}):
        failures.append(f"{case}: prediction unique values {pred_unique} not subset of {{0,1}}")
    checks += 1
    if pred_shape != original_shape:
        failures.append(f"{case}: prediction shape {pred_shape} != original image shape {original_shape}")

    if failures:
        return failures, None, 0.0, checks

    t0 = time.monotonic()
    result = render_visuals(case, pred, overlays_dir, skeletons_dir, original_path, logger)
    render_seconds = time.monotonic() - t0
    result["pred_dtype"] = pred_dtype
    result["pred_unique"] = pred_unique
    result["pred_shape"] = pred_shape

    for label, path in (("mask", result["mask_path"]), ("overlay", result["overlay_path"]), ("skeleton", result["skeleton_path"])):
        checks += 1
        if not path.is_file():
            failures.append(f"{case}: {label} file missing at {path}")
            continue
        with Image.open(path) as im:
            if im.size != (IMAGE_SIZE, IMAGE_SIZE):
                failures.append(f"{case}: {label} size {im.size} != ({IMAGE_SIZE},{IMAGE_SIZE})")

    checks += 1
    if result["skeleton_pixels"] > result["mask_pixels"]:
        failures.append(
            f"{case}: skeleton_pixels ({result['skeleton_pixels']}) > mask_pixels ({result['mask_pixels']})"
        )

    return failures, result, render_seconds, checks


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
    ext_trainer_dir = smoke_root / "ext_trainer"
    clean_dirs = (input_dir, nnunet_input_dir, predictions_dir, overlays_dir, skeletons_dir, ext_trainer_dir)

    # --dry-run is checked before --clean so "smoke_test.py --dry-run --clean" only logs what
    # --clean would remove and deletes/writes nothing (docs/INTERFACES.md §0.3).
    if args.dry_run:
        if args.clean:
            logger.info("dry-run: would remove %s", ", ".join(str(d) for d in clean_dirs))
        logger.info("dry-run: would generate fixtures, run nnUNetv2_predict_from_modelfolder, render visuals")
        logger.info("dry-run: logs/smoke_test_latest.json not written")
        return EXIT_OK

    if args.clean:
        for d in clean_dirs:
            if d.exists():
                shutil.rmtree(d)
        logger.info("--clean: removed prior %s", ", ".join(str(d) for d in clean_dirs))

    if args.device == "cuda":
        import torch

        if not torch.cuda.is_available():
            logger.error("--device cuda requested but torch.cuda.is_available() is False")
            summary.add_error("cuda requested but unavailable")
            summary.write(cfg, "smoke_test", "precondition", EXIT_PRECONDITION)
            return EXIT_PRECONDITION

    missing_model_paths = check_model_folder(cfg)
    if missing_model_paths:
        for path_str in missing_model_paths:
            logger.error("model file missing: %s", path_str)
        logger.error(
            "run: ./env.sh python scripts/fetch_model.py (TC-004), then: "
            "./env.sh python scripts/verify_model.py (TC-005)"
        )
        summary.add_error(f"model folder incomplete; missing: {', '.join(missing_model_paths)}")
        summary.write(cfg, "smoke_test", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    timings: dict[str, float] = {}

    t0 = time.monotonic()
    fixtures = generate_fixtures(input_dir, logger)
    timings["generate_fixtures"] = time.monotonic() - t0

    t0 = time.monotonic()
    convert_to_nnunet_input(fixtures, nnunet_input_dir, logger)
    timings["convert_inputs"] = time.monotonic() - t0

    t0 = time.monotonic()
    inference_result = run_inference(
        cfg,
        device=args.device,
        input_dir=nnunet_input_dir,
        output_dir=predictions_dir,
        logger=logger,
    )
    timings["nnunet_predict"] = time.monotonic() - t0
    command = inference_result.get("command", [])

    if inference_result["status"] != "ok":
        for err in inference_result.get("errors", []):
            summary.add_error(err)
        summary.write(cfg, "smoke_test", "failed", EXIT_RUNTIME)
        print("=" * 60)
        print("  SMOKE TEST FAILED   (nnU-Net subprocess did not exit 0)")
        print("=" * 60)
        return EXIT_RUNTIME

    failures: list[str] = []
    per_case: dict[str, dict[str, Any]] = {}
    assertions_per_case: dict[str, int] = {}

    for case in CASES:
        original_path = fixtures[case]
        case_failures, result, render_seconds, checks = evaluate_case(
            case, predictions_dir, original_path, overlays_dir, skeletons_dir, logger
        )
        failures.extend(case_failures)
        assertions_per_case[case] = checks
        if result is not None:
            timings[f"render_{case}"] = render_seconds
            per_case[case] = result

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
    n_assertions_per_image = max(assertions_per_case.values(), default=0)
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
