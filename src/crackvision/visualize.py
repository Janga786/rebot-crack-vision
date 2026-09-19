"""crackvision.visualize — human-viewable masks, overlays and comparison strips. Built by TC-009.

Predictions are `{0,1}`-valued uint8 PNGs (docs/RISKS.md R-03) that look solid black to the human
eye. This module is the only place that renders them: `{case}_mask.png` (binarised, `{0,255}`),
`{case}_overlay.png` (alpha-blended over the original) and `{case}_comparison.png`
(ORIGINAL | MASK | OVERLAY, captioned). It never modifies a prediction file.

The original is loaded with the same `exif_transpose` + `convert("RGB")` steps `prepare_inputs`
(TC-007) applied before generating the nnU-Net input, so its dimensions match the prediction even
for an EXIF-rotated source (docs/COMPLETION_LOG.md, TC-007 review integration risk).
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)

CAPTION_TEXT_COLOR = (255, 255, 255)
CAPTION_BG_COLOR = (0, 0, 0)
CAPTION_PADDING_PX = 4


def _load_default_font() -> ImageFont.ImageFont:
    try:
        return ImageFont.load_default(size=16)
    except TypeError:
        return ImageFont.load_default()


def load_original_rgb(path: Path) -> Image.Image:
    """Open, exif-transpose and force to RGB — the same treatment `prepare_inputs` gave it."""
    with Image.open(path) as img:
        img.load()
        transposed = ImageOps.exif_transpose(img) or img
        return transposed.convert("RGB")


def load_prediction(path: Path) -> np.ndarray:
    """Load a prediction PNG as a 2-D uint8 array, values whatever nnU-Net wrote (0/1 or 0/255)."""
    with Image.open(path) as img:
        img.load()
        return np.array(img)


def binarize(prediction: np.ndarray) -> np.ndarray:
    """`pred > 0` — never assume 255, never assume 1 (docs/RISKS.md R-03)."""
    return np.asarray(prediction) > 0


def render_mask(binary: np.ndarray) -> np.ndarray:
    """Single-channel uint8, `binary * 255`."""
    return binary.astype(np.uint8) * 255


def render_overlay(original_rgb: np.ndarray, binary: np.ndarray, alpha: float, color: tuple[int, int, int]) -> np.ndarray:
    """Alpha-blend `color` over `original_rgb` where `binary`, elsewhere pass the original through.

    Blended in float, cast back to uint8 at the end (never blend in uint8 — INTERFACES.md §3.3).
    """
    orig_f = original_rgb.astype(np.float32)
    color_f = np.array(color, dtype=np.float32)
    blended = orig_f * (1.0 - alpha) + color_f * alpha
    out = np.where(binary[..., None], blended, orig_f)
    return np.clip(np.round(out), 0, 255).astype(np.uint8)


def _caption(panel: Image.Image, text: str) -> Image.Image:
    """Draw `text` top-left on a dark rectangle, legible over any background."""
    captioned = panel.copy()
    draw = ImageDraw.Draw(captioned)
    font = _load_default_font()
    left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
    text_w, text_h = right - left, bottom - top
    pad = CAPTION_PADDING_PX
    rect = (2, 2, 2 + text_w + 2 * pad, 2 + text_h + 2 * pad)
    draw.rectangle(rect, fill=CAPTION_BG_COLOR)
    draw.text((2 + pad - left, 2 + pad - top), text, fill=CAPTION_TEXT_COLOR, font=font)
    return captioned


def render_comparison(
    original_rgb: Image.Image, mask: Image.Image, overlay: Image.Image, gutter_px: int, crack_pct: float
) -> Image.Image:
    """ORIGINAL | MASK | OVERLAY concatenated horizontally with a white gutter, each captioned."""
    width, height = original_rgb.size
    panels = [
        _caption(original_rgb, "ORIGINAL"),
        _caption(mask.convert("RGB"), f"MASK {crack_pct:.3f}%"),
        _caption(overlay, "OVERLAY"),
    ]

    canvas = Image.new("RGB", (width * 3 + gutter_px * 2, height), color=(255, 255, 255))
    x = 0
    for panel in panels:
        canvas.paste(panel, (x, 0))
        x += panel.width + gutter_px
    return canvas


def visualize_case(
    cfg: Config,
    case_entry: dict[str, Any],
    *,
    alpha: float,
    color: tuple[int, int, int],
    gutter_px: int,
    predictions_dir: Path,
    overlays_dir: Path,
    comparisons_dir: Path,
    skip_existing: bool = False,
    logger: logging.Logger | None = None,
) -> dict[str, Any]:
    """Render one case's mask/overlay/comparison. Returns a result dict with at least `status`."""
    log = logger or logging.getLogger("crackvision.visualize")
    case_id = case_entry["case_id"]

    source_path = Path(case_entry["source_path"])
    if not source_path.is_absolute():
        source_path = cfg.root / source_path

    prediction_path = predictions_dir / f"{case_id}.png"
    mask_out = overlays_dir / f"{case_id}_mask.png"
    overlay_out = overlays_dir / f"{case_id}_overlay.png"
    comparison_out = comparisons_dir / f"{case_id}_comparison.png"

    if skip_existing and mask_out.is_file() and overlay_out.is_file() and comparison_out.is_file():
        log.info("skip-existing: case %s already has all three outputs", case_id)
        return {
            "case_id": case_id,
            "status": "skipped",
            "mask_path": mask_out,
            "overlay_path": overlay_out,
            "comparison_path": comparison_out,
        }

    if not prediction_path.is_file():
        msg = f"prediction missing: {prediction_path}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    if not source_path.is_file():
        msg = f"original missing: {source_path}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    try:
        original_img = load_original_rgb(source_path)
    except Exception as exc:  # noqa: BLE001 — one bad case must never abort the batch
        msg = f"failed to read original {source_path}: {type(exc).__name__}: {exc}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    try:
        prediction = load_prediction(prediction_path)
    except Exception as exc:  # noqa: BLE001
        msg = f"failed to read prediction {prediction_path}: {type(exc).__name__}: {exc}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    original_arr = np.array(original_img)
    if prediction.shape[:2] != original_arr.shape[:2]:
        msg = f"shape mismatch: prediction {prediction.shape[:2]} != original {original_arr.shape[:2]}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    binary = binarize(prediction)
    crack_pixels = int(binary.sum())
    total_pixels = int(binary.size)
    crack_fraction = crack_pixels / total_pixels if total_pixels else 0.0

    mask_arr = render_mask(binary)
    overlay_arr = render_overlay(original_arr, binary, alpha, color)

    mask_img = Image.fromarray(mask_arr, mode="L")
    overlay_img = Image.fromarray(overlay_arr, mode="RGB")
    comparison_img = render_comparison(original_img, mask_img, overlay_img, gutter_px, crack_fraction * 100)

    for path in (mask_out, overlay_out, comparison_out):
        path.parent.mkdir(parents=True, exist_ok=True)
    mask_img.save(mask_out, format="PNG")
    overlay_img.save(overlay_out, format="PNG")
    comparison_img.save(comparison_out, format="PNG")

    log.info("case %s: %d crack pixels (%.3f%%)", case_id, crack_pixels, crack_fraction * 100)

    return {
        "case_id": case_id,
        "status": "ok",
        "mask_path": mask_out,
        "overlay_path": overlay_out,
        "comparison_path": comparison_out,
        "width": original_arr.shape[1],
        "height": original_arr.shape[0],
        "crack_pixels": crack_pixels,
        "crack_fraction": crack_fraction,
    }


def _parse_color(text: str) -> tuple[int, int, int]:
    parts = text.split(",")
    if len(parts) != 3:
        raise ValueError(f"expected 'R,G,B', got {text!r}")
    try:
        r, g, b = (int(p.strip()) for p in parts)
    except ValueError as exc:
        raise ValueError(f"expected three integers 'R,G,B', got {text!r}") from exc
    for channel in (r, g, b):
        if not 0 <= channel <= 255:
            raise ValueError(f"color channel out of range 0-255: {text!r}")
    return (r, g, b)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="visualize.py",
        description=(
            "Render human-viewable masks, alpha-blended overlays and comparison strips from "
            "nnU-Net predictions (docs/INTERFACES.md §3.3)."
        ),
    )
    add_common_args(parser)
    parser.add_argument(
        "--cases", nargs="+", default=None, metavar="CASE_ID", help="restrict to named case_id(s) (default: all)"
    )
    parser.add_argument("--alpha", type=float, default=None, help="overlay blend alpha (default: config/project.yaml)")
    parser.add_argument(
        "--color", type=str, default=None, metavar="R,G,B", help="overlay colour (default: config/project.yaml)"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"visualize: {exc}", file=sys.stderr)
        return EXIT_USAGE

    try:
        color = _parse_color(args.color) if args.color is not None else tuple(cfg.visualization["crack_color"])
    except ValueError as exc:
        print(f"visualize: --color: {exc}", file=sys.stderr)
        return EXIT_USAGE

    alpha = cfg.visualization["overlay_alpha"] if args.alpha is None else args.alpha
    gutter_px = cfg.visualization["panel_gutter_px"]

    logger = setup_logging("visualize", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    case_map_path = cfg.root / "data" / "case_map.json"
    if not case_map_path.is_file():
        msg = "run: ./env.sh python -m crackvision.prepare_inputs first"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "visualize", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    try:
        case_map = json.loads(case_map_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        msg = f"malformed {case_map_path}: {exc}"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "visualize", "failed", EXIT_RUNTIME)
        return EXIT_RUNTIME

    all_cases: list[dict[str, Any]] = case_map.get("cases", [])
    if args.cases:
        wanted = set(args.cases)
        cases = [c for c in all_cases if c["case_id"] in wanted]
        for name in sorted(wanted - {c["case_id"] for c in all_cases}):
            logger.error("requested case_id not found in case_map.json: %s", name)
            summary.add_error(f"unknown case_id: {name}")
            summary.increment("failed")
    else:
        cases = all_cases

    predictions_dir = cfg.paths["predictions"]
    overlays_dir = cfg.paths["overlays"]
    comparisons_dir = cfg.paths["comparisons"]

    if args.dry_run:
        for case_entry in cases:
            logger.info("dry-run: would render mask/overlay/comparison for case %s", case_entry["case_id"])
        logger.info("dry-run: no files written")
        return EXIT_OK

    for case_entry in cases:
        result = visualize_case(
            cfg,
            case_entry,
            alpha=alpha,
            color=color,
            gutter_px=gutter_px,
            predictions_dir=predictions_dir,
            overlays_dir=overlays_dir,
            comparisons_dir=comparisons_dir,
            skip_existing=args.skip_existing,
            logger=logger,
        )
        status = result["status"]
        if status == "failed":
            summary.increment("failed")
            summary.add_error(f"{result['case_id']}: {result.get('error', 'failed')}")
        elif status == "skipped":
            summary.increment("skipped")
        else:
            summary.increment("processed")

    failed = summary.counts.get("failed", 0)
    if failed:
        summary.write(cfg, "visualize", "partial", EXIT_RUNTIME)
        return EXIT_RUNTIME

    summary.write(cfg, "visualize", "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
