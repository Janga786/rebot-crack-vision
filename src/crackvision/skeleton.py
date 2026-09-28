"""crackvision.skeleton — binary mask cleanup and one-pixel skeletonization. Built by TC-010.

Algorithm is exactly `remove_small_objects` then `skimage.morphology.skeletonize` — nothing more
(`adr/008`). Any further analysis of the skeleton's shape or connectivity is deliberately out of
scope: the one-pixel raster is this phase's perception endpoint, and its statistics are the
evidence base for a later, separate design of that analysis.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
from skimage.measure import label
from skimage.morphology import remove_small_objects, skeletonize

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)

STATS_SCHEMA_VERSION = 1


def load_prediction(path: Path) -> np.ndarray:
    """Load a prediction PNG as a 2-D uint8 array, values whatever nnU-Net wrote (0/1 or 0/255)."""
    with Image.open(path) as img:
        img.load()
        return np.array(img)


def binarize(prediction: np.ndarray) -> np.ndarray:
    """`pred > 0` — never assume 255, never assume 1 (docs/RISKS.md R-03)."""
    return np.asarray(prediction) > 0


def clean_and_skeletonize(binary: np.ndarray, min_component_size: int) -> tuple[np.ndarray, np.ndarray, int, int]:
    """Return `(cleaned, skeleton, components_before, components_after)` for a boolean mask.

    Exactly the algorithm from docs/INTERFACES.md §3.4: `remove_small_objects` (skipped when
    `min_component_size <= 0`) then `skeletonize` with the default method. Nothing else.
    """
    components_before = int(label(binary, connectivity=2).max())
    if min_component_size > 0:
        # skimage >= 0.26 renamed min_size -> max_size and made it inclusive (removes area <=
        # max_size, where min_size removed area < min_size); max_size=min_component_size - 1
        # reproduces the "drop components below min_component_size" behaviour this card requires.
        cleaned = remove_small_objects(binary, max_size=min_component_size - 1)
    else:
        cleaned = binary
    components_after = int(label(cleaned, connectivity=2).max())
    skeleton = skeletonize(cleaned)
    return cleaned, skeleton, components_before, components_after


def render_skeleton(skeleton: np.ndarray) -> np.ndarray:
    """Single-channel uint8, `skeleton * 255`, same H×W as the input."""
    return skeleton.astype(np.uint8) * 255


def render_overlay(original_rgb: np.ndarray, skeleton: np.ndarray, color: tuple[int, int, int], dilate: int) -> np.ndarray:
    """Draw `skeleton` opaque over `original_rgb` in `color`; `dilate` thickens for viewing only."""
    draw_mask = skeleton
    if dilate > 0:
        from scipy.ndimage import binary_dilation

        draw_mask = binary_dilation(skeleton, iterations=dilate)
    out = original_rgb.copy()
    out[draw_mask] = np.array(color, dtype=out.dtype)
    return out


def skeletonize_case(
    cfg: Config,
    case_entry: dict[str, Any],
    *,
    min_component_size: int,
    dilate: int,
    color: tuple[int, int, int],
    predictions_dir: Path,
    skeletons_dir: Path,
    write_overlay: bool = True,
    skip_existing: bool = False,
    logger: logging.Logger | None = None,
) -> dict[str, Any]:
    """Clean, skeletonize and score one case. Returns a result dict with at least `status`."""
    log = logger or logging.getLogger("crackvision.skeleton")
    case_id = case_entry["case_id"]

    source_path = Path(case_entry["source_path"])
    if not source_path.is_absolute():
        source_path = cfg.root / source_path

    prediction_path = predictions_dir / f"{case_id}.png"
    skeleton_out = skeletons_dir / f"{case_id}_skeleton.png"
    overlay_out = skeletons_dir / f"{case_id}_skeleton_overlay.png"
    stats_out = skeletons_dir / f"{case_id}_skeleton_stats.json"

    expected_outputs = [skeleton_out, stats_out] + ([overlay_out] if write_overlay else [])
    if skip_existing and all(p.is_file() for p in expected_outputs):
        log.info("skip-existing: case %s already has all outputs", case_id)
        return {"case_id": case_id, "status": "skipped", "skeleton_path": skeleton_out, "stats_path": stats_out}

    if not prediction_path.is_file():
        msg = f"prediction missing: {prediction_path}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    try:
        prediction = load_prediction(prediction_path)
    except Exception as exc:  # noqa: BLE001 — one bad case must never abort the batch
        msg = f"failed to read prediction {prediction_path}: {type(exc).__name__}: {exc}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    binary = binarize(prediction)
    height, width = binary.shape[:2]

    _cleaned, skeleton, components_before, components_after = clean_and_skeletonize(binary, min_component_size)

    mask_pixels = int(binary.sum())
    total_pixels = int(binary.size)
    mask_fraction = mask_pixels / total_pixels if total_pixels else 0.0
    skeleton_pixels = int(skeleton.sum())
    skeleton_fraction = skeleton_pixels / total_pixels if total_pixels else 0.0

    for path in expected_outputs:
        path.parent.mkdir(parents=True, exist_ok=True)

    skeleton_img = Image.fromarray(render_skeleton(skeleton), mode="L")
    skeleton_img.save(skeleton_out, format="PNG")

    if write_overlay:
        if not source_path.is_file():
            msg = f"original missing: {source_path}"
            log.error("case %s: %s", case_id, msg)
            return {"case_id": case_id, "status": "failed", "error": msg}
        try:
            with Image.open(source_path) as img:
                img.load()
                original_rgb = np.array(img.convert("RGB"))
        except Exception as exc:  # noqa: BLE001
            msg = f"failed to read original {source_path}: {type(exc).__name__}: {exc}"
            log.error("case %s: %s", case_id, msg)
            return {"case_id": case_id, "status": "failed", "error": msg}
        if original_rgb.shape[:2] != (height, width):
            msg = f"shape mismatch: prediction {(height, width)} != original {original_rgb.shape[:2]}"
            log.error("case %s: %s", case_id, msg)
            return {"case_id": case_id, "status": "failed", "error": msg}
        overlay_arr = render_overlay(original_rgb, skeleton, color, dilate)
        Image.fromarray(overlay_arr, mode="RGB").save(overlay_out, format="PNG")

    stats = {
        "case_id": case_id,
        "schema_version": STATS_SCHEMA_VERSION,
        "image_height": height,
        "image_width": width,
        "mask_pixels": mask_pixels,
        "mask_fraction": mask_fraction,
        "components_before": components_before,
        "components_after": components_after,
        "min_component_size": min_component_size,
        "skeleton_pixels": skeleton_pixels,
        "skeleton_fraction": skeleton_fraction,
    }
    stats_out.write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")

    log.info(
        "case %s: mask %d px, skeleton %d px, components %d -> %d",
        case_id,
        mask_pixels,
        skeleton_pixels,
        components_before,
        components_after,
    )

    return {
        "case_id": case_id,
        "status": "ok",
        "skeleton_path": skeleton_out,
        "overlay_path": overlay_out if write_overlay else None,
        "stats_path": stats_out,
        **stats,
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
        prog="skeleton.py",
        description=(
            "Clean small components and skeletonize nnU-Net predictions to a one-pixel centerline "
            "raster (docs/INTERFACES.md §3.4)."
        ),
    )
    add_common_args(parser)
    parser.add_argument(
        "--cases", nargs="+", default=None, metavar="CASE_ID", help="restrict to named case_id(s) (default: all)"
    )
    parser.add_argument(
        "--min-component-size",
        type=int,
        default=None,
        metavar="N",
        help="drop connected components below N pixels before skeletonizing; 0 disables (default: config/project.yaml)",
    )
    parser.add_argument(
        "--dilate", type=int, default=0, metavar="N", help="thicken the skeleton in the overlay only, for viewing"
    )
    parser.add_argument("--no-overlay", action="store_true", help="skip writing the *_skeleton_overlay.png file")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"skeleton: {exc}", file=sys.stderr)
        return EXIT_USAGE

    min_component_size = (
        cfg.skeleton["min_component_size"] if args.min_component_size is None else args.min_component_size
    )
    color = tuple(cfg.visualization["skeleton_color"])

    logger = setup_logging("skeleton", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    case_map_path = cfg.root / "data" / "case_map.json"
    if not case_map_path.is_file():
        msg = "run: ./env.sh python -m crackvision.prepare_inputs first"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "skeleton", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    try:
        case_map = json.loads(case_map_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        msg = f"malformed {case_map_path}: {exc}"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "skeleton", "failed", EXIT_RUNTIME)
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
    skeletons_dir = cfg.paths["skeletons"]
    write_overlay = not args.no_overlay

    if args.dry_run:
        for case_entry in cases:
            logger.info("dry-run: would skeletonize case %s", case_entry["case_id"])
        logger.info("dry-run: no files written")
        return EXIT_OK

    for case_entry in cases:
        result = skeletonize_case(
            cfg,
            case_entry,
            min_component_size=min_component_size,
            dilate=args.dilate,
            color=color,
            predictions_dir=predictions_dir,
            skeletons_dir=skeletons_dir,
            write_overlay=write_overlay,
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
        summary.write(cfg, "skeleton", "partial", EXIT_RUNTIME)
        return EXIT_RUNTIME

    summary.write(cfg, "skeleton", "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
