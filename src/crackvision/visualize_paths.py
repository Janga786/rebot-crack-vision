"""crackvision.visualize_paths — path overlays: order, direction and branches. Built by PERC-04.

Reads a `{case}_paths.json` document (PERC-03, docs/INTERFACES.md §3.13) and the case's original
image, and draws a human-viewable overlay showing crack order, travel direction and branch/main
distinction. This module only reads existing data artefacts and writes a new overlay PNG — it
never modifies `{case}_paths.json`, a skeleton, a prediction or an original image.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)
from crackvision.visualize import load_original_rgb

DEFAULT_ARROW_INTERVAL_PX = 40.0
DEFAULT_LINE_WIDTH = 2
ARROW_HEAD_LEN_PX = 9.0
ARROW_HEAD_ANGLE_DEG = 25.0

MAIN_PATH_COLOR = (255, 0, 0)
BRANCH_PALETTE: tuple[tuple[int, int, int], ...] = (
    (0, 200, 255),
    (255, 210, 0),
    (255, 0, 255),
    (0, 160, 60),
    (255, 128, 0),
    (140, 0, 255),
    (0, 220, 160),
)
ORDER_LABEL_BG = (0, 0, 0)
ORDER_LABEL_FG = (255, 255, 255)

Pixel = tuple[int, int]


def _load_default_font() -> ImageFont.ImageFont:
    try:
        return ImageFont.load_default(size=14)
    except TypeError:
        return ImageFont.load_default()


def _points(entries: list[dict[str, int]]) -> list[Pixel]:
    return [(int(p["row"]), int(p["col"])) for p in entries]


def branch_color(index: int) -> tuple[int, int, int]:
    """Deterministic, cycling colour for the `index`-th branch drawn (any component)."""
    return BRANCH_PALETTE[index % len(BRANCH_PALETTE)]


def arrow_segments(dense: list[Pixel], interval_px: float) -> list[tuple[Pixel, Pixel]]:
    """`(from, to)` pixel pairs, spaced roughly every `interval_px` of travelled distance.

    Each pair gives the local direction of travel at that point along `dense`, in the order the
    path is walked (start to end) — used to draw direction arrowheads. Returns `[]` for a path
    shorter than two points or a non-positive interval.
    """
    if len(dense) < 2 or interval_px <= 0:
        return []

    segments: list[tuple[Pixel, Pixel]] = []
    travelled = 0.0
    next_mark = interval_px
    for i in range(len(dense) - 1):
        p0, p1 = dense[i], dense[i + 1]
        step = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if travelled + step >= next_mark:
            segments.append((p0, p1))
            next_mark += interval_px
        travelled += step
    return segments


def _draw_arrowhead(draw: ImageDraw.ImageDraw, p_from: Pixel, p_to: Pixel, color: tuple[int, int, int]) -> None:
    """Draw a small filled triangle at `p_to`, pointing from `p_from` towards `p_to`."""
    (r0, c0), (r1, c1) = p_from, p_to
    angle = math.atan2(r1 - r0, c1 - c0)
    spread = math.radians(ARROW_HEAD_ANGLE_DEG)
    tip = (c1, r1)
    left = (
        c1 - ARROW_HEAD_LEN_PX * math.cos(angle - spread),
        r1 - ARROW_HEAD_LEN_PX * math.sin(angle - spread),
    )
    right = (
        c1 - ARROW_HEAD_LEN_PX * math.cos(angle + spread),
        r1 - ARROW_HEAD_LEN_PX * math.sin(angle + spread),
    )
    draw.polygon([tip, left, right], fill=color)


def _draw_polyline(draw: ImageDraw.ImageDraw, dense: list[Pixel], color: tuple[int, int, int], width: int) -> None:
    if len(dense) < 2:
        if dense:
            r, c = dense[0]
            draw.ellipse((c - width, r - width, c + width, r + width), fill=color)
        return
    xy = [(c, r) for r, c in dense]
    draw.line(xy, fill=color, width=width, joint="curve")


def _draw_order_label(draw: ImageDraw.ImageDraw, at: Pixel, text: str, font: ImageFont.ImageFont) -> None:
    r, c = at
    left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
    text_w, text_h = right - left, bottom - top
    pad = 3
    box = (c - pad, r - pad, c + text_w + pad, r + text_h + pad)
    draw.rectangle(box, fill=ORDER_LABEL_BG)
    draw.text((c - left, r - top), text, fill=ORDER_LABEL_FG, font=font)


def render_path_overlay(
    original_rgb: np.ndarray,
    doc: dict[str, Any],
    *,
    arrow_interval_px: float = DEFAULT_ARROW_INTERVAL_PX,
    line_width: int = DEFAULT_LINE_WIDTH,
    draw_labels: bool = True,
) -> np.ndarray:
    """Draw ordered crack paths over `original_rgb`. Never mutates its input.

    Main paths are drawn in `MAIN_PATH_COLOR`, branches cycle through `BRANCH_PALETTE`, direction
    arrowheads mark travel order along every polyline, and (when `draw_labels`) each component's
    main path start is tagged with its 1-based visiting order.
    """
    image = Image.fromarray(np.asarray(original_rgb).copy(), mode="RGB")
    draw = ImageDraw.Draw(image)
    font = _load_default_font()

    branch_index = 0
    for comp in doc.get("components", []):
        main_dense = _points(comp["main_path"]["dense"])
        _draw_polyline(draw, main_dense, MAIN_PATH_COLOR, line_width)
        for p_from, p_to in arrow_segments(main_dense, arrow_interval_px):
            _draw_arrowhead(draw, p_from, p_to, MAIN_PATH_COLOR)

        if draw_labels and main_dense:
            _draw_order_label(draw, main_dense[0], str(comp["order_index"] + 1), font)

        for branch in comp.get("branches", []):
            color = branch_color(branch_index)
            branch_index += 1
            branch_dense = _points(branch["dense"])
            _draw_polyline(draw, branch_dense, color, max(1, line_width - 1))
            for p_from, p_to in arrow_segments(branch_dense, arrow_interval_px):
                _draw_arrowhead(draw, p_from, p_to, color)

    return np.array(image)


# ---------------------------------------------------------------------------
# Per-case rendering
# ---------------------------------------------------------------------------


def visualize_paths_case(
    cfg: Config,
    case_entry: dict[str, Any],
    *,
    arrow_interval_px: float,
    line_width: int,
    paths_dir: Path,
    skip_existing: bool = False,
    logger: logging.Logger | None = None,
) -> dict[str, Any]:
    """Render one case's path overlay. Returns a result dict with at least `status`."""
    log = logger or logging.getLogger("crackvision.visualize_paths")
    case_id = case_entry["case_id"]

    source_path = Path(case_entry["source_path"])
    if not source_path.is_absolute():
        source_path = cfg.root / source_path

    doc_path = paths_dir / f"{case_id}_paths.json"
    overlay_out = paths_dir / f"{case_id}_paths_overlay.png"

    if skip_existing and overlay_out.is_file():
        log.info("skip-existing: case %s already has a paths overlay", case_id)
        return {"case_id": case_id, "status": "skipped", "overlay_path": overlay_out}

    if not doc_path.is_file():
        msg = f"paths missing: {doc_path} (run: ./env.sh python -m crackvision.paths first)"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    if not source_path.is_file():
        msg = f"original missing: {source_path}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    try:
        doc = json.loads(doc_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        msg = f"malformed {doc_path}: {exc}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    try:
        original_img = load_original_rgb(source_path)
    except Exception as exc:  # noqa: BLE001 — one bad case must never abort the batch
        msg = f"failed to read original {source_path}: {type(exc).__name__}: {exc}"
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    original_arr = np.array(original_img)
    if (doc["image_height"], doc["image_width"]) != original_arr.shape[:2]:
        msg = (
            f"shape mismatch: paths {(doc['image_height'], doc['image_width'])} "
            f"!= original {original_arr.shape[:2]}"
        )
        log.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    overlay_arr = render_path_overlay(
        original_arr, doc, arrow_interval_px=arrow_interval_px, line_width=line_width
    )
    overlay_out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(overlay_arr, mode="RGB").save(overlay_out, format="PNG")

    log.info("case %s: %d component(s) drawn", case_id, doc.get("component_count", 0))
    return {"case_id": case_id, "status": "ok", "overlay_path": overlay_out}


# ---------------------------------------------------------------------------
# CLI (docs/INTERFACES.md §3, common flags per §0)
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="visualize_paths.py",
        description="Overlay ordered crack paths (order, direction, branches) over the original image.",
    )
    add_common_args(parser)
    parser.add_argument(
        "--cases", nargs="+", default=None, metavar="CASE_ID", help="restrict to named case_id(s) (default: all)"
    )
    parser.add_argument(
        "--arrow-interval-px",
        type=float,
        default=DEFAULT_ARROW_INTERVAL_PX,
        metavar="PX",
        help=f"spacing between direction arrowheads along a path (default: {DEFAULT_ARROW_INTERVAL_PX})",
    )
    parser.add_argument(
        "--line-width",
        type=int,
        default=DEFAULT_LINE_WIDTH,
        metavar="N",
        help=f"main-path line width in px (default: {DEFAULT_LINE_WIDTH})",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"visualize_paths: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("visualize_paths", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    case_map_path = cfg.root / "data" / "case_map.json"
    if not case_map_path.is_file():
        msg = "run: ./env.sh python -m crackvision.prepare_inputs first"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "visualize_paths", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    try:
        case_map = json.loads(case_map_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        msg = f"malformed {case_map_path}: {exc}"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "visualize_paths", "failed", EXIT_RUNTIME)
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

    paths_dir = cfg.root / "data" / "paths"

    if args.dry_run:
        for case_entry in cases:
            logger.info("dry-run: would render path overlay for case %s", case_entry["case_id"])
        logger.info("dry-run: no files written")
        return EXIT_OK

    for case_entry in cases:
        result = visualize_paths_case(
            cfg,
            case_entry,
            arrow_interval_px=args.arrow_interval_px,
            line_width=args.line_width,
            paths_dir=paths_dir,
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
        summary.write(cfg, "visualize_paths", "partial", EXIT_RUNTIME)
        return EXIT_RUNTIME

    summary.write(cfg, "visualize_paths", "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
