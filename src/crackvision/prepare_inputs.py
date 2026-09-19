"""crackvision.prepare_inputs — arbitrary imagery -> nnU-Net-ready 3-channel RGB PNGs. Built by TC-007.

`NaturalImage2DIO` rejects JPEG outright and asserts a 3- or 4-channel array, while the plans
declare exactly 3 channels (docs/ARCHITECTURE.md §4.3). This module is the only place the
"force everything to 3-channel 8-bit RGB PNG" guarantee lives (docs/RISKS.md R-04): anything that
skips it fails deep inside nnU-Net with an opaque error. It also writes `data/case_map.json`, the
authoritative case_id <-> source-file join table every later stage reads (docs/INTERFACES.md §2).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from PIL import Image, ImageOps

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)
from crackvision.naming import assign_case_ids, nnunet_input_path

SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
CASE_MAP_SCHEMA_VERSION = 1


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _relposix(path: Path, root: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(root).as_posix()
    except ValueError:
        return resolved.as_posix()


def discover_sources(input_dir: Path) -> list[Path]:
    """Non-recursive scan of input_dir for supported extensions (case-insensitive), sorted by path."""
    if not input_dir.is_dir():
        return []
    found = [
        p for p in input_dir.iterdir() if p.is_file() and p.suffix.lower() in SUPPORTED_EXTENSIONS
    ]
    return sorted(found, key=str)


def convert_one(src: Path, dst: Path, max_side: int | None, write: bool) -> dict[str, Any]:
    """Open src, exif-transpose, force 3-channel RGB, optionally downscale, optionally write to dst.

    Returns metadata about the resulting (post-transpose, post-downscale) image. Raises whatever
    Pillow raises for a corrupt/unreadable/truncated file; the caller is responsible for catching
    that per-file so one bad image never aborts the batch.
    """
    with Image.open(src) as img:
        img.load()  # force full decode now, so a truncated file raises here, not later
        source_mode = img.mode
        source_format = img.format

        transposed = ImageOps.exif_transpose(img) or img  # some Pillow versions return None
        rgb = transposed.convert("RGB")  # mandatory, unconditional (docs/INTERFACES.md §0.5, §3.1)

        downscaled = False
        scale_factor = 1.0
        if max_side is not None and max(rgb.width, rgb.height) > max_side:
            scale_factor = max_side / max(rgb.width, rgb.height)
            new_size = (
                max(1, round(rgb.width * scale_factor)),
                max(1, round(rgb.height * scale_factor)),
            )
            rgb = rgb.resize(new_size, Image.LANCZOS)
            downscaled = True

        if write:
            dst.parent.mkdir(parents=True, exist_ok=True)
            rgb.save(dst, format="PNG", compress_level=6)

        return {
            "width": rgb.width,
            "height": rgb.height,
            "source_mode": source_mode,
            "source_format": source_format,
            "downscaled": downscaled,
            "scale_factor": round(scale_factor, 6),
        }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="prepare_inputs.py",
        description=(
            "Convert arbitrary imagery in data/input_originals/ into nnU-Net-ready 3-channel RGB "
            "PNGs, and write data/case_map.json (docs/INTERFACES.md §3.1)."
        ),
    )
    add_common_args(parser)
    parser.add_argument("--input-dir", type=Path, default=None, help="override data/input_originals")
    parser.add_argument("--output-dir", type=Path, default=None, help="override data/nnunet_input")
    parser.add_argument(
        "--max-side",
        type=int,
        default=None,
        help="downscale so max(h, w) <= N (Image.LANCZOS); off by default — breaks the frame invariant",
    )
    parser.add_argument(
        "--clean", action="store_true", help="remove existing data/nnunet_input/*.png before writing"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"prepare_inputs: {exc}", file=sys.stderr)
        return EXIT_USAGE

    if args.max_side is not None and args.max_side <= 0:
        print("prepare_inputs: --max-side must be a positive integer", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("prepare_inputs", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    input_dir = Path(args.input_dir) if args.input_dir is not None else cfg.paths["input_originals"]
    output_dir = Path(args.output_dir) if args.output_dir is not None else cfg.paths["nnunet_input"]

    sources = discover_sources(input_dir)
    summary.increment("found", len(sources))

    if not sources:
        extensions = ", ".join(sorted(SUPPORTED_EXTENSIONS))
        logger.error("no supported images found in %s (accepted extensions: %s)", input_dir, extensions)
        summary.add_error(f"no supported images found in {input_dir} (accepted extensions: {extensions})")
        summary.write(cfg, "prepare_inputs", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    case_ids = assign_case_ids(sources)

    if args.dry_run:
        logger.info("dry-run: found %d supported image(s) in %s", len(sources), input_dir)
        if args.clean:
            logger.info("dry-run: would remove existing PNGs in %s", output_dir)
        for src in sources:
            case_id = case_ids[src]
            dst = output_dir / nnunet_input_path(case_id).name
            logger.info("dry-run: would convert %s -> %s (case_id=%s)", src, dst, case_id)
        logger.info("dry-run: data/case_map.json not written")
        return EXIT_OK

    if args.clean and output_dir.is_dir():
        removed = 0
        for existing in output_dir.glob("*.png"):
            existing.unlink()
            removed += 1
        logger.info("--clean: removed %d existing PNG(s) from %s", removed, output_dir)

    cases: list[dict[str, Any]] = []
    for src in sources:
        case_id = case_ids[src]
        dst = output_dir / nnunet_input_path(case_id).name
        skip = bool(args.skip_existing and dst.is_file())

        try:
            meta = convert_one(src, dst, args.max_side, write=not skip)
        except Exception as exc:  # noqa: BLE001 — one bad file must never abort the batch
            logger.error("failed to convert %s (case_id=%s): %s: %s", src, case_id, type(exc).__name__, exc)
            summary.add_error(f"{src}: {type(exc).__name__}: {exc}")
            summary.increment("failed")
            continue

        if skip:
            summary.increment("skipped")
            logger.info("skip-existing: %s already present, not regenerated", dst)
        else:
            summary.increment("converted")
            logger.info(
                "converted %s -> %s (%s %s -> RGB, %dx%d)",
                src,
                dst,
                meta["source_mode"],
                meta["source_format"],
                meta["width"],
                meta["height"],
            )

        cases.append(
            {
                "case_id": case_id,
                "source_path": _relposix(src, cfg.root),
                "source_name": src.name,
                "nnunet_input": _relposix(dst, cfg.root),
                "width": meta["width"],
                "height": meta["height"],
                "source_mode": meta["source_mode"],
                "source_format": meta["source_format"],
                "converted": True,
                "downscaled": meta["downscaled"],
                "scale_factor": meta["scale_factor"],
                "sha256_source": _sha256(src),
                "sha256_nnunet_input": _sha256(dst),
            }
        )

    case_map = {
        "schema_version": CASE_MAP_SCHEMA_VERSION,
        "generated_utc": _utc_iso(),
        "root": str(cfg.root),
        "cases": sorted(cases, key=lambda c: c["case_id"]),
    }
    case_map_path = cfg.root / "data" / "case_map.json"
    case_map_path.parent.mkdir(parents=True, exist_ok=True)
    case_map_path.write_text(json.dumps(case_map, indent=2) + "\n", encoding="utf-8")
    logger.info("wrote %s (%d case(s))", case_map_path, len(cases))

    failed = summary.counts.get("failed", 0)
    if failed:
        summary.write(cfg, "prepare_inputs", "partial", EXIT_RUNTIME)
        return EXIT_RUNTIME

    summary.write(cfg, "prepare_inputs", "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
