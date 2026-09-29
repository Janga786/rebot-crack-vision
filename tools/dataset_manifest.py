"""tools/dataset_manifest.py — D405 evaluation-manifest tooling. Built by TC-015.

Creates, populates, validates and summarises `data/d405/eval_manifest.csv`, the one-row-per-image
metadata file that makes the Level-3 evaluation interpretable (docs/D405_TEST_PLAN.md §3, the single
source of truth for the schema and controlled vocabularies below). This card delivers tooling only:
no image is captured or labelled here, and `init` writes nothing but a header.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE, RunSummary, setup_logging

TOOL_NAME = "dataset_manifest"

# Column order and enum vocabularies mirror docs/D405_TEST_PLAN.md §3 exactly. That document is the
# single source of truth — if the schema needs to change, edit it there, not here.
COLUMNS: list[str] = [
    "image_id",
    "timestamp_utc",
    "camera",
    "color_path",
    "depth_path",
    "metadata_path",
    "distance_cm",
    "camera_angle_deg",
    "surface_type",
    "lighting_condition",
    "crack_present",
    "crack_type",
    "negative_class",
    "block",
    "notes",
    "depth_valid_fraction",
    "exposure_us",
    "pred_crack_fraction",
    "pred_components",
    "rating_detection",
    "rating_continuity",
    "rating_false_positive",
]

REQUIRED_COLUMNS: list[str] = [
    "image_id",
    "timestamp_utc",
    "camera",
    "color_path",
    "depth_path",
    "metadata_path",
    "distance_cm",
    "camera_angle_deg",
    "surface_type",
    "lighting_condition",
    "crack_present",
    "crack_type",
    "block",
]

TOOL_COLUMNS: list[str] = [
    "image_id",
    "timestamp_utc",
    "camera",
    "color_path",
    "depth_path",
    "metadata_path",
    "depth_valid_fraction",
    "exposure_us",
]

ENUMS: dict[str, set[str]] = {
    "camera": {"D405", "other"},
    "surface_type": {
        "concrete_smooth",
        "concrete_aggregate",
        "concrete_formed",
        "asphalt",
        "masonry",
        "mortar",
        "other",
    },
    "lighting_condition": {"ambient", "ring", "oblique", "direct_harsh", "dim", "shadowed"},
    "crack_present": {"yes", "no", "ambiguous"},
    "crack_type": {"wide", "thin", "hairline", "branching", "none"},
    "negative_class": {"none", "joint", "seam", "stain", "aggregate", "scratch", "marker", "shadow"},
    "block": {"distance", "crack_type", "negative", "angle", "lighting"},
}

RATING_COLUMNS: list[str] = ["rating_detection", "rating_continuity", "rating_false_positive"]

# Target allocation per block, docs/D405_TEST_PLAN.md §2.
BLOCK_TARGETS: dict[str, int] = {
    "distance": 25,
    "crack_type": 15,
    "negative": 20,
    "angle": 6,
    "lighting": 6,
}


def _blank_row() -> dict[str, str]:
    return {column: "" for column in COLUMNS}


def _read_manifest(path: Path) -> list[dict[str, str]]:
    with open(path, "r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        rows = [dict(row) for row in reader]
    return rows


def _write_manifest_atomic(path: Path, rows: list[dict[str, str]]) -> None:
    """Write header + rows atomically: temp file in the same directory, then os.replace."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + f".tmp{os.getpid()}")
    with open(tmp_path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({column: row.get(column, "") for column in COLUMNS})
    os.replace(tmp_path, path)


def _default_manifest_path(cfg: Config) -> Path:
    return Path(cfg.paths["d405"]) / "eval_manifest.csv"


def _default_metadata_dir(cfg: Config) -> Path:
    return Path(cfg.paths["d405"]) / "metadata"


def _resolve(cfg: Config, path_arg: Path | None, default: Path) -> Path:
    if path_arg is None:
        return default
    path_arg = Path(path_arg)
    if path_arg.is_absolute():
        return path_arg
    return cfg.root / path_arg


def cmd_init(cfg: Config, args: argparse.Namespace) -> int:
    out_path = _resolve(cfg, args.out, _default_manifest_path(cfg))

    if out_path.exists() and not args.force:
        print(f"dataset_manifest: {out_path} already exists; pass --force to overwrite", file=sys.stderr)
        return EXIT_USAGE

    if args.dry_run:
        print(f"dry-run: would write header-only manifest to {out_path}")
        return EXIT_OK

    _write_manifest_atomic(out_path, [])
    print(f"wrote header-only manifest: {out_path}")
    return EXIT_OK


def _depth_valid_fraction(depth_path: Path) -> str:
    if not depth_path.is_file():
        return ""
    try:
        with Image.open(depth_path) as img:
            arr = np.array(img.convert("I;16"), dtype=np.uint16)
    except Exception:  # noqa: BLE001
        return ""
    if arr.size == 0:
        return ""
    return f"{float(np.count_nonzero(arr)) / arr.size:.6f}"


def _rel_to_root(cfg: Config, path: Path) -> str:
    try:
        return str(path.resolve().relative_to(cfg.root))
    except ValueError:
        return str(path)


def _scan_one(cfg: Config, metadata_path: Path) -> dict[str, str] | None:
    try:
        payload: dict[str, Any] = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if "files" not in payload:
        return None

    row = _blank_row()
    row["image_id"] = metadata_path.stem
    row["timestamp_utc"] = payload.get("timestamp_utc", "")
    row["camera"] = "D405"
    color_path = cfg.root / payload["files"]["color"]
    depth_path = cfg.root / payload["files"]["depth"]
    row["color_path"] = payload["files"]["color"]
    row["depth_path"] = payload["files"]["depth"]
    row["metadata_path"] = _rel_to_root(cfg, metadata_path)
    row["depth_valid_fraction"] = _depth_valid_fraction(depth_path)
    exposure_us = payload.get("exposure_us")
    row["exposure_us"] = "" if exposure_us is None else str(exposure_us)
    return row


def cmd_scan(cfg: Config, args: argparse.Namespace) -> int:
    metadata_dir = _resolve(cfg, args.metadata_dir, _default_metadata_dir(cfg))
    out_path = _resolve(cfg, args.out, _default_manifest_path(cfg))

    existing: dict[str, dict[str, str]] = {}
    if args.merge and out_path.exists():
        for row in _read_manifest(out_path):
            existing[row.get("image_id", "")] = row

    metadata_files = sorted(
        p for p in metadata_dir.glob("*.json") if not p.stem.endswith("_session")
    ) if metadata_dir.is_dir() else []

    new_rows: list[dict[str, str]] = []
    for metadata_path in metadata_files:
        scanned = _scan_one(cfg, metadata_path)
        if scanned is None:
            continue
        image_id = scanned["image_id"]
        if image_id in existing:
            merged_row = dict(existing[image_id])
            for column in TOOL_COLUMNS:
                merged_row[column] = scanned[column]
            new_rows.append(merged_row)
        else:
            new_rows.append(scanned)

    if args.dry_run:
        print(f"dry-run: would write {len(new_rows)} row(s) to {out_path}")
        return EXIT_OK

    _write_manifest_atomic(out_path, new_rows)
    print(f"scanned {len(new_rows)} row(s) into {out_path}")
    return EXIT_OK


def _validate_rows(cfg: Config, rows: list[dict[str, str]], header: list[str]) -> list[str]:
    problems: list[str] = []

    missing_columns = [c for c in COLUMNS if c not in header]
    if missing_columns:
        problems.append(f"header: missing required column(s) {missing_columns}")
        return problems

    seen_ids: dict[str, int] = {}
    for line_no, row in enumerate(rows, start=2):
        for column in REQUIRED_COLUMNS:
            if not row.get(column, "").strip():
                problems.append(f"row {line_no}: missing required value in column {column!r}")

        for column, allowed in ENUMS.items():
            value = row.get(column, "")
            if value and value not in allowed:
                problems.append(f"row {line_no}: {column}={value!r} is not one of {sorted(allowed)}")

        image_id = row.get("image_id", "")
        if image_id:
            if image_id in seen_ids:
                problems.append(f"row {line_no}: duplicate image_id {image_id!r} (first seen row {seen_ids[image_id]})")
            else:
                seen_ids[image_id] = line_no

        for path_column in ("color_path", "depth_path", "metadata_path"):
            value = row.get(path_column, "")
            if value and not (cfg.root / value).is_file():
                problems.append(f"row {line_no}: {path_column} does not exist: {value}")

        crack_present = row.get("crack_present", "")
        negative_class = row.get("negative_class", "")
        if crack_present == "no" and not negative_class.strip():
            problems.append(f"row {line_no}: negative_class is required when crack_present == 'no'")

        distance_cm = row.get("distance_cm", "")
        if distance_cm:
            try:
                distance_value = float(distance_cm)
            except ValueError:
                problems.append(f"row {line_no}: distance_cm is not numeric: {distance_cm!r}")
            else:
                if not (0 < distance_value <= 200):
                    problems.append(f"row {line_no}: distance_cm out of range (0, 200]: {distance_value}")

        camera_angle_deg = row.get("camera_angle_deg", "")
        if camera_angle_deg:
            try:
                angle_value = float(camera_angle_deg)
            except ValueError:
                problems.append(f"row {line_no}: camera_angle_deg is not numeric: {camera_angle_deg!r}")
            else:
                if not (0 <= angle_value <= 90):
                    problems.append(f"row {line_no}: camera_angle_deg out of range [0, 90]: {angle_value}")

        for rating_column in RATING_COLUMNS:
            value = row.get(rating_column, "")
            if value:
                try:
                    rating_value = int(value)
                except ValueError:
                    problems.append(f"row {line_no}: {rating_column} is not an integer: {value!r}")
                else:
                    if not (0 <= rating_value <= 3):
                        problems.append(f"row {line_no}: {rating_column} out of range 0-3: {rating_value}")

    return problems


def cmd_validate(cfg: Config, args: argparse.Namespace) -> int:
    in_path = _resolve(cfg, args.in_path, _default_manifest_path(cfg))
    if not in_path.is_file():
        print(f"dataset_manifest: {in_path} does not exist", file=sys.stderr)
        return EXIT_PRECONDITION

    with open(in_path, "r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        header = reader.fieldnames or []
        rows = [dict(row) for row in reader]

    problems = _validate_rows(cfg, rows, header)
    for problem in problems:
        print(problem)

    if problems:
        print(f"validate: {len(problems)} problem(s) in {in_path}")
        return EXIT_RUNTIME

    print(f"validate: {in_path} is clean ({len(rows)} row(s))")
    return EXIT_OK


def cmd_summary(cfg: Config, args: argparse.Namespace) -> int:
    in_path = _resolve(cfg, args.in_path, _default_manifest_path(cfg))
    if not in_path.is_file():
        print(f"dataset_manifest: {in_path} does not exist", file=sys.stderr)
        return EXIT_PRECONDITION

    rows = _read_manifest(in_path)
    total = len(rows)
    print(f"total rows: {total}")

    def _counts(column: str) -> dict[str, int]:
        counts: dict[str, int] = {}
        for row in rows:
            value = row.get(column, "") or "(blank)"
            counts[value] = counts.get(value, 0) + 1
        return counts

    for label, column in (
        ("distance_cm", "distance_cm"),
        ("surface_type", "surface_type"),
        ("lighting_condition", "lighting_condition"),
        ("crack_present", "crack_present"),
        ("negative_class", "negative_class"),
    ):
        print(f"\n{label}:")
        for value, count in sorted(_counts(column).items()):
            print(f"  {value}: {count}")

    print("\nblock allocation (docs/D405_TEST_PLAN.md §2):")
    block_counts = _counts("block")
    for block, target in BLOCK_TARGETS.items():
        have = block_counts.get(block, 0)
        shortfall = max(target - have, 0)
        if shortfall:
            print(f"  {block}: {have}/{target} (need {shortfall} more)")
        else:
            print(f"  {block}: {have}/{target} (met)")

    print("\nratings (mean of populated values):")
    for column in RATING_COLUMNS:
        values = [float(row[column]) for row in rows if row.get(column, "").strip()]
        if values:
            print(f"  {column}: {sum(values) / len(values):.2f} (n={len(values)})")
        else:
            print(f"  {column}: no data")

    return EXIT_OK


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="dataset_manifest.py",
        description="Create, populate, validate and summarise data/d405/eval_manifest.csv "
        "(docs/INTERFACES.md §3.12, docs/D405_TEST_PLAN.md §3).",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    init_parser = subparsers.add_parser("init", help="write a header-only manifest")
    init_parser.add_argument("--out", type=Path, default=None, help="manifest path (default: data/d405/eval_manifest.csv)")
    init_parser.add_argument("--force", action="store_true", help="overwrite an existing manifest")
    add_common_args(init_parser)

    scan_parser = subparsers.add_parser("scan", help="populate tool-derived columns from metadata JSONs")
    scan_parser.add_argument("--metadata-dir", type=Path, default=None, help="default: data/d405/metadata")
    scan_parser.add_argument("--out", type=Path, default=None, help="manifest path (default: data/d405/eval_manifest.csv)")
    scan_parser.add_argument("--merge", action="store_true", help="preserve human-entered values for existing image_ids")
    add_common_args(scan_parser)

    validate_parser = subparsers.add_parser("validate", help="validate the manifest against the schema")
    validate_parser.add_argument("--in", dest="in_path", type=Path, default=None, help="default: data/d405/eval_manifest.csv")
    add_common_args(validate_parser)

    summary_parser = subparsers.add_parser("summary", help="print coverage counts and block shortfalls")
    summary_parser.add_argument("--in", dest="in_path", type=Path, default=None, help="default: data/d405/eval_manifest.csv")
    add_common_args(summary_parser)

    return parser


COMMANDS = {
    "init": cmd_init,
    "scan": cmd_scan,
    "validate": cmd_validate,
    "summary": cmd_summary,
}


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"dataset_manifest: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging(TOOL_NAME, cfg, verbose=args.verbose, quiet=args.quiet)
    logger.info("running %s", args.command)

    exit_code = COMMANDS[args.command](cfg, args)

    summary = RunSummary()
    summary.increment(args.command)
    status = "ok" if exit_code == EXIT_OK else "failed"
    if not args.dry_run:
        summary.write(cfg, TOOL_NAME, status, exit_code)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
