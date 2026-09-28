"""summarize_run.py — human-readable summary table for one run_test.sh pass. Built by TC-011.

Reads only artefacts other stages already wrote (`data/case_map.json`, each case's
`{case}_skeleton_stats.json`, and `logs/inference_latest.json`) and never fails the run: a missing
or partial stats file just shows `-` in that cell, and the process exits 0 regardless. This module
adds no new analysis — everything printed here already exists on disk.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_USAGE, RunSummary, setup_logging
from crackvision.naming import comparison_path, skeleton_stats_path

COLUMNS = ("CASE", "SIZE", "CRACK %", "SKEL PX", "COMPONENTS", "COMPARISON")


def load_case_map(root: Path) -> list[dict[str, Any]]:
    """Return the `cases` list from `data/case_map.json`, or `[]` if it does not exist / is malformed."""
    case_map_path = root / "data" / "case_map.json"
    if not case_map_path.is_file():
        return []
    try:
        data = json.loads(case_map_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return []
    cases = data.get("cases")
    return cases if isinstance(cases, list) else []


def load_skeleton_stats(root: Path, case_id: str) -> dict[str, Any] | None:
    stats_path = root / skeleton_stats_path(case_id)
    if not stats_path.is_file():
        return None
    try:
        return json.loads(stats_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None


def load_inference_summary(cfg: Config) -> dict[str, Any] | None:
    latest_path = cfg.paths["logs"] / "inference_latest.json"
    if not latest_path.is_file():
        return None
    try:
        return json.loads(latest_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None


def build_rows(root: Path, cases: list[dict[str, Any]]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for case in cases:
        case_id = case.get("case_id", "-")
        width = case.get("width")
        height = case.get("height")
        size = f"{width}x{height}" if width is not None and height is not None else "-"

        stats = load_skeleton_stats(root, case_id)
        if stats is None:
            crack_pct, skel_px, components = "-", "-", "-"
        else:
            crack_pct = f"{stats.get('mask_fraction', 0.0) * 100:.3f}"
            skel_px = str(stats.get("skeleton_pixels", "-"))
            components = str(stats.get("components_after", "-"))

        comparison = comparison_path(case_id).as_posix()

        rows.append(
            {
                "CASE": str(case_id),
                "SIZE": size,
                "CRACK %": crack_pct,
                "SKEL PX": skel_px,
                "COMPONENTS": components,
                "COMPARISON": comparison,
            }
        )
    return rows


def render_table(rows: list[dict[str, str]]) -> str:
    right_align = {"SIZE", "CRACK %", "SKEL PX", "COMPONENTS"}
    widths = {col: len(col) for col in COLUMNS}
    for row in rows:
        for col in COLUMNS:
            widths[col] = max(widths[col], len(row[col]))

    def fmt_cell(col: str, value: str) -> str:
        return value.rjust(widths[col]) if col in right_align else value.ljust(widths[col])

    lines = ["   ".join(fmt_cell(col, col) for col in COLUMNS).rstrip()]
    for row in rows:
        lines.append("   ".join(fmt_cell(col, row[col]) for col in COLUMNS).rstrip())
    return "\n".join(lines)


def render_run_line(n_cases: int, inference_summary: dict[str, Any] | None) -> str:
    if not inference_summary:
        return f"{n_cases} case{'s' if n_cases != 1 else ''} · no inference log found"

    duration = inference_summary.get("inference_duration_s")
    per_image = inference_summary.get("seconds_per_image")
    device = inference_summary.get("device")

    if duration is None or per_image is None or device is None:
        return f"{n_cases} case{'s' if n_cases != 1 else ''} · inference log incomplete"

    return (
        f"{n_cases} case{'s' if n_cases != 1 else ''} · "
        f"inference {duration:.1f} s ({per_image:.1f} s/image, {device})"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="summarize_run.py",
        description=(
            "Print a per-case summary table (size, crack %, skeleton pixels, components, comparison "
            "path) from the artefacts already written by prepare_inputs, inference and skeleton "
            "(docs/INTERFACES.md §3.11). Reads only; never fails the run it summarizes."
        ),
    )
    add_common_args(parser)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"summarize_run: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("summarize_run", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    cases = load_case_map(cfg.root)
    summary.increment("cases", len(cases))

    rows: list[dict[str, str]] = []
    if not cases:
        print("No cases found — data/case_map.json is missing or empty.")
        print("Run ./env.sh python -m crackvision.prepare_inputs first.")
    else:
        rows = build_rows(cfg.root, cases)
        inference_summary = load_inference_summary(cfg)

        print(render_table(rows))
        print()
        print(render_run_line(len(cases), inference_summary))

    comparisons_dir = cfg.paths["comparisons"]
    overlays_dir = cfg.paths["overlays"]
    print()
    print("Done. Open the results:")
    print(f"  {comparisons_dir}")
    print(f"  {overlays_dir}")

    if args.dry_run:
        logger.info("dry-run: logs/summarize_run_latest.json not written")
        return EXIT_OK

    latest_path = summary.write(cfg, "summarize_run", "ok", EXIT_OK)
    payload = json.loads(latest_path.read_text(encoding="utf-8"))
    payload["cases"] = rows
    payload["comparisons_dir"] = str(comparisons_dir)
    payload["overlays_dir"] = str(overlays_dir)
    latest_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
