#!/usr/bin/env python3
"""operator_segment_paths — single-case `segment`/`paths` wrapper (OPS-01.4, docs/INTERFACES.md §19).

Sequences the already-accepted `prepare_inputs`/`inference` (stage `segment`) and
`skeleton`/`paths`/`visualize_paths` (stage `paths`) modules for exactly one `--case-id`/`--source`
pair, via their own `main(argv)` entry points — never by shelling out to `run_test.sh`'s whole-folder
scan, and never by re-implementing any of those modules' own logic. Wires the result into
`crackvision.operator_state` (§19.2/§19.3): a stage already `done` and fresh is skipped unless
`--force`; a delegated CLI's non-zero exit is recorded as `status="failed"`, never a silent pass.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from crackvision import naming, operator_state
from crackvision import inference, paths as paths_mod, prepare_inputs, skeleton, visualize_paths
from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE, setup_logging

STAGE_CHOICES = ("segment", "paths", "both")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="operator_segment_paths.py",
        description=(
            "Run the segment (prepare_inputs + inference) and/or paths (skeleton + paths + "
            "visualize_paths) stages for one case_id/source pair (docs/INTERFACES.md §19.1)."
        ),
    )
    add_common_args(parser)
    parser.add_argument("--case-id", required=True, help="case_id this source maps to")
    parser.add_argument("--source", type=Path, required=True, help="path to the one source image")
    parser.add_argument("--stage", choices=STAGE_CHOICES, required=True, help="which stage(s) to run")
    parser.add_argument(
        "--force", action="store_true", help="re-run the requested stage(s) even if recorded fresh"
    )
    return parser


def _common_delegate_argv(args: argparse.Namespace) -> list[str]:
    argv: list[str] = []
    if args.root is not None:
        argv += ["--root", str(args.root)]
    if args.config is not None:
        argv += ["--config", str(args.config)]
    if args.verbose:
        argv.append("-v")
    if args.quiet:
        argv.append("-q")
    if args.dry_run:
        argv.append("--dry-run")
    if args.skip_existing:
        argv.append("--skip-existing")
    return argv


def _segment_output_path(cfg: Config, case_id: str) -> Path:
    return cfg.root / naming.prediction_path(case_id)


def _paths_output_paths(cfg: Config, case_id: str) -> list[Path]:
    return [
        cfg.root / naming.skeleton_path(case_id),
        cfg.root / naming.skeleton_overlay_path(case_id),
        cfg.root / naming.skeleton_stats_path(case_id),
        cfg.root / "data" / "paths" / f"{case_id}_paths.json",
        cfg.root / "data" / "paths" / f"{case_id}_paths_overlay.png",
    ]


def run_segment(cfg: Config, args: argparse.Namespace, logger) -> int:
    case_id = args.case_id
    root = cfg.root

    if not args.force and operator_state.stage_is_fresh(case_id, "segment", root):
        logger.info("segment: skip — run-state for case %s is already fresh (use --force to re-run)", case_id)
        return EXIT_OK

    common = _common_delegate_argv(args)
    source = Path(args.source)

    logger.info("segment: prepare_inputs for case %s (source=%s)", case_id, source)
    rc = prepare_inputs.main([*common, "--input-dir", str(source.parent)])
    if rc != 0:
        logger.error("segment: prepare_inputs failed (exit %d)", rc)
        operator_state.mark_stage(case_id, "segment", "failed", [], root, notes=f"prepare_inputs exit {rc}")
        return rc

    logger.info("segment: inference for case %s", case_id)
    rc = inference.main(common)
    if rc != 0:
        logger.error("segment: inference failed (exit %d)", rc)
        operator_state.mark_stage(case_id, "segment", "failed", [], root, notes=f"inference exit {rc}")
        return rc

    output = _segment_output_path(cfg, case_id)
    if args.dry_run:
        logger.info("segment: dry-run, not marking run-state")
        return EXIT_OK

    if not output.is_file():
        msg = f"segment: expected output missing for case {case_id}: {output}"
        logger.error(msg)
        operator_state.mark_stage(case_id, "segment", "failed", [], root, notes=msg)
        return EXIT_RUNTIME

    operator_state.mark_stage(case_id, "segment", "done", [output], root)
    logger.info("segment: done for case %s", case_id)
    return EXIT_OK


def run_paths(cfg: Config, args: argparse.Namespace, logger) -> int:
    case_id = args.case_id
    root = cfg.root

    segment_output = _segment_output_path(cfg, case_id)
    if not segment_output.is_file():
        msg = f"paths: precondition not met — no mask for case {case_id} (run --stage segment first): {segment_output}"
        logger.error(msg)
        return EXIT_PRECONDITION

    if not args.force and operator_state.stage_is_fresh(case_id, "paths", root):
        logger.info("paths: skip — run-state for case %s is already fresh (use --force to re-run)", case_id)
        return EXIT_OK

    common = _common_delegate_argv(args)
    cases_argv = ["--cases", case_id]

    logger.info("paths: skeleton for case %s", case_id)
    rc = skeleton.main([*common, *cases_argv])
    if rc != 0:
        logger.error("paths: skeleton failed (exit %d)", rc)
        operator_state.mark_stage(case_id, "paths", "failed", [], root, notes=f"skeleton exit {rc}")
        return rc

    logger.info("paths: paths for case %s", case_id)
    rc = paths_mod.main([*common, *cases_argv])
    if rc != 0:
        logger.error("paths: paths failed (exit %d)", rc)
        operator_state.mark_stage(case_id, "paths", "failed", [], root, notes=f"paths exit {rc}")
        return rc

    logger.info("paths: visualize_paths for case %s", case_id)
    rc = visualize_paths.main([*common, *cases_argv])
    if rc != 0:
        logger.error("paths: visualize_paths failed (exit %d)", rc)
        operator_state.mark_stage(case_id, "paths", "failed", [], root, notes=f"visualize_paths exit {rc}")
        return rc

    if args.dry_run:
        logger.info("paths: dry-run, not marking run-state")
        return EXIT_OK

    outputs = _paths_output_paths(cfg, case_id)
    missing = [p for p in outputs if not p.is_file()]
    if missing:
        msg = f"paths: expected output(s) missing for case {case_id}: {missing}"
        logger.error(msg)
        operator_state.mark_stage(case_id, "paths", "failed", [], root, notes=msg)
        return EXIT_RUNTIME

    operator_state.mark_stage(case_id, "paths", "done", outputs, root)
    logger.info("paths: done for case %s", case_id)
    return EXIT_OK


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"operator_segment_paths: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("operator_segment_paths", cfg, verbose=args.verbose, quiet=args.quiet)

    if args.stage in ("segment", "both"):
        rc = run_segment(cfg, args, logger)
        if rc != 0:
            return rc

    if args.stage in ("paths", "both"):
        rc = run_paths(cfg, args, logger)
        if rc != 0:
            return rc

    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
