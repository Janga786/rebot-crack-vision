"""crackvision.operator_state — per-case `crackvision.operator_run/1` state file (OPS-01.2).

Implements docs/INTERFACES.md §19.2/§19.3: the run-state file every later OPS-01 stage script
reads/writes, and the mechanical cascade-invalidation freshness rule. `doctor` is global and never
appears in this file (§19.1); it is handled elsewhere.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE, RunSummary, setup_logging

SCHEMA = "crackvision.operator_run/1"

# Fixed §19.1 case-scoped stage order and §19.2 depends_on graph. `doctor` is global, not here.
STAGES: tuple[str, ...] = ("capture", "segment", "paths", "plan", "preview", "execute")

STAGE_DEPENDS_ON: dict[str, tuple[str, ...]] = {
    "capture": (),
    "segment": ("capture",),
    "paths": ("segment",),
    "plan": ("paths",),
    "preview": ("plan",),
    "execute": ("preview",),
}


class OperatorStateError(Exception):
    """A run-state file is malformed, or an unknown stage name was requested."""


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def run_state_path(root: Path, case_id: str) -> Path:
    return Path(root) / "data" / "operator_runs" / f"{case_id}_run_state.json"


def _fresh_stage_entry(stage: str) -> dict[str, Any]:
    return {
        "status": "pending",
        "updated_utc": None,
        "outputs": [],
        "depends_on": list(STAGE_DEPENDS_ON[stage]),
        "notes": "",
    }


def _fresh_state(case_id: str) -> dict[str, Any]:
    now = _utc_iso()
    return {
        "schema": SCHEMA,
        "case_id": case_id,
        "created_utc": now,
        "updated_utc": now,
        "stages": {stage: _fresh_stage_entry(stage) for stage in STAGES},
    }


def load(case_id: str, root: Path) -> dict[str, Any]:
    """Load the run-state for `case_id`, or a fresh all-pending state if no file exists yet."""
    path = run_state_path(root, case_id)
    if not path.is_file():
        return _fresh_state(case_id)
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise OperatorStateError(f"could not read run-state {path}: {exc}") from exc
    if doc.get("schema") != SCHEMA:
        raise OperatorStateError(f"{path}: unexpected schema {doc.get('schema')!r}, expected {SCHEMA!r}")
    return doc


def save(state: dict[str, Any], root: Path) -> None:
    path = run_state_path(root, state["case_id"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def mark_stage(
    case_id: str,
    stage: str,
    status: str,
    outputs: list[str | Path],
    root: Path,
    *,
    notes: str | None = None,
) -> dict[str, Any]:
    """Record `stage`'s result: sha256 of each declared output path, plus an updated_utc stamp."""
    if stage not in STAGES:
        raise OperatorStateError(f"unknown stage {stage!r}, must be one of {STAGES}")

    root = Path(root)
    state = load(case_id, root)
    now = _utc_iso()

    output_entries = []
    for out in outputs:
        out_path = Path(out)
        abs_path = out_path if out_path.is_absolute() else root / out_path
        output_entries.append({"path": str(out_path), "sha256": _sha256_file(abs_path)})

    state["stages"][stage] = {
        "status": status,
        "updated_utc": now,
        "outputs": output_entries,
        "depends_on": list(STAGE_DEPENDS_ON[stage]),
        "notes": notes or "",
    }
    state["updated_utc"] = now
    save(state, root)
    return state


def _stage_is_fresh(state: dict[str, Any], stage: str, root: Path, cache: dict[str, bool]) -> bool:
    if stage in cache:
        return cache[stage]

    entry = state["stages"].get(stage)
    fresh = bool(entry) and entry.get("status") == "done"

    if fresh:
        for out in entry.get("outputs", []):
            out_path = Path(out["path"])
            abs_path = out_path if out_path.is_absolute() else root / out_path
            if not abs_path.is_file() or _sha256_file(abs_path) != out["sha256"]:
                fresh = False
                break

    if fresh:
        for dep in entry.get("depends_on", STAGE_DEPENDS_ON[stage]):
            if not _stage_is_fresh(state, dep, root, cache):
                fresh = False
                break

    cache[stage] = fresh
    return fresh


def stage_is_fresh(case_id: str, stage: str, root: Path) -> bool:
    """§19.3's cascade freshness rule: a `done` stage with intact outputs and fresh upstreams."""
    if stage not in STAGES:
        raise OperatorStateError(f"unknown stage {stage!r}, must be one of {STAGES}")
    root = Path(root)
    state = load(case_id, root)
    return _stage_is_fresh(state, stage, root, {})


def reset_stage_and_downstream(case_id: str, stage: str, root: Path) -> dict[str, Any]:
    """Reset `stage` and every stage after it (§19.1 order) to pending; return the new state."""
    if stage not in STAGES:
        raise OperatorStateError(f"unknown stage {stage!r}, must be one of {STAGES}")

    root = Path(root)
    state = load(case_id, root)
    start = STAGES.index(stage)
    for name in STAGES[start:]:
        state["stages"][name] = _fresh_stage_entry(name)
    state["updated_utc"] = _utc_iso()
    save(state, root)
    return state


def list_cases(root: Path) -> list[str]:
    run_dir = Path(root) / "data" / "operator_runs"
    if not run_dir.is_dir():
        return []
    cases = []
    for path in sorted(run_dir.glob("*_run_state.json")):
        cases.append(path.name[: -len("_run_state.json")])
    return cases


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="operator_state.py",
        description="Inspect/reset per-case operator run-state files (docs/INTERFACES.md §19).",
    )
    add_common_args(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status", help="print a per-stage table for a case, or list all cases")
    status.add_argument("--case", default=None, metavar="CASE_ID")

    reset = sub.add_parser("reset", help="reset a stage and every stage downstream of it to pending")
    reset.add_argument("--case", required=True, metavar="CASE_ID")
    reset.add_argument("--stage", required=True, choices=STAGES, metavar="NAME")

    return parser


def _print_status_table(case_id: str, state: dict[str, Any], root: Path, logger) -> None:
    print(f"case {case_id}:")
    for stage in STAGES:
        entry = state["stages"][stage]
        fresh = _stage_is_fresh(state, stage, root, {})
        print(
            f"  {stage:<8} status={entry['status']:<7} updated_utc={entry['updated_utc']!s:<22} "
            f"fresh={fresh}"
        )


def _cmd_status(args: argparse.Namespace, cfg: Config, logger) -> int:
    summary = RunSummary()

    if args.case is None:
        cases = list_cases(cfg.root)
        if not cases:
            logger.info("no cases found under %s", Path(cfg.root) / "data" / "operator_runs")
        for case_id in cases:
            print(case_id)
        summary.increment("cases", by=len(cases))
        summary.write(cfg, "operator_state", "ok", EXIT_OK)
        return EXIT_OK

    try:
        state = load(args.case, cfg.root)
    except OperatorStateError as exc:
        logger.error("%s", exc)
        summary.add_error(str(exc))
        summary.write(cfg, "operator_state", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    _print_status_table(args.case, state, cfg.root, logger)
    summary.write(cfg, "operator_state", "ok", EXIT_OK)
    return EXIT_OK


def _cmd_reset(args: argparse.Namespace, cfg: Config, logger) -> int:
    summary = RunSummary()

    if args.dry_run:
        logger.info("dry-run: would reset case %s stage %s and downstream", args.case, args.stage)
        summary.write(cfg, "operator_state", "ok", EXIT_OK)
        return EXIT_OK

    try:
        reset_stage_and_downstream(args.case, args.stage, cfg.root)
    except OperatorStateError as exc:
        logger.error("%s", exc)
        summary.add_error(str(exc))
        summary.write(cfg, "operator_state", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    logger.info("reset case %s: stage %s and downstream stages set to pending", args.case, args.stage)
    summary.write(cfg, "operator_state", "ok", EXIT_OK)
    return EXIT_OK


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"operator_state: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("operator_state", cfg, verbose=args.verbose, quiet=args.quiet)

    if args.command == "status":
        return _cmd_status(args, cfg, logger)
    elif args.command == "reset":
        return _cmd_reset(args, cfg, logger)
    return EXIT_USAGE


if __name__ == "__main__":
    sys.exit(main())
