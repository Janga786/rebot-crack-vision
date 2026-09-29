"""cli_common — §0.2 exit codes, §0.3 common flags and §0.4 log artefacts for the ROS-side CLIs.

Runs under the system python3 (ROS Humble, py3.10), never the conda `crackvision` env, so this
module deliberately mirrors (rather than imports) `src/crackvision/config.py` and
`src/crackvision/logging_setup.py` (ADR-007: importing them here would pull py3.11-only code and
a different interpreter's site-packages into this process).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
import traceback
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

from .reachability_core import ConfigError
from .reachability_map import MapError, PlacementError

EXIT_OK = 0
EXIT_RUNTIME = 1
EXIT_USAGE = 2
EXIT_PRECONDITION = 3

_LOG_FORMAT = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"


class PreconditionError(Exception):
    """Raised by a CLI's main_fn when a required precondition is not met. Maps to exit 3."""


class _UTCFormatter(logging.Formatter):
    converter = time.gmtime


def utc_stamp() -> str:
    """Current UTC time as YYYYmmddTHHMMSSZ, for log/summary filenames."""
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def add_common_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Add the flags every crackvision CLI shares (docs/INTERFACES.md §0.3)."""
    parser.add_argument("--root", type=Path, default=None, help="project root override")
    parser.add_argument(
        "--config", type=Path, default=None, help="config file override (default: config/project.yaml)"
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="DEBUG logging")
    parser.add_argument("-q", "--quiet", action="store_true", help="WARNING+ only")
    parser.add_argument("--dry-run", action="store_true", help="log what would be done, write nothing, exit 0")
    parser.add_argument(
        "--skip-existing", action="store_true", help="do not regenerate outputs that already exist"
    )
    return parser


def _looks_like_root(directory: Path) -> bool:
    return (directory / "pyproject.toml").is_file() and (directory / "docs" / "INTERFACES.md").is_file()


def find_root(start: Optional[Path] = None) -> Path:
    """Resolve the project root via $CRACKVISION_ROOT, else by walking up from `start`."""
    env_root = os.environ.get("CRACKVISION_ROOT")
    if env_root:
        candidate = Path(env_root).expanduser().resolve()
        if _looks_like_root(candidate):
            return candidate
        raise PreconditionError(
            f"CRACKVISION_ROOT={env_root!r} does not contain pyproject.toml and docs/INTERFACES.md"
        )

    here = (start or Path(__file__)).resolve()
    for directory in [here, *here.parents]:
        if _looks_like_root(directory):
            return directory
    raise PreconditionError(
        f"could not locate project root: no ancestor of {here} contains "
        "both pyproject.toml and docs/INTERFACES.md"
    )


def setup_logging(tool: str, log_dir: Path, verbose: bool = False, quiet: bool = False) -> logging.Logger:
    """Configure a logger that writes to stderr and to logs/<tool>_<UTC>.log (docs/INTERFACES.md §0.4)."""
    level = logging.DEBUG if verbose else (logging.WARNING if quiet else logging.INFO)

    logger = logging.getLogger(f"crackvision_motion.{tool}")
    logger.setLevel(level)
    logger.handlers.clear()
    logger.propagate = False

    formatter = _UTCFormatter(_LOG_FORMAT)

    stream_handler = logging.StreamHandler(sys.stderr)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    log_dir.mkdir(parents=True, exist_ok=True)
    file_handler = logging.FileHandler(log_dir / f"{tool}_{utc_stamp()}.log", encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    return logger


@dataclass
class RunSummary:
    """Accumulates counts/errors for a CLI run and writes the machine-summary JSON on completion."""

    schema_version: int = 1
    counts: Dict[str, int] = field(default_factory=dict)
    errors: List[str] = field(default_factory=list)
    started_utc: str = field(default_factory=_utc_iso)
    _start_monotonic: float = field(default_factory=time.monotonic, repr=False)

    def increment(self, key: str, by: int = 1) -> None:
        self.counts[key] = self.counts.get(key, 0) + by

    def add_error(self, message: str) -> None:
        self.errors.append(message)

    def write(self, log_dir: Path, tool: str, status: str, exit_code: int) -> Path:
        """Write logs/<tool>_<UTC>.json and logs/<tool>_latest.json; return the latest path."""
        finished_utc = _utc_iso()
        duration_s = round(time.monotonic() - self._start_monotonic, 3)
        summary = {
            "tool": tool,
            "schema_version": self.schema_version,
            "started_utc": self.started_utc,
            "finished_utc": finished_utc,
            "duration_s": duration_s,
            "status": status,
            "exit_code": exit_code,
            "counts": self.counts,
            "errors": self.errors,
        }

        log_dir.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(summary, indent=2) + "\n"

        timestamped_path = log_dir / f"{tool}_{utc_stamp()}.json"
        timestamped_path.write_text(payload, encoding="utf-8")
        latest_path = log_dir / f"{tool}_latest.json"
        latest_path.write_text(payload, encoding="utf-8")

        return latest_path


_STATUS_EXIT_CODES = {
    "ok": EXIT_OK,
    "partial": EXIT_RUNTIME,
    "failed": EXIT_RUNTIME,
    "precondition": EXIT_PRECONDITION,
}


def run_cli(
    tool: str,
    main_fn: Callable[[argparse.Namespace, logging.Logger, RunSummary], Optional[Dict[str, Any]]],
    argv: Sequence[str],
    extra_args: Optional[Callable[[argparse.ArgumentParser], None]] = None,
) -> int:
    """Parse argv, set up logging/summary, run `main_fn`, and map outcomes to §0.2 exit codes.

    `main_fn(args, log, summary)` does the tool's work and may return `{"status": "..."}`
    (default "ok" if it returns nothing). `extra_args`, if given, is called with the parser
    before parsing so a tool can add its own flags (docs/INTERFACES.md §0.3: "Tool-specific
    inputs use their own flags").
    """
    parser = argparse.ArgumentParser(prog=tool)
    add_common_args(parser)
    if extra_args is not None:
        extra_args(parser)
    args = parser.parse_args(list(argv))

    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    log_dir = root / "logs"
    logger = setup_logging(tool, log_dir, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    status = "ok"
    exit_code = EXIT_OK
    try:
        if args.dry_run:
            logger.info("--dry-run: skipping %s, writing nothing but logs", tool)
        else:
            result = main_fn(args, logger, summary)
            if isinstance(result, dict) and "status" in result:
                status = result["status"]
                if status not in _STATUS_EXIT_CODES:
                    raise ValueError(f"main_fn returned unknown status {status!r}")
                exit_code = _STATUS_EXIT_CODES[status]
    except (ConfigError, MapError, PlacementError) as exc:
        logger.error("%s", exc)
        summary.add_error(str(exc))
        status, exit_code = "failed", EXIT_USAGE
    except PreconditionError as exc:
        logger.error("%s", exc)
        summary.add_error(str(exc))
        status, exit_code = "precondition", EXIT_PRECONDITION
    except Exception as exc:  # noqa: BLE001 - logged with traceback, then mapped to exit 1
        logger.error("unhandled exception: %s", exc)
        logger.debug("%s", traceback.format_exc())
        summary.add_error(str(exc))
        status, exit_code = "failed", EXIT_RUNTIME

    summary.write(log_dir, tool, status, exit_code)
    return exit_code
