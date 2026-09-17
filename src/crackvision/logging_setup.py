"""crackvision.logging_setup — uniform logging, exit codes, and run-summary JSON. Built in TC-001."""

from __future__ import annotations

import json
import logging
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

EXIT_OK = 0
EXIT_RUNTIME = 1
EXIT_USAGE = 2
EXIT_PRECONDITION = 3

_LOG_FORMAT = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"


class _UTCFormatter(logging.Formatter):
    converter = time.gmtime


def utc_stamp() -> str:
    """Return the current UTC time as YYYYmmddTHHMMSSZ, for log/summary filenames."""
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def setup_logging(tool: str, cfg: Any, verbose: bool = False, quiet: bool = False) -> logging.Logger:
    """Configure a logger that writes to stderr and to logs/<tool>_<UTC>.log (docs/INTERFACES.md §0.4)."""
    level = logging.DEBUG if verbose else (logging.WARNING if quiet else logging.INFO)

    logger = logging.getLogger(tool)
    logger.setLevel(level)
    logger.handlers.clear()
    logger.propagate = False

    formatter = _UTCFormatter(_LOG_FORMAT)

    stream_handler = logging.StreamHandler(sys.stderr)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    log_dir = Path(cfg.paths["logs"])
    log_dir.mkdir(parents=True, exist_ok=True)
    file_handler = logging.FileHandler(log_dir / f"{tool}_{utc_stamp()}.log", encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    return logger


@dataclass
class RunSummary:
    """Accumulates counts/errors for a CLI run and writes the machine-summary JSON on completion."""

    schema_version: int = 1
    counts: dict[str, int] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)
    started_utc: str = field(default_factory=_utc_iso)
    _start_monotonic: float = field(default_factory=time.monotonic, repr=False)

    def increment(self, key: str, by: int = 1) -> None:
        self.counts[key] = self.counts.get(key, 0) + by

    def add_error(self, message: str) -> None:
        self.errors.append(message)

    def write(self, cfg: Any, tool: str, status: str, exit_code: int) -> Path:
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

        log_dir = Path(cfg.paths["logs"])
        log_dir.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(summary, indent=2) + "\n"

        timestamped_path = log_dir / f"{tool}_{utc_stamp()}.json"
        timestamped_path.write_text(payload, encoding="utf-8")
        latest_path = log_dir / f"{tool}_latest.json"
        latest_path.write_text(payload, encoding="utf-8")

        return latest_path
