#!/usr/bin/env python3
"""scripts/operator_doctor.py — aggregated operator-facing diagnostics. Built by OPS-01.3.

Runs the `doctor` stage of docs/INTERFACES.md §19.1: it subprocess-invokes
`scripts/check_env.py`, `scripts/check_host.py` and `scripts/check_realsense.py` (each with
`--json`), reads their exit code + JSON summary, and adds one local informational row for
whether the ROS overlay (`ros2_ws/install/setup.bash`) has been built. It never reimplements
any of those checks itself.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_FAIL = 3

STATUS_PASS = "PASS"
STATUS_WARN = "WARN"
STATUS_FAIL = "FAIL"

REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Row:
    name: str
    status: str
    value: str
    hint: str = ""

    def to_dict(self) -> dict[str, str]:
        payload = {"name": self.name, "status": self.status, "value": self.value}
        if self.hint:
            payload["hint"] = self.hint
        return payload


def _run_script(cmd: list[str], timeout: float) -> tuple[int | None, str, str]:
    """Run a diagnostic subprocess, returning (returncode, stdout, stderr). returncode is None on launch failure."""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError) as exc:
        return None, "", str(exc)
    return result.returncode, result.stdout, result.stderr


def _rows_from_checks_json(prefix: str, payload: dict) -> list[Row]:
    rows = []
    for check in payload.get("checks", []):
        name = check.get("name", "?")
        status = check.get("status", STATUS_WARN)
        value = check.get("value", "")
        hint = check.get("hint", "")
        rows.append(Row(f"{prefix}.{name}", status, value, hint))
    return rows


def _rows_from_host_json(payload: dict) -> list[Row]:
    rows = []
    status = payload.get("status")
    if status == "ok":
        rows.append(Row("host.status", STATUS_PASS, "ok"))
        return rows
    for diff in payload.get("diffs", []):
        if diff.get("matches", True) or diff.get("warn_only", False):
            continue
        field = diff.get("field", "?")
        rows.append(
            Row(
                f"host.{field}",
                STATUS_FAIL,
                f"expected={diff.get('expected')} actual={diff.get('actual')}",
                "run scripts/check_host.py for details",
            )
        )
    for check in payload.get("isolation", []):
        if check.get("ok", True):
            continue
        rows.append(Row(f"host.isolation.{check.get('name', '?')}", STATUS_FAIL, str(check), ""))
    if not rows:
        rows.append(Row("host.status", STATUS_FAIL, "drift", "run scripts/check_host.py for details"))
    return rows


def check_subprocess(label: str, cmd: list[str], parser_fn, timeout: float = 60.0) -> list[Row]:
    returncode, stdout, stderr = _run_script(cmd, timeout)
    if returncode is None:
        return [Row(label, STATUS_FAIL, "failed to launch", stderr.strip() or "subprocess launch error")]

    try:
        payload = json.loads(stdout.strip()) if stdout.strip() else {}
    except json.JSONDecodeError:
        return [
            Row(
                label,
                STATUS_WARN,
                f"non-JSON output (exit {returncode})",
                f"run `{' '.join(cmd)}` directly to see details",
            )
        ]

    rows = parser_fn(payload)
    if not rows:
        rows = [Row(label, STATUS_WARN, f"no checks reported (exit {returncode})", "")]
    return rows


def check_ros_overlay(root: Path) -> Row:
    setup_bash = root / "ros2_ws" / "install" / "setup.bash"
    if setup_bash.exists():
        return Row("ros2_ws.overlay", STATUS_PASS, str(setup_bash))
    return Row(
        "ros2_ws.overlay",
        STATUS_WARN,
        f"missing: {setup_bash}",
        "build it with scripts/ros/build_ws.sh",
    )


def collect_rows(root: Path, python: str) -> list[Row]:
    rows: list[Row] = []
    rows.extend(
        check_subprocess(
            "env", [python, str(root / "scripts" / "check_env.py"), "--json", "--dry-run"], lambda p: _rows_from_checks_json("env", p)
        )
    )
    rows.extend(
        check_subprocess(
            "host", [python, str(root / "scripts" / "check_host.py"), "--json"], _rows_from_host_json
        )
    )
    rows.extend(
        check_subprocess(
            "realsense",
            [python, str(root / "scripts" / "check_realsense.py"), "--json", "--dry-run"],
            lambda p: _rows_from_checks_json("realsense", p),
        )
    )
    rows.append(check_ros_overlay(root))
    return rows


def _print_table(rows: list[Row]) -> None:
    width = max((len(r.name) for r in rows), default=4)
    for row in rows:
        line = f"{row.name:<{width}}  {row.status:<4}  {row.value}"
        if row.hint:
            line += f"  ({row.hint})"
        print(line)


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _write_summary(root: Path, rows: list[Row], status: str, exit_code: int) -> None:
    log_dir = root / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "tool": "operator_doctor",
        "schema_version": 1,
        "finished_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "status": status,
        "exit_code": exit_code,
        "checks": [row.to_dict() for row in rows],
    }
    text = json.dumps(payload, indent=2) + "\n"
    (log_dir / f"operator_doctor_{_utc_stamp()}.json").write_text(text, encoding="utf-8")
    (log_dir / "operator_doctor_latest.json").write_text(text, encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="operator_doctor.py",
        description="Aggregated operator `doctor` stage: env + host + camera + ROS overlay (docs/INTERFACES.md §19.1).",
    )
    parser.add_argument("--json", action="store_true", help="print the JSON summary instead of the table")
    parser.add_argument("--root", type=Path, default=REPO_ROOT, help="project root")
    parser.add_argument("--dry-run", action="store_true", help="do not write logs/operator_doctor_*.json")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        return exc.code if isinstance(exc.code, int) else EXIT_USAGE

    rows = collect_rows(args.root, sys.executable)

    has_fail = any(row.status == STATUS_FAIL for row in rows)
    exit_code = EXIT_FAIL if has_fail else EXIT_OK
    status = "failed" if has_fail else "ok"

    if args.json:
        print(json.dumps({"checks": [row.to_dict() for row in rows], "status": status}, indent=2))
    else:
        _print_table(rows)

    if not args.dry_run:
        _write_summary(args.root, rows, status, exit_code)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
