"""scripts/verify_lock.py — drift check for the crackvision conda environment. Built by HOST-02.

Compares the live `crackvision` conda env against the committed lock files
(requirements/crackvision.conda-explicit.txt, requirements/crackvision.pip-freeze.txt) and
reports whether they match. Diagnoses only — it never installs, updates, or repairs anything.

Usage:
    ./env.sh python scripts/verify_lock.py [--root PATH] [-v|-q]

Exit codes (docs/INTERFACES.md §0.2):
    0 — live env matches the lock files
    1 — drift detected (a diff is printed)
    2 — usage error (bad args)
    3 — precondition not met (conda missing, env missing, or lock files missing)
"""

from __future__ import annotations

import argparse
import difflib
import re
import shutil
import subprocess
import sys
from pathlib import Path

EXIT_OK = 0
EXIT_DRIFT = 1
EXIT_USAGE = 2
EXIT_PRECONDITION = 3

CONDA_ENV_NAME = "crackvision"
CONDA_LOCK_FILE = "requirements/crackvision.conda-explicit.txt"
PIP_LOCK_FILE = "requirements/crackvision.pip-freeze.txt"


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _strip_header(lines: list[str]) -> list[str]:
    """Drop leading '#'-comment lines; keep the rest (comparable content)."""
    idx = 0
    while idx < len(lines) and lines[idx].startswith("#"):
        idx += 1
    return lines[idx:]


_SELF_EDITABLE_RE = re.compile(
    r"^-e git\+https://github\.com/Janga786/rebot-crack-vision\.git@[0-9a-f]+#egg=crackvision$"
)


def _normalize_self_editable(lines: list[str]) -> list[str]:
    """Replace this repo's own editable-install commit pin with a placeholder.

    That line's commit hash always matches the current HEAD, which changes with
    every commit (including the one that updates the lock files themselves), so
    comparing it verbatim would report permanent false-positive drift.
    """
    return [
        "-e git+https://github.com/Janga786/rebot-crack-vision.git@HEAD#egg=crackvision"
        if _SELF_EDITABLE_RE.match(line)
        else line
        for line in lines
    ]


def _find_conda() -> str | None:
    conda = shutil.which("conda")
    if conda:
        return conda
    candidate = Path.home() / "miniconda3" / "bin" / "conda"
    if candidate.exists():
        return str(candidate)
    candidate = Path.home() / "anaconda3" / "bin" / "conda"
    if candidate.exists():
        return str(candidate)
    return None


def _run(cmd: list[str]) -> tuple[int, str, str]:
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode, proc.stdout, proc.stderr


def _diff(name: str, expected: list[str], actual: list[str]) -> list[str]:
    return list(
        difflib.unified_diff(
            expected,
            actual,
            fromfile=f"{name} (locked)",
            tofile=f"{name} (live)",
            lineterm="",
        )
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify the crackvision env matches its lock files.")
    parser.add_argument("--root", type=Path, default=_project_root(), help="project root override")
    parser.add_argument("--config", type=Path, default=None, help="unused; accepted for CLI convention")
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument("--quiet", "-q", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="unused; accepted for CLI convention")
    parser.add_argument("--skip-existing", action="store_true", help="unused; accepted for CLI convention")
    args = parser.parse_args(argv)

    root: Path = args.root
    conda_lock_path = root / CONDA_LOCK_FILE
    pip_lock_path = root / PIP_LOCK_FILE

    if not conda_lock_path.exists() or not pip_lock_path.exists():
        print(f"verify_lock: missing lock file(s) under {root}/requirements/", file=sys.stderr)
        return EXIT_PRECONDITION

    conda_bin = _find_conda()
    if conda_bin is None:
        print("verify_lock: conda executable not found on PATH or in ~/miniconda3, ~/anaconda3", file=sys.stderr)
        return EXIT_PRECONDITION

    rc, out, err = _run([conda_bin, "list", "-n", CONDA_ENV_NAME, "--explicit"])
    if rc != 0:
        print(f"verify_lock: conda env '{CONDA_ENV_NAME}' not found or query failed:\n{err}", file=sys.stderr)
        return EXIT_PRECONDITION
    live_conda_lines = out.splitlines()

    rc, out, err = _run([sys.executable, "-m", "pip", "freeze"])
    if rc != 0:
        print(f"verify_lock: 'pip freeze' failed:\n{err}", file=sys.stderr)
        return EXIT_PRECONDITION
    live_pip_lines = out.splitlines()

    locked_conda_lines = _strip_header(conda_lock_path.read_text().splitlines())
    locked_pip_lines = _normalize_self_editable(_strip_header(pip_lock_path.read_text().splitlines()))
    live_conda_lines_body = _strip_header(live_conda_lines)
    live_pip_lines_body = _normalize_self_editable(_strip_header(live_pip_lines))

    diffs: list[str] = []
    diffs += _diff(CONDA_LOCK_FILE, locked_conda_lines, live_conda_lines_body)
    diffs += _diff(PIP_LOCK_FILE, locked_pip_lines, live_pip_lines_body)

    if diffs:
        for line in diffs:
            print(line)
        if not args.quiet:
            print(
                "\nverify_lock: DRIFT DETECTED — live env differs from committed lock files.",
                file=sys.stderr,
            )
        return EXIT_DRIFT

    if args.verbose:
        print(f"verify_lock: '{CONDA_ENV_NAME}' matches {CONDA_LOCK_FILE} and {PIP_LOCK_FILE}.")
    if not args.quiet:
        print("verify_lock: OK — live env matches lock files.")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
