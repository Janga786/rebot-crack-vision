"""crackvision.naming — the case_id contract and path mapping. Built by TC-007.

Pure functions, no I/O. This is the single source of truth for turning an arbitrary source image
path into a deterministic `case_id` and for mapping a `case_id` to every generated artifact's
default project-relative path (docs/INTERFACES.md §1).
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Iterable

_SANITIZE_RE = re.compile(r"[^A-Za-z0-9-]")
_COLLAPSE_UNDERSCORE_RE = re.compile(r"_+")


def derive_case_id(source: Path) -> str:
    """Derive the deterministic case_id for one source path (docs/INTERFACES.md §1.1)."""
    stem = Path(source).stem
    s = _SANITIZE_RE.sub("_", stem)
    s = _COLLAPSE_UNDERSCORE_RE.sub("_", s).strip("_")
    if s.endswith("_0000"):
        s = s[:-5].rstrip("_")
    if not s:
        s = "image"
    return s


def assign_case_ids(sources: Iterable[Path]) -> dict[Path, str]:
    """Assign a case_id to every source, resolving collisions deterministically.

    Sources are processed in sorted-full-path order; the first source to produce a given base
    case_id keeps it bare, later sources that produce the same base get `__2`, `__3`, ... This is
    re-runnable: the same set of source paths always yields the same assignment.
    """
    ordered = sorted(sources, key=str)
    occurrences: dict[str, int] = {}
    assigned: dict[Path, str] = {}
    for src in ordered:
        base = derive_case_id(src)
        occurrences[base] = occurrences.get(base, 0) + 1
        count = occurrences[base]
        assigned[src] = base if count == 1 else f"{base}__{count}"
    return assigned


def nnunet_input_path(case: str) -> Path:
    return Path("data/nnunet_input") / f"{case}_0000.png"


def prediction_path(case: str) -> Path:
    return Path("data/predictions") / f"{case}.png"


def mask_path(case: str) -> Path:
    return Path("data/overlays") / f"{case}_mask.png"


def overlay_path(case: str) -> Path:
    return Path("data/overlays") / f"{case}_overlay.png"


def comparison_path(case: str) -> Path:
    return Path("data/comparisons") / f"{case}_comparison.png"


def skeleton_path(case: str) -> Path:
    return Path("data/skeletons") / f"{case}_skeleton.png"


def skeleton_overlay_path(case: str) -> Path:
    return Path("data/skeletons") / f"{case}_skeleton_overlay.png"


def skeleton_stats_path(case: str) -> Path:
    return Path("data/skeletons") / f"{case}_skeleton_stats.json"
