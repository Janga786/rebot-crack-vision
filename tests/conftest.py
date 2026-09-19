"""tests/conftest.py — throwaway project root fixture. Built by TC-007, extended by TC-014.

Every test that needs a Config gets one rooted under pytest's own tmp_path, so nothing under this
repo's real data/ is ever read or written by the test suite.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from crackvision.config import Config, load_config

_REAL_PROJECT_YAML = Path(__file__).resolve().parents[1] / "config" / "project.yaml"


@pytest.fixture
def tmp_project(tmp_path: Path) -> Config:
    """Build a throwaway project root: config/project.yaml (a copy of the real one) + the data/ tree."""
    (tmp_path / "config").mkdir(parents=True, exist_ok=True)
    shutil.copy(_REAL_PROJECT_YAML, tmp_path / "config" / "project.yaml")

    cfg = load_config(root=tmp_path)
    for directory in cfg.paths.values():
        Path(directory).mkdir(parents=True, exist_ok=True)
    return cfg
