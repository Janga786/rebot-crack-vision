"""Tests for scripts/operator_segment_paths.py (OPS-01.4)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import operator_segment_paths as ops  # noqa: E402
from crackvision import naming, operator_state  # noqa: E402
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME  # noqa: E402

CASE_ID = "Test_Case_01"
IMG_SIZE = 100


def _make_source(tmp_path: Path) -> Path:
    src_dir = tmp_path / "sources"
    src_dir.mkdir()
    src = src_dir / f"{CASE_ID}.png"
    arr = np.full((IMG_SIZE, IMG_SIZE, 3), 200, dtype=np.uint8)
    Image.fromarray(arr, mode="RGB").save(src)
    return src


def _fake_inference_main_factory(root: Path):
    """Writes a synthetic prediction mask with a thick diagonal line, mimicking real inference."""

    def _fake_main(argv):
        output_dir = root / "data" / "predictions"
        output_dir.mkdir(parents=True, exist_ok=True)
        mask = np.zeros((IMG_SIZE, IMG_SIZE), dtype=np.uint8)
        for i in range(IMG_SIZE):
            for w in range(-2, 3):
                j = i + w
                if 0 <= j < IMG_SIZE:
                    mask[i, j] = 1
        Image.fromarray(mask, mode="L").save(output_dir / f"{CASE_ID}.png")
        return EXIT_OK

    return _fake_main


def _fake_inference_main_fail(argv):
    return EXIT_RUNTIME


def test_segment_happy_path(tmp_path, monkeypatch):
    src = _make_source(tmp_path)
    monkeypatch.setattr(ops.inference, "main", _fake_inference_main_factory(tmp_path))

    rc = ops.main(
        ["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "segment"]
    )
    assert rc == EXIT_OK

    state = operator_state.load(CASE_ID, tmp_path)
    assert state["stages"]["segment"]["status"] == "done"
    assert state["stages"]["segment"]["outputs"]
    assert (tmp_path / naming.prediction_path(CASE_ID)).is_file()


def test_segment_marks_failed_on_delegated_nonzero_exit(tmp_path, monkeypatch):
    src = _make_source(tmp_path)
    monkeypatch.setattr(ops.inference, "main", _fake_inference_main_fail)

    rc = ops.main(
        ["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "segment"]
    )
    assert rc == EXIT_RUNTIME

    state = operator_state.load(CASE_ID, tmp_path)
    assert state["stages"]["segment"]["status"] == "failed"
    assert state["stages"]["segment"]["outputs"] == []


def test_paths_before_segment_exits_precondition(tmp_path):
    src = _make_source(tmp_path)
    rc = ops.main(
        ["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "paths"]
    )
    assert rc == EXIT_PRECONDITION

    state = operator_state.load(CASE_ID, tmp_path)
    assert state["stages"]["paths"]["status"] == "pending"


def test_paths_happy_path_after_segment(tmp_path, monkeypatch):
    src = _make_source(tmp_path)
    monkeypatch.setattr(ops.inference, "main", _fake_inference_main_factory(tmp_path))

    rc = ops.main(
        ["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "segment"]
    )
    assert rc == EXIT_OK

    rc = ops.main(
        ["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "paths"]
    )
    assert rc == EXIT_OK

    state = operator_state.load(CASE_ID, tmp_path)
    assert state["stages"]["paths"]["status"] == "done"
    assert (tmp_path / naming.skeleton_path(CASE_ID)).is_file()
    assert (tmp_path / "data" / "paths" / f"{CASE_ID}_paths.json").is_file()


def test_paths_skips_when_fresh(tmp_path, monkeypatch):
    src = _make_source(tmp_path)
    monkeypatch.setattr(ops.inference, "main", _fake_inference_main_factory(tmp_path))

    # segment's own depends_on (§19.2) includes the upstream "capture" stage, which this card
    # does not implement — mark it done so the cascade freshness rule can reach segment/paths.
    operator_state.mark_stage(CASE_ID, "capture", "done", [], tmp_path)

    ops.main(["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "segment"])
    ops.main(["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "paths"])

    called = {"count": 0}
    real_skeleton_main = ops.skeleton.main

    def _counting_skeleton_main(argv):
        called["count"] += 1
        return real_skeleton_main(argv)

    monkeypatch.setattr(ops.skeleton, "main", _counting_skeleton_main)

    rc = ops.main(
        ["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "paths"]
    )
    assert rc == EXIT_OK
    assert called["count"] == 0


def test_paths_force_reruns_even_when_fresh(tmp_path, monkeypatch):
    src = _make_source(tmp_path)
    monkeypatch.setattr(ops.inference, "main", _fake_inference_main_factory(tmp_path))
    operator_state.mark_stage(CASE_ID, "capture", "done", [], tmp_path)

    ops.main(["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "segment"])
    ops.main(["--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src), "--stage", "paths"])

    called = {"count": 0}
    real_skeleton_main = ops.skeleton.main

    def _counting_skeleton_main(argv):
        called["count"] += 1
        return real_skeleton_main(argv)

    monkeypatch.setattr(ops.skeleton, "main", _counting_skeleton_main)

    rc = ops.main(
        [
            "--root", str(tmp_path), "--case-id", CASE_ID, "--source", str(src),
            "--stage", "paths", "--force",
        ]
    )
    assert rc == EXIT_OK
    assert called["count"] == 1
