"""Tests for crackvision.operator_state (OPS-01.2)."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION
from crackvision.operator_state import (
    SCHEMA,
    STAGES,
    OperatorStateError,
    load,
    main,
    mark_stage,
    reset_stage_and_downstream,
    run_state_path,
    save,
    stage_is_fresh,
)

CASE_ID = "Test_Case_01"


def _write_output(root: Path, rel_path: str, content: bytes = b"hello") -> Path:
    path = root / rel_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return path


def _cfg(root: Path) -> Config:
    return Config(
        root=root,
        paths={"logs": root / "logs"},
        model={},
        inference={},
        visualization={},
        skeleton={},
        realsense={},
        raw={},
    )


def test_fresh_state_creation(tmp_path):
    state = load(CASE_ID, tmp_path)
    assert state["schema"] == SCHEMA
    assert state["case_id"] == CASE_ID
    assert set(state["stages"]) == set(STAGES)
    for stage, entry in state["stages"].items():
        assert entry["status"] == "pending"
        assert entry["updated_utc"] is None
        assert entry["outputs"] == []
    assert not run_state_path(tmp_path, CASE_ID).is_file()


def test_save_load_round_trip(tmp_path):
    state = load(CASE_ID, tmp_path)
    save(state, tmp_path)
    path = run_state_path(tmp_path, CASE_ID)
    assert path.is_file()
    loaded = load(CASE_ID, tmp_path)
    assert loaded == state


def test_mark_stage_records_sha256_and_rejects_unknown_stage(tmp_path):
    out_path = _write_output(tmp_path, "data/captures/Test_Case_01_capture.json")
    state = mark_stage(CASE_ID, "capture", "done", [out_path.relative_to(tmp_path)], tmp_path)
    entry = state["stages"]["capture"]
    assert entry["status"] == "done"
    assert entry["updated_utc"] is not None
    assert len(entry["outputs"]) == 1
    assert entry["outputs"][0]["path"] == "data/captures/Test_Case_01_capture.json"
    assert len(entry["outputs"][0]["sha256"]) == 64

    with pytest.raises(OperatorStateError):
        mark_stage(CASE_ID, "not_a_stage", "done", [], tmp_path)


def test_stage_is_fresh_basic(tmp_path):
    out_path = _write_output(tmp_path, "data/captures/Test_Case_01_capture.json")
    mark_stage(CASE_ID, "capture", "done", [out_path.relative_to(tmp_path)], tmp_path)
    assert stage_is_fresh(CASE_ID, "capture", tmp_path) is True

    # pending stage is never fresh
    assert stage_is_fresh(CASE_ID, "segment", tmp_path) is False


def test_stale_upstream_invalidates_downstream_done_stage(tmp_path):
    capture_out = _write_output(tmp_path, "data/captures/Test_Case_01_capture.json")
    mark_stage(CASE_ID, "capture", "done", [capture_out.relative_to(tmp_path)], tmp_path)

    segment_out = _write_output(tmp_path, "data/predictions/Test_Case_01.png")
    mark_stage(CASE_ID, "segment", "done", [segment_out.relative_to(tmp_path)], tmp_path)

    assert stage_is_fresh(CASE_ID, "segment", tmp_path) is True

    # capture's output file changes without re-running mark_stage for capture:
    # capture itself is now stale, which must cascade to invalidate segment too,
    # even though segment's own output file was never touched.
    capture_out.write_bytes(b"tampered capture content")
    assert stage_is_fresh(CASE_ID, "capture", tmp_path) is False
    assert stage_is_fresh(CASE_ID, "segment", tmp_path) is False


def test_tampered_output_file_detected(tmp_path):
    out_path = _write_output(tmp_path, "data/captures/Test_Case_01_capture.json")
    mark_stage(CASE_ID, "capture", "done", [out_path.relative_to(tmp_path)], tmp_path)
    assert stage_is_fresh(CASE_ID, "capture", tmp_path) is True

    out_path.write_bytes(b"tampered")
    assert stage_is_fresh(CASE_ID, "capture", tmp_path) is False

    out_path.unlink()
    assert stage_is_fresh(CASE_ID, "capture", tmp_path) is False


def test_reset_cascades_downstream_only(tmp_path):
    capture_out = _write_output(tmp_path, "data/captures/Test_Case_01_capture.json")
    mark_stage(CASE_ID, "capture", "done", [capture_out.relative_to(tmp_path)], tmp_path)
    segment_out = _write_output(tmp_path, "data/predictions/Test_Case_01.png")
    mark_stage(CASE_ID, "segment", "done", [segment_out.relative_to(tmp_path)], tmp_path)
    paths_out = _write_output(tmp_path, "data/paths/Test_Case_01_paths.json")
    mark_stage(CASE_ID, "paths", "done", [paths_out.relative_to(tmp_path)], tmp_path)

    state = reset_stage_and_downstream(CASE_ID, "segment", tmp_path)

    assert state["stages"]["capture"]["status"] == "done"
    assert state["stages"]["segment"]["status"] == "pending"
    assert state["stages"]["segment"]["updated_utc"] is None
    assert state["stages"]["paths"]["status"] == "pending"
    assert state["stages"]["plan"]["status"] == "pending"


def test_reset_does_not_touch_unrelated_case(tmp_path):
    other_case = "Other_Case_02"
    out_path = _write_output(tmp_path, "data/captures/Other_Case_02_capture.json")
    mark_stage(other_case, "capture", "done", [out_path.relative_to(tmp_path)], tmp_path)

    cap_out = _write_output(tmp_path, "data/captures/Test_Case_01_capture.json")
    mark_stage(CASE_ID, "capture", "done", [cap_out.relative_to(tmp_path)], tmp_path)

    reset_stage_and_downstream(CASE_ID, "capture", tmp_path)

    other_state = load(other_case, tmp_path)
    assert other_state["stages"]["capture"]["status"] == "done"


def test_status_cli_lists_cases_and_case_table(tmp_path, capsys):
    out_path = _write_output(tmp_path, "data/captures/Test_Case_01_capture.json")
    mark_stage(CASE_ID, "capture", "done", [out_path.relative_to(tmp_path)], tmp_path)

    rc = main(["--root", str(tmp_path), "status"])
    assert rc == EXIT_OK
    out = capsys.readouterr().out
    assert CASE_ID in out

    rc = main(["--root", str(tmp_path), "status", "--case", CASE_ID])
    assert rc == EXIT_OK
    out = capsys.readouterr().out
    assert "capture" in out
    assert "fresh=True" in out


def test_reset_cli_cascades(tmp_path):
    cap_out = _write_output(tmp_path, "data/captures/Test_Case_01_capture.json")
    mark_stage(CASE_ID, "capture", "done", [cap_out.relative_to(tmp_path)], tmp_path)
    seg_out = _write_output(tmp_path, "data/predictions/Test_Case_01.png")
    mark_stage(CASE_ID, "segment", "done", [seg_out.relative_to(tmp_path)], tmp_path)

    rc = main(["--root", str(tmp_path), "reset", "--case", CASE_ID, "--stage", "capture"])
    assert rc == EXIT_OK

    state = load(CASE_ID, tmp_path)
    assert state["stages"]["capture"]["status"] == "pending"
    assert state["stages"]["segment"]["status"] == "pending"
