"""tests/test_visualize_paths.py — path overlay rendering + CLI stage. Built by PERC-04."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME
from crackvision.paths import build_case_paths
from crackvision.visualize_paths import (
    MAIN_PATH_COLOR,
    arrow_segments,
    branch_color,
    main,
    render_path_overlay,
)


def _canvas(height: int, width: int) -> np.ndarray:
    return np.zeros((height, width), dtype=bool)


def _draw(canvas: np.ndarray, pixels: list[tuple[int, int]]) -> np.ndarray:
    for r, c in pixels:
        canvas[r, c] = True
    return canvas


def _blank_rgb(height: int, width: int) -> np.ndarray:
    return np.full((height, width, 3), 200, dtype=np.uint8)


# ---------------------------------------------------------------------------
# arrow_segments
# ---------------------------------------------------------------------------


def test_arrow_segments_empty_for_short_or_degenerate_input():
    assert arrow_segments([], 10.0) == []
    assert arrow_segments([(0, 0)], 10.0) == []
    assert arrow_segments([(0, 0), (0, 1)], 0.0) == []


def test_arrow_segments_spaced_along_a_straight_line():
    dense = [(0, c) for c in range(0, 100)]
    segments = arrow_segments(dense, 10.0)
    assert len(segments) >= 5
    for p_from, p_to in segments:
        assert p_to[1] > p_from[1]  # direction points forward, left to right


def test_branch_color_cycles_deterministically():
    assert branch_color(0) == branch_color(0)
    colors = [branch_color(i) for i in range(10)]
    assert colors[0] != MAIN_PATH_COLOR
    assert len(set(colors)) > 1  # not all identical


# ---------------------------------------------------------------------------
# render_path_overlay
# ---------------------------------------------------------------------------


def test_render_path_overlay_never_mutates_input():
    skel = _canvas(10, 10)
    _draw(skel, [(5, c) for c in range(2, 8)])
    doc = build_case_paths(skel, "line", min_spur_length_px=0, rdp_tolerance_px=1.0)

    original = _blank_rgb(10, 10)
    before = original.copy()
    render_path_overlay(original, doc)
    assert np.array_equal(original, before)


def test_render_path_overlay_draws_the_main_path_in_its_colour():
    skel = _canvas(10, 10)
    _draw(skel, [(5, c) for c in range(2, 8)])
    doc = build_case_paths(skel, "line", min_spur_length_px=0, rdp_tolerance_px=1.0)

    original = _blank_rgb(10, 10)
    overlay = render_path_overlay(original, doc, draw_labels=False)
    assert tuple(overlay[5, 5]) == MAIN_PATH_COLOR


def test_render_path_overlay_draws_branches_in_a_different_colour():
    # A stem with two branches so the doc has a "branches" list to render.
    skel = _canvas(20, 20)
    _draw(skel, [(r, 10) for r in range(2, 10)])
    _draw(skel, [(10, c) for c in range(10, 16)])
    _draw(skel, [(10 - i, 10 - i) for i in range(0, 6)])
    doc = build_case_paths(skel, "fork", min_spur_length_px=0, rdp_tolerance_px=1.0)
    comp = doc["components"][0]
    assert comp["branches"]  # sanity: fixture actually produced a branch

    original = _blank_rgb(20, 20)
    overlay = render_path_overlay(original, doc, draw_labels=False)

    branch_pixel = comp["branches"][0]["dense"][-1]
    color = tuple(overlay[branch_pixel["row"], branch_pixel["col"]])
    assert color != tuple(original[branch_pixel["row"], branch_pixel["col"]])


def test_render_path_overlay_empty_document_leaves_image_unchanged():
    doc = {"components": []}
    original = _blank_rgb(8, 8)
    overlay = render_path_overlay(original, doc)
    assert np.array_equal(overlay, original)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _make_cfg(tmp_path: Path) -> Config:
    cfg_paths = {
        key: tmp_path / f"data/{key}"
        for key in [
            "input_originals",
            "nnunet_input",
            "predictions",
            "overlays",
            "comparisons",
            "skeletons",
            "smoke_test",
            "d405",
        ]
    }
    cfg_paths["models"] = tmp_path / "models/opencrack-nnunet"
    cfg_paths["logs"] = tmp_path / "logs"
    cfg = Config(
        root=tmp_path,
        paths=cfg_paths,
        model={},
        inference={},
        visualization={},
        skeleton={},
        realsense={},
        raw={},
    )
    for directory in cfg.paths.values():
        Path(directory).mkdir(parents=True, exist_ok=True)
    (tmp_path / "config").mkdir(parents=True, exist_ok=True)
    (tmp_path / "config" / "project.yaml").write_text("schema_version: 1\n", encoding="utf-8")
    return cfg


def _write_case_map(cfg: Config, cases: list[dict[str, Any]]) -> Path:
    case_map_path = cfg.root / "data" / "case_map.json"
    case_map_path.parent.mkdir(parents=True, exist_ok=True)
    case_map_path.write_text(
        json.dumps({"schema_version": 1, "root": str(cfg.root), "cases": cases}), encoding="utf-8"
    )
    return case_map_path


def test_cli_writes_overlay_png(tmp_path: Path):
    cfg = _make_cfg(tmp_path)

    height, width = 10, 10
    original_path = cfg.paths["input_originals"] / "Case_01.png"
    Image.fromarray(_blank_rgb(height, width), mode="RGB").save(original_path)

    skel = _canvas(height, width)
    _draw(skel, [(5, c) for c in range(2, 8)])
    doc = build_case_paths(skel, "Case_01", min_spur_length_px=0, rdp_tolerance_px=1.0)
    paths_dir = tmp_path / "data" / "paths"
    paths_dir.mkdir(parents=True, exist_ok=True)
    (paths_dir / "Case_01_paths.json").write_text(json.dumps(doc), encoding="utf-8")

    _write_case_map(cfg, [{"case_id": "Case_01", "source_path": "data/input_originals/Case_01.png"}])

    exit_code = main(["--root", str(tmp_path)])
    assert exit_code == EXIT_OK

    out_path = paths_dir / "Case_01_paths_overlay.png"
    assert out_path.is_file()
    with Image.open(out_path) as img:
        assert img.size == (width, height)

    # Never touches the data artefact it reads from.
    assert json.loads((paths_dir / "Case_01_paths.json").read_text()) == doc


def test_cli_missing_case_map_is_precondition_failure(tmp_path: Path):
    (tmp_path / "config").mkdir(parents=True)
    (tmp_path / "config" / "project.yaml").write_text("schema_version: 1\n", encoding="utf-8")

    exit_code = main(["--root", str(tmp_path)])
    assert exit_code == EXIT_PRECONDITION


def test_cli_missing_paths_json_is_a_failure(tmp_path: Path):
    cfg = _make_cfg(tmp_path)
    original_path = cfg.paths["input_originals"] / "Case_01.png"
    Image.fromarray(_blank_rgb(6, 6), mode="RGB").save(original_path)
    _write_case_map(cfg, [{"case_id": "Case_01", "source_path": "data/input_originals/Case_01.png"}])

    exit_code = main(["--root", str(tmp_path)])
    assert exit_code == EXIT_RUNTIME
    assert not (tmp_path / "data" / "paths" / "Case_01_paths_overlay.png").exists()


def test_cli_dry_run_writes_nothing(tmp_path: Path):
    cfg = _make_cfg(tmp_path)
    _write_case_map(cfg, [{"case_id": "Case_01", "source_path": "data/input_originals/Case_01.png"}])

    exit_code = main(["--root", str(tmp_path), "--dry-run"])
    assert exit_code == EXIT_OK
    assert not (tmp_path / "data" / "paths").exists()
