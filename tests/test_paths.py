"""tests/test_paths.py — ordered crack polylines and the path-extraction contract. Built by PERC-03.

Synthetic {0,255}/bool skeleton rasters only, mirroring tests/test_skeleton_graph.py's fixtures.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from PIL import Image

from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION
from crackvision.paths import (
    DEFAULT_RDP_TOLERANCE_PX,
    build_case_paths,
    main,
    order_components,
    rdp_simplify,
    rdp_tolerance_from_config,
)


def _canvas(height: int, width: int) -> np.ndarray:
    return np.zeros((height, width), dtype=bool)


def _draw(canvas: np.ndarray, pixels: list[tuple[int, int]]) -> np.ndarray:
    for r, c in pixels:
        canvas[r, c] = True
    return canvas


def _points(entries: list[dict[str, int]]) -> list[tuple[int, int]]:
    return [(p["row"], p["col"]) for p in entries]


# ---------------------------------------------------------------------------
# RDP simplification
# ---------------------------------------------------------------------------


def test_rdp_simplify_straight_line_collapses_to_endpoints():
    points = [(0, c) for c in range(10)]
    assert rdp_simplify(points, tolerance_px=0.5) == [(0, 0), (0, 9)]


def test_rdp_simplify_right_angle_keeps_the_corner():
    points = [(0, c) for c in range(6)] + [(r, 5) for r in range(1, 6)]
    simplified = rdp_simplify(points, tolerance_px=0.5)
    assert simplified[0] == (0, 0)
    assert simplified[-1] == (5, 5)
    assert (0, 5) in simplified  # the corner survives


def test_rdp_simplify_short_input_returned_unchanged():
    assert rdp_simplify([(0, 0)], 1.0) == [(0, 0)]
    assert rdp_simplify([(0, 0), (0, 1)], 1.0) == [(0, 0), (0, 1)]


def test_rdp_tolerance_from_config_default_and_override():
    class Cfg:
        raw: dict[str, Any] = {}

    assert rdp_tolerance_from_config(Cfg()) == DEFAULT_RDP_TOLERANCE_PX

    Cfg.raw = {"path_extraction": {"rdp_tolerance_px": 3.25}}
    assert rdp_tolerance_from_config(Cfg()) == 3.25


# ---------------------------------------------------------------------------
# Straight and curved single-component skeletons
# ---------------------------------------------------------------------------


def test_straight_line_single_main_path_no_branches():
    skel = _canvas(5, 10)
    _draw(skel, [(2, c) for c in range(2, 8)])

    doc = build_case_paths(skel, "line", min_spur_length_px=0, rdp_tolerance_px=1.0)

    assert doc["component_count"] == 1
    comp = doc["components"][0]
    assert comp["branches"] == []
    main_path = comp["main_path"]
    assert main_path["length_px"] == 5
    dense_points = _points(main_path["dense"])
    # the diameter search may walk either endpoint-to-endpoint direction (a documented,
    # arbitrary-but-deterministic choice); the pixel set and both endpoints are what matter.
    expected = [(2, c) for c in range(2, 8)]
    assert dense_points in (expected, list(reversed(expected)))
    assert {main_path["simplified"][0]["row"], main_path["simplified"][-1]["row"]} == {2}
    assert {main_path["simplified"][0]["col"], main_path["simplified"][-1]["col"]} == {2, 7}


def test_curved_line_dense_path_covers_every_pixel_and_bend_survives_simplification():
    skel = _canvas(10, 10)
    pixels = [(1, 1), (2, 2), (3, 3), (3, 4), (3, 5), (3, 6)]
    _draw(skel, pixels)

    doc = build_case_paths(skel, "curve", min_spur_length_px=0, rdp_tolerance_px=0.1)

    comp = doc["components"][0]
    main_path = comp["main_path"]
    dense_points = _points(main_path["dense"])
    assert set(dense_points) == set(pixels)
    assert len(dense_points) == len(pixels)
    simplified_points = _points(main_path["simplified"])
    assert (3, 3) in simplified_points  # the bend is a real corner, kept at a tight tolerance


# ---------------------------------------------------------------------------
# Branching
# ---------------------------------------------------------------------------


def test_t_shape_main_path_is_the_long_bar_branch_is_the_stem():
    skel = _canvas(12, 15)
    _draw(skel, [(6, c) for c in range(1, 12)])  # horizontal bar, row 6 (both legs length 5)
    _draw(skel, [(4, 6), (5, 6)])  # short (length-2) stem, touches the bar at (6, 6)

    doc = build_case_paths(skel, "t", min_spur_length_px=0, rdp_tolerance_px=1.0)

    comp = doc["components"][0]
    assert len(comp["branches"]) == 1
    main_dense = _points(comp["main_path"]["dense"])
    assert {main_dense[0], main_dense[-1]} == {(6, 1), (6, 11)}
    branch = comp["branches"][0]
    assert branch["kind"] == "branch"
    branch_dense = _points(branch["dense"])
    assert (4, 6) in (branch_dense[0], branch_dense[-1])


def test_branches_ordered_longest_first():
    # A junction with three legs of distinct lengths: main path = the two longest joined,
    # the remaining (shortest) leg is the sole branch.
    skel = _canvas(20, 20)
    _draw(skel, [(10, c) for c in range(1, 11)])  # west leg, length 9
    _draw(skel, [(10, c) for c in range(10, 17)])  # east leg, length 6
    _draw(skel, [(r, 10) for r in range(1, 10)])  # north leg, length 8 (shortest after the merge)

    doc = build_case_paths(skel, "y", min_spur_length_px=0, rdp_tolerance_px=1.0)

    comp = doc["components"][0]
    lengths = [b["length_px"] for b in comp["branches"]]
    assert lengths == sorted(lengths, reverse=True)
    assert comp["main_path"]["length_px"] >= max(lengths, default=0)


def test_min_spur_length_px_prunes_short_spur_before_extraction():
    skel = _canvas(12, 15)
    _draw(skel, [(6, c) for c in range(1, 11)])  # both bar arms long
    _draw(skel, [(4, 6), (5, 6)])  # a short 2-px spur off the bar

    pruned = build_case_paths(skel, "pruned", min_spur_length_px=100, rdp_tolerance_px=1.0)
    unpruned = build_case_paths(skel, "unpruned", min_spur_length_px=0, rdp_tolerance_px=1.0)

    assert pruned["components"][0]["branches"] == []
    assert unpruned["components"][0]["branches"] != []


# ---------------------------------------------------------------------------
# Loops
# ---------------------------------------------------------------------------


def test_looped_diamond_main_path_is_the_whole_cycle_no_branches():
    # Diagonal-only diamond: a genuine 1-px cycle under 8-connectivity (see test_skeleton_graph.py).
    skel = _canvas(9, 9)
    ring = [
        (1, 4), (2, 5), (3, 6), (4, 7),
        (5, 6), (6, 5), (7, 4),
        (6, 3), (5, 2), (4, 1),
        (3, 2), (2, 3),
    ]
    _draw(skel, ring)

    doc = build_case_paths(skel, "loop", min_spur_length_px=0, rdp_tolerance_px=1.0)

    assert doc["component_count"] == 1
    comp = doc["components"][0]
    assert comp["branches"] == []
    assert comp["main_path"]["length_px"] == len(ring)
    dense_points = _points(comp["main_path"]["dense"])
    assert set(dense_points[:-1]) == set(ring)
    assert dense_points[0] == dense_points[-1]  # closed loop


# ---------------------------------------------------------------------------
# Isolated points and empty input
# ---------------------------------------------------------------------------


def test_isolated_single_pixel_emits_one_point_main_path():
    skel = _canvas(5, 5)
    skel[2, 2] = True

    doc = build_case_paths(skel, "dot", min_spur_length_px=0, rdp_tolerance_px=1.0)

    comp = doc["components"][0]
    assert comp["main_path"]["dense"] == [{"row": 2, "col": 2, "u": 2, "v": 2}]
    assert comp["main_path"]["simplified"] == comp["main_path"]["dense"]
    assert comp["main_path"]["length_px"] == 0
    assert comp["branches"] == []


def test_empty_skeleton_has_no_components():
    skel = _canvas(6, 6)
    doc = build_case_paths(skel, "empty", min_spur_length_px=0, rdp_tolerance_px=1.0)
    assert doc["component_count"] == 0
    assert doc["components"] == []


# ---------------------------------------------------------------------------
# Multi-crack ordering
# ---------------------------------------------------------------------------


def test_multi_crack_ordering_visits_nearest_component_next():
    skel = _canvas(30, 30)
    _draw(skel, [(2, c) for c in range(1, 5)])  # component A near the top-left
    _draw(skel, [(28, c) for c in range(25, 29)])  # component B far away
    _draw(skel, [(5, c) for c in range(1, 5)])  # component C, close to A (3-row gap, not touching)

    doc = build_case_paths(skel, "multi", min_spur_length_px=0, rdp_tolerance_px=1.0)

    assert doc["component_count"] == 3
    order_indices = [c["order_index"] for c in doc["components"]]
    assert order_indices == [0, 1, 2]
    first, second, third = doc["components"]
    d_first_second = _endpoint_distance(first, second)
    d_first_third = _endpoint_distance(first, third)
    assert d_first_second <= d_first_third


def _endpoint_distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    ax, ay = a["main_path"]["dense"][-1]["row"], a["main_path"]["dense"][-1]["col"]
    b_start = b["main_path"]["dense"][0]
    return ((ax - b_start["row"]) ** 2 + (ay - b_start["col"]) ** 2) ** 0.5


def test_ordering_is_deterministic_across_rebuilds():
    skel = _canvas(30, 30)
    _draw(skel, [(2, c) for c in range(1, 5)])
    _draw(skel, [(28, c) for c in range(25, 29)])
    _draw(skel, [(15, c) for c in range(10, 14)])

    doc_a = build_case_paths(skel, "multi", min_spur_length_px=0, rdp_tolerance_px=1.0)
    doc_b = build_case_paths(skel, "multi", min_spur_length_px=0, rdp_tolerance_px=1.0)
    assert doc_a == doc_b


def test_order_components_flips_direction_to_minimise_travel():
    from crackvision.paths import ComponentPaths, Polyline

    # Component 0 ends at (0, 0); component 1's *end* (not start) is much closer to that point
    # than its start, so the tour should flip it.
    comp0 = ComponentPaths(
        component_id=0,
        main_path=Polyline(kind="main", dense=((10, 10), (0, 0)), simplified=((10, 10), (0, 0))),
        branches=(),
    )
    comp1 = ComponentPaths(
        component_id=1,
        main_path=Polyline(kind="main", dense=((5, 5), (0, 1)), simplified=((5, 5), (0, 1))),
        branches=(),
    )

    ordered = order_components([comp0, comp1])
    assert ordered[0].component_id == 0
    assert ordered[1].component_id == 1
    assert ordered[1].main_path.dense[0] == (0, 1)  # flipped: nearer endpoint visited first


# ---------------------------------------------------------------------------
# Frame invariant and JSON point schema
# ---------------------------------------------------------------------------


def test_frame_invariant_shape_recorded_not_altered():
    height, width = 37, 91
    skel = _canvas(height, width)
    _draw(skel, [(18, c) for c in range(10, 20)])

    doc = build_case_paths(skel, "shape", min_spur_length_px=0, rdp_tolerance_px=1.0)

    assert doc["image_height"] == height
    assert doc["image_width"] == width


def test_point_schema_has_row_col_and_uv_explicitly():
    skel = _canvas(5, 10)
    _draw(skel, [(2, c) for c in range(2, 8)])

    doc = build_case_paths(skel, "line", min_spur_length_px=0, rdp_tolerance_px=1.0)
    dense = doc["components"][0]["main_path"]["dense"]
    assert {dense[0]["col"], dense[-1]["col"]} == {2, 7}
    for p in dense:
        assert p["u"] == p["col"]
        assert p["v"] == p["row"]


def test_document_records_extraction_parameters():
    skel = _canvas(5, 10)
    _draw(skel, [(2, c) for c in range(2, 8)])
    doc = build_case_paths(skel, "line", min_spur_length_px=7, rdp_tolerance_px=2.5)
    assert doc["min_spur_length_px"] == 7
    assert doc["rdp_tolerance_px"] == 2.5
    assert doc["case_id"] == "line"
    assert doc["schema_version"] == 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _write_case_map(cfg: Config, cases: list[dict[str, Any]]) -> Path:
    case_map_path = cfg.root / "data" / "case_map.json"
    case_map_path.parent.mkdir(parents=True, exist_ok=True)
    case_map_path.write_text(
        json.dumps({"schema_version": 1, "root": str(cfg.root), "cases": cases}), encoding="utf-8"
    )
    return case_map_path


def _write_skeleton(cfg: Config, case_id: str, skel: np.ndarray) -> Path:
    path = cfg.paths["skeletons"] / f"{case_id}_skeleton.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray((skel.astype(np.uint8) * 255), mode="L").save(path)
    return path


def test_cli_writes_paths_json(tmp_path: Path):
    cfg = Config(
        root=tmp_path,
        paths={
            "input_originals": tmp_path / "data/input_originals",
            "nnunet_input": tmp_path / "data/nnunet_input",
            "predictions": tmp_path / "data/predictions",
            "overlays": tmp_path / "data/overlays",
            "comparisons": tmp_path / "data/comparisons",
            "skeletons": tmp_path / "data/skeletons",
            "smoke_test": tmp_path / "data/smoke_test",
            "d405": tmp_path / "data/d405",
            "models": tmp_path / "models/opencrack-nnunet",
            "logs": tmp_path / "logs",
        },
        model={},
        inference={},
        visualization={},
        skeleton={},
        realsense={},
        raw={"path_extraction": {"rdp_tolerance_px": 1.0}},
    )
    for directory in cfg.paths.values():
        Path(directory).mkdir(parents=True, exist_ok=True)
    (tmp_path / "config").mkdir(parents=True, exist_ok=True)
    (tmp_path / "config" / "project.yaml").write_text(
        "schema_version: 1\npath_extraction:\n  rdp_tolerance_px: 1.0\n", encoding="utf-8"
    )

    _write_case_map(cfg, [{"case_id": "Case_01", "source_path": "data/input_originals/Case_01.png"}])
    skel = _canvas(10, 10)
    _draw(skel, [(5, c) for c in range(2, 8)])
    _write_skeleton(cfg, "Case_01", skel)

    exit_code = main(["--root", str(tmp_path)])
    assert exit_code == EXIT_OK

    out_path = tmp_path / "data" / "paths" / "Case_01_paths.json"
    assert out_path.is_file()
    doc = json.loads(out_path.read_text(encoding="utf-8"))
    assert doc["case_id"] == "Case_01"
    assert doc["component_count"] == 1
    assert doc["rdp_tolerance_px"] == 1.0


def test_cli_missing_case_map_is_precondition_failure(tmp_path: Path):
    (tmp_path / "config").mkdir(parents=True)
    (tmp_path / "config" / "project.yaml").write_text("schema_version: 1\n", encoding="utf-8")
    (tmp_path / "src" / "crackvision").mkdir(parents=True)

    exit_code = main(["--root", str(tmp_path)])
    assert exit_code == EXIT_PRECONDITION


def test_cli_dry_run_writes_nothing(tmp_path: Path):
    cfg_paths = {
        key: tmp_path / f"data/{key}"
        for key in ["input_originals", "predictions", "overlays", "comparisons", "skeletons", "smoke_test", "d405"]
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
    _write_case_map(cfg, [{"case_id": "Case_01", "source_path": "data/input_originals/Case_01.png"}])

    exit_code = main(["--root", str(tmp_path), "--dry-run"])
    assert exit_code == EXIT_OK
    assert not (tmp_path / "data" / "paths").exists()
