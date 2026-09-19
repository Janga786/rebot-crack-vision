"""tests/test_visualize.py — mask/overlay/comparison rendering. Built by TC-009.

No GPU, no camera, no network: every fixture here is a synthetic original + a hand-built
prediction PNG, written directly under the `tmp_project` fixture's tree, per
docs/INTERFACES.md §3.3 and task_cards/TC-009-visualization.md. `prepare_inputs`/`inference` are
never invoked here.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from PIL import Image

from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME
from crackvision.visualize import main, render_overlay


def _write_case_map(cfg: Config, cases: list[dict[str, Any]]) -> Path:
    case_map_path = cfg.root / "data" / "case_map.json"
    case_map_path.parent.mkdir(parents=True, exist_ok=True)
    case_map_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "generated_utc": "2026-01-01T00:00:00Z",
                "root": str(cfg.root),
                "cases": cases,
            }
        ),
        encoding="utf-8",
    )
    return case_map_path


def _write_original(cfg: Config, name: str, size: tuple[int, int], mode: str = "RGB") -> Path:
    width, height = size
    path = cfg.paths["input_originals"] / name
    rng = np.random.default_rng(0)
    if mode == "L":
        arr = rng.integers(0, 255, size=(height, width), dtype=np.uint8)
        Image.fromarray(arr, mode="L").save(path)
    else:
        arr = rng.integers(0, 255, size=(height, width, 3), dtype=np.uint8)
        Image.fromarray(arr, mode="RGB").save(path)
    return path


def _write_prediction(cfg: Config, case_id: str, arr: np.ndarray) -> Path:
    path = cfg.paths["predictions"] / f"{case_id}.png"
    Image.fromarray(arr.astype(np.uint8), mode="L").save(path)
    return path


def _case_entry(cfg: Config, source_path: Path, case_id: str, size: tuple[int, int], mode: str = "RGB") -> dict[str, Any]:
    width, height = size
    return {
        "case_id": case_id,
        "source_path": source_path.resolve().relative_to(cfg.root).as_posix(),
        "source_name": source_path.name,
        "nnunet_input": f"data/nnunet_input/{case_id}_0000.png",
        "width": width,
        "height": height,
        "source_mode": mode,
        "source_format": "PNG",
        "converted": True,
        "downscaled": False,
        "scale_factor": 1.0,
        "sha256_source": "0" * 64,
        "sha256_nnunet_input": "0" * 64,
    }


def _read_summary(cfg: Config) -> dict[str, Any]:
    return json.loads((cfg.paths["logs"] / "visualize_latest.json").read_text(encoding="utf-8"))


def test_prediction_values_0_1_yield_mask_0_255(tmp_project: Config) -> None:
    size = (20, 16)
    src = _write_original(tmp_project, "a.png", size)
    pred = np.zeros(size[::-1], dtype=np.uint8)
    pred[4:8, 4:8] = 1
    _write_prediction(tmp_project, "a", pred)
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "a", size)])

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    mask = np.array(Image.open(tmp_project.paths["overlays"] / "a_mask.png"))
    assert set(np.unique(mask).tolist()) == {0, 255}
    assert mask[4:8, 4:8].sum() == 16 * 255


def test_prediction_values_0_255_yield_same_mask_and_crack_pixels(tmp_project: Config) -> None:
    size = (20, 16)
    src01 = _write_original(tmp_project, "a.png", size)
    src0255 = _write_original(tmp_project, "b.png", size)

    pred01 = np.zeros(size[::-1], dtype=np.uint8)
    pred01[4:8, 4:8] = 1
    pred0255 = np.zeros(size[::-1], dtype=np.uint8)
    pred0255[4:8, 4:8] = 255
    _write_prediction(tmp_project, "a", pred01)
    _write_prediction(tmp_project, "b", pred0255)
    _write_case_map(
        tmp_project,
        [_case_entry(tmp_project, src01, "a", size), _case_entry(tmp_project, src0255, "b", size)],
    )

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    mask_a = np.array(Image.open(tmp_project.paths["overlays"] / "a_mask.png"))
    mask_b = np.array(Image.open(tmp_project.paths["overlays"] / "b_mask.png"))
    assert set(np.unique(mask_b).tolist()) == {0, 255}
    assert np.array_equal(mask_a, mask_b)


def test_all_zero_prediction_is_not_an_error(tmp_project: Config) -> None:
    size = (12, 10)
    src = _write_original(tmp_project, "blank.png", size)
    _write_prediction(tmp_project, "blank", np.zeros(size[::-1], dtype=np.uint8))
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "blank", size)])

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    mask = np.array(Image.open(tmp_project.paths["overlays"] / "blank_mask.png"))
    assert (mask == 0).all()
    assert (tmp_project.paths["overlays"] / "blank_overlay.png").is_file()
    assert (tmp_project.paths["comparisons"] / "blank_comparison.png").is_file()

    summary = _read_summary(tmp_project)
    assert summary["status"] == "ok"
    assert summary["exit_code"] == EXIT_OK


def test_all_ones_prediction_overlay_equals_blend_everywhere(tmp_project: Config) -> None:
    size = (10, 8)
    src = _write_original(tmp_project, "full.png", size)
    original_arr = np.array(Image.open(src).convert("RGB"))
    _write_prediction(tmp_project, "full", np.ones(size[::-1], dtype=np.uint8))
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "full", size)])

    alpha = 0.5
    color = (255, 0, 0)
    rc = main(["--root", str(tmp_project.root), "--alpha", str(alpha), "--color", "255,0,0"])
    assert rc == EXIT_OK

    overlay = np.array(Image.open(tmp_project.paths["overlays"] / "full_overlay.png"))
    binary_all_true = np.ones(size[::-1], dtype=bool)
    expected = render_overlay(original_arr, binary_all_true, alpha, color)
    assert np.array_equal(overlay, expected)


def test_output_dimensions_match_frame_invariant(tmp_project: Config) -> None:
    size = (30, 20)  # (width, height)
    src = _write_original(tmp_project, "dims.png", size)
    pred = np.zeros(size[::-1], dtype=np.uint8)
    pred[0:5, 0:5] = 1
    _write_prediction(tmp_project, "dims", pred)
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "dims", size)])

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    width, height = size
    gutter = tmp_project.visualization["panel_gutter_px"]
    with Image.open(tmp_project.paths["overlays"] / "dims_mask.png") as mask_img:
        assert mask_img.size == (width, height)
    with Image.open(tmp_project.paths["overlays"] / "dims_overlay.png") as overlay_img:
        assert overlay_img.size == (width, height)
    with Image.open(tmp_project.paths["comparisons"] / "dims_comparison.png") as cmp_img:
        assert cmp_img.size == (width * 3 + gutter * 2, height)


def test_shape_mismatch_fails_that_case_and_others_continue(tmp_project: Config) -> None:
    good_size = (16, 12)
    good_src = _write_original(tmp_project, "good.png", good_size)
    _write_prediction(tmp_project, "good", np.zeros(good_size[::-1], dtype=np.uint8))

    mismatched_size = (16, 12)
    mismatched_src = _write_original(tmp_project, "mismatch.png", mismatched_size)
    # prediction has a different shape than the (16, 12) original -> hard per-case error
    _write_prediction(tmp_project, "mismatch", np.zeros((5, 5), dtype=np.uint8))

    _write_case_map(
        tmp_project,
        [
            _case_entry(tmp_project, good_src, "good", good_size),
            _case_entry(tmp_project, mismatched_src, "mismatch", mismatched_size),
        ],
    )

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_RUNTIME

    assert (tmp_project.paths["overlays"] / "good_mask.png").is_file()
    assert not (tmp_project.paths["overlays"] / "mismatch_mask.png").exists()

    summary = _read_summary(tmp_project)
    assert summary["status"] == "partial"
    assert summary["counts"]["failed"] == 1
    assert summary["counts"]["processed"] == 1


def test_grayscale_original_yields_rgb_overlay(tmp_project: Config) -> None:
    size = (14, 11)
    src = _write_original(tmp_project, "gray.png", size, mode="L")
    _write_prediction(tmp_project, "gray", np.zeros(size[::-1], dtype=np.uint8))
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "gray", size, mode="L")])

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    with Image.open(tmp_project.paths["overlays"] / "gray_overlay.png") as overlay_img:
        assert overlay_img.mode == "RGB"
        arr = np.array(overlay_img)
        assert arr.shape == (size[1], size[0], 3)


def test_dry_run_writes_zero_files(tmp_project: Config) -> None:
    size = (10, 10)
    src = _write_original(tmp_project, "x.png", size)
    _write_prediction(tmp_project, "x", np.zeros(size[::-1], dtype=np.uint8))
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "x", size)])

    rc = main(["--root", str(tmp_project.root), "--dry-run"])
    assert rc == EXIT_OK

    assert list(tmp_project.paths["overlays"].glob("*")) == []
    assert list(tmp_project.paths["comparisons"].glob("*")) == []


def test_missing_case_map_exits_precondition(tmp_project: Config) -> None:
    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_PRECONDITION
