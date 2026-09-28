"""tests/test_skeleton.py — mask cleanup and skeletonization. Built by TC-010.

Synthetic masks only, no model needed, per task_cards/TC-010-skeletonization.md. `prepare_inputs`/
`inference` are never invoked here.
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
from crackvision.skeleton import clean_and_skeletonize, main


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


def _write_original(cfg: Config, name: str, size: tuple[int, int]) -> Path:
    width, height = size
    path = cfg.paths["input_originals"] / name
    rng = np.random.default_rng(0)
    arr = rng.integers(0, 255, size=(height, width, 3), dtype=np.uint8)
    Image.fromarray(arr, mode="RGB").save(path)
    return path


def _write_prediction(cfg: Config, case_id: str, arr: np.ndarray) -> Path:
    path = cfg.paths["predictions"] / f"{case_id}.png"
    Image.fromarray(arr.astype(np.uint8), mode="L").save(path)
    return path


def _case_entry(cfg: Config, source_path: Path, case_id: str, size: tuple[int, int]) -> dict[str, Any]:
    width, height = size
    return {
        "case_id": case_id,
        "source_path": source_path.resolve().relative_to(cfg.root).as_posix(),
        "source_name": source_path.name,
        "nnunet_input": f"data/nnunet_input/{case_id}_0000.png",
        "width": width,
        "height": height,
        "source_mode": "RGB",
        "source_format": "PNG",
        "converted": True,
        "downscaled": False,
        "scale_factor": 1.0,
        "sha256_source": "0" * 64,
        "sha256_nnunet_input": "0" * 64,
    }


def _read_stats(cfg: Config, case_id: str) -> dict[str, Any]:
    return json.loads((cfg.paths["skeletons"] / f"{case_id}_skeleton_stats.json").read_text(encoding="utf-8"))


# ---------------------------------------------------------------------------
# Pure-function tests against clean_and_skeletonize / stats math
# ---------------------------------------------------------------------------


def test_horizontal_bar_skeletonizes_to_one_pixel_tall_line() -> None:
    mask = np.zeros((20, 220), dtype=bool)
    mask[7:12, 10:210] = True  # 5 px tall, 200 px long

    _cleaned, skeleton, before, after = clean_and_skeletonize(mask, min_component_size=64)

    assert before == 1
    assert after == 1
    rows_with_pixels = np.where(skeleton.any(axis=1))[0]
    for row in rows_with_pixels:
        assert skeleton[row].sum() <= mask[row].sum()
    col_counts = skeleton.sum(axis=0)
    assert (col_counts[10:210] <= 1).all()
    assert abs(int(skeleton.sum()) - 200) <= 5


def test_filled_square_skeleton_much_smaller_than_mask() -> None:
    mask = np.zeros((60, 60), dtype=bool)
    mask[5:55, 5:55] = True  # 50x50 filled square

    _cleaned, skeleton, _before, _after = clean_and_skeletonize(mask, min_component_size=64)

    assert int(skeleton.sum()) < int(mask.sum()) // 10


def test_small_component_removed_with_default_min_size() -> None:
    mask = np.zeros((100, 100), dtype=bool)
    mask[0:3, 0:3] = True  # ~9 px blob, below min_size=64
    mask[20:90, 20:90] = True  # 4900 px blob

    _cleaned, _skeleton, before, after = clean_and_skeletonize(mask, min_component_size=64)

    assert before == 2
    assert after == 1


def test_small_component_kept_when_min_size_zero() -> None:
    mask = np.zeros((100, 100), dtype=bool)
    mask[0:3, 0:3] = True
    mask[20:90, 20:90] = True

    _cleaned, _skeleton, before, after = clean_and_skeletonize(mask, min_component_size=0)

    assert before == 2
    assert after == 2


def test_all_zero_mask_yields_all_zero_skeleton() -> None:
    mask = np.zeros((50, 50), dtype=bool)

    _cleaned, skeleton, before, after = clean_and_skeletonize(mask, min_component_size=64)

    assert before == 0
    assert after == 0
    assert not skeleton.any()


@pytest.mark.parametrize("min_size", [0, 64])
def test_skeleton_pixels_never_exceed_mask_pixels(min_size: int) -> None:
    rng = np.random.default_rng(1)
    mask = rng.integers(0, 2, size=(80, 80)).astype(bool)

    _cleaned, skeleton, _before, _after = clean_and_skeletonize(mask, min_component_size=min_size)

    assert int(skeleton.sum()) <= int(mask.sum())
    assert skeleton.shape == mask.shape


# ---------------------------------------------------------------------------
# CLI-level tests
# ---------------------------------------------------------------------------


def test_missing_case_map_exits_precondition(tmp_project: Config) -> None:
    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_PRECONDITION


def test_prediction_0_1_and_0_255_give_identical_results(tmp_project: Config) -> None:
    size = (60, 60)
    src01 = _write_original(tmp_project, "a.png", size)
    src0255 = _write_original(tmp_project, "b.png", size)

    pred01 = np.zeros(size[::-1], dtype=np.uint8)
    pred01[10:50, 10:50] = 1
    pred0255 = np.zeros(size[::-1], dtype=np.uint8)
    pred0255[10:50, 10:50] = 255
    _write_prediction(tmp_project, "a", pred01)
    _write_prediction(tmp_project, "b", pred0255)
    _write_case_map(
        tmp_project,
        [_case_entry(tmp_project, src01, "a", size), _case_entry(tmp_project, src0255, "b", size)],
    )

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    skel_a = np.array(Image.open(tmp_project.paths["skeletons"] / "a_skeleton.png"))
    skel_b = np.array(Image.open(tmp_project.paths["skeletons"] / "b_skeleton.png"))
    assert np.array_equal(skel_a, skel_b)

    stats_a = _read_stats(tmp_project, "a")
    stats_b = _read_stats(tmp_project, "b")
    assert stats_a["skeleton_pixels"] == stats_b["skeleton_pixels"]


def test_stats_schema_and_mask_fraction(tmp_project: Config) -> None:
    size = (40, 30)
    src = _write_original(tmp_project, "c.png", size)
    pred = np.zeros(size[::-1], dtype=np.uint8)
    pred[5:25, 5:25] = 1
    _write_prediction(tmp_project, "c", pred)
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "c", size)])

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    stats = _read_stats(tmp_project, "c")
    width, height = size
    expected_fields = {
        "case_id",
        "schema_version",
        "image_height",
        "image_width",
        "mask_pixels",
        "mask_fraction",
        "components_before",
        "components_after",
        "min_component_size",
        "skeleton_pixels",
        "skeleton_fraction",
    }
    assert expected_fields <= stats.keys()
    assert stats["image_height"] == height
    assert stats["image_width"] == width
    assert stats["mask_pixels"] == 400
    assert stats["mask_fraction"] == pytest.approx(400 / (width * height))
    assert stats["skeleton_pixels"] <= stats["mask_pixels"]


def test_all_zero_mask_case_is_not_an_error(tmp_project: Config) -> None:
    size = (20, 20)
    src = _write_original(tmp_project, "blank.png", size)
    _write_prediction(tmp_project, "blank", np.zeros(size[::-1], dtype=np.uint8))
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "blank", size)])

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    skeleton = np.array(Image.open(tmp_project.paths["skeletons"] / "blank_skeleton.png"))
    assert (skeleton == 0).all()
    stats = _read_stats(tmp_project, "blank")
    assert stats["mask_pixels"] == 0
    assert stats["skeleton_pixels"] == 0
    assert stats["mask_fraction"] == 0.0


def test_output_dimensions_match_frame_invariant(tmp_project: Config) -> None:
    size = (35, 22)  # (width, height)
    src = _write_original(tmp_project, "dims.png", size)
    pred = np.zeros(size[::-1], dtype=np.uint8)
    pred[0:5, 0:5] = 1
    _write_prediction(tmp_project, "dims", pred)
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "dims", size)])

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    width, height = size
    with Image.open(tmp_project.paths["skeletons"] / "dims_skeleton.png") as img:
        assert img.size == (width, height)
        assert img.mode == "L"
        assert set(np.array(img).flatten().tolist()) <= {0, 255}


def test_min_component_size_flag_drops_small_blob(tmp_project: Config) -> None:
    size = (100, 100)
    src = _write_original(tmp_project, "blobs.png", size)
    pred = np.zeros(size[::-1], dtype=np.uint8)
    pred[0:3, 0:3] = 1  # small blob, below min_size=64
    pred[20:90, 20:90] = 1  # large blob
    _write_prediction(tmp_project, "blobs", pred)
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "blobs", size)])

    rc = main(["--root", str(tmp_project.root), "--min-component-size", "64"])
    assert rc == EXIT_OK
    stats = _read_stats(tmp_project, "blobs")
    assert stats["components_before"] == 2
    assert stats["components_after"] == 1

    rc = main(["--root", str(tmp_project.root), "--min-component-size", "0"])
    assert rc == EXIT_OK
    stats = _read_stats(tmp_project, "blobs")
    assert stats["components_before"] == 2
    assert stats["components_after"] == 2


def test_dilate_does_not_alter_skeleton_png(tmp_project: Config) -> None:
    size = (60, 60)
    src = _write_original(tmp_project, "dil.png", size)
    pred = np.zeros(size[::-1], dtype=np.uint8)
    pred[10:50, 10:50] = 1
    _write_prediction(tmp_project, "dil", pred)
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "dil", size)])

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK
    skeleton_no_dilate = np.array(Image.open(tmp_project.paths["skeletons"] / "dil_skeleton.png"))
    overlay_no_dilate = np.array(Image.open(tmp_project.paths["skeletons"] / "dil_skeleton_overlay.png"))

    rc = main(["--root", str(tmp_project.root), "--dilate", "3"])
    assert rc == EXIT_OK
    skeleton_dilated = np.array(Image.open(tmp_project.paths["skeletons"] / "dil_skeleton.png"))
    overlay_dilated = np.array(Image.open(tmp_project.paths["skeletons"] / "dil_skeleton_overlay.png"))

    assert np.array_equal(skeleton_no_dilate, skeleton_dilated)
    assert not np.array_equal(overlay_no_dilate, overlay_dilated)


def test_no_overlay_flag_skips_overlay_file(tmp_project: Config) -> None:
    size = (20, 20)
    src = _write_original(tmp_project, "noov.png", size)
    pred = np.zeros(size[::-1], dtype=np.uint8)
    pred[5:15, 5:15] = 1
    _write_prediction(tmp_project, "noov", pred)
    _write_case_map(tmp_project, [_case_entry(tmp_project, src, "noov", size)])

    rc = main(["--root", str(tmp_project.root), "--no-overlay"])
    assert rc == EXIT_OK
    assert (tmp_project.paths["skeletons"] / "noov_skeleton.png").is_file()
    assert not (tmp_project.paths["skeletons"] / "noov_skeleton_overlay.png").is_file()
