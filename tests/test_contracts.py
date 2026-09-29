"""tests/test_contracts.py — schema validation for the on-disk contracts. Built by TC-014.

Hand-written schema checks (no `jsonschema` dependency) for every JSON artefact a downstream
consumer indexes by key rather than reading end-to-end: `data/case_map.json` (INTERFACES.md §2),
`{case}_skeleton_stats.json` (§3.4), D405 per-frame metadata (§3.10), and the `logs/*_latest.json`
run-summary shape (§0.4). Every fixture here is produced by the real component under a throwaway
`tmp_project` root — never a hand-typed JSON blob standing in for the real output.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION
from crackvision.prepare_inputs import main as prepare_inputs_main
from crackvision.realsense_capture import main as realsense_capture_main
from crackvision.skeleton import skeletonize_case

_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def _write_rgb(path: Path, size: tuple[int, int]) -> None:
    width, height = size
    rng = np.random.default_rng(0)
    arr = rng.integers(0, 255, size=(height, width, 3), dtype=np.uint8)
    Image.fromarray(arr, mode="RGB").save(path)


# ---------------------------------------------------------------------------
# data/case_map.json (INTERFACES.md §2)
# ---------------------------------------------------------------------------


def test_case_map_contract(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb(in_dir / "b.png", (30, 20))
    _write_rgb(in_dir / "a.png", (10, 8))

    rc = prepare_inputs_main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    case_map = json.loads((tmp_project.root / "data" / "case_map.json").read_text(encoding="utf-8"))

    assert case_map["schema_version"] == 1
    assert isinstance(case_map["generated_utc"], str) and case_map["generated_utc"]
    assert case_map["root"] == str(tmp_project.root)

    cases = case_map["cases"]
    assert isinstance(cases, list) and len(cases) == 2
    assert [c["case_id"] for c in cases] == sorted(c["case_id"] for c in cases)

    required_str_keys = ("case_id", "source_path", "source_name", "nnunet_input", "source_mode", "source_format")
    required_int_keys = ("width", "height")
    required_bool_keys = ("converted", "downscaled")

    for case in cases:
        for key in required_str_keys:
            assert isinstance(case[key], str) and case[key], f"{key} missing/empty in {case}"
        for key in required_int_keys:
            assert isinstance(case[key], int) and case[key] > 0, f"{key} invalid in {case}"
        for key in required_bool_keys:
            assert isinstance(case[key], bool), f"{key} not bool in {case}"
        assert isinstance(case["scale_factor"], (int, float))

        for path_key in ("source_path", "nnunet_input"):
            rel = case[path_key]
            assert not rel.startswith("/"), f"{path_key} not project-relative: {rel}"
            assert "\\" not in rel, f"{path_key} is not POSIX-style: {rel}"
            assert (tmp_project.root / rel).is_file(), f"{path_key} does not point at an existing file: {rel}"

        for hash_key in ("sha256_source", "sha256_nnunet_input"):
            assert _SHA256_RE.match(case[hash_key]), f"{hash_key} is not 64 hex chars: {case[hash_key]!r}"


# ---------------------------------------------------------------------------
# {case}_skeleton_stats.json (INTERFACES.md §3.4)
# ---------------------------------------------------------------------------


def test_skeleton_stats_contract(tmp_project: Config) -> None:
    size = (48, 32)  # (width, height)
    width, height = size
    original_path = tmp_project.paths["input_originals"] / "case.png"
    _write_rgb(original_path, size)

    pred = np.zeros((height, width), dtype=np.uint8)
    pred[4:8, 4:20] = 1  # one horizontal bar: a clean, connected mask
    pred[20, 5] = 1  # an isolated single-pixel speck, a second component
    Image.fromarray(pred, mode="L").save(tmp_project.paths["predictions"] / "case.png")

    case_entry = {"case_id": "case", "source_path": "data/input_originals/case.png"}
    result = skeletonize_case(
        tmp_project,
        case_entry,
        min_component_size=0,  # disabled: keep both components so before/after differ meaningfully
        dilate=0,
        color=(0, 255, 0),
        predictions_dir=tmp_project.paths["predictions"],
        skeletons_dir=tmp_project.paths["skeletons"],
    )
    assert result["status"] == "ok"

    stats = json.loads(Path(result["stats_path"]).read_text(encoding="utf-8"))

    required_keys = {
        "case_id", "schema_version", "image_height", "image_width",
        "mask_pixels", "mask_fraction", "components_before", "components_after",
        "min_component_size", "skeleton_pixels", "skeleton_fraction",
    }
    assert required_keys <= stats.keys()

    assert stats["image_height"] == height
    assert stats["image_width"] == width

    expected_fraction = stats["mask_pixels"] / (stats["image_height"] * stats["image_width"])
    assert math.isclose(stats["mask_fraction"], expected_fraction, rel_tol=1e-9, abs_tol=1e-9)

    assert stats["skeleton_pixels"] <= stats["mask_pixels"]
    assert stats["components_after"] <= stats["components_before"]
    assert stats["components_before"] == 2  # the bar and the speck, disjoint


# ---------------------------------------------------------------------------
# D405 per-frame metadata (INTERFACES.md §3.10)
# ---------------------------------------------------------------------------


def test_d405_metadata_contract(tmp_project: Config) -> None:
    rc = realsense_capture_main(["--root", str(tmp_project.root), "--session", "contracttest", "--synthetic", "1"])
    assert rc == EXIT_OK

    meta_dir = tmp_project.paths["d405"] / "metadata"
    frame_files = sorted(p for p in meta_dir.glob("synth_contracttest_*.json") if not p.name.endswith("_session.json"))
    assert len(frame_files) == 1
    meta = json.loads(frame_files[0].read_text(encoding="utf-8"))

    required_keys = {
        "schema_version", "session", "frame_index", "timestamp_utc", "device",
        "aligned_to", "warmup_frames", "depth_scale_m_per_unit", "color", "depth",
        "extrinsics_depth_to_color", "exposure_us", "gain", "synthetic", "files",
    }
    assert required_keys <= meta.keys()

    depth_scale = meta["depth_scale_m_per_unit"]
    assert isinstance(depth_scale, float) and depth_scale > 0.0

    assert meta["aligned_to"] == "color"

    for stream in ("color", "depth"):
        intrinsics = meta[stream]["intrinsics"]
        for key in ("fx", "fy", "ppx", "ppy", "model", "coeffs"):
            assert key in intrinsics, f"{stream} intrinsics missing {key}"
        assert isinstance(intrinsics["coeffs"], list)

    extrinsics = meta["extrinsics_depth_to_color"]
    assert len(extrinsics["rotation"]) == 9
    assert len(extrinsics["translation"]) == 3


# ---------------------------------------------------------------------------
# logs/*_latest.json (INTERFACES.md §0.4)
# ---------------------------------------------------------------------------


def _assert_run_summary_contract(summary: dict[str, Any]) -> None:
    required_keys = {
        "tool", "schema_version", "started_utc", "finished_utc", "duration_s",
        "status", "exit_code", "counts", "errors",
    }
    assert required_keys <= summary.keys()
    assert summary["status"] in {"ok", "partial", "failed", "precondition"}
    assert isinstance(summary["exit_code"], int)
    assert isinstance(summary["counts"], dict)
    assert isinstance(summary["errors"], list)
    assert isinstance(summary["duration_s"], (int, float)) and summary["duration_s"] >= 0


def test_logs_latest_json_contract_ok_run(tmp_project: Config) -> None:
    _write_rgb(tmp_project.paths["input_originals"] / "a.png", (12, 9))

    rc = prepare_inputs_main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    summary = json.loads((tmp_project.paths["logs"] / "prepare_inputs_latest.json").read_text(encoding="utf-8"))
    assert summary["tool"] == "prepare_inputs"
    assert summary["status"] == "ok"
    assert summary["exit_code"] == EXIT_OK
    _assert_run_summary_contract(summary)


def test_logs_latest_json_contract_precondition_run(tmp_project: Config) -> None:
    # input_originals/ is empty: prepare_inputs must report status "precondition" (docs/INTERFACES.md §0.4).
    rc = prepare_inputs_main(["--root", str(tmp_project.root)])
    assert rc == EXIT_PRECONDITION

    summary = json.loads((tmp_project.paths["logs"] / "prepare_inputs_latest.json").read_text(encoding="utf-8"))
    assert summary["status"] == "precondition"
    _assert_run_summary_contract(summary)
