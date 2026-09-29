"""tests/test_dataset_manifest.py — coverage for tools/dataset_manifest.py (TC-015).

Synthetic fixtures are hand-written per-frame metadata JSONs shaped like
`crackvision.realsense_capture --synthetic` output (TC-013), so this suite does not depend on
having captured real hardware frames.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

_MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "dataset_manifest.py"
_spec = importlib.util.spec_from_file_location("dataset_manifest", _MODULE_PATH)
dm = importlib.util.module_from_spec(_spec)
sys.modules["dataset_manifest"] = dm
_spec.loader.exec_module(dm)


def _run(tmp_root: Path, args: list[str]) -> int:
    (tmp_root / "config").mkdir(exist_ok=True)
    command, rest = args[0], args[1:]
    return dm.main([command, "--root", str(tmp_root), *rest])


def _manifest_path(tmp_root: Path) -> Path:
    return tmp_root / "data" / "d405" / "eval_manifest.csv"


def _read_rows(path: Path) -> list[dict[str, str]]:
    with open(path, "r", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def _write_depth_png(path: Path, valid_fraction: float, shape=(4, 4)) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    arr = np.zeros(shape, dtype=np.uint16)
    n_valid = int(round(valid_fraction * arr.size))
    flat = arr.reshape(-1)
    flat[:n_valid] = 1000
    img = Image.new("I;16", (shape[1], shape[0]))
    img.frombytes(arr.tobytes())
    img.save(path)


def _write_color_png(path: Path, shape=(4, 4)) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.zeros((*shape, 3), dtype=np.uint8)).save(path)


def _make_frame(tmp_root: Path, session: str, index: int, depth_valid_fraction: float = 0.5) -> Path:
    stem = f"synth_{session}_{index:06d}"
    color_path = tmp_root / "data" / "d405" / "color" / f"{stem}_color.png"
    depth_path = tmp_root / "data" / "d405" / "depth" / f"{stem}_depth.png"
    metadata_path = tmp_root / "data" / "d405" / "metadata" / f"{stem}.json"
    _write_color_png(color_path)
    _write_depth_png(depth_path, depth_valid_fraction)

    payload = {
        "schema_version": 1,
        "session": session,
        "frame_index": index,
        "timestamp_utc": "2026-09-28T16:14:45Z",
        "synthetic": True,
        "exposure_us": None,
        "files": {
            "color": f"data/d405/color/{stem}_color.png",
            "depth": f"data/d405/depth/{stem}_depth.png",
        },
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(payload), encoding="utf-8")
    return metadata_path


def _base_row(**overrides) -> dict[str, str]:
    row = {column: "" for column in dm.COLUMNS}
    row.update(
        {
            "image_id": "manual_000000",
            "timestamp_utc": "2026-09-28T16:14:45Z",
            "camera": "D405",
            "color_path": "data/d405/color/x_color.png",
            "depth_path": "data/d405/depth/x_depth.png",
            "metadata_path": "data/d405/metadata/x.json",
            "distance_cm": "20",
            "camera_angle_deg": "0",
            "surface_type": "concrete_smooth",
            "lighting_condition": "ambient",
            "crack_present": "yes",
            "crack_type": "hairline",
            "negative_class": "",
            "block": "distance",
        }
    )
    row.update(overrides)
    return row


# --- init ---


def test_init_writes_header_only_csv_with_exact_columns(tmp_path):
    exit_code = _run(tmp_path, ["init"])
    assert exit_code == 0
    path = _manifest_path(tmp_path)
    with open(path, "r", encoding="utf-8", newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        rows = list(reader)
    assert header == dm.COLUMNS
    assert rows == []


def test_init_without_force_on_existing_file_exits_2_and_leaves_file_untouched(tmp_path):
    assert _run(tmp_path, ["init"]) == 0
    path = _manifest_path(tmp_path)
    original = path.read_bytes()

    exit_code = _run(tmp_path, ["init"])
    assert exit_code == 2
    assert path.read_bytes() == original


def test_init_force_overwrites(tmp_path):
    assert _run(tmp_path, ["init"]) == 0
    assert _run(tmp_path, ["init", "--force"]) == 0


# --- scan ---


def test_scan_populates_tool_columns_and_blanks_human_columns(tmp_path):
    _make_frame(tmp_path, "cachk", 0, depth_valid_fraction=0.75)
    _make_frame(tmp_path, "cachk", 1, depth_valid_fraction=0.25)

    exit_code = _run(tmp_path, ["scan"])
    assert exit_code == 0

    rows = _read_rows(_manifest_path(tmp_path))
    assert len(rows) == 2
    by_id = {row["image_id"]: row for row in rows}
    assert set(by_id) == {"synth_cachk_000000", "synth_cachk_000001"}

    row0 = by_id["synth_cachk_000000"]
    assert row0["camera"] == "D405"
    assert row0["timestamp_utc"] == "2026-09-28T16:14:45Z"
    assert row0["color_path"] == "data/d405/color/synth_cachk_000000_color.png"
    assert row0["depth_path"] == "data/d405/depth/synth_cachk_000000_depth.png"
    assert float(row0["depth_valid_fraction"]) == pytest.approx(0.75, abs=0.01)
    assert row0["exposure_us"] == ""

    for human_column in ("distance_cm", "surface_type", "crack_present", "notes"):
        assert row0[human_column] == ""


def test_scan_merge_preserves_human_entered_values(tmp_path):
    _make_frame(tmp_path, "cachk", 0)
    assert _run(tmp_path, ["scan"]) == 0

    rows = _read_rows(_manifest_path(tmp_path))
    rows[0]["distance_cm"] = "20"
    rows[0]["crack_present"] = "yes"
    rows[0]["surface_type"] = "concrete_smooth"
    dm._write_manifest_atomic(_manifest_path(tmp_path), rows)

    # Re-scan without --merge would blank the human columns again; with --merge it must not.
    exit_code = _run(tmp_path, ["scan", "--merge"])
    assert exit_code == 0

    rows_after = _read_rows(_manifest_path(tmp_path))
    assert len(rows_after) == 1
    assert rows_after[0]["distance_cm"] == "20"
    assert rows_after[0]["crack_present"] == "yes"
    assert rows_after[0]["surface_type"] == "concrete_smooth"
    # Tool columns should still reflect the freshly scanned metadata.
    assert rows_after[0]["color_path"] == "data/d405/color/synth_cachk_000000_color.png"


def test_scan_without_merge_does_not_preserve_human_values(tmp_path):
    _make_frame(tmp_path, "cachk", 0)
    assert _run(tmp_path, ["scan"]) == 0
    rows = _read_rows(_manifest_path(tmp_path))
    rows[0]["distance_cm"] = "20"
    dm._write_manifest_atomic(_manifest_path(tmp_path), rows)

    assert _run(tmp_path, ["scan"]) == 0
    rows_after = _read_rows(_manifest_path(tmp_path))
    assert rows_after[0]["distance_cm"] == ""


# --- validate ---


def test_validate_on_header_only_manifest_exits_0(tmp_path):
    assert _run(tmp_path, ["init"]) == 0
    assert _run(tmp_path, ["validate"]) == 0


def _write_manifest_rows(tmp_path, rows):
    dm._write_manifest_atomic(_manifest_path(tmp_path), rows)


def test_validate_catches_missing_required_column(tmp_path, capsys):
    path = _manifest_path(tmp_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = [c for c in dm.COLUMNS if c != "surface_type"]
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
    exit_code = _run(tmp_path, ["validate"])
    assert exit_code == 1
    assert "surface_type" in capsys.readouterr().out


def test_validate_catches_bad_enum_value(tmp_path, capsys):
    _write_manifest_rows(tmp_path, [_base_row(camera="Nikon")])
    exit_code = _run(tmp_path, ["validate"])
    assert exit_code == 1
    assert "camera" in capsys.readouterr().out


def test_validate_catches_duplicate_image_id(tmp_path, capsys):
    _write_manifest_rows(
        tmp_path,
        [_base_row(image_id="dup_000000"), _base_row(image_id="dup_000000")],
    )
    exit_code = _run(tmp_path, ["validate"])
    assert exit_code == 1
    assert "duplicate" in capsys.readouterr().out


def test_validate_catches_missing_referenced_file(tmp_path, capsys):
    _write_manifest_rows(tmp_path, [_base_row()])
    exit_code = _run(tmp_path, ["validate"])
    assert exit_code == 1
    assert "does not exist" in capsys.readouterr().out


def test_validate_catches_negative_no_without_negative_class(tmp_path, capsys):
    row = _base_row(crack_present="no", crack_type="none", negative_class="")
    _write_manifest_rows(tmp_path, [row])
    exit_code = _run(tmp_path, ["validate"])
    assert exit_code == 1
    assert "negative_class" in capsys.readouterr().out


def test_validate_catches_distance_out_of_range(tmp_path, capsys):
    _write_manifest_rows(tmp_path, [_base_row(distance_cm="250")])
    exit_code = _run(tmp_path, ["validate"])
    assert exit_code == 1
    assert "distance_cm" in capsys.readouterr().out


def test_validate_catches_non_integer_rating(tmp_path, capsys):
    _write_manifest_rows(tmp_path, [_base_row(rating_detection="2.5")])
    exit_code = _run(tmp_path, ["validate"])
    assert exit_code == 1
    assert "rating_detection" in capsys.readouterr().out


def test_validate_passes_complete_valid_row(tmp_path):
    _make_frame(tmp_path, "cachk", 0)
    row = _base_row(
        image_id="synth_cachk_000000",
        color_path="data/d405/color/synth_cachk_000000_color.png",
        depth_path="data/d405/depth/synth_cachk_000000_depth.png",
        metadata_path="data/d405/metadata/synth_cachk_000000.json",
    )
    _write_manifest_rows(tmp_path, [row])
    assert _run(tmp_path, ["validate"]) == 0


def test_notes_with_comma_and_newline_round_trip(tmp_path):
    _make_frame(tmp_path, "cachk", 0)
    row = _base_row(
        image_id="synth_cachk_000000",
        color_path="data/d405/color/synth_cachk_000000_color.png",
        depth_path="data/d405/depth/synth_cachk_000000_depth.png",
        metadata_path="data/d405/metadata/synth_cachk_000000.json",
        notes="chalky, dusty surface\nsecond line",
    )
    _write_manifest_rows(tmp_path, [row])
    rows_after = _read_rows(_manifest_path(tmp_path))
    assert rows_after[0]["notes"] == "chalky, dusty surface\nsecond line"
    assert _run(tmp_path, ["validate"]) == 0


# --- summary ---


def test_summary_runs_on_empty_manifest_without_dividing_by_zero(tmp_path, capsys):
    assert _run(tmp_path, ["init"]) == 0
    exit_code = _run(tmp_path, ["summary"])
    assert exit_code == 0
    out = capsys.readouterr().out
    assert "total rows: 0" in out
    assert "distance: 0/25" in out


def test_summary_reports_shortfall_and_ratings(tmp_path, capsys):
    _make_frame(tmp_path, "cachk", 0)
    row = _base_row(
        image_id="synth_cachk_000000",
        color_path="data/d405/color/synth_cachk_000000_color.png",
        depth_path="data/d405/depth/synth_cachk_000000_depth.png",
        metadata_path="data/d405/metadata/synth_cachk_000000.json",
        block="distance",
        rating_detection="3",
    )
    _write_manifest_rows(tmp_path, [row])
    exit_code = _run(tmp_path, ["summary"])
    assert exit_code == 0
    out = capsys.readouterr().out
    assert "distance: 1/25 (need 24 more)" in out
    assert "rating_detection: 3.00 (n=1)" in out


# --- no pandas ---


def test_no_pandas_import():
    assert "pandas" not in sys.modules or True  # importing this module must not require pandas
    source = _MODULE_PATH.read_text(encoding="utf-8")
    assert "import pandas" not in source


def test_no_hardcoded_home_path():
    source = _MODULE_PATH.read_text(encoding="utf-8")
    marker = "/" + "home" + "/"
    assert marker not in source
