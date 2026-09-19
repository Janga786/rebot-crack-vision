"""tests/test_prepare_inputs.py — the 3-channel RGB PNG guarantee + case_map.json. Built by TC-007.

No GPU, no camera, no network: every fixture here is generated in-test with PIL, per
docs/INTERFACES.md §3.1 and task_cards/TC-007-input-preparation.md.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME
from crackvision.prepare_inputs import main


def _sha256_bytes(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_rgb_jpeg(path: Path, size=(64, 48)) -> None:
    rng = np.random.default_rng(1)
    arr = rng.integers(0, 255, size=(size[1], size[0], 3), dtype=np.uint8)
    Image.fromarray(arr, mode="RGB").save(path, format="JPEG")


def _write_rgba_png(path: Path, size=(30, 20)) -> None:
    Image.new("RGBA", size, (10, 20, 30, 128)).save(path, format="PNG")


def _write_grayscale_png(path: Path, size=(22, 14)) -> None:
    Image.new("L", size, 77).save(path, format="PNG")


def _write_palette_png(path: Path, size=(33, 17)) -> None:
    im = Image.new("P", size)
    im.putpalette([i % 256 for i in range(768)])
    im.save(path, format="PNG")


def _write_cmyk_tiff(path: Path, size=(40, 25)) -> None:
    Image.new("CMYK", size, (10, 20, 30, 5)).save(path, format="TIFF")


def _write_16bit_png(path: Path, size=(28, 16)) -> None:
    rng = np.random.default_rng(2)
    arr = rng.integers(0, 65535, size=(size[1], size[0]), dtype=np.uint16)
    im = Image.new("I;16", size)
    im.frombytes(arr.tobytes())
    im.save(path, format="PNG")


def _write_exif_rotated_jpeg(path: Path, size=(100, 60), orientation: int = 6) -> None:
    im = Image.new("RGB", size, (200, 100, 50))
    exif = im.getexif()
    exif[0x0112] = orientation
    im.save(path, format="JPEG", exif=exif.tobytes())


def _write_bmp(path: Path, size=(18, 12)) -> None:
    Image.new("RGB", size, (5, 6, 7)).save(path, format="BMP")


def test_all_supported_modes_convert_to_rgb_png(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "rgb.jpg")
    _write_rgba_png(in_dir / "rgba.png")
    _write_grayscale_png(in_dir / "gray.png")
    _write_palette_png(in_dir / "palette.png")
    _write_cmyk_tiff(in_dir / "cmyk.tif")
    _write_16bit_png(in_dir / "sixteen.png")
    _write_bmp(in_dir / "flat.bmp")

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    out_dir = tmp_project.paths["nnunet_input"]
    outputs = sorted(out_dir.glob("*_0000.png"))
    assert len(outputs) == 7

    for out in outputs:
        with Image.open(out) as im:
            assert im.mode == "RGB"
            assert im.format == "PNG"
            arr = np.array(im)
            assert arr.dtype == np.uint8
            assert arr.shape[-1] == 3


def test_output_filenames_are_exactly_case_0000_png(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "Deck Crack 01.JPG")

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    out_dir = tmp_project.paths["nnunet_input"]
    assert (out_dir / "Deck_Crack_01_0000.png").is_file()
    assert list(out_dir.glob("*.png")) == [out_dir / "Deck_Crack_01_0000.png"]


def test_exif_rotated_fixture_is_upright(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_exif_rotated_jpeg(in_dir / "rotated.jpg", size=(100, 60), orientation=6)

    # sanity: confirm the raw file really is stored sideways before conversion
    with Image.open(in_dir / "rotated.jpg") as raw:
        assert raw.size == (100, 60)

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    out_path = tmp_project.paths["nnunet_input"] / "rotated_0000.png"
    with Image.open(out_path) as out:
        assert out.mode == "RGB"
        assert out.size == (60, 100)  # width/height swapped -> upright


def test_case_map_schema_and_relative_posix_paths(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "Deck Crack 01.JPG")
    _write_bmp(in_dir / "flat.bmp")

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    case_map_path = tmp_project.root / "data" / "case_map.json"
    assert case_map_path.is_file()
    case_map = json.loads(case_map_path.read_text(encoding="utf-8"))

    assert case_map["schema_version"] == 1
    assert case_map["root"] == str(tmp_project.root)
    assert "generated_utc" in case_map

    cases = case_map["cases"]
    assert [c["case_id"] for c in cases] == sorted(c["case_id"] for c in cases)
    assert {c["case_id"] for c in cases} == {"Deck_Crack_01", "flat"}

    for case in cases:
        assert not case["source_path"].startswith("/")
        assert not case["nnunet_input"].startswith("/")
        assert "\\" not in case["source_path"]
        source_abs = tmp_project.root / case["source_path"]
        nnunet_abs = tmp_project.root / case["nnunet_input"]
        assert source_abs.is_file()
        assert nnunet_abs.is_file()
        assert case["sha256_source"] == _sha256_bytes(source_abs)
        assert case["sha256_nnunet_input"] == _sha256_bytes(nnunet_abs)
        assert case["converted"] is True
        assert case["downscaled"] is False
        assert case["scale_factor"] == 1.0
        with Image.open(nnunet_abs) as im:
            assert (im.width, im.height) == (case["width"], case["height"])


def test_dry_run_writes_zero_files(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "rgb.jpg")

    rc = main(["--root", str(tmp_project.root), "--dry-run"])
    assert rc == EXIT_OK

    assert list(tmp_project.paths["nnunet_input"].glob("*")) == []
    assert not (tmp_project.root / "data" / "case_map.json").exists()


def test_clean_removes_stale_outputs(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "rgb.jpg")

    assert main(["--root", str(tmp_project.root)]) == EXIT_OK
    out_dir = tmp_project.paths["nnunet_input"]
    stale = out_dir / "stale_case_0000.png"
    stale.write_bytes(b"not a real png, just a stale leftover")
    assert stale.is_file()

    assert main(["--root", str(tmp_project.root), "--clean"]) == EXIT_OK
    assert not stale.exists()
    assert (out_dir / "rgb_0000.png").is_file()


def test_rerun_is_idempotent_same_hashes(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "rgb.jpg")
    _write_cmyk_tiff(in_dir / "cmyk.tif")

    assert main(["--root", str(tmp_project.root)]) == EXIT_OK
    case_map_path = tmp_project.root / "data" / "case_map.json"
    first = {c["case_id"]: c["sha256_nnunet_input"] for c in json.loads(case_map_path.read_text())["cases"]}

    assert main(["--root", str(tmp_project.root)]) == EXIT_OK
    second = {c["case_id"]: c["sha256_nnunet_input"] for c in json.loads(case_map_path.read_text())["cases"]}

    assert first == second
    assert len(first) == 2


def test_skip_existing_does_not_regenerate(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "rgb.jpg")

    assert main(["--root", str(tmp_project.root)]) == EXIT_OK
    out_path = tmp_project.paths["nnunet_input"] / "rgb_0000.png"
    mtime_before = out_path.stat().st_mtime_ns

    assert main(["--root", str(tmp_project.root), "--skip-existing"]) == EXIT_OK
    assert out_path.stat().st_mtime_ns == mtime_before

    summary = json.loads((tmp_project.paths["logs"] / "prepare_inputs_latest.json").read_text())
    assert summary["counts"].get("skipped", 0) == 1
    assert summary["counts"].get("converted", 0) == 0


def test_truncated_file_counts_as_failed_and_run_continues(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "good.jpg")
    (in_dir / "empty.png").write_bytes(b"")

    good_full = Image.new("RGB", (200, 200), (1, 2, 3))
    import io

    buf = io.BytesIO()
    good_full.save(buf, format="PNG")
    full_bytes = buf.getvalue()
    (in_dir / "truncated.png").write_bytes(full_bytes[: len(full_bytes) // 2])

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_RUNTIME

    summary = json.loads((tmp_project.paths["logs"] / "prepare_inputs_latest.json").read_text())
    assert summary["status"] == "partial"
    assert summary["exit_code"] == EXIT_RUNTIME
    assert summary["counts"]["found"] == 3
    assert summary["counts"]["failed"] == 2
    assert summary["counts"]["converted"] == 1

    case_map = json.loads((tmp_project.root / "data" / "case_map.json").read_text())
    assert {c["case_id"] for c in case_map["cases"]} == {"good"}
    assert (tmp_project.paths["nnunet_input"] / "good_0000.png").is_file()


def test_empty_input_dir_exits_precondition(tmp_project: Config) -> None:
    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_PRECONDITION


def test_missing_input_dir_exits_precondition(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    in_dir.rmdir()
    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_PRECONDITION


def test_input_originals_is_never_modified(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "rgb.jpg")
    _write_bmp(in_dir / "flat.bmp")

    before = {p.name: _sha256_bytes(p) for p in sorted(in_dir.iterdir())}

    assert main(["--root", str(tmp_project.root), "--max-side", "10"]) == EXIT_OK

    after = {p.name: _sha256_bytes(p) for p in sorted(in_dir.iterdir())}
    assert before == after


def test_max_side_downscales_and_records_scale_factor(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "big.jpg", size=(200, 100))

    assert main(["--root", str(tmp_project.root), "--max-side", "50"]) == EXIT_OK

    out_path = tmp_project.paths["nnunet_input"] / "big_0000.png"
    with Image.open(out_path) as im:
        assert max(im.size) == 50

    case_map = json.loads((tmp_project.root / "data" / "case_map.json").read_text())
    (case,) = case_map["cases"]
    assert case["downscaled"] is True
    assert case["scale_factor"] == pytest.approx(0.25)


def test_unsupported_extension_is_skipped_silently(tmp_project: Config) -> None:
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "rgb.jpg")
    (in_dir / "notes.txt").write_text("not an image")

    rc = main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    summary = json.loads((tmp_project.paths["logs"] / "prepare_inputs_latest.json").read_text())
    assert summary["counts"]["found"] == 1
