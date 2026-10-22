"""tests/test_camera_pipeline.py — check / record / snap / live with a fake camera, replayed recordings and a
stub model (no camera, no GPU); the two `gpu`+`model` tests at the end run the real network."""

from __future__ import annotations

import json
import socket
import threading
import time
import types
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from crackvision import camera_pipeline as cp
from crackvision import d405_live as dl
from crackvision import realsense_capture as rc
from crackvision.recording import CaptureFrame, RecordingWriter
from fake_rs import FakeDevice, FakeRS

REPO = Path(__file__).resolve().parents[1]


def _recording(root: Path, n: int = 6, w: int = 320, h: int = 240, depth_mm: int = 200) -> Path:
    intr = {"fx": 300.0, "fy": 300.0, "ppx": w / 2, "ppy": h / 2, "model": "brown_conrady", "coeffs": [0.0] * 5,
            "width": w, "height": h}
    with RecordingWriter(root, session=root.name, device={"serial": "S", "firmware": "5.17.0.10", "usb_type": "3.2"},
                         color={"width": w, "height": h, "fps": 30, "format": "rgb8"},
                         depth={"width": w, "height": h, "fps": 30, "format": "z16"}, color_intrinsics=intr,
                         depth_scale_m_per_unit=1e-4, extrinsics_depth_to_color=rc._synthetic_extrinsics()) as wr:
        for i in range(n):
            rgb = rc.synthetic_color_frame(w, h, i)
            if i != 3:                                    # frame 3 stays sharp, the rest are blurred
                rgb = np.asarray(Image.fromarray(rgb).resize((w // 4, h // 4)).resize((w, h)))
            wr.write_frame(CaptureFrame(i, rgb, np.full((h, w), depth_mm * 10, np.uint16), i / 30, "hardware_clock",
                                        i / 30, "hardware_clock", i + 1, i + 1, time.monotonic_ns()))
    return root


def _run(cfg, *args) -> int:
    return cp.main(["--root", str(cfg.root), "-q", *args])


@pytest.fixture
def fake_camera(monkeypatch):
    rs = FakeRS()
    monkeypatch.setattr(dl, "_import_rs", lambda: rs)
    monkeypatch.setattr(cp, "_sdk_version", lambda: "2.58.4")
    return rs


# ---- helpers -------------------------------------------------------------------------------------------------

def test_sharpness_prefers_the_sharp_frame():
    sharp = rc.synthetic_color_frame(320, 240, 0)
    blurred = np.asarray(Image.fromarray(sharp).resize((80, 60)).resize((320, 240)))
    assert cp.sharpness(sharp) > 3 * cp.sharpness(blurred)


def test_case_ids_are_sanitised():
    assert cp.safe_case_id("d405 run #1") == "d405_run_1"
    assert cp.safe_case_id("../../etc") == "etc"


def test_overlay_paints_only_masked_pixels():
    rgb = np.full((4, 4, 3), 100, np.uint8)
    mask = np.zeros((4, 4), np.uint8)
    mask[1, 2] = 1
    out = cp.overlay(rgb, mask, (255, 0, 0), 0.5)
    assert tuple(out[1, 2]) == (177, 50, 50) and tuple(out[0, 0]) == (100, 100, 100)


# ---- check ---------------------------------------------------------------------------------------------------

def test_check_passes_writes_evidence_and_a_valid_recording(tmp_project, fake_camera):
    assert _run(tmp_project, "check", "--frames", "8", "--warmup", "2", "--session", "c1") == 0
    ev = json.loads((tmp_project.root / "evidence/camera/d405_discovery.json").read_text())
    assert ev["schema"] == cp.EVIDENCE_SCHEMA and ev["verdict"] == "PASS"
    assert ev["device"]["serial"] == "123456789012" and ev["usb_3"] and ev["firmware_status"] == "up_to_date"
    assert ev["validation"]["status"] == "PASS" and ev["capture"]["depth_scale_m_per_unit"] == 1e-4
    assert ev["recording"]["path"] == "data/recordings/c1"
    assert (tmp_project.root / "data/recordings/c1/session.json").is_file()


def test_check_fails_on_usb2_and_warns_on_old_firmware(tmp_project, monkeypatch):
    rs = FakeRS(devices=[FakeDevice(usb="2.1", firmware="5.16.0.1")])
    monkeypatch.setattr(dl, "_import_rs", lambda: rs)
    monkeypatch.setattr(cp, "_sdk_version", lambda: "2.58.4")
    assert _run(tmp_project, "check", "--frames", "4", "--warmup", "0") == 1
    ev = json.loads((tmp_project.root / "evidence/camera/d405_discovery.json").read_text())
    assert ev["verdict"] == "FAIL" and any("USB 3" in p for p in ev["problems"])
    assert any("5.17.0.9" in w for w in ev["warnings"])


def test_check_without_a_device_explains_and_writes_nothing(tmp_project, monkeypatch, capsys):
    monkeypatch.setattr(dl, "_import_rs", lambda: FakeRS(devices=[]))
    monkeypatch.setattr(cp, "_sdk_version", lambda: "2.58.4")
    monkeypatch.setattr(cp, "_lsusb_sees_d405", lambda: False)
    assert _run(tmp_project, "check") == 3
    assert "Micro-B" in capsys.readouterr().out
    assert not (tmp_project.root / "evidence/camera/d405_discovery.json").exists()


# ---- record --------------------------------------------------------------------------------------------------

def test_record_from_the_camera_is_validated(tmp_project, fake_camera, capsys):
    assert _run(tmp_project, "record", "--frames", "5", "--warmup", "0", "--session", "r1") == 0
    assert "PASS" in capsys.readouterr().out
    assert len(list((tmp_project.root / "data/recordings/r1/color").glob("*.png"))) == 5
    assert _run(tmp_project, "record", "--session", "r1") == 2           # never overwrites a recording


# ---- snap ----------------------------------------------------------------------------------------------------

def test_snap_keeps_the_sharpest_frame_alone_in_its_input_folder(tmp_project, tmp_path):
    rec = _recording(tmp_path / "rec")
    assert _run(tmp_project, "snap", "--replay", str(rec), "--burst", "6", "--name", "s1", "--no-model") == 0
    case = tmp_project.root / "data/camera/s1"
    record = json.loads((case / "snap.json").read_text())
    assert record["chosen"] == 3 and record["source"] == "replay"
    assert [p.name for p in (case / "input").iterdir()] == ["s1.png"]   # prepare_inputs converts the whole folder
    assert (case / "s1_depth.png").is_file()
    assert _run(tmp_project, "snap", "--replay", str(rec), "--name", "s1", "--no-model") == 2


def test_snap_runs_the_pipeline_and_reports_scale(tmp_project, tmp_path, monkeypatch):
    rec = _recording(tmp_path / "rec", depth_mm=200)
    calls = []

    def fake_osp_main(argv):
        calls.append(argv)
        case = argv[argv.index("--case-id") + 1]
        mask = np.zeros((240, 320), np.uint8)
        mask[100:102, 20:300] = 1
        (tmp_project.root / "data/predictions").mkdir(parents=True, exist_ok=True)
        Image.fromarray(mask).save(tmp_project.root / f"data/predictions/{case}.png")
        (tmp_project.root / "data/paths").mkdir(parents=True, exist_ok=True)
        (tmp_project.root / f"data/paths/{case}_paths.json").write_text(json.dumps(
            {"components": [{"main_path": {"length_px": 280}}]}))
        return 0

    monkeypatch.setattr(cp, "_load_script", lambda name: types.SimpleNamespace(main=fake_osp_main))
    import crackvision.visualize as vis
    monkeypatch.setattr(vis, "main", lambda argv: 0)
    assert _run(tmp_project, "snap", "--replay", str(rec), "--name", "s2") == 0
    argv = calls[0]
    assert argv[argv.index("--stage") + 1] == "both" and argv[argv.index("--source") + 1].endswith("input/s2.png")
    res = json.loads((tmp_project.root / "data/camera/s2/snap.json").read_text())["result"]
    assert res["crack_pixels"] == 560 and res["crack_paths"] == 1
    assert res["distance_m"] == pytest.approx(0.2) and res["distance_from"] == "crack pixels"
    assert res["mm_per_px"] == pytest.approx(200 / 300, abs=1e-3)      # z / f
    assert res["path_lengths_mm"] == [pytest.approx(280 * 200 / 300, abs=0.1)]


# ---- live ----------------------------------------------------------------------------------------------------

def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_live_server_endpoints(tmp_project):
    state = cp._LiveState()
    server = ThreadingHTTPServer(("127.0.0.1", 0), cp._make_handler(state, tmp_project, {"device": {}}))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        assert b"/stream" in urllib.request.urlopen(base + "/").read()
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(base + "/snapshot")                  # 503: no frame yet
        rgb = rc.synthetic_color_frame(64, 48, 0)
        with state.lock:
            state.frame = CaptureFrame(0, rgb, np.full((48, 64), 900, np.uint16), 0.0, "h", 0.0, "h", 1, 1, 0)
            state.mask, state.stats = np.zeros((48, 64), np.uint8), {"frames": 1}
        assert json.loads(urllib.request.urlopen(base + "/stats").read()) == {"frames": 1}
        snap = json.loads(urllib.request.urlopen(base + "/snapshot").read())
        saved = tmp_project.root / snap["saved"]
        assert {p.name for p in saved.iterdir()} == {"color.png", "depth.png", "live_mask.png", "info.json"}
        assert "snap --image" in snap["full_pipeline"]
    finally:
        server.shutdown()


def test_live_loop_runs_frames_through_the_model(tmp_project, tmp_path, monkeypatch, capsys):
    rec = _recording(tmp_path / "rec")
    seen = []

    class StubPredictor:
        def __init__(self, cfg, tta=False):
            pass

        def predict(self, rgb):
            seen.append(rgb.shape)
            return np.zeros(rgb.shape[:2], np.uint8)

    import crackvision.live_predictor as lp
    monkeypatch.setattr(lp, "LivePredictor", StubPredictor)
    assert _run(tmp_project, "live", "--replay", str(rec), "--port", str(_free_port()), "--max-frames", "9") == 0
    assert len(seen) == 9 and seen[0] == (240, 320, 3)                   # replay loops past its 6 frames
    assert "stopped after 9 frames" in capsys.readouterr().out


# ---- the real network (GPU + downloaded checkpoint) ---------------------------------------------------------

def _model_ready() -> bool:
    try:
        import torch
        from crackvision.config import load_config
        from crackvision.inference import missing_model_files
        return torch.cuda.is_available() and not missing_model_files(load_config(root=REPO))
    except Exception:  # noqa: BLE001
        return False


def _link_model(cfg) -> None:
    """Point the throwaway root's (empty) model folder at the real downloaded checkpoint."""
    target = cfg.paths["models"]
    if target.is_dir() and not target.is_symlink():
        target.rmdir()
    target.symlink_to(REPO / "models" / "opencrack-nnunet")


@pytest.mark.gpu
@pytest.mark.model
@pytest.mark.skipif(not _model_ready(), reason="needs CUDA and the OpenCrack checkpoint")
def test_live_predictor_matches_the_batch_path(tmp_project):
    """The in-memory network (live view) gives the batch CLI's mask, pixel for pixel, with the same TTA."""
    from crackvision import inference, prepare_inputs
    from crackvision.live_predictor import LivePredictor

    _link_model(tmp_project)
    src_dir = tmp_project.root / "data" / "input_originals"
    rgb = rc.synthetic_color_frame(400, 300, 7)
    Image.fromarray(rgb).save(src_dir / "parity.png")
    root = ["--root", str(tmp_project.root), "-q"]
    assert prepare_inputs.main([*root, "--input-dir", str(src_dir)]) == 0
    assert inference.main(root) == 0
    batch = np.asarray(Image.open(tmp_project.root / "data/predictions/parity.png"))
    live = LivePredictor(tmp_project, tta=True).predict(rgb)
    assert live.shape == batch.shape and (live == batch).mean() >= 0.9999


@pytest.mark.gpu
@pytest.mark.model
@pytest.mark.skipif(not _model_ready(), reason="needs CUDA and the OpenCrack checkpoint")
def test_snap_end_to_end_on_a_replayed_recording(tmp_project, tmp_path):
    _link_model(tmp_project)
    rec = _recording(tmp_path / "rec", w=480, h=360)
    assert _run(tmp_project, "snap", "--replay", str(rec), "--name", "e2e") == 0
    res = json.loads((tmp_project.root / "data/camera/e2e/snap.json").read_text())["result"]
    assert res["crack_pixels"] > 0 and res["crack_paths"] >= 1 and res["mm_per_px"] > 0
    assert {"overlay", "comparison", "paths_overlay", "skeleton_overlay"} <= set(res["files"])
