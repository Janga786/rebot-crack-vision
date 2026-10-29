"""crackvision.crack_filter: which components of the model's mask count as a crack (synthetic rasters), its wiring
into skeleton / crack_trace, and two real-model checks (the crack-free desk frame, crack print #1)."""

from __future__ import annotations

import glob
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from crackvision import crack_filter as cf  # noqa: E402
from synth_cracks import _dense_centerline, generate_wavy_crack, rasterize_centerline  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
H, W = 300, 420
INTR = {"fx": 436.0, "fy": 436.0, "ppx": W / 2, "ppy": H / 2}
SCALE = 1e-4                                    # m per depth unit (D405)


def scene(seed: int = 0):
    """Grey surface 160 (+ mild noise), crack probability 0, flat depth 0.25 m."""
    rng = np.random.default_rng(seed)
    gray = 160 + rng.normal(0, 2, (H, W))
    return gray.astype(np.float32), np.zeros((H, W), np.float32), np.full((H, W), 2500, np.uint16)


def line_mask(points, thickness):
    m = np.zeros((H, W), bool)
    rasterize_centerline(m, _dense_centerline(points), thickness)
    return m


def plant(gray, prob, mask, p=0.97, dark=60.0, fringe_p=0.6):
    """A crack-like object: confident core, a 1 px fringe at fringe_p, darker than its surroundings."""
    from scipy import ndimage
    fringe = ndimage.binary_dilation(mask) & ~mask & (prob < fringe_p)
    prob[fringe] = fringe_p
    prob[mask] = p
    if dark is not None:
        gray[mask] = dark


def wavy():
    return generate_wavy_crack(H, W, amplitude_px=18, frequency=0.03, thickness_px=3).mask


def run(prob, gray=None, depth=None, **kw):
    params = cf.FilterParams(**kw)
    return cf.analyse(prob, params, gray=gray, depth_u16=depth, depth_scale=SCALE if depth is not None else None,
                      intrinsics=INTR if depth is not None else None)


def comp_at(res, r, c):
    cid = int(res.labels[r, c])
    assert cid > 0, f"no component at {(r, c)}"
    return next(x for x in res.components if x["id"] == cid)


# ---- synthetic rasters -------------------------------------------------------------------------------------------

def test_a_crack_with_planted_strays_keeps_only_the_crack():
    gray, prob, depth = scene()
    crack = wavy()
    plant(gray, prob, crack)
    blob = np.zeros((H, W), bool)                       # texture blob without a confident core: hysteresis drops it
    blob[30:40, 30:45] = True
    prob[blob] = 0.7
    weak = np.zeros((H, W), bool)                       # a core, but a low mean probability
    weak[260:266, 300:380] = True
    prob[weak] = 0.62
    prob[262, 330:333] = 0.95
    speck = np.zeros((H, W), bool)                      # confident but tiny
    speck[270:273, 30:33] = True
    plant(gray, prob, speck)
    res = run(prob, gray, depth)
    assert res.summary["argmax_components"] == 4 and res.summary["candidate_components"] == 3
    assert [c["id"] for c in res.accepted] == [comp_at(res, *np.argwhere(crack)[0])["id"]]
    acc = res.accepted[0]
    assert acc["length_mm"] == pytest.approx(W * 0.25 / 436 * 1000, rel=0.25)   # ~ the image width at 0.573 mm/px
    assert acc["width_mm"] < 3 and acc["ridge_fraction"] == 1.0 and not acc["depth_step"]
    assert "low_confidence" in comp_at(res, 262, 340)["reject_reasons"]
    assert {"too_short_px", "too_short_mm"} <= set(comp_at(res, 271, 31)["reject_reasons"])
    assert res.accepted_mask[crack].all() and not res.accepted_mask[weak | speck | blob].any()
    assert res.best["id"] == acc["id"]


def test_a_bright_dark_edge_is_not_a_dark_ridge():
    gray, prob, depth = scene()
    gray[:, :200] = 230                                  # a lamp panel next to a darker tile: a one-sided step
    gray[:, 200:] = 110
    edge = np.zeros((H, W), bool)
    edge[40:260, 197:203] = True
    plant(gray, prob, edge, dark=None)
    crack = line_mask([(150.0, 260.0), (170.0, 300.0), (160.0, 340.0), (185.0, 400.0)], 3)
    plant(gray, prob, crack, dark=40)
    res = run(prob, gray, depth)
    e = comp_at(res, 150, 200)
    assert e["ridge_fraction"] < 0.5 and "not_a_dark_ridge" in e["reject_reasons"]
    assert comp_at(res, 170, 300)["accepted"]


def test_a_straight_even_joint_is_flagged_and_rejected_only_when_asked():
    gray, prob, depth = scene()
    joint = line_mask([(150.0, 20.0), (150.0, 400.0)], 4)
    plant(gray, prob, joint)
    res = run(prob, gray, depth)
    j = res.components[0]
    assert j["tortuosity"] == pytest.approx(1.0, abs=0.02) and j["width_cv"] < 0.25
    assert "joint_like" in j["flags"] and j["accepted"]                     # flag only by default (ADR-010)
    res2 = run(prob, gray, depth, joint_reject=True)
    assert res2.components[0]["reject_reasons"] == ["joint_like"]
    gray, prob, depth = scene()
    plant(gray, prob, wavy())
    c = run(prob, gray, depth).components[0]
    assert c["tortuosity"] > 1.05 and "joint_like" not in c["flags"]


def test_a_raised_object_and_an_object_edge_are_depth_steps():
    gray, prob, depth = scene()
    cable = line_mask([(80.0, 40.0), (95.0, 200.0), (85.0, 380.0)], 5)
    plant(gray, prob, cable)
    depth[cable] = 2500 - 80                             # 8 mm above the table
    res = run(prob, gray, depth)
    c = res.components[0]
    assert c["interior_offset_mm"] == pytest.approx(-8.0, abs=0.6) and c["plane_rms_mm"] < 0.5
    assert c["depth_step"] and c["reject_reasons"] == ["depth_step"]

    gray, prob, depth = scene()
    depth[:, 210:] = 3000                                # two surfaces 5 cm apart; a dark line along the edge
    edge = line_mask([(30.0, 208.0), (270.0, 208.0)], 3)
    plant(gray, prob, edge)
    res = run(prob, gray, depth)
    c = res.components[0]
    assert c["interior_offset_mm"] < -5 and "depth_step" in c["reject_reasons"]
    depth[res.candidate] = 0                             # no depth on the line itself: the ring alone must tell
    c = run(prob, gray, depth).components[0]
    assert c["interior_offset_mm"] is None and c["surface_tilt_deg"] > 60 and "depth_step" in c["reject_reasons"]
    gray, prob, depth = scene()                          # a line over clutter: half the ring 10 mm higher
    plant(gray, prob, edge)
    clutter = np.random.default_rng(1).random((H, W)) < 0.5
    depth[clutter] -= 100
    c = run(prob, gray, depth).components[0]
    assert c["plane_rms_mm"] > 3 and "depth_step" in c["reject_reasons"]

    gray, prob, depth = scene()                          # the same crack, flat: no step
    plant(gray, prob, wavy())
    c = run(prob, gray, depth).components[0]
    assert c["accepted"] and abs(c["interior_offset_mm"]) < 0.5 and c["surface_tilt_deg"] < 1


def test_working_distance_and_missing_depth_are_never_silently_passed():
    gray, prob, depth = scene()
    plant(gray, prob, wavy())
    far = np.full((H, W), 20000, np.uint16)              # 2 m away
    c = run(prob, gray, far).components[0]
    assert c["depth_m"] == pytest.approx(2.0) and "out_of_working_range" in c["reject_reasons"]
    nodepth = run(prob, gray, None)
    assert nodepth.components[0]["accepted"] and nodepth.summary["depth"] == "missing"
    assert {"working_range", "depth_step", "length_mm", "width_mm"} <= set(nodepth.summary["gates_skipped"])
    assert nodepth.components[0]["length_mm"] is None and nodepth.components[0]["score_units"] == "mean_p*px"
    holes = depth.copy()                                 # a depth map, but none around this component
    holes[60:240, :] = 0
    c = run(prob, gray, holes).components[0]
    assert c["reject_reasons"] == ["no_surface_depth"]
    c = run(prob, gray, holes, require_surface_depth=False).components[0]
    assert c["accepted"]
    nogray = run(prob, None, depth)
    assert "ridge" in nogray.summary["gates_skipped"] and nogray.components[0]["ridge_fraction"] is None


def test_too_wide_in_mm_is_rejected():
    gray, prob, depth = scene()
    band = line_mask([(150.0, 40.0), (165.0, 200.0), (150.0, 380.0)], 26)   # ~15 mm wide at 25 cm
    plant(gray, prob, band)
    c = run(prob, gray, depth).components[0]
    assert c["width_mm"] > 10 and "too_wide_mm" in c["reject_reasons"]


def test_hysteresis_and_vote():
    prob = np.zeros((20, 20), np.float32)
    prob[5, 2:10] = 0.6
    prob[5, 4] = 0.95                                     # seeded: the whole run survives
    prob[12, 2:10] = 0.7                                  # no seed: gone
    m = cf.hysteresis_mask(prob, 0.9, 0.5)
    assert m[5, 2:10].all() and not m[12].any()
    a, b, c = (np.zeros((4, 4), bool) for _ in range(3))
    a[0, 0] = b[0, 0] = True
    c[1, 1] = True
    v = cf.vote([a, b, c], 2)
    assert v[0, 0] and not v[1, 1]
    assert cf.pick_vote_frames([1.0, 5.0, 5.0, 3.0], 3) == [1, 2, 3]


def test_burst_probabilities_votes_out_a_flickering_speck():
    base = np.zeros((40, 60), np.float32)
    base[20, 5:55] = 0.97

    class Stub:
        def __init__(self):
            self.n = 0

        def predict_probs(self, rgb):
            p = base.copy()
            if self.n == 0:
                p[5:8, 5:8] = 0.97                         # a speck in one frame only
            self.n += 1
            return p

    rgbs = [np.zeros((40, 60, 3), np.uint8)] * 5
    k, prob, vm, info = cf.burst_probabilities(Stub(), rgbs, [1, 9, 3, 8, 2], cf.FilterParams())
    assert k == 1 and info["frames"] == [1, 3, 2] and info["min_votes"] == 2
    res = cf.analyse(prob, cf.FilterParams(min_length_px=5), vote_mask=vm)
    assert res.summary["candidate_components"] == 1 and not res.candidate[5:8, 5:8].any()


def test_case_files_round_trip(tmp_path):
    gray, prob, depth = scene()
    plant(gray, prob, wavy())
    res = run(prob, gray, depth)
    paths = cf.save_case_files(tmp_path / "data/camera/c1", "c1", res, prob, cf.FilterParams())
    doc = json.loads(paths["components_json"].read_text())
    assert doc["schema"] == cf.SCHEMA and doc["best_id"] == res.best["id"] and len(doc["components"]) == 1
    lab = cf.read_u16(paths["components_png"])
    assert (lab == res.labels).all()
    p16 = cf.read_u16(paths["crack_prob_png"])
    assert abs(p16.max() / 65535 - 0.97) < 1e-4
    table, labels = cf.find_case_table(tmp_path, "c1")
    assert table["case_id"] == "c1" and labels.shape == (H, W)
    assert cf.find_case_table(tmp_path, "nope") == (None, None)


def test_params_from_config():
    p = cf.FilterParams.from_config({"crack_filter": {"min_mean_p": 0.85, "working_range_m": [0.1, 0.4],
                                                      "unknown_key": 1}})
    assert p.min_mean_p == 0.85 and p.working_range_m == (0.1, 0.4) and p.seed_p == 0.9


# ---- wiring ------------------------------------------------------------------------------------------------------

def test_locate_crack_targets_the_given_component_not_the_whole_mask():
    from crackvision import crack_trace as ct
    mask = np.zeros((480, 848), np.uint8)
    mask[200:204, 300:500] = 1
    mask[400:440, 700:760] = 1                           # a blob elsewhere in the same mask
    depth = np.full((480, 848), 2500, np.uint16)
    intr = {"fx": 436.0, "fy": 436.0, "ppx": 424.0, "ppy": 240.0}
    comp = np.zeros_like(mask)
    comp[200:204, 300:500] = 1
    whole = ct.locate_crack(mask, depth, 1e-4, intr, np.eye(4))
    one = ct.locate_crack(mask, depth, 1e-4, intr, np.eye(4), component=comp)
    assert one["centroid_px"] == [399.5, 201.5] and whole["centroid_px"] != one["centroid_px"]
    small = np.zeros_like(mask)
    small[300:302, 300:330] = 1                           # under min_px, but the filter accepted it
    assert ct.locate_crack(small, depth, 1e-4, intr, np.eye(4), component=small) is not None
    far = np.full((480, 848), 20000, np.uint16)
    assert ct.locate_crack(mask, far, 1e-4, intr, np.eye(4), component=comp, valid_range_m=(0.07, 0.5)) is None


def test_select_crack_and_plan_trace_default_to_the_best_component(tmp_path):
    import logging
    from crackvision import crack_trace as ct
    case = "sel"
    gray, prob, depth = scene()
    a = line_mask([(60.0, 40.0), (70.0, 200.0), (60.0, 380.0)], 3)       # long crack
    b = line_mask([(220.0, 100.0), (235.0, 180.0)], 3)                   # short crack
    plant(gray, prob, a)
    plant(gray, prob, b)
    res = run(prob, gray, depth)
    assert len(res.accepted) == 2
    cf.save_case_files(tmp_path / "data/camera" / case, case, res, prob, cf.FilterParams())
    ida, idb = int(res.labels[np.argwhere(a)[0][0], np.argwhere(a)[0][1]]), int(res.labels[226, 140])
    pts = lambda m: [{"row": int(r), "col": int(c)} for r, c in np.argwhere(m)[::3]]
    (tmp_path / "data/paths").mkdir(parents=True)
    (tmp_path / f"data/paths/{case}_paths.json").write_text(json.dumps({"components": [
        {"component_id": 0, "main_path": {"dense": pts(b)}}, {"component_id": 1, "main_path": {"dense": pts(a)}}]}))
    sel = ct.select_crack(tmp_path, case)
    assert sel["paths_component"] == 1 and sel["filter_component"] == ida and ida != idb

    def comp(cid, y0):
        ys = np.linspace(-0.04, 0.04, 40)
        wps = [{"phase": "trace", "surface_point_m": [0.30 + y0, y, 0.0], "normal_base": [0, 0, 1.0],
                "sigma_along_normal_m": 0.005} for y in ys]
        return {"component_id": cid, "polylines": [{"kind": "main", "segments": [{"waypoints": wps}]}]}
    doc = {"components": [comp(0, 0.0), comp(1, 0.04)]}
    kw = dict(hover_m=0.05, approach_m=0.04, trace_speed=0.02, transit_speed=0.04, max_tilt_deg=30.0,
              min_length_m=0.01, all_paths=False, max_segments=1, smooth=1, flat=True, speed_fraction=0.6,
              log=logging.getLogger("t"))
    _, _, plan = ct.plan_trace(doc, component=1, **kw)
    assert [s["component"] for s in plan["segments"]] == [1]
    _, _, plan = ct.plan_trace(doc, **(kw | {"max_segments": 3}))
    assert len(plan["segments"]) == 2


def test_inference_command_saves_probabilities_only_when_asked(tmp_project):
    from crackvision import inference
    kw = dict(device="cuda", disable_tta=False, not_on_device=False, step_size=0.5, npp=3, nps=3,
              skip_existing=False, use_dataset_id=False)
    base = inference.build_predict_command(tmp_project, Path("i"), Path("o"), **kw)
    assert "--save_probabilities" not in base
    assert inference.build_predict_command(tmp_project, Path("i"), Path("o"), save_probabilities=True,
                                           **kw) == base + ["--save_probabilities"]


def _batch_case(cfg, case="b1"):
    gray, prob, _ = scene()
    plant(gray, prob, wavy())
    stray = np.zeros((H, W), bool)
    stray[20:30, 20:30] = True
    prob[stray] = 0.7
    prob[25, 25] = 0.95                                   # a core but a low mean p: argmax keeps it, the filter not
    src = cfg.root / "data/input_originals" / f"{case}.png"
    Image.fromarray(np.dstack([gray] * 3).clip(0, 255).astype(np.uint8)).save(src)
    (cfg.root / "data/case_map.json").write_text(json.dumps({"cases": [
        {"case_id": case, "source_path": str(src)}]}))
    Image.fromarray((prob > 0.5).astype(np.uint8)).save(cfg.paths["predictions"] / f"{case}.png")
    np.savez_compressed(cfg.paths["predictions"] / f"{case}.npz",
                        probabilities=np.stack([1 - prob, prob])[:, None].astype(np.float32))
    return case


def test_skeleton_crack_filter_is_opt_in_and_off_by_default(tmp_project):
    from crackvision import skeleton
    case = _batch_case(tmp_project)
    root = ["--root", str(tmp_project.root), "-q"]
    assert skeleton.main([*root, "--no-overlay"]) == 0
    stats = json.loads((tmp_project.paths["skeletons"] / f"{case}_skeleton_stats.json").read_text())
    assert "crack_filter" not in stats and stats["components_after"] == 2 and stats["min_component_size"] == 64
    assert not (tmp_project.paths["skeletons"] / f"{case}_components.json").exists()
    off = np.asarray(Image.open(tmp_project.paths["skeletons"] / f"{case}_skeleton.png"))
    assert skeleton.main([*root, "--no-overlay", "--crack-filter"]) == 0
    stats = json.loads((tmp_project.paths["skeletons"] / f"{case}_skeleton_stats.json").read_text())
    assert stats["components_after"] == 1 and stats["crack_filter"]["accepted_components"] == 1
    assert stats["crack_filter"]["depth"] == "missing"
    on = np.asarray(Image.open(tmp_project.paths["skeletons"] / f"{case}_skeleton.png"))
    assert on.sum() < off.sum() and not on[20:30, 20:30].any()
    table = json.loads((tmp_project.paths["skeletons"] / f"{case}_components.json").read_text())
    assert table["source"] == "batch" and len(table["components"]) == 2


def test_skeleton_crack_filter_without_probabilities_fails_the_case(tmp_project):
    from crackvision import skeleton
    case = _batch_case(tmp_project)
    (tmp_project.paths["predictions"] / f"{case}.npz").unlink()
    assert skeleton.main(["--root", str(tmp_project.root), "-q", "--no-overlay", "--crack-filter"]) == 1


# ---- the real model (GPU + checkpoint) ---------------------------------------------------------------------------

def _model_ready() -> bool:
    try:
        import torch
        from crackvision.config import load_config
        from crackvision.inference import missing_model_files
        return torch.cuda.is_available() and not missing_model_files(load_config(root=REPO))
    except Exception:  # noqa: BLE001
        return False


DESK = REPO / "data/recordings/cam01_20261022T174624Z"
PRINT01 = REPO / "data/crack_targets/images/01_crack.jpg"


@pytest.fixture(scope="module")
def predictor():
    from crackvision.config import load_config
    from crackvision.live_predictor import LivePredictor
    return LivePredictor(load_config(root=REPO))


@pytest.mark.gpu
@pytest.mark.model
@pytest.mark.skipif(not _model_ready(), reason="needs CUDA and the OpenCrack checkpoint")
@pytest.mark.skipif(not (DESK / "session.json").is_file(), reason="the 2026-10-22 desk recording is not here")
def test_the_crack_free_desk_frame_yields_no_accepted_component(predictor):
    from crackvision.config import load_config
    from crackvision.recording import ReplaySource
    params = cf.FilterParams.from_config(load_config(root=REPO))
    src = ReplaySource(DESK)
    info = src.session_info
    frames = list(src)
    for i in (len(frames) // 2, 0, len(frames) - 1):
        fr = frames[i]
        mask, prob = predictor.predict_with_probs(fr.color_rgb)
        assert (mask == predictor.predict(fr.color_rgb)).all()            # predict() is unchanged
        res = cf.analyse(prob, params, gray=cf.to_gray(fr.color_rgb), depth_u16=fr.depth_u16,
                         depth_scale=info["depth_scale_m_per_unit"], intrinsics=info["color_intrinsics"])
        assert res.summary["argmax_px"] >= 150 or i != len(frames) // 2   # the old chain called frame 15 a crack
        assert res.summary["accepted_components"] == 0, [cf.describe(c) for c in res.components]


@pytest.mark.gpu
@pytest.mark.model
@pytest.mark.skipif(not _model_ready(), reason="needs CUDA and the OpenCrack checkpoint")
@pytest.mark.skipif(not PRINT01.is_file(), reason="crack print #1 is not here (tools/make_crack_targets.py)")
def test_crack_print_01_keeps_its_one_crack(predictor):
    from crackvision.config import load_config
    params = cf.FilterParams.from_config(load_config(root=REPO))
    rgb = np.asarray(Image.open(PRINT01).convert("RGB"))
    s = 848 / rgb.shape[1]
    rgb = np.ascontiguousarray(np.asarray(Image.fromarray(rgb).resize((848, int(rgb.shape[0] * s)), Image.BILINEAR)))
    prob = predictor.predict_probs(rgb)
    res = cf.analyse(prob, params, gray=cf.to_gray(rgb))
    assert res.summary["accepted_components"] == 1
    assert res.accepted_mask.sum() >= 0.95 * (prob > 0.5).sum() and res.best["mean_p"] > 0.9
    # the same print, 20 cm wide, seen from 25 cm with depth (as the demo renders it)
    ci = json.loads((DESK / "session.json").read_text())["color_intrinsics"] if (DESK / "session.json").is_file() \
        else {"width": 848, "height": 480, "fx": 436.0, "fy": 436.0, "ppx": 424.0, "ppy": 240.0}
    from probe_components import render_flat
    rr, dd = render_flat(np.asarray(Image.open(PRINT01).convert("RGB")), ci, 1e-4, 0.25, 0.20)
    res = cf.analyse(predictor.predict_probs(rr), params, gray=cf.to_gray(rr), depth_u16=dd, depth_scale=1e-4,
                     intrinsics=ci)
    assert res.summary["accepted_components"] == 1
    assert 150 < res.best["length_mm"] < 260 and res.best["width_mm"] < 5
