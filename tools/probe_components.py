"""tools/probe_components.py — what the crack model emits per connected component, and what crack_filter keeps.

The measurement behind docs/perception/CRACK_SELECTION_RESEARCH.md, made repeatable. For every input it runs the
OpenCrack model in-process (crackvision.live_predictor, softmax kept), then reports:

  before  the chain as it was: argmax mask pixels and components, components that survive skeleton's 64 px filter,
          and whether crack_trace search would have called it a crack (mask sum >= 150 px);
  after   crack_filter (config/project.yaml crack_filter:): hysteresis candidates, accepted components, the best
          one's numbers, and why the others were rejected.

Inputs (default: all of them):
  prints    data/crack_targets/images/*.jpg, two ways: "resized" (848 px wide, no depth: the depth gates are skipped)
            and "rendered" (printed --print-width-m wide, flat on the table, seen by the wrist camera from --view-m
            straight above: a pinhole render like crack_trace.render_view, exact depth; D405 intrinsics from --like)
  recording a D405 recording (default the 2026-10-22 desk session, no crack in view): every frame, single-frame
            filter, plus the capture-style burst vote over the 10 middle frames

  ./env.sh python tools/probe_components.py                       # markdown tables to stdout
  ./env.sh python tools/probe_components.py --views 0.15 0.25 0.40 --json data/probe_components.json

GPU (about 60 ms per frame). Writes nothing unless --json / --overlays is given.
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

from crackvision import crack_filter as cf
from crackvision.config import load_config

REPO = Path(__file__).resolve().parents[1]
DESK = REPO / "data" / "recordings" / "cam01_20261022T174624Z"


def before_stats(prob: np.ndarray) -> dict:
    from skimage.measure import label
    from skimage.morphology import remove_small_objects
    m = prob > 0.5
    lab = label(m, connectivity=2)
    kept = remove_small_objects(m, max_size=63)
    return {"mask_px": int(m.sum()), "components": int(lab.max()),
            "after_64px": int(label(kept, connectivity=2).max()), "search_says_crack": bool(m.sum() >= 150)}


def after_stats(res: cf.FilterResult) -> dict:
    s = res.summary
    reasons: dict[str, int] = {}
    for c in res.components:
        for r in c["reject_reasons"]:
            reasons[r] = reasons.get(r, 0) + 1
    b = res.best
    return {"candidates": s["candidate_components"], "accepted": s["accepted_components"],
            "best": cf.describe(b) if b else None, "best_area_px": b["area_px"] if b else None,
            "accepted_rows": [cf.describe(c) for c in sorted(res.accepted, key=lambda c: -c["score"])],
            "joint_flagged": sum("joint_like" in c["flags"] for c in res.components),
            "reject_reasons": reasons, "gates_skipped": s["gates_skipped"]}


def resized(path: str, width: int = 848) -> np.ndarray:
    rgb = np.asarray(Image.open(path).convert("RGB"))
    if rgb.shape[1] > width:
        s = width / rgb.shape[1]
        rgb = np.asarray(Image.fromarray(rgb).resize((width, int(rgb.shape[0] * s)), Image.BILINEAR))
    return np.ascontiguousarray(rgb)


def render_flat(photo: np.ndarray, ci: dict, depth_scale: float, view_m: float, print_width_m: float
                ) -> tuple[np.ndarray, np.ndarray]:
    """The print, print_width_m wide, centred on the optical axis of a camera view_m straight above it (pinhole,
    no distortion; grey table around it, exact depth). Same model as crack_trace.render_view, without the arm (the
    arm cannot look straight down from 40 cm)."""
    W, H = int(ci["width"]), int(ci["height"])
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    x = (u - ci["ppx"]) / ci["fx"] * view_m
    y = (v - ci["ppy"]) / ci["fy"] * view_m
    ph, pw = photo.shape[:2]
    scale = pw / print_width_m
    a, b = x * scale + pw / 2, y * scale + ph / 2
    inside = (a >= 0) & (a < pw - 1) & (b >= 0) & (b < ph - 1)
    rgb = np.full((H, W, 3), 205, np.uint8)
    rgb[inside] = photo[b[inside].astype(int), a[inside].astype(int)]
    return rgb, np.full((H, W), round(view_m / depth_scale), np.uint16)


def save_overlay(out_dir: Path | None, name: str, rgb: np.ndarray, res: cf.FilterResult) -> None:
    if out_dir is None:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.hstack([rgb, cf.overlay(rgb, res)])).save(out_dir / f"{name}.png")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--images", nargs="*", default=None, help="default data/crack_targets/images/*.jpg")
    ap.add_argument("--recording", type=Path, default=DESK, help="a D405 recording (default: the desk session)")
    ap.add_argument("--views", nargs="*", type=float, default=[0.25], help="rendered view distances (m)")
    ap.add_argument("--print-width-m", type=float, default=0.20)
    ap.add_argument("--like", type=Path, default=DESK, help="recording whose intrinsics the renders use")
    ap.add_argument("--tta", action="store_true")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--overlays", type=Path, default=None, help="write original | accepted red, rejected yellow")
    args = ap.parse_args(argv)

    from crackvision.live_predictor import LivePredictor
    cfg = load_config()
    params = cf.FilterParams.from_config(cfg)
    lp = LivePredictor(cfg, tta=args.tta)
    rows: list[dict] = []
    images = args.images if args.images is not None else sorted(glob.glob(str(REPO / "data/crack_targets/images/*.jpg")))

    for f in images:
        name = Path(f).stem
        rgb = resized(f)
        prob = lp.predict_probs(rgb)
        res = cf.analyse(prob, params, gray=cf.to_gray(rgb))
        rows.append({"input": name, "mode": "resized 848 px, no depth", **before_stats(prob), **after_stats(res)})
        save_overlay(args.overlays, f"{name}_resized", rgb, res)
        if args.views:
            info = json.loads((args.like / "session.json").read_text())
            ci, ds = info["color_intrinsics"], float(info["depth_scale_m_per_unit"])
            photo = np.asarray(Image.open(f).convert("RGB"))
            for v in args.views:
                rr, dd = render_flat(photo, ci, ds, v, args.print_width_m)
                prob = lp.predict_probs(rr)
                res = cf.analyse(prob, params, gray=cf.to_gray(rr), depth_u16=dd, depth_scale=ds, intrinsics=ci)
                rows.append({"input": name, "mode": f"rendered {args.print_width_m * 100:.0f} cm print at {v * 100:.0f} cm",
                             **before_stats(prob), **after_stats(res)})
                save_overlay(args.overlays, f"{name}_rendered_{int(v * 100)}cm", rr, res)

    if args.recording and (args.recording / "session.json").is_file():
        from crackvision.recording import ReplaySource
        src = ReplaySource(args.recording)
        info = dict(src.session_info)
        ci, ds = info["color_intrinsics"], float(info["depth_scale_m_per_unit"])
        frames = list(src)
        per_frame = []
        for i, fr in enumerate(frames):
            prob = lp.predict_probs(fr.color_rgb)
            res = cf.analyse(prob, params, gray=cf.to_gray(fr.color_rgb), depth_u16=fr.depth_u16, depth_scale=ds,
                             intrinsics=ci)
            b, a = before_stats(prob), after_stats(res)
            per_frame.append({"frame": i, **b, **a})
            if i == len(frames) // 2:
                rows.append({"input": f"{args.recording.name} frame {i}", "mode": "real D405, depth", **b, **a})
                save_overlay(args.overlays, f"{args.recording.name}_{i:03d}", fr.color_rgb, res)
        n_before = sum(1 for r in per_frame if r["after_64px"] > 0 and r["search_says_crack"])
        n_after = sum(1 for r in per_frame if r["accepted"] > 0)
        mid = frames[len(frames) // 2 - 5: len(frames) // 2 + 5]
        from crackvision.camera_pipeline import sharpness
        scores = [sharpness(fr.color_rgb) for fr in mid]
        k, prob, vm, binfo = cf.burst_probabilities(lp, [fr.color_rgb for fr in mid], scores, params)
        res = cf.analyse(prob, params, gray=cf.to_gray(mid[k].color_rgb), depth_u16=mid[k].depth_u16, depth_scale=ds,
                         intrinsics=ci, vote_mask=vm)
        rows.append({"input": f"{args.recording.name} burst (10 middle frames)", "mode": "real D405, depth, vote "
                     f"{binfo['min_votes']}/{len(binfo['frames'])}", **before_stats(prob), **after_stats(res)})
        summary_rec = {"recording": args.recording.name, "frames": len(frames),
                       "frames_old_chain_calls_crack": n_before, "frames_with_accepted_component": n_after,
                       "per_frame": per_frame}
    else:
        summary_rec = None

    print("| input | mode | mask px | comps | after 64 px | search: crack? | candidates | accepted | best accepted |")
    print("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        print(f"| {r['input']} | {r['mode']} | {r['mask_px']} | {r['components']} | {r['after_64px']} | "
              f"{'yes' if r['search_says_crack'] else 'no'} | {r['candidates']} | {r['accepted']} | {r['best'] or '-'} |")
    print("\nrejections:")
    for r in rows:
        print(f"  {r['input']} [{r['mode']}]: {r['reject_reasons']}  joint-flagged {r['joint_flagged']}"
              + (f"  skipped {r['gates_skipped']}" if r["gates_skipped"] else ""))
    if summary_rec:
        print(f"\n{summary_rec['recording']}: {summary_rec['frames']} frames; old chain calls a crack in "
              f"{summary_rec['frames_old_chain_calls_crack']}, crack_filter accepts a component in "
              f"{summary_rec['frames_with_accepted_component']}")
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({"params": cf.to_doc(cf.analyse(np.zeros((4, 4), np.float32)), params)["params"],
                                         "tta": args.tta, "rows": rows, "recording": summary_rec}, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
