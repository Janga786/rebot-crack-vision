"""crackvision.crack_filter — keep the components of the crack mask that look like a crack, before skeletonising.

The model's argmax mask holds the crack and, often, crack-like background: texture blobs, the dark edge of a light
panel, specks that flicker between frames (docs/perception/CRACK_SELECTION_RESEARCH.md measures this). This module
takes the crack-class softmax (and, when there is one, the aligned depth and the grey image) and decides per
connected component:

  1. hysteresis mask: seed pixels with p > seed_p, grown into p > grow_p (drops blobs without a confident core);
  2. per-component table: area, skeleton length (px; mm with depth), width, mean/max p, elongation, tortuosity
     (main-path length / end-to-end distance), width uniformity (coefficient of variation of the local width along
     the skeleton), border contact, surface depth from a ring around the component, the ring's plane residual and
     the interior's offset from that plane (a depth step), and the fraction of skeleton pixels that sit on a dark
     ridge of the grey image (black-hat response);
  3. accepted = every enabled gate passes; the filtered mask holds only accepted components.

Fail-safe: a gate whose input is missing (no depth map, no grey image) is skipped and listed in `gates_skipped`,
never silently passed. With a depth map present, a component without enough valid surface depth around it is
rejected (`no_surface_depth`, switchable with `require_surface_depth`): the D405 returns no depth closer than 7 cm,
which is outside the working distance anyway.

Joints and seams (straight, even width) satisfy the other gates. Tortuosity and width uniformity flag them
(`joint_like`); rejecting on that flag is off by default until the D405 negative block measures the threshold.

All arrays are in the image's (row, col) frame (docs/INTERFACES.md §0.5); nothing is resampled.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Sequence

import numpy as np
from scipy import ndimage

SCHEMA = "crackvision.crack_components/1"
SQRT2 = math.sqrt(2.0)


@dataclass
class FilterParams:
    """Thresholds; defaults are config/project.yaml `crack_filter:` (justified in CRACK_SELECTION_RESEARCH.md)."""

    seed_p: float = 0.9
    grow_p: float = 0.5
    min_mean_p: float = 0.8
    min_length_px: float = 15.0
    min_length_mm: float = 10.0
    max_width_mm: float = 10.0
    working_range_m: tuple[float, float] = (0.07, 0.50)
    require_surface_depth: bool = True
    annulus_inner_px: int = 4
    annulus_outer_px: int = 14
    min_annulus_px: int = 50
    min_interior_px: int = 20
    max_plane_rms_mm: float = 3.0
    max_step_mm: float = 5.0
    max_surface_tilt_deg: float = 60.0
    ridge_min_contrast: float = 8.0
    min_ridge_fraction: float = 0.5
    max_flank_asymmetry: float | None = None
    joint_max_tortuosity: float = 1.08
    joint_max_width_cv: float = 0.30
    joint_min_width_px: float = 2.0
    joint_reject: bool = False
    border_px: int = 24
    burst_vote_frames: int = 3
    burst_vote_min: int = 2

    @classmethod
    def from_config(cls, cfg: Any) -> "FilterParams":
        """`cfg.raw['crack_filter']` (a Config or anything with a `.raw` dict, or a plain dict) over the defaults."""
        raw = cfg if isinstance(cfg, dict) else (getattr(cfg, "raw", {}) or {})
        section = dict(raw.get("crack_filter", {}) or {})
        known = {f for f in cls.__dataclass_fields__}
        kw = {k: v for k, v in section.items() if k in known}
        if "working_range_m" in kw:
            kw["working_range_m"] = tuple(float(v) for v in kw["working_range_m"])
        return cls(**kw)


def config_flag(cfg: Any, key: str, default: bool) -> bool:
    raw = cfg if isinstance(cfg, dict) else (getattr(cfg, "raw", {}) or {})
    return bool((raw.get("crack_filter", {}) or {}).get(key, default))


# ---- masks -------------------------------------------------------------------------------------------------------

def hysteresis_mask(prob: np.ndarray, seed_p: float, grow_p: float) -> np.ndarray:
    """Pixels with p > grow_p that are 8-connected to at least one pixel with p > seed_p."""
    from skimage.filters import apply_hysteresis_threshold
    return apply_hysteresis_threshold(np.asarray(prob, np.float32), grow_p, seed_p)


def vote(masks: Sequence[np.ndarray], min_votes: int) -> np.ndarray:
    """Pixels present in at least min_votes of the masks."""
    stack = np.stack([np.asarray(m, bool) for m in masks])
    return stack.sum(axis=0) >= int(min_votes)


# ---- per-component measurements ----------------------------------------------------------------------------------

def _skeleton_length(skel: np.ndarray) -> float:
    """Polyline length of a one-pixel skeleton: 4-neighbour links count 1, diagonal links sqrt(2) unless an L-corner
    already joins the two pixels."""
    s = skel.astype(bool)
    if not s.any():
        return 0.0
    right = s[:, :-1] & s[:, 1:]
    down = s[:-1, :] & s[1:, :]
    d1 = s[:-1, :-1] & s[1:, 1:] & ~(s[:-1, 1:] | s[1:, :-1])
    d2 = s[:-1, 1:] & s[1:, :-1] & ~(s[:-1, :-1] | s[1:, 1:])
    return float(right.sum() + down.sum() + SQRT2 * (d1.sum() + d2.sum()))


def _main_path(skel: np.ndarray) -> tuple[float, float]:
    """(geodesic length of the skeleton's longest shortest path, straight distance between its ends), in px.
    Two Dijkstra sweeps over the skeleton pixel graph (exact for trees, the usual heuristic with loops)."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    pts = np.argwhere(skel)
    n = len(pts)
    if n < 2:
        return 0.0, 0.0
    index = -np.ones(skel.shape, np.int64)
    index[pts[:, 0], pts[:, 1]] = np.arange(n)
    h, w = skel.shape
    rows, cols, wts = [], [], []
    for dr, dc, wt in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, SQRT2), (1, -1, SQRT2)):
        r2, c2 = pts[:, 0] + dr, pts[:, 1] + dc
        ok = (r2 >= 0) & (r2 < h) & (c2 >= 0) & (c2 < w)
        j = np.full(n, -1)
        j[ok] = index[r2[ok], c2[ok]]
        m = j >= 0
        rows.append(np.arange(n)[m])
        cols.append(j[m])
        wts.append(np.full(int(m.sum()), wt))
    r, c, wt = np.concatenate(rows), np.concatenate(cols), np.concatenate(wts)
    g = coo_matrix((np.concatenate([wt, wt]), (np.concatenate([r, c]), np.concatenate([c, r]))), shape=(n, n)).tocsr()
    d0 = dijkstra(g, indices=0)
    a = int(np.argmax(np.where(np.isfinite(d0), d0, -1)))
    da = dijkstra(g, indices=a)
    b = int(np.argmax(np.where(np.isfinite(da), da, -1)))
    return float(da[b]), float(np.hypot(*(pts[a] - pts[b])))


def _elongation(rr: np.ndarray, cc: np.ndarray) -> float:
    """Major / minor axis of the pixel set's inertia ellipse (minor floored at 1 px, as regionprops lengths)."""
    if len(rr) < 2:
        return 1.0
    cov = np.cov(np.stack([rr, cc]).astype(np.float64)) + np.eye(2) / 12.0
    ev = np.sort(np.linalg.eigvalsh(cov))[::-1]
    major, minor = 4 * math.sqrt(max(ev[0], 0)), 4 * math.sqrt(max(ev[1], 0))
    return major / max(minor, 1.0)


def _plane_fit(p: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(centroid, unit normal pointing away from the camera, residuals along it) of a least-squares plane."""
    c = p.mean(axis=0)
    _, _, vt = np.linalg.svd(p - c, full_matrices=False)
    n = vt[2]
    if n @ c < 0:
        n = -n
    return c, n, (p - c) @ n


def _backproject(rr: np.ndarray, cc: np.ndarray, z: np.ndarray, intr: dict) -> np.ndarray:
    return np.stack([(cc - intr["ppx"]) / intr["fx"] * z, (rr - intr["ppy"]) / intr["fy"] * z, z], axis=1)


def _black_hat(gray: np.ndarray, ksize: int) -> np.ndarray:
    import cv2
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    return cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, k).astype(np.float32)


def _flank_asymmetry(comp: np.ndarray, skel: np.ndarray, gray: np.ndarray, r0: int, c0: int,
                     width_px: float) -> float | None:
    """median |left flank - right flank| / median (mean flank - centre), sampled across the component at each skeleton
    pixel (the across direction from the structure tensor of the blurred mask). A crack's two sides are the same
    surface (about 0.1-0.3); a dark line between a bright and a grey surface (a lamp frame, an object edge with a
    shadow) is one-sided (larger)."""
    from scipy.ndimage import gaussian_filter, map_coordinates
    sr, sc = np.nonzero(skel)
    if len(sr) < 3:
        return None
    m = gaussian_filter(comp.astype(np.float32), max(1.0, width_px / 2))
    gy, gx = np.gradient(m)
    rho = max(2.0, width_px)
    jxx, jxy, jyy = (gaussian_filter(a, rho) for a in (gx * gx, gx * gy, gy * gy))
    th = 0.5 * np.arctan2(2 * jxy[sr, sc], jxx[sr, sc] - jyy[sr, sc])
    nx, ny = np.cos(th), np.sin(th)
    d = width_px / 2 + 3.0
    R, C = sr + r0, sc + c0
    samp = lambda rr, cc: map_coordinates(gray, [rr, cc], order=1, mode="nearest")
    left, right = samp(R + d * ny, C + d * nx), samp(R - d * ny, C - d * nx)
    centre = samp(R.astype(float), C.astype(float))
    depth = np.median(0.5 * (left + right) - centre)
    if depth <= 1.0:
        return None
    return float(np.median(np.abs(left - right)) / depth)


def to_gray(rgb: np.ndarray) -> np.ndarray:
    return (np.asarray(rgb, np.float32) @ np.array([0.299, 0.587, 0.114], np.float32)).astype(np.float32)


@dataclass
class FilterResult:
    labels: np.ndarray                      # int32 (H, W): component id (1..N) of the candidate mask, 0 elsewhere
    candidate: np.ndarray                   # bool: the hysteresis mask (and the burst vote, if any)
    accepted_mask: np.ndarray               # bool: accepted components only
    components: list[dict]                  # one row per component, ordered by id
    summary: dict = field(default_factory=dict)

    @property
    def accepted(self) -> list[dict]:
        return [c for c in self.components if c["accepted"]]

    @property
    def best(self) -> dict | None:
        acc = self.accepted
        return max(acc, key=lambda c: c["score"]) if acc else None

    def component_mask(self, cid: int) -> np.ndarray:
        return self.labels == int(cid)


def _r(x: float | None, nd: int = 3) -> float | None:
    if x is None or not np.isfinite(x):
        return None
    return round(float(x), nd)


def analyse(prob: np.ndarray, params: FilterParams | None = None, *, gray: np.ndarray | None = None,
            depth_u16: np.ndarray | None = None, depth_scale: float | None = None, intrinsics: dict | None = None,
            vote_mask: np.ndarray | None = None) -> FilterResult:
    """Score every component of the hysteresis mask of `prob` (crack-class softmax, (H, W) in [0, 1])."""
    from skimage.morphology import skeletonize
    P = params or FilterParams()
    prob = np.asarray(prob, np.float32)
    h, w = prob.shape
    argmax = prob > 0.5
    cand = hysteresis_mask(prob, P.seed_p, P.grow_p)
    if vote_mask is not None:
        cand &= np.asarray(vote_mask, bool)
    labels, n = ndimage.label(cand, structure=np.ones((3, 3), int))
    labels = labels.astype(np.int32)

    have_depth = (depth_u16 is not None and depth_scale is not None and intrinsics is not None
                  and bool(np.asarray(depth_u16).any()))
    have_gray = gray is not None
    skipped: list[str] = []
    if not have_depth:
        skipped += ["working_range", "surface_depth", "depth_step", "length_mm", "width_mm"]
    if not have_gray:
        skipped += ["ridge"]
    depth_m = np.asarray(depth_u16, np.float64) * float(depth_scale) if have_depth else None
    pad = max(P.annulus_outer_px, 1) + 2
    rows: list[dict] = []
    slices = ndimage.find_objects(labels)
    for cid, sl in enumerate(slices, start=1):
        if sl is None:
            continue
        r0, r1 = max(sl[0].start - pad, 0), min(sl[0].stop + pad, h)
        c0, c1 = max(sl[1].start - pad, 0), min(sl[1].stop + pad, w)
        comp = labels[r0:r1, c0:c1] == cid
        rr, cc = np.nonzero(comp)
        area = int(len(rr))
        p_in = prob[r0:r1, c0:c1][comp]
        skel = skeletonize(comp)
        skel_len = max(_skeleton_length(skel), 1.0)
        main_len, chord = _main_path(skel)
        tort = main_len / chord if chord >= 1.0 else None
        width_px = area / skel_len
        if skel.any():
            dt = ndimage.distance_transform_edt(comp)
            loc_w = 2.0 * dt[skel] - 1.0
            width_cv = float(loc_w.std() / loc_w.mean()) if loc_w.mean() > 0 else None
        else:
            width_cv = None
        R, C = rr + r0, cc + c0
        touches = bool(C.min() < P.border_px or R.min() < P.border_px or C.max() >= w - P.border_px
                       or R.max() >= h - P.border_px)
        row: dict[str, Any] = {
            "id": cid, "area_px": area, "centroid_px": [round(float(C.mean()), 1), round(float(R.mean()), 1)],
            "bbox_rc": [int(R.min()), int(C.min()), int(R.max()), int(C.max())],
            "skeleton_length_px": round(skel_len, 1), "main_path_px": round(main_len, 1),
            "width_px": round(width_px, 2), "mean_p": round(float(p_in.mean()), 3), "max_p": round(float(p_in.max()), 3),
            "core_px": int((p_in > P.seed_p).sum()), "elongation": round(_elongation(rr, cc), 1),
            "tortuosity": _r(tort), "width_cv": _r(width_cv), "touches_border": touches,
            "depth_m": None, "mm_per_px": None, "length_mm": None, "width_mm": None, "annulus_px": 0,
            "plane_rms_mm": None, "surface_tilt_deg": None, "interior_offset_mm": None, "depth_step": None, "ridge_fraction": None,
        }
        reasons: list[str] = []
        if row["mean_p"] < P.min_mean_p:
            reasons.append("low_confidence")
        if skel_len < P.min_length_px:
            reasons.append("too_short_px")
        if have_depth:
            dm = depth_m[r0:r1, c0:c1]
            any_crack = argmax[r0:r1, c0:c1] | cand[r0:r1, c0:c1]
            dist = ndimage.distance_transform_cdt(~comp, metric="taxicab")   # = binary_dilation iterations
            ring = (dist > P.annulus_inner_px) & (dist <= P.annulus_outer_px) & ~any_crack & (dm > 0)
            ar, ac = np.nonzero(ring)
            row["annulus_px"] = int(len(ar))
            if len(ar) >= P.min_annulus_px:
                z = dm[ar, ac]
                zmed = float(np.median(z))
                row["depth_m"] = round(zmed, 4)
                f = 0.5 * (intrinsics["fx"] + intrinsics["fy"])
                mmpx = 1000.0 * zmed / f
                row["mm_per_px"] = round(mmpx, 4)
                row["length_mm"] = round(skel_len * mmpx, 1)
                row["width_mm"] = round(width_px * mmpx, 2)
                lo, hi = P.working_range_m
                if not lo <= zmed <= hi:
                    reasons.append("out_of_working_range")
                pts = _backproject(ar + r0, ac + c0, z, intrinsics)
                if len(pts) >= 3:
                    c_pl, n_pl, res = _plane_fit(pts)
                    a = np.sort(np.abs(res))
                    keep = a[: max(3, int(math.ceil(0.95 * len(a))))]          # trim the worst 5 % (flying pixels)
                    rms = float(np.sqrt(np.mean(keep ** 2))) * 1000.0
                    row["plane_rms_mm"] = round(rms, 2)
                    tilt = math.degrees(math.acos(min(1.0, abs(float(n_pl[2])))))   # vs the optical axis
                    row["surface_tilt_deg"] = round(tilt, 1)
                    iz = dm[rr, cc]
                    okz = iz > 0
                    if okz.sum() >= P.min_interior_px:
                        ip = _backproject(rr[okz] + r0, cc[okz] + c0, iz[okz], intrinsics)
                        off = float(np.median((ip - c_pl) @ n_pl)) * 1000.0
                        row["interior_offset_mm"] = round(off, 2)
                    # a ring straddling two surfaces: a large residual, or (a straight edge between two parallel
                    # strips fits a plane exactly) a plane seen almost edge-on
                    step = (rms > P.max_plane_rms_mm or tilt > P.max_surface_tilt_deg
                            or (row["interior_offset_mm"] is not None
                                and abs(row["interior_offset_mm"]) > P.max_step_mm))
                    row["depth_step"] = bool(step)
                    if step:
                        reasons.append("depth_step")
                if row["length_mm"] < P.min_length_mm:
                    reasons.append("too_short_mm")
                if row["width_mm"] > P.max_width_mm:
                    reasons.append("too_wide_mm")
            elif P.require_surface_depth:
                reasons.append("no_surface_depth")
        if have_gray and skel.any():
            ksize = int(np.clip(2 * math.ceil(width_px) + 5, 7, 41)) | 1
            gp = ksize
            g0, g1 = max(r0 - gp, 0), min(r1 + gp, h)
            q0, q1 = max(c0 - gp, 0), min(c1 + gp, w)
            bh = _black_hat(np.ascontiguousarray(gray[g0:g1, q0:q1], np.float32), ksize)
            bh = ndimage.maximum_filter(bh, size=3)                         # the mask may sit a pixel off the line
            sr, sc = np.nonzero(skel)
            vals = bh[sr + r0 - g0, sc + c0 - q0]
            row["ridge_fraction"] = round(float((vals >= P.ridge_min_contrast).mean()), 3)
            row["ridge_contrast"] = round(float(np.median(vals)), 1)
            if row["ridge_fraction"] < P.min_ridge_fraction:
                reasons.append("not_a_dark_ridge")
            asym = _flank_asymmetry(comp, skel, gray, r0, c0, width_px)
            row["flank_asymmetry"] = _r(asym)
            if P.max_flank_asymmetry is not None and asym is not None and asym > P.max_flank_asymmetry:
                reasons.append("one_sided_edge")
        flags: list[str] = []
        if touches:
            flags.append("touches_border")
        if (tort is not None and tort <= P.joint_max_tortuosity and width_cv is not None
                and width_cv <= P.joint_max_width_cv and width_px >= P.joint_min_width_px):
            flags.append("joint_like")
            if P.joint_reject:
                reasons.append("joint_like")
        row["flags"] = flags
        row["reject_reasons"] = reasons
        row["accepted"] = not reasons
        L = row["length_mm"] if row["length_mm"] is not None else skel_len
        row["score"] = round(row["mean_p"] * L, 2)
        row["score_units"] = "mean_p*mm" if row["length_mm"] is not None else "mean_p*px"
        rows.append(row)
    accepted_ids = [r["id"] for r in rows if r["accepted"]]
    accepted_mask = np.isin(labels, accepted_ids) if accepted_ids else np.zeros_like(cand)
    arg_lab, arg_n = ndimage.label(argmax, structure=np.ones((3, 3), int))
    summary = {
        "argmax_px": int(argmax.sum()), "argmax_components": int(arg_n),
        "candidate_px": int(cand.sum()), "candidate_components": int(n),
        "accepted_components": len(accepted_ids), "accepted_px": int(accepted_mask.sum()),
        "depth": "used" if have_depth else "missing", "gray": "used" if have_gray else "missing",
        "vote": vote_mask is not None, "gates_skipped": skipped,
    }
    res = FilterResult(labels=labels, candidate=cand, accepted_mask=accepted_mask, components=rows, summary=summary)
    b = res.best
    summary["best_id"] = b["id"] if b else None
    return res


def to_doc(result: FilterResult, params: FilterParams, *, case_id: str | None = None,
           extra: dict | None = None) -> dict:
    """The JSON document written next to a case (schema crackvision.crack_components/1)."""
    p = asdict(params)
    p["working_range_m"] = list(p["working_range_m"])
    return {"schema": SCHEMA, "case_id": case_id, "params": p, "summary": result.summary,
            "best_id": result.summary.get("best_id"), "components": result.components, **(extra or {})}


# ---- burst -------------------------------------------------------------------------------------------------------

def pick_vote_frames(scores: Sequence[float], k: int) -> list[int]:
    """Indices of the k sharpest frames, sharpest first (ties: the earlier frame, as np.argmax)."""
    order = sorted(range(len(scores)), key=lambda i: (-float(scores[i]), i))
    return [int(i) for i in order[:max(1, int(k))]]


def burst_probabilities(predictor, rgbs: Sequence[np.ndarray], scores: Sequence[float], params: FilterParams
                        ) -> tuple[int, np.ndarray, np.ndarray | None, dict]:
    """Run the model on the burst_vote_frames sharpest frames. Returns (index of the sharpest frame, its crack
    probability, the vote mask (pixels with p > grow_p in at least burst_vote_min frames; None for one frame),
    info)."""
    import time
    idx = pick_vote_frames(scores, params.burst_vote_frames)
    t0 = time.monotonic()
    probs = [predictor.predict_probs(rgbs[i]) for i in idx]
    dt = time.monotonic() - t0
    vm = vote([p > params.grow_p for p in probs], min(params.burst_vote_min, len(probs))) if len(probs) > 1 else None
    info = {"frames": idx, "min_votes": min(params.burst_vote_min, len(probs)), "model_s": round(dt, 3)}
    return idx[0], probs[0], vm, info


# ---- rendering helpers -------------------------------------------------------------------------------------------

def overlay(rgb: np.ndarray, result: FilterResult, *, alpha: float = 0.55) -> np.ndarray:
    """Accepted components in red, rejected ones in yellow."""
    out = np.asarray(rgb).astype(np.float32).copy()
    rej = result.candidate & ~result.accepted_mask
    for m, col in ((result.accepted_mask, (255, 0, 0)), (rej, (255, 220, 0))):
        out[m] = (1 - alpha) * out[m] + alpha * np.array(col, np.float32)
    return out.astype(np.uint8)


def describe(c: dict) -> str:
    """One line per component, for logs and image labels."""
    parts = [f"#{c['id']}"]
    if c.get("length_mm") is not None:
        parts.append(f"L {c['length_mm']:.0f} mm")
        parts.append(f"W {c['width_mm']:.1f} mm")
    else:
        parts.append(f"L {c['skeleton_length_px']:.0f} px")
    parts.append(f"p {c['mean_p']:.2f}")
    if c.get("ridge_fraction") is not None:
        parts.append(f"ridge {c['ridge_fraction']:.2f}")
    if c.get("flank_asymmetry") is not None:
        parts.append(f"asym {c['flank_asymmetry']:.2f}")
    if c.get("tortuosity") is not None:
        parts.append(f"tort {c['tortuosity']:.2f}")
    if c.get("width_cv") is not None:
        parts.append(f"wcv {c['width_cv']:.2f}")
    if c.get("depth_m") is not None:
        parts.append(f"z {100 * c['depth_m']:.0f} cm")
    if "joint_like" in c.get("flags", []):
        parts.append("JOINT?")
    if not c["accepted"]:
        parts.append("rejected: " + ",".join(c["reject_reasons"]))
    return " ".join(parts)


def save_case_files(out_dir, case_id: str, result: FilterResult, prob: np.ndarray, params: FilterParams,
                    extra: dict | None = None) -> dict:
    """<case>_components.json (the table), <case>_components.png (uint16 component ids), <case>_crack_prob.png
    (uint16, p * 65535) and <case>_argmax.png ({0, 255}) in out_dir. Returns {kind: path}."""
    import json
    from pathlib import Path
    from PIL import Image
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {"components_json": out_dir / f"{case_id}_components.json",
             "components_png": out_dir / f"{case_id}_components.png",
             "crack_prob_png": out_dir / f"{case_id}_crack_prob.png",
             "argmax_png": out_dir / f"{case_id}_argmax.png"}
    _write_u16(result.labels.astype(np.uint16), paths["components_png"])
    _write_u16(np.round(np.clip(prob, 0, 1) * 65535).astype(np.uint16), paths["crack_prob_png"])
    Image.fromarray(((np.asarray(prob) > 0.5) * 255).astype(np.uint8)).save(paths["argmax_png"])
    doc = to_doc(result, params, case_id=case_id, extra=extra)
    paths["components_json"].write_text(json.dumps(doc, indent=1) + "\n")
    return paths


def _write_u16(arr: np.ndarray, path) -> None:
    from PIL import Image
    img = Image.new("I;16", (arr.shape[1], arr.shape[0]))
    img.frombytes(np.ascontiguousarray(arr, dtype="<u2").tobytes())
    img.save(path)


def read_u16(path) -> np.ndarray:
    from PIL import Image
    with Image.open(path) as img:
        return np.array(img.convert("I;16"), dtype=np.uint16)


def find_case_table(root, case_id: str):
    """(components doc, component-id raster) for a case, or (None, None): the live path writes them under
    data/camera/<case>/, the batch path (skeleton --crack-filter) under data/skeletons/."""
    import json
    from pathlib import Path
    for d in (Path(root) / "data" / "camera" / case_id, Path(root) / "data" / "skeletons"):
        j, p = d / f"{case_id}_components.json", d / f"{case_id}_components.png"
        if j.is_file():
            doc = json.loads(j.read_text())
            return doc, (read_u16(p) if p.is_file() else None)
    return None, None
