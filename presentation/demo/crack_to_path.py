#!/usr/bin/env python
"""DEMO: crack mask -> ordered 3D inspection path for the reBot B601-DM arm.

    ./env.sh python presentation/demo/crack_to_path.py

This is presentation scaffolding, NOT part of the validated phase-1 pipeline.
`docs/adr/008` defers ordered path extraction and `docs/adr/002` defers
depth -> 3D projection; this script does both against a *known synthetic plane*
purely so the capstone story can be shown end to end. Nothing here is imported
by `src/crackvision/`.

Honest list of what is assumed rather than measured:
  * the inspected surface is a flat plane at a known pose (no D405 depth yet)
  * camera -> robot extrinsics are declared, not hand-eye calibrated
  * the arm base pose relative to the coupon is chosen, not surveyed

What is real: the crack mask and its skeleton come from the actual OpenCrack
nnU-Net v2 inference run recorded in `presentation/assets/`.
"""
from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "presentation" / "assets"
OUT = ROOT / "presentation" / "demo"

# --- scene definition (the declared, not-measured part) ---------------------
COUPON_SIZE_M = 0.30      # the 512x512 frame images a 30 cm x 30 cm coupon
COUPON_CENTER = (0.42, 0.0)   # (x, y) of the coupon centre in the robot base frame
COUPON_Z = 0.12           # coupon top surface height above the robot base plane
STANDOFF_M = 0.045        # tool tip hovers this far above the surface
SIMPLIFY_TOL_PX = 1.6     # Ramer-Douglas-Peucker tolerance
MAX_WAYPOINTS = 32


def skeleton_pixels(path: Path) -> np.ndarray:
    return np.argwhere(np.array(Image.open(path)) > 0)  # (row, col)


def longest_path(pix: np.ndarray) -> list[tuple[int, int]]:
    """Order the skeleton: BFS twice to get the graph diameter (the main crack).

    Branches (the small spur in this frame) fall away - an inspection pass
    follows the dominant centerline.
    """
    S = {(int(r), int(c)) for r, c in pix}
    nbr = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def bfs(start):
        prev = {start: None}
        q = deque([start])
        last = start
        while q:
            cur = q.popleft()
            last = cur
            for dr, dc in nbr:
                nxt = (cur[0] + dr, cur[1] + dc)
                if nxt in S and nxt not in prev:
                    prev[nxt] = cur
                    q.append(nxt)
        return last, prev

    seed = min(S)
    far_a, _ = bfs(seed)
    far_b, prev = bfs(far_a)
    chain, node = [], far_b
    while node is not None:
        chain.append(node)
        node = prev[node]
    return chain[::-1]


def rdp(points: list[tuple[int, int]], tol: float) -> list[tuple[int, int]]:
    """Ramer-Douglas-Peucker: keep the corners that matter, drop the rest."""
    if len(points) < 3:
        return list(points)
    p0 = np.array(points[0], dtype=float)
    p1 = np.array(points[-1], dtype=float)
    seg = p1 - p0
    norm = np.linalg.norm(seg)
    pts = np.array(points, dtype=float)
    if norm == 0:
        d = np.linalg.norm(pts - p0, axis=1)
    else:
        rel = pts - p0
        d = np.abs(seg[0] * rel[:, 1] - seg[1] * rel[:, 0]) / norm
    i = int(d.argmax())
    if d[i] <= tol:
        return [points[0], points[-1]]
    return rdp(points[:i + 1], tol)[:-1] + rdp(points[i:], tol)


def pixel_to_world(rc: tuple[int, int], shape: tuple[int, int]) -> tuple[float, float, float]:
    """(row, col) in the ORIGINAL image frame -> (x, y, z) in the robot base frame.

    The frame-invariant contract (INTERFACES.md 0.5) is what makes this a plain
    affine map: the mask was never resized, cropped or padded, so image pixels
    and coupon metres differ by one scale factor.
    """
    h, w = shape
    m_per_px = COUPON_SIZE_M / w
    r, c = rc
    # image +col -> robot -y ; image +row -> robot -x (coupon faces the base)
    x = COUPON_CENTER[0] + (h / 2.0 - r) * m_per_px
    y = COUPON_CENTER[1] + (w / 2.0 - c) * m_per_px
    return (float(x), float(y), float(COUPON_Z + STANDOFF_M))


def main() -> int:
    skel_path = ASSETS / "synthetic_crack_skeleton.png"
    shape = np.array(Image.open(skel_path)).shape
    pix = skeleton_pixels(skel_path)
    chain = longest_path(pix)

    simplified = rdp(chain, SIMPLIFY_TOL_PX)
    while len(simplified) > MAX_WAYPOINTS:
        simplified = rdp(chain, SIMPLIFY_TOL_PX * (len(simplified) / MAX_WAYPOINTS))

    world = [pixel_to_world(p, shape) for p in simplified]
    seg = np.linalg.norm(np.diff(np.array(world), axis=0), axis=1)

    doc = {
        "generated_by": "presentation/demo/crack_to_path.py",
        "status": "DEMO - not part of the validated phase-1 pipeline",
        "source_mask": "presentation/assets/synthetic_crack_prediction.png",
        "source_skeleton": str(skel_path.relative_to(ROOT)),
        "image_shape": [int(shape[0]), int(shape[1])],
        "skeleton_pixels": int(len(pix)),
        "ordered_centerline_pixels": int(len(chain)),
        "waypoints": len(world),
        "assumptions": {
            "coupon_size_m": COUPON_SIZE_M,
            "coupon_center_xy_m": list(COUPON_CENTER),
            "coupon_surface_z_m": COUPON_Z,
            "tool_standoff_m": STANDOFF_M,
            "depth_source": "declared plane (D405 depth deferred - adr/002)",
            "hand_eye": "declared transform (calibration deferred - adr/009)",
        },
        "path_length_m": round(float(seg.sum()), 4),
        "pixel_waypoints_rc": [[int(r), int(c)] for r, c in simplified],
        "world_waypoints_xyz_m": [[round(v, 5) for v in p] for p in world],
    }
    out = OUT / "crack_path_3d.json"
    out.write_text(json.dumps(doc, indent=2) + "\n")
    print(f"skeleton pixels      : {len(pix)}")
    print(f"ordered centerline   : {len(chain)} px (graph diameter)")
    print(f"waypoints after RDP  : {len(world)}")
    print(f"path length          : {doc['path_length_m']} m")
    print(f"wrote {out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
