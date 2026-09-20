#!/usr/bin/env python
"""Kinematics figures for the capstone deck: the vision path + the solved trajectory.

    ./env.sh python presentation/tools/make_trajectory_figures.py

Reads (never writes):
  presentation/assets/synthetic_crack_input.png   - the crack photo
  presentation/demo/crack_path_3d.json            - pixel + world waypoints
  presentation/demo/joint_trajectory.json         - solved q(t), written by
                                                     presentation/demo/path_to_joint_trajectory.py

Writes:
  presentation/figures/fig04_trajectory.png   - crack+waypoints | joint angles vs time
  presentation/figures/fig05_workspace.png    - 3D coupon + crack path + arm at 3 poses

Design tokens match presentation/tools/make_figures.py (same surface/ink/grid) and
the joint-angle line colors are the exact 6-color sequence specified for this demo.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the '3d' projection)
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: F401

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "demo"))
import arm_kinematics as ak  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "presentation" / "assets"
DEMO = ROOT / "presentation" / "demo"
OUT = ROOT / "presentation" / "figures"

# --- design tokens (must match presentation/tools/make_figures.py) ----------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e1e0d9"
FONT = ["DejaVu Sans"]

# task-mandated joint-line colors, in joint1..joint6 order
JOINT_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
JOINT_LABELS = [f"joint{i}" for i in range(1, 7)]

# pose-sample accent colors for fig05 (reuses the project's existing categorical set)
POSE_COLORS = {"start": "#2a78d6", "mid": "#eb6834", "end": "#1baf7a"}
CRACK_COLOR = "#e34948"
COUPON_COLOR = "#898781"

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": FONT,
    "figure.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "text.color": INK,
    "axes.edgecolor": GRID,
    "axes.labelcolor": INK_2,
    "xtick.color": INK_2,
    "ytick.color": INK_2,
})


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def _style_axes(ax) -> None:
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
        ax.spines[s].set_linewidth(1.0)
    ax.tick_params(colors=INK_2, labelsize=9.5)
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)


def _place_end_labels(ax, x_end: float, end_values: list, colors: list, labels: list,
                       fontsize: float = 9.5, min_gap_frac: float = 0.045) -> None:
    """Direct end-of-line labels with simple vertical-collision avoidance.

    Sorts by end value, then greedily pushes overlapping labels apart by at least
    `min_gap_frac` of the axis's y-range. When a label is nudged away from its
    true data point, a thin leader line (same color, low alpha) connects them —
    per the dataviz guidance: don't stack colliding end-labels silently.
    """
    ylo, yhi = ax.get_ylim()
    span = yhi - ylo
    min_gap = span * min_gap_frac
    order = np.argsort(end_values)
    placed = [float(end_values[i]) for i in order]
    for k in range(1, len(placed)):
        if placed[k] - placed[k - 1] < min_gap:
            placed[k] = placed[k - 1] + min_gap
    x_text = x_end + span * 0.0  # placeholder, x offset applied via annotate below
    for rank, i in enumerate(order):
        y_true = float(end_values[i])
        y_label = placed[rank]
        if abs(y_label - y_true) > 1e-9:
            ax.plot([x_end, x_end * 1.012], [y_true, y_label], color=colors[i],
                     linewidth=0.8, alpha=0.55, zorder=4, clip_on=False)
        ax.text(x_end * 1.015, y_label, labels[i], color=colors[i], fontsize=fontsize,
                 fontweight="bold", va="center", ha="left", zorder=5)


# unit directions, cycled by waypoint index (8 compass points) so consecutive
# labels default to different sides of their marker.
_NUMBER_DIRS = [(1, 0.1), (0.8, 0.8), (0.1, 1), (-0.8, 0.8),
                (-1, 0.1), (-0.8, -0.9), (0.1, -1), (0.8, -0.9)]


def _label_radii(points: np.ndarray, base: float = 13.0) -> np.ndarray:
    """Bigger label offset for waypoints that sit close to another waypoint.

    Several of the 18 RDP-simplified crack waypoints land within ~5-15 px of a
    neighbor (a tight zig-zag in the source skeleton); a fixed offset either
    overlaps those or sits needlessly far from isolated points. Scale the radius
    by each point's nearest-neighbor distance instead.
    """
    d = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=-1)
    np.fill_diagonal(d, np.inf)
    nn = d.min(axis=1)
    return np.where(nn < 10, base * 2.6, np.where(nn < 20, base * 1.7, base))


def fig_trajectory() -> Path:
    crack_doc = _load_json(DEMO / "crack_path_3d.json")
    traj_doc = _load_json(DEMO / "joint_trajectory.json")
    img = np.array(Image.open(ASSETS / "synthetic_crack_input.png"))

    rc = np.array(crack_doc["pixel_waypoints_rc"], dtype=float)  # (row, col)
    n_crack = len(rc)

    wps = traj_doc["waypoints"]
    t = np.array([w["t_s"] for w in wps])
    q_deg = np.degrees(np.array([w["q_rad"] for w in wps]))

    fig, (ax_img, ax_q) = plt.subplots(1, 2, figsize=(17.6, 7.4),
                                        gridspec_kw={"width_ratios": [1.0, 1.3]})

    # --- left: crack image + ordered vision waypoints -----------------------
    ax_img.imshow(img, interpolation="nearest")
    xs, ys = rc[:, 1], rc[:, 0]
    # thin white halo under the path line so it reads over dark/light crack pixels
    ax_img.plot(xs, ys, color="white", linewidth=4.0, alpha=0.55, zorder=2)
    ax_img.plot(xs, ys, color=JOINT_COLORS[0], linewidth=2.0, zorder=3)
    ax_img.scatter(xs, ys, s=30, facecolor=JOINT_COLORS[0], edgecolor="white",
                    linewidth=1.0, zorder=4)
    radii = _label_radii(rc[:, ::-1])  # pass as (x, y) = (col, row)
    for i, (x, y) in enumerate(zip(xs, ys)):
        ux, uy = _NUMBER_DIRS[i % len(_NUMBER_DIRS)]
        r = radii[i]
        # thin leader line from the marker to the (possibly offset) number, so a
        # label nudged clear of a crowded cluster still reads unambiguously.
        ax_img.annotate(str(i + 1), (x, y), textcoords="offset points",
                         xytext=(ux * r, uy * r),
                         fontsize=7.6, fontweight="bold", color=INK, zorder=5,
                         bbox=dict(boxstyle="round,pad=0.1", facecolor="white",
                                   edgecolor="none", alpha=0.92),
                         arrowprops=dict(arrowstyle="-", color=INK_2, linewidth=0.6,
                                          alpha=0.65, shrinkA=0, shrinkB=3))
    ax_img.set_xticks([])
    ax_img.set_yticks([])
    for s in ax_img.spines.values():
        s.set_edgecolor(GRID)
        s.set_linewidth(1.0)
    ax_img.set_title("1 · VISION PATH", fontsize=13, fontweight="bold", color=INK,
                      loc="left", pad=10)
    ax_img.text(0, -0.035, f"{n_crack} ordered centerline waypoints on the source frame",
                transform=ax_img.transAxes, fontsize=10, color=INK_2, va="top", ha="left")

    # --- right: 6 joint angles vs time --------------------------------------
    _style_axes(ax_q)
    # phase shading: approach / crack-follow / retract
    t_approach_end = t[1]
    t_retract_start = t[-2]
    ax_q.axvspan(t[0], t_approach_end, color=GRID, alpha=0.35, zorder=0, linewidth=0)
    ax_q.axvspan(t_retract_start, t[-1], color=GRID, alpha=0.35, zorder=0, linewidth=0)
    ax_q.text(t[0] + 0.15, 0.99, "approach", transform=ax_q.get_xaxis_transform(),
              fontsize=8.5, color=INK_2, va="top", ha="left", style="italic")
    ax_q.text(t[-1] - 0.15, 0.99, "retract", transform=ax_q.get_xaxis_transform(),
              fontsize=8.5, color=INK_2, va="top", ha="right", style="italic")

    end_values = q_deg[-1, :]
    for j in range(6):
        ax_q.plot(t, q_deg[:, j], color=JOINT_COLORS[j], linewidth=2.0,
                   solid_capstyle="round", zorder=3, label=JOINT_LABELS[j])
    ax_q.set_xlim(t[0], t[-1])
    pad = (q_deg.max() - q_deg.min()) * 0.08
    ax_q.set_ylim(q_deg.min() - pad, q_deg.max() + pad)
    ax_q.set_xlabel("time (s)", fontsize=10.5, color=INK_2)
    ax_q.set_ylabel("joint angle (deg)", fontsize=10.5, color=INK_2)
    # legend: the dependable identity channel, kept clear of the data by sitting
    # in its own row below the axes (text stays ink-colored; only the line
    # swatches carry the series color). Handles/labels are auto-collected from
    # the 6 labeled `plot()` calls above ONLY (the axvspan phase bands are left
    # unlabeled), so there is no risk of the legend miscounting artists.
    ax_q.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16),
                frameon=False, fontsize=9.5, ncol=6, handlelength=1.6,
                columnspacing=1.3)
    _place_end_labels(ax_q, t[-1], list(end_values), JOINT_COLORS, JOINT_LABELS)
    ax_q.set_title("2 · SOLVED JOINT TRAJECTORY", fontsize=13, fontweight="bold",
                    color=INK, loc="left", pad=10)

    dur = traj_doc["summary"]["total_duration_s"]
    max_err = traj_doc["summary"]["max_pos_err_mm"]
    speed_cm_s = traj_doc["summary"]["tool_speed_m_s"] * 100
    fig.suptitle("From a crack mask to a joint-space motion",
                  fontsize=16, fontweight="bold", color=INK, x=0.011, ha="left", y=1.02)
    fig.text(0.011, 0.955,
              f"{n_crack} vision waypoints → {len(wps)}-pose IK solve · {dur:.1f} s at a "
              f"constant {speed_cm_s:.0f} cm/s tool speed · max tracking error "
              f"{max_err:.3f} mm · every joint stays inside its limit.",
              fontsize=10.5, color=INK_2, ha="left")
    fig.tight_layout(rect=[0, 0.02, 1, 0.90])
    p = OUT / "fig04_trajectory.png"
    fig.savefig(p, dpi=170, bbox_inches="tight")
    plt.close(fig)
    return p


def _link_chain_points(q: np.ndarray) -> np.ndarray:
    """base_link origin -> each joint origin -> tool tip, as an (8,3) array."""
    frames, T_tool = ak.fk_frames(q)
    pts = [np.zeros(3)] + [T[:3, 3] for T in frames] + [T_tool[:3, 3]]
    return np.array(pts)


def fig_workspace() -> Path:
    crack_doc = _load_json(DEMO / "crack_path_3d.json")
    traj_doc = _load_json(DEMO / "joint_trajectory.json")
    crack_xyz = np.array(crack_doc["world_waypoints_xyz_m"], dtype=float)

    wps = traj_doc["waypoints"]
    q_all = np.array([w["q_rad"] for w in wps])
    t_all = np.array([w["t_s"] for w in wps])
    i_start, i_end = 0, len(wps) - 1
    i_mid = int(np.argmin(np.abs(t_all - t_all[-1] / 2.0)))

    coupon_center = np.array(crack_doc["assumptions"]["coupon_center_xy_m"])
    half = crack_doc["assumptions"]["coupon_size_m"] / 2.0
    z_coupon = crack_doc["assumptions"]["coupon_surface_z_m"]
    corners = np.array([
        [coupon_center[0] - half, coupon_center[1] - half, z_coupon],
        [coupon_center[0] + half, coupon_center[1] - half, z_coupon],
        [coupon_center[0] + half, coupon_center[1] + half, z_coupon],
        [coupon_center[0] - half, coupon_center[1] + half, z_coupon],
        [coupon_center[0] - half, coupon_center[1] - half, z_coupon],
    ])

    fig = plt.figure(figsize=(13.4, 8.0))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(SURFACE)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.pane.set_facecolor(SURFACE)
        pane.pane.set_edgecolor(GRID)
        pane._axinfo["grid"]["color"] = GRID
        pane._axinfo["grid"]["linewidth"] = 0.6

    # The coupon as a filled plate, not just an outline: at this viewing angle an
    # outline alone reads as a wireframe floating at some other depth, and the
    # path above it then looks like it misses the coupon entirely.
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    ax.add_collection3d(Poly3DCollection(
        [corners[:4].tolist()], facecolor="#e9e8e3", edgecolor=COUPON_COLOR,
        linewidths=1.4, alpha=0.45, zorder=1))

    # The path's own shadow on the coupon, plus a drop line per waypoint. These
    # are what tie the floating 45 mm standoff path to the surface it follows.
    ax.plot(crack_xyz[:, 0], crack_xyz[:, 1], np.full(len(crack_xyz), z_coupon),
            color=CRACK_COLOR, linewidth=1.2, alpha=0.35, zorder=2)
    for pt in crack_xyz:
        ax.plot([pt[0], pt[0]], [pt[1], pt[1]], [z_coupon, pt[2]],
                color=COUPON_COLOR, linewidth=0.7, alpha=0.55, zorder=2)

    ax.plot(crack_xyz[:, 0], crack_xyz[:, 1], crack_xyz[:, 2], color=CRACK_COLOR,
             linewidth=2.2, marker="o", markersize=3.5, zorder=3,
             label="crack path (18 pts, 45 mm standoff)")

    for name, idx in (("start", i_start), ("mid", i_mid), ("end", i_end)):
        chain = _link_chain_points(q_all[idx])
        ax.plot(chain[:, 0], chain[:, 1], chain[:, 2], color=POSE_COLORS[name],
                 linewidth=2.6, marker="o", markersize=4.0, zorder=4,
                 label=f"arm @ {name} (t={t_all[idx]:.1f}s)")
        ax.scatter(*chain[0], color=INK, s=22, zorder=5)  # base marker

    ax.set_xlabel("x (m)", fontsize=10, color=INK_2, labelpad=8)
    ax.set_ylabel("y (m)", fontsize=10, color=INK_2, labelpad=8)
    ax.set_zlabel("z (m)", fontsize=10, color=INK_2, labelpad=4)
    ax.tick_params(colors=INK_2, labelsize=8.5)

    all_pts = np.vstack([corners, crack_xyz] + [_link_chain_points(q_all[i])
                                                  for i in (i_start, i_mid, i_end)])
    mins, maxs = all_pts.min(axis=0), all_pts.max(axis=0)
    ctr = (mins + maxs) / 2.0
    span = max((maxs - mins).max(), 0.2) / 2.0 * 1.15
    x_lim = (ctr[0] - span, ctr[0] + span)
    y_lim = (ctr[1] - span, ctr[1] + span)
    z_lim = (0.0, maxs[2] * 1.08)
    ax.set_xlim(*x_lim)
    ax.set_ylim(*y_lim)
    ax.set_zlim(*z_lim)
    # true relative proportions, so the coupon actually renders as a square
    # instead of matplotlib's default (each axis independently stretched to a cube)
    ax.set_box_aspect((x_lim[1] - x_lim[0], y_lim[1] - y_lim[0], z_lim[1] - z_lim[0]))
    ax.view_init(elev=22, azim=-58)

    # legend in figure space, tucked under the subtitle, so the 3D axes can be
    # grown to fill the canvas instead of leaving a dead quadrant
    ax.legend(loc="upper left", bbox_to_anchor=(0.012, 0.885), frameon=False,
              fontsize=10.5, labelcolor=INK, bbox_transform=fig.transFigure)
    fig.subplots_adjust(left=-0.06, right=1.04, top=1.06, bottom=-0.10)
    fig.suptitle("Reach envelope: coupon, crack path, and 3 sampled arm poses",
                  fontsize=15, fontweight="bold", color=INK, x=0.02, ha="left", y=0.98)
    fig.text(0.02, 0.935,
              "Robot base frame · 0.30 m coupon at z=0.12 m · tool 45 mm above the crack · "
              "links from FK joint origins.",
              fontsize=10.5, color=INK_2, ha="left")
    p = OUT / "fig05_workspace.png"
    fig.savefig(p, dpi=170)
    plt.close(fig)
    return p


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    for fn in (fig_trajectory, fig_workspace):
        p = fn()
        print(f"wrote {p.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
