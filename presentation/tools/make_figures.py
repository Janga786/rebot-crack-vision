#!/usr/bin/env python
"""Regenerate the presentation figures from the committed smoke-test artefacts.

Run with the project launcher so the pinned env is used:

    ./env.sh python presentation/tools/make_figures.py

Every figure is derived from files under `presentation/assets/` (copies of the
Level-2 smoke-test outputs), so the figures are reproducible without a GPU.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "presentation" / "assets"
OUT = ROOT / "presentation" / "figures"

# Design tokens (dataviz reference palette)
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
CRACK = "#e34948"      # categorical slot 8 (red) - the crack class
CENTER = "#2a78d6"     # categorical slot 1 (blue) - the derived centerline
GOOD = "#0ca30c"       # status: good
FONT = ["DejaVu Sans"]

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": FONT,
    "figure.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
})


def load(name: str) -> np.ndarray:
    return np.array(Image.open(ASSETS / name))


def dilate(mask: np.ndarray, r: int = 2) -> np.ndarray:
    """Binary dilation with a square structuring element, numpy-only."""
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def tint(rgb: np.ndarray, mask: np.ndarray, hex_color: str, alpha: float = 0.85,
         dim: float = 1.0) -> np.ndarray:
    c = np.array([int(hex_color[i:i + 2], 16) for i in (1, 3, 5)], dtype=np.float32)
    out = rgb.astype(np.float32) * dim
    out[mask] = (1.0 - alpha) * out[mask] + alpha * c
    return np.clip(out, 0, 255).astype(np.uint8)


def panel(ax, img, title: str, subtitle: str, cmap=None) -> None:
    ax.imshow(img, cmap=cmap, interpolation="nearest")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_edgecolor("#e1e0d9")
        s.set_linewidth(1.0)
    ax.set_title(title, fontsize=13, fontweight="bold", color=INK, pad=10, loc="left")
    ax.text(0, -0.045, subtitle, transform=ax.transAxes, fontsize=10,
            color=INK_2, va="top", ha="left")


def fig_pipeline_result() -> Path:
    """4-panel: input -> mask -> overlay -> centerline."""
    orig = load("synthetic_crack_input.png")
    pred = load("synthetic_crack_prediction.png")
    skel = load("synthetic_crack_skeleton.png")

    crack = pred > 0
    skel_b = skel > 0
    n_crack, n_skel = int(crack.sum()), int(skel_b.sum())
    pct = 100.0 * crack.mean()

    overlay = tint(orig, crack, CRACK, alpha=0.85)
    center = tint(orig, dilate(skel_b, 2), CENTER, alpha=1.0, dim=0.55)

    fig, axes = plt.subplots(1, 4, figsize=(17.0, 4.9))
    panel(axes[0], orig, "1 · INPUT",
          "512×512 RGB, unmodified frame")
    panel(axes[1], (crack * 255).astype(np.uint8), "2 · MASK",
          f"binary {{0,1}} PNG · {n_crack:,} crack px ({pct:.2f} %)", cmap="gray")
    panel(axes[2], overlay, "3 · OVERLAY",
          "α-blended review render")
    panel(axes[3], center, "4 · CENTERLINE",
          f"1-px skeleton, {n_skel:,} px (thickened for print)")

    fig.suptitle("OpenCrack nnU-Net v2 · zero-shot, end-to-end on an RTX 3090",
                 fontsize=15.5, fontweight="bold", color=INK, x=0.011, ha="left", y=0.995)
    fig.text(0.011, 0.915,
             "Every stage stays in the original image frame — no resize, crop or pad — so a "
             "(row, col) here indexes the aligned depth frame later.",
             fontsize=10.5, color=INK_2, ha="left")
    fig.tight_layout(rect=[0, 0.085, 1, 0.885])
    p = OUT / "fig01_pipeline_result.png"
    fig.savefig(p, dpi=170)
    plt.close(fig)
    return p


def fig_negative_control() -> Path:
    """The blank frame: the model must return nothing."""
    blank = load("synthetic_blank_input.png")
    bpred = load("synthetic_blank_prediction.png")
    crack = load("synthetic_crack_input.png")
    cpred = load("synthetic_crack_prediction.png")

    fig, axes = plt.subplots(1, 4, figsize=(17.0, 4.9))
    panel(axes[0], blank, "NEGATIVE CONTROL · input",
          "textured concrete, no crack present")
    panel(axes[1], (bpred > 0).astype(np.uint8) * 255, "NEGATIVE CONTROL · output",
          f"{int((bpred > 0).sum())} crack px — zero false positives", cmap="gray")
    panel(axes[2], crack, "POSITIVE CASE · input",
          "same texture, one synthetic crack")
    panel(axes[3], (cpred > 0).astype(np.uint8) * 255, "POSITIVE CASE · output",
          f"{int((cpred > 0).sum()):,} crack px — crack recovered", cmap="gray")

    fig.suptitle("The smoke test asserts both directions, every run",
                 fontsize=15.5, fontweight="bold", color=INK, x=0.011, ha="left", y=0.995)
    fig.text(0.011, 0.915,
             "A pipeline that always fires looks identical to a working one on crack imagery alone. "
             "The blank frame is what makes the positive case mean something.",
             fontsize=10.5, color=INK_2, ha="left")
    fig.tight_layout(rect=[0, 0.085, 1, 0.885])
    p = OUT / "fig02_negative_control.png"
    fig.savefig(p, dpi=170)
    plt.close(fig)
    return p


def fig_scoreboard() -> Path:
    """Build status: 16 task cards, where the week landed."""
    cards = [
        ("TC-001", "Scaffolding & git hygiene", "done"),
        ("TC-002", "Python 3.11 env + env.sh", "done"),
        ("TC-003", "Environment verification", "done"),
        ("TC-004", "OpenCrack model acquisition", "done"),
        ("TC-005", "nnU-Net wiring + validation", "done"),
        ("TC-006", "Synthetic smoke test (L2)", "done"),
        ("TC-007", "Input prep + naming contract", "done"),
        ("TC-008", "Batch inference runner", "done"),
        ("TC-009", "Visualization pipeline", "done"),
        ("TC-010", "Skeletonization utility", "ready"),
        ("TC-012", "RealSense software validation", "ready"),
        ("TC-011", "One-command run_test.sh", "blocked"),
        ("TC-013", "D405 capture utility", "blocked"),
        ("TC-014", "Integration + hygiene tests", "blocked"),
        ("TC-015", "Evaluation-manifest tooling", "blocked"),
        ("TC-016", "Docs & reproducibility", "blocked"),
    ]
    style = {
        "done":    (GOOD,   "#e8f6e8", "APPROVED"),
        "ready":   ("#2a78d6", "#e6effb", "READY"),
        "blocked": (MUTED,  "#f0efec", "QUEUED"),
    }

    fig, ax = plt.subplots(figsize=(15.0, 6.4))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, len(cards) + 1.4)
    ax.axis("off")

    for i, (cid, title, state) in enumerate(cards):
        y = len(cards) - i
        fg, bg, label = style[state]
        ax.add_patch(Rectangle((0.15, y - 0.36), 9.7, 0.72, facecolor=bg,
                               edgecolor="none", zorder=1))
        ax.add_patch(Rectangle((0.15, y - 0.36), 0.07, 0.72, facecolor=fg,
                               edgecolor="none", zorder=2))
        ax.text(0.42, y, cid, fontsize=11.5, fontweight="bold", color=INK,
                va="center", ha="left", family="monospace")
        ax.text(1.62, y, title, fontsize=12, color=INK, va="center", ha="left")
        ax.text(9.62, y, label, fontsize=10, fontweight="bold", color=fg,
                va="center", ha="right")

    ax.text(0.15, len(cards) + 1.05,
            "9 of 16 task cards shipped and independently approved this week",
            fontsize=15.5, fontweight="bold", color=INK, va="center", ha="left")
    ax.text(0.15, len(cards) + 0.55,
            "Each card: written spec → implementation agent → independent adversarial review → "
            "approve or send back. Three cards came back for revision before they were approved.",
            fontsize=10.5, color=INK_2, va="center", ha="left")
    fig.tight_layout()
    p = OUT / "fig03_scoreboard.png"
    fig.savefig(p, dpi=170)
    plt.close(fig)
    return p


def main() -> int:
    if not ASSETS.exists():
        print(f"ERROR: missing {ASSETS}", file=sys.stderr)
        return 3
    OUT.mkdir(parents=True, exist_ok=True)
    for fn in (fig_pipeline_result, fig_negative_control, fig_scoreboard,
               fig_pipeline_diagram):
        p = fn()
        print(f"wrote {p.relative_to(ROOT)}")
    return 0




def fig_pipeline_diagram() -> Path:
    """The phase-1 data path, and what it feeds."""
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

    BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
    fig, ax = plt.subplots(figsize=(16.0, 5.6))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 5.6)
    ax.axis("off")

    built = [
        (0.35, "D405 / image\ncapture", "RGB frame", MUTED, False),
        (3.25, "prepare_inputs", "force 3-ch RGB,\nlossless PNG", BLUE, True),
        (6.15, "OpenCrack\nnnU-Net v2", "92.5 M params,\n256² patches, fold 0", BLUE, True),
        (9.05, "binary mask", "{0,1} PNG,\noriginal frame", BLUE, True),
        (11.95, "visualize", "mask · overlay ·\ncomparison", BLUE, True),
    ]
    y = 3.1
    w, h = 2.45, 1.45
    for x, title, sub, color, shipped in built:
        face = "#e6effb" if shipped else "#f0efec"
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.035,rounding_size=0.09",
                                    facecolor=face, edgecolor=color, linewidth=1.6))
        ax.text(x + w / 2, y + h * 0.66, title, fontsize=12.5, fontweight="bold",
                color=INK, ha="center", va="center")
        ax.text(x + w / 2, y + h * 0.27, sub, fontsize=9.5, color=INK_2,
                ha="center", va="center")
        if shipped:
            ax.text(x + w - 0.08, y + h - 0.12, "✓", fontsize=13, fontweight="bold",
                    color=GOOD, ha="right", va="top")

    for x in (2.95, 5.85, 8.75, 11.65):
        ax.add_patch(FancyArrowPatch((x - 0.15, y + h / 2), (x + 0.28, y + h / 2),
                                     arrowstyle="-|>", mutation_scale=17,
                                     color=MUTED, linewidth=1.8))

    # deferred downstream
    ax.add_patch(FancyBboxPatch((8.72, 0.75), 6.02, 1.35,
                                boxstyle="round,pad=0.035,rounding_size=0.09",
                                facecolor=SURFACE, edgecolor=MUTED, linewidth=1.4,
                                linestyle=(0, (5, 3))))
    ax.text(11.73, 1.72, "DEFERRED — next phase  (adr/002, adr/008, adr/009)",
            fontsize=10, fontweight="bold", color=MUTED, ha="center", va="center")
    ax.text(11.73, 1.18,
            "skeleton → ordered path → depth association → hand-eye → arm trajectory",
            fontsize=10.5, color=INK_2, ha="center", va="center")
    ax.add_patch(FancyArrowPatch((10.28, y - 0.1), (10.28, 2.2), arrowstyle="-|>",
                                 mutation_scale=17, color=MUTED, linewidth=1.6,
                                 linestyle=(0, (4, 3))))

    ax.text(0.35, 5.25, "One frozen contract holds the whole chain together",
            fontsize=15.5, fontweight="bold", color=INK, ha="left", va="center")
    ax.text(0.35, 4.75,
            "Nothing resizes, crops or pads. A (row, col) in the mask is the same (row, col) in the "
            "original frame — and later, in the aligned depth frame.",
            fontsize=10.5, color=INK_2, ha="left", va="center")
    ax.text(0.35, 0.30, "✓ = shipped and independently reviewed this week",
            fontsize=10, color=GOOD, fontweight="bold", ha="left", va="center")
    fig.tight_layout()
    p = OUT / "fig06_pipeline_diagram.png"
    fig.savefig(p, dpi=170)
    plt.close(fig)
    return p

if __name__ == "__main__":
    raise SystemExit(main())
