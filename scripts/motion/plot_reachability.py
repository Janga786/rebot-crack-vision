"""plot_reachability.py — render a MOT-04 reachability map as one PNG figure. Built by MOT-04.5.

Reads a `crackvision.reachability_map/1` JSON (docs/INTERFACES.md §7.2) directly with `json` — it
never imports the ROS-side `crackvision_motion` modules — and, optionally, a
`crackvision.specimen_placement/1` YAML (§7.3). Draws one panel per `surface_z`: each (x, y) grid
cell is coloured by the fraction of the map's standoffs that are `reachable`, fully-reachable cells
carry contour lines of their min joint-limit margin, and the recommended footprint (+ tolerance
outline) and the alternatives are overlaid. Axes are metres in `base_link`.

Follows §0: common flags, exit codes 0/1/2/3, logs/plot_reachability_<UTC>.{log,json}.
`--dry-run` validates the inputs and writes no figure.

    ./env.sh python scripts/motion/plot_reachability.py \
        [--map data/motion/reachability_map.json] [--placement config/motion/specimen_placement.yaml] \
        [--out docs/motion/figures/reachability.png]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any

import yaml

from crackvision.config import ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)

TOOL = "plot_reachability"
MAP_SCHEMA = "crackvision.reachability_map/1"
PLACEMENT_SCHEMA = "crackvision.specimen_placement/1"
DEFAULT_MAP = "data/motion/reachability_map.json"
DEFAULT_OUT = "docs/motion/figures/reachability.png"
DPI = 120
MAX_PNG_BYTES = 400_000

# Sequential single-hue ramp (light -> dark) for 0 / partial / all standoffs reachable, plus ink
# colours for overlays; text stays in neutral ink, never a series colour.
FRACTION_COLOURS = ["#f1f3f5", "#9ec5e8", "#1f5f99"]
INK = "#1d2126"
MUTED = "#6b7280"
FOOTPRINT_COLOUR = "#c2410c"


class InputError(Exception):
    """Input file missing or incomplete (§0.2 exit 3)."""


def _zkey(z: float) -> str:
    return format(z, ".6g")


def _axis_values(axis: dict[str, Any]) -> list[float]:
    count = round((axis["max"] - axis["min"]) / axis["step"]) + 1
    return [axis["min"] + i * axis["step"] for i in range(count)]


def load_map(path: Path) -> dict[str, Any]:
    """Load and sanity-check a reachability map; InputError if absent/incomplete, ValueError if malformed."""
    if not path.is_file():
        raise InputError(f"reachability map not found: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"malformed JSON in {path}: {exc}") from exc
    if not isinstance(data, dict) or data.get("schema") != MAP_SCHEMA:
        raise ValueError(f"{path}: schema is not {MAP_SCHEMA!r}")
    if data.get("complete") is not True:
        # §7.2: consumers MUST refuse an incomplete map, treating it like a missing file.
        raise InputError(f"{path}: complete is not true (interrupted sweep); refusing to plot it")
    try:
        grid = data["grid"]["grid"]
        for axis in ("x_m", "y_m"):
            for key in ("min", "max", "step"):
                float(grid[axis][key])
        [float(z) for z in grid["surface_z_m"]]
        [float(s) for s in grid["standoffs_m"]]
        for target in data["targets"]:
            float(target["x"]), float(target["y"]), float(target["surface_z"])
            str(target["status"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{path}: malformed map structure: {exc!r}") from exc
    return data


def load_placement(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise InputError(f"placement file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"malformed YAML in {path}: {exc}") from exc
    if not isinstance(data, dict) or data.get("schema") != PLACEMENT_SCHEMA:
        raise ValueError(f"{path}: schema is not {PLACEMENT_SCHEMA!r}")
    if "feasible" not in data or "placement" not in data:
        raise ValueError(f"{path}: missing 'feasible'/'placement'")
    return data


def layer_arrays(data: dict[str, Any]) -> dict[str, Any]:
    """Per surface_z: (ny, nx) arrays of reachable-standoff fraction and min joint margin (NaN if not all reachable)."""
    import numpy as np

    grid = data["grid"]["grid"]
    xs, ys = _axis_values(grid["x_m"]), _axis_values(grid["y_m"])
    x_step, y_step = grid["x_m"]["step"], grid["y_m"]["step"]
    n_standoffs = len(grid["standoffs_m"])
    layers: dict[str, Any] = {}
    for z in grid["surface_z_m"]:
        layers[_zkey(float(z))] = {
            "z": float(z),
            "reach": np.zeros((len(ys), len(xs))),
            "margin": np.full((len(ys), len(xs)), np.inf),
        }
    for target in data["targets"]:
        layer = layers[_zkey(float(target["surface_z"]))]
        col = round((float(target["x"]) - grid["x_m"]["min"]) / x_step)
        row = round((float(target["y"]) - grid["y_m"]["min"]) / y_step)
        if target["status"] == "reachable":
            layer["reach"][row, col] += 1.0 / n_standoffs
            layer["margin"][row, col] = min(layer["margin"][row, col], float(target["min_joint_limit_margin_rad"]))
    for layer in layers.values():
        full = layer["reach"] >= 1.0 - 1e-9
        layer["margin"] = np.where(full, layer["margin"], np.nan)
    return {"xs": xs, "ys": ys, "x_step": x_step, "y_step": y_step, "layers": layers}


def _rect_corners(center: list[float], yaw: float, w: float, d: float) -> list[tuple[float, float]]:
    c, s = math.cos(yaw), math.sin(yaw)
    corners = []
    for lx, ly in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2), (-w / 2, -d / 2)):
        corners.append((center[0] + c * lx - s * ly, center[1] + s * lx + c * ly))
    return corners


def render(data: dict[str, Any], placement: dict[str, Any] | None, out_path: Path, logs_dir: Path) -> None:
    os.environ.setdefault("MPLCONFIGDIR", str(logs_dir / "mplconfig"))
    Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    arrays = layer_arrays(data)
    xs, ys = np.array(arrays["xs"]), np.array(arrays["ys"])
    hx, hy = arrays["x_step"] / 2, arrays["y_step"] / 2
    extent = (xs[0] - hx, xs[-1] + hx, ys[0] - hy, ys[-1] + hy)
    layers = sorted(arrays["layers"].values(), key=lambda layer: layer["z"])
    n = len(layers)
    ncols = min(n, 3)
    nrows = math.ceil(n / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(max(4.2 * ncols, 9.0), 4.4 * nrows + 1.2), squeeze=False)
    cmap = ListedColormap(FRACTION_COLOURS)
    norm = BoundaryNorm([-0.01, 0.01, 0.99, 1.01], cmap.N)

    winner = placement.get("placement") if placement else None
    alternatives = (placement.get("alternatives") or []) if placement else []

    for index, ax in enumerate(axes.flat):
        if index >= n:
            ax.axis("off")
            continue
        layer = layers[index]
        ax.imshow(layer["reach"], origin="lower", extent=extent, cmap=cmap, norm=norm,
                  interpolation="nearest", aspect="equal")
        margin = layer["margin"]
        if np.count_nonzero(~np.isnan(margin)) >= 4 and np.nanmax(margin) > np.nanmin(margin):
            contours = ax.contour(xs, ys, margin, levels=5, colors=INK, linewidths=0.8)
            ax.clabel(contours, fmt="%.2f", fontsize=6, colors=INK)
        ax.plot([0.0], [0.0], marker="+", markersize=10, color=INK, mew=1.5)
        ax.annotate("base_link", (0.0, 0.0), xytext=(4, 4), textcoords="offset points", fontsize=7, color=MUTED)
        for alt in alternatives:
            if abs(float(alt["surface_z_m"]) - layer["z"]) < 1e-9:
                w, d = alt["footprint_m"]
                pts = _rect_corners(alt["center_xy_m"], float(alt["yaw_rad"]), w, d)
                ax.plot(*zip(*pts), color=MUTED, linestyle="--", linewidth=1.0)
        if winner and abs(float(winner["surface_z_m"]) - layer["z"]) < 1e-9:
            w, d = winner["footprint_m"]
            tol = float(winner["tolerance_m"])
            yaw = float(winner["yaw_rad"])
            ax.plot(*zip(*_rect_corners(winner["center_xy_m"], yaw, w, d)), color=FOOTPRINT_COLOUR, linewidth=2.0)
            ax.plot(*zip(*_rect_corners(winner["center_xy_m"], yaw, w + 2 * tol, d + 2 * tol)),
                    color=FOOTPRINT_COLOUR, linewidth=1.0, linestyle=":")
        fraction = float(np.mean(layer["reach"] >= 1.0 - 1e-9))
        ax.set_title(f"surface_z = {layer['z']:.3f} m  (all standoffs reachable: {fraction:.0%})", fontsize=9, color=INK)
        ax.set_xlabel("x in base_link (m)", fontsize=8, color=INK)
        ax.set_ylabel("y in base_link (m)", fontsize=8, color=INK)
        ax.set_xlim(min(extent[0], -0.02), extent[1])
        ax.tick_params(labelsize=7, colors=MUTED)

    handles = [
        Patch(facecolor=FRACTION_COLOURS[0], edgecolor=MUTED, label="no standoff reachable"),
        Patch(facecolor=FRACTION_COLOURS[1], edgecolor=MUTED, label="some standoffs reachable"),
        Patch(facecolor=FRACTION_COLOURS[2], edgecolor=MUTED, label="all standoffs reachable"),
        Line2D([], [], color=INK, linewidth=0.8, label="min joint margin (rad), all-reachable cells"),
        Line2D([], [], color=FOOTPRINT_COLOUR, linewidth=2.0, label="recommended footprint"),
        Line2D([], [], color=FOOTPRINT_COLOUR, linewidth=1.0, linestyle=":", label="footprint + tolerance"),
        Line2D([], [], color=MUTED, linewidth=1.0, linestyle="--", label="alternatives"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=4 if ncols >= 3 else 3, fontsize=7, frameon=False)
    status = "no placement given"
    if placement is not None:
        status = "feasible placement" if placement.get("feasible") else "NO feasible placement"
    standoffs = ", ".join(f"{s:g}" for s in data["grid"]["grid"]["standoffs_m"])
    collision = data["grid"].get("surface_collision", {})
    model = collision.get("model", "slab")
    env = data["grid"].get("environment")
    env_desc = f"+ {', '.join(env['objects'])}" if env else "no workcell objects"
    fig.suptitle(
        f"B601-DM reachability (task frame {data['robot']['ik_link']}, {data['robot']['ik_solver']}, boresight down)\n"
        f"standoffs {standoffs} m - collision model {model} ({env_desc}) - {status}",
        fontsize=10, color=INK,
    )
    fig.tight_layout(rect=(0, 0.1, 1, 0.93))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=DPI)
    plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot a MOT-04 reachability map (docs/INTERFACES.md §7.2) to one PNG.")
    add_common_args(parser)
    parser.add_argument("--map", type=Path, default=None, help=f"reachability map JSON (default: {DEFAULT_MAP})")
    parser.add_argument("--placement", type=Path, default=None, help="optional specimen placement YAML to overlay")
    parser.add_argument("--out", type=Path, default=None, help=f"output PNG (default: {DEFAULT_OUT})")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        cfg = load_config(args.config, root=args.root)
    except ConfigError as exc:
        print(f"{TOOL}: config error: {exc}", file=sys.stderr)
        return EXIT_USAGE

    log = setup_logging(TOOL, cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()
    map_path = (args.map or cfg.root / DEFAULT_MAP).resolve()
    out_path = (args.out or cfg.root / DEFAULT_OUT).resolve()

    def finish(status: str, code: int) -> int:
        summary.write(cfg, TOOL, status, code)
        return code

    try:
        data = load_map(map_path)
        placement = load_placement(args.placement.resolve()) if args.placement is not None else None
    except InputError as exc:
        log.error("%s", exc)
        summary.add_error(str(exc))
        return finish("precondition", EXIT_PRECONDITION)
    except ValueError as exc:
        log.error("%s", exc)
        summary.add_error(str(exc))
        return finish("failed", EXIT_USAGE)

    summary.increment("targets", len(data["targets"]))
    summary.increment("surface_z_layers", len(data["grid"]["grid"]["surface_z_m"]))
    log.info("map %s: %d targets, counts_by_status=%s", map_path, len(data["targets"]),
             data.get("summary", {}).get("counts_by_status"))
    if placement is not None:
        log.info("placement %s: feasible=%s", args.placement, placement["feasible"])

    if args.dry_run:
        log.info("--dry-run: inputs valid; would write %s", out_path)
        return finish("ok", EXIT_OK)
    if args.skip_existing and out_path.exists():
        log.info("--skip-existing: %s exists, not regenerating", out_path)
        summary.increment("skipped")
        return finish("ok", EXIT_OK)

    try:
        render(data, placement, out_path, Path(cfg.paths["logs"]))
    except Exception as exc:  # noqa: BLE001 - any plotting failure is a §0.2 runtime failure
        log.exception("rendering failed")
        summary.add_error(f"render: {exc!r}")
        return finish("failed", EXIT_RUNTIME)

    size = out_path.stat().st_size
    log.info("wrote %s (%d bytes)", out_path, size)
    summary.increment("written")
    if size > MAX_PNG_BYTES:
        msg = f"{out_path} is {size} bytes, over the {MAX_PNG_BYTES}-byte limit"
        log.error("%s", msg)
        summary.add_error(msg)
        return finish("failed", EXIT_RUNTIME)
    return finish("ok", EXIT_OK)


if __name__ == "__main__":
    sys.exit(main())
