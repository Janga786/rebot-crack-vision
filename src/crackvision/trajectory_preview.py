"""crackvision.trajectory_preview — `crackvision.trajectory_preview/1` CLI (docs/INTERFACES.md §18,
ADR-020).

Wraps `crackvision.preview_core`'s pure binding/summary/manifest logic with deterministic, headless
matplotlib rendering of three PNGs (joints, tool path, tool speed) plus the `preview.json` manifest.
Never imports `crackvision_motion`/`rclpy`, never opens a window (`matplotlib.use("Agg")` is forced
before `pyplot` is ever imported, below).
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def _ensure_writable_mplconfigdir() -> None:
    """The sandboxed host's default `~/.config/matplotlib` is not writable (ADR-020 rendering
    note); point `MPLCONFIGDIR` at a private writable temp dir before matplotlib is imported at
    all, rather than letting matplotlib fall back to its own (noisy, non-deterministic-path) temp
    dir on every run."""
    if os.environ.get("MPLCONFIGDIR"):
        return
    default_dir = Path.home() / ".config" / "matplotlib"
    try:
        default_dir.mkdir(parents=True, exist_ok=True)
        probe = default_dir / ".write_probe"
        probe.touch()
        probe.unlink()
        return
    except OSError:
        pass
    fallback = Path(tempfile.gettempdir()) / "crackvision_mplconfig"
    fallback.mkdir(parents=True, exist_ok=True)
    os.environ["MPLCONFIGDIR"] = str(fallback)


_ensure_writable_mplconfigdir()

import matplotlib  # noqa: E402

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402

from crackvision.config import Config, ConfigError, add_common_args, load_config  # noqa: E402
from crackvision.kinematics import KinematicsError, load_chain, load_end_effector  # noqa: E402
from crackvision.logging_setup import (  # noqa: E402
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)
from crackvision.preview_core import (  # noqa: E402
    PreviewRefusal,
    PreviewUsageError,
    bind_inputs,
    build_manifest,
    compute_summary,
    file_sha256,
    tool_tip_path,
    validate_manifest,
)

TOOL = "trajectory_preview"

JOINTS_FIGSIZE = (12.0, 18.0)
JOINTS_DPI = 100
TOOLPATH_FIGSIZE = (15.0, 5.0)
TOOLPATH_DPI = 100
TOOLSPEED_FIGSIZE = (10.0, 4.0)
TOOLSPEED_DPI = 100

_SAVEFIG_KWARGS: dict[str, Any] = {"metadata": {"Software": "", "Creation Time": ""}}

_LEG_COLORS: dict[str, str] = {
    "transit": "#808080",
    "approach": "#1f77b4",
    "trace": "#d62728",
    "retract": "#2ca02c",
}
_LEG_SPAN_ALPHA = 0.12
_CRACK_COLOR = "#000000"
_SCENE_BOX_COLOR = "#8b5a2b"

_PROJECTIONS = (("xy", 0, 1, "X [m]", "Y [m]"), ("xz", 0, 2, "X [m]", "Z [m]"), ("yz", 1, 2, "Y [m]", "Z [m]"))


class UsageError(Exception):
    """A CLI/config input (other than the §18.1 four preview inputs) could not be parsed (exit 2)."""


def _iso_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _load_yaml(path: Path) -> dict:
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise UsageError(f"cannot open {path}: {exc}") from exc
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise UsageError(f"malformed YAML in {path}: {exc}") from exc
    if not isinstance(doc, dict):
        raise UsageError(f"{path} must contain a mapping at the top level")
    return doc


def _load_json(path: Path) -> Any:
    import json

    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise UsageError(f"cannot open {path}: {exc}") from exc
    try:
        return json.loads(text)
    except json.JSONDecodeError as exc:
        raise UsageError(f"malformed JSON in {path}: {exc}") from exc


# ---------------------------------------------------------------------------
# Leg geometry helpers
# ---------------------------------------------------------------------------


def _box_corners_world(obj: dict) -> np.ndarray:
    """8 corners of a `shape: box` scene object (§scene.yaml), in `base_link`."""
    from crackvision.kinematics import transform_from_xyz_rpy

    dx, dy, dz = [float(v) / 2.0 for v in obj["dimensions_m"]]
    local_corners = np.array(
        [[sx * dx, sy * dy, sz * dz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    )
    pose = obj["pose"]
    T = transform_from_xyz_rpy(pose["position_m"], pose["rpy_rad"])
    world = (T[:3, :3] @ local_corners.T).T + T[:3, 3]
    return world


_BOX_EDGES = [
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
    (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
]


def _load_scene_boxes(scene_path: Path) -> tuple[list[np.ndarray], list[str]]:
    """Returns `[(8, 3) corner arrays]` for every `shape: box` object in `scene.yaml`, plus one
    warning string per skipped non-box object."""
    warnings: list[str] = []
    try:
        doc = _load_yaml(scene_path)
    except UsageError as exc:
        return [], [str(exc)]
    boxes = []
    for obj in doc.get("objects") or []:
        if obj.get("shape") != "box":
            warnings.append(f"scene object {obj.get('id')!r} has non-box shape {obj.get('shape')!r}, skipped")
            continue
        boxes.append(_box_corners_world(obj))
    return boxes, warnings


def _load_crack_polylines(
    traj_doc: dict, root: Path
) -> tuple[list[np.ndarray], list[str]]:
    """`[(M, 3) valid p_base_m arrays]`, one per `paths3d` component, when `source.paths3d` is
    non-null and the current file's sha256 matches; otherwise `([], [one warning])`."""
    paths3d_ref = (traj_doc.get("source") or {}).get("paths3d")
    if not paths3d_ref:
        return [], []

    path = Path(paths3d_ref["path"])
    if not path.is_absolute():
        path = root / path
    if not path.is_file():
        return [], [f"paths3d file {paths3d_ref['path']} not found, crack polylines not drawn"]
    if file_sha256(path) != paths3d_ref["sha256"]:
        return [], [f"paths3d file {paths3d_ref['path']} sha256 mismatch, crack polylines not drawn"]

    try:
        doc = _load_json(path)
    except UsageError as exc:
        return [], [f"paths3d file could not be read: {exc}"]

    polylines: list[np.ndarray] = []
    for comp in doc.get("components") or []:
        for polyline in comp.get("polylines") or []:
            pts = [
                p["p_base_m"]
                for p in polyline.get("points") or []
                if p.get("valid") and p.get("p_base_m") is not None
            ]
            if pts:
                polylines.append(np.array(pts, dtype=float))
    return polylines, []


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------


def _apply_failure_banner(fig, all_pass: bool, failed_checks: list[str]) -> None:
    if all_pass:
        return
    fig.suptitle(
        "CHECK REPORT FAILED: " + ", ".join(failed_checks),
        color="red",
        fontweight="bold",
    )


def render_joints_png(
    out_path: Path, traj: dict, limits: dict, legs: list[dict], all_pass: bool, failed_checks: list[str]
) -> None:
    points = traj["points"]
    n = len(points)
    joint_names = traj["joint_names"]
    t = [float(p["t_s"]) for p in points]

    fig, axes = plt.subplots(len(joint_names), 3, figsize=JOINTS_FIGSIZE, dpi=JOINTS_DPI)
    _apply_failure_banner(fig, all_pass, failed_checks)

    for j in range(len(joint_names)):
        jcfg = limits["joints"][joint_names[j]]
        lower, upper = float(jcfg["lower"]), float(jcfg["upper"])
        v_limit, a_limit = float(jcfg["velocity"]), float(jcfg["acceleration"])
        q = [float(p["positions"][j]) for p in points]
        v = [(q[i + 1] - q[i]) / (t[i + 1] - t[i]) for i in range(n - 1)]
        a = []
        for i in range(1, n - 1):
            denom = ((t[i] - t[i - 1]) + (t[i + 1] - t[i])) / 2.0
            a.append((v[i] - v[i - 1]) / denom)

        ax_q, ax_v, ax_a = axes[j]
        for ax in (ax_q, ax_v, ax_a):
            for leg in legs:
                first, last = leg["point_range"]
                ax.axvspan(
                    t[first], t[last], color=_LEG_COLORS.get(leg["kind"], "#808080"), alpha=_LEG_SPAN_ALPHA, lw=0
                )

        ax_q.plot(t, q, color="black", lw=1.0)
        ax_q.axhline(lower, color="gray", ls="--", lw=0.8)
        ax_q.axhline(upper, color="gray", ls="--", lw=0.8)
        ax_q.set_ylabel(f"{joint_names[j]}\nq [rad]")

        ax_v.plot(t[:-1], v, color="black", lw=1.0)
        ax_v.axhline(v_limit, color="gray", ls="--", lw=0.8)
        ax_v.axhline(-v_limit, color="gray", ls="--", lw=0.8)
        ax_v.set_ylabel("v [rad/s]")

        ax_a.plot(t[1:-1], a, color="black", lw=1.0)
        ax_a.axhline(a_limit, color="gray", ls="--", lw=0.8)
        ax_a.axhline(-a_limit, color="gray", ls="--", lw=0.8)
        ax_a.set_ylabel("a [rad/s^2]")

        if j == 0:
            ax_q.set_title("position")
            ax_v.set_title("velocity (FD)")
            ax_a.set_title("acceleration (FD)")
        if j == len(joint_names) - 1:
            for ax in (ax_q, ax_v, ax_a):
                ax.set_xlabel("t [s]")

    fig.tight_layout()
    fig.savefig(out_path, **_SAVEFIG_KWARGS)
    plt.close(fig)


def render_toolpath_png(
    out_path: Path,
    tool_xyz: np.ndarray,
    legs: list[dict],
    scene_boxes: list[np.ndarray],
    crack_polylines: list[np.ndarray],
    all_pass: bool,
    failed_checks: list[str],
) -> None:
    fig, axes = plt.subplots(1, 3, figsize=TOOLPATH_FIGSIZE, dpi=TOOLPATH_DPI)
    _apply_failure_banner(fig, all_pass, failed_checks)

    for ax, (name, ia, ib, xlabel, ylabel) in zip(axes, _PROJECTIONS):
        for leg in legs:
            first, last = leg["point_range"]
            seg = tool_xyz[first : last + 1]
            ax.plot(
                seg[:, ia], seg[:, ib], color=_LEG_COLORS.get(leg["kind"], "#808080"), lw=1.5,
                label=leg["kind"],
            )
        for box in scene_boxes:
            for e0, e1 in _BOX_EDGES:
                ax.plot(
                    [box[e0, ia], box[e1, ia]], [box[e0, ib], box[e1, ib]],
                    color=_SCENE_BOX_COLOR, lw=0.8, ls="-",
                )
        for crack in crack_polylines:
            ax.plot(crack[:, ia], crack[:, ib], color=_CRACK_COLOR, lw=1.0, ls="--")

        ax.scatter([tool_xyz[0, ia]], [tool_xyz[0, ib]], color="green", marker="o", s=40, zorder=5)
        ax.scatter([tool_xyz[-1, ia]], [tool_xyz[-1, ib]], color="red", marker="s", s=40, zorder=5)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(name.upper())
        ax.set_aspect("equal", adjustable="datalim")

    handles, labels = axes[0].get_legend_handles_labels()
    seen = dict(zip(labels, handles))
    if seen:
        fig.legend(seen.values(), seen.keys(), loc="lower center", ncol=len(seen))

    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(out_path, **_SAVEFIG_KWARGS)
    plt.close(fig)


def render_toolspeed_png(
    out_path: Path,
    tool_xyz: np.ndarray,
    t: list[float],
    legs: list[dict],
    retime_doc: dict,
    preview_speed_scale: float,
    all_pass: bool,
    failed_checks: list[str],
) -> None:
    n = len(tool_xyz)
    seg_len = [float(np.linalg.norm(tool_xyz[i + 1] - tool_xyz[i])) for i in range(n - 1)]
    speeds = [seg_len[i] / (t[i + 1] - t[i]) if (t[i + 1] - t[i]) > 0 else 0.0 for i in range(n - 1)]
    t_mid = t[:-1]

    fig, ax = plt.subplots(1, 1, figsize=TOOLSPEED_FIGSIZE, dpi=TOOLSPEED_DPI)
    _apply_failure_banner(fig, all_pass, failed_checks)

    tol = ((retime_doc.get("parameters") or {}).get("trace") or {}).get("speed_tolerance")
    report_legs = {leg["leg_index"]: leg for leg in (retime_doc.get("legs") or [])}

    for leg in legs:
        first, last = leg["point_range"]
        ax.axvline(t[first], color="gray", ls=":", lw=0.6)
        ax.axvline(t[last], color="gray", ls=":", lw=0.6)
        if leg["kind"] != "trace" or tol is None:
            continue
        report_leg = report_legs.get(leg["leg_index"])
        if not report_leg or "tool_speed" not in report_leg:
            continue
        v_c = float(report_leg["tool_speed"]["constant"])
        ax.fill_between(
            [t[first], t[last]], v_c * (1.0 - tol), v_c * (1.0 + tol),
            color=_LEG_COLORS["trace"], alpha=_LEG_SPAN_ALPHA,
        )

    ax.plot(t_mid, speeds, color="black", lw=1.2, label="unscaled")
    ax.plot(t_mid, [s * preview_speed_scale for s in speeds], color="tab:blue", lw=1.2, label="preview-scaled")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("tool speed [m/s]")
    ax.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(out_path, **_SAVEFIG_KWARGS)
    plt.close(fig)


# ---------------------------------------------------------------------------
# CLI (§0, §18.1)
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="trajectory_preview.py",
        description="Deterministic headless preview render of a crackvision.joint_trajectory/1 (docs/INTERFACES.md §18).",
    )
    add_common_args(parser)
    parser.add_argument("--trajectory", type=Path, required=True, help="crackvision.joint_trajectory/1 file")
    parser.add_argument(
        "--plan", type=Path, required=True,
        help="crackvision.cartesian_plan/1 or crackvision.view_plan/1 file",
    )
    parser.add_argument("--retime-report", type=Path, default=None, help="default <trajectory>.retime.json")
    parser.add_argument("--check-report", type=Path, default=None, help="default <trajectory>.check.json")
    parser.add_argument("--limits", type=Path, default=None, help="default config/robot/b601_dm_limits.yaml")
    parser.add_argument(
        "--end-effector-config", type=Path, default=None, help="default config/robot/end_effector.yaml"
    )
    parser.add_argument("--scene-config", type=Path, default=None, help="default config/scene/scene.yaml")
    parser.add_argument(
        "--execution-config", type=Path, default=None, help="default config/motion/execution.yaml"
    )
    parser.add_argument(
        "--urdf", type=Path, default=None,
        help="default presentation/sim/reBot_B601_DM_with_gripper.urdf",
    )
    parser.add_argument("--out-dir", type=Path, default=None, help="default data/previews/{case}_{sha8}/")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"{TOOL}: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging(TOOL, cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    limits_path = args.limits or (cfg.root / "config" / "robot" / "b601_dm_limits.yaml")
    end_effector_path = args.end_effector_config or (cfg.root / "config" / "robot" / "end_effector.yaml")
    scene_path = args.scene_config or (cfg.root / "config" / "scene" / "scene.yaml")
    execution_path = args.execution_config or (cfg.root / "config" / "motion" / "execution.yaml")
    urdf_path = args.urdf or (cfg.root / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf")
    retime_report_path = args.retime_report or Path(str(args.trajectory) + ".retime.json")
    check_report_path = args.check_report or Path(str(args.trajectory) + ".check.json")

    try:
        bound = bind_inputs(
            args.trajectory, args.plan, retime_report_path, check_report_path, root=cfg.root,
        )
    except PreviewRefusal as exc:
        logger.error("refused: %s", exc)
        summary.add_error(str(exc))
        summary.write(cfg, TOOL, "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION
    except PreviewUsageError as exc:
        logger.error("input error: %s", exc)
        summary.add_error(str(exc))
        summary.write(cfg, TOOL, "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    try:
        limits_doc = _load_yaml(limits_path)
        execution_doc = _load_yaml(execution_path)
        preview_speed_scale = float(execution_doc["speed_scale"]["real_default"])
        chain = load_chain(urdf_path=urdf_path)
        end_effector = load_end_effector(end_effector_path)
    except (UsageError, KinematicsError, KeyError, TypeError, ValueError) as exc:
        logger.error("config error: %s", exc)
        summary.add_error(str(exc))
        summary.write(cfg, TOOL, "failed", EXIT_USAGE)
        return EXIT_USAGE

    traj_doc = bound["trajectory"]["doc"]
    retime_doc = bound["retime_report"]["doc"]

    tool_xyz = tool_tip_path(traj_doc, chain, end_effector)
    summary_doc = compute_summary(traj_doc, limits_doc, tool_xyz, retime_doc, preview_speed_scale)

    case_id = (
        bound["plan"]["doc"].get("case_id") if bound["plan"]["schema"] == "crackvision.cartesian_plan/1" else "view"
    )
    if case_id is None:
        case_id = "view"
    sha8 = bound["trajectory"]["sha256"][:8]
    out_dir = args.out_dir or (cfg.root / "data" / "previews" / f"{case_id}_{sha8}")

    if args.dry_run:
        logger.info(
            "dry-run: would render preview for %s into %s; n_points=%d duration_s=%.3f all_pass=%s",
            args.trajectory, out_dir, summary_doc["n_points"], summary_doc["duration_s"],
            bound["check_report"]["all_pass"],
        )
        summary.write(cfg, TOOL, "ok", EXIT_OK)
        return EXIT_OK

    if args.skip_existing and (out_dir / "preview.json").is_file():
        logger.info("skip-existing: %s already exists", out_dir / "preview.json")
        summary.write(cfg, TOOL, "ok", EXIT_OK)
        return EXIT_OK

    scene_boxes, scene_warnings = _load_scene_boxes(scene_path)
    crack_polylines, crack_warnings = _load_crack_polylines(traj_doc, cfg.root)

    all_pass = bound["check_report"]["all_pass"]
    failed_checks = bound["check_report"]["failed_checks"]
    legs = summary_doc["tool"]["legs"]
    points = traj_doc["points"]
    t = [float(p["t_s"]) for p in points]

    created_dir = False
    try:
        out_dir.mkdir(parents=True, exist_ok=True)
        created_dir = True

        joints_path = out_dir / "joints.png"
        toolpath_path = out_dir / "toolpath.png"
        toolspeed_path = out_dir / "toolspeed.png"

        render_joints_png(joints_path, traj_doc, limits_doc, legs, all_pass, failed_checks)
        render_toolpath_png(
            toolpath_path, tool_xyz, legs, scene_boxes, crack_polylines, all_pass, failed_checks
        )
        render_toolspeed_png(
            toolspeed_path, tool_xyz, t, legs, retime_doc, preview_speed_scale, all_pass, failed_checks
        )

        artifacts = [
            {"kind": "joints", "path": str(joints_path.resolve().relative_to(cfg.root)), "sha256": file_sha256(joints_path)},
            {"kind": "toolpath", "path": str(toolpath_path.resolve().relative_to(cfg.root)), "sha256": file_sha256(toolpath_path)},
            {"kind": "toolspeed", "path": str(toolspeed_path.resolve().relative_to(cfg.root)), "sha256": file_sha256(toolspeed_path)},
        ]

        manifest = build_manifest(
            bound, summary_doc, artifacts, created_utc=_iso_now(), producer=TOOL,
        )
        manifest["warnings"].extend(scene_warnings)
        manifest["warnings"].extend(crack_warnings)

        errors = validate_manifest(manifest)
        if errors:
            raise PreviewRefusal("built an invalid manifest: " + "; ".join(errors))

        manifest_path = out_dir / "preview.json"
        import json

        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    except PreviewRefusal as exc:
        logger.error("internal manifest validation failed: %s", exc)
        summary.add_error(str(exc))
        if created_dir and out_dir.is_dir():
            shutil.rmtree(out_dir)
        summary.write(cfg, TOOL, "failed", EXIT_RUNTIME)
        return EXIT_RUNTIME
    except Exception as exc:  # unexpected rendering failure
        logger.exception("unexpected error while rendering preview")
        summary.add_error(str(exc))
        if created_dir and out_dir.is_dir():
            shutil.rmtree(out_dir)
        summary.write(cfg, TOOL, "failed", EXIT_RUNTIME)
        return EXIT_RUNTIME

    logger.info("preview written to %s (all_pass=%s)", out_dir, all_pass)
    summary.write(cfg, TOOL, "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
