"""crackvision.path3d — robot-frame 3D crack paths and tool waypoints CLI (GEOM-08.7).

Consumes a §3.13 `{case}_paths.json`, the case mask and a §9 capture record, and lifts every
main path and branch to `base_link` surface points, normals, segments and §10.5 `tool_tip`
waypoints, writing exactly the docs/INTERFACES.md §10 `data/paths3d/{case}_paths3d.json`
document. Pure CLI glue over `crackvision.capture_record`, `crackvision.lift3d` and
`crackvision.tool_waypoints` — no new geometry/uncertainty logic lives here.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from crackvision import capture_record, kinematics, lift3d, naming, tool_waypoints
from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)

SCHEMA = "crackvision.paths3d/1"
FRAME = "base_link"

UNCERTAINTY_TERMS = [
    "pixel_noise_px",
    "depth_uncertainty_stereo",
    "wrist_camera_calibration",
    "tool_calibration",
]
UNCERTAINTY_UNMODELLED = [
    "joint_encoder_and_fk",
    "depth_bias",
    "distortion_jacobian",
    "capture_time_skew_motion",
    "thermal_drift",
]


def paths3d_path(root: Path, case_id: str) -> Path:
    return Path(root) / "data" / "paths3d" / f"{case_id}_paths3d.json"


# ---------------------------------------------------------------------------
# NaN/Inf -> JSON null helpers (§10.2: invalid points carry null geometry fields)
# ---------------------------------------------------------------------------


def _scalar_or_none(v: float) -> float | None:
    v = float(v)
    return v if math.isfinite(v) else None


def _vec_or_none(v: np.ndarray) -> list[float] | None:
    arr = np.asarray(v, dtype=np.float64)
    if not np.all(np.isfinite(arr)):
        return None
    return [float(x) for x in arr]


def _mat_or_none(m: np.ndarray) -> list[float] | None:
    arr = np.asarray(m, dtype=np.float64)
    if not np.all(np.isfinite(arr)):
        return None
    return [float(x) for x in arr.reshape(-1)]


def _sigma_base_or_none(cov: np.ndarray) -> list[float] | None:
    arr = np.asarray(cov, dtype=np.float64)
    if not np.all(np.isfinite(arr)):
        return None
    diag = np.diag(arr)
    return [float(math.sqrt(max(d, 0.0))) for d in diag]


# ---------------------------------------------------------------------------
# Per-point / per-waypoint JSON assembly
# ---------------------------------------------------------------------------


def _point_dict(src: dict[str, int], lifted: "lift3d.LiftedPolyline", i: int) -> dict[str, Any]:
    valid = bool(lifted.valid[i])
    reason = None if valid else (str(lifted.reason[i]) or None)
    cov = lifted.cov_base[i]
    return {
        "row": int(src["row"]),
        "col": int(src["col"]),
        "u": int(src["u"]),
        "v": int(src["v"]),
        "valid": valid,
        "reason": reason,
        "interpolated": bool(lifted.interpolated[i]),
        "depth_m": _scalar_or_none(lifted.depth_m[i]),
        "fraction_valid": _scalar_or_none(lifted.fraction_valid[i]),
        "p_optical_m": _vec_or_none(lifted.p_optical[i]),
        "p_base_m": _vec_or_none(lifted.p_base[i]),
        "n_base": _vec_or_none(lifted.n_base[i]),
        "normal_rms_m": _scalar_or_none(lifted.normal_rms_m[i]),
        "sigma_normal_rad": _scalar_or_none(lifted.sigma_normal_rad[i]),
        "cov_base_m2": _mat_or_none(cov),
        "sigma_base_m": _sigma_base_or_none(cov),
    }


def _waypoint_dict(wp: "tool_waypoints.Waypoint") -> dict[str, Any]:
    return {
        "phase": wp.phase,
        "position_m": [float(x) for x in wp.position],
        "quat_xyzw": [float(x) for x in wp.quat_xyzw],
        "surface_point_m": [float(x) for x in wp.surface_point],
        "normal_base": [float(x) for x in wp.normal],
        "clearance_m": float(wp.clearance_m),
        "sigma_along_normal_m": float(wp.sigma_along_normal_m),
        "sigma_max_m": float(wp.sigma_max_m),
        "interpolated": bool(wp.interpolated),
        "within_budget": bool(wp.within_budget),
    }


def _lift_and_assemble_polyline(
    polyline_doc: dict[str, Any],
    depth: np.ndarray,
    mask: np.ndarray,
    intrinsics,
    depth_scale: float,
    T_base_link_optical: np.ndarray,
    calib: "lift3d.CalibSigmas",
    lift_params: "lift3d.LiftParams",
    waypoint_params: "tool_waypoints.WaypointParams",
    capture_tool_z_base: np.ndarray,
    tool_sigma_pos_m: float,
    counters: dict[str, Any],
) -> dict[str, Any]:
    dense = polyline_doc["dense"]
    points_rc = [(p["row"], p["col"]) for p in dense]
    lifted = lift3d.lift_polyline(
        points_rc, depth, mask, intrinsics, depth_scale, T_base_link_optical, calib, lift_params
    )

    points_out = [_point_dict(dense[i], lifted, i) for i in range(len(dense))]

    counters["points"] += len(dense)
    counters["valid"] += int(np.sum(lifted.valid))
    counters["interpolated"] += int(np.sum(lifted.interpolated))
    for i in range(len(dense)):
        if not lifted.valid[i]:
            reason = str(lifted.reason[i]) or "unknown"
            counters["invalid_by_reason"][reason] = counters["invalid_by_reason"].get(reason, 0) + 1

    segments_out = []
    for seg in lifted.segments:
        wps = tool_waypoints.segment_waypoints(
            lifted, seg, capture_tool_z_base, tool_sigma_pos_m, waypoint_params
        )
        for wp in wps:
            if not wp.within_budget:
                counters["uncertainty_exceeds_clearance"] = True
        segments_out.append(
            {
                "start_index": int(seg[0]),
                "end_index": int(seg[1]),
                "waypoints": [_waypoint_dict(w) for w in wps],
            }
        )

    return {"points": points_out, "segments": segments_out}


# ---------------------------------------------------------------------------
# Refusal result helper
# ---------------------------------------------------------------------------


def _failure(case_id: str, reason: str, message: str) -> dict[str, Any]:
    return {"case_id": case_id, "status": "failed", "reason": reason, "error": message}


def build_case_paths3d(
    cfg: Config,
    case_id: str,
    case_map_entry: dict[str, Any] | None,
    *,
    end_effector: "kinematics.EndEffector",
    annulus_inner_px: int,
    annulus_outer_px: int,
    min_fraction_valid: float,
    max_gap_px: int,
    pixel_sigma_px: float,
    waypoint_spacing_m: float,
    trace_clearance_m: float,
    approach_clearance_m: float,
    max_skew_s: float,
    accept_end_effector_change: bool,
) -> dict[str, Any]:
    """Build the §10 document for one case, or a `_failure(...)` dict on any refusal."""
    if case_map_entry is not None and case_map_entry.get("downscaled"):
        return _failure(case_id, "downscaled_case", f"case {case_id} is downscaled (§0.5)")

    paths_json_path = cfg.root / "data" / "paths" / f"{case_id}_paths.json"
    if not paths_json_path.is_file():
        return _failure(
            case_id, "paths_json_missing",
            f"{paths_json_path} missing (run: ./env.sh python -m crackvision.paths first)",
        )
    try:
        paths_doc = json.loads(paths_json_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return _failure(case_id, "paths_json_malformed", f"malformed {paths_json_path}: {exc}")

    mask_rel = naming.mask_path(case_id)
    mask_file = cfg.root / mask_rel
    if not mask_file.is_file():
        return _failure(case_id, "mask_missing", f"{mask_file} missing")
    try:
        with Image.open(mask_file) as img:
            img.load()
            mask = np.array(img) > 0
        if mask.ndim > 2:
            mask = mask.any(axis=-1)
    except Exception as exc:  # noqa: BLE001 — one bad case must never abort the batch
        return _failure(case_id, "mask_unreadable", f"could not read {mask_file}: {exc}")

    capture_path = capture_record.capture_record_path(cfg.root, case_id)
    if not capture_path.is_file():
        return _failure(case_id, "capture_record_missing", f"{capture_path} missing")

    try:
        record = capture_record.load_capture_record(
            capture_path,
            root=cfg.root,
            end_effector=end_effector,
            max_skew_s=max_skew_s,
            accept_end_effector_change=accept_end_effector_change,
        )
    except capture_record.CaptureRecordError as exc:
        return _failure(case_id, "capture_record_invalid", str(exc))

    try:
        depth = capture_record.load_depth(record, cfg.root)
    except capture_record.CaptureRecordError as exc:
        return _failure(case_id, "depth_load_failed", str(exc))

    paths_hw = (int(paths_doc["image_height"]), int(paths_doc["image_width"]))
    if not (paths_hw == mask.shape == depth.shape == record.image_hw):
        return _failure(
            case_id, "image_shape_mismatch",
            f"paths.json {paths_hw}, mask {mask.shape}, depth {depth.shape} and capture record "
            f"{record.image_hw} must all match (§0.5)",
        )

    try:
        chain = kinematics.load_chain(urdf_path=cfg.root / record.kinematic_model_path)
    except (OSError, kinematics.KinematicsError) as exc:
        return _failure(case_id, "kinematic_model_load_failed", str(exc))

    T_base_link_optical = capture_record.T_base_link_optical(record, chain, end_effector)
    capture_tool_z_base = tool_waypoints.capture_tool_z(chain, end_effector, record.positions_rad)

    calib = lift3d.CalibSigmas(
        cam_pos_m=end_effector.camera_sigma_pos_m, cam_rot_rad=end_effector.camera_sigma_rot_rad
    )
    lift_params = lift3d.LiftParams(
        annulus_inner_px=annulus_inner_px,
        annulus_outer_px=annulus_outer_px,
        min_fraction_valid=min_fraction_valid,
        pixel_sigma_px=pixel_sigma_px,
        max_gap_px=max_gap_px,
    )
    waypoint_params = tool_waypoints.WaypointParams(
        trace_clearance_m=trace_clearance_m,
        approach_clearance_m=approach_clearance_m,
        waypoint_spacing_m=waypoint_spacing_m,
    )

    counters: dict[str, Any] = {
        "points": 0,
        "valid": 0,
        "interpolated": 0,
        "invalid_by_reason": {},
        "uncertainty_exceeds_clearance": False,
    }

    components_out = []
    for comp in paths_doc["components"]:
        polylines_out = []
        polylines_out.append(
            {
                "kind": "main",
                "branch_index": None,
                **_lift_and_assemble_polyline(
                    comp["main_path"], depth, mask, record.color_intrinsics,
                    record.depth_scale_m_per_unit, T_base_link_optical, calib, lift_params,
                    waypoint_params, capture_tool_z_base, end_effector.tool_sigma_pos_m, counters,
                ),
            }
        )
        for branch_idx, branch in enumerate(comp["branches"]):
            polylines_out.append(
                {
                    "kind": "branch",
                    "branch_index": branch_idx,
                    **_lift_and_assemble_polyline(
                        branch, depth, mask, record.color_intrinsics,
                        record.depth_scale_m_per_unit, T_base_link_optical, calib, lift_params,
                        waypoint_params, capture_tool_z_base, end_effector.tool_sigma_pos_m, counters,
                    ),
                }
            )
        components_out.append(
            {
                "order_index": comp["order_index"],
                "component_id": comp["component_id"],
                "polylines": polylines_out,
            }
        )

    end_effector_sha256_now = kinematics.file_sha256(kinematics.DEFAULT_END_EFFECTOR)

    ineligible_reasons: list[str] = []
    if end_effector.wrist_camera_value_status != "measured":
        ineligible_reasons.append("wrist_camera_nominal")
    if end_effector.tool_value_status != "measured":
        ineligible_reasons.append("tool_nominal")
    if record.optical_source == "nominal_d405":
        ineligible_reasons.append("optical_frames_nominal")
    if record.synthetic:
        ineligible_reasons.append("synthetic_capture")
    if record.end_effector_sha256 != end_effector_sha256_now:
        ineligible_reasons.append("end_effector_changed_since_capture")
    if counters["uncertainty_exceeds_clearance"]:
        ineligible_reasons.append("uncertainty_exceeds_clearance")
    if counters["valid"] == 0:
        ineligible_reasons.append("no_valid_points")

    doc = {
        "schema": SCHEMA,
        "case_id": case_id,
        "frame": FRAME,
        "image_height": record.image_hw[0],
        "image_width": record.image_hw[1],
        "sources": {
            "paths_json": paths_json_path.relative_to(cfg.root).as_posix(),
            "paths_json_sha256": kinematics.file_sha256(paths_json_path),
            "capture_record": capture_path.relative_to(cfg.root).as_posix(),
            "capture_record_sha256": kinematics.file_sha256(capture_path),
            "mask": Path(mask_rel).as_posix(),
            "mask_sha256": kinematics.file_sha256(mask_file),
            "end_effector": {
                "sha256_at_capture": record.end_effector_sha256,
                "sha256_now": end_effector_sha256_now,
            },
            "urdf_sha256": record.kinematic_model_sha256,
        },
        "parameters": {
            "annulus_inner_px": int(annulus_inner_px),
            "annulus_outer_px": int(annulus_outer_px),
            "annulus_min_fraction_valid": float(min_fraction_valid),
            "valid_depth_range_m": list(lift_params.valid_range_m),
            "max_gap_px": int(max_gap_px),
            "sigma_px": float(pixel_sigma_px),
            "waypoint_spacing_m": float(waypoint_spacing_m),
            "approach_retract_offset_m": float(approach_clearance_m),
            "trace_clearance_m": float(trace_clearance_m),
            "roll_free": True,
            "max_clock_skew_s": float(max_skew_s),
        },
        "calibration": {
            "wrist_camera": {
                "value_status": end_effector.wrist_camera_value_status,
                "position_sigma_m": float(end_effector.camera_sigma_pos_m),
                "rotation_sigma_rad": float(end_effector.camera_sigma_rot_rad),
                "source": end_effector.sigma_source["wrist_camera"],
            },
            "tool": {
                "value_status": end_effector.tool_value_status,
                "position_sigma_m": float(end_effector.tool_sigma_pos_m),
                "source": end_effector.sigma_source["tool"],
            },
        },
        "uncertainty_model": {
            "terms": UNCERTAINTY_TERMS,
            "unmodelled": UNCERTAINTY_UNMODELLED,
        },
        "execution_eligible": len(ineligible_reasons) == 0,
        "ineligible_reasons": ineligible_reasons,
        "counts": {
            "points": counters["points"],
            "valid": counters["valid"],
            "interpolated": counters["interpolated"],
            "invalid_by_reason": counters["invalid_by_reason"],
        },
        "components": components_out,
    }

    return {"case_id": case_id, "status": "ok", "doc": doc}


# ---------------------------------------------------------------------------
# CLI (docs/INTERFACES.md §10)
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="path3d.py",
        description="Robot-frame 3D crack paths + tool waypoints (docs/INTERFACES.md §10).",
    )
    add_common_args(parser)
    parser.add_argument("--cases", nargs="+", default=None, metavar="CASE_ID")
    parser.add_argument("--trace-clearance-m", type=float, default=0.01)
    parser.add_argument("--approach-clearance-m", type=float, default=0.04)
    parser.add_argument("--waypoint-spacing-m", type=float, default=0.002)
    parser.add_argument("--annulus-inner-px", type=int, default=3)
    parser.add_argument("--annulus-outer-px", type=int, default=8)
    parser.add_argument("--min-fraction-valid", type=float, default=0.3)
    parser.add_argument("--max-gap-px", type=int, default=5)
    parser.add_argument("--pixel-sigma-px", type=float, default=1.0)
    parser.add_argument("--max-skew-s", type=float, default=capture_record.DEFAULT_MAX_SKEW_S)
    parser.add_argument("--accept-end-effector-change", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"path3d: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("path3d", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    case_map_path = cfg.root / "data" / "case_map.json"
    if not case_map_path.is_file():
        msg = "run: ./env.sh python -m crackvision.prepare_inputs first"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "path3d", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    try:
        case_map = json.loads(case_map_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        msg = f"malformed {case_map_path}: {exc}"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "path3d", "failed", EXIT_RUNTIME)
        return EXIT_RUNTIME

    all_cases: list[dict[str, Any]] = case_map.get("cases", [])
    entries_by_id = {c["case_id"]: c for c in all_cases}

    unknown_count = 0
    if args.cases:
        wanted = list(dict.fromkeys(args.cases))
        case_ids = [c for c in wanted if c in entries_by_id]
        for name in wanted:
            if name not in entries_by_id:
                logger.error("requested case_id not found in case_map.json: %s", name)
                summary.add_error(f"{name}: unknown_case_id")
                unknown_count += 1
    else:
        wanted = [c["case_id"] for c in all_cases]
        case_ids = list(wanted)

    total_requested = len(wanted)

    if args.dry_run:
        for case_id in case_ids:
            logger.info("dry-run: would lift paths for case %s", case_id)
        logger.info("dry-run: no files written")
        summary.write(cfg, "path3d", "ok", EXIT_OK)
        return EXIT_OK

    try:
        end_effector = kinematics.load_end_effector()
    except (OSError, kinematics.KinematicsError) as exc:
        msg = f"could not load end_effector.yaml: {exc}"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "path3d", "failed", EXIT_RUNTIME)
        return EXIT_RUNTIME

    failed_count = unknown_count
    ok_count = 0
    skipped_count = 0

    for case_id in case_ids:
        out_path = paths3d_path(cfg.root, case_id)
        if args.skip_existing and out_path.is_file():
            logger.info("skip-existing: case %s already has paths3d output", case_id)
            summary.increment("skipped")
            skipped_count += 1
            continue

        result = build_case_paths3d(
            cfg,
            case_id,
            entries_by_id.get(case_id),
            end_effector=end_effector,
            annulus_inner_px=args.annulus_inner_px,
            annulus_outer_px=args.annulus_outer_px,
            min_fraction_valid=args.min_fraction_valid,
            max_gap_px=args.max_gap_px,
            pixel_sigma_px=args.pixel_sigma_px,
            waypoint_spacing_m=args.waypoint_spacing_m,
            trace_clearance_m=args.trace_clearance_m,
            approach_clearance_m=args.approach_clearance_m,
            max_skew_s=args.max_skew_s,
            accept_end_effector_change=args.accept_end_effector_change,
        )

        if result["status"] == "failed":
            logger.error("case %s: REFUSED (%s): %s", case_id, result["reason"], result["error"])
            summary.add_error(f"{case_id}: {result['reason']}: {result['error']}")
            summary.increment("failed")
            failed_count += 1
            continue

        doc = result["doc"]
        out_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = out_path.with_suffix(out_path.suffix + ".tmp")
        tmp_path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        tmp_path.replace(out_path)

        logger.info(
            "case %s: %d component(s), %d/%d points valid, execution_eligible=%s",
            case_id,
            len(doc["components"]),
            doc["counts"]["valid"],
            doc["counts"]["points"],
            doc["execution_eligible"],
        )
        summary.increment("ok")
        ok_count += 1

    if total_requested > 0 and failed_count == total_requested:
        summary.write(cfg, "path3d", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION
    if failed_count > 0:
        summary.write(cfg, "path3d", "partial", EXIT_RUNTIME)
        return EXIT_RUNTIME

    summary.write(cfg, "path3d", "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
