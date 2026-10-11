"""crackvision.uncertainty_budget — pixel-to-tool-pose uncertainty budget CLI (GEOM-09.4.1).

Reads an already-written docs/INTERFACES.md §10 `paths3d` file plus its capture record,
`end_effector.yaml`, kinematic chain and optional `config/robot/b601_dm_limits.yaml` /
`config/calibration/uncertainty_budget.yaml`, and assembles the §14
`crackvision.uncertainty_budget/1` report: the `crackvision.uncertainty` analytic per-term
decomposition (pixel/depth/handeye/fk) plus the `crackvision.uncertainty_mc` Monte-Carlo
cross-check, for every valid dense point and every segment waypoint. Pure CLI glue — no new
geometry/uncertainty logic lives here.

Exit codes (docs/INTERFACES.md §0): 0 ok; 1 runtime — either some/all requested cases failed to
process, or every case processed but at least one case's `summary.overall_pass` is `false` (an
MC/analytic disagreement was flagged); 2 usage — malformed `--config` or
`config/calibration/uncertainty_budget.yaml`; 3 precondition — every requested case is missing a
required input file. A disagreeing report is still written to disk (§14.3: diagnostic, never a
gate) even when exit code 1 is returned for that reason.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from crackvision import capture_record, kinematics, lift3d, path3d, uncertainty, uncertainty_mc
from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)

SCHEMA = "crackvision.uncertainty_budget/1"
CONFIG_SCHEMA = "crackvision.uncertainty_budget_config/1"
FRAME = "base_link"
PRODUCER = "crackvision.uncertainty_budget"

DEFAULT_N_SAMPLES = 2000
DEFAULT_SEED = 0
DEFAULT_SUBSAMPLE: int | str = "all"
DEFAULT_AGREEMENT_BAND = (0.5, 2.0)
NOMINAL_SIGMA_JOINT_RAD = 0.001  # ADR-018 §3

CAVEAT_TEXT = (
    "Camera intrinsics uncertainty is not budgeted as a separate additive term; it is only "
    "indirectly exercised by the Monte-Carlo resample's nonlinear deprojection/annulus geometry "
    "(ADR-018 §2)."
)

_TOP_LEVEL_CONFIG_KEYS = {"schema", "mc", "sigma_joint_rad"}
_MC_CONFIG_KEYS = {"n_samples", "seed", "subsample", "agreement_band"}


class UncertaintyBudgetConfigError(ValueError):
    """Malformed `config/calibration/uncertainty_budget.yaml` (`crackvision.uncertainty_budget_config/1`)."""


def uncertainty_budget_path(root: Path, case_id: str) -> Path:
    return Path(root) / "data" / "uncertainty" / f"{case_id}_uncertainty_budget.json"


def limits_path(root: Path) -> Path:
    return Path(root) / "config" / "robot" / "b601_dm_limits.yaml"


def uncertainty_budget_config_path(root: Path) -> Path:
    return Path(root) / "config" / "calibration" / "uncertainty_budget.yaml"


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ---------------------------------------------------------------------------
# config/calibration/uncertainty_budget.yaml (§14.6)
# ---------------------------------------------------------------------------


@dataclass
class MCParams:
    n_samples: int
    seed: int
    subsample: int | str
    agreement_band: tuple[float, float]


@dataclass
class BudgetConfig:
    mc: MCParams
    sigma_joint_rad_override: float | None


def _check_keys(d: Any, allowed: set[str], ctx: str) -> None:
    if not isinstance(d, dict):
        raise UncertaintyBudgetConfigError(f"'{ctx}' must be a mapping")
    unknown = set(d.keys()) - allowed
    if unknown:
        raise UncertaintyBudgetConfigError(f"unknown key(s) in '{ctx}': {sorted(unknown)}")


def _default_budget_config() -> BudgetConfig:
    return BudgetConfig(
        mc=MCParams(
            n_samples=DEFAULT_N_SAMPLES,
            seed=DEFAULT_SEED,
            subsample=DEFAULT_SUBSAMPLE,
            agreement_band=tuple(DEFAULT_AGREEMENT_BAND),
        ),
        sigma_joint_rad_override=None,
    )


def load_budget_config(path: Path) -> BudgetConfig:
    """Load+validate `uncertainty_budget.yaml`; absence is not an error (§14.6 defaults apply)."""
    path = Path(path)
    if not path.is_file():
        return _default_budget_config()

    try:
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
    except yaml.YAMLError as exc:
        raise UncertaintyBudgetConfigError(f"malformed YAML in {path}: {exc}") from exc

    _check_keys(doc, _TOP_LEVEL_CONFIG_KEYS, "uncertainty_budget.yaml")

    schema = doc.get("schema")
    if schema != CONFIG_SCHEMA:
        raise UncertaintyBudgetConfigError(f"'schema' must be {CONFIG_SCHEMA!r}, got {schema!r}")

    mc_doc = doc.get("mc", {}) or {}
    _check_keys(mc_doc, _MC_CONFIG_KEYS, "mc")

    n_samples = mc_doc.get("n_samples", DEFAULT_N_SAMPLES)
    if isinstance(n_samples, bool) or not isinstance(n_samples, int) or n_samples <= 0:
        raise UncertaintyBudgetConfigError(f"'mc.n_samples' must be a positive int, got {n_samples!r}")

    seed = mc_doc.get("seed", DEFAULT_SEED)
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise UncertaintyBudgetConfigError(f"'mc.seed' must be an int, got {seed!r}")

    subsample = mc_doc.get("subsample", DEFAULT_SUBSAMPLE)
    if subsample != "all":
        if isinstance(subsample, bool) or not isinstance(subsample, int) or subsample <= 0:
            raise UncertaintyBudgetConfigError(
                f"'mc.subsample' must be 'all' or a positive int, got {subsample!r}"
            )

    band = mc_doc.get("agreement_band", list(DEFAULT_AGREEMENT_BAND))
    if not isinstance(band, list) or len(band) != 2:
        raise UncertaintyBudgetConfigError(f"'mc.agreement_band' must be a 2-element list, got {band!r}")
    lo_raw, hi_raw = band
    for v in (lo_raw, hi_raw):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise UncertaintyBudgetConfigError(f"'mc.agreement_band' entries must be numeric, got {band!r}")
    lo, hi = float(lo_raw), float(hi_raw)
    if not (lo > 0 and hi > 0 and lo < 1.0 < hi):
        raise UncertaintyBudgetConfigError(
            f"'mc.agreement_band' must satisfy min < 1.0 < max, both > 0; got {band!r}"
        )

    sigma_joint_rad_override = doc.get("sigma_joint_rad")
    if sigma_joint_rad_override is not None:
        if isinstance(sigma_joint_rad_override, bool) or not isinstance(sigma_joint_rad_override, (int, float)):
            raise UncertaintyBudgetConfigError(
                f"'sigma_joint_rad' must be numeric or null, got {sigma_joint_rad_override!r}"
            )
        sigma_joint_rad_override = float(sigma_joint_rad_override)
        if sigma_joint_rad_override <= 0:
            raise UncertaintyBudgetConfigError(f"'sigma_joint_rad' must be > 0, got {sigma_joint_rad_override!r}")

    return BudgetConfig(
        mc=MCParams(n_samples=n_samples, seed=seed, subsample=subsample, agreement_band=(lo, hi)),
        sigma_joint_rad_override=sigma_joint_rad_override,
    )


def _resolve_sigma_joint_rad(root: Path, override: float | None) -> tuple[float, str, dict[str, str] | None]:
    """`(sigma_joint_rad, source, sources.limits)` per §14.1/§14.3."""
    if override is not None:
        return override, "nominal", None

    path = limits_path(root)
    if path.is_file():
        with open(path, "r", encoding="utf-8") as f:
            limits_doc = yaml.safe_load(f) or {}
        repeatability = limits_doc.get("repeatability_rad")
        if repeatability is not None:
            sources_limits = {
                "path": path.relative_to(root).as_posix(),
                "sha256": kinematics.file_sha256(path),
            }
            return float(repeatability), "limits_file", sources_limits

    return NOMINAL_SIGMA_JOINT_RAD, "nominal", None


# ---------------------------------------------------------------------------
# Per-case assembly (§14.3-§14.5)
# ---------------------------------------------------------------------------


def _mat3(flat9: list[float] | None) -> np.ndarray:
    if flat9 is None:
        return np.full((3, 3), np.nan)
    return np.asarray(flat9, dtype=np.float64).reshape(3, 3)


def _flat9(m: np.ndarray) -> list[float]:
    return [float(x) for x in np.asarray(m, dtype=np.float64).reshape(-1)]


def _select_subsample(n_total: int, subsample: int | str) -> set[int]:
    if subsample == "all" or n_total <= 0:
        return set(range(n_total))
    k = min(int(subsample), n_total)
    if k <= 0:
        return set()
    if k == 1:
        return {0}
    picks = np.round(np.linspace(0, n_total - 1, k)).astype(int)
    return {int(x) for x in picks}


def _failure(case_id: str, reason: str, message: str) -> dict[str, Any]:
    return {"case_id": case_id, "status": "failed", "reason": reason, "error": message}


def build_case_uncertainty_budget(
    cfg: Config,
    case_id: str,
    *,
    end_effector: "kinematics.EndEffector",
    budget_cfg: BudgetConfig,
) -> dict[str, Any]:
    """Build the §14 report dict for one case, or a `_failure(...)` dict on any refusal."""
    paths3d_file = path3d.paths3d_path(cfg.root, case_id)
    if not paths3d_file.is_file():
        return _failure(
            case_id, "paths3d_missing",
            f"{paths3d_file} missing (run: ./env.sh python -m crackvision.path3d first)",
        )
    try:
        doc = json.loads(paths3d_file.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return _failure(case_id, "paths3d_malformed", f"malformed {paths3d_file}: {exc}")

    if doc.get("schema") != path3d.SCHEMA:
        return _failure(
            case_id, "paths3d_schema_mismatch",
            f"expected schema {path3d.SCHEMA!r}, got {doc.get('schema')!r}",
        )

    capture_rel = doc["sources"]["capture_record"]
    capture_full = cfg.root / capture_rel
    if not capture_full.is_file():
        return _failure(case_id, "capture_record_missing", f"{capture_full} missing")

    try:
        record = capture_record.load_capture_record(
            capture_full,
            root=cfg.root,
            end_effector=end_effector,
            accept_end_effector_change=True,
        )
    except capture_record.CaptureRecordError as exc:
        return _failure(case_id, "capture_record_invalid", str(exc))

    try:
        chain = kinematics.load_chain(urdf_path=cfg.root / record.kinematic_model_path)
    except (OSError, kinematics.KinematicsError) as exc:
        return _failure(case_id, "kinematic_model_load_failed", str(exc))

    T_base_optical = capture_record.T_base_link_optical(record, chain, end_effector)
    R_base_optical = T_base_optical[:3, :3]

    def T_base_optical_of_q(q: np.ndarray) -> np.ndarray:
        return chain.fk(q) @ end_effector.T_gripper_link_camera_link @ kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME

    wrist_block = doc["calibration"]["wrist_camera"]
    tool_block = doc["calibration"]["tool"]
    calib = lift3d.CalibSigmas(
        cam_pos_m=float(wrist_block["position_sigma_m"]), cam_rot_rad=float(wrist_block["rotation_sigma_rad"]),
    )
    tool_sigma_pos_m = float(tool_block["position_sigma_m"])
    sigma_px = float(doc["parameters"]["sigma_px"])
    spacing_m = float(doc["parameters"]["waypoint_spacing_m"])

    sigma_joint_rad, sigma_joint_rad_source, sources_limits = _resolve_sigma_joint_rad(
        cfg.root, budget_cfg.sigma_joint_rad_override
    )

    intrinsics = record.color_intrinsics
    fx, fy = intrinsics.fx, intrinsics.fy
    mc = budget_cfg.mc

    breakdown_cache: dict[tuple[int, int, int | None], uncertainty.PointBreakdown] = {}

    def get_breakdown(polyline_key: tuple[int, int, int | None], arrays: dict, idx: int) -> uncertainty.PointBreakdown:
        cache_key = (*polyline_key, idx)
        cached = breakdown_cache.get(cache_key)
        if cached is not None:
            return cached
        p_opt = arrays["p_optical"][idx]
        cov = arrays["cov_base"][idx]
        fk_cov = uncertainty.fk_covariance(T_base_optical_of_q, record.positions_rad, p_opt, sigma_joint_rad)
        bd = uncertainty.decompose_point(
            p_opt, fx, fy, R_base_optical, sigma_px, calib,
            valid=True, interpolated=False, cov_base=cov, fk_cov=fk_cov,
        )
        breakdown_cache[cache_key] = bd
        return bd

    # -- pass 1: build per-polyline arrays and collect every valid-point candidate (global order) --
    polylines_meta: list[dict[str, Any]] = []
    candidates: list[dict[str, Any]] = []

    for comp in doc["components"]:
        component_id = comp["component_id"]
        for pl in comp["polylines"]:
            pts = pl["points"]
            n = len(pts)
            valid_arr = np.array([bool(p["valid"]) for p in pts])
            interp_arr = np.array([bool(p["interpolated"]) for p in pts])
            cov_arr = np.stack([_mat3(p["cov_base_m2"]) for p in pts])
            p_optical_arr = [p["p_optical_m"] for p in pts]
            p_base_arr = np.array([p["p_base_m"] if p["p_base_m"] is not None else [np.nan] * 3 for p in pts])
            row_arr = [p["row"] for p in pts]
            col_arr = [p["col"] for p in pts]
            depth_arr = [p["depth_m"] for p in pts]

            key = (component_id, 0 if pl["kind"] == "main" else 1, pl["branch_index"])
            arrays = {
                "valid": valid_arr, "interpolated": interp_arr, "cov_base": cov_arr,
                "p_optical": p_optical_arr, "p_base": p_base_arr,
                "row": row_arr, "col": col_arr, "depth_m": depth_arr,
            }
            polylines_meta.append({"component_id": component_id, "kind": pl["kind"],
                                    "branch_index": pl["branch_index"], "key": key,
                                    "arrays": arrays, "segments": pl["segments"]})

            for i in range(n):
                if not valid_arr[i]:
                    continue
                candidates.append({"polyline_idx": len(polylines_meta) - 1, "point_index": i})

    selected = _select_subsample(len(candidates), mc.subsample)

    points_out: list[dict[str, Any]] = []
    for global_idx, cand in enumerate(candidates):
        if global_idx not in selected:
            continue
        pl_meta = polylines_meta[cand["polyline_idx"]]
        key, arrays = pl_meta["key"], pl_meta["arrays"]
        i = cand["point_index"]

        source_idx = i if not arrays["interpolated"][i] else uncertainty.measured_source_index(
            arrays["valid"], arrays["interpolated"], arrays["cov_base"], i
        )
        breakdown = get_breakdown(key, arrays, source_idx)
        total = breakdown.total

        mc_result = uncertainty_mc.mc_point_covariance(
            row=arrays["row"][i], col=arrays["col"][i], depth_m=arrays["depth_m"][i],
            intrinsics=intrinsics, sigma_px=sigma_px, chain=chain, q=record.positions_rad,
            T_gripper_camera=end_effector.T_gripper_link_camera_link,
            T_camera_optical=kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME,
            sigma_joint_rad=sigma_joint_rad, cam_pos_m=calib.cam_pos_m, cam_rot_rad=calib.cam_rot_rad,
            n_samples=mc.n_samples, seed=mc.seed, analytic_cov_base=total, agreement_band=mc.agreement_band,
        )

        points_out.append({
            "component_id": pl_meta["component_id"], "polyline_kind": pl_meta["kind"],
            "branch_index": pl_meta["branch_index"], "point_index": i,
            "row": int(arrays["row"][i]), "col": int(arrays["col"][i]),
            "sigma_total_m": [float(np.sqrt(max(d, 0.0))) for d in np.diag(total)],
            "contributions_m2": {
                "pixel": _flat9(breakdown.pixel), "depth": _flat9(breakdown.depth),
                "handeye": _flat9(breakdown.handeye), "fk": _flat9(breakdown.fk),
            },
            "mc": {
                "sigma_total_m": [float(np.sqrt(max(d, 0.0))) for d in np.diag(mc_result.cov_base_mc)],
                "n_samples_used": int(mc.n_samples),
            },
            "mc_analytic_agreement": {
                "ratio_trace": mc_result.ratio_trace, "ratio_eig_max": mc_result.ratio_eig_max,
                "flagged": mc_result.flagged,
            },
        })

    waypoints_out: list[dict[str, Any]] = []
    for pl_meta in polylines_meta:
        key, arrays = pl_meta["key"], pl_meta["arrays"]
        for segment_index, seg in enumerate(pl_meta["segments"]):
            seg_tuple = (seg["start_index"], seg["end_index"])
            nearest_indices = uncertainty.waypoint_source_indices(arrays["p_base"], seg_tuple, spacing_m)
            for waypoint_index, (wp, nearest_idx) in enumerate(zip(seg["waypoints"], nearest_indices)):
                source_idx = nearest_idx if not arrays["interpolated"][nearest_idx] else uncertainty.measured_source_index(
                    arrays["valid"], arrays["interpolated"], arrays["cov_base"], nearest_idx
                )
                breakdown = get_breakdown(key, arrays, source_idx)
                normal = wp["normal_base"]
                wp_breakdown = uncertainty.decompose_waypoint(breakdown, normal, tool_sigma_pos_m)
                sigma_along_normal_m = float(np.sqrt(wp_breakdown.total))

                mc_wp = uncertainty_mc.mc_waypoint_sigma(
                    row=arrays["row"][source_idx], col=arrays["col"][source_idx],
                    depth_m=arrays["depth_m"][source_idx], intrinsics=intrinsics, sigma_px=sigma_px,
                    chain=chain, q=record.positions_rad, T_gripper_camera=end_effector.T_gripper_link_camera_link,
                    T_camera_optical=kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME,
                    sigma_joint_rad=sigma_joint_rad, cam_pos_m=calib.cam_pos_m, cam_rot_rad=calib.cam_rot_rad,
                    normal=normal, clearance_m=wp["clearance_m"], tool_sigma_pos_m=tool_sigma_pos_m,
                    n_samples=mc.n_samples, seed=mc.seed,
                    analytic_sigma_along_normal_m=sigma_along_normal_m, agreement_band=mc.agreement_band,
                )

                waypoints_out.append({
                    "component_id": pl_meta["component_id"], "polyline_kind": pl_meta["kind"],
                    "branch_index": pl_meta["branch_index"], "segment_index": segment_index,
                    "phase": wp["phase"], "waypoint_index": waypoint_index,
                    "sigma_along_normal_base_m": float(wp["sigma_along_normal_m"]),
                    "sigma_along_normal_m": sigma_along_normal_m,
                    "contributions_m2": {
                        "pixel": wp_breakdown.pixel, "depth": wp_breakdown.depth,
                        "handeye": wp_breakdown.handeye, "tcp": wp_breakdown.tcp, "fk": wp_breakdown.fk,
                    },
                    "mc": {"sigma_along_normal_m": mc_wp.sigma_along_normal_mc_m},
                    "mc_analytic_agreement": {
                        "ratio_along_normal": mc_wp.ratio_along_normal, "flagged": mc_wp.flagged,
                    },
                    "within_budget": bool(wp["within_budget"]),
                })

    points_flagged = sum(1 for p in points_out if p["mc_analytic_agreement"]["flagged"])
    waypoints_flagged = sum(1 for w in waypoints_out if w["mc_analytic_agreement"]["flagged"])

    end_effector_rel = kinematics.DEFAULT_END_EFFECTOR
    try:
        end_effector_rel_path = end_effector_rel.relative_to(cfg.root).as_posix()
    except ValueError:
        end_effector_rel_path = str(end_effector_rel)

    sources_config = None
    config_file = uncertainty_budget_config_path(cfg.root)
    if config_file.is_file():
        sources_config = {
            "path": config_file.relative_to(cfg.root).as_posix(), "sha256": kinematics.file_sha256(config_file),
        }

    report = {
        "schema": SCHEMA,
        "case_id": case_id,
        "created_utc": _utc_iso(),
        "producer": PRODUCER,
        "frame": FRAME,
        "sources": {
            "paths3d": {
                "path": paths3d_file.relative_to(cfg.root).as_posix(), "sha256": kinematics.file_sha256(paths3d_file),
            },
            "capture_record": {
                "path": Path(capture_rel).as_posix(), "sha256": kinematics.file_sha256(capture_full),
            },
            "end_effector": {"path": end_effector_rel_path, "sha256": end_effector.sha256},
            "limits": sources_limits,
            "config": sources_config,
        },
        "parameters": {
            "sigma_joint_rad": float(sigma_joint_rad),
            "sigma_joint_rad_source": sigma_joint_rad_source,
            "mc": {
                "n_samples": int(mc.n_samples), "seed": int(mc.seed), "subsample": mc.subsample,
                "agreement_band": [float(mc.agreement_band[0]), float(mc.agreement_band[1])],
            },
        },
        "points": points_out,
        "waypoints": waypoints_out,
        "summary": {
            "points_flagged": points_flagged, "waypoints_flagged": waypoints_flagged,
            "overall_pass": points_flagged == 0 and waypoints_flagged == 0,
        },
        "caveats": [CAVEAT_TEXT],
    }

    return {"case_id": case_id, "status": "ok", "doc": report}


# ---------------------------------------------------------------------------
# CLI (docs/INTERFACES.md §14)
# ---------------------------------------------------------------------------


def _discover_cases(root: Path) -> list[str]:
    paths3d_dir = Path(root) / "data" / "paths3d"
    if not paths3d_dir.is_dir():
        return []
    suffix = "_paths3d.json"
    return sorted(p.name[: -len(suffix)] for p in paths3d_dir.glob(f"*{suffix}"))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="uncertainty_budget.py",
        description=(
            "Pixel-to-tool-pose uncertainty budget: analytic decomposition + Monte-Carlo "
            "cross-check (docs/INTERFACES.md §14). Exit codes: 0 ok; 1 runtime (a case failed, "
            "or every case processed but at least one case's summary.overall_pass is false — "
            "an MC/analytic disagreement; the report is still written in that case); 2 usage "
            "(malformed --config or uncertainty_budget.yaml); 3 precondition (every requested "
            "case is missing a required input file)."
        ),
    )
    add_common_args(parser)
    parser.add_argument("--cases", nargs="+", default=None, metavar="CASE_ID")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"uncertainty_budget: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("uncertainty_budget", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    budget_config_file = uncertainty_budget_config_path(cfg.root)
    try:
        budget_cfg = load_budget_config(budget_config_file)
    except UncertaintyBudgetConfigError as exc:
        msg = f"malformed {budget_config_file}: {exc}"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "uncertainty_budget", "failed", EXIT_USAGE)
        return EXIT_USAGE

    if args.cases:
        case_ids = list(dict.fromkeys(args.cases))
    else:
        case_ids = _discover_cases(cfg.root)

    total_requested = len(case_ids)
    if total_requested == 0:
        msg = "no --cases given and no data/paths3d/*_paths3d.json found"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "uncertainty_budget", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    if args.dry_run:
        for case_id in case_ids:
            logger.info("dry-run: would compute uncertainty budget for case %s", case_id)
        logger.info("dry-run: no files written")
        summary.write(cfg, "uncertainty_budget", "ok", EXIT_OK)
        return EXIT_OK

    try:
        end_effector = kinematics.load_end_effector()
    except (OSError, kinematics.KinematicsError) as exc:
        msg = f"could not load end_effector.yaml: {exc}"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "uncertainty_budget", "failed", EXIT_RUNTIME)
        return EXIT_RUNTIME

    failed_count = 0
    ok_count = 0
    skipped_count = 0
    any_flagged = False

    for case_id in case_ids:
        out_path = uncertainty_budget_path(cfg.root, case_id)
        if args.skip_existing and out_path.is_file():
            logger.info("skip-existing: case %s already has an uncertainty budget", case_id)
            summary.increment("skipped")
            skipped_count += 1
            continue

        result = build_case_uncertainty_budget(cfg, case_id, end_effector=end_effector, budget_cfg=budget_cfg)

        if result["status"] == "failed":
            logger.error("case %s: REFUSED (%s): %s", case_id, result["reason"], result["error"])
            summary.add_error(f"{case_id}: {result['reason']}: {result['error']}")
            summary.increment("failed")
            failed_count += 1
            continue

        report = result["doc"]
        out_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = out_path.with_suffix(out_path.suffix + ".tmp")
        tmp_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        tmp_path.replace(out_path)

        if report["summary"]["overall_pass"]:
            logger.info("case %s: overall_pass=True", case_id)
        else:
            any_flagged = True
            logger.warning(
                "case %s: overall_pass=False (%d point(s), %d waypoint(s) flagged)",
                case_id, report["summary"]["points_flagged"], report["summary"]["waypoints_flagged"],
            )
        summary.increment("ok")
        ok_count += 1

    if total_requested > 0 and failed_count == total_requested:
        summary.write(cfg, "uncertainty_budget", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION
    if failed_count > 0:
        summary.write(cfg, "uncertainty_budget", "partial", EXIT_RUNTIME)
        return EXIT_RUNTIME
    if any_flagged:
        summary.write(cfg, "uncertainty_budget", "flagged", EXIT_RUNTIME)
        return EXIT_RUNTIME

    summary.write(cfg, "uncertainty_budget", "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
