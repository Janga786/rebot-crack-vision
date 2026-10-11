"""tests/test_uncertainty_budget_cli.py — GEOM-09.4.1 crackvision.uncertainty_budget CLI.

Builds the same synthetic case tests/test_path3d_cli.py builds (paths.json + mask + depth PNG +
capture record), runs the real `path3d` CLI to produce the `_paths3d.json` input fixture, then
runs this CLI end-to-end: full schema key sets, the `contributions_m2` summing identities, a
clean `overall_pass: true` / exit 0 run with a small `mc.n_samples` config for speed, and a
forced-disagreement run (an impossibly tight `mc.agreement_band`) that still writes a
schema-valid report but returns a non-zero exit.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from crackvision import capture_record as cr
from crackvision import kinematics as kin
from crackvision import naming
from crackvision import path3d
from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE
from crackvision.uncertainty_budget import build_parser, main, uncertainty_budget_path

SCALE = 1e-4
SPACING = 30
N_POINTS = 28
ROW = 50
INVALID_SHORT = set(range(5, 8))  # 3-point run -> interpolated (<= default max_gap_px=5)
INVALID_LONG = set(range(13, 23))  # 10-point run -> splits (> default max_gap_px=5)


def _cols() -> list[int]:
    return [SPACING * (i + 1) for i in range(N_POINTS)]


def _island_depth_image(invalid_indices: set[int], valid_depth_m: float = 0.25):
    height = 100
    cols = _cols()
    width = SPACING * (N_POINTS + 2)
    boundaries = [0] + [(cols[i] + cols[i + 1]) // 2 for i in range(N_POINTS - 1)] + [width]
    depth_u16 = np.zeros((height, width), dtype=np.uint16)
    for i in range(N_POINTS):
        c0, c1 = boundaries[i], boundaries[i + 1]
        depth_u16[:, c0:c1] = int(round(valid_depth_m / SCALE))
    patch = 12
    for idx, col in enumerate(cols):
        if idx in invalid_indices:
            r0, r1 = max(0, ROW - patch), min(height, ROW + patch + 1)
            c0, c1 = max(0, col - patch), min(width, col + patch + 1)
            depth_u16[r0:r1, c0:c1] = 0
    return depth_u16, height, width


def _point_dict(row: int, col: int) -> dict:
    return {"row": row, "col": col, "u": col, "v": row}


def _write_paths_json(root: Path, case_id: str, height: int, width: int) -> None:
    cols = _cols()
    main_dense = [_point_dict(ROW, c) for c in cols]
    # Far enough from the image edges that the MC cross-check's own pixel-noise perturbation
    # (which needs a few extra px of margin beyond the real lift's own annulus_outer_px) never
    # pushes a perturbed sample off the synthetic patch buffer (uncertainty_mc.py's own margin).
    branch_dense = [_point_dict(30, cols[i]) for i in range(3)]
    doc = {
        "case_id": case_id,
        "schema_version": 1,
        "image_height": height,
        "image_width": width,
        "min_spur_length_px": 5,
        "rdp_tolerance_px": 1.5,
        "component_count": 1,
        "components": [
            {
                "order_index": 0,
                "component_id": 0,
                "main_path": {
                    "kind": "main",
                    "length_px": len(main_dense) - 1,
                    "dense": main_dense,
                    "simplified": main_dense,
                },
                "branches": [
                    {
                        "kind": "branch",
                        "length_px": len(branch_dense) - 1,
                        "dense": branch_dense,
                        "simplified": branch_dense,
                    }
                ],
            }
        ],
    }
    path = root / "data" / "paths" / f"{case_id}_paths.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc), encoding="utf-8")


def _write_mask(root: Path, case_id: str, height: int, width: int) -> None:
    mask_path = root / naming.mask_path(case_id)
    mask_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.zeros((height, width), dtype=np.uint8)).save(mask_path)


def _write_depth_png(root: Path, rel_path: str, depth_u16: np.ndarray) -> None:
    path = root / rel_path
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(depth_u16, mode="I;16").save(path)


def _color_intrinsics(width: int, height: int) -> dict:
    return {
        "width": width,
        "height": height,
        "fx": 500.0,
        "fy": 500.0,
        "ppx": width / 2.0,
        "ppy": height / 2.0,
        "model": "none",
        "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    }


def _write_capture_record(root: Path, case_id: str, height: int, width: int, *, ee) -> Path:
    optical_xyz, optical_quat = kin.transform_to_xyz_quat(
        kin.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME
    )
    depth_rel = f"data/d405/depth/{case_id}_depth.png"
    doc = cr.build_record(
        case_id=case_id,
        synthetic=True,
        image_height=height,
        image_width=width,
        color_intrinsics=_color_intrinsics(width, height),
        depth_file=depth_rel,
        depth_scale_m_per_unit=SCALE,
        depth_aligned_to="color",
        source_kind="synthetic",
        source_ref="synthetic",
        source_frame_index=None,
        capture_stamp_ns=1_000_000_000_000,
        joint_names=list(kin.load_chain().joint_names),
        # Comfortably inside every joint's limits (not at a boundary like q=0 for joint2/3,
        # whose upper limit is exactly 0.0) -- the MC cross-check perturbs each joint by
        # sigma_joint_rad and a boundary position would spuriously reject valid perturbations.
        positions_rad=[0.1, -0.3, -0.3, 0.1, 0.1, 0.1],
        robot_stamp_ns=1_000_000_000_000,
        robot_stamp_source="/joint_states",
        kinematic_model_path=str(kin.DEFAULT_URDF),
        kinematic_model_sha256=kin.file_sha256(kin.DEFAULT_URDF),
        end_effector_path="config/robot/end_effector.yaml",
        end_effector_sha256=ee.sha256,
        wrist_camera_value_status=ee.wrist_camera_value_status,
        tool_value_status=ee.tool_value_status,
        optical_xyz_m=list(optical_xyz),
        optical_quat_xyzw=list(optical_quat),
        optical_source="nominal_d405",
    )
    path = cr.capture_record_path(root, case_id)
    cr.write_record(doc, path)
    return path


def _write_case_map(root: Path, case_id: str) -> None:
    path = root / "data" / "case_map.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"cases": [{"case_id": case_id, "downscaled": False}]}), encoding="utf-8")


def _build_paths3d_case(tmp_project: Config, case_id: str = "case_a") -> None:
    """Build a full paths.json + mask + depth + capture record case, then run the real
    `path3d` CLI to produce `data/paths3d/{case_id}_paths3d.json`."""
    depth_u16, height, width = _island_depth_image(INVALID_SHORT | INVALID_LONG)
    _write_paths_json(tmp_project.root, case_id, height, width)
    _write_mask(tmp_project.root, case_id, height, width)
    _write_depth_png(tmp_project.root, f"data/d405/depth/{case_id}_depth.png", depth_u16)
    ee = kin.load_end_effector()
    _write_capture_record(tmp_project.root, case_id, height, width, ee=ee)
    _write_case_map(tmp_project.root, case_id)

    # A larger waypoint spacing keeps the number of resampled waypoints (and hence MC calls in
    # the tests below) small without affecting anything this card's acceptance criteria check.
    exit_code = path3d.main(["--root", str(tmp_project.root), "--waypoint-spacing-m", "0.01"])
    assert exit_code == EXIT_OK


def _run(tmp_project: Config, *extra_args: str) -> int:
    return main(["--root", str(tmp_project.root), *extra_args])


def _load_report(tmp_project: Config, case_id: str = "case_a") -> dict:
    return json.loads(uncertainty_budget_path(tmp_project.root, case_id).read_text(encoding="utf-8"))


def _write_small_mc_config(root: Path, *, agreement_band: list[float] | None = None) -> None:
    cfg_dir = root / "config" / "calibration"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    doc = {
        "schema": "crackvision.uncertainty_budget_config/1",
        "mc": {
            "n_samples": 20,
            "seed": 0,
            "subsample": 3,
            "agreement_band": agreement_band if agreement_band is not None else [0.5, 2.0],
        },
    }
    (cfg_dir / "uncertainty_budget.yaml").write_text(json.dumps(doc), encoding="utf-8")


# ---------------------------------------------------------------------------
# Full key set + arithmetic identities + clean pass
# ---------------------------------------------------------------------------

_TOP_LEVEL_KEYS = {
    "schema", "case_id", "created_utc", "producer", "frame", "sources", "parameters",
    "points", "waypoints", "summary", "caveats",
}
_SOURCES_KEYS = {"paths3d", "capture_record", "end_effector", "limits", "config"}
_PARAMETERS_KEYS = {"sigma_joint_rad", "sigma_joint_rad_source", "mc"}
_MC_PARAM_KEYS = {"n_samples", "seed", "subsample", "agreement_band"}
_POINT_KEYS = {
    "component_id", "polyline_kind", "branch_index", "point_index", "row", "col",
    "sigma_total_m", "contributions_m2", "mc", "mc_analytic_agreement",
}
_POINT_CONTRIB_KEYS = {"pixel", "depth", "handeye", "fk"}
_POINT_MC_KEYS = {"sigma_total_m", "n_samples_used"}
_POINT_AGREEMENT_KEYS = {"ratio_trace", "ratio_eig_max", "flagged"}
_WAYPOINT_KEYS = {
    "component_id", "polyline_kind", "branch_index", "segment_index", "phase", "waypoint_index",
    "sigma_along_normal_base_m", "sigma_along_normal_m", "contributions_m2", "mc",
    "mc_analytic_agreement", "within_budget",
}
_WAYPOINT_CONTRIB_KEYS = {"pixel", "depth", "handeye", "tcp", "fk"}
_WAYPOINT_MC_KEYS = {"sigma_along_normal_m"}
_WAYPOINT_AGREEMENT_KEYS = {"ratio_along_normal", "flagged"}
_SUMMARY_KEYS = {"points_flagged", "waypoints_flagged", "overall_pass"}
_CAVEAT_TEXT = (
    "Camera intrinsics uncertainty is not budgeted as a separate additive term; it is only "
    "indirectly exercised by the Monte-Carlo resample's nonlinear deprojection/annulus geometry "
    "(ADR-018 §2)."
)


def test_full_schema_and_overall_pass_true(tmp_project: Config):
    _build_paths3d_case(tmp_project)
    _write_small_mc_config(tmp_project.root)

    exit_code = _run(tmp_project, "--cases", "case_a")
    assert exit_code == EXIT_OK
    report = _load_report(tmp_project)

    assert set(report.keys()) == _TOP_LEVEL_KEYS
    assert report["schema"] == "crackvision.uncertainty_budget/1"
    assert report["case_id"] == "case_a"
    assert report["frame"] == "base_link"
    assert set(report["sources"].keys()) == _SOURCES_KEYS
    assert set(report["sources"]["paths3d"].keys()) == {"path", "sha256"}
    assert set(report["sources"]["capture_record"].keys()) == {"path", "sha256"}
    assert set(report["sources"]["end_effector"].keys()) == {"path", "sha256"}
    assert set(report["parameters"].keys()) == _PARAMETERS_KEYS
    assert set(report["parameters"]["mc"].keys()) == _MC_PARAM_KEYS
    assert set(report["summary"].keys()) == _SUMMARY_KEYS
    assert report["caveats"] == [_CAVEAT_TEXT]

    assert len(report["points"]) > 0
    assert len(report["waypoints"]) > 0

    for pt in report["points"]:
        assert set(pt.keys()) == _POINT_KEYS
        assert set(pt["contributions_m2"].keys()) == _POINT_CONTRIB_KEYS
        assert set(pt["mc"].keys()) == _POINT_MC_KEYS
        assert set(pt["mc_analytic_agreement"].keys()) == _POINT_AGREEMENT_KEYS

        total_diag = np.zeros(3)
        for term in _POINT_CONTRIB_KEYS:
            mat = np.asarray(pt["contributions_m2"][term]).reshape(3, 3)
            total_diag += np.diag(mat)
        sigma_total_m = np.asarray(pt["sigma_total_m"])
        assert np.allclose(np.sqrt(np.maximum(total_diag, 0.0)), sigma_total_m, atol=1e-9)

    for wp in report["waypoints"]:
        assert set(wp.keys()) == _WAYPOINT_KEYS
        assert set(wp["contributions_m2"].keys()) == _WAYPOINT_CONTRIB_KEYS
        assert set(wp["mc"].keys()) == _WAYPOINT_MC_KEYS
        assert set(wp["mc_analytic_agreement"].keys()) == _WAYPOINT_AGREEMENT_KEYS

        total = sum(wp["contributions_m2"][term] for term in _WAYPOINT_CONTRIB_KEYS)
        assert abs(np.sqrt(total) - wp["sigma_along_normal_m"]) < 1e-9
        assert wp["sigma_along_normal_m"] >= wp["sigma_along_normal_base_m"] - 1e-9

    assert report["summary"]["overall_pass"] is True
    assert report["summary"]["points_flagged"] == 0
    assert report["summary"]["waypoints_flagged"] == 0


def test_sigma_total_at_least_source_sigma_base(tmp_project: Config):
    _build_paths3d_case(tmp_project)
    _write_small_mc_config(tmp_project.root)
    assert _run(tmp_project, "--cases", "case_a") == EXIT_OK

    paths3d_doc = json.loads(
        (tmp_project.root / "data" / "paths3d" / "case_a_paths3d.json").read_text(encoding="utf-8")
    )
    report = _load_report(tmp_project)

    main_pl = paths3d_doc["components"][0]["polylines"][0]
    for pt in report["points"]:
        if pt["polyline_kind"] != "main" or pt["branch_index"] is not None:
            continue
        src_point = main_pl["points"][pt["point_index"]]
        sigma_base = np.asarray(src_point["sigma_base_m"])
        sigma_total = np.asarray(pt["sigma_total_m"])
        assert np.all(sigma_total >= sigma_base - 1e-9)


# ---------------------------------------------------------------------------
# Forced disagreement (tight agreement_band) -> non-zero exit, file still written
# ---------------------------------------------------------------------------


def test_forced_disagreement_nonzero_exit_but_file_written(tmp_project: Config):
    _build_paths3d_case(tmp_project, case_id="case_b")
    _write_small_mc_config(tmp_project.root, agreement_band=[0.99, 1.01])

    exit_code = _run(tmp_project, "--cases", "case_b")
    assert exit_code != EXIT_OK
    assert exit_code == EXIT_RUNTIME

    out_path = uncertainty_budget_path(tmp_project.root, "case_b")
    assert out_path.is_file()
    report = json.loads(out_path.read_text(encoding="utf-8"))
    assert set(report.keys()) == _TOP_LEVEL_KEYS
    assert report["summary"]["overall_pass"] is False
    assert report["summary"]["points_flagged"] + report["summary"]["waypoints_flagged"] > 0


# ---------------------------------------------------------------------------
# §0 common flags / config / refusals
# ---------------------------------------------------------------------------


def test_dry_run_writes_nothing(tmp_project: Config):
    _build_paths3d_case(tmp_project)
    _write_small_mc_config(tmp_project.root)
    exit_code = _run(tmp_project, "--dry-run")
    assert exit_code == EXIT_OK
    assert not uncertainty_budget_path(tmp_project.root, "case_a").exists()


def test_skip_existing(tmp_project: Config):
    _build_paths3d_case(tmp_project)
    _write_small_mc_config(tmp_project.root)
    assert _run(tmp_project, "--cases", "case_a") == EXIT_OK
    out_path = uncertainty_budget_path(tmp_project.root, "case_a")
    out_path.write_text("sentinel", encoding="utf-8")

    exit_code = _run(tmp_project, "--cases", "case_a", "--skip-existing")
    assert exit_code == EXIT_OK
    assert out_path.read_text(encoding="utf-8") == "sentinel"


def test_refusal_missing_paths3d(tmp_project: Config):
    exit_code = _run(tmp_project, "--cases", "no_such_case")
    assert exit_code == EXIT_PRECONDITION


def test_bad_budget_config_exit_2(tmp_project: Config):
    _build_paths3d_case(tmp_project)
    cfg_dir = tmp_project.root / "config" / "calibration"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    (cfg_dir / "uncertainty_budget.yaml").write_text(
        json.dumps({"schema": "crackvision.uncertainty_budget_config/1", "mc": {"bogus_key": 1}}),
        encoding="utf-8",
    )
    exit_code = _run(tmp_project, "--cases", "case_a")
    assert exit_code == EXIT_USAGE


def test_bad_project_config_exit_2(tmp_path: Path):
    bad_config = tmp_path / "bad.yaml"
    bad_config.write_text("not: valid: yaml: [", encoding="utf-8")
    exit_code = main(["--root", str(tmp_path), "--config", str(bad_config)])
    assert exit_code == EXIT_USAGE


def test_cli_help_exits_0():
    with pytest.raises(SystemExit) as exc_info:
        main(["--help"])
    assert exc_info.value.code == EXIT_OK


def test_build_parser_smoke():
    parser = build_parser()
    args = parser.parse_args(["--cases", "a", "b"])
    assert args.cases == ["a", "b"]


def test_no_config_file_uses_section_14_6_defaults(tmp_path: Path):
    """Absence of uncertainty_budget.yaml is not an error; §14.6 defaults apply."""
    from crackvision.uncertainty_budget import load_budget_config

    budget_cfg = load_budget_config(tmp_path / "does_not_exist.yaml")
    assert budget_cfg.mc.n_samples == 2000
    assert budget_cfg.mc.seed == 0
    assert budget_cfg.mc.subsample == "all"
    assert budget_cfg.mc.agreement_band == (0.5, 2.0)
    assert budget_cfg.sigma_joint_rad_override is None


def test_config_absent_in_report_sources(tmp_project: Config):
    _build_paths3d_case(tmp_project)
    _write_small_mc_config(tmp_project.root)
    assert _run(tmp_project, "--cases", "case_a") == EXIT_OK
    report = _load_report(tmp_project)
    assert set(report["sources"]["config"].keys()) == {"path", "sha256"}
    assert report["sources"]["limits"] is None
