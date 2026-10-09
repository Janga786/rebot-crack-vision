"""tests/test_path3d_cli.py — GEOM-08.7 crackvision.path3d CLI.

Builds a throwaway `paths.json` (dense points authored directly, "island" per-point depth
patches mirroring tests/test_lift3d.py's technique) plus a §9 capture record assembled with
`capture_record.build_record` at `q = 0`, then exercises the full CLI: schema key set,
component/polyline ordering, segment split, analytic position check, eligibility reasons, every
§9.4-style refusal, and the §0 common flags.
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
from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE
from crackvision.path3d import build_parser, main, paths3d_path

SCALE = 1e-4
SPACING = 30
N_POINTS = 28
ROW = 50
INVALID_SHORT = set(range(5, 8))  # 3-point run -> interpolated (<= default max_gap_px=5)
INVALID_LONG = set(range(13, 23))  # 10-point run -> splits (> default max_gap_px=5)


def _cols() -> list[int]:
    return [SPACING * (i + 1) for i in range(N_POINTS)]


def _island_depth_image(invalid_indices: set[int], valid_depth_m: float = 0.25):
    """Per-point column "islands" far enough apart that each point's own annulus (outer_px=8)
    never crosses into a neighbouring island -- mirrors tests/test_lift3d.py's technique."""
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
    branch_dense = [_point_dict(10, cols[i]) for i in range(3)]
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


def _write_capture_record(
    root: Path,
    case_id: str,
    height: int,
    width: int,
    *,
    ee,
    synthetic: bool = True,
    ee_sha: str | None = None,
    robot_block: bool = True,
) -> Path:
    optical_xyz, optical_quat = kin.transform_to_xyz_quat(
        kin.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME
    )
    depth_rel = f"data/d405/depth/{case_id}_depth.png"
    doc = cr.build_record(
        case_id=case_id,
        synthetic=synthetic,
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
        positions_rad=[0.0] * 6,
        robot_stamp_ns=1_000_000_000_000,
        robot_stamp_source="/joint_states",
        kinematic_model_path=str(kin.DEFAULT_URDF),
        kinematic_model_sha256=kin.file_sha256(kin.DEFAULT_URDF),
        end_effector_path="config/robot/end_effector.yaml",
        end_effector_sha256=ee_sha if ee_sha is not None else ee.sha256,
        wrist_camera_value_status=ee.wrist_camera_value_status,
        tool_value_status=ee.tool_value_status,
        optical_xyz_m=list(optical_xyz),
        optical_quat_xyzw=list(optical_quat),
        optical_source="nominal_d405",
    )
    if not robot_block:
        del doc["robot"]
    path = cr.capture_record_path(root, case_id)
    cr.write_record(doc, path)
    return path


def _write_case_map(root: Path, case_id: str, *, downscaled: bool = False) -> None:
    path = root / "data" / "case_map.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"cases": [{"case_id": case_id, "downscaled": downscaled}]}), encoding="utf-8")


def _build_valid_case(tmp_project: Config, case_id: str = "case_a", **capture_kwargs) -> tuple[int, int]:
    depth_u16, height, width = _island_depth_image(INVALID_SHORT | INVALID_LONG)
    _write_paths_json(tmp_project.root, case_id, height, width)
    _write_mask(tmp_project.root, case_id, height, width)
    _write_depth_png(tmp_project.root, f"data/d405/depth/{case_id}_depth.png", depth_u16)
    ee = kin.load_end_effector()
    _write_capture_record(tmp_project.root, case_id, height, width, ee=ee, **capture_kwargs)
    _write_case_map(tmp_project.root, case_id)
    return height, width


def _run(tmp_project: Config, *extra_args: str) -> int:
    return main(["--root", str(tmp_project.root), *extra_args])


def _load_doc(tmp_project: Config, case_id: str = "case_a") -> dict:
    return json.loads(paths3d_path(tmp_project.root, case_id).read_text(encoding="utf-8"))


# ---------------------------------------------------------------------------
# Full key set + ordering
# ---------------------------------------------------------------------------

_TOP_LEVEL_KEYS = {
    "schema", "case_id", "frame", "image_height", "image_width", "sources", "parameters",
    "calibration", "uncertainty_model", "execution_eligible", "ineligible_reasons", "counts",
    "components",
}
_SOURCES_KEYS = {
    "paths_json", "paths_json_sha256", "capture_record", "capture_record_sha256", "mask",
    "mask_sha256", "end_effector", "urdf_sha256",
}
_PARAMETERS_KEYS = {
    "annulus_inner_px", "annulus_outer_px", "annulus_min_fraction_valid", "valid_depth_range_m",
    "max_gap_px", "sigma_px", "waypoint_spacing_m", "approach_retract_offset_m",
    "trace_clearance_m", "roll_free", "max_clock_skew_s",
}
_CALIBRATION_BLOCK_KEYS = {"value_status", "position_sigma_m", "rotation_sigma_rad", "source"}
_TOOL_CALIBRATION_KEYS = {"value_status", "position_sigma_m", "source"}
_COMPONENT_KEYS = {"order_index", "component_id", "polylines"}
_POLYLINE_KEYS = {"kind", "branch_index", "points", "segments"}
_POINT_KEYS = {
    "row", "col", "u", "v", "valid", "reason", "interpolated", "depth_m", "fraction_valid",
    "p_optical_m", "p_base_m", "n_base", "normal_rms_m", "sigma_normal_rad", "cov_base_m2",
    "sigma_base_m",
}
_SEGMENT_KEYS = {"start_index", "end_index", "waypoints"}
_WAYPOINT_KEYS = {
    "phase", "position_m", "quat_xyzw", "surface_point_m", "normal_base", "clearance_m",
    "sigma_along_normal_m", "sigma_max_m", "interpolated", "within_budget",
}


def test_full_key_set_and_order(tmp_project: Config):
    _build_valid_case(tmp_project)
    assert _run(tmp_project) == EXIT_OK
    doc = _load_doc(tmp_project)

    assert set(doc.keys()) == _TOP_LEVEL_KEYS
    assert doc["schema"] == "crackvision.paths3d/1"
    assert doc["frame"] == "base_link"
    assert set(doc["sources"].keys()) == _SOURCES_KEYS
    assert set(doc["sources"]["end_effector"].keys()) == {"sha256_at_capture", "sha256_now"}
    assert set(doc["parameters"].keys()) == _PARAMETERS_KEYS
    assert set(doc["calibration"]["wrist_camera"].keys()) == _CALIBRATION_BLOCK_KEYS
    assert set(doc["calibration"]["tool"].keys()) == _TOOL_CALIBRATION_KEYS
    assert set(doc["uncertainty_model"].keys()) == {"terms", "unmodelled"}
    assert set(doc["counts"].keys()) == {"points", "valid", "interpolated", "invalid_by_reason"}

    assert len(doc["components"]) == 1
    comp = doc["components"][0]
    assert set(comp.keys()) == _COMPONENT_KEYS
    assert comp["order_index"] == 0
    assert comp["component_id"] == 0

    # polyline order mirrors paths.json: main first (branch_index null), then branches in order
    assert len(comp["polylines"]) == 2
    main_pl, branch_pl = comp["polylines"]
    assert main_pl["kind"] == "main" and main_pl["branch_index"] is None
    assert branch_pl["kind"] == "branch" and branch_pl["branch_index"] == 0

    for pl in comp["polylines"]:
        assert set(pl.keys()) == _POLYLINE_KEYS
        assert len(pl["points"]) >= 1
        for pt in pl["points"]:
            assert set(pt.keys()) == _POINT_KEYS
        for seg in pl["segments"]:
            assert set(seg.keys()) == _SEGMENT_KEYS
            for wp in seg["waypoints"]:
                assert set(wp.keys()) == _WAYPOINT_KEYS

    # main path point order equals paths.json dense order (same row, ascending columns)
    cols = _cols()
    for i, pt in enumerate(main_pl["points"]):
        assert pt["row"] == ROW
        assert pt["col"] == cols[i]


# ---------------------------------------------------------------------------
# Analytic position check
# ---------------------------------------------------------------------------


def test_position_within_1mm_of_analytic(tmp_project: Config):
    _build_valid_case(tmp_project)
    assert _run(tmp_project) == EXIT_OK
    doc = _load_doc(tmp_project)
    main_pl = doc["components"][0]["polylines"][0]

    cols = _cols()
    row, col = ROW, cols[0]
    width_total = SPACING * (N_POINTS + 2)
    height_total = 100
    intr = _color_intrinsics(width_total, height_total)
    x = (col - intr["ppx"]) / intr["fx"]
    y = (row - intr["ppy"]) / intr["fy"]
    z = 0.25
    p_optical = np.array([x * z, y * z, z])

    ee = kin.load_end_effector()
    chain = kin.load_chain()
    T_fk = chain.fk([0.0] * 6)
    T_base_optical = (
        T_fk @ ee.T_gripper_link_camera_link @ kin.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME
    )
    p_base_expected = (T_base_optical @ np.array([*p_optical, 1.0]))[:3]

    pt0 = main_pl["points"][0]
    assert pt0["valid"] is True
    p_base_actual = np.array(pt0["p_base_m"])
    assert np.allclose(p_base_actual, p_base_expected, atol=1e-3)


# ---------------------------------------------------------------------------
# Segment split / gap policy wiring
# ---------------------------------------------------------------------------


def test_segment_split_and_interpolation(tmp_project: Config):
    _build_valid_case(tmp_project)
    assert _run(tmp_project) == EXIT_OK
    doc = _load_doc(tmp_project)
    main_pl = doc["components"][0]["polylines"][0]

    for i in INVALID_SHORT:
        pt = main_pl["points"][i]
        assert pt["valid"] is True
        assert pt["interpolated"] is True
        assert pt["reason"] is None

    for i in INVALID_LONG:
        pt = main_pl["points"][i]
        assert pt["valid"] is False
        assert pt["interpolated"] is False
        assert pt["reason"] is not None
        assert pt["p_base_m"] is None
        assert pt["cov_base_m2"] is None

    segments = main_pl["segments"]
    assert [(s["start_index"], s["end_index"]) for s in segments] == [(0, 12), (23, 27)]
    for seg in segments:
        phases = [wp["phase"] for wp in seg["waypoints"]]
        assert phases[0] == "approach"
        assert phases[-1] == "retract"
        assert all(p == "trace" for p in phases[1:-1])


# ---------------------------------------------------------------------------
# Eligibility
# ---------------------------------------------------------------------------


def test_eligibility_reasons_today_nominal_and_synthetic(tmp_project: Config):
    _build_valid_case(tmp_project, synthetic=True)
    assert _run(tmp_project) == EXIT_OK
    doc = _load_doc(tmp_project)

    assert doc["execution_eligible"] is False
    assert "wrist_camera_nominal" in doc["ineligible_reasons"]
    assert "tool_nominal" in doc["ineligible_reasons"]
    assert "synthetic_capture" in doc["ineligible_reasons"]


def test_end_effector_change_reason_and_override(tmp_project: Config):
    case_id = "case_b"
    depth_u16, height, width = _island_depth_image(set())
    _write_paths_json(tmp_project.root, case_id, height, width)
    _write_mask(tmp_project.root, case_id, height, width)
    _write_depth_png(tmp_project.root, f"data/d405/depth/{case_id}_depth.png", depth_u16)
    ee = kin.load_end_effector()
    _write_capture_record(tmp_project.root, case_id, height, width, ee=ee, ee_sha="0" * 64)
    _write_case_map(tmp_project.root, case_id)

    # without override: refused
    exit_code = _run(tmp_project, "--cases", case_id)
    assert exit_code == EXIT_PRECONDITION
    latest = json.loads((tmp_project.paths["logs"] / "path3d_latest.json").read_text())
    assert any("capture_record_invalid" in e for e in latest["errors"])

    # with override: accepted, flagged ineligible
    exit_code = _run(tmp_project, "--cases", case_id, "--accept-end-effector-change")
    assert exit_code == EXIT_OK
    doc = _load_doc(tmp_project, case_id)
    assert "end_effector_changed_since_capture" in doc["ineligible_reasons"]
    assert doc["sources"]["end_effector"]["sha256_at_capture"] == "0" * 64
    assert doc["sources"]["end_effector"]["sha256_now"] == ee.sha256


# ---------------------------------------------------------------------------
# Refusals
# ---------------------------------------------------------------------------


def test_refusal_missing_capture_record(tmp_project: Config):
    case_id = "case_missing"
    depth_u16, height, width = _island_depth_image(set())
    _write_paths_json(tmp_project.root, case_id, height, width)
    _write_mask(tmp_project.root, case_id, height, width)
    _write_case_map(tmp_project.root, case_id)

    exit_code = _run(tmp_project, "--cases", case_id)
    assert exit_code == EXIT_PRECONDITION
    latest = json.loads((tmp_project.paths["logs"] / "path3d_latest.json").read_text())
    assert any("capture_record_missing" in e for e in latest["errors"])
    assert not paths3d_path(tmp_project.root, case_id).exists()


def test_refusal_no_robot_block(tmp_project: Config):
    case_id = "case_2d_only"
    depth_u16, height, width = _island_depth_image(set())
    _write_paths_json(tmp_project.root, case_id, height, width)
    _write_mask(tmp_project.root, case_id, height, width)
    _write_depth_png(tmp_project.root, f"data/d405/depth/{case_id}_depth.png", depth_u16)
    ee = kin.load_end_effector()
    _write_capture_record(tmp_project.root, case_id, height, width, ee=ee, robot_block=False)
    _write_case_map(tmp_project.root, case_id)

    exit_code = _run(tmp_project, "--cases", case_id)
    assert exit_code == EXIT_PRECONDITION
    latest = json.loads((tmp_project.paths["logs"] / "path3d_latest.json").read_text())
    assert any("capture_record_invalid" in e for e in latest["errors"])


def test_refusal_downscaled_case(tmp_project: Config):
    case_id = "case_downscaled"
    depth_u16, height, width = _island_depth_image(set())
    _write_paths_json(tmp_project.root, case_id, height, width)
    _write_mask(tmp_project.root, case_id, height, width)
    _write_depth_png(tmp_project.root, f"data/d405/depth/{case_id}_depth.png", depth_u16)
    ee = kin.load_end_effector()
    _write_capture_record(tmp_project.root, case_id, height, width, ee=ee)
    _write_case_map(tmp_project.root, case_id, downscaled=True)

    exit_code = _run(tmp_project, "--cases", case_id)
    assert exit_code == EXIT_PRECONDITION
    latest = json.loads((tmp_project.paths["logs"] / "path3d_latest.json").read_text())
    assert any("downscaled_case" in e for e in latest["errors"])


def test_refusal_image_shape_mismatch(tmp_project: Config):
    case_id = "case_shape_mismatch"
    depth_u16, height, width = _island_depth_image(set())
    _write_paths_json(tmp_project.root, case_id, height, width)
    # mask with the wrong shape: trips capture_record's §0.5 consistency check
    mask_path = tmp_project.root / naming.mask_path(case_id)
    mask_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.zeros((10, 10), dtype=np.uint8)).save(mask_path)
    _write_depth_png(tmp_project.root, f"data/d405/depth/{case_id}_depth.png", depth_u16)
    ee = kin.load_end_effector()
    _write_capture_record(tmp_project.root, case_id, height, width, ee=ee)
    _write_case_map(tmp_project.root, case_id)

    exit_code = _run(tmp_project, "--cases", case_id)
    assert exit_code == EXIT_PRECONDITION
    latest = json.loads((tmp_project.paths["logs"] / "path3d_latest.json").read_text())
    assert any("case_shape_mismatch" in e for e in latest["errors"])


# ---------------------------------------------------------------------------
# §0 common flags + logs
# ---------------------------------------------------------------------------


def test_dry_run_writes_nothing(tmp_project: Config):
    _build_valid_case(tmp_project)
    exit_code = _run(tmp_project, "--dry-run")
    assert exit_code == EXIT_OK
    assert not paths3d_path(tmp_project.root, "case_a").exists()


def test_skip_existing(tmp_project: Config):
    _build_valid_case(tmp_project)
    assert _run(tmp_project) == EXIT_OK
    out_path = paths3d_path(tmp_project.root, "case_a")
    out_path.write_text("sentinel", encoding="utf-8")

    exit_code = _run(tmp_project, "--skip-existing")
    assert exit_code == EXIT_OK
    assert out_path.read_text(encoding="utf-8") == "sentinel"


def test_logs_written(tmp_project: Config):
    _build_valid_case(tmp_project)
    assert _run(tmp_project) == EXIT_OK
    latest_path = tmp_project.paths["logs"] / "path3d_latest.json"
    assert latest_path.is_file()
    latest = json.loads(latest_path.read_text())
    assert latest["tool"] == "path3d"
    assert latest["status"] == "ok"
    assert latest["exit_code"] == EXIT_OK


def test_root_flag_and_config_override(tmp_project: Config):
    _build_valid_case(tmp_project)
    exit_code = main(
        ["--root", str(tmp_project.root), "--config", str(tmp_project.root / "config" / "project.yaml")]
    )
    assert exit_code == EXIT_OK


def test_mixed_cases_partial(tmp_project: Config):
    _build_valid_case(tmp_project, case_id="case_ok")
    _write_case_map(tmp_project.root, "case_ok")
    case_map_path = tmp_project.root / "data" / "case_map.json"
    case_map = json.loads(case_map_path.read_text())
    case_map["cases"].append({"case_id": "case_bad", "downscaled": True})
    case_map_path.write_text(json.dumps(case_map), encoding="utf-8")

    exit_code = _run(tmp_project)
    assert exit_code == EXIT_RUNTIME


def test_bad_config_exit_2(tmp_path: Path):
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
    args = parser.parse_args(["--cases", "a", "b", "--trace-clearance-m", "0.02"])
    assert args.cases == ["a", "b"]
    assert args.trace_clearance_m == 0.02
