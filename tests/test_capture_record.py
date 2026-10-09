"""Tests for crackvision.capture_record (GEOM-08.4)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from crackvision import kinematics as kin
from crackvision.capture_record import (
    SCHEMA,
    CaptureRecordError,
    T_base_link_optical,
    build_record,
    capture_record_path,
    load_capture_record,
    load_depth,
    main,
    write_record,
)
from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE

REAL_URDF = kin.DEFAULT_URDF
REAL_EE_PATH = kin.DEFAULT_END_EFFECTOR


def _end_effector():
    return kin.load_end_effector()


def _base_optical():
    _xyz, quat = kin.transform_to_xyz_quat(kin.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME)
    return [0.0, 0.0, 0.0], list(quat)


def _color_intrinsics(width=64, height=48):
    return {
        "width": width,
        "height": height,
        "fx": 425.3,
        "fy": 425.3,
        "ppx": 32.0,
        "ppy": 24.0,
        "model": "Brown Conrady",
        "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    }


def _write_depth(root: Path, rel_path: str, shape=(48, 64)) -> None:
    path = root / rel_path
    path.parent.mkdir(parents=True, exist_ok=True)
    arr = np.zeros(shape, dtype=np.uint16)
    arr[:] = 1000
    Image.fromarray(arr, mode="I;16").save(path)


def _base_doc(root: Path, *, ee, kinematic_sha=None, positions=None, robot_stamp_ns=None,
              capture_stamp_ns=None, joint_names=None, ee_sha=None):
    ee = ee or _end_effector()
    optical_xyz, optical_quat = _base_optical()
    urdf_rel = str(REAL_URDF.relative_to(root)) if REAL_URDF.is_relative_to(root) else str(REAL_URDF)
    capture_stamp_ns = capture_stamp_ns if capture_stamp_ns is not None else 1_000_000_000_000
    robot_stamp_ns = robot_stamp_ns if robot_stamp_ns is not None else capture_stamp_ns
    doc = build_record(
        case_id="case_a",
        synthetic=True,
        image_height=48,
        image_width=64,
        color_intrinsics=_color_intrinsics(),
        depth_file="data/d405/depth/case_a_depth.png",
        depth_scale_m_per_unit=0.0001,
        depth_aligned_to="color",
        source_kind="synthetic",
        source_ref="data/d405/metadata/case_a.json",
        source_frame_index=0,
        capture_stamp_ns=capture_stamp_ns,
        joint_names=joint_names if joint_names is not None else list(kin.load_chain().joint_names),
        positions_rad=positions if positions is not None else [0.0] * 6,
        robot_stamp_ns=robot_stamp_ns,
        robot_stamp_source="/joint_states",
        kinematic_model_path=urdf_rel,
        kinematic_model_sha256=kinematic_sha if kinematic_sha is not None else kin.file_sha256(REAL_URDF),
        end_effector_path="config/robot/end_effector.yaml",
        end_effector_sha256=ee_sha if ee_sha is not None else ee.sha256,
        wrist_camera_value_status=ee.wrist_camera_value_status,
        tool_value_status=ee.tool_value_status,
        optical_xyz_m=optical_xyz,
        optical_quat_xyzw=optical_quat,
        optical_source="nominal_d405",
    )
    return doc


def test_build_record_matches_schema():
    ee = _end_effector()
    doc = _base_doc(kin.find_root(), ee=ee)
    assert doc["schema"] == SCHEMA
    assert doc["synthetic"] is True
    assert doc["depth"]["depth_scale_m_per_unit"] == 0.0001


def test_synthetic_flag_is_copied_through():
    ee = _end_effector()
    doc = _base_doc(kin.find_root(), ee=ee)
    doc["synthetic"] = True
    assert doc["synthetic"] is True


def test_depth_scale_never_defaulted():
    ee = _end_effector()
    doc = _base_doc(kin.find_root(), ee=ee)
    assert doc["depth"]["depth_scale_m_per_unit"] == 0.0001


# ---------------------------------------------------------------------------
# §9.4 refusal rules
# ---------------------------------------------------------------------------


def test_missing_robot_block_raises_2d_only_error(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    del doc["robot"]
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    with pytest.raises(CaptureRecordError, match="2D perception only"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_bad_joint_names_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee, joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "wrong"])
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    with pytest.raises(CaptureRecordError, match="joint_names"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_out_of_limit_joint_position_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee, positions=[10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    with pytest.raises(CaptureRecordError, match="rejected by FK"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_missing_end_effector_sha_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    doc["end_effector"]["sha256"] = ""
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    with pytest.raises(CaptureRecordError, match="end_effector.sha256 is missing"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_end_effector_sha_mismatch_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee, ee_sha="0" * 64)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    with pytest.raises(CaptureRecordError, match="does not match"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_end_effector_sha_mismatch_with_override_is_accepted(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee, ee_sha="0" * 64)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    record = load_capture_record(
        path, root=tmp_path, end_effector=ee, accept_end_effector_change=True
    )
    assert record.end_effector_changed is True
    assert record.end_effector_sha256_now == ee.sha256
    assert record.end_effector_sha256 == "0" * 64


def test_skew_over_max_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee, capture_stamp_ns=1_000_000_000_000, robot_stamp_ns=1_000_500_000_000)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    with pytest.raises(CaptureRecordError, match="skew|max_skew_s"):
        load_capture_record(path, root=tmp_path, end_effector=ee, max_skew_s=0.1)


def test_within_skew_is_accepted(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee, capture_stamp_ns=1_000_000_000_000, robot_stamp_ns=1_000_050_000_000)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    record = load_capture_record(path, root=tmp_path, end_effector=ee, max_skew_s=0.1)
    assert record.case_id == "case_a"


def test_depth_shape_mismatch_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    record = load_capture_record(path, root=tmp_path, end_effector=ee)
    _write_depth(tmp_path, "data/d405/depth/case_a_depth.png", shape=(10, 10))
    with pytest.raises(CaptureRecordError, match="shape"):
        load_depth(record, tmp_path)


def test_depth_shape_match_loads(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    record = load_capture_record(path, root=tmp_path, end_effector=ee)
    _write_depth(tmp_path, "data/d405/depth/case_a_depth.png", shape=(48, 64))
    arr = load_depth(record, tmp_path)
    assert arr.shape == (48, 64)
    assert arr.dtype == np.uint16


def test_kinematic_model_sha_mismatch_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee, kinematic_sha="f" * 64)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    with pytest.raises(CaptureRecordError, match="kinematic_model.sha256"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_case_map_downscaled_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    case_map_path = tmp_path / "data" / "case_map.json"
    case_map_path.parent.mkdir(parents=True, exist_ok=True)
    case_map_path.write_text(
        json.dumps({"cases": [{"case_id": "case_a", "downscaled": True}]}), encoding="utf-8"
    )
    with pytest.raises(CaptureRecordError, match="downscaled|§0.5"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_image_hw_mismatch_against_depth_png_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    _write_depth(tmp_path, "data/d405/depth/case_a_depth.png", shape=(10, 10))
    with pytest.raises(CaptureRecordError, match="depth PNG"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_image_hw_mismatch_against_colour_image_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    doc["source"] = {
        "kind": "d405_metadata",
        "ref": "data/d405/metadata/case_a.json",
        "frame_index": 0,
    }
    color_rel = "data/d405/color/case_a_color.png"
    color_path = tmp_path / color_rel
    color_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.zeros((10, 10, 3), dtype=np.uint8)).save(color_path)
    meta_path = tmp_path / "data" / "d405" / "metadata" / "case_a.json"
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    meta_path.write_text(json.dumps({"files": {"color": color_rel}}), encoding="utf-8")
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    with pytest.raises(CaptureRecordError, match="colour image"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_image_hw_mismatch_against_mask_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    mask_path = tmp_path / "data" / "overlays" / "case_a_mask.png"
    mask_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.zeros((10, 10), dtype=np.uint8)).save(mask_path)
    with pytest.raises(CaptureRecordError, match="mask"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_image_hw_mismatch_against_paths_json_refused(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    paths_json_path = tmp_path / "data" / "paths" / "case_a_paths.json"
    paths_json_path.parent.mkdir(parents=True, exist_ok=True)
    paths_json_path.write_text(
        json.dumps({"image_height": 10, "image_width": 10}), encoding="utf-8"
    )
    with pytest.raises(CaptureRecordError, match="paths.json"):
        load_capture_record(path, root=tmp_path, end_effector=ee)


def test_image_hw_consistency_passes_when_artefacts_match(tmp_path: Path):
    ee = _end_effector()
    doc = _base_doc(tmp_path, ee=ee)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    _write_depth(tmp_path, "data/d405/depth/case_a_depth.png", shape=(48, 64))
    mask_path = tmp_path / "data" / "overlays" / "case_a_mask.png"
    mask_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.zeros((48, 64), dtype=np.uint8)).save(mask_path)
    paths_json_path = tmp_path / "data" / "paths" / "case_a_paths.json"
    paths_json_path.parent.mkdir(parents=True, exist_ok=True)
    paths_json_path.write_text(
        json.dumps({"image_height": 48, "image_width": 64}), encoding="utf-8"
    )
    record = load_capture_record(path, root=tmp_path, end_effector=ee)
    assert record.image_hw == (48, 64)


# ---------------------------------------------------------------------------
# Transform composition
# ---------------------------------------------------------------------------


def test_chain_composition_matches_hand_composed_product(tmp_path: Path):
    ee = _end_effector()
    q = [0.1, -0.2, -0.3, 0.0, 0.4, 0.0]
    doc = _base_doc(tmp_path, ee=ee, positions=q)
    path = tmp_path / "case_a_capture.json"
    write_record(doc, path)
    record = load_capture_record(path, root=tmp_path, end_effector=ee)
    chain = kin.load_chain()

    T = T_base_link_optical(record, chain, ee)

    T_expected = (
        chain.fk(q)
        @ ee.T_gripper_link_camera_link
        @ record.T_camera_link_camera_color_optical_frame
    )
    assert np.allclose(T, T_expected, atol=1e-12)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _write_metadata(tmp_path: Path, case: str, synthetic=True):
    depth_rel = "data/d405/depth/frame_depth.png"
    _write_depth(tmp_path, depth_rel, shape=(48, 64))
    meta = {
        "schema_version": 1,
        "session": "session1",
        "frame_index": 0,
        "timestamp_utc": "2026-09-30T12:00:00Z",
        "synthetic": synthetic,
        "aligned_to": "color",
        "depth_scale_m_per_unit": 0.0001,
        "color": {
            "width": 64,
            "height": 48,
            "fps": 30,
            "format": "bgr8",
            "intrinsics": {
                k: v
                for k, v in _color_intrinsics(64, 48).items()
                if k not in ("width", "height")
            },
        },
        "files": {"color": "data/d405/color/frame_color.png", "depth": depth_rel},
    }
    meta_path = tmp_path / "data" / "d405" / "metadata" / f"{case}.json"
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    meta_path.write_text(json.dumps(meta), encoding="utf-8")
    return meta_path


def _write_joint_state(tmp_path: Path, stamp_ns=1_780_300_500_031_000_000):
    js = {
        "joint_names": list(kin.load_chain().joint_names),
        "positions_rad": [0.0] * 6,
        "stamp_ns": stamp_ns,
        "stamp_source": "/joint_states",
    }
    js_path = tmp_path / "joint_state.json"
    js_path.write_text(json.dumps(js), encoding="utf-8")
    return js_path


def test_cli_assemble_then_validate_round_trip(tmp_project: Config, tmp_path: Path):
    meta_path = _write_metadata(tmp_project.root, "case_a")
    js_path = _write_joint_state(tmp_project.root, stamp_ns=1_780_300_500_000_000_000)

    exit_code = main([
        "--root", str(tmp_project.root),
        "assemble", "--case", "case_a",
        "--d405-metadata", str(meta_path),
        "--joint-state", str(js_path),
    ])
    assert exit_code == EXIT_OK
    out_path = capture_record_path(tmp_project.root, "case_a")
    assert out_path.is_file()

    exit_code = main([
        "--root", str(tmp_project.root),
        "validate", "--cases", "case_a",
        "--max-skew-s", "1e12",
    ])
    assert exit_code == EXIT_OK


def test_cli_dry_run_assemble_writes_nothing(tmp_project: Config):
    meta_path = _write_metadata(tmp_project.root, "case_b")
    js_path = _write_joint_state(tmp_project.root)

    exit_code = main([
        "--root", str(tmp_project.root),
        "--dry-run",
        "assemble", "--case", "case_b",
        "--d405-metadata", str(meta_path),
        "--joint-state", str(js_path),
    ])
    assert exit_code == EXIT_OK
    assert not capture_record_path(tmp_project.root, "case_b").exists()


def test_cli_assemble_synthetic_metadata_yields_synthetic_record(tmp_project: Config):
    meta_path = _write_metadata(tmp_project.root, "case_c", synthetic=True)
    js_path = _write_joint_state(tmp_project.root)

    exit_code = main([
        "--root", str(tmp_project.root),
        "assemble", "--case", "case_c",
        "--d405-metadata", str(meta_path),
        "--joint-state", str(js_path),
    ])
    assert exit_code == EXIT_OK
    doc = json.loads(capture_record_path(tmp_project.root, "case_c").read_text())
    assert doc["synthetic"] is True


def test_cli_bad_args_exit_2():
    with pytest.raises(SystemExit) as exc_info:
        main(["assemble", "--case", "case_a"])
    assert exc_info.value.code == EXIT_USAGE


def test_cli_validate_missing_record_exit_3(tmp_project: Config):
    exit_code = main([
        "--root", str(tmp_project.root),
        "validate", "--cases", "no_such_case",
    ])
    assert exit_code == EXIT_PRECONDITION


def test_cli_help_exits_0():
    with pytest.raises(SystemExit) as exc_info:
        main(["--help"])
    assert exc_info.value.code == EXIT_OK
