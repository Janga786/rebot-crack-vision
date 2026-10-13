"""Unit tests for preview_trajectory (§18.4, MOT-08.6). Pure builders only -- no ROS graph.

Usage: bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; \
    python3 -m pytest ros2_ws/src/crackvision_motion/test/test_preview_trajectory.py -q -p no:cacheprovider'
"""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

import crackvision_motion.preview_trajectory as pt
from crackvision_motion.cartesian_core import load_paths3d
from crackvision_motion.cli_common import EXIT_PRECONDITION
from crackvision_motion.joint_trajectory import JOINT_NAMES, load_trajectory
from crackvision_motion.tool_fk import load_tool_chain

REPO_ROOT = Path(__file__).resolve().parents[4]
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
TRAJECTORY_SMOKE = FIXTURES_DIR / "trajectory_smoke.json"
PATHS3D_SMOKE = FIXTURES_DIR / "cartesian" / "smoke" / "paths3d.json"
URDF_PATH = REPO_ROOT / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf"
END_EFFECTOR_PATH = REPO_ROOT / "config" / "robot" / "end_effector.yaml"


@pytest.fixture(scope="module")
def traj():
    return load_trajectory(TRAJECTORY_SMOKE)


@pytest.fixture(scope="module")
def tool_xyz(traj):
    chain = load_tool_chain(URDF_PATH, END_EFFECTOR_PATH)
    return chain.tool_positions([p["positions"] for p in traj["points"]])


# --------------------------------------------------------------------------------------
# build_display_trajectory
# --------------------------------------------------------------------------------------

def test_build_display_trajectory_fields(traj):
    display = pt.build_display_trajectory(traj, speed_scale=1.0)
    assert display.model_id == "rebotarm"
    assert list(display.trajectory[0].joint_trajectory.joint_names) == list(JOINT_NAMES)
    assert list(display.trajectory_start.joint_state.position) == pytest.approx(traj["points"][0]["positions"])


def test_build_display_trajectory_time_scaling(traj):
    display = pt.build_display_trajectory(traj, speed_scale=0.5)
    points = display.trajectory[0].joint_trajectory.points
    for src_point, msg_point in zip(traj["points"], points):
        expected_t = src_point["t_s"] / 0.5
        actual_t = msg_point.time_from_start.sec + msg_point.time_from_start.nanosec / 1e9
        assert actual_t == pytest.approx(expected_t, abs=1e-6)


def test_build_display_trajectory_rejects_non_positive_speed_scale(traj):
    with pytest.raises(ValueError):
        pt.build_display_trajectory(traj, speed_scale=0.0)


# --------------------------------------------------------------------------------------
# build_markers
# --------------------------------------------------------------------------------------

def test_build_markers_tool_path_only(traj, tool_xyz):
    markers = pt.build_markers(traj, tool_xyz, paths3d_doc=None)
    assert len(markers.markers) == 1
    marker = markers.markers[0]
    assert marker.ns == "tool_path"
    assert marker.id == 0
    assert marker.header.frame_id == "base_link"
    assert marker.type == marker.LINE_STRIP
    assert len(marker.points) == len(traj["points"])


def test_build_markers_with_crack_polylines(traj, tool_xyz):
    paths3d_doc, _sha = load_paths3d(PATHS3D_SMOKE)
    markers = pt.build_markers(traj, tool_xyz, paths3d_doc=paths3d_doc)
    # 1 tool-path marker + 4 valid crack polylines (crack_a main/branch, crack_far, crack_sunk)
    assert len(markers.markers) == 5
    crack_markers = [m for m in markers.markers if m.ns == "crack_path"]
    assert len(crack_markers) == 4
    assert sorted(m.id for m in crack_markers) == [1, 2, 3, 4]
    for m in crack_markers:
        assert m.header.frame_id == "base_link"
        assert len(m.points) >= 2


def test_build_markers_rejects_mismatched_tool_xyz_length(traj, tool_xyz):
    with pytest.raises(ValueError):
        pt.build_markers(traj, tool_xyz[:-1], paths3d_doc=None)


# --------------------------------------------------------------------------------------
# paths3d sha-verification (§18.4 / §18.1.1 bindings.paths3d_sha256 convention)
# --------------------------------------------------------------------------------------

def test_resolve_paths3d_accepts_matching_sha(traj):
    _doc, sha256 = load_paths3d(PATHS3D_SMOKE)
    traj_with_source = copy.deepcopy(traj)
    traj_with_source["source"] = {"paths3d": {"path": str(PATHS3D_SMOKE), "sha256": sha256, "execution_eligible": False}}
    resolved = pt._resolve_paths3d(traj_with_source, str(PATHS3D_SMOKE))
    assert resolved is not None
    assert resolved["case_id"] == "mot067_smoke"


def test_resolve_paths3d_ignores_sha_mismatch(traj, capsys):
    traj_with_source = copy.deepcopy(traj)
    traj_with_source["source"] = {
        "paths3d": {"path": str(PATHS3D_SMOKE), "sha256": "0" * 64, "execution_eligible": False}
    }
    resolved = pt._resolve_paths3d(traj_with_source, str(PATHS3D_SMOKE))
    assert resolved is None
    captured = capsys.readouterr()
    assert "does not match" in captured.err


def test_resolve_paths3d_none_when_trajectory_has_no_source(traj):
    traj_no_source = copy.deepcopy(traj)
    traj_no_source["source"] = {"paths3d": None}
    resolved = pt._resolve_paths3d(traj_no_source, str(PATHS3D_SMOKE))
    assert resolved is None


def test_resolve_paths3d_none_when_flag_omitted(traj):
    assert pt._resolve_paths3d(traj, None) is None


# --------------------------------------------------------------------------------------
# main(): invalid trajectory -> exit 3, bad flag -> exit 2, offline refusal never touches rclpy
# --------------------------------------------------------------------------------------

def test_main_invalid_trajectory_exits_3_without_touching_rclpy(tmp_path, monkeypatch):
    bad_trajectory = tmp_path / "bad.json"
    bad_trajectory.write_text(json.dumps({"schema": "not-a-real-schema"}))

    def _boom(*_args, **_kwargs):
        raise AssertionError("rclpy.init must not be called for an invalid trajectory")

    monkeypatch.setattr(pt.rclpy, "init", _boom)

    rc = pt.main(["--root", str(REPO_ROOT), "--trajectory", str(bad_trajectory)])
    assert rc == EXIT_PRECONDITION


def test_main_missing_trajectory_file_exits_3(tmp_path, monkeypatch):
    def _boom(*_args, **_kwargs):
        raise AssertionError("rclpy.init must not be called when the trajectory file is missing")

    monkeypatch.setattr(pt.rclpy, "init", _boom)

    rc = pt.main(["--root", str(REPO_ROOT), "--trajectory", str(tmp_path / "does_not_exist.json")])
    assert rc == EXIT_PRECONDITION


def test_main_bad_flag_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        pt.main(["--trajectory", str(TRAJECTORY_SMOKE), "--not-a-real-flag"])
    assert exc_info.value.code == 2
