"""Unit tests for execution_gate (MOT-05.3). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_execution_gate.py -q -p no:cacheprovider
"""

from __future__ import annotations

import copy
import json
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "crackvision_description"))

from crackvision_motion.execution_gate import (  # noqa: E402
    G_APPROVAL,
    G_ARM,
    G_COMMISSIONING,
    G_EE,
    G_ELIGIBLE,
    G_ESTOP,
    G_LIMITS,
    G_LIMITS_HASH,
    G_SCENE,
    G_SPEED,
    G_STALE_CONFIG,
    G_TRAJ,
    GATE_IDS,
    GateCheck,
    GateConfigError,
    GateReport,
    OUTCOMES,
    confirmation_ok,
    confirmation_phrase,
    evaluate_offline,
    load_approval,
    load_commissioning,
    load_execution_config,
)
from crackvision_motion.cartesian_plan import (  # noqa: E402
    add_view_candidate,
    build_leg,
    finalize_view,
    new_view_plan,
    write_view_plan,
)
from crackvision_motion.joint_trajectory import file_sha256, load_limits  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[4]
PRODUCTION_LIMITS = REPO_ROOT / "config" / "robot" / "b601_dm_limits.yaml"
PRODUCTION_EXECUTION_CONFIG = REPO_ROOT / "config" / "motion" / "execution.yaml"
PRODUCTION_COMMISSIONING = REPO_ROOT / "config" / "robot" / "commissioning.yaml"
PRODUCTION_SCENE = REPO_ROOT / "config" / "scene" / "scene.yaml"
PRODUCTION_EE = REPO_ROOT / "config" / "robot" / "end_effector.yaml"

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
GATE_FIXTURES = FIXTURES_DIR / "execution_gate"
TRAJECTORY_SMOKE = FIXTURES_DIR / "trajectory_smoke.json"  # purpose: test, all shas null (MOT-05.2)

SCENE_MEASURED = GATE_FIXTURES / "scene_measured.yaml"
EE_MEASURED = GATE_FIXTURES / "end_effector_measured.yaml"
COMMISSIONING_READY = GATE_FIXTURES / "commissioning_ready.yaml"
PATHS3D_ELIGIBLE = GATE_FIXTURES / "paths3d_eligible.json"

VIEW_LIMITS = load_limits(PRODUCTION_LIMITS)

_NO_ARM = {}
_ARM_REAL = {"CRACKVISION_ARM_REAL": "1"}


def _common_kwargs(tmp_path, **overrides):
    kwargs = dict(
        root=tmp_path,
        execution_config_path=PRODUCTION_EXECUTION_CONFIG,
        commissioning_path=PRODUCTION_COMMISSIONING,
        limits_path=PRODUCTION_LIMITS,
        scene_config_path=PRODUCTION_SCENE,
        end_effector_config_path=PRODUCTION_EE,
        approval_path=None,
        speed_scale=None,
        env=_NO_ARM,
    )
    kwargs.update(overrides)
    return kwargs


def _get(report: GateReport, gate_id: str) -> GateCheck:
    by_id = {c.id: c for c in report.checks}
    return by_id[gate_id]


# --------------------------------------------------------------------------------------
# repo defaults refuse real mode
# --------------------------------------------------------------------------------------

def test_repo_defaults_refuse_real_mode(tmp_path):
    report = evaluate_offline("real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path))
    assert not report.passed
    refusals = set(report.refusals)
    assert {G_ARM, G_SCENE, G_EE, G_COMMISSIONING, G_ESTOP}.issubset(refusals)
    assert _get(report, G_ARM).outcome == "fail"
    assert "CRACKVISION_ARM_REAL" in _get(report, G_ARM).message


def test_mock_and_dry_never_pass_real_only_gates_but_otherwise_pass(tmp_path):
    for mode in ("mock", "dry"):
        report = evaluate_offline(mode, TRAJECTORY_SMOKE, **_common_kwargs(tmp_path))
        assert _get(report, G_ARM).outcome == "skip"
        assert _get(report, G_COMMISSIONING).outcome == "skip"
        assert _get(report, G_ESTOP).outcome == "skip"
        assert report.passed, f"mode={mode} unexpectedly refused: {report.refusals}"


def test_gate_ids_match_matrix_exactly():
    report = evaluate_offline("real", TRAJECTORY_SMOKE, **_common_kwargs(Path(".")))
    assert [c.id for c in report.checks] == list(GATE_IDS)


# --------------------------------------------------------------------------------------
# measured fixture set: real mode passes offline, with single-input flips failing exactly
# one gate
# --------------------------------------------------------------------------------------

def _write_json(path: Path, obj: dict) -> Path:
    path.write_text(json.dumps(obj, indent=2))
    return path


def _build_preview_artifacts(tmp_path: Path, traj_sha: str, *, all_pass: bool = True) -> list:
    """Writes a minimal valid `crackvision.trajectory_preview/1` manifest plus three PNG
    stand-ins bound to `traj_sha`, and returns the four `{path, sha256}` entries §18.2 says
    `approve_trajectory` writes to `preview_artifacts[]` (manifest first, then the three PNGs)."""
    stem = f"preview_{traj_sha[:8]}"
    joints_png = tmp_path / f"{stem}_joints.png"
    toolpath_png = tmp_path / f"{stem}_toolpath.png"
    toolspeed_png = tmp_path / f"{stem}_toolspeed.png"
    joints_png.write_bytes(b"\x89PNG-joints-stand-in")
    toolpath_png.write_bytes(b"\x89PNG-toolpath-stand-in")
    toolspeed_png.write_bytes(b"\x89PNG-toolspeed-stand-in")
    manifest = {
        "schema": "crackvision.trajectory_preview/1",
        "trajectory": {"path": "trajectory.json", "sha256": traj_sha},
        "check_report": {"all_pass": all_pass, "failed_checks": [] if all_pass else ["G-LIMITS"]},
    }
    manifest_path = _write_json(tmp_path / f"{stem}.json", manifest)
    return [
        {"path": p.name, "sha256": file_sha256(p)}
        for p in (manifest_path, joints_png, toolpath_png, toolspeed_png)
    ]


def _passing_fixture_set(tmp_path):
    """Builds a self-consistent measured fixture set under tmp_path (root) that passes real
    mode offline, by computing real shas against the fixture files (never hardcoded, so this
    cannot silently drift stale)."""
    traj = {
        "schema": "crackvision.joint_trajectory/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "MOT-05.3 fixture",
        "purpose": "crack_task",
        "planning_frame": "base_link",
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        "points": [
            {"t_s": 0.0, "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            {"t_s": 1.5, "positions": [0.05, -0.3, -0.45, 0.05, 0.05, 0.05]},
            {"t_s": 3.0, "positions": [0.1, -0.6, -0.9, 0.1, 0.1, 0.1]},
        ],
        "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": file_sha256(PRODUCTION_LIMITS)},
        "end_effector_config_sha256": file_sha256(EE_MEASURED),
        "scene_config_sha256": file_sha256(SCENE_MEASURED),
        "source": {
            "paths3d": {
                "path": "paths3d_eligible.json",
                "sha256": file_sha256(PATHS3D_ELIGIBLE),
                "execution_eligible": True,
            }
        },
    }
    (tmp_path / "paths3d_eligible.json").write_text(PATHS3D_ELIGIBLE.read_text())
    traj_path = _write_json(tmp_path / "trajectory.json", traj)
    traj_sha = file_sha256(traj_path)

    approval = {
        "schema": "crackvision.execution_approval/1",
        "trajectory_sha256": traj_sha,
        "approved_by": "fixture_operator",
        "approved_utc": datetime.now(timezone.utc).isoformat(),
        "preview_artifacts": _build_preview_artifacts(tmp_path, traj_sha),
    }
    approval_path = _write_json(tmp_path / "approval.json", approval)

    kwargs = dict(
        root=tmp_path,
        execution_config_path=PRODUCTION_EXECUTION_CONFIG,
        commissioning_path=COMMISSIONING_READY,
        limits_path=PRODUCTION_LIMITS,
        scene_config_path=SCENE_MEASURED,
        end_effector_config_path=EE_MEASURED,
        approval_path=approval_path,
        speed_scale=None,
        env=_ARM_REAL,
    )
    return traj_path, traj, approval, approval_path, kwargs


def test_commissioning_fixture_limits_sha_is_current():
    commissioning = load_commissioning(COMMISSIONING_READY)
    assert commissioning["limits_file_sha256"] == file_sha256(PRODUCTION_LIMITS)


def test_measured_fixture_set_passes_real_mode_offline(tmp_path):
    traj_path, _traj, _approval, _approval_path, kwargs = _passing_fixture_set(tmp_path)
    report = evaluate_offline("real", traj_path, **kwargs)
    assert report.passed, report.refusals
    for check in report.checks:
        assert check.outcome in ("pass",), f"{check.id}: {check.outcome}: {check.message}"


def test_limits_hash_mismatch_fails_only_limits_hash(tmp_path):
    traj_path, traj, _approval, _approval_path, kwargs = _passing_fixture_set(tmp_path)
    traj = copy.deepcopy(traj)
    traj["limits_file"]["sha256"] = "a" * 64
    traj_path = _write_json(tmp_path / "trajectory_bad_hash.json", traj)
    # approval must still bind the (new) trajectory sha, including a preview bound to it
    new_sha = file_sha256(traj_path)
    approval = json.loads((tmp_path / "approval.json").read_text())
    approval["trajectory_sha256"] = new_sha
    approval["preview_artifacts"] = _build_preview_artifacts(tmp_path, new_sha)
    _write_json(tmp_path / "approval.json", approval)

    report = evaluate_offline("real", traj_path, **kwargs)
    assert set(report.refusals) == {G_LIMITS_HASH}


def test_stale_approval_fails_only_approval(tmp_path):
    traj_path, _traj, approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = copy.deepcopy(approval)
    approval["approved_utc"] = (datetime.now(timezone.utc) - timedelta(hours=10)).isoformat()
    _write_json(approval_path, approval)

    report = evaluate_offline("real", traj_path, **kwargs)
    assert set(report.refusals) == {G_APPROVAL}


# --------------------------------------------------------------------------------------
# §18.3 G-APPROVAL amendment: preview_artifacts verification (MOT-08.5)
# --------------------------------------------------------------------------------------

@pytest.mark.parametrize("mode", ["mock", "real"])
def test_empty_preview_artifacts_violates_approval(tmp_path, mode):
    traj_path, _traj, _approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = json.loads(approval_path.read_text())
    approval["preview_artifacts"] = []
    _write_json(approval_path, approval)

    report = evaluate_offline(mode, traj_path, **kwargs)
    check = _get(report, G_APPROVAL)
    if mode == "real":
        assert check.outcome == "fail"
        assert not report.passed
    else:
        assert check.outcome == "warn"
        assert report.passed
    assert "preview_artifacts" in check.message and "empty" in check.message
    assert "§18.3 check 1" in check.message


@pytest.mark.parametrize("mode", ["mock", "real"])
def test_preview_artifact_sha_mismatch_violates_approval(tmp_path, mode):
    traj_path, _traj, _approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = json.loads(approval_path.read_text())
    approval["preview_artifacts"][0]["sha256"] = "a" * 64
    _write_json(approval_path, approval)

    report = evaluate_offline(mode, traj_path, **kwargs)
    check = _get(report, G_APPROVAL)
    assert check.outcome == ("fail" if mode == "real" else "warn")
    assert "does not match current sha256" in check.message
    assert "§18.3 check 2" in check.message
    assert report.passed == (mode != "real")


@pytest.mark.parametrize("mode", ["mock", "real"])
def test_preview_artifact_missing_file_does_not_raise(tmp_path, mode):
    traj_path, _traj, _approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = json.loads(approval_path.read_text())
    missing_entry = approval["preview_artifacts"][1]
    (tmp_path / missing_entry["path"]).unlink()

    report = evaluate_offline(mode, traj_path, **kwargs)
    check = _get(report, G_APPROVAL)
    assert check.outcome == ("fail" if mode == "real" else "warn")
    assert "unreadable" in check.message
    assert "§18.3 check 2" in check.message
    assert report.passed == (mode != "real")


_UNSET = object()


@pytest.mark.parametrize(
    "bad_path,bad_sha256",
    [
        (5, _UNSET),
        (None, _UNSET),
        (["x"], _UNSET),
        ("preview_\u0000bad.json", _UNSET),
        (_UNSET, 5),
    ],
    ids=["int-path", "null-path", "list-path", "nul-path", "non-str-sha256"],
)
@pytest.mark.parametrize("mode", ["mock", "real"])
def test_malformed_preview_artifact_path_or_sha256_does_not_raise(
    tmp_path, mode, bad_path, bad_sha256
):
    """§18.3/MOT-08.5.R1 finding 6f6a3f33357e1085: a non-string, NUL-containing, or otherwise
    malformed `path`/`sha256` value must become a G-APPROVAL violation detail, never an
    uncaught TypeError/ValueError."""
    traj_path, _traj, _approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = json.loads(approval_path.read_text())
    entry = approval["preview_artifacts"][1]
    if bad_path is not _UNSET:
        entry["path"] = bad_path
    if bad_sha256 is not _UNSET:
        entry["sha256"] = bad_sha256
    _write_json(approval_path, approval)

    report = evaluate_offline(mode, traj_path, **kwargs)
    check = _get(report, G_APPROVAL)
    assert check.outcome == ("fail" if mode == "real" else "warn")
    assert "§18.3 check 2" in check.message
    assert report.passed == (mode != "real")


@pytest.mark.parametrize("mode", ["mock", "real"])
def test_preview_manifest_invalid_json_counts_as_zero_manifests(tmp_path, mode):
    traj_path, _traj, _approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = json.loads(approval_path.read_text())
    manifest_entry = approval["preview_artifacts"][0]
    manifest_path = tmp_path / manifest_entry["path"]
    manifest_path.write_text("{this is not valid json")
    manifest_entry["sha256"] = file_sha256(manifest_path)
    _write_json(approval_path, approval)

    report = evaluate_offline(mode, traj_path, **kwargs)
    check = _get(report, G_APPROVAL)
    assert check.outcome == ("fail" if mode == "real" else "warn")
    assert "found 0" in check.message
    assert "§18.3 check 3" in check.message
    assert report.passed == (mode != "real")


@pytest.mark.parametrize("mode", ["mock", "real"])
def test_preview_manifest_non_utf8_counts_as_zero_manifests(tmp_path, mode):
    traj_path, _traj, _approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = json.loads(approval_path.read_text())
    manifest_entry = approval["preview_artifacts"][0]
    manifest_path = tmp_path / manifest_entry["path"]
    manifest_path.write_bytes(b"\xff\xfe\x00bad")
    manifest_entry["sha256"] = file_sha256(manifest_path)
    _write_json(approval_path, approval)

    report = evaluate_offline(mode, traj_path, **kwargs)
    check = _get(report, G_APPROVAL)
    assert check.outcome == ("fail" if mode == "real" else "warn")
    assert "found 0" in check.message
    assert "§18.3 check 3" in check.message
    assert report.passed == (mode != "real")


@pytest.mark.parametrize("mode", ["mock", "real"])
def test_two_preview_manifests_violates_approval(tmp_path, mode):
    traj_path, _traj, _approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = json.loads(approval_path.read_text())
    traj_sha = approval["trajectory_sha256"]
    extra_manifest_path = tmp_path / "second_preview.json"
    _write_json(extra_manifest_path, {
        "schema": "crackvision.trajectory_preview/1",
        "trajectory": {"sha256": traj_sha},
        "check_report": {"all_pass": True},
    })
    approval["preview_artifacts"].append(
        {"path": extra_manifest_path.name, "sha256": file_sha256(extra_manifest_path)}
    )
    _write_json(approval_path, approval)

    report = evaluate_offline(mode, traj_path, **kwargs)
    check = _get(report, G_APPROVAL)
    assert check.outcome == ("fail" if mode == "real" else "warn")
    assert "found 2" in check.message
    assert "§18.3 check 3" in check.message
    assert report.passed == (mode != "real")


@pytest.mark.parametrize("mode", ["mock", "real"])
def test_preview_manifest_names_another_trajectory_violates_approval(tmp_path, mode):
    traj_path, _traj, _approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = json.loads(approval_path.read_text())
    manifest_entry = approval["preview_artifacts"][0]
    manifest_path = tmp_path / manifest_entry["path"]
    manifest = json.loads(manifest_path.read_text())
    manifest["trajectory"]["sha256"] = "b" * 64
    _write_json(manifest_path, manifest)
    manifest_entry["sha256"] = file_sha256(manifest_path)
    _write_json(approval_path, approval)

    report = evaluate_offline(mode, traj_path, **kwargs)
    check = _get(report, G_APPROVAL)
    assert check.outcome == ("fail" if mode == "real" else "warn")
    assert "trajectory.sha256 does not match" in check.message
    assert "§18.3 check 4" in check.message
    assert report.passed == (mode != "real")


@pytest.mark.parametrize("mode", ["mock", "real"])
def test_preview_manifest_all_pass_false_violates_approval(tmp_path, mode):
    traj_path, _traj, _approval, approval_path, kwargs = _passing_fixture_set(tmp_path)
    approval = json.loads(approval_path.read_text())
    traj_sha = approval["trajectory_sha256"]
    approval["preview_artifacts"] = _build_preview_artifacts(tmp_path, traj_sha, all_pass=False)
    _write_json(approval_path, approval)

    report = evaluate_offline(mode, traj_path, **kwargs)
    check = _get(report, G_APPROVAL)
    assert check.outcome == ("fail" if mode == "real" else "warn")
    assert "all_pass is not true" in check.message
    assert "§18.3 check 5" in check.message
    assert report.passed == (mode != "real")


def test_valid_preview_manifest_and_artifacts_pass_approval(tmp_path):
    traj_path, _traj, _approval, _approval_path, kwargs = _passing_fixture_set(tmp_path)
    report = evaluate_offline("real", traj_path, **kwargs)
    assert _get(report, G_APPROVAL).outcome == "pass"
    assert report.passed


def test_ineligible_paths3d_fails_only_eligible(tmp_path):
    traj_path, traj, _approval, _approval_path, kwargs = _passing_fixture_set(tmp_path)
    ineligible = json.loads(PATHS3D_ELIGIBLE.read_text())
    ineligible["execution_eligible"] = False
    ineligible["ineligible_reasons"] = ["no_valid_points"]
    p3d_path = _write_json(tmp_path / "paths3d_ineligible.json", ineligible)

    traj = copy.deepcopy(traj)
    traj["source"]["paths3d"] = {
        "path": "paths3d_ineligible.json",
        "sha256": file_sha256(p3d_path),
        "execution_eligible": False,
    }
    traj_path = _write_json(tmp_path / "trajectory_ineligible.json", traj)
    new_sha = file_sha256(traj_path)
    approval = json.loads((tmp_path / "approval.json").read_text())
    approval["trajectory_sha256"] = new_sha
    approval["preview_artifacts"] = _build_preview_artifacts(tmp_path, new_sha)
    _write_json(tmp_path / "approval.json", approval)

    report = evaluate_offline("real", traj_path, **kwargs)
    assert set(report.refusals) == {G_ELIGIBLE}


def test_speed_scale_over_cap_fails_only_speed(tmp_path):
    traj_path, _traj, _approval, _approval_path, kwargs = _passing_fixture_set(tmp_path)
    kwargs = dict(kwargs)
    kwargs["speed_scale"] = 0.5  # commissioning fixture cap is 0.10

    report = evaluate_offline("real", traj_path, **kwargs)
    assert set(report.refusals) == {G_SPEED}


def test_purpose_test_fails_only_traj(tmp_path):
    traj_path, traj, _approval, _approval_path, kwargs = _passing_fixture_set(tmp_path)
    traj = copy.deepcopy(traj)
    traj["purpose"] = "test"
    traj_path = _write_json(tmp_path / "trajectory_test_purpose.json", traj)
    new_sha = file_sha256(traj_path)
    approval = json.loads((tmp_path / "approval.json").read_text())
    approval["trajectory_sha256"] = new_sha
    approval["preview_artifacts"] = _build_preview_artifacts(tmp_path, new_sha)
    _write_json(tmp_path / "approval.json", approval)

    report = evaluate_offline("real", traj_path, **kwargs)
    assert set(report.refusals) == {G_TRAJ}


def test_nominal_scene_fails_only_scene(tmp_path):
    traj_path, _traj, _approval, _approval_path, kwargs = _passing_fixture_set(tmp_path)
    kwargs = dict(kwargs)
    kwargs["scene_config_path"] = PRODUCTION_SCENE  # real scene.yaml ships nominal

    # scene_config_sha256 in the trajectory was bound to SCENE_MEASURED, so swapping the file
    # alone would also trip G-STALE-CONFIG; rebuild the trajectory bound to the nominal scene so
    # only G-SCENE is exercised.
    traj = json.loads(traj_path.read_text())
    traj["scene_config_sha256"] = file_sha256(PRODUCTION_SCENE)
    traj_path = _write_json(tmp_path / "trajectory_nominal_scene.json", traj)
    new_sha = file_sha256(traj_path)
    approval = json.loads((tmp_path / "approval.json").read_text())
    approval["trajectory_sha256"] = new_sha
    approval["preview_artifacts"] = _build_preview_artifacts(tmp_path, new_sha)
    _write_json(tmp_path / "approval.json", approval)

    report = evaluate_offline("real", traj_path, **kwargs)
    assert set(report.refusals) == {G_SCENE}


# --------------------------------------------------------------------------------------
# upstream gates: exceptions become failed checks, nothing propagates
# --------------------------------------------------------------------------------------

def test_upstream_scene_and_ee_exceptions_become_failed_checks_in_real(tmp_path):
    report = evaluate_offline("real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path))
    scene_check = _get(report, G_SCENE)
    ee_check = _get(report, G_EE)
    assert scene_check.outcome == "fail" and "nominal" in scene_check.message
    assert ee_check.outcome == "fail" and "nominal" in ee_check.message


def test_upstream_scene_and_ee_not_commissioning_ready_warn_outside_real(tmp_path):
    # Repo-shipped scene.yaml/end_effector.yaml are nominal, so they load fine but are not
    # commissioning-ready -- §11.7 says that is `warn` in mock/dry (never refuses), `fail` in real.
    for mode in ("mock", "dry"):
        report = evaluate_offline(mode, TRAJECTORY_SMOKE, **_common_kwargs(tmp_path))
        assert _get(report, G_SCENE).outcome == "warn"
        assert _get(report, G_EE).outcome == "warn"
        assert report.passed, f"mode={mode} unexpectedly refused: {report.refusals}"


def test_malformed_scene_config_fails_scene_in_every_mode(tmp_path):
    bad_scene = tmp_path / "bad_scene.yaml"
    bad_scene.write_text("schema: wrong\n")
    for mode in ("mock", "dry", "real"):
        report = evaluate_offline(mode, TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, scene_config_path=bad_scene))
        assert _get(report, G_SCENE).outcome == "fail"


# --------------------------------------------------------------------------------------
# MOT-05.3.R1 finding 1: evaluate_offline never raises, whatever the inputs
# --------------------------------------------------------------------------------------

def _unparseable_yaml(tmp_path, name: str) -> Path:
    path = tmp_path / name
    path.write_text("key: [this is not valid yaml\n")
    return path


def test_missing_trajectory_does_not_raise(tmp_path):
    report = evaluate_offline(
        "real", tmp_path / "does_not_exist.json", **_common_kwargs(tmp_path)
    )
    assert not report.passed
    assert _get(report, G_TRAJ).outcome in ("fail", "error")


def test_missing_scene_config_does_not_raise(tmp_path):
    report = evaluate_offline(
        "real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, scene_config_path=tmp_path / "no_such_scene.yaml")
    )
    assert not report.passed
    assert _get(report, G_SCENE).outcome in ("fail", "error")


def test_missing_end_effector_config_does_not_raise(tmp_path):
    report = evaluate_offline(
        "real", TRAJECTORY_SMOKE,
        **_common_kwargs(tmp_path, end_effector_config_path=tmp_path / "no_such_ee.yaml"),
    )
    assert not report.passed
    assert _get(report, G_EE).outcome in ("fail", "error")


def test_unparseable_scene_yaml_does_not_raise(tmp_path):
    bad_scene = _unparseable_yaml(tmp_path, "bad_scene.yaml")
    report = evaluate_offline("real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, scene_config_path=bad_scene))
    assert not report.passed
    assert _get(report, G_SCENE).outcome in ("fail", "error")


def test_unparseable_end_effector_yaml_does_not_raise(tmp_path):
    bad_ee = _unparseable_yaml(tmp_path, "bad_ee.yaml")
    report = evaluate_offline(
        "real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, end_effector_config_path=bad_ee)
    )
    assert not report.passed
    assert _get(report, G_EE).outcome in ("fail", "error")


def test_unparseable_execution_config_yaml_does_not_raise(tmp_path):
    bad_exec = _unparseable_yaml(tmp_path, "bad_execution.yaml")
    report = evaluate_offline(
        "real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, execution_config_path=bad_exec)
    )
    assert not report.passed
    assert _get(report, G_SPEED).outcome in ("fail", "error")
    assert _get(report, G_LIMITS).outcome in ("fail", "error")


def test_unparseable_commissioning_yaml_does_not_raise(tmp_path):
    bad_commissioning = _unparseable_yaml(tmp_path, "bad_commissioning.yaml")
    report = evaluate_offline(
        "real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, commissioning_path=bad_commissioning, env=_ARM_REAL)
    )
    assert not report.passed
    assert _get(report, G_COMMISSIONING).outcome in ("fail", "error")
    assert _get(report, G_ESTOP).outcome in ("fail", "error")
    assert _get(report, G_SPEED).outcome in ("fail", "error")


@pytest.mark.parametrize("bad_scale", [0.0, -1.0, 2.0])
def test_out_of_range_speed_scale_does_not_raise(tmp_path, bad_scale):
    for mode in ("mock", "dry", "real"):
        kwargs = _common_kwargs(tmp_path, speed_scale=bad_scale, env=_ARM_REAL)
        report = evaluate_offline(mode, TRAJECTORY_SMOKE, **kwargs)
        assert not report.passed
        assert _get(report, G_SPEED).outcome == "fail"
        assert _get(report, G_LIMITS).outcome == "error"


def test_load_execution_config_raises_gate_config_error_on_malformed_yaml(tmp_path):
    bad = _unparseable_yaml(tmp_path, "bad_execution.yaml")
    with pytest.raises(GateConfigError):
        load_execution_config(bad)


def test_load_commissioning_raises_gate_config_error_on_malformed_yaml(tmp_path):
    bad = _unparseable_yaml(tmp_path, "bad_commissioning.yaml")
    with pytest.raises(GateConfigError):
        load_commissioning(bad)


# --------------------------------------------------------------------------------------
# MOT-05.3.R1 finding 2: §11.7 outcomes (warn/error) and the §11.9 gate_report shape
# --------------------------------------------------------------------------------------

def test_gate_check_to_dict_is_the_section_11_9_shape(tmp_path):
    report = evaluate_offline("real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path))
    for entry in report.to_dict()["gate_report"]:
        assert set(entry) == {"gate", "category", "outcome", "detail"}
        assert entry["outcome"] in OUTCOMES
    arm_entry = next(e for e in report.to_dict()["gate_report"] if e["gate"] == G_ARM)
    assert arm_entry["category"] == "confirmation"
    scene_entry = next(e for e in report.to_dict()["gate_report"] if e["gate"] == G_SCENE)
    assert scene_entry["category"] == "offline"


def test_warn_never_refuses_and_never_appears_in_real(tmp_path):
    for mode in ("mock", "dry"):
        report = evaluate_offline(mode, TRAJECTORY_SMOKE, **_common_kwargs(tmp_path))
        assert _get(report, G_APPROVAL).outcome == "warn"
        assert _get(report, G_SCENE).outcome == "warn"
        assert _get(report, G_EE).outcome == "warn"
        assert report.passed

    real_report = evaluate_offline("real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, env=_ARM_REAL))
    assert not any(c.outcome == "warn" for c in real_report.checks)


def test_error_for_limits_when_traj_cannot_load(tmp_path):
    report = evaluate_offline(
        "real", tmp_path / "does_not_exist.json", **_common_kwargs(tmp_path, env=_ARM_REAL)
    )
    assert _get(report, G_LIMITS).outcome == "error"
    assert _get(report, G_LIMITS_HASH).outcome == "error"
    assert not report.passed


# --------------------------------------------------------------------------------------
# MOT-05.3.R1 attempt-1 feedback: dry-mode G-SPEED previews the commissioning cap as warn
# --------------------------------------------------------------------------------------

def test_dry_speed_above_commissioning_cap_warns(tmp_path):
    # repo commissioning.yaml cap is 0.10; 0.5 is within execution.yaml's cap (1.0) but above it.
    report = evaluate_offline("dry", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, speed_scale=0.5))
    assert _get(report, G_SPEED).outcome == "warn"
    assert report.passed


def test_dry_speed_unreadable_commissioning_warns(tmp_path):
    bad_commissioning = _unparseable_yaml(tmp_path, "bad_commissioning.yaml")
    report = evaluate_offline(
        "dry", TRAJECTORY_SMOKE,
        **_common_kwargs(tmp_path, commissioning_path=bad_commissioning, speed_scale=0.5),
    )
    assert _get(report, G_SPEED).outcome == "warn"
    assert report.passed


def test_dry_speed_within_commissioning_cap_passes(tmp_path):
    report = evaluate_offline("dry", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, speed_scale=0.05))
    assert _get(report, G_SPEED).outcome == "pass"
    assert report.passed


def test_mock_speed_above_commissioning_cap_unaffected(tmp_path):
    # mock never consults the commissioning record at all.
    report = evaluate_offline("mock", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, speed_scale=0.5))
    assert _get(report, G_SPEED).outcome == "pass"


# --------------------------------------------------------------------------------------
# MOT-05.3.R1 attempt-1 feedback: strict loaders raise GateConfigError (not TypeError) on a
# non-mapping section, and evaluate_offline reports 'error' rather than raising
# --------------------------------------------------------------------------------------

def test_load_execution_config_rejects_non_mapping_speed_scale(tmp_path):
    import yaml

    raw = yaml.safe_load(PRODUCTION_EXECUTION_CONFIG.read_text())
    raw["speed_scale"] = 0.5
    bad = tmp_path / "execution.yaml"
    bad.write_text(yaml.safe_dump(raw))
    with pytest.raises(GateConfigError):
        load_execution_config(bad)


def test_load_execution_config_rejects_non_mapping_tolerances(tmp_path):
    import yaml

    raw = yaml.safe_load(PRODUCTION_EXECUTION_CONFIG.read_text())
    raw["tolerances"] = 1.0
    bad = tmp_path / "execution.yaml"
    bad.write_text(yaml.safe_dump(raw))
    with pytest.raises(GateConfigError):
        load_execution_config(bad)


def test_load_execution_config_rejects_non_mapping_driver_profile(tmp_path):
    import yaml

    raw = yaml.safe_load(PRODUCTION_EXECUTION_CONFIG.read_text())
    raw["driver_profiles"]["vendor"] = "not a mapping"
    bad = tmp_path / "execution.yaml"
    bad.write_text(yaml.safe_dump(raw))
    with pytest.raises(GateConfigError):
        load_execution_config(bad)


def test_load_commissioning_rejects_non_mapping_estop(tmp_path):
    import yaml

    raw = yaml.safe_load(PRODUCTION_COMMISSIONING.read_text())
    raw["estop"] = "not a mapping"
    bad = tmp_path / "commissioning.yaml"
    bad.write_text(yaml.safe_dump(raw))
    with pytest.raises(GateConfigError):
        load_commissioning(bad)


def test_non_mapping_speed_scale_in_execution_config_does_not_raise(tmp_path):
    import yaml

    raw = yaml.safe_load(PRODUCTION_EXECUTION_CONFIG.read_text())
    raw["speed_scale"] = 0.5
    bad = tmp_path / "execution.yaml"
    bad.write_text(yaml.safe_dump(raw))
    report = evaluate_offline(
        "dry", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, execution_config_path=bad)
    )
    assert not report.passed
    assert _get(report, G_SPEED).outcome == "error"
    assert _get(report, G_LIMITS).outcome == "error"


# --------------------------------------------------------------------------------------
# MOT-05.3.R1 attempt-1 feedback: non-UTF-8 input files become 'error', never raise
# --------------------------------------------------------------------------------------

def _non_utf8_file(tmp_path, name: str) -> Path:
    path = tmp_path / name
    path.write_bytes(b"\xff\xfe\x00bad")
    return path


def test_non_utf8_trajectory_does_not_raise(tmp_path):
    bad_traj = _non_utf8_file(tmp_path, "bad_traj.json")
    report = evaluate_offline("real", bad_traj, **_common_kwargs(tmp_path))
    assert not report.passed
    assert _get(report, G_TRAJ).outcome == "error"


def test_non_utf8_scene_config_does_not_raise(tmp_path):
    bad_scene = _non_utf8_file(tmp_path, "bad_scene.yaml")
    report = evaluate_offline(
        "real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, scene_config_path=bad_scene)
    )
    assert not report.passed
    assert _get(report, G_SCENE).outcome == "error"


def test_non_utf8_end_effector_config_does_not_raise(tmp_path):
    bad_ee = _non_utf8_file(tmp_path, "bad_ee.yaml")
    report = evaluate_offline(
        "real", TRAJECTORY_SMOKE, **_common_kwargs(tmp_path, end_effector_config_path=bad_ee)
    )
    assert not report.passed
    assert _get(report, G_EE).outcome == "error"


def test_non_utf8_commissioning_record_does_not_raise(tmp_path):
    bad_commissioning = _non_utf8_file(tmp_path, "bad_commissioning.yaml")
    report = evaluate_offline(
        "real", TRAJECTORY_SMOKE,
        **_common_kwargs(tmp_path, commissioning_path=bad_commissioning, env=_ARM_REAL),
    )
    assert not report.passed
    assert _get(report, G_COMMISSIONING).outcome == "error"
    assert _get(report, G_ESTOP).outcome == "error"
    assert _get(report, G_SPEED).outcome == "error"


def test_non_utf8_approval_file_does_not_raise(tmp_path):
    bad_approval = _non_utf8_file(tmp_path, "bad_approval.json")
    report = evaluate_offline(
        "real", TRAJECTORY_SMOKE,
        **_common_kwargs(tmp_path, approval_path=bad_approval, env=_ARM_REAL),
    )
    assert not report.passed
    assert _get(report, G_APPROVAL).outcome == "fail"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_non_utf8_paths3d_does_not_raise(tmp_path, mode):
    traj_path, traj, _approval, _approval_path, kwargs = _passing_fixture_set(tmp_path)
    bad_p3d = tmp_path / "p3d.bin"
    bad_p3d.write_bytes(b"\x89PNG\r\n\x1a\n\xff\xff\xfe\x00")
    traj = copy.deepcopy(traj)
    traj["source"]["paths3d"] = {"path": "p3d.bin", "sha256": file_sha256(bad_p3d), "execution_eligible": True}
    traj_path = _write_json(tmp_path / "trajectory_bad_p3d.json", traj)
    approval = json.loads((tmp_path / "approval.json").read_text())
    approval["trajectory_sha256"] = file_sha256(traj_path)
    _write_json(tmp_path / "approval.json", approval)

    report = evaluate_offline(mode, traj_path, **kwargs)
    assert not report.passed
    assert _get(report, G_ELIGIBLE).outcome == "fail"
    assert G_ELIGIBLE in report.refusals


# --------------------------------------------------------------------------------------
# confirmation helpers (pure)
# --------------------------------------------------------------------------------------

def test_confirmation_phrase_format():
    sha = "ab" * 32
    assert confirmation_phrase(sha) == "EXECUTE abababab"


def test_confirmation_ok_exact_match_after_strip():
    sha = "f" * 64
    phrase = confirmation_phrase(sha)
    assert confirmation_ok(f"  {phrase}  ", sha)
    assert confirmation_ok(phrase, sha)


def test_confirmation_ok_rejects_case_variant():
    sha = "f" * 64
    assert not confirmation_ok(confirmation_phrase(sha).lower(), sha)


def test_confirmation_ok_rejects_missing_sha_suffix():
    assert not confirmation_ok("EXECUTE", "f" * 64)


def test_confirmation_ok_rejects_wrong_sha():
    sha = "f" * 64
    other = "e" * 64
    assert not confirmation_ok(confirmation_phrase(sha), other)


# --------------------------------------------------------------------------------------
# strict config loaders
# --------------------------------------------------------------------------------------

def test_load_execution_config_shipped_file_loads():
    cfg = load_execution_config(PRODUCTION_EXECUTION_CONFIG)
    assert cfg["schema"] == "crackvision.execution_config/1"
    assert cfg["speed_scale"]["real_default"] == pytest.approx(0.10)


def test_load_execution_config_rejects_unknown_key(tmp_path):
    raw = load_execution_config(PRODUCTION_EXECUTION_CONFIG)
    import yaml

    raw = dict(raw)
    raw["unexpected_key"] = 1
    bad = tmp_path / "execution.yaml"
    bad.write_text(yaml.safe_dump(raw))
    with pytest.raises(GateConfigError):
        load_execution_config(bad)


def test_load_commissioning_shipped_file_is_uncommissioned():
    cfg = load_commissioning(PRODUCTION_COMMISSIONING)
    assert cfg["commissioned"] is False
    assert cfg["limits_file_sha256"] is None
    assert cfg["estop"]["verified"] is False


def test_load_commissioning_rejects_unknown_key(tmp_path):
    import yaml

    raw = yaml.safe_load(PRODUCTION_COMMISSIONING.read_text())
    raw["unexpected_key"] = 1
    bad = tmp_path / "commissioning.yaml"
    bad.write_text(yaml.safe_dump(raw))
    with pytest.raises(GateConfigError):
        load_commissioning(bad)


def test_load_approval_rejects_unknown_key(tmp_path):
    approval = {
        "schema": "crackvision.execution_approval/1",
        "trajectory_sha256": "a" * 64,
        "approved_by": "op",
        "approved_utc": datetime.now(timezone.utc).isoformat(),
        "preview_artifacts": [],
        "unexpected_key": 1,
    }
    bad = _write_json(tmp_path / "approval.json", approval)
    with pytest.raises(GateConfigError):
        load_approval(bad)


# --------------------------------------------------------------------------------------
# §17.1/§17.2 purpose "view_move": G-TRAJ and G-ELIGIBLE (MOT-07.10.2)
# --------------------------------------------------------------------------------------

_VM_POS0 = [0.0] * 6
_VM_POS1 = [0.2, -0.1, -0.1, 0.0, 0.0, 0.0]


def _view_recheck():
    return {
        "max_pos_err_m": 0.0001, "max_ang_err_rad": 0.001,
        "all_valid": True, "within_limits": True, "max_joint_step_rad": 0.01,
    }


def _view_pose():
    return {"position_m": [0.0, 0.0, 0.5], "quat_xyzw": [0.0, 0.0, 0.0, 1.0]}


def _view_fov_check():
    return {"passed": True, "horizontal_margins_deg": [1.0] * 4, "vertical_margins_deg": [1.0] * 4}


def _view_plan_sources():
    return {
        "scene": {"path": "config/scene/scene.yaml", "sha256": file_sha256(SCENE_MEASURED)},
        "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": file_sha256(EE_MEASURED)},
        "limits": {"path": "config/robot/b601_dm_limits.yaml", "sha256": file_sha256(PRODUCTION_LIMITS)},
        "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": "c" * 64},
    }


def _write_view_plan(tmp_path, name="view_plan.json", *, chosen=True, positions=None) -> Path:
    plan = new_view_plan(_view_plan_sources(), {"view_distance_m": 0.25}, "--center", [0.1, 0.2], 0.0, [0.2, 0.2])
    if chosen:
        add_view_candidate(plan, 0.0, 0.0, _view_pose(), _view_fov_check(), True, True)
        pts = positions if positions is not None else [_VM_POS0, _VM_POS1]
        transit_leg = build_leg("transit", None, None, None, pts, [None] * len(pts), _view_recheck())
        finalize_view(plan, {"tilt_deg": 0.0, "yaw_deg": 0.0}, transit_leg, VIEW_LIMITS)
    else:
        finalize_view(plan, None, None, VIEW_LIMITS)
    path = tmp_path / name
    write_view_plan(plan, path)
    return path


def _view_move_trajectory(
    view_plan_path: Path, *, points=None, limits_sha=None, ee_sha=None, scene_sha=None,
) -> dict:
    return {
        "schema": "crackvision.joint_trajectory/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "MOT-07.10.2 fixture",
        "purpose": "view_move",
        "planning_frame": "base_link",
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        "points": points if points is not None else [
            {"t_s": 0.0, "positions": _VM_POS0},
            {"t_s": 1.0, "positions": _VM_POS1},
        ],
        "limits_file": {
            "path": "config/robot/b601_dm_limits.yaml",
            "sha256": limits_sha if limits_sha is not None else file_sha256(PRODUCTION_LIMITS),
        },
        "end_effector_config_sha256": ee_sha if ee_sha is not None else file_sha256(EE_MEASURED),
        "scene_config_sha256": scene_sha if scene_sha is not None else file_sha256(SCENE_MEASURED),
        "source": {"paths3d": None, "view_plan": {"path": view_plan_path.name, "sha256": file_sha256(view_plan_path)}},
    }


def _view_move_kwargs(tmp_path, **overrides):
    return _common_kwargs(
        tmp_path,
        scene_config_path=SCENE_MEASURED,
        end_effector_config_path=EE_MEASURED,
        env=_ARM_REAL,
        **overrides,
    )


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_valid_binding_passes_traj_and_eligible(tmp_path, mode):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_TRAJ).outcome == "pass"
    assert _get(report, G_ELIGIBLE).outcome == "pass"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_missing_view_plan_file_fails_eligible(tmp_path, mode):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    vp_path.unlink()
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_ELIGIBLE).outcome == "fail"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_sha_mismatch_fails_eligible(tmp_path, mode):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    traj["source"]["view_plan"]["sha256"] = "a" * 64
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_ELIGIBLE).outcome == "fail"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_non_view_plan_file_fails_eligible(tmp_path, mode):
    not_a_plan = tmp_path / "view_plan.json"
    _write_json(not_a_plan, {"schema": "crackvision.paths3d/1", "counts": {}, "execution_eligible": True})
    traj = _view_move_trajectory(not_a_plan)
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_ELIGIBLE).outcome == "fail"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_null_chosen_transit_fails_eligible(tmp_path, mode):
    vp_path = _write_view_plan(tmp_path, chosen=False)
    traj = _view_move_trajectory(vp_path)
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_ELIGIBLE).outcome == "fail"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_endpoint_mismatch_fails_eligible(tmp_path, mode):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    traj["points"][-1]["positions"] = [0.9] * 6  # no longer matches transit_leg's last position
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_ELIGIBLE).outcome == "fail"


@pytest.mark.parametrize(
    "sha_field", ["limits_file", "end_effector_config_sha256", "scene_config_sha256"],
)
def test_view_move_sha_binding_mismatch_warns_outside_real_fails_in_real(tmp_path, sha_field):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    bad_sha = "d" * 64
    if sha_field == "limits_file":
        traj["limits_file"]["sha256"] = bad_sha
    else:
        traj[sha_field] = bad_sha
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    for mode in ("mock", "dry"):
        report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
        assert _get(report, G_ELIGIBLE).outcome == "warn"

    report = evaluate_offline("real", traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_ELIGIBLE).outcome == "fail"


@pytest.mark.parametrize(
    "sha_field", ["limits_file", "end_effector_config_sha256", "scene_config_sha256"],
)
def test_view_move_null_sha_binding_counts_as_mismatch(tmp_path, sha_field):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    if sha_field == "limits_file":
        traj["limits_file"]["sha256"] = None
    else:
        traj[sha_field] = None
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline("real", traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_ELIGIBLE).outcome == "fail"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_schema_refusal_nonnull_paths3d_fails_traj(tmp_path, mode):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    traj["source"]["paths3d"] = {"path": "a", "sha256": "a" * 64, "execution_eligible": True}
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_TRAJ).outcome == "fail"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_test_purpose_with_view_plan_key_fails_traj(tmp_path, mode):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    traj["purpose"] = "test"
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
    assert _get(report, G_TRAJ).outcome == "fail"


# --------------------------------------------------------------------------------------
# MOT-07.10.2.R1 (finding c13b05680a0183f6): non-UTF-8 view_plan is a G-ELIGIBLE 'fail'
# (sha matched, so it is the bound file -- just not a view plan); an unreadable or
# missing limits file is a G-ELIGIBLE 'error' (or 'fail') with G-LIMITS/G-LIMITS-HASH
# staying 'error' too. Neither case raises out of evaluate_offline.
# --------------------------------------------------------------------------------------

@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_non_utf8_view_plan_fails_eligible(tmp_path, mode):
    vp_path = tmp_path / "view_plan.json"
    vp_path.write_bytes(b"\xff\xfe\x00bad")
    traj = _view_move_trajectory(vp_path)
    traj_path = _write_json(tmp_path / "trajectory.json", traj)

    report = evaluate_offline(mode, traj_path, **_view_move_kwargs(tmp_path))
    assert not report.passed
    assert _get(report, G_ELIGIBLE).outcome == "fail"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_unparseable_limits_path_does_not_raise(tmp_path, mode):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    traj_path = _write_json(tmp_path / "trajectory.json", traj)
    bad_limits = tmp_path / "bad_limits.yaml"
    bad_limits.write_text("joints: [unterminated flow sequence\n")

    report = evaluate_offline(
        mode, traj_path, **_view_move_kwargs(tmp_path, limits_path=bad_limits)
    )
    assert not report.passed
    assert _get(report, G_ELIGIBLE).outcome in ("error", "fail")
    assert _get(report, G_LIMITS).outcome == "error"
    assert _get(report, G_LIMITS_HASH).outcome == "error"


@pytest.mark.parametrize("mode", ["mock", "dry", "real"])
def test_view_move_missing_limits_path_does_not_raise(tmp_path, mode):
    vp_path = _write_view_plan(tmp_path)
    traj = _view_move_trajectory(vp_path)
    traj_path = _write_json(tmp_path / "trajectory.json", traj)
    missing_limits = tmp_path / "no_such_limits.yaml"

    report = evaluate_offline(
        mode, traj_path, **_view_move_kwargs(tmp_path, limits_path=missing_limits)
    )
    assert not report.passed
    assert _get(report, G_ELIGIBLE).outcome in ("error", "fail")
    assert _get(report, G_LIMITS).outcome == "error"
    assert _get(report, G_LIMITS_HASH).outcome == "error"
