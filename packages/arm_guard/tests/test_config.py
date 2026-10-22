"""Config validation and the approval (hash) gate."""

import json
import shutil

import pytest

from arm_guard import GuardConfigError
from arm_guard import config as C
from conftest import CONFIG, REPO


@pytest.fixture
def cfg_copy(tmp_path):
    """The repo config + URDF copied to a temp tree with the same relative layout, and approved."""
    (tmp_path / "config/robot").mkdir(parents=True)
    (tmp_path / "presentation/sim").mkdir(parents=True)
    shutil.copy(REPO / "presentation/sim/reBot_B601_DM_with_gripper.urdf", tmp_path / "presentation/sim")
    p = tmp_path / "config/robot/arm_guard.toml"
    shutil.copy(CONFIG, p)
    _approve(p)
    return p


def _approve(p):
    urdf = p.parents[2] / "presentation/sim/reBot_B601_DM_with_gripper.urdf"
    C.approval_path(p).write_text(json.dumps({"config_sha256": C.sha256_file(p), "urdf_sha256": C.sha256_file(urdf)}))


def _edit(p, old, new, approve=True):
    s = p.read_text()
    assert old in s
    p.write_text(s.replace(old, new, 1))
    if approve:
        _approve(p)


def test_repo_config_is_approved():
    C.load(CONFIG)                                             # the committed config must be loadable as-is


def test_unapproved_edit_refused(cfg_copy):
    C.load(cfg_copy)
    _edit(cfg_copy, "vel_cap = 1.2", "vel_cap = 1.3", approve=False)
    with pytest.raises(GuardConfigError, match="changed since approval"):
        C.load(cfg_copy)


def test_missing_approval_refused(cfg_copy):
    C.approval_path(cfg_copy).unlink()
    with pytest.raises(GuardConfigError, match="not approved"):
        C.load(cfg_copy)


def test_urdf_change_refused(cfg_copy):
    urdf = cfg_copy.parents[2] / "presentation/sim/reBot_B601_DM_with_gripper.urdf"
    urdf.write_text(urdf.read_text().replace('lower="-2.8"', 'lower="-3.8"', 1))
    with pytest.raises(GuardConfigError, match="urdf_sha256"):
        C.load(cfg_copy)


def test_wider_than_urdf_needs_reason(cfg_copy):
    _edit(cfg_copy, "lower = -2.8\n", "lower = -3.9\n")
    with pytest.raises(GuardConfigError, match="wider than URDF"):
        C.load(cfg_copy)
    _edit(cfg_copy, "lower = -3.9\n", 'lower = -3.9\nbeyond_urdf_reason = "operator demo: audience angle"\n')
    assert C.load(cfg_copy).joints["j1"].lower == -3.9


def test_limits_inside_mit_mapping(cfg_copy):
    _edit(cfg_copy, "tau_ff_max = 2.5", "tau_ff_max = 12.0")   # 4310 TMAX is 10
    with pytest.raises(GuardConfigError, match="PMAX/VMAX/TMAX"):
        C.load(cfg_copy)


@pytest.mark.parametrize("old,new,match", [
    ("vel_abort = 1.5", "vel_abort = 1.0", "vel_abort"),
    ("hold_kp = 100.0", "hold_kp = 500.0", "hold gains"),
    ("motor_id = 0x02", "motor_id = 0x01", "duplicate"),
    ('link = "link4"', 'link = "nolink"', "not in URDF"),
    ("max_step = 0.15", "max_step = -0.15", "must be > 0"),
    ("stale_feedback_s = 0.25", "stale_feedback_s = nan", "finite"),
    ("j5 = [-0.40, 0.40]\n", "", "every joint needs a startup window"),
    ("j3 = [-0.40, 0.05]", "j3 = [-0.40, 0.20]", "must lie within"),
    ("j1 = [-0.40, 0.40]", "j1 = [-2.0, 2.0]", "narrower than"),
    ('turn_ambiguous = ["j1"]', 'turn_ambiguous = ["j9"]', "unknown joints"),
    ("rest_tolerance_rad = 0.50", "rest_tolerance_rad = -0.5", "must be >= 0"),
])
def test_invalid_values_refused(cfg_copy, old, new, match):
    _edit(cfg_copy, old, new)
    with pytest.raises(GuardConfigError, match=match):
        C.load(cfg_copy)


def test_missing_arm_joint_refused(cfg_copy):
    s = cfg_copy.read_text()
    start = s.index("[joints.j6]")
    cfg_copy.write_text(s[:start] + s[s.index("[joints.j7]"):])
    with pytest.raises(GuardConfigError, match="arm joints"):
        C.load(cfg_copy, check_approval=False)


def test_malformed_toml(cfg_copy):
    cfg_copy.write_text("[guard\n")
    with pytest.raises(GuardConfigError, match="cannot read"):
        C.load(cfg_copy)
