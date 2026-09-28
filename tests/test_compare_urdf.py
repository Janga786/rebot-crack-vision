"""tests/test_compare_urdf.py — URDF diff logic for MOT-01, synthetic fixtures only.

Never reads ~/rebot_ws or presentation/sim: the real robot files are exercised manually
(docs/motion/ROBOT_MODEL.md records that run), not by this suite.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

_MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "motion" / "compare_urdf.py"
_spec = importlib.util.spec_from_file_location("compare_urdf", _MODULE_PATH)
compare_urdf = importlib.util.module_from_spec(_spec)
sys.modules["compare_urdf"] = compare_urdf
_spec.loader.exec_module(compare_urdf)


_VENDOR_URDF = """<?xml version="1.0"?>
<robot name="fixture">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <limit lower="-2.8" upper="2.8" effort="27" velocity="50"/>
  </joint>
  <joint name="joint2" type="revolute">
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="link1"/>
    <child link="link2"/>
    <limit lower="-3.14" upper="0" effort="27" velocity="50"/>
  </joint>
</robot>
"""

# Differs from vendor: joint2 axis flipped and effort changed, an extra "gripper" joint/link
# added, and "link2" renamed to "link2b" (so it shows up as link-only-in-other / vendor).
_OTHER_URDF = """<?xml version="1.0"?>
<robot name="fixture">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2b"/>
  <link name="gripper_link"/>
  <joint name="joint1" type="revolute">
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <limit lower="-2.8" upper="2.8" effort="27" velocity="50"/>
  </joint>
  <joint name="joint2" type="revolute">
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 -1"/>
    <parent link="link1"/>
    <child link="link2b"/>
    <limit lower="-3.14" upper="0" effort="99" velocity="50"/>
  </joint>
  <joint name="gripper_joint" type="prismatic">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <parent link="link2b"/>
    <child link="gripper_link"/>
    <limit lower="0" upper="0.07" effort="100" velocity="15"/>
  </joint>
</robot>
"""


@pytest.fixture
def vendor_path(tmp_path: Path) -> Path:
    path = tmp_path / "vendor.urdf"
    path.write_text(_VENDOR_URDF, encoding="utf-8")
    return path


@pytest.fixture
def other_path(tmp_path: Path) -> Path:
    path = tmp_path / "other.urdf"
    path.write_text(_OTHER_URDF, encoding="utf-8")
    return path


def test_parse_urdf_extracts_joints_and_links(vendor_path: Path) -> None:
    model = compare_urdf.parse_urdf(vendor_path)
    assert set(model["links"]) == {"base_link", "link1", "link2"}
    assert model["joints"]["joint1"]["axis_xyz"] == "0 0 1"
    assert model["joints"]["joint1"]["limit_lower"] == "-2.8"


def test_identical_models_report_no_diffs(vendor_path: Path) -> None:
    model = compare_urdf.parse_urdf(vendor_path)
    report = compare_urdf.compare(model, model)
    assert report["identical"] is True
    assert report["joint_diffs"] == {}


def test_numeric_origin_formatting_does_not_count_as_a_diff(vendor_path: Path, tmp_path: Path) -> None:
    # "0 0 0.1" vs "0.0 0.0 0.1" is the same vector, not a real difference.
    reformatted = tmp_path / "reformatted.urdf"
    reformatted.write_text(_VENDOR_URDF.replace('xyz="0 0 0.1"', 'xyz="0.0 0.0 0.1"'), encoding="utf-8")
    vendor_model = compare_urdf.parse_urdf(vendor_path)
    other_model = compare_urdf.parse_urdf(reformatted)
    report = compare_urdf.compare(vendor_model, other_model)
    assert report["identical"] is True


def test_link_and_joint_set_diffs_detected(vendor_path: Path, other_path: Path) -> None:
    vendor_model = compare_urdf.parse_urdf(vendor_path)
    other_model = compare_urdf.parse_urdf(other_path)
    report = compare_urdf.compare(vendor_model, other_model)

    assert report["identical"] is False
    assert "link2" in report["links_only_in_vendor"]
    assert "link2b" in report["links_only_in_other"]
    assert "gripper_link" in report["links_only_in_other"]
    assert "gripper_joint" in report["joints_only_in_other"]


def test_joint_axis_and_effort_diffs_reported_with_both_values(vendor_path: Path, other_path: Path) -> None:
    vendor_model = compare_urdf.parse_urdf(vendor_path)
    other_model = compare_urdf.parse_urdf(other_path)
    report = compare_urdf.compare(vendor_model, other_model)

    diffs = {d["field"]: d for d in report["joint_diffs"]["joint2"]}
    assert diffs["axis_xyz"] == {"field": "axis_xyz", "vendor": "0 0 1", "other": "0 0 -1"}
    assert diffs["limit_effort"] == {"field": "limit_effort", "vendor": "27", "other": "99"}


def test_cli_main_writes_json_report_and_exits_ok(vendor_path: Path, other_path: Path, tmp_path: Path) -> None:
    out = tmp_path / "report.json"
    exit_code = compare_urdf.main([
        "--vendor", str(vendor_path),
        "--other", str(other_path),
        "--json", str(out),
        "--quiet",
    ])
    assert exit_code == compare_urdf.EXIT_OK
    report = json.loads(out.read_text(encoding="utf-8"))
    assert report["identical"] is False


def test_cli_main_missing_file_is_precondition_failure(tmp_path: Path) -> None:
    exit_code = compare_urdf.main([
        "--vendor", str(tmp_path / "does_not_exist.urdf"),
        "--other", str(tmp_path / "also_missing.urdf"),
        "--quiet",
    ])
    assert exit_code == compare_urdf.EXIT_PRECONDITION
