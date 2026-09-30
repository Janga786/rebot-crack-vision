"""Offline tests for config/robot/end_effector.yaml and the ADR-014 description overlay.

Run: bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_description/test -q -p no:cacheprovider
(the xacro render tests need env_ros.sh so `$(find rebotarm_moveit_config)` resolves; they skip otherwise).
"""

from __future__ import annotations

import copy
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PKG))

from crackvision_description.end_effector import (  # noqa: E402
    EndEffectorError,
    EndEffectorNotCalibratedError,
    assert_commissioning_ready,
    load_config,
    nominal_items,
    rpy_to_matrix,
    transform,
)

CONFIG = ROOT / "config" / "robot" / "end_effector.yaml"
URDF_XACRO = PKG / "urdf" / "b601_dm_end_effector.urdf.xacro"
SRDF_XACRO = PKG / "srdf" / "b601_dm_end_effector.srdf.xacro"


def _raw():
    return yaml.safe_load(CONFIG.read_text())


def _write(tmp_path, raw) -> Path:
    p = tmp_path / "ee.yaml"
    p.write_text(yaml.safe_dump(raw))
    return p


def test_committed_config_loads_and_is_all_nominal():
    cfg = load_config(CONFIG)
    assert cfg["parent_link"] == "gripper_link"
    assert cfg["tool"]["frame"] == "tool_tip" and cfg["wrist_camera"]["frame"] == "camera_link"
    assert nominal_items(cfg) == ["collision:camera_housing_link", "collision:camera_mount_link", "tool", "wrist_camera"]
    with pytest.raises(EndEffectorNotCalibratedError):
        assert_commissioning_ready(cfg)


def test_gate_passes_only_when_everything_is_measured(tmp_path):
    raw = _raw()
    raw["tool"]["value_status"] = "measured"
    raw["wrist_camera"]["value_status"] = "measured"
    with pytest.raises(EndEffectorNotCalibratedError, match="collision"):
        assert_commissioning_ready(load_config(_write(tmp_path, raw)))
    for c in raw["collision"]:
        c["value_status"] = "measured"
    assert_commissioning_ready(load_config(_write(tmp_path, raw)))


def test_documented_derived_numbers_match_the_transform():
    cfg = load_config(CONFIG)
    t_cam = transform(cfg["wrist_camera"])
    axis = t_cam[:3, :3] @ np.array([1.0, 0.0, 0.0])
    derived = cfg["wrist_camera"]["derived"]
    assert np.allclose(axis, derived["optical_axis_in_parent"], atol=1e-3)
    # 15 deg between the optical axis and the tool boresight (stock Seeed mount pitch)
    assert math.degrees(math.acos(float(np.dot(axis, cfg["tool"]["pointing_axis"])))) == pytest.approx(15.0, abs=1e-6)
    tip = transform(cfg["tool"])[:3, 3]
    assert np.linalg.norm(t_cam[:3, 3] - tip) == pytest.approx(derived["optical_centre_to_tool_tip_m"], abs=5e-4)
    gripper_tcp = np.array([-0.0443, 0.0, 0.0])  # vendor rebotarm.urdf.xacro, for the documented distance only
    assert np.linalg.norm(t_cam[:3, 3] - gripper_tcp) == pytest.approx(derived["optical_centre_to_gripper_tcp_m"], abs=5e-4)
    # camera_link is right-handed with +z pointing away from the gripper axis (camera "up")
    r = t_cam[:3, :3]
    assert np.linalg.det(r) == pytest.approx(1.0) and (r @ [0, 0, 1])[2] < -0.9


@pytest.mark.parametrize("mutate, message", [
    (lambda r: r.update(schema="crackvision.end_effector/2"), "schema"),
    (lambda r: r.update(extra=1), "unknown key"),
    (lambda r: r["tool"].update(value_status="guessed"), "value_status"),
    (lambda r: r["tool"].update(pointing_axis=[0, 0, 0]), "non-zero"),
    (lambda r: r["tool"].pop("source"), "missing"),
    (lambda r: r["wrist_camera"].update(frame="d405_link"), "camera_link"),
    (lambda r: r["wrist_camera"].update(xyz_m=[0, 0]), "3-element"),
    (lambda r: r.update(collision_padding_m=0.2), "collision_padding_m"),
    (lambda r: r["collision"][0].update(box_size_m=[0.01, 0, 0.01]), "all-positive"),
    (lambda r: r["collision"][1].update(id=r["collision"][0]["id"]), "duplicates"),
    (lambda r: r["collision"][0].update(frame="link6"), "must be one of"),
    (lambda r: r["allowed_self_collisions"].append(["link5", "link6", "vendor pair"]), "overlay collision link"),
    (lambda r: r["allowed_self_collisions"].append(list(r["allowed_self_collisions"][0])), "duplicate pair"),
])
def test_invalid_configs_are_refused(tmp_path, mutate, message):
    raw = copy.deepcopy(_raw())
    mutate(raw)
    with pytest.raises(EndEffectorError, match=message):
        load_config(_write(tmp_path, raw))


def test_missing_file_is_refused(tmp_path):
    with pytest.raises(EndEffectorError, match="not found"):
        load_config(tmp_path / "nope.yaml")


def test_rpy_matches_urdf_convention():
    # URDF rpy = fixed-axis XYZ: R = Rz(y) Ry(p) Rx(r)
    r = rpy_to_matrix([math.pi, -math.radians(15.0), 0.0])
    c, s = math.cos(math.radians(15.0)), math.sin(math.radians(15.0))
    assert np.allclose(r, [[c, 0, s], [0, -1, 0], [s, 0, -c]], atol=1e-12)


xacro = pytest.importorskip("xacro")


def _render(path: Path, **mappings) -> ET.Element:
    try:
        doc = xacro.process_file(str(path), mappings={k: str(v) for k, v in mappings.items()})
    except Exception as exc:  # noqa: BLE001 - underlay not sourced
        pytest.skip(f"xacro render needs scripts/ros/env_ros.sh (underlay): {exc}")
    return ET.fromstring(doc.toxml())


def _origin(urdf: ET.Element, child: str):
    for j in urdf.findall("joint"):
        if j.find("child").get("link") == child:
            o = j.find("origin")
            return (j.find("parent").get("link"), [float(v) for v in o.get("xyz").split()],
                    [float(v) for v in o.get("rpy").split()])
    return None


def test_urdf_overlay_places_every_frame_from_the_config():
    cfg = load_config(CONFIG)
    urdf = _render(URDF_XACRO, end_effector_config=CONFIG, nominal_camera_frames="false")
    links = {link.get("name") for link in urdf.findall("link")}
    assert {"base_link", "gripper_link", "gripper_tcp", "tool_tip", "camera_link",
            "camera_mount_link", "camera_housing_link"} <= links
    assert "camera_color_optical_frame" not in links  # the real driver owns these frames
    for block, parent in ((cfg["tool"], "gripper_link"), (cfg["wrist_camera"], "gripper_link")):
        p, xyz, rpy = _origin(urdf, block["frame"])
        assert p == parent and np.allclose(xyz, block["xyz_m"]) and np.allclose(rpy, block["rpy_rad"])
    pad = cfg["collision_padding_m"]
    for c in cfg["collision"]:
        p, xyz, rpy = _origin(urdf, c["id"])
        assert p == c["frame"] and np.allclose(xyz, c["xyz_m"]) and np.allclose(rpy, c["rpy_rad"])
        link = next(link for link in urdf.findall("link") if link.get("name") == c["id"])
        size = [float(v) for v in link.find("collision/geometry/box").get("size").split()]
        assert np.allclose(size, np.array(c["box_size_m"]) + 2 * pad)


def test_nominal_camera_frames_only_on_request():
    urdf = _render(URDF_XACRO, end_effector_config=CONFIG, nominal_camera_frames="true")
    p, xyz, rpy = _origin(urdf, "camera_color_optical_frame")
    assert p == "camera_color_frame" and np.allclose(xyz, 0) and np.allclose(rpy, [-math.pi / 2, 0, -math.pi / 2])


def test_srdf_overlay_keeps_vendor_groups_and_adds_rigid_pairs_only():
    cfg = load_config(CONFIG)
    srdf = _render(SRDF_XACRO, end_effector_config=CONFIG)
    groups = {g.get("name") for g in srdf.findall("group")}
    assert {"arm", "gripper"} <= groups
    chain = srdf.find("group[@name='arm']/chain")
    assert chain.get("tip_link") == "gripper_tcp"  # vendor chain untouched; task frames are resolved by MoveIt
    pairs = {frozenset((d.get("link1"), d.get("link2"))) for d in srdf.findall("disable_collisions")}
    for a, b, _ in cfg["allowed_self_collisions"]:
        assert frozenset((a, b)) in pairs
    overlay = {c["id"] for c in cfg["collision"]}
    for arm_link in ("link1", "link2", "link3", "link4", "link5", "link6", "base_link"):
        for o in overlay:
            assert frozenset((arm_link, o)) not in pairs, f"{o} vs {arm_link} must stay collision-checked"
