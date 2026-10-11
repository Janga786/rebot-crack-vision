"""tests/test_check_vendor_driver.py — MOT-11 vendor-driver readiness checker tests."""

from __future__ import annotations

import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "motion"))

import check_vendor_driver as cvd  # noqa: E402

FIXTURES = REPO_ROOT / "tests" / "fixtures" / "vendor_driver" / "good"


def _hash_tree(root: Path) -> dict[str, str]:
    hashes = {}
    for path in sorted(root.rglob("*")):
        if path.is_file():
            hashes[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return hashes


def _make_tree(tmp_path: Path) -> Path:
    dest = tmp_path / "rebot_ws"
    shutil.copytree(FIXTURES / "rebot_ws", dest)
    return dest


def _run(tmp_path: Path, rebot_ws: Path, json_out: Path | None = None, **overrides):
    argv = [
        "--rebot-ws", str(rebot_ws),
        "--facts", str(overrides.get("facts", FIXTURES / "facts.yaml")),
        "--limits-file", str(overrides.get("limits_file", FIXTURES / "limits.yaml")),
        "--execution-config", str(overrides.get("execution_config", FIXTURES / "execution.yaml")),
        "--executor-file", str(overrides.get("executor_file", FIXTURES / "executor.py")),
        "--driver-pythonpath", str(FIXTURES / "deps"),
        "--format", "json",
        "--root", str(tmp_path),
    ]
    if json_out is not None:
        argv += ["--json-out", str(json_out)]
    return cvd.main(argv)


def _run_capture(tmp_path: Path, rebot_ws: Path, **overrides) -> dict:
    json_out = tmp_path / "report.json"
    rc = _run(tmp_path, rebot_ws, json_out=json_out, **overrides)
    report = json.loads(json_out.read_text())
    return {"rc": rc, "report": report}


def _fail_ids(report: dict) -> list[str]:
    return [c["id"] for c in report["checks"] if c["status"] == "FAIL"]


def test_good_fixture_exits_zero_and_has_no_fail(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    result = _run_capture(tmp_path, rebot_ws)
    assert result["rc"] == cvd.EXIT_OK
    assert _fail_ids(result["report"]) == []


def test_read_only_does_not_modify_vendor_tree(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    before = _hash_tree(rebot_ws)
    _run(tmp_path, rebot_ws)
    after = _hash_tree(rebot_ws)
    assert before == after


def test_missing_facts_file_exits_usage_error(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    rc = _run(tmp_path, rebot_ws, facts=tmp_path / "does_not_exist.yaml")
    assert rc == cvd.EXIT_USAGE


def test_wrong_motor_id_fails_v_ids(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    arm_yaml = rebot_ws / "src/rebotarm_bringup/config/arm.yaml"
    arm_yaml.write_text(arm_yaml.read_text().replace("motor_id: 0x02", "motor_id: 0x09"))
    result = _run_capture(tmp_path, rebot_ws)
    assert result["rc"] == cvd.EXIT_NOT_READY
    assert _fail_ids(result["report"]) == ["V-IDS"]


def test_swapped_joint_order_fails_v_order(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    arm_yaml = rebot_ws / "src/rebotarm_bringup/config/arm.yaml"
    text = arm_yaml.read_text()
    header, _, rest = text.partition("joints:\n")
    blocks = ["  - name: joint" + b for b in rest.split("  - name: joint")[1:]]
    assert len(blocks) == 6
    blocks[0], blocks[1] = blocks[1], blocks[0]
    arm_yaml.write_text(header + "joints:\n" + "".join(blocks))
    result = _run_capture(tmp_path, rebot_ws)
    assert result["rc"] == cvd.EXIT_NOT_READY
    assert _fail_ids(result["report"]) == ["V-ORDER"]


def test_direction_key_fails_v_direction(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    arm_yaml = rebot_ws / "src/rebotarm_bringup/config/arm.yaml"
    arm_yaml.write_text(arm_yaml.read_text().replace(
        "  - name: joint1\n", "  - name: joint1\n    direction: -1\n"
    ))
    result = _run_capture(tmp_path, rebot_ws)
    assert result["rc"] == cvd.EXIT_NOT_READY
    assert _fail_ids(result["report"]) == ["V-DIRECTION"]


def test_positive_gripper_open_fails_v_gripper_sign(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    ros_services = rebot_ws / "src/rebotarmcontroller/rebotarmcontroller/ros_services.py"
    ros_services.write_text(ros_services.read_text().replace("_GRIPPER_OPEN_POSITION = -4.9", "_GRIPPER_OPEN_POSITION = 4.9"))
    result = _run_capture(tmp_path, rebot_ws)
    assert result["rc"] == cvd.EXIT_NOT_READY
    assert _fail_ids(result["report"]) == ["V-GRIPPER-SIGN"]


def test_safe_park_q_above_limit_fails_v_safe_park(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    safe_park = rebot_ws / "src/rebotarm_bringup/config/safe_park.yaml"
    safe_park.write_text(safe_park.read_text().replace("- -0.01\n      - -0.01\n", "- 0.5\n      - -0.01\n"))
    result = _run_capture(tmp_path, rebot_ws)
    assert result["rc"] == cvd.EXIT_NOT_READY
    assert _fail_ids(result["report"]) == ["V-SAFE-PARK"]


def test_missing_sdk_export_fails_v_sdk_api(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    actuator_init = rebot_ws / "third_party/reBotArm_control_py/reBotArm_control_py/actuator/__init__.py"
    actuator_init.write_text("__all__ = []\n")
    result = _run_capture(tmp_path, rebot_ws)
    assert result["rc"] == cvd.EXIT_NOT_READY
    assert _fail_ids(result["report"]) == ["V-SDK-API"]


def test_missing_deps_fails_v_pydeps(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    empty_deps = tmp_path / "empty_deps"
    empty_deps.mkdir()
    json_out = tmp_path / "report.json"
    rc = cvd.main([
        "--rebot-ws", str(rebot_ws),
        "--facts", str(FIXTURES / "facts.yaml"),
        "--limits-file", str(FIXTURES / "limits.yaml"),
        "--execution-config", str(FIXTURES / "execution.yaml"),
        "--executor-file", str(FIXTURES / "executor.py"),
        "--driver-pythonpath", str(empty_deps),
        "--format", "json",
        "--root", str(tmp_path),
        "--json-out", str(json_out),
    ])
    report = json.loads(json_out.read_text())
    assert rc == cvd.EXIT_NOT_READY
    assert _fail_ids(report) == ["V-PYDEPS"]


def test_reliable_executor_qos_fails_v_qos(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    bad_executor = tmp_path / "bad_executor.py"
    bad_executor.write_text(
        "from rclpy.qos import QoSProfile, QoSReliabilityPolicy\n\n"
        "def subscribe_joint_states(node, topic, cb):\n"
        "    qos = QoSProfile(depth=10)\n"
        "    qos.reliability = QoSReliabilityPolicy.RELIABLE\n"
        "    return node.create_subscription(JointState, topic, cb, qos)\n"
    )
    result = _run_capture(tmp_path, rebot_ws, executor_file=bad_executor)
    assert result["rc"] == cvd.EXIT_NOT_READY
    assert _fail_ids(result["report"]) == ["V-QOS"]


def test_bad_channel_fails_v_port(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    arm_yaml = rebot_ws / "src/rebotarm_bringup/config/arm.yaml"
    arm_yaml.write_text(arm_yaml.read_text().replace("channel: /dev/ttyACM0", "channel: can0"))
    result = _run_capture(tmp_path, rebot_ws)
    assert result["rc"] == cvd.EXIT_NOT_READY
    assert _fail_ids(result["report"]) == ["V-PORT"]


def test_every_check_citation_has_a_line_number(tmp_path):
    rebot_ws = _make_tree(tmp_path)
    result = _run_capture(tmp_path, rebot_ws)
    for check in result["report"]["checks"]:
        assert re.search(r":\d+", check["citation"]), f"{check['id']} citation lacks a line number: {check['citation']!r}"


def test_text_format_lists_every_check_id(tmp_path, capsys):
    rebot_ws = _make_tree(tmp_path)
    rc = _run(tmp_path, rebot_ws)
    out = capsys.readouterr().out
    for check_id in (
        "V-PORT", "V-BAUD", "V-IDS", "V-ORDER", "V-DIRECTION", "V-ZERO", "V-GRIPPER-SIGN",
        "V-GRIPPER-RANGE", "V-GRIPPER-WIDTH", "V-GRIPPER-UNITS", "V-SAFE-PARK", "V-STARTUP",
        "V-GAINS", "V-SDK-API", "V-PYDEPS", "V-QOS", "V-PROFILE", "V-ARBITRATION", "V-URDF",
    ):
        assert check_id in out
    assert rc == cvd.EXIT_OK
