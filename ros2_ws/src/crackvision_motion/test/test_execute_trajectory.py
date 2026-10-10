"""Unit tests for execute_trajectory (MOT-05.4).

Usage: bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; \
    python3 -m pytest ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py -q -p no:cacheprovider'

These tests never need a running ROS graph: offline-refusal paths never call `rclpy.init`
(verified by monkeypatching it to raise), and the few tests that do construct a real `rclpy`
node/context fake out `OnlineChecks`/`ActionClient` so no live MoveIt/driver stack is required.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from types import SimpleNamespace
from pathlib import Path
from typing import Dict, List

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import crackvision_motion.execute_trajectory as et  # noqa: E402
from crackvision_motion.cli_common import PreconditionError  # noqa: E402
from crackvision_motion.execution_gate import load_execution_config  # noqa: E402
from crackvision_motion.joint_trajectory import JOINT_NAMES  # noqa: E402
from crackvision_motion.reachability_core import ConfigError  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[4]
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
TRAJECTORY_SMOKE = FIXTURES_DIR / "trajectory_smoke.json"

_NO_ARM: Dict[str, str] = {}
_ARM_REAL = {"CRACKVISION_ARM_REAL": "1"}


def _args(tmp_path, **overrides) -> argparse.Namespace:
    defaults = dict(
        root=REPO_ROOT,
        mode=None,
        trajectory=str(TRAJECTORY_SMOKE),
        profile=None,
        execution_config=None,
        commissioning=None,
        approval=None,
        speed_scale=None,
        limits=None,
        scene_config=None,
        end_effector_config=None,
        service_timeout_s=5.0,
        record_dir=str(tmp_path / "execution"),
    )
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


# --------------------------------------------------------------------------------------
# order: an offline refusal never touches rclpy
# --------------------------------------------------------------------------------------

def test_offline_refusal_never_calls_rclpy_init(tmp_path, monkeypatch):
    def _boom(*a, **kw):
        raise AssertionError("rclpy.init must not be called on an offline refusal")

    monkeypatch.setattr(et.rclpy, "init", _boom)
    args = _args(tmp_path, mode="real")  # CRACKVISION_ARM_REAL unset -> G-ARM fails
    with pytest.raises(PreconditionError):
        et.run(args, env=_NO_ARM)

    records = list((tmp_path / "execution").glob("*.json"))
    assert len(records) == 1
    record = json.loads(records[0].read_text())
    assert record["outcome"] == "refused"
    assert record["mode"] == "real"
    assert any(c["gate"] == "G-ARM" and c["outcome"] == "fail" for c in record["gate_report"])


def test_offline_refusal_record_written_even_though_refused(tmp_path, monkeypatch):
    monkeypatch.setattr(et.rclpy, "init", lambda *a, **kw: (_ for _ in ()).throw(AssertionError("no ROS")))
    args = _args(tmp_path, mode="real")
    with pytest.raises(PreconditionError):
        et.run(args, env=_NO_ARM)
    records = list((tmp_path / "execution").glob("*_real_*.json"))
    assert len(records) == 1


# --------------------------------------------------------------------------------------
# dry mode: full offline+online gate run, never constructs an ActionClient
# --------------------------------------------------------------------------------------

class _FakeOnlineChecks:
    """Stands in for OnlineChecks so dry/mock-mode tests never need a live ROS graph."""

    def __init__(self, node, timeout_s):
        self.node = node

    def graph(self, profile, expected_node, action_name):
        return et._gate_dict(et.G_GRAPH, "online read-only", "pass", "fake: graph ok")

    def start_state(self, mode, profile, joint_states_topic, arm_status_topic, first_point, start_tol, timeout):
        return et._gate_dict(et.G_START_STATE, "online read-only", "pass", "fake: start state ok"), {
            j: q for j, q in zip(JOINT_NAMES, first_point)
        }

    def collision(self, configs, expected_object_ids):
        return et._gate_dict(et.G_COLLISION, "online read-only", "pass", "fake: collision ok")


def test_dry_mode_never_constructs_action_client(tmp_path, monkeypatch):
    monkeypatch.setattr(et, "OnlineChecks", _FakeOnlineChecks)

    def _boom(*a, **kw):
        raise AssertionError("dry mode must never construct an ActionClient")

    monkeypatch.setattr(et, "ActionClient", _boom)

    args = _args(tmp_path, mode="dry")
    result = et.run(args, env=_NO_ARM)
    assert result["status"] == "ok"

    records = list((tmp_path / "execution").glob("*_dry_*.json"))
    assert len(records) == 1
    record = json.loads(records[0].read_text())
    assert record["outcome"] == "rehearsed"
    assert record["mode"] == "dry"
    assert record["real_preview"], "dry must preview the real-only gates"
    assert any(c["gate"] == "G-CONFIRM" and c["outcome"] == "skip" for c in record["gate_report"])


def test_dry_is_the_default_mode(tmp_path, monkeypatch):
    monkeypatch.setattr(et, "OnlineChecks", _FakeOnlineChecks)
    monkeypatch.setattr(et, "ActionClient", lambda *a, **kw: (_ for _ in ()).throw(AssertionError()))

    args = _args(tmp_path, mode=None)  # omitted -> execution.yaml default_mode ("dry")
    result = et.run(args, env=_NO_ARM)
    assert result["status"] == "ok"
    records = list((tmp_path / "execution").glob("*_dry_*.json"))
    assert len(records) == 1


def test_online_gate_refusal_skips_confirm_and_execute(tmp_path, monkeypatch):
    class _FailingCollision(_FakeOnlineChecks):
        def collision(self, configs, expected_object_ids):
            return et._gate_dict(et.G_COLLISION, "online read-only", "fail", "fake: 1 waypoint invalid: indices [2]")

    monkeypatch.setattr(et, "OnlineChecks", _FailingCollision)
    monkeypatch.setattr(et, "ActionClient", lambda *a, **kw: (_ for _ in ()).throw(AssertionError()))

    args = _args(tmp_path, mode="mock")
    with pytest.raises(PreconditionError):
        et.run(args, env=_NO_ARM)

    records = list((tmp_path / "execution").glob("*_mock_*.json"))
    record = json.loads(records[0].read_text())
    assert record["outcome"] == "refused"
    assert any(c["gate"] == "G-CONFIRM" and c["outcome"] == "skip" for c in record["gate_report"])


# --------------------------------------------------------------------------------------
# mock mode: execute() is monkeypatched (no vendor/mock driver has to be running)
# --------------------------------------------------------------------------------------

def test_mock_mode_calls_execute_and_records_completion(tmp_path, monkeypatch):
    monkeypatch.setattr(et, "OnlineChecks", _FakeOnlineChecks)
    calls = []

    def _fake_execute(node, action_name, exec_cfg, scaled_traj, joint_states_topic):
        calls.append(action_name)
        return "completed", [{"t_s": 0.0, "commanded": [0.0] * 6, "actual": [0.0] * 6}]

    monkeypatch.setattr(et, "execute", _fake_execute)
    args = _args(tmp_path, mode="mock")
    result = et.run(args, env=_NO_ARM)
    assert result["status"] == "ok"
    assert calls == ["/rebotarm_controller/follow_joint_trajectory"]

    records = list((tmp_path / "execution").glob("*_mock_*.json"))
    record = json.loads(records[0].read_text())
    assert record["outcome"] == "completed"
    assert record["joint_trace"]


# --------------------------------------------------------------------------------------
# usage errors: mode/profile pair outside the table
# --------------------------------------------------------------------------------------

def test_explicit_bad_mode_profile_pair_is_a_usage_error_before_any_gate(tmp_path, monkeypatch):
    def _boom(*a, **kw):
        raise AssertionError("must not evaluate any gate for an explicit bad pair")

    monkeypatch.setattr(et, "evaluate_offline", _boom)
    args = _args(tmp_path, mode="real", profile="moveit_mock")
    with pytest.raises(ConfigError):
        et.run(args, env=_ARM_REAL)
    assert not (tmp_path / "execution").exists()


def test_default_profile_mismatch_with_mode_real_is_offline_refusal_not_usage_error(tmp_path, monkeypatch):
    # No --profile given: default_driver_profile (moveit_mock) is incompatible with --mode real,
    # but the offline gate refuses first (G-ARM unset), matching the real-refused-offline
    # acceptance check (profile pairing is only re-checked after an offline pass).
    monkeypatch.setattr(et.rclpy, "init", lambda *a, **kw: (_ for _ in ()).throw(AssertionError("no ROS")))
    args = _args(tmp_path, mode="real", profile=None)
    with pytest.raises(PreconditionError):
        et.run(args, env=_NO_ARM)


# --------------------------------------------------------------------------------------
# the §11.1 mode/profile table is hard-coded: a tampered --execution-config is a usage error
# --------------------------------------------------------------------------------------

SHIPPED_EXECUTION_CONFIG = REPO_ROOT / "config" / "motion" / "execution.yaml"


def _tampered_config(tmp_path, old: str, new: str) -> Path:
    text = SHIPPED_EXECUTION_CONFIG.read_text(encoding="utf-8")
    assert text.count(old) == 1, old
    path = tmp_path / "execution_tampered.yaml"
    path.write_text(text.replace(old, new), encoding="utf-8")
    load_execution_config(path)  # still schema-valid: only the §11.1 table check can catch it
    return path


def test_shipped_execution_config_matches_hard_coded_table():
    et._check_profiles_match_table(load_execution_config(SHIPPED_EXECUTION_CONFIG))


@pytest.mark.parametrize(
    "old,new",
    [
        ("    modes: [dry, real]", "    modes: [mock, dry, real]"),  # vendor allows mock
        ("    action: /rebotarm_controller/follow_joint_trajectory", "    action: /rebotarm/follow_joint_trajectory"),
        ("    expected_node: mock_rebotarm_driver", "    expected_node: reBotArmController"),
    ],
)
def test_tampered_profile_table_mock_vendor_is_config_error_and_never_executes(tmp_path, monkeypatch, old, new):
    def _boom(*a, **kw):
        raise AssertionError("a tampered §11.1 table must be refused before any gate or execute()")

    monkeypatch.setattr(et, "execute", _boom)
    monkeypatch.setattr(et, "evaluate_offline", _boom)
    monkeypatch.setattr(et.rclpy, "init", _boom)
    monkeypatch.setattr(et, "OnlineChecks", _FakeOnlineChecks)
    cfg = _tampered_config(tmp_path, old, new)
    args = _args(tmp_path, mode="mock", profile="vendor", execution_config=str(cfg))
    with pytest.raises(ConfigError):
        et.run(args, env=_NO_ARM)
    assert not (tmp_path / "execution").exists()


def test_mock_vendor_pair_refused_even_if_config_lists_it(tmp_path, monkeypatch):
    # Even without the table check, the pair itself is judged from PROFILE_TABLE, not the config.
    with pytest.raises(ConfigError):
        et._validate_mode_profile("mock", "vendor")
    with pytest.raises(ConfigError):
        et._validate_mode_profile("real", "vendor_mock")
    et._validate_mode_profile("real", "vendor")


# --------------------------------------------------------------------------------------
# --dry-run: offline gate only, writes nothing, exits 0 regardless of outcome
# --------------------------------------------------------------------------------------

def test_offline_dry_run_writes_nothing_and_exits_0(tmp_path, capsys):
    args = _args(tmp_path, mode="real", dry_run=True)
    assert not (tmp_path / "execution").exists()
    rc = et._offline_dry_run(args)
    assert rc == et.EXIT_OK
    assert not (tmp_path / "execution").exists()
    assert not list(tmp_path.glob("**/*.log"))
    out = json.loads(capsys.readouterr().out)
    assert out["mode"] == "real"
    assert out["passed"] is False  # G-ARM unset, etc.


def test_offline_dry_run_defaults_mode_to_execution_config_default(tmp_path, capsys):
    args = _args(tmp_path, mode=None, dry_run=True)
    rc = et._offline_dry_run(args)
    assert rc == et.EXIT_OK
    out = json.loads(capsys.readouterr().out)
    assert out["mode"] == "dry"


# --------------------------------------------------------------------------------------
# confirmation (§11.8): only /dev/tty, exact phrase match
# --------------------------------------------------------------------------------------

class _FakeTty:
    def __init__(self, line: str):
        self._line = line
        self.closed = False

    def readline(self):
        return self._line

    def close(self):
        self.closed = True


def test_confirm_real_accepts_exact_phrase():
    sha = "ab" * 32
    phrase = et.confirmation_phrase(sha)
    tty = _FakeTty(phrase + "\n")
    ok, detail = et.confirm_real(["line1"], sha, tty_opener=lambda: tty)
    assert ok is True
    assert tty.closed is True


def test_confirm_real_rejects_wrong_phrase():
    sha = "cd" * 32
    tty = _FakeTty("EXECUTE wrongwrong\n")
    ok, detail = et.confirm_real(["line1"], sha, tty_opener=lambda: tty)
    assert ok is False
    assert "did not match" in detail


def test_confirm_real_refuses_when_no_controlling_tty():
    sha = "ef" * 32

    def _raise():
        raise OSError("no such device")

    ok, detail = et.confirm_real(["line1"], sha, tty_opener=_raise)
    assert ok is False
    assert "no controlling" in detail.lower()


# --------------------------------------------------------------------------------------
# pure gate-decision functions (G-GRAPH / G-START-STATE / G-COLLISION)
# --------------------------------------------------------------------------------------

def test_check_graph_mock_refuses_if_real_driver_visible():
    result = et.check_graph("moveit_mock", "ros2_control mock hardware", {"reBotArmController"}, None, [])
    assert result["outcome"] == "fail"


def test_check_graph_mock_passes_when_real_driver_absent():
    result = et.check_graph("vendor_mock", "mock_rebotarm_driver", {"mock_rebotarm_driver"}, None, [])
    assert result["outcome"] == "pass"


def test_check_graph_vendor_refuses_if_mock_driver_visible():
    result = et.check_graph(
        "vendor", "reBotArmController", {"reBotArmController", "mock_rebotarm_driver"}, "reBotArmController", []
    )
    assert result["outcome"] == "fail"
    assert "mock driver" in result["detail"]


def test_check_graph_vendor_refuses_if_action_server_host_missing():
    result = et.check_graph("vendor", "reBotArmController", {"reBotArmController"}, None, [])
    assert result["outcome"] == "fail"


def test_check_graph_vendor_refuses_on_passthrough_publisher():
    result = et.check_graph(
        "vendor", "reBotArmController", {"reBotArmController"}, "reBotArmController",
        ["/rebotarm/joints/joint1/cmd/vel"],
    )
    assert result["outcome"] == "fail"


def test_check_graph_vendor_passes_clean():
    result = et.check_graph("vendor", "reBotArmController", {"reBotArmController"}, "reBotArmController", [])
    assert result["outcome"] == "pass"


def test_check_start_state_fails_with_no_joint_states():
    result = et.check_start_state("mock", "moveit_mock", None, None, [0.0] * 6, 0.02, 0.5, 5.0, None)
    assert result["outcome"] == "fail"
    assert "5.0" in result["detail"]


def test_check_start_state_fails_when_stale():
    positions = {j: 0.0 for j in JOINT_NAMES}
    result = et.check_start_state("mock", "moveit_mock", positions, 5.0, [0.0] * 6, 0.02, 0.5, 5.0, None)
    assert result["outcome"] == "fail"
    assert "stale" in result["detail"]


def test_check_start_state_fails_outside_tolerance():
    positions = {j: 0.0 for j in JOINT_NAMES}
    first_point = [1.0] + [0.0] * 5
    result = et.check_start_state("mock", "moveit_mock", positions, 0.1, first_point, 0.02, 0.5, 5.0, None)
    assert result["outcome"] == "fail"


def test_check_start_state_passes_fresh_and_within_tolerance():
    positions = {j: 0.0 for j in JOINT_NAMES}
    result = et.check_start_state("mock", "moveit_mock", positions, 0.1, [0.0] * 6, 0.02, 0.5, 5.0, None)
    assert result["outcome"] == "pass"


def test_check_start_state_vendor_arm_status_warn_in_dry():
    positions = {j: 0.0 for j in JOINT_NAMES}
    arm_status = {"enabled": False, "state_machine": "IDLE", "error_codes": []}
    result = et.check_start_state("dry", "vendor", positions, 0.1, [0.0] * 6, 0.02, 0.5, 5.0, arm_status)
    assert result["outcome"] == "warn"


def test_check_start_state_vendor_arm_status_fails_in_real():
    positions = {j: 0.0 for j in JOINT_NAMES}
    arm_status = {"enabled": False, "state_machine": "IDLE", "error_codes": []}
    result = et.check_start_state("real", "vendor", positions, 0.1, [0.0] * 6, 0.02, 0.5, 5.0, arm_status)
    assert result["outcome"] == "fail"


# --------------------------------------------------------------------------------------
# joint_states QoS (MOT-05.7): BEST_EFFORT sensor-data for the non-latched subscriptions,
# unchanged RELIABLE+TRANSIENT_LOCAL for the latched arm_status wait.
# --------------------------------------------------------------------------------------

def test_wait_for_message_joint_states_uses_best_effort_sensor_qos(monkeypatch):
    node = _FakeNode()
    monkeypatch.setattr(et.rclpy, "spin_once", lambda n, timeout_sec=0.0: None)
    et._wait_for_message(node, "/rebotarm/joint_states", et.JointState, 0.01)
    assert len(node.created) == 1
    qos = node.created[0].qos
    assert qos.reliability == et.QoSReliabilityPolicy.BEST_EFFORT


def test_wait_for_message_latched_arm_status_stays_reliable_transient_local(monkeypatch):
    node = _FakeNode()
    monkeypatch.setattr(et.rclpy, "spin_once", lambda n, timeout_sec=0.0: None)
    et._wait_for_message(node, "/rebotarm/arm_status", et.ArmStatus, 0.01, latched=True)
    assert len(node.created) == 1
    qos = node.created[0].qos
    assert qos.reliability == et.QoSReliabilityPolicy.RELIABLE
    assert qos.durability == et.QoSDurabilityPolicy.TRANSIENT_LOCAL


def test_execute_js_sub_uses_best_effort_sensor_qos(monkeypatch):
    goal_handle = _FakeGoalHandle(_FakeFuture(result=RESULT_SUCCESSFUL, done=True))
    node = _FakeNode()

    def _fake_spin_once(n, timeout_sec=0.0):
        n.publish(JS_TOPIC, _js([0.0] * 6))

    monkeypatch.setattr(et, "ActionClient", _fake_action_client(_FakeFuture(result=goal_handle, done=True)))
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)
    et.execute(node, ACTION, EXEC_CFG_FAST, SHORT_TRAJ, JS_TOPIC)

    js_subs = [s for s in node.created if s.topic == JS_TOPIC]
    assert js_subs
    assert js_subs[0].qos.reliability == et.QoSReliabilityPolicy.BEST_EFFORT


# --------------------------------------------------------------------------------------
# G-START-STATE discovery bound vs. staleness (MOT-05.7)
# --------------------------------------------------------------------------------------

class _TimedFakeNode:
    """A joint_states subscription that only delivers its first message after `delay_s` of
    simulated spin_once calls, with a header stamp `stamp_age_s` old at delivery time -- proves
    the discovery wait (how long the first message took to arrive) is independent of
    `joint_state_timeout_s` (how old that message's own stamp is)."""

    def __init__(self, delay_s: float, stamp_age_s: float):
        self.delay_s = delay_s
        self.stamp_age_s = stamp_age_s
        self.subs: List[Any] = []

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = SimpleNamespace(topic=topic, callback=callback, qos=qos)
        self.subs.append(sub)
        return sub

    def destroy_subscription(self, sub):
        if sub in self.subs:
            self.subs.remove(sub)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(100.0 * 1e9)))

    def maybe_deliver(self, elapsed_s: float) -> None:
        if elapsed_s < self.delay_s:
            return
        stamp_s = 100.0 - self.stamp_age_s
        sec = int(stamp_s)
        nanosec = int(round((stamp_s - sec) * 1e9))
        msg = SimpleNamespace(
            name=list(JOINT_NAMES), position=[0.0] * 6,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        )
        for sub in list(self.subs):
            sub.callback(msg)


def _drive_timed_node(monkeypatch, node: "_TimedFakeNode"):
    fake_clock = {"t": 0.0}

    def _fake_monotonic():
        return fake_clock["t"]

    def _fake_spin_once(n, timeout_sec=0.0):
        fake_clock["t"] += 0.1
        node.maybe_deliver(fake_clock["t"])

    monkeypatch.setattr(et.time, "monotonic", _fake_monotonic)
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)


def test_start_state_passes_when_first_message_arrives_within_discovery_bound(monkeypatch):
    # Discovery takes a simulated 1.5s -- longer than the shipped joint_state_s=0.5 -- but the
    # message itself is fresh, so G-START-STATE must still pass.
    node = _TimedFakeNode(delay_s=1.5, stamp_age_s=0.01)
    _drive_timed_node(monkeypatch, node)
    fake_self = SimpleNamespace(node=node, timeout_s=5.0)
    result, positions = et.OnlineChecks.start_state(
        fake_self, "mock", "moveit_mock", "/rebotarm/joint_states", None, [0.0] * 6, 0.02, 0.5,
    )
    assert result["outcome"] == "pass", result
    assert positions is not None


def test_start_state_refuses_naming_discovery_bound_when_no_message_arrives(monkeypatch):
    node = _TimedFakeNode(delay_s=999.0, stamp_age_s=0.0)
    _drive_timed_node(monkeypatch, node)
    fake_self = SimpleNamespace(node=node, timeout_s=0.5)
    result, positions = et.OnlineChecks.start_state(
        fake_self, "mock", "moveit_mock", "/rebotarm/joint_states", None, [0.0] * 6, 0.02, 0.5,
    )
    assert result["outcome"] == "fail"
    assert "0.5" in result["detail"]
    assert positions is None


def test_start_state_vendor_arm_status_wait_uses_discovery_bound_not_joint_state_timeout(monkeypatch):
    calls = []

    def _fake_wait_for_message(node, topic, msg_type, timeout_s, latched=False):
        calls.append((topic, timeout_s, latched))
        if topic == "/rebotarm/joint_states":
            return SimpleNamespace(
                name=list(JOINT_NAMES), position=[0.0] * 6,
                header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
            )
        return None

    monkeypatch.setattr(et, "_wait_for_message", _fake_wait_for_message)
    fake_node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)))
    fake_self = SimpleNamespace(node=fake_node, timeout_s=7.0)
    et.OnlineChecks.start_state(
        fake_self, "dry", "vendor", "/rebotarm/joint_states", "/rebotarm/arm_status", [0.0] * 6, 0.02, 0.05,
    )
    arm_status_calls = [c for c in calls if c[0] == "/rebotarm/arm_status"]
    assert len(arm_status_calls) == 1
    _topic, timeout_s, latched = arm_status_calls[0]
    assert timeout_s == 7.0
    assert latched is True


def test_check_collision_fails_on_missing_scene_object():
    result = et.check_collision([], ["table"])
    assert result["outcome"] == "fail"
    assert "table" in result["detail"]


def test_check_collision_fails_on_invalid_waypoint():
    result = et.check_collision([2, 5], [])
    assert result["outcome"] == "fail"
    assert "2" in result["detail"]


def test_check_collision_passes():
    result = et.check_collision([], [])
    assert result["outcome"] == "pass"


# --------------------------------------------------------------------------------------
# monitoring pure helpers
# --------------------------------------------------------------------------------------

def test_tracking_error_rad():
    assert et.tracking_error_rad([0.0, 1.0], [0.0, 1.2]) == pytest.approx(0.2)


def test_is_stale():
    assert et.is_stale(None, 0.5) is True
    assert et.is_stale(0.6, 0.5) is True
    assert et.is_stale(0.1, 0.5) is False


def test_interpolate_positions_midpoint():
    traj = {"points": [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 2.0, "positions": [2.0] * 6}]}
    assert et.interpolate_positions(traj, 1.0) == pytest.approx([1.0] * 6)


def test_interpolate_positions_before_start_and_after_end():
    traj = {"points": [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 2.0, "positions": [2.0] * 6}]}
    assert et.interpolate_positions(traj, -1.0) == pytest.approx([0.0] * 6)
    assert et.interpolate_positions(traj, 10.0) == pytest.approx([2.0] * 6)


# --------------------------------------------------------------------------------------
# JointTrajectory conversion
# --------------------------------------------------------------------------------------

def test_joint_trajectory_msg_holds_scaled_points_and_time_from_start():
    scaled = {
        "points": [
            {"t_s": 0.0, "positions": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]},
            {"t_s": 1.5, "positions": [0.2, 0.3, 0.4, 0.5, 0.6, 0.7], "velocities": [0.1] * 6},
        ]
    }
    msg = et.joint_trajectory_msg(scaled)
    assert list(msg.joint_names) == list(JOINT_NAMES)
    assert len(msg.points) == 2
    assert list(msg.points[0].positions) == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert msg.points[0].time_from_start.sec == 0
    assert msg.points[0].time_from_start.nanosec == 0
    assert msg.points[1].time_from_start.sec == 1
    assert msg.points[1].time_from_start.nanosec == pytest.approx(500_000_000, abs=1)
    assert list(msg.points[1].velocities) == pytest.approx([0.1] * 6)


# --------------------------------------------------------------------------------------
# execute(): the single ActionClient site -- trips cancel-and-hold, bounded waits, arrival check
# --------------------------------------------------------------------------------------

CANCEL_OK = SimpleNamespace(return_code=0)  # action_msgs/CancelGoal ERROR_NONE
CANCEL_REJECTED = SimpleNamespace(return_code=1)  # ERROR_REJECTED
RESULT_SUCCESSFUL = SimpleNamespace(result=SimpleNamespace(error_code=0))


class _FakeFuture:
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._result

    def complete(self, result):
        self._result = result
        self._done = True


class _FakeGoalHandle:
    def __init__(self, result_future, cancel_future=None):
        self.accepted = True
        self._result_future = result_future
        self._cancel_future = cancel_future if cancel_future is not None else _FakeFuture(CANCEL_OK)
        self.cancel_calls = 0

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self._cancel_future


class _FakeSub:
    def __init__(self, topic, callback, qos=None):
        self.topic = topic
        self.callback = callback
        self.qos = qos


class _FakeNode:
    def __init__(self):
        self.subs: List[_FakeSub] = []
        self.created: List[_FakeSub] = []  # never pruned by destroy_subscription, for QoS assertions

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = _FakeSub(topic, callback, qos)
        self.subs.append(sub)
        self.created.append(sub)
        return sub

    def destroy_subscription(self, sub):
        if sub in self.subs:
            self.subs.remove(sub)

    def publish(self, topic, msg):
        for sub in list(self.subs):
            if sub.topic == topic:
                sub.callback(msg)


class _Bool:
    def __init__(self, data: bool):
        self.data = data


def _js(positions):
    return SimpleNamespace(name=list(JOINT_NAMES), position=[float(v) for v in positions])


def _fake_action_client(send_future):
    class _FakeActionClient:
        def __init__(self, node, action_type, action_name):
            self.node = node

        def wait_for_server(self, timeout_sec=0.0):
            return True

        def send_goal_async(self, goal_msg):
            return send_future

    return _FakeActionClient


EXEC_CFG_FAST = {
    "estop_topics": ["/crackvision/estop", "/rebot_motion/estop"],
    "tolerances": {"tracking_rad": 0.15, "start_state_rad": 0.02},
    "timeouts": {"joint_state_s": 0.05, "goal_s": 5.0, "cancel_settle_s": 0.5},
}
JS_TOPIC = "/rebotarm/joint_states"
ACTION = "/rebotarm/follow_joint_trajectory"
SHORT_TRAJ = {"points": [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]}


def test_execute_cancels_on_estop_and_returns_estopped(monkeypatch):
    goal_handle = _FakeGoalHandle(_FakeFuture(done=False))
    node = _FakeNode()
    counter = {"n": 0}

    def _fake_spin_once(n, timeout_sec=0.0):
        counter["n"] += 1
        time.sleep(0.002)
        n.publish(JS_TOPIC, _js([0.0] * 6))
        if counter["n"] == 3:
            n.publish("/crackvision/estop", _Bool(True))

    monkeypatch.setattr(et, "ActionClient", _fake_action_client(_FakeFuture(result=goal_handle, done=True)))
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)

    outcome, trace = et.execute(node, ACTION, EXEC_CFG_FAST, SHORT_TRAJ, JS_TOPIC)

    assert outcome == "estopped"
    assert goal_handle.cancel_calls == 1
    assert node.subs == []  # js_sub + estop subs all destroyed on the way out


def test_execute_trip_while_send_pending_then_accepted_cancels_exactly_once(monkeypatch, capsys):
    goal_handle = _FakeGoalHandle(_FakeFuture(done=False))
    send_future = _FakeFuture(done=False)
    node = _FakeNode()
    counter = {"n": 0}

    def _fake_spin_once(n, timeout_sec=0.0):
        counter["n"] += 1
        time.sleep(0.002)
        n.publish(JS_TOPIC, _js([0.0] * 6))
        if counter["n"] == 3:
            n.publish("/crackvision/estop", _Bool(True))  # trip before the goal response
        if counter["n"] == 10:
            send_future.complete(goal_handle)  # ...then the driver accepts the goal

    monkeypatch.setattr(et, "ActionClient", _fake_action_client(send_future))
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)

    outcome, _trace = et.execute(node, ACTION, EXEC_CFG_FAST, SHORT_TRAJ, JS_TOPIC)

    assert counter["n"] >= 10, "execute() must keep waiting for the pending goal response after a trip"
    assert outcome == "estopped"
    assert goal_handle.cancel_calls == 1
    assert "PRESS THE HARDWARE E-STOP" not in capsys.readouterr().out  # acked cancel, arm settled


def test_execute_trip_while_send_pending_and_no_response_prints_estop(monkeypatch, capsys):
    send_future = _FakeFuture(done=False)  # never answered
    node = _FakeNode()
    counter = {"n": 0}

    def _fake_spin_once(n, timeout_sec=0.0):
        counter["n"] += 1
        time.sleep(0.002)
        if counter["n"] == 2:
            n.publish("/crackvision/estop", _Bool(True))

    monkeypatch.setattr(et, "ActionClient", _fake_action_client(send_future))
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)

    t0 = time.monotonic()
    outcome, _trace = et.execute(node, ACTION, EXEC_CFG_FAST, SHORT_TRAJ, JS_TOPIC)
    assert time.monotonic() - t0 < EXEC_CFG_FAST["timeouts"]["cancel_settle_s"] + 0.5
    assert outcome == "estopped"
    assert "PRESS THE HARDWARE E-STOP" in capsys.readouterr().out


@pytest.mark.parametrize("cancel_future", [_FakeFuture(done=False), _FakeFuture(CANCEL_REJECTED)], ids=["never_done", "rejected"])
def test_execute_unacknowledged_cancel_is_bounded_and_prints_estop(monkeypatch, capsys, cancel_future):
    goal_handle = _FakeGoalHandle(_FakeFuture(done=False), cancel_future=cancel_future)
    node = _FakeNode()
    counter = {"n": 0}

    def _fake_spin_once(n, timeout_sec=0.0):
        counter["n"] += 1
        time.sleep(0.002)
        n.publish(JS_TOPIC, _js([0.0] * 6))
        if counter["n"] == 3:
            n.publish("/crackvision/estop", _Bool(True))

    monkeypatch.setattr(et, "ActionClient", _fake_action_client(_FakeFuture(result=goal_handle, done=True)))
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)

    cancel_settle_s = EXEC_CFG_FAST["timeouts"]["cancel_settle_s"]
    t0 = time.monotonic()
    outcome, _trace = et.execute(node, ACTION, EXEC_CFG_FAST, SHORT_TRAJ, JS_TOPIC)
    elapsed = time.monotonic() - t0

    assert elapsed < cancel_settle_s + 0.3, f"execute() blocked {elapsed:.2f}s on an unacknowledged cancel"
    assert outcome == "estopped"
    assert goal_handle.cancel_calls == 1
    assert "PRESS THE HARDWARE E-STOP" in capsys.readouterr().out


def test_execute_acked_cancel_but_arm_keeps_moving_prints_estop(monkeypatch, capsys):
    goal_handle = _FakeGoalHandle(_FakeFuture(done=False))
    node = _FakeNode()
    counter = {"n": 0}

    def _fake_spin_once(n, timeout_sec=0.0):
        counter["n"] += 1
        time.sleep(0.002)
        # tracks SHORT_TRAJ closely before the trip, keeps drifting 0.01 rad/spin after it
        drift = 0.0 if counter["n"] <= 3 else 0.01 * (counter["n"] - 3)
        n.publish(JS_TOPIC, _js([drift] * 6))
        if counter["n"] == 3:
            n.publish("/crackvision/estop", _Bool(True))

    monkeypatch.setattr(et, "ActionClient", _fake_action_client(_FakeFuture(result=goal_handle, done=True)))
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)

    t0 = time.monotonic()
    outcome, _trace = et.execute(node, ACTION, EXEC_CFG_FAST, SHORT_TRAJ, JS_TOPIC)
    assert time.monotonic() - t0 < EXEC_CFG_FAST["timeouts"]["cancel_settle_s"] + 0.3
    assert outcome == "estopped"
    assert goal_handle.cancel_calls == 1
    assert "PRESS THE HARDWARE E-STOP" in capsys.readouterr().out


class _FakeClock:
    def __init__(self, t0: float = 100.0):
        self.t = t0

    def __call__(self) -> float:
        return self.t


def _run_tracked_goal(monkeypatch, scaled_traj, exec_cfg, tail: str):
    """Drive execute() on a fake clock: joint_states track `scaled_traj` perfectly; the driver
    reports SUCCESSFUL 0.2 s after the last t_s. `tail` sets what joint_states do afterwards:
    "arrived" (at the final point), "short" (0.1 rad short of it) or "silent" (stop publishing)."""
    clock = _FakeClock()
    t_start = clock.t  # send_future is done immediately, so the goal starts at t0
    duration = scaled_traj["points"][-1]["t_s"]
    result_future = _FakeFuture(done=False)
    goal_handle = _FakeGoalHandle(result_future)
    node = _FakeNode()

    def _fake_spin_once(n, timeout_sec=0.0):
        clock.t += 0.05
        rel = clock.t - t_start
        if rel >= duration + 0.2 and not result_future.done():
            result_future.complete(RESULT_SUCCESSFUL)
            if tail == "silent":
                return
        if result_future.done() and tail == "silent":
            return
        positions = et.interpolate_positions(scaled_traj, rel)
        if tail == "short" and rel >= duration:
            positions = [q - 0.1 for q in positions]
        n.publish(JS_TOPIC, _js(positions))

    monkeypatch.setattr(et, "_now", clock)
    monkeypatch.setattr(et, "ActionClient", _fake_action_client(_FakeFuture(result=goal_handle, done=True)))
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)
    outcome, trace = et.execute(node, ACTION, exec_cfg, scaled_traj, JS_TOPIC)
    return outcome, trace, goal_handle, clock.t - t_start


def _shipped_cfg():
    cfg = load_execution_config(SHIPPED_EXECUTION_CONFIG)
    assert cfg["timeouts"]["goal_s"] == 5.0
    return cfg


def _scaled_smoke(scale: float):
    return et.scale_time(et.load_trajectory(TRAJECTORY_SMOKE), scale)


def test_execute_goal_longer_than_goal_s_completes_when_tracking(monkeypatch):
    cfg = _shipped_cfg()
    scaled = _scaled_smoke(cfg["speed_scale"]["real_default"])
    assert scaled["points"][-1]["t_s"] > cfg["timeouts"]["goal_s"]
    outcome, trace, goal_handle, sim_elapsed = _run_tracked_goal(monkeypatch, scaled, cfg, tail="arrived")
    assert outcome == "completed"
    assert goal_handle.cancel_calls == 0
    assert sim_elapsed > cfg["timeouts"]["goal_s"]
    assert trace


def test_execute_stuck_goal_cancelled_after_scaled_duration_plus_goal_s(monkeypatch):
    cfg = _shipped_cfg()
    scaled = {"points": [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 6.0, "positions": [0.0] * 6}]}
    clock = _FakeClock()
    goal_handle = _FakeGoalHandle(_FakeFuture(done=False))  # never finishes
    node = _FakeNode()

    def _fake_spin_once(n, timeout_sec=0.0):
        clock.t += 0.05
        n.publish(JS_TOPIC, _js([0.0] * 6))

    monkeypatch.setattr(et, "_now", clock)
    monkeypatch.setattr(et, "ActionClient", _fake_action_client(_FakeFuture(result=goal_handle, done=True)))
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)
    outcome, _trace = et.execute(node, ACTION, cfg, scaled, JS_TOPIC)
    assert outcome == "aborted"
    assert goal_handle.cancel_calls == 1
    # cancelled just past 6.0 + 5.0 s, plus at most cancel_settle_s for cancel-and-hold
    assert 11.0 < clock.t - 100.0 <= 11.0 + cfg["timeouts"]["cancel_settle_s"] + 0.2


def test_execute_successful_but_not_arrived_is_aborted(monkeypatch):
    cfg = _shipped_cfg()
    outcome, _trace, _gh, _ = _run_tracked_goal(monkeypatch, _scaled_smoke(1.0), cfg, tail="short")
    assert outcome == "aborted"


def test_execute_successful_with_stale_joint_states_is_aborted(monkeypatch):
    cfg = _shipped_cfg()
    outcome, _trace, _gh, _ = _run_tracked_goal(monkeypatch, _scaled_smoke(1.0), cfg, tail="silent")
    assert outcome == "aborted"


def test_execute_successful_and_arrived_is_completed(monkeypatch):
    cfg = _shipped_cfg()
    outcome, trace, _gh, _ = _run_tracked_goal(monkeypatch, _scaled_smoke(1.0), cfg, tail="arrived")
    assert outcome == "completed"
    assert trace[-1]["commanded"] == pytest.approx(trace[-1]["actual"], abs=et.ARRIVAL_TOL_RAD)
