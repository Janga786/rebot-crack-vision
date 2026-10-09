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
from pathlib import Path
from typing import Dict, List

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import crackvision_motion.execute_trajectory as et  # noqa: E402
from crackvision_motion.cli_common import PreconditionError  # noqa: E402
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
    result = et.check_start_state("mock", "moveit_mock", None, None, [0.0] * 6, 0.02, 0.5, None)
    assert result["outcome"] == "fail"


def test_check_start_state_fails_when_stale():
    positions = {j: 0.0 for j in JOINT_NAMES}
    result = et.check_start_state("mock", "moveit_mock", positions, 5.0, [0.0] * 6, 0.02, 0.5, None)
    assert result["outcome"] == "fail"


def test_check_start_state_fails_outside_tolerance():
    positions = {j: 0.0 for j in JOINT_NAMES}
    first_point = [1.0] + [0.0] * 5
    result = et.check_start_state("mock", "moveit_mock", positions, 0.1, first_point, 0.02, 0.5, None)
    assert result["outcome"] == "fail"


def test_check_start_state_passes_fresh_and_within_tolerance():
    positions = {j: 0.0 for j in JOINT_NAMES}
    result = et.check_start_state("mock", "moveit_mock", positions, 0.1, [0.0] * 6, 0.02, 0.5, None)
    assert result["outcome"] == "pass"


def test_check_start_state_vendor_arm_status_warn_in_dry():
    positions = {j: 0.0 for j in JOINT_NAMES}
    arm_status = {"enabled": False, "state_machine": "IDLE", "error_codes": []}
    result = et.check_start_state("dry", "vendor", positions, 0.1, [0.0] * 6, 0.02, 0.5, arm_status)
    assert result["outcome"] == "warn"


def test_check_start_state_vendor_arm_status_fails_in_real():
    positions = {j: 0.0 for j in JOINT_NAMES}
    arm_status = {"enabled": False, "state_machine": "IDLE", "error_codes": []}
    result = et.check_start_state("real", "vendor", positions, 0.1, [0.0] * 6, 0.02, 0.5, arm_status)
    assert result["outcome"] == "fail"


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
# execute(): the single ActionClient site -- an e-stop trips cancel-and-hold
# --------------------------------------------------------------------------------------

class _FakeFuture:
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._result


class _FakeGoalHandle:
    def __init__(self, result_future):
        self.accepted = True
        self._result_future = result_future
        self.cancel_calls = 0

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _FakeFuture(result=None, done=True)


class _FakeSub:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback


class _FakeNode:
    def __init__(self):
        self.subs: List[_FakeSub] = []

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = _FakeSub(topic, callback)
        self.subs.append(sub)
        return sub

    def destroy_subscription(self, sub):
        if sub in self.subs:
            self.subs.remove(sub)


class _Bool:
    def __init__(self, data: bool):
        self.data = data


def test_execute_cancels_on_estop_and_returns_estopped(monkeypatch):
    result_future = _FakeFuture(done=False)
    goal_handle = _FakeGoalHandle(result_future)

    class _FakeActionClient:
        def __init__(self, node, action_type, action_name):
            self.node = node

        def wait_for_server(self, timeout_sec=0.0):
            return True

        def send_goal_async(self, goal_msg):
            return _FakeFuture(result=goal_handle, done=True)

    node = _FakeNode()
    counter = {"n": 0}

    def _fake_spin_once(n, timeout_sec=0.0):
        counter["n"] += 1
        if counter["n"] == 3:
            for sub in n.subs:
                if sub.topic == "/crackvision/estop":
                    sub.callback(_Bool(True))

    monkeypatch.setattr(et, "ActionClient", _FakeActionClient)
    monkeypatch.setattr(et.rclpy, "spin_once", _fake_spin_once)

    exec_cfg = {
        "estop_topics": ["/crackvision/estop", "/rebot_motion/estop"],
        "tolerances": {"tracking_rad": 0.15, "start_state_rad": 0.02},
        "timeouts": {"joint_state_s": 0.5, "goal_s": 30.0, "cancel_settle_s": 0.05},
    }
    scaled_traj = {"points": [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]}

    outcome, trace = et.execute(node, "/rebotarm/follow_joint_trajectory", exec_cfg, scaled_traj, "/rebotarm/joint_states")

    assert outcome == "estopped"
    assert goal_handle.cancel_calls == 1
    assert node.subs == []  # js_sub + estop subs all destroyed on the way out
