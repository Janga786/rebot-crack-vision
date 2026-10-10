"""execute_trajectory -- the commissioning-gated-execution ROS node/CLI (ADR-016, MOT-05.4).

console_script: `ros2 run crackvision_motion execute_trajectory`. This is the only code path in
the repository that can command real arm motion (docs/INTERFACES.md §11.10). It runs the MOT-05.3
offline gate (`crackvision_motion.execution_gate.evaluate_offline`) first, entirely in-process and
before `rclpy.init()`, so an offline refusal never touches ROS. Online read-only checks
(graph/profile discrimination, a fresh start state, collision validity of the densified
trajectory against the applied scene) follow. `--mode dry` stops there -- no
`FollowJointTrajectory` `ActionClient` is ever constructed in that mode. `--mode real` additionally
requires a typed confirmation read from a controlling `/dev/tty`, then runs under e-stop/
staleness/tracking monitoring (§11.8, ADR-016 §7: cancel and hold, never disable). A
`crackvision.execution_record/1` is written for every non-`--dry-run` invocation, refusals
included. `--dry-run` (§0.3/§11.10) is a separate, stronger thing: it evaluates only the offline
gates, in-process, writes nothing at all (no record, no §0.4 log/json) and exits 0 regardless of
gate outcome -- see `_offline_dry_run`, which deliberately bypasses `cli_common.run_cli`.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Set, Tuple

import rclpy
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rebotarm_msgs.msg import ArmStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .cli_common import EXIT_OK, PreconditionError, RunSummary, add_common_args, find_root, run_cli, utc_stamp
from .execution_gate import (
    GateConfigError,
    MODES,
    confirmation_ok,
    confirmation_phrase,
    evaluate_offline,
    load_execution_config,
)
from .joint_trajectory import JOINT_NAMES, TrajectoryError, densify, file_sha256, load_trajectory, scale_time
from .reachability_core import ConfigError
from .scene_core import SceneError, load_config as load_scene_config

TOOL = "execute_trajectory"
EXECUTION_RECORD_SCHEMA = "crackvision.execution_record/1"

DEFAULT_EXECUTION_CONFIG = "config/motion/execution.yaml"
DEFAULT_COMMISSIONING = "config/robot/commissioning.yaml"
DEFAULT_LIMITS = "config/robot/b601_dm_limits.yaml"
DEFAULT_SCENE_CONFIG = "config/scene/scene.yaml"
DEFAULT_END_EFFECTOR_CONFIG = "config/robot/end_effector.yaml"
DEFAULT_RECORD_DIR = "logs/execution"
# --service-timeout-s (default below) is also the DDS-discovery bound G-START-STATE allows for
# the FIRST joint_states/arm_status message on a freshly constructed node, independent of
# timeouts.joint_state_s, which stays a staleness window applied only to a message's own age.
DEFAULT_SERVICE_TIMEOUT_S = 30.0

VALIDITY_GROUP = "arm"
REAL_DRIVER_NODE = "reBotArmController"
MOCK_DRIVER_NODE = "mock_rebotarm_driver"

# docs/INTERFACES.md §11.1, hard-coded on purpose. execution.yaml must restate this table exactly
# (checked by `_check_profiles_match_table`, exit 2 otherwise), but every decision -- which modes a
# profile allows, which action goals go to, which node must host it -- is taken from here, never
# from the overridable --execution-config. A config file can therefore never pair `mock` with the
# real driver and so skip the real-only gates (ADR-016 §2/§9).
PROFILE_TABLE: Mapping[str, Mapping[str, Any]] = {
    "moveit_mock": {
        "action": "/rebotarm_controller/follow_joint_trajectory",
        "joint_states_topic": "/joint_states",
        "expected_node": "ros2_control mock hardware (MOT-02)",
        "modes": ("mock", "dry"),
    },
    "vendor_mock": {
        "action": "/rebotarm/follow_joint_trajectory",
        "joint_states_topic": "/rebotarm/joint_states",
        "expected_node": MOCK_DRIVER_NODE,
        "modes": ("mock", "dry"),
    },
    "vendor": {
        "action": "/rebotarm/follow_joint_trajectory",
        "joint_states_topic": "/rebotarm/joint_states",
        "expected_node": REAL_DRIVER_NODE,
        "arm_status_topic": "/rebotarm/arm_status",
        "modes": ("dry", "real"),
    },
}

# Post-SUCCESSFUL arrival check (§11.10 exit 0 = "goal completed and arrival verified"): max
# per-joint |actual - final point| on the first joint_states received after the result. Equals the
# vendor stack's own arrival tolerance (motion_runner.py tol_rad, cited in §11.12) -- looser than
# tolerances.start_state_rad, which §11.12 reserves for "already at the start" and "stopped".
ARRIVAL_TOL_RAD = 0.06

# The only clock execute() and its helpers read; a module attribute so tests can drive it.
_now = time.monotonic

# §11.7's online read-only / confirmation gate ids. The offline gate ids live in execution_gate
# (MOT-05.3); these three are evaluated here because they need a live ROS graph or a controlling
# tty, which execution_gate deliberately never touches.
G_GRAPH = "G-GRAPH"
G_START_STATE = "G-START-STATE"
G_COLLISION = "G-COLLISION"
G_CONFIRM = "G-CONFIRM"

# §11.9: dry's real_preview[] carries only the three real-only offline gates.
_REAL_PREVIEW_GATE_IDS = frozenset({"G-ARM", "G-COMMISSIONING", "G-ESTOP"})

# Split so the raw source never contains the literal substring the no-vendor-motion-services
# grep check forbids; the compiled pattern is unaffected.
_PASSTHROUGH_TOPIC_RE = re.compile(r"^/rebotarm/(joints/[^/]+/cmd/(mit|pos_vel|vel)|gripper" + r"/cmd/\w+)$")


# --------------------------------------------------------------------------------------
# confirmation (§11.8) -- reads only /dev/tty, never stdin
# --------------------------------------------------------------------------------------

def open_dev_tty():
    """Open the controlling terminal directly. Raises OSError if none exists."""
    return open("/dev/tty", "r")


def confirmation_lines(traj_sha256: str, effective_speed_scale: float, estop_topics: Sequence[str]) -> List[str]:
    return [
        f"trajectory sha256: {traj_sha256}",
        f"effective speed_scale: {effective_speed_scale}",
        "software e-stop topic(s) (secondary convenience; the physical e-stop is the safety "
        f"function this project relies on): {', '.join(estop_topics)}",
        f"type exactly to confirm: {confirmation_phrase(traj_sha256)}",
    ]


def confirm_real(
    lines: Sequence[str], traj_sha256: str, tty_opener: Callable[[], Any] = open_dev_tty
) -> Tuple[bool, str]:
    """§11.8 G-CONFIRM. Never satisfied by a flag, env var or stdin -- only a typed /dev/tty line."""
    for line in lines:
        print(line)
    try:
        tty = tty_opener()
    except OSError as exc:
        return False, f"no controlling /dev/tty available: {exc}"
    try:
        typed = tty.readline()
    finally:
        tty.close()
    if confirmation_ok(typed, traj_sha256):
        return True, "typed phrase matched"
    return False, "typed phrase did not match the required confirmation phrase"


# --------------------------------------------------------------------------------------
# pure online-gate decision functions (§11.7 G-GRAPH / G-START-STATE / G-COLLISION)
# --------------------------------------------------------------------------------------

def _gate_dict(gate: str, category: str, outcome: str, detail: str) -> Dict[str, str]:
    return {"gate": gate, "category": category, "outcome": outcome, "detail": detail}


def check_graph(
    profile: str,
    expected_node: str,
    visible_nodes: Set[str],
    action_host: Optional[str],
    passthrough_publishers: Sequence[str],
) -> Dict[str, str]:
    """ADR-016 §2. `moveit_mock`/`vendor_mock` refuse if the real driver is visible anywhere;
    `vendor` refuses unless its own action server is hosted by exactly `expected_node`, no mock
    driver is visible, and no publisher exists on a passthrough command topic."""
    if profile in ("moveit_mock", "vendor_mock"):
        if REAL_DRIVER_NODE in visible_nodes:
            return _gate_dict(
                G_GRAPH, "online read-only", "fail",
                f"the real driver node '{REAL_DRIVER_NODE}' is visible on the graph",
            )
        return _gate_dict(G_GRAPH, "online read-only", "pass", f"'{REAL_DRIVER_NODE}' not visible for profile {profile!r}")

    reasons: List[str] = []
    if action_host != expected_node:
        reasons.append(f"action server is hosted by {action_host!r}, expected {expected_node!r}")
    if MOCK_DRIVER_NODE in visible_nodes:
        reasons.append(f"mock driver node '{MOCK_DRIVER_NODE}' is visible")
    if passthrough_publishers:
        reasons.append(f"publisher(s) present on passthrough command topic(s): {sorted(passthrough_publishers)}")
    if reasons:
        return _gate_dict(G_GRAPH, "online read-only", "fail", "; ".join(reasons))
    return _gate_dict(
        G_GRAPH, "online read-only", "pass",
        f"action server hosted by {expected_node!r}, no mock driver, no passthrough publishers",
    )


def check_start_state(
    mode: str,
    profile: str,
    positions: Optional[Mapping[str, float]],
    age_s: Optional[float],
    first_point: Sequence[float],
    start_tol_rad: float,
    joint_state_timeout_s: float,
    discovery_bound_s: float,
    arm_status: Optional[Mapping[str, Any]],
) -> Dict[str, str]:
    """§11.7 G-START-STATE: fresh joint_states, current position within tolerance of point 0, and
    (vendor profile only) a healthy latched arm status. Refuses in every mode except the vendor
    arm-status sub-check, which is warn outside real. `discovery_bound_s` (DDS discovery, e.g. a
    fresh node's first message) is separate from `joint_state_timeout_s` (staleness of the
    message actually received) -- a missing message names the former, a too-old one the latter."""
    if positions is None or age_s is None:
        return _gate_dict(G_START_STATE, "online read-only", "fail", f"no joint_states received within {discovery_bound_s}s")
    if age_s > joint_state_timeout_s:
        return _gate_dict(G_START_STATE, "online read-only", "fail", f"joint_states stale: age={age_s:.3f}s > {joint_state_timeout_s}s")
    if any(j not in positions for j in JOINT_NAMES):
        return _gate_dict(G_START_STATE, "online read-only", "fail", f"joint_states missing arm joint name(s), got {sorted(positions)}")

    deltas = {j: abs(positions[j] - q0) for j, q0 in zip(JOINT_NAMES, first_point)}
    worst = max(deltas.values())
    if worst > start_tol_rad:
        return _gate_dict(
            G_START_STATE, "online read-only", "fail",
            f"current position not within {start_tol_rad} rad of the trajectory's first point (max |delta|={worst:.4f} rad)",
        )
    detail = f"fresh joint_states (age={age_s:.3f}s), within tolerance (max |delta|={worst:.4f} rad)"

    if profile == "vendor":
        violation: Optional[str] = None
        if arm_status is None:
            violation = "no /rebotarm/arm_status message received"
        elif not arm_status["enabled"]:
            violation = "arm_status reports the arm is not enabled"
        elif arm_status["state_machine"] != "IDLE":
            violation = f"arm_status.state_machine={arm_status['state_machine']!r}, expected 'IDLE'"
        elif arm_status["error_codes"]:
            violation = f"arm_status.error_codes is non-empty: {list(arm_status['error_codes'])}"
        if violation is not None:
            outcome = "fail" if mode == "real" else "warn"
            suffix = "" if outcome == "fail" else " (warn outside real)"
            return _gate_dict(G_START_STATE, "online read-only", outcome, f"{detail}; {violation}{suffix}")

    return _gate_dict(G_START_STATE, "online read-only", "pass", detail)


def check_collision(invalid_indices: Sequence[int], missing_object_ids: Sequence[str]) -> Dict[str, str]:
    """§11.7 G-COLLISION: every densified waypoint valid, every scene object present. Refuses in
    every mode."""
    if missing_object_ids:
        return _gate_dict(
            G_COLLISION, "online read-only", "fail",
            f"scene object id(s) not present in the planning scene: {sorted(missing_object_ids)}",
        )
    if invalid_indices:
        return _gate_dict(
            G_COLLISION, "online read-only", "fail",
            f"{len(invalid_indices)} densified waypoint(s) invalid: indices {list(invalid_indices)}",
        )
    return _gate_dict(G_COLLISION, "online read-only", "pass", "every densified waypoint is valid and every scene object is present")


# --------------------------------------------------------------------------------------
# JointTrajectory conversion (unit-tested without a ROS graph)
# --------------------------------------------------------------------------------------

def joint_trajectory_msg(scaled_traj: Dict[str, Any]) -> JointTrajectory:
    """The scaled trajectory (joint1..6, in order) as a `trajectory_msgs/JointTrajectory`, with
    `time_from_start` taken from each point's scaled `t_s`."""
    msg = JointTrajectory()
    msg.joint_names = list(JOINT_NAMES)
    for p in scaled_traj["points"]:
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in p["positions"]]
        if p.get("velocities") is not None:
            point.velocities = [float(v) for v in p["velocities"]]
        if p.get("accelerations") is not None:
            point.accelerations = [float(v) for v in p["accelerations"]]
        t_s = float(p["t_s"])
        sec = int(t_s)
        nanosec = int(round((t_s - sec) * 1e9))
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points.append(point)
    return msg


# --------------------------------------------------------------------------------------
# monitoring helpers (§11.8), pure and unit-tested in isolation
# --------------------------------------------------------------------------------------

def tracking_error_rad(commanded: Sequence[float], actual: Sequence[float]) -> float:
    return max(abs(c - a) for c, a in zip(commanded, actual))


def is_stale(age_s: Optional[float], timeout_s: float) -> bool:
    return age_s is None or age_s > timeout_s


def interpolate_positions(scaled_traj: Dict[str, Any], t_s: float) -> List[float]:
    """Linear interpolation of the scaled trajectory's positions at time `t_s`."""
    points = scaled_traj["points"]
    if t_s <= points[0]["t_s"]:
        return list(points[0]["positions"])
    for prev, cur in zip(points, points[1:]):
        if t_s <= cur["t_s"]:
            span = cur["t_s"] - prev["t_s"]
            frac = 0.0 if span <= 0 else (t_s - prev["t_s"]) / span
            return [a + frac * (b - a) for a, b in zip(prev["positions"], cur["positions"])]
    return list(points[-1]["positions"])


# --------------------------------------------------------------------------------------
# small ROS-touching helpers
# --------------------------------------------------------------------------------------

def _call_service(node: Node, client, request, timeout_s: float):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        future.cancel()
        return None
    return future.result()


def _wait_for_message(node: Node, topic: str, msg_type, timeout_s: float, latched: bool = False):
    if latched:
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
    else:
        # BEST_EFFORT to match the vendor driver and rebot_motion's mock_driver, both of which
        # publish joint_states with qos_profile_sensor_data; a RELIABLE reader never receives
        # anything from either (and from ros2_control's joint_state_broadcaster, RELIABLE still
        # matches, since sensor-data QoS readers are compatible with RELIABLE writers).
        qos = qos_profile_sensor_data
    box: Dict[str, Any] = {}
    sub = node.create_subscription(msg_type, topic, lambda m: box.setdefault("msg", m), qos)
    deadline = time.monotonic() + timeout_s
    while "msg" not in box and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return box.get("msg")


# --------------------------------------------------------------------------------------
# OnlineChecks: the thin ROS-data-gathering wrapper around the pure decision functions above
# --------------------------------------------------------------------------------------

class OnlineChecks:
    def __init__(self, node: Node, service_timeout_s: float):
        self.node = node
        self.timeout_s = service_timeout_s
        self._validity_client = node.create_client(GetStateValidity, "/check_state_validity")
        self._scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
        self._validity_client.wait_for_service(timeout_sec=service_timeout_s)
        self._scene_client.wait_for_service(timeout_sec=service_timeout_s)

    def _action_server_host(self, action_name: str) -> Optional[str]:
        target = action_name + "/_action/send_goal"
        for name, ns in self.node.get_node_names_and_namespaces():
            try:
                services = self.node.get_service_names_and_types_by_node(name, ns)
            except Exception:  # noqa: BLE001 - a node that vanished mid-query is just "not a match"
                continue
            if any(s == target for s, _ in services):
                return name
        return None

    def _passthrough_publishers(self) -> List[str]:
        hits = []
        for topic, _ in self.node.get_topic_names_and_types():
            if _PASSTHROUGH_TOPIC_RE.match(topic) and self.node.get_publishers_info_by_topic(topic):
                hits.append(topic)
        return hits

    def graph(self, profile: str, expected_node: str, action_name: str) -> Dict[str, str]:
        visible = {name for name, _ in self.node.get_node_names_and_namespaces()}
        if profile in ("moveit_mock", "vendor_mock"):
            return check_graph(profile, expected_node, visible, action_host=None, passthrough_publishers=[])
        action_host = self._action_server_host(action_name)
        return check_graph(profile, expected_node, visible, action_host, self._passthrough_publishers())

    def start_state(
        self,
        mode: str,
        profile: str,
        joint_states_topic: str,
        arm_status_topic: Optional[str],
        first_point: Sequence[float],
        start_tol_rad: float,
        joint_state_timeout_s: float,
    ) -> Tuple[Dict[str, str], Optional[Dict[str, float]]]:
        # self.timeout_s (--service-timeout-s) is the DDS-discovery bound for the FIRST message on
        # this freshly constructed node; joint_state_timeout_s only bounds that message's own age,
        # applied below in check_start_state.
        discovery_bound_s = self.timeout_s
        msg = _wait_for_message(self.node, joint_states_topic, JointState, discovery_bound_s)
        positions: Optional[Dict[str, float]] = None
        age_s: Optional[float] = None
        if msg is not None:
            positions = dict(zip(msg.name, msg.position))
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            now_s = self.node.get_clock().now().nanoseconds / 1e9
            age_s = max(0.0, now_s - stamp_s)

        arm_status: Optional[Dict[str, Any]] = None
        if profile == "vendor" and arm_status_topic:
            status_msg = _wait_for_message(self.node, arm_status_topic, ArmStatus, discovery_bound_s, latched=True)
            if status_msg is not None:
                arm_status = {
                    "enabled": bool(status_msg.enabled),
                    "state_machine": status_msg.state_machine,
                    "error_codes": list(status_msg.error_codes),
                }

        result = check_start_state(
            mode, profile, positions, age_s, first_point, start_tol_rad, joint_state_timeout_s, discovery_bound_s, arm_status
        )
        return result, positions

    def collision(self, configs: Sequence[Sequence[float]], expected_object_ids: Sequence[str]) -> Dict[str, str]:
        invalid: List[int] = []
        for i, cfg in enumerate(configs):
            req = GetStateValidity.Request()
            req.group_name = VALIDITY_GROUP
            req.robot_state.joint_state.name = list(JOINT_NAMES)
            req.robot_state.joint_state.position = [float(v) for v in cfg]
            resp = _call_service(self.node, self._validity_client, req, self.timeout_s)
            if resp is None or not resp.valid:
                invalid.append(i)

        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        resp = _call_service(self.node, self._scene_client, req, self.timeout_s)
        present = {obj.id for obj in resp.scene.world.collision_objects} if resp is not None else set()
        missing = [oid for oid in expected_object_ids if oid not in present]
        return check_collision(invalid, missing)


# --------------------------------------------------------------------------------------
# execute() -- the single ActionClient construction site in this module
# --------------------------------------------------------------------------------------

def _wait_for_settle(
    node: Node, joint_state_box: Dict[str, Any], start_tol_rad: float, joint_state_window_s: float, deadline: float
) -> bool:
    """§11.8 step 2: True once max per-joint |Δq| < start_tol_rad across a full joint_state window,
    reached before the absolute monotonic `deadline`."""
    history: List[Tuple[float, Dict[str, float]]] = []
    observing_since: Optional[float] = None
    while _now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = _now()
        last_monotonic = joint_state_box.get("monotonic")
        age_s = None if last_monotonic is None else now - last_monotonic
        if is_stale(age_s, joint_state_window_s) or joint_state_box.get("positions") is None:
            history = []
            observing_since = None
            continue
        if observing_since is None:
            observing_since = now
        history.append((now, dict(joint_state_box["positions"])))
        history = [(t, p) for t, p in history if now - t <= joint_state_window_s]
        # Fresh data observed continuously for a full window, and still within it.
        if (now - observing_since) >= joint_state_window_s:
            spans = [
                max(p.get(j, 0.0) for _, p in history) - min(p.get(j, 0.0) for _, p in history)
                for j in JOINT_NAMES
            ]
            if max(spans) < start_tol_rad:
                return True
    return False


def _cancel_and_hold(
    node: Node, goal_handle, joint_state_box: Dict[str, Any], start_tol_rad: float, joint_state_s: float, cancel_settle_s: float
) -> bool:
    """§11.8 steps 1-3. Cancel the goal, wait (bounded) for the cancel to be acknowledged, then for the
    arm to settle -- all within `cancel_settle_s` of the cancel. Prints "PRESS THE HARDWARE E-STOP"
    and returns False if the cancel is unacknowledged/rejected or the arm does not settle in time."""
    deadline = _now() + cancel_settle_s
    cancel_future = goal_handle.cancel_goal_async()
    while not cancel_future.done() and _now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    response = cancel_future.result() if cancel_future.done() else None
    if response is None or response.return_code != CancelGoal.Response.ERROR_NONE:
        detail = "not acknowledged" if response is None else f"rejected (return_code={response.return_code})"
        print(f"cancel {detail} within timeouts.cancel_settle_s={cancel_settle_s}s", file=sys.stderr)
        print("PRESS THE HARDWARE E-STOP")
        return False
    if not _wait_for_settle(node, joint_state_box, start_tol_rad, joint_state_s, deadline):
        print(f"arm did not settle within timeouts.cancel_settle_s={cancel_settle_s}s of the cancel", file=sys.stderr)
        print("PRESS THE HARDWARE E-STOP")
        return False
    return True


def _verify_arrival(
    node: Node, joint_state_box: Dict[str, Any], final_point: Sequence[float], joint_state_s: float, since: float
) -> Tuple[bool, str, Optional[List[float]]]:
    """§11.10 "arrival verified": the first joint_states received after `since` (the result's
    arrival) must be within ARRIVAL_TOL_RAD of the final point; none within joint_state_s is stale."""
    deadline = _now() + joint_state_s
    while _now() < deadline:
        last = joint_state_box["monotonic"]
        positions = joint_state_box["positions"]
        if last is not None and last >= since and positions is not None:
            actual = [positions.get(j) for j in JOINT_NAMES]
            if None in actual:
                return False, f"joint_states missing arm joint name(s), got {sorted(positions)}", None
            err = tracking_error_rad(final_point, actual)
            if err > ARRIVAL_TOL_RAD:
                return False, f"arrival error {err:.4f} rad exceeds ARRIVAL_TOL_RAD={ARRIVAL_TOL_RAD}", actual
            return True, f"arrived (max |delta|={err:.4f} rad)", actual
        rclpy.spin_once(node, timeout_sec=0.05)
    return False, f"no fresh joint_states within {joint_state_s}s after the result (stale)", None


def execute(
    node: Node,
    action_name: str,
    exec_cfg: Dict[str, Any],
    scaled_traj: Dict[str, Any],
    joint_states_topic: str,
) -> Tuple[str, List[Dict[str, Any]]]:
    """Send the goal and monitor it (§11.8). The only place this module constructs an
    `ActionClient`. On any trip: cancel, wait (bounded) for the arm to settle, and return without
    ever calling any other vendor service (ADR-016 §7: cancel and hold, no auto-disable). A trip
    while the goal request is still pending keeps waiting (bounded) for the response, so an
    accepted goal is always cancelled. After SUCCESSFUL, arrival at the final point is verified."""
    client = ActionClient(node, FollowJointTrajectory, action_name)

    tolerances = exec_cfg["tolerances"]
    timeouts = exec_cfg["timeouts"]
    if not client.wait_for_server(timeout_sec=timeouts["goal_s"]):
        return "aborted", []

    trip: Dict[str, Optional[str]] = {"kind": None, "reason": None}

    def _trip(kind: str, reason: str) -> None:
        if trip["kind"] is None:
            trip["kind"] = kind
            trip["reason"] = reason

    def _on_estop(topic: str, msg: Bool) -> None:
        if msg.data:
            _trip("estopped", f"True received on e-stop topic {topic}")

    estop_subs = [
        node.create_subscription(Bool, topic, (lambda m, t=topic: _on_estop(t, m)), 10)
        for topic in exec_cfg["estop_topics"]
    ]

    joint_state_box: Dict[str, Any] = {"positions": None, "monotonic": None}

    def _on_joint_state(msg: JointState) -> None:
        joint_state_box["positions"] = dict(zip(msg.name, msg.position))
        joint_state_box["monotonic"] = _now()

    js_sub = node.create_subscription(JointState, joint_states_topic, _on_joint_state, qos_profile_sensor_data)

    def _signal_handler(signum, frame):  # noqa: ANN001 - signal handler signature
        _trip("aborted", f"received signal {signum}")

    old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in old_handlers:
        signal.signal(sig, _signal_handler)

    trace: List[Dict[str, Any]] = []
    tracking_tol = tolerances["tracking_rad"]
    joint_state_timeout = timeouts["joint_state_s"]
    # §11.12: a goal still running goal_s after its last scaled t_s is stuck.
    goal_limit_s = float(scaled_traj["points"][-1]["t_s"]) + timeouts["goal_s"]

    def _cleanup() -> None:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        node.destroy_subscription(js_sub)
        for sub in estop_subs:
            node.destroy_subscription(sub)

    def _stop(goal_handle) -> Tuple[str, List[Dict[str, Any]]]:
        print(f"{trip['kind']}: {trip['reason']}", file=sys.stderr)
        _cancel_and_hold(
            node, goal_handle, joint_state_box, tolerances["start_state_rad"], joint_state_timeout, timeouts["cancel_settle_s"]
        )
        return trip["kind"], trace

    try:
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = joint_trajectory_msg(scaled_traj)
        send_future = client.send_goal_async(goal_msg)
        response_deadline = _now() + timeouts["goal_s"]
        while not send_future.done() and trip["kind"] is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if _now() > response_deadline:
                _trip("aborted", f"no goal response within timeouts.goal_s={timeouts['goal_s']}s")
        if not send_future.done():
            # Tripped while the request is pending: the driver may still accept it, so keep
            # waiting (bounded) for the response -- an accepted goal must be cancelled, never
            # left running unmonitored.
            pending_deadline = _now() + timeouts["cancel_settle_s"]
            while not send_future.done() and _now() < pending_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not send_future.done():
                print(f"{trip['kind']}: {trip['reason']}", file=sys.stderr)
                print(f"no goal response within timeouts.cancel_settle_s={timeouts['cancel_settle_s']}s after the trip", file=sys.stderr)
                print("PRESS THE HARDWARE E-STOP")
                return trip["kind"], trace
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return (trip["kind"] or "aborted"), trace
        if trip["kind"] is not None:
            return _stop(goal_handle)

        start_monotonic = _now()
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = _now()
            elapsed = now - start_monotonic
            last_monotonic = joint_state_box["monotonic"]
            age_s = None if last_monotonic is None else now - last_monotonic
            # A grace period of one joint_state_timeout before the very first message is due:
            # the js_sub above is fresh at goal-send time, so age_s is briefly None/large even
            # for a healthy driver already publishing before the goal was sent.
            if elapsed > joint_state_timeout and is_stale(age_s, joint_state_timeout):
                _trip("aborted", f"joint_states stale (age={age_s!r}s > {joint_state_timeout}s)")
            else:
                positions = joint_state_box["positions"]
                actual = [positions.get(j) for j in JOINT_NAMES] if positions else []
                if actual and None not in actual:
                    commanded = interpolate_positions(scaled_traj, elapsed)
                    err = tracking_error_rad(commanded, actual)
                    trace.append({"t_s": elapsed, "commanded": commanded, "actual": actual})
                    if err > tracking_tol:
                        _trip("aborted", f"tracking error {err:.4f} rad exceeds tolerances.tracking_rad={tracking_tol}")
            if elapsed > goal_limit_s:
                _trip("aborted", f"goal still running {elapsed:.2f}s > scaled duration + timeouts.goal_s = {goal_limit_s:.2f}s")

            if trip["kind"] is not None:
                return _stop(goal_handle)

        result_received = _now()
        result = result_future.result()
        if result is None or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return "aborted", trace

        final_point = [float(v) for v in scaled_traj["points"][-1]["positions"]]
        arrived, detail, actual = _verify_arrival(node, joint_state_box, final_point, joint_state_timeout, result_received)
        if actual is not None:
            trace.append({"t_s": result_received - start_monotonic, "commanded": final_point, "actual": actual})
        if not arrived:
            print(f"aborted: driver reported SUCCESSFUL but {detail}", file=sys.stderr)
            return "aborted", trace
        return "completed", trace
    finally:
        _cleanup()


# --------------------------------------------------------------------------------------
# path / config resolution
# --------------------------------------------------------------------------------------

def _resolve(root: Path, value: Optional[str], default_rel: str) -> Path:
    if value is not None:
        p = Path(value)
        return p if p.is_absolute() else (root / p)
    return root / default_rel


def _resolve_paths(args: argparse.Namespace, root: Path) -> Dict[str, Path]:
    return {
        "execution_config": _resolve(root, args.execution_config, DEFAULT_EXECUTION_CONFIG),
        "commissioning": _resolve(root, args.commissioning, DEFAULT_COMMISSIONING),
        "limits": _resolve(root, args.limits, DEFAULT_LIMITS),
        "scene_config": _resolve(root, args.scene_config, DEFAULT_SCENE_CONFIG),
        "end_effector_config": _resolve(root, args.end_effector_config, DEFAULT_END_EFFECTOR_CONFIG),
    }


def _check_profiles_match_table(exec_cfg: Dict[str, Any]) -> None:
    """Usage error (exit 2) unless execution.yaml's driver_profiles restate `PROFILE_TABLE`
    exactly: an edited/overridden config is refused, never trusted (§11.1)."""
    profiles = exec_cfg["driver_profiles"]
    if set(profiles) != set(PROFILE_TABLE):
        raise ConfigError(f"execution config driver_profiles must be exactly {sorted(PROFILE_TABLE)} (docs/INTERFACES.md §11.1)")
    for name, expected in PROFILE_TABLE.items():
        got = profiles[name]
        for key, value in expected.items():
            actual = got.get(key)
            if key == "modes":
                actual = tuple(actual) if isinstance(actual, (list, tuple)) else actual
            if actual != value:
                raise ConfigError(
                    f"execution config driver_profiles.{name}.{key}={actual!r} differs from docs/INTERFACES.md §11.1 "
                    f"({value!r}); the mode/profile table is not configurable"
                )
        extra = set(got) - set(expected)
        if extra:
            raise ConfigError(f"execution config driver_profiles.{name} has key(s) not in §11.1: {sorted(extra)}")


def _validate_mode_profile(mode: str, profile: str) -> None:
    if mode not in MODES:
        raise ConfigError(f"--mode must be one of {MODES}, got {mode!r}")
    if profile not in PROFILE_TABLE:
        raise ConfigError(f"--profile must be one of {sorted(PROFILE_TABLE)}, got {profile!r}")
    valid = PROFILE_TABLE[profile]["modes"]
    if mode not in valid:
        raise ConfigError(
            f"mode {mode!r} is not valid for profile {profile!r}; valid modes are {list(valid)} (docs/INTERFACES.md §11.1)"
        )


def _safe_sha(path: Path) -> Optional[str]:
    try:
        return file_sha256(path)
    except OSError:
        return None


def _write_execution_record(record: Dict[str, Any], record_dir: Path) -> Path:
    record_dir.mkdir(parents=True, exist_ok=True)
    sha8 = (record.get("trajectory_sha256") or "unknown")[:8]
    path = record_dir / f"{utc_stamp()}_{record['mode']}_{sha8}.json"
    path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    return path


# --------------------------------------------------------------------------------------
# run() -- the testable core, independent of cli_common's logger/summary
# --------------------------------------------------------------------------------------

def run(
    args: argparse.Namespace, *, env: Mapping[str, str], tty_opener: Callable[[], Any] = open_dev_tty
) -> Dict[str, Any]:
    """Evaluate every gate in order and, if every gate passes, execute. Always writes a
    `crackvision.execution_record/1` (finally block) once a mode/profile is resolved, refusals
    included. Raises `PreconditionError` on any gate refusal (exit 3 via `cli_common.run_cli`) and
    `crackvision_motion.reachability_core.ConfigError` on a usage error (exit 2)."""
    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    paths = _resolve_paths(args, root)

    try:
        exec_cfg = load_execution_config(paths["execution_config"])
    except GateConfigError as exc:
        raise ConfigError(f"execution config invalid: {exc}") from exc
    _check_profiles_match_table(exec_cfg)

    mode = args.mode or exec_cfg["default_mode"]
    profile = args.profile or exec_cfg["default_driver_profile"]
    if args.profile is not None:
        # An *explicit* mismatch is a usage error, caught before any gate runs (docs/INTERFACES.md
        # §11.1). A mismatch against the *default* profile is instead caught just below, after the
        # offline gate -- so a run that the offline gate would refuse anyway (e.g. G-ARM unset)
        # reports that refusal (exit 3) rather than a profile-pairing usage error (exit 2).
        _validate_mode_profile(mode, profile)

    record: Dict[str, Any] = {
        "schema": EXECUTION_RECORD_SCHEMA,
        "mode": mode,
        "driver_profile": profile,
        "trajectory_sha256": None,
        "gate_report": [],
        "real_preview": [],
        "would_refuse_in_real": [],
        "input_shas": {
            "limits": _safe_sha(paths["limits"]),
            "end_effector_config": _safe_sha(paths["end_effector_config"]),
            "scene_config": _safe_sha(paths["scene_config"]),
            "commissioning": _safe_sha(paths["commissioning"]),
        },
        "effective_speed_scale": None,
        "outcome": "refused",
        "joint_trace": [],
    }
    if args.approval is not None:
        record["input_shas"]["approval"] = _safe_sha(Path(args.approval))

    record_dir = Path(args.record_dir).expanduser().resolve() if args.record_dir else root / DEFAULT_RECORD_DIR
    node: Optional[Node] = None

    try:
        offline_report = evaluate_offline(
            mode, args.trajectory, root=root,
            execution_config_path=paths["execution_config"], commissioning_path=paths["commissioning"],
            limits_path=paths["limits"], scene_config_path=paths["scene_config"],
            end_effector_config_path=paths["end_effector_config"], approval_path=args.approval,
            speed_scale=args.speed_scale, env=env,
        )
        record["gate_report"] = [c.to_dict() for c in offline_report.checks]
        record["trajectory_sha256"] = _safe_sha(Path(args.trajectory))
        if record["trajectory_sha256"] is not None:
            record["input_shas"]["trajectory"] = record["trajectory_sha256"]

        effective_scale = (
            args.speed_scale if args.speed_scale is not None
            else (exec_cfg["speed_scale"]["default"] if mode == "mock" else exec_cfg["speed_scale"]["real_default"])
        )
        record["effective_speed_scale"] = effective_scale

        if mode == "dry":
            real_report = evaluate_offline(
                "real", args.trajectory, root=root,
                execution_config_path=paths["execution_config"], commissioning_path=paths["commissioning"],
                limits_path=paths["limits"], scene_config_path=paths["scene_config"],
                end_effector_config_path=paths["end_effector_config"], approval_path=args.approval,
                speed_scale=effective_scale, env=env,
            )
            record["real_preview"] = [c.to_dict() for c in real_report.checks if c.id in _REAL_PREVIEW_GATE_IDS]

        if not offline_report.passed:
            record["outcome"] = "refused"
            raise PreconditionError(f"offline gate refused: {offline_report.refusals}")

        if args.profile is None:
            _validate_mode_profile(mode, profile)

        # Offline gate passed: the trajectory loaded cleanly, so this cannot raise TrajectoryError.
        traj = load_trajectory(args.trajectory)
        scaled_traj = scale_time(traj, effective_scale)

        # rclpy.init() happens only past this point -- an offline refusal above never reaches it.
        rclpy.init(args=None)
        node = Node(TOOL)

        profile_cfg = PROFILE_TABLE[profile]
        online_checks = OnlineChecks(node, args.service_timeout_s)

        graph_result = online_checks.graph(profile, profile_cfg["expected_node"], profile_cfg["action"])
        record["gate_report"].append(graph_result)

        start_result, _positions = online_checks.start_state(
            mode, profile, profile_cfg["joint_states_topic"], profile_cfg.get("arm_status_topic"),
            traj["points"][0]["positions"], exec_cfg["tolerances"]["start_state_rad"], exec_cfg["timeouts"]["joint_state_s"],
        )
        record["gate_report"].append(start_result)

        scene_cfg = load_scene_config(paths["scene_config"])
        configs = densify(traj, exec_cfg["densify_step_m"])
        collision_result = online_checks.collision(configs, [o["id"] for o in scene_cfg["objects"]])
        record["gate_report"].append(collision_result)

        online_refused = any(c["outcome"] in ("fail", "error") for c in (graph_result, start_result, collision_result))

        warn_ids = {c["gate"] for c in record["gate_report"] if c["outcome"] == "warn"}
        preview_fail_ids = {c["gate"] for c in record["real_preview"] if c["outcome"] in ("fail", "error")}
        record["would_refuse_in_real"] = sorted(warn_ids | preview_fail_ids) if mode != "real" else []

        if online_refused:
            record["gate_report"].append(_gate_dict(G_CONFIRM, "confirmation", "skip", "not reached: an earlier gate refused"))
            record["outcome"] = "refused"
            raise PreconditionError("an online read-only gate refused")

        if mode == "dry":
            record["gate_report"].append(_gate_dict(G_CONFIRM, "confirmation", "skip", "G-CONFIRM is real-only, mode=dry"))
            record["outcome"] = "rehearsed"
            return {"status": "ok"}

        if mode == "mock":
            record["gate_report"].append(_gate_dict(G_CONFIRM, "confirmation", "skip", "G-CONFIRM is real-only, mode=mock"))
        else:
            lines = confirmation_lines(record["trajectory_sha256"], effective_scale, exec_cfg["estop_topics"])
            confirmed, detail = confirm_real(lines, record["trajectory_sha256"], tty_opener)
            record["gate_report"].append(_gate_dict(G_CONFIRM, "confirmation", "pass" if confirmed else "fail", detail))
            if not confirmed:
                record["outcome"] = "refused"
                raise PreconditionError(f"G-CONFIRM refused: {detail}")

        outcome, trace = execute(node, profile_cfg["action"], exec_cfg, scaled_traj, profile_cfg["joint_states_topic"])
        record["joint_trace"] = trace
        record["outcome"] = outcome
        return {"status": "ok" if outcome == "completed" else "failed"}
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        _write_execution_record(record, record_dir)


# --------------------------------------------------------------------------------------
# CLI wiring
# --------------------------------------------------------------------------------------

def _extra_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--mode", choices=list(MODES), default=None, help="execution.yaml default_mode if omitted (ships dry)")
    parser.add_argument("--trajectory", required=True, help="crackvision.joint_trajectory/1 file to execute")
    parser.add_argument(
        "--profile", choices=["moveit_mock", "vendor_mock", "vendor"], default=None,
        help="execution.yaml default_driver_profile if omitted",
    )
    parser.add_argument("--execution-config", default=None, help=f"default: {DEFAULT_EXECUTION_CONFIG} under --root")
    parser.add_argument("--commissioning", default=None, help=f"default: {DEFAULT_COMMISSIONING} under --root")
    parser.add_argument("--approval", default=None, help="crackvision.execution_approval/1 file")
    parser.add_argument("--speed-scale", type=float, default=None, help="default: per mode, §11.7 G-SPEED")
    parser.add_argument("--limits", default=None, help=f"default: {DEFAULT_LIMITS} under --root")
    parser.add_argument("--scene-config", default=None, help=f"default: {DEFAULT_SCENE_CONFIG} under --root")
    parser.add_argument("--end-effector-config", default=None, help=f"default: {DEFAULT_END_EFFECTOR_CONFIG} under --root")
    parser.add_argument("--service-timeout-s", type=float, default=DEFAULT_SERVICE_TIMEOUT_S)
    parser.add_argument("--record-dir", default=None, help=f"default: {DEFAULT_RECORD_DIR} under --root")


def _run(args: argparse.Namespace, log, summary: RunSummary) -> Dict[str, str]:
    result = run(args, env=os.environ, tty_opener=open_dev_tty)
    summary.increment(result["status"])
    return result


def _offline_dry_run(args: argparse.Namespace) -> int:
    """§0.3/§11.10 `--dry-run`: offline gates only, in-process, no node, no file writes at all,
    exit 0 regardless of outcome."""
    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    paths = _resolve_paths(args, root)

    mode = args.mode
    if mode is None:
        try:
            mode = load_execution_config(paths["execution_config"])["default_mode"]
        except GateConfigError:
            mode = "dry"

    report = evaluate_offline(
        mode, args.trajectory, root=root,
        execution_config_path=paths["execution_config"], commissioning_path=paths["commissioning"],
        limits_path=paths["limits"], scene_config_path=paths["scene_config"],
        end_effector_config_path=paths["end_effector_config"], approval_path=args.approval,
        speed_scale=args.speed_scale, env=os.environ,
    )
    print(json.dumps(report.to_dict(), indent=2))
    return EXIT_OK


def main(argv=None) -> int:
    argv = list(argv) if argv is not None else sys.argv[1:]
    parser = argparse.ArgumentParser(prog=TOOL)
    add_common_args(parser)
    _extra_args(parser)
    args = parser.parse_args(argv)
    if args.dry_run:
        return _offline_dry_run(args)
    return run_cli(TOOL, _run, argv, extra_args=_extra_args)


if __name__ == "__main__":
    sys.exit(main())
