#!/usr/bin/env python3
"""scripts/motion/check_vendor_driver.py — vendor-driver readiness check (MOT-11).

Pure, read-only static checker. Reads vendor files under --rebot-ws (never ~/rebot_ws itself from
this repo's perspective: this script never writes there) plus this repo's limits/execution config
and executor source, and reports whether reBotArmController's port, motor IDs, joint order,
direction and gripper mapping match the operator's bring-up evidence (--facts). It never imports
motorbridge/serial/rclpy/subprocess, starts no process and opens no device — every vendor file is
read with `pathlib.Path.read_text`/`read_bytes` only.

A PASS here does not commission anything (see docs/motion/VENDOR_DRIVER.md).

Usage:
    ./env.sh python scripts/motion/check_vendor_driver.py [--rebot-ws PATH] [--facts PATH]
        [--limits-file PATH] [--execution-config PATH] [--executor-file PATH]
        [--driver-pythonpath PATH ...] [--format text|json] [--json-out PATH]
        [--root PATH] [--config PATH] [--verbose] [--quiet] [--dry-run] [--skip-existing]

Exit codes (docs/INTERFACES.md §0.2, redefined per this card's acceptance criteria):
    0  no FAIL (WARN/INFO allowed)
    1  unexpected error
    2  usage error or a required input file is missing
    3  at least one FAIL (vendor stack not ready)
"""

from __future__ import annotations

import argparse
import ast
import json
import logging
import re
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

EXIT_OK = 0
EXIT_RUNTIME = 1
EXIT_USAGE = 2
EXIT_NOT_READY = 3

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_NAME = "check_vendor_driver"

PASS, FAIL, WARN, INFO = "PASS", "FAIL", "WARN", "INFO"

DIRECTION_KEYS = {"sign", "direction", "invert", "offset"}
ZERO_KEYS = {"zero_offset", "zero"}

STANDARD_PYTHON_SEARCH_DIRS = [
    "/usr/lib/python3.10/dist-packages",
    "/usr/lib/python3/dist-packages",
    "/usr/local/lib/python3.10/dist-packages",
    str(Path.home() / ".local" / "lib" / "python3.10" / "site-packages"),
]

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@dataclass
class Check:
    id: str
    status: str
    message: str
    citation: str = ""
    compared: dict[str, Any] = field(default_factory=dict)
    remedy: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "status": self.status,
            "message": self.message,
            "citation": self.citation,
            "compared": self.compared,
            "remedy": self.remedy,
        }


class InputMissing(Exception):
    pass


# --------------------------------------------------------------------------------------
# read-only file helpers
# --------------------------------------------------------------------------------------


def _read_text(path: Path) -> str:
    if not path.is_file():
        raise InputMissing(str(path))
    return path.read_text(encoding="utf-8", errors="replace")


def _read_yaml(path: Path) -> Any:
    import yaml

    return yaml.safe_load(_read_text(path))


def _hex_to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, int):
        return value
    try:
        return int(str(value), 0)
    except ValueError:
        return None


class VendorLayout:
    """Relative paths of the vendor files this checker reads, rooted at --rebot-ws."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.arm_yaml = root / "src/rebotarm_bringup/config/arm.yaml"
        self.gripper_yaml = root / "src/rebotarm_bringup/config/gripper.yaml"
        self.driver_params_yaml = root / "src/rebotarm_bringup/config/driver_params.yaml"
        self.safe_park_yaml = root / "src/rebotarm_bringup/config/safe_park.yaml"
        self.bringup_launch = root / "src/rebotarm_bringup/launch/bringup.launch.py"
        ctrl = root / "src/rebotarmcontroller/rebotarmcontroller"
        self.hardware_manager = ctrl / "hardware_manager.py"
        self.ros_services = ctrl / "ros_services.py"
        self.ros_publishers = ctrl / "ros_publishers.py"
        self.ros_actions = ctrl / "ros_actions.py"
        self.rebotarm_controller = ctrl / "rebotarm_controller.py"
        sdk = root / "third_party/reBotArm_control_py"
        self.sdk_root = sdk
        self.sdk_pkg = sdk / "reBotArm_control_py"
        self.actuator_init = self.sdk_pkg / "actuator" / "__init__.py"
        self.actuator_rebotarm = self.sdk_pkg / "actuator" / "rebotarm.py"
        self.gripper_module = self.sdk_pkg / "actuator" / "gripper.py"
        self.controllers_init = self.sdk_pkg / "controllers" / "__init__.py"
        self.sdk_git_dir = sdk / ".git"


# --------------------------------------------------------------------------------------
# small, self-contained static-analysis helpers (no execution of vendor code)
# --------------------------------------------------------------------------------------


def _exported_names(init_source: str) -> set[str]:
    """Names a package's __init__.py re-exports: `from .mod import A, B as C` -> {A, C}."""
    names: set[str] = set()
    try:
        tree = ast.parse(init_source)
    except SyntaxError:
        return names
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                names.add(alias.asname or alias.name)
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "__all__":
                    if isinstance(node.value, (ast.List, ast.Tuple)):
                        for elt in node.value.elts:
                            if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                                names.add(elt.value)
    return names


def _wanted_sdk_imports(source: str) -> list[tuple[str, list[str], int]]:
    """`from reBotArm_control_py.X import A, B` lines in a consumer file -> [(X, [A, B], lineno), ...]."""
    wanted: list[tuple[str, list[str], int]] = []
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return wanted
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith("reBotArm_control_py"):
            submodule = node.module[len("reBotArm_control_py") :].lstrip(".")
            names = [alias.name for alias in node.names]
            wanted.append((submodule, names, node.lineno))
    return wanted


def _line_no(text: str, pos: int) -> int:
    """1-based line number of a character offset within `text`."""
    return text.count("\n", 0, pos) + 1


def _mline(text: str, match: Optional[re.Match]) -> Optional[int]:
    """1-based line number of a regex match's start, or None if there was no match."""
    return _line_no(text, match.start()) if match else None


def _cite(path: Path, text: str, *needles: str) -> str:
    """`path:line1,line2` for every literal `needle` found in `text`, else a bare path."""
    lines = sorted({_line_no(text, idx) for idx in (text.find(n) for n in needles) if idx != -1})
    if not lines:
        return str(path)
    return f"{path}:{','.join(str(line) for line in lines)}"


def _cite_multi(*entries: tuple[Path, str, list[str]]) -> str:
    return "; ".join(_cite(path, text, *needles) for path, text, needles in entries)


def _git_head_commit(git_dir: Path) -> Optional[str]:
    """Resolve a plain (non-submodule) .git directory's HEAD to a commit hash, no git invoked."""
    head_file = git_dir / "HEAD"
    if not head_file.is_file():
        return None
    head = head_file.read_text(encoding="utf-8").strip()
    if not head.startswith("ref:"):
        return head or None
    ref = head.split(":", 1)[1].strip()
    ref_file = git_dir / ref
    if ref_file.is_file():
        return ref_file.read_text(encoding="utf-8").strip()
    packed = git_dir / "packed-refs"
    if packed.is_file():
        for line in packed.read_text(encoding="utf-8").splitlines():
            if line.endswith(" " + ref):
                return line.split()[0]
    return None


# --------------------------------------------------------------------------------------
# individual checks
# --------------------------------------------------------------------------------------


def check_port(layout: VendorLayout, facts: dict) -> Check:
    arm_text = _read_text(layout.arm_yaml)
    gripper_text = _read_text(layout.gripper_yaml)
    driver_params_text = _read_text(layout.driver_params_yaml)
    arm_cfg = _read_yaml(layout.arm_yaml) or {}
    gripper_cfg = _read_yaml(layout.gripper_yaml) or {}
    driver_params = _read_yaml(layout.driver_params_yaml) or {}
    arm_channel = arm_cfg.get("channel", "")
    gripper_channel = gripper_cfg.get("channel", "")
    driver_channel = (
        driver_params.get("reBotArmController", {}).get("ros__parameters", {}).get("channel", "")
    )
    facts_port = (facts.get("bus") or {}).get("port")

    channels = {
        f"{layout.arm_yaml.name}:channel": arm_channel,
        f"{layout.gripper_yaml.name}:channel": gripper_channel,
        f"{layout.driver_params_yaml.name}:channel": driver_channel,
    }
    bad = {k: v for k, v in channels.items() if v and not str(v).startswith("/dev/tty")}
    compared = {**channels, "facts.bus.port": facts_port}
    channel_citation = _cite_multi(
        (layout.arm_yaml, arm_text, ["channel:"]),
        (layout.gripper_yaml, gripper_text, ["channel:"]),
        (layout.driver_params_yaml, driver_params_text, ["channel:"]),
    )

    rebot_arm_warn = Path("/dev/rebot_arm").exists()
    if bad:
        return Check(
            "V-PORT",
            FAIL,
            f"configured channel(s) do not start with /dev/tty: {sorted(bad)}",
            citation=f"{channel_citation} vs facts.bus.port",
            compared=compared,
            remedy="set the vendor arm.yaml/gripper.yaml channel to a /dev/tty* device path",
        )
    status = PASS if arm_channel == facts_port else WARN
    msg = (
        f"arm channel {arm_channel!r} matches facts.bus.port {facts_port!r}"
        if status == PASS
        else f"arm channel {arm_channel!r} does not match facts.bus.port {facts_port!r}"
    )
    if rebot_arm_warn:
        status = WARN
        msg += "; WARN: /dev/rebot_arm exists on this host and deploy/_env.sh:59 would prefer it"
    return Check("V-PORT", status, msg, citation=f"{channel_citation} vs facts.bus.port", compared=compared)


def check_baud(layout: VendorLayout, facts: dict) -> Check:
    sdk_source = _read_text(layout.actuator_rebotarm)
    match = re.search(r"from_dm_serial\([^,]+,\s*(\d+)\)", sdk_source)
    sdk_baud = int(match.group(1)) if match else None
    facts_baud = (facts.get("bus") or {}).get("serial_baud")
    compared = {"sdk_hardcoded_baud": sdk_baud, "facts.bus.serial_baud": facts_baud}
    if sdk_baud is None:
        return Check(
            "V-BAUD", WARN, "could not find a hard-coded baud in the SDK",
            citation=str(layout.actuator_rebotarm), compared=compared,
        )
    status = PASS if sdk_baud == facts_baud else FAIL
    return Check(
        "V-BAUD",
        status,
        f"SDK hard-coded baud {sdk_baud} vs facts {facts_baud}",
        citation=f"{layout.actuator_rebotarm}:{_mline(sdk_source, match)} Controller.from_dm_serial(...)",
        compared=compared,
        remedy="" if status == PASS else "the SDK baud is hard-coded; cannot be fixed by config",
    )


def check_ids(layout: VendorLayout, facts: dict) -> Check:
    arm_text = _read_text(layout.arm_yaml)
    gripper_text = _read_text(layout.gripper_yaml)
    arm_cfg = _read_yaml(layout.arm_yaml) or {}
    gripper_cfg = _read_yaml(layout.gripper_yaml) or {}
    joints_facts = facts.get("joints") or {}

    mismatches = []
    compared: dict[str, Any] = {}
    for joint in arm_cfg.get("joints", []):
        name = joint.get("name")
        f = joints_facts.get(name) or {}
        vendor_vals = {
            "motor_id": _hex_to_int(joint.get("motor_id")),
            "feedback_id": _hex_to_int(joint.get("feedback_id")),
            "model": joint.get("model"),
        }
        facts_vals = {
            "motor_id": _hex_to_int(f.get("motor_id")),
            "feedback_id": _hex_to_int(f.get("feedback_id")),
            "model": f.get("model"),
        }
        compared[name] = {"vendor": vendor_vals, "facts": facts_vals}
        if vendor_vals != facts_vals:
            mismatches.append(name)

    for gripper in gripper_cfg.get("gripper", []):
        name = gripper.get("name", "gripper")
        f = joints_facts.get("gripper") or {}
        vendor_vals = {
            "motor_id": _hex_to_int(gripper.get("motor_id")),
            "feedback_id": _hex_to_int(gripper.get("feedback_id")),
            "model": gripper.get("model"),
        }
        facts_vals = {
            "motor_id": _hex_to_int(f.get("motor_id")),
            "feedback_id": _hex_to_int(f.get("feedback_id")),
            "model": f.get("model"),
        }
        compared[name] = {"vendor": vendor_vals, "facts": facts_vals}
        if vendor_vals != facts_vals:
            mismatches.append(name)

    status = PASS if not mismatches else FAIL
    citation = _cite_multi(
        (layout.arm_yaml, arm_text, ["joints:"]),
        (layout.gripper_yaml, gripper_text, ["gripper:"]),
    )
    return Check(
        "V-IDS",
        status,
        "motor/feedback IDs and models match facts" if status == PASS else f"mismatch on {mismatches}",
        citation=f"{citation} vs facts.joints",
        compared=compared,
        remedy="" if status == PASS else "fix motor_id/feedback_id/model in arm.yaml or gripper.yaml to match facts",
    )


def check_order(layout: VendorLayout) -> Check:
    arm_text = _read_text(layout.arm_yaml)
    arm_cfg = _read_yaml(layout.arm_yaml) or {}
    names = [j.get("name") for j in arm_cfg.get("joints", [])]
    status = PASS if names == ARM_JOINTS else FAIL
    return Check(
        "V-ORDER",
        status,
        f"arm.yaml joint order = {names}",
        citation=f"{_cite(layout.arm_yaml, arm_text, 'joints:')} joints[].name vs INTERFACES §11.2 / URDF joint1..joint6",
        compared={"vendor_order": names, "expected_order": ARM_JOINTS},
        remedy="" if status == PASS else "reorder arm.yaml's joints list to joint1..joint6",
    )


def _find_direction_keys(cfg_joints: list[dict]) -> dict[str, list[str]]:
    found: dict[str, list[str]] = {}
    for joint in cfg_joints:
        hit = sorted(DIRECTION_KEYS & set(joint.keys()))
        if hit:
            found[joint.get("name", "?")] = hit
    return found


def check_direction(layout: VendorLayout, facts: dict) -> Check:
    arm_text = _read_text(layout.arm_yaml)
    gripper_text = _read_text(layout.gripper_yaml)
    arm_cfg = _read_yaml(layout.arm_yaml) or {}
    gripper_cfg = _read_yaml(layout.gripper_yaml) or {}
    key_hits = _find_direction_keys(arm_cfg.get("joints", []))
    key_hits.update(_find_direction_keys(gripper_cfg.get("gripper", [])))

    joints_facts = facts.get("joints") or {}
    bad_verdicts = [
        name
        for name in ARM_JOINTS
        if ((joints_facts.get(name) or {}).get("sign_check") or {}).get("verdict") != "match"
    ]

    compared = {"direction_keys_found": key_hits, "sign_check_verdicts_not_match": bad_verdicts}
    found_keys = sorted({k for hits in key_hits.values() for k in hits})
    citation = _cite_multi(
        (layout.arm_yaml, arm_text, found_keys or ["joints:"]),
        (layout.gripper_yaml, gripper_text, found_keys or ["gripper:"]),
    )
    if key_hits or bad_verdicts:
        return Check(
            "V-DIRECTION",
            FAIL,
            f"direction/offset keys={key_hits or 'none'}, non-matching sign checks={bad_verdicts or 'none'}",
            citation=f"{citation} vs facts.joints.*.sign_check.verdict",
            compared=compared,
            remedy="remove any sign/direction/offset/invert key from the vendor joint config; re-run the facts sign check",
        )
    return Check(
        "V-DIRECTION",
        PASS,
        "no direction/offset keys; every facts sign_check verdict is 'match'",
        citation=f"{citation} vs facts.joints.*.sign_check.verdict",
        compared=compared,
    )


def check_zero(layout: VendorLayout, facts: dict) -> Check:
    arm_text = _read_text(layout.arm_yaml)
    gripper_text = _read_text(layout.gripper_yaml)
    arm_cfg = _read_yaml(layout.arm_yaml) or {}
    gripper_cfg = _read_yaml(layout.gripper_yaml) or {}
    zero_hits = {}
    for joint in arm_cfg.get("joints", []) + gripper_cfg.get("gripper", []):
        hit = sorted(ZERO_KEYS & set(joint.keys()))
        if hit:
            zero_hits[joint.get("name", "?")] = hit

    zero_facts = facts.get("zero") or {}
    consistent = zero_facts.get("consistent_with_urdf_home")
    verified = zero_facts.get("verified_after_power_cycle")
    compared = {"zero_keys_found": zero_hits, "facts.zero.consistent_with_urdf_home": consistent,
                "facts.zero.verified_after_power_cycle": verified}

    found_keys = sorted({k for hits in zero_hits.values() for k in hits})
    citation = _cite_multi(
        (layout.arm_yaml, arm_text, found_keys or ["joints:"]),
        (layout.gripper_yaml, gripper_text, found_keys or ["gripper:"]),
    )
    ok = not zero_hits and bool(consistent) and bool(verified)
    return Check(
        "V-ZERO",
        PASS if ok else FAIL,
        "no zero-offset keys in vendor configs; facts zero is consistent with the URDF home"
        if ok
        else f"zero_keys={zero_hits or 'none'} consistent={consistent!r} verified={verified!r}",
        citation=f"{citation} vs facts.zero",
        compared=compared,
        remedy="" if ok else "remove zero-offset keys from vendor configs, or re-run the facts zero procedure",
    )


def check_gripper_sign(layout: VendorLayout, facts: dict) -> Check:
    services_src = _read_text(layout.ros_services)
    publishers_src = _read_text(layout.ros_publishers)

    def _const(src: str, name: str) -> tuple[Optional[float], Optional[re.Match]]:
        m = re.search(rf"{name}\s*=\s*(-?\d+\.?\d*)", src)
        return (float(m.group(1)), m) if m else (None, None)

    open_pos, open_m = _const(services_src, "_GRIPPER_OPEN_POSITION")
    close_pos, close_m = _const(services_src, "_GRIPPER_CLOSE_POSITION")
    open_motor, open_motor_m = _const(publishers_src, "_GRIPPER_OPEN_MOTOR_POSITION")
    closed_motor, closed_motor_m = _const(publishers_src, "_GRIPPER_CLOSED_MOTOR_POSITION")
    facts_open_sign = ((facts.get("joints") or {}).get("gripper") or {}).get("open_sign")

    compared = {
        "ros_services._GRIPPER_OPEN_POSITION": open_pos,
        "ros_services._GRIPPER_CLOSE_POSITION": close_pos,
        "ros_publishers._GRIPPER_OPEN_MOTOR_POSITION": open_motor,
        "ros_publishers._GRIPPER_CLOSED_MOTOR_POSITION": closed_motor,
        "facts.joints.gripper.open_sign": facts_open_sign,
    }
    services_lines = sorted({ln for ln in (_mline(services_src, open_m), _mline(services_src, close_m)) if ln})
    publishers_lines = sorted({ln for ln in (_mline(publishers_src, open_motor_m), _mline(publishers_src, closed_motor_m)) if ln})
    citation_parts = []
    if services_lines:
        citation_parts.append(f"{layout.ros_services}:{','.join(map(str, services_lines))}")
    if publishers_lines:
        citation_parts.append(f"{layout.ros_publishers}:{','.join(map(str, publishers_lines))}")
    citation = (", ".join(citation_parts) or f"{layout.ros_services}, {layout.ros_publishers}") + " vs facts.joints.gripper.open_sign"
    ok = (
        open_pos is not None and open_pos < 0
        and close_pos == 0.0
        and open_motor is not None and open_motor < 0
        and closed_motor == 0.0
        and facts_open_sign == "negative"
    )
    return Check(
        "V-GRIPPER-SIGN",
        PASS if ok else FAIL,
        "gripper open is negative, closed is 0.0, matching facts.open_sign"
        if ok
        else "gripper open/closed sign does not match the negative-open convention",
        citation=citation,
        compared=compared,
        remedy="" if ok else "fix _GRIPPER_OPEN_POSITION/_GRIPPER_OPEN_MOTOR_POSITION to be negative with a 0.0 closed position",
    )


def check_gripper_range(layout: VendorLayout, facts: dict) -> Check:
    gripper_facts = (facts.get("joints") or {}).get("gripper") or {}
    full_open_rad = gripper_facts.get("full_open_rad")
    largest_observed = ((gripper_facts.get("sign_check") or {}).get("peak_rad"))
    services_src = _read_text(layout.ros_services)
    publishers_src = _read_text(layout.ros_publishers)
    m1 = re.search(r"_GRIPPER_OPEN_POSITION\s*=\s*(-?\d+\.?\d*)", services_src)
    m2 = re.search(r"_GRIPPER_OPEN_MOTOR_POSITION\s*=\s*(-?\d+\.?\d*)", publishers_src)
    compared = {
        "facts.full_open_rad": full_open_rad,
        "largest_observed_peak_rad": largest_observed,
        "vendor_full_open_command": float(m1.group(1)) if m1 else None,
        "publisher_full_open_rad": float(m2.group(1)) if m2 else None,
    }
    citation = (
        f"{_cite(layout.ros_services, services_src, '_GRIPPER_OPEN_POSITION')}, "
        f"{_cite(layout.ros_publishers, publishers_src, '_GRIPPER_OPEN_MOTOR_POSITION')} vs facts.joints.gripper"
    )
    if full_open_rad is not None:
        return Check("V-GRIPPER-RANGE", PASS, "full_open_rad has been measured", citation=citation, compared=compared)
    return Check(
        "V-GRIPPER-RANGE",
        WARN,
        "the vendor full-open command and the publisher's full-open constant are unverified on "
        "this unit: facts.full_open_rad is null and the largest observed opening is "
        f"{largest_observed} rad",
        citation=citation,
        compared=compared,
        remedy="measure the true full-open motor angle before relying on the -4.9/-5.0 rad constants",
    )


def check_gripper_width(layout: VendorLayout, limits: dict) -> Check:
    publishers_src = _read_text(layout.ros_publishers)
    m = re.search(r"_GRIPPER_MAX_WIDTH\s*=\s*(-?\d+\.?\d*)", publishers_src)
    max_width = float(m.group(1)) if m else None
    published_half_width = max_width / 2.0 if max_width is not None else None
    joints = limits.get("joints") or {}
    urdf_upper = (joints.get("gripper_joint1") or {}).get("upper")
    compared = {
        "published_max_half_width_m": published_half_width,
        "urdf_gripper_joint_upper_m": urdf_upper,
    }
    citation = f"{_cite(layout.ros_publishers, publishers_src, '_GRIPPER_MAX_WIDTH')} vs b601_dm_limits.yaml joints.gripper_joint1.upper"
    if published_half_width is not None and urdf_upper is not None and abs(published_half_width - urdf_upper) < 1e-9:
        return Check("V-GRIPPER-WIDTH", PASS, "published finger width matches the URDF upper limit", citation=citation, compared=compared)
    return Check(
        "V-GRIPPER-WIDTH",
        WARN,
        f"published finger width max {published_half_width} m vs URDF gripper_joint1/2 upper {urdf_upper} m",
        citation=citation,
        compared=compared,
        remedy="reconcile the publisher's _GRIPPER_MAX_WIDTH with the URDF finger travel, or document the discrepancy",
    )


def check_gripper_units(layout: VendorLayout) -> Check:
    actions_src = _read_text(layout.ros_actions)
    hw_src = _read_text(layout.hardware_manager)
    hit = "set_gripper_target(goal.position)" in actions_src
    citation = _cite_multi(
        (layout.ros_actions, actions_src, ["set_gripper_target(goal.position)"]),
        (layout.hardware_manager, hw_src, ["def set_gripper_target"]),
    )
    return Check(
        "V-GRIPPER-UNITS",
        WARN,
        "GripperCommand goal.position (metres, ROS convention) is passed straight to "
        "set_gripper_target as motor radians"
        if hit
        else "could not confirm the goal.position -> motor-radians passthrough",
        citation=citation,
        compared={"goal_position_passed_as_motor_radians": hit},
        remedy="convert GripperCommand goal.position (metres) to a motor angle before calling set_gripper_target",
    )


def check_safe_park(layout: VendorLayout, limits: dict) -> Check:
    safe_park_text = _read_text(layout.safe_park_yaml)
    safe_park = (_read_yaml(layout.safe_park_yaml) or {}).get("reBotArmController", {}).get("ros__parameters", {})
    enabled = bool(safe_park.get("safe_park_enabled"))
    q = safe_park.get("safe_park_q") or []
    joints_limits = limits.get("joints") or {}

    violations = []
    for name, value in zip(ARM_JOINTS, q):
        bounds = joints_limits.get(name)
        if not bounds:
            continue
        lower, upper = bounds.get("lower"), bounds.get("upper")
        if (lower is not None and value < lower) or (upper is not None and value > upper):
            violations.append({"joint": name, "value": value, "lower": lower, "upper": upper})

    compared = {"safe_park_enabled": enabled, "safe_park_q": q, "violations": violations}
    hw_src = _read_text(layout.hardware_manager)
    seeds_on_connect = "_seed_endpos_target" in hw_src and "connect" in hw_src
    parks_on_shutdown = "self.safe_park()" in hw_src and "shutdown" in hw_src

    config_citation = _cite(layout.safe_park_yaml, safe_park_text, "safe_park_q:")
    hw_citation = (
        f"{_cite(layout.hardware_manager, hw_src, 'def connect', '_seed_endpos_target()')} seeds this "
        f"target at connect; {_cite(layout.hardware_manager, hw_src, 'def shutdown', 'self.safe_park()')} "
        "parks to it on shutdown"
    )
    citation = f"{config_citation} vs b601_dm_limits.yaml; {hw_citation}"

    if enabled and violations:
        return Check(
            "V-SAFE-PARK",
            FAIL,
            f"safe_park enabled with safe_park_q outside the joint limits: {violations}",
            citation=citation,
            compared=compared,
            remedy="fix safe_park_q so every joint is within b601_dm_limits.yaml, or disable safe_park until it is",
        )
    status = PASS if not enabled or not violations else FAIL
    return Check(
        "V-SAFE-PARK",
        status,
        "safe_park_q is within every joint limit" if status == PASS else "safe_park_q violates a joint limit",
        citation=citation,
        compared={**compared, "seeds_on_connect": seeds_on_connect, "parks_on_shutdown": parks_on_shutdown},
    )


def check_startup(layout: VendorLayout) -> Check:
    hw_src = _read_text(layout.hardware_manager)
    has_mode_write = "mode_pos_vel" in hw_src
    has_enable_all = "enable_all" in hw_src
    citation = (
        f"{_cite(layout.hardware_manager, hw_src, 'def connect', 'def init_gripper')} connect()/init_gripper(); "
        f"{_cite(layout.hardware_manager, hw_src, '.start()')} SDK start (ArmEndPos)"
    )
    return Check(
        "V-STARTUP",
        WARN,
        "launching the driver writes the POS_VEL control mode and enables every motor "
        "(including the gripper) before starting the position-hold loop"
        if (has_mode_write and has_enable_all)
        else "could not confirm the mode_pos_vel/enable_all startup sequence",
        citation=citation,
        compared={"mode_pos_vel_present": has_mode_write, "enable_all_present": has_enable_all},
        remedy="confirm with the operator that every motor should energise on driver launch",
    )


def check_gains(layout: VendorLayout, limits: dict, kp_cap: float, kd_cap: float) -> Check:
    arm_text = _read_text(layout.arm_yaml)
    arm_cfg = _read_yaml(layout.arm_yaml) or {}
    joints_limits = limits.get("joints") or {}
    over_velocity = []
    over_gain = []
    for joint in arm_cfg.get("joints", []):
        name = joint.get("name")
        vlim = (joint.get("POS_VEL") or {}).get("vlim")
        kp = (joint.get("MIT") or {}).get("kp")
        kd = (joint.get("MIT") or {}).get("kd")
        planning_vel = (joints_limits.get(name) or {}).get("velocity")
        if vlim is not None and planning_vel is not None and vlim > planning_vel:
            over_velocity.append({"joint": name, "vlim": vlim, "planning_velocity": planning_vel})
        if kp is not None and kp > kp_cap:
            over_gain.append({"joint": name, "kp": kp, "kp_cap": kp_cap})
        if kd is not None and kd > kd_cap:
            over_gain.append({"joint": name, "kd": kd, "kd_cap": kd_cap})
    compared = {"over_velocity": over_velocity, "over_gain": over_gain}
    citation = (
        f"{_cite(layout.arm_yaml, arm_text, 'POS_VEL:', 'MIT:')} POS_VEL.vlim/MIT.kp,kd vs "
        "b601_dm_limits.yaml velocity and the 2026-10-10 bring-up caps (operator statement)"
    )
    if not over_velocity and not over_gain:
        return Check("V-GAINS", PASS, "POS_VEL vlim and MIT kp/kd are within the planning/bring-up caps", citation=citation, compared=compared)
    return Check(
        "V-GAINS",
        WARN,
        f"POS_VEL vlim exceeds the planning velocity limit for {[v['joint'] for v in over_velocity]}; "
        f"MIT kp/kd exceeds the bring-up cap for {[g['joint'] for g in over_gain]}",
        citation=citation,
        compared=compared,
        remedy="lower POS_VEL vlim to the planning velocity limit and MIT kp/kd to the bring-up caps before any real run",
    )


# Last SDK commit known to export RobotArm/ArmEndPos (the names hardware_manager.py imports).
LAST_SDK_COMMIT_WITH_ROBOTARM = "8427281"


def check_sdk_api(layout: VendorLayout) -> Check:
    hw_src = _read_text(layout.hardware_manager)
    wanted = _wanted_sdk_imports(hw_src)

    missing: list[str] = []
    for submodule, names, _lineno in wanted:
        if submodule in ("actuator", "controllers"):
            init_path = layout.actuator_init if submodule == "actuator" else layout.controllers_init
            exported = _exported_names(_read_text(init_path)) if init_path.is_file() else set()
            for name in names:
                if name not in exported:
                    missing.append(f"{submodule}.{name}")
        elif submodule == "actuator.gripper":
            gripper_src = _read_text(layout.gripper_module) if layout.gripper_module.is_file() else ""
            for name in names:
                if f"def {name}" not in gripper_src and f"{name} =" not in gripper_src:
                    missing.append(f"actuator.gripper.{name}")

    head = _git_head_commit(layout.sdk_git_dir)
    import_lines = sorted({lineno for _submodule, _names, lineno in wanted})
    import_citation = f"{layout.hardware_manager}:{','.join(map(str, import_lines))}" if import_lines else str(layout.hardware_manager)
    compared = {"wanted_imports": [(s, n) for s, n, _l in wanted], "missing": missing, "sdk_head": head}
    if missing:
        return Check(
            "V-SDK-API",
            FAIL,
            f"the vendored SDK does not export: {missing} (SDK HEAD {head})",
            citation=(
                f"{import_citation} imports vs {layout.actuator_init}, {layout.controllers_init}, "
                f"{layout.gripper_module}; commit {LAST_SDK_COMMIT_WITH_ROBOTARM} is the last known SDK commit exporting "
                "RobotArm and ArmEndPos"
            ),
            compared=compared,
            remedy=f"pin the vendored SDK submodule to a commit that exports these names (last known: {LAST_SDK_COMMIT_WITH_ROBOTARM}), or update hardware_manager.py's imports to the SDK's current API",
        )
    return Check(
        "V-SDK-API",
        PASS,
        f"every import hardware_manager.py needs is exported by the SDK (HEAD {head})",
        citation=(
            f"{import_citation} imports vs {layout.actuator_init}, {layout.controllers_init}, "
            f"{layout.gripper_module}; commit {LAST_SDK_COMMIT_WITH_ROBOTARM} is the last known SDK commit exporting "
            "RobotArm and ArmEndPos"
        ),
        compared=compared,
    )


def check_pydeps(driver_pythonpaths: list[str]) -> Check:
    search_dirs = STANDARD_PYTHON_SEARCH_DIRS + list(driver_pythonpaths)
    missing = []
    found = {}
    for pkg in ("motorbridge", "pinocchio"):
        hits = []
        for d in search_dirs:
            dp = Path(d)
            if not dp.is_dir():
                continue
            if (dp / pkg).exists() or list(dp.glob(f"{pkg}*")):
                hits.append(d)
        found[pkg] = hits
        if not hits:
            missing.append(pkg)
    compared = {"search_dirs": search_dirs, "found": found}
    citation_parts = []
    for pkg in ("motorbridge", "pinocchio"):
        hits = found.get(pkg) or []
        base = Path(hits[0]) if hits else Path(search_dirs[0] if search_dirs else ".")
        citation_parts.append(f"{base / pkg}:1")
    citation = ", ".join(citation_parts)
    if missing:
        return Check(
            "V-PYDEPS",
            FAIL,
            f"not importable on the driver interpreter's search path: {missing}",
            citation=citation,
            compared=compared,
            remedy="install motorbridge and pinocchio into the interpreter the driver actually runs under (/usr/bin/python3), or pass --driver-pythonpath",
        )
    return Check(
        "V-PYDEPS",
        PASS,
        "motorbridge and pinocchio are both importable on the driver interpreter's search path",
        citation=citation,
        compared=compared,
    )


def check_qos(executor_path: Path) -> Check:
    src = _read_text(executor_path)
    matches = list(re.finditer(
        r"create_subscription\(\s*JointState\s*,\s*[^,]+,\s*[^,]+,\s*([^\)]+)\)", src
    ))
    if not matches:
        return Check("V-QOS", WARN, f"no JointState subscription found in {executor_path}", citation=str(executor_path))

    calls = [m.group(1).strip() for m in matches]
    lines = sorted({_mline(src, m) for m in matches})
    citation = f"{executor_path}:{','.join(map(str, lines))} vs the vendor driver's qos_profile_sensor_data joint_states publisher"

    unresolved = []
    for qos_expr in calls:
        if qos_expr == "qos_profile_sensor_data":
            continue
        if re.search(rf"\b{re.escape(qos_expr)}\s*=\s*qos_profile_sensor_data\b", src):
            continue
        unresolved.append(qos_expr)

    compared = {"subscription_qos_exprs": calls, "not_confirmed_best_effort": unresolved}
    if unresolved:
        return Check(
            "V-QOS",
            FAIL,
            f"joint_states subscription QoS not confirmed BEST_EFFORT/sensor-data: {unresolved}",
            citation=citation,
            compared=compared,
            remedy="subscribe to joint_states with rclpy.qos.qos_profile_sensor_data (BEST_EFFORT), matching the vendor driver's publisher",
        )
    return Check("V-QOS", PASS, "joint_states subscription(s) use qos_profile_sensor_data (BEST_EFFORT)", citation=citation, compared=compared)


def check_profile(layout: VendorLayout, execution_cfg: dict) -> Check:
    controller_src = _read_text(layout.rebotarm_controller)
    actions_src = _read_text(layout.ros_actions)
    publishers_src = _read_text(layout.ros_publishers)

    class_match = re.search(r"class\s+(\w+)\(Node\)", controller_src)
    node_name = class_match.group(1) if class_match else None
    ns_match = re.search(r'declare_parameter\("arm_namespace",\s*"([^"]*)"\)', controller_src)
    namespace = ns_match.group(1) if ns_match else None

    expected_action = f"/{namespace}/follow_joint_trajectory" if namespace else None
    expected_joint_states = f"/{namespace}/joint_states" if namespace else None
    expected_arm_status = f"/{namespace}/arm_status" if namespace else None

    vendor_profile = (execution_cfg.get("driver_profiles") or {}).get("vendor") or {}
    action_idx = actions_src.find('f"/{namespace}/follow_joint_trajectory"')
    joint_states_idx = publishers_src.find('f"/{namespace}/joint_states"')
    arm_status_idx = publishers_src.find('f"/{namespace}/arm_status"')
    compared = {
        "driver_node_name": node_name,
        "driver_namespace": namespace,
        "expected_action": expected_action,
        "expected_joint_states_topic": expected_joint_states,
        "expected_arm_status_topic": expected_arm_status,
        "execution_yaml.vendor": vendor_profile,
        "action_template_in_ros_actions": action_idx != -1,
        "joint_states_template_in_ros_publishers": joint_states_idx != -1,
        "arm_status_template_in_ros_publishers": arm_status_idx != -1,
    }

    controller_lines = sorted({ln for ln in (_mline(controller_src, class_match), _mline(controller_src, ns_match)) if ln})
    publishers_lines = sorted({_line_no(publishers_src, i) for i in (joint_states_idx, arm_status_idx) if i != -1})
    citation_parts = []
    if controller_lines:
        citation_parts.append(f"{layout.rebotarm_controller}:{','.join(map(str, controller_lines))}")
    if action_idx != -1:
        citation_parts.append(f"{layout.ros_actions}:{_line_no(actions_src, action_idx)}")
    if publishers_lines:
        citation_parts.append(f"{layout.ros_publishers}:{','.join(map(str, publishers_lines))}")
    citation = (", ".join(citation_parts) or f"{layout.rebotarm_controller}, {layout.ros_actions}, {layout.ros_publishers}") + " vs execution.yaml driver_profiles.vendor"

    ok = (
        node_name == vendor_profile.get("expected_node")
        and expected_action == vendor_profile.get("action")
        and expected_joint_states == vendor_profile.get("joint_states_topic")
        and expected_arm_status == vendor_profile.get("arm_status_topic")
    )
    return Check(
        "V-PROFILE",
        PASS if ok else FAIL,
        "execution.yaml's vendor profile matches the driver's node name/namespace/topics"
        if ok
        else "execution.yaml's vendor profile does not match the driver",
        citation=citation,
        compared=compared,
        remedy="" if ok else "fix config/motion/execution.yaml's vendor profile (action/joint_states_topic/arm_status_topic/expected_node) to match the driver",
    )


def check_arbitration(layout: VendorLayout) -> Check:
    driver_params_text = _read_text(layout.driver_params_yaml)
    driver_params = (_read_yaml(layout.driver_params_yaml) or {}).get("reBotArmController", {}).get("ros__parameters", {})
    arbitration = driver_params.get("cmd_arbitration")
    status = PASS if arbitration == "reject" else FAIL
    return Check(
        "V-ARBITRATION",
        status,
        f"cmd_arbitration = {arbitration!r}",
        citation=f"{_cite(layout.driver_params_yaml, driver_params_text, 'cmd_arbitration')} reBotArmController.ros__parameters.cmd_arbitration",
        compared={"cmd_arbitration": arbitration},
        remedy="" if status == PASS else "set cmd_arbitration: reject in driver_params.yaml",
    )


def check_urdf(layout: VendorLayout) -> Check:
    launch_src = _read_text(layout.bringup_launch)
    m = re.search(r'"urdf",\s*"([^"]+)"', launch_src)
    urdf_name = m.group(1) if m else None
    gripperless = bool(urdf_name) and "fixend" in urdf_name
    citation = f"{layout.bringup_launch}:{_mline(launch_src, m)} urdf_file, docs/motion/ROBOT_MODEL.md §3" if m else f"{layout.bringup_launch} urdf_file, docs/motion/ROBOT_MODEL.md §3"
    return Check(
        "V-URDF",
        WARN if gripperless else PASS,
        f"bringup.launch.py publishes robot_description from {urdf_name!r}"
        + (" (gripper-less)" if gripperless else ""),
        citation=citation,
        compared={"urdf_file": urdf_name, "gripperless": gripperless},
        remedy="repoint bringup.launch.py's robot_description at the gripper URDF if the real driver session needs the gripper in TF" if gripperless else "",
    )


# --------------------------------------------------------------------------------------
# orchestration
# --------------------------------------------------------------------------------------


def run_checks(
    rebot_ws: Path,
    facts_path: Path,
    limits_path: Path,
    execution_config_path: Path,
    executor_path: Path,
    driver_pythonpaths: list[str],
    kp_cap: float,
    kd_cap: float,
) -> list[Check]:
    layout = VendorLayout(rebot_ws)
    facts = _read_yaml(facts_path) or {}
    limits = _read_yaml(limits_path) or {}
    execution_cfg = _read_yaml(execution_config_path) or {}

    checks = [
        check_port(layout, facts),
        check_baud(layout, facts),
        check_ids(layout, facts),
        check_order(layout),
        check_direction(layout, facts),
        check_zero(layout, facts),
        check_gripper_sign(layout, facts),
        check_gripper_range(layout, facts),
        check_gripper_width(layout, limits),
        check_gripper_units(layout),
        check_safe_park(layout, limits),
        check_startup(layout),
        check_gains(layout, limits, kp_cap, kd_cap),
        check_sdk_api(layout),
        check_pydeps(driver_pythonpaths),
        check_qos(executor_path),
        check_profile(layout, execution_cfg),
        check_arbitration(layout),
        check_urdf(layout),
    ]
    return checks


def _format_text(checks: list[Check]) -> str:
    lines = [f"{'id':<16}{'status':<6}message", "-" * 80]
    for c in checks:
        lines.append(f"{c.id:<16}{c.status:<6}{c.message}")
        if c.citation:
            lines.append(f"{'':<22}citation: {c.citation}")
        if c.remedy:
            lines.append(f"{'':<22}remedy:   {c.remedy}")
    return "\n".join(lines)


def _setup_logging(verbose: bool, quiet: bool) -> logging.Logger:
    level = logging.DEBUG if verbose else (logging.WARNING if quiet else logging.INFO)
    logger = logging.getLogger(TOOL_NAME)
    logger.setLevel(level)
    logger.handlers.clear()
    logger.propagate = False
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)-7s %(message)s"))
    logger.addHandler(handler)
    return logger


def _write_logs(log_dir: Path, summary: dict[str, Any], human_lines: list[str]) -> None:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    (log_dir / f"{TOOL_NAME}_{stamp}.log").write_text("\n".join(human_lines) + "\n", encoding="utf-8")
    payload = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    (log_dir / f"{TOOL_NAME}_{stamp}.json").write_text(payload, encoding="utf-8")
    (log_dir / f"{TOOL_NAME}_latest.json").write_text(payload, encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--rebot-ws", type=Path, default=Path("~/rebot_ws").expanduser())
    p.add_argument(
        "--facts", type=Path, default=REPO_ROOT / "evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml"
    )
    p.add_argument("--limits-file", type=Path, default=REPO_ROOT / "config/robot/b601_dm_limits.yaml")
    p.add_argument("--execution-config", type=Path, default=REPO_ROOT / "config/motion/execution.yaml")
    p.add_argument(
        "--executor-file",
        type=Path,
        default=REPO_ROOT / "ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py",
    )
    p.add_argument("--driver-pythonpath", action="append", default=[], help="extra dir(s) to search for motorbridge/pinocchio")
    p.add_argument("--kp-cap", type=float, default=10.0, help="bring-up MIT kp cap (operator statement, 2026-10-10)")
    p.add_argument("--kd-cap", type=float, default=1.0, help="bring-up MIT kd cap (operator statement, 2026-10-10)")
    p.add_argument("--format", choices=["text", "json"], default="text")
    p.add_argument("--json-out", type=Path, default=None)
    p.add_argument("--root", type=Path, default=None, help="project root override (for §0.3 parity; logs/ is resolved relative to it)")
    p.add_argument("--config", type=Path, default=None, help="unused, accepted for §0.3 parity")
    p.add_argument("-v", "--verbose", action="store_true")
    p.add_argument("-q", "--quiet", action="store_true")
    p.add_argument("--dry-run", action="store_true", help="this tool never writes vendor state regardless; accepted for parity")
    p.add_argument("--skip-existing", action="store_true", help="unused, accepted for §0.3 parity")
    return p


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logger = _setup_logging(args.verbose, args.quiet)
    root = args.root or REPO_ROOT
    started = time.time()

    for required in (args.facts, args.limits_file, args.execution_config, args.executor_file):
        if not required.is_file():
            logger.error("required input file missing: %s", required)
            return EXIT_USAGE

    try:
        checks = run_checks(
            args.rebot_ws,
            args.facts,
            args.limits_file,
            args.execution_config,
            args.executor_file,
            args.driver_pythonpath,
            args.kp_cap,
            args.kd_cap,
        )
    except InputMissing as exc:
        logger.error("required vendor file missing: %s", exc)
        return EXIT_USAGE
    except Exception:
        logger.exception("unexpected error running checks")
        return EXIT_RUNTIME

    fail_count = sum(1 for c in checks if c.status == FAIL)
    warn_count = sum(1 for c in checks if c.status == WARN)
    pass_count = sum(1 for c in checks if c.status == PASS)

    if args.format == "text" and not args.quiet:
        print(_format_text(checks))

    report = {
        "tool": TOOL_NAME,
        "schema_version": 1,
        "rebot_ws": str(args.rebot_ws),
        "facts": str(args.facts),
        "checks": [c.to_dict() for c in checks],
        "counts": {"pass": pass_count, "warn": warn_count, "fail": fail_count},
    }
    if args.format == "json":
        print(json.dumps(report, indent=2))

    if args.json_out:
        args.json_out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    finished = time.time()
    summary = {
        "tool": TOOL_NAME,
        "schema_version": 1,
        "started_utc": datetime.fromtimestamp(started, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "finished_utc": datetime.fromtimestamp(finished, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "duration_s": round(finished - started, 3),
        "status": "ok" if fail_count == 0 else "precondition",
        "exit_code": EXIT_OK if fail_count == 0 else EXIT_NOT_READY,
        "counts": report["counts"],
        "errors": [],
    }
    _write_logs(root / "logs", summary, _format_text(checks).splitlines())

    return EXIT_OK if fail_count == 0 else EXIT_NOT_READY


if __name__ == "__main__":
    raise SystemExit(main())
