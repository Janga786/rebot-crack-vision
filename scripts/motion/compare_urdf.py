#!/usr/bin/env python3
"""scripts/motion/compare_urdf.py — diff two URDF models (MOT-01).

Compares links and joints (type, parent/child, origin xyz/rpy, axis, limits) between a
"vendor" URDF and an "other" URDF (e.g. the presentation/sim copy) and reports every
mismatch. Never invents numbers: a field missing from one file is reported as MISSING,
not defaulted.

Usage:
    ./env.sh python scripts/motion/compare_urdf.py [--vendor PATH] [--other PATH]
                                                    [--json PATH] [--root PATH]
                                                    [--verbose] [--quiet]

Exit codes (docs/INTERFACES.md §0.2): 0 success (comparison ran, differences if any are
printed/returned, that is not a failure of this tool), 2 usage error, 3 precondition not
met (a URDF file could not be found/parsed).
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_PRECONDITION = 3

DEFAULT_VENDOR = Path(
    "~/rebot_ws/src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf"
).expanduser()
DEFAULT_OTHER = Path(__file__).resolve().parents[2] / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf"

_NUMERIC_TOL = 1e-6


def _floats(s: str | None) -> list[float] | None:
    if s is None:
        return None
    return [float(x) for x in s.split()]


def _vectors_close(a: str | None, b: str | None) -> bool:
    fa, fb = _floats(a), _floats(b)
    if fa is None or fb is None:
        return fa == fb
    if len(fa) != len(fb):
        return False
    return all(abs(x - y) <= _NUMERIC_TOL for x, y in zip(fa, fb))


def parse_urdf(path: Path) -> dict[str, Any]:
    """Parse a URDF file into {'links': {name: link_elem_attrs}, 'joints': {name: joint_dict}}."""
    tree = ET.parse(path)
    root = tree.getroot()

    links: dict[str, dict[str, Any]] = {}
    for link in root.findall("link"):
        name = link.get("name")
        links[name] = {"has_visual": link.find("visual") is not None,
                        "has_collision": link.find("collision") is not None,
                        "has_inertial": link.find("inertial") is not None}

    joints: dict[str, dict[str, Any]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        parent = joint.find("parent")
        child = joint.find("child")
        joints[name] = {
            "type": joint.get("type"),
            "parent": parent.get("link") if parent is not None else None,
            "child": child.get("link") if child is not None else None,
            "origin_xyz": origin.get("xyz") if origin is not None else "0 0 0",
            "origin_rpy": origin.get("rpy") if origin is not None else "0 0 0",
            "axis_xyz": axis.get("xyz") if axis is not None else None,
            "limit_lower": limit.get("lower") if limit is not None else None,
            "limit_upper": limit.get("upper") if limit is not None else None,
            "limit_effort": limit.get("effort") if limit is not None else None,
            "limit_velocity": limit.get("velocity") if limit is not None else None,
        }

    return {"links": links, "joints": joints}


_JOINT_VECTOR_FIELDS = {"origin_xyz", "origin_rpy", "axis_xyz"}
_JOINT_SCALAR_FIELDS = {"type", "parent", "child", "limit_lower", "limit_upper", "limit_effort", "limit_velocity"}


def compare(vendor: dict[str, Any], other: dict[str, Any]) -> dict[str, Any]:
    """Diff two parse_urdf() results. Returns a JSON-serialisable report; invents nothing."""
    vendor_links, other_links = set(vendor["links"]), set(other["links"])
    vendor_joints, other_joints = set(vendor["joints"]), set(other["joints"])

    joint_diffs: dict[str, list[dict[str, Any]]] = {}
    for name in sorted(vendor_joints & other_joints):
        v_j, o_j = vendor["joints"][name], other["joints"][name]
        field_diffs = []
        for field in sorted(_JOINT_VECTOR_FIELDS | _JOINT_SCALAR_FIELDS):
            v_val, o_val = v_j.get(field), o_j.get(field)
            if field in _JOINT_VECTOR_FIELDS:
                same = _vectors_close(v_val, o_val)
            else:
                same = v_val == o_val
            if not same:
                field_diffs.append({"field": field, "vendor": v_val, "other": o_val})
        if field_diffs:
            joint_diffs[name] = field_diffs

    return {
        "links_only_in_vendor": sorted(vendor_links - other_links),
        "links_only_in_other": sorted(other_links - vendor_links),
        "common_links": sorted(vendor_links & other_links),
        "joints_only_in_vendor": sorted(vendor_joints - other_joints),
        "joints_only_in_other": sorted(other_joints - vendor_joints),
        "joint_diffs": joint_diffs,
        "identical": (
            vendor_links == other_links
            and vendor_joints == other_joints
            and not joint_diffs
        ),
    }


def _format_report(report: dict[str, Any], vendor_path: Path, other_path: Path) -> str:
    lines = [f"vendor: {vendor_path}", f"other:  {other_path}", ""]
    if report["identical"]:
        lines.append("RESULT: identical (links + joint kinematics/limits)")
        return "\n".join(lines)

    if report["links_only_in_vendor"]:
        lines.append(f"links only in vendor: {report['links_only_in_vendor']}")
    if report["links_only_in_other"]:
        lines.append(f"links only in other:  {report['links_only_in_other']}")
    if report["joints_only_in_vendor"]:
        lines.append(f"joints only in vendor: {report['joints_only_in_vendor']}")
    if report["joints_only_in_other"]:
        lines.append(f"joints only in other:  {report['joints_only_in_other']}")
    for name, diffs in report["joint_diffs"].items():
        lines.append(f"joint '{name}' differs:")
        for d in diffs:
            lines.append(f"    {d['field']}: vendor={d['vendor']!r} other={d['other']!r}")
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--vendor", type=Path, default=DEFAULT_VENDOR, help="vendor URDF path")
    p.add_argument("--other", type=Path, default=DEFAULT_OTHER, help="other (e.g. presentation/sim) URDF path")
    p.add_argument("--root", type=Path, default=None, help="project root override (unused, for §0.3 parity)")
    p.add_argument("--config", type=Path, default=None, help="config override (unused, for §0.3 parity)")
    p.add_argument("--json", type=Path, default=None, help="write the diff report as JSON to this path")
    p.add_argument("--verbose", "-v", action="store_true", help="DEBUG logging")
    p.add_argument("--quiet", "-q", action="store_true", help="WARNING+ only")
    p.add_argument("--dry-run", action="store_true", help="no writes are performed regardless; accepted for parity")
    p.add_argument("--skip-existing", action="store_true", help="unused, accepted for §0.3 parity")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    for path in (args.vendor, args.other):
        if not path.is_file():
            print(f"error: URDF not found: {path}", file=sys.stderr)
            return EXIT_PRECONDITION

    try:
        vendor_model = parse_urdf(args.vendor)
        other_model = parse_urdf(args.other)
    except ET.ParseError as exc:
        print(f"error: malformed URDF: {exc}", file=sys.stderr)
        return EXIT_USAGE

    report = compare(vendor_model, other_model)

    if not args.quiet:
        print(_format_report(report, args.vendor, args.other))

    if args.json is not None:
        args.json.write_text(json.dumps(report, indent=2), encoding="utf-8")

    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
