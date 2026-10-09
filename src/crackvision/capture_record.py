"""crackvision.capture_record — eye-in-hand capture record library + CLI (GEOM-08.4).

Implements docs/INTERFACES.md §9 (`crackvision.capture_3d/1`), the on-disk record §8.4 requires
for any capture whose pixels get lifted to `base_link`. Loads/validates existing records with
every §9.4 refusal rule made explicit, and assembles new records from a §3.10 D405 per-frame
metadata JSON plus a joint-state JSON.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from crackvision import kinematics, naming
from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.geometry import Intrinsics
from crackvision.kinematics import KinematicsError
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)

SCHEMA = "crackvision.capture_3d/1"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
DEFAULT_MAX_SKEW_S = 0.1


class CaptureRecordError(Exception):
    """A §9 capture record fails to load, or violates a §9.4 refusal rule."""


def capture_record_path(root: Path, case_id: str) -> Path:
    return Path(root) / "data" / "captures" / f"{case_id}_capture.json"


@dataclass
class CaptureRecord:
    schema: str
    case_id: str
    synthetic: bool
    image_hw: tuple[int, int]
    color_intrinsics: Intrinsics
    depth_file: str
    depth_scale_m_per_unit: float
    depth_aligned_to: str
    source_kind: str
    source_ref: str
    source_frame_index: int | None
    capture_stamp_ns: int
    clock: str
    joint_names: list[str]
    positions_rad: np.ndarray
    robot_stamp_ns: int
    robot_stamp_source: str
    kinematic_model_path: str
    kinematic_model_sha256: str
    end_effector_path: str
    end_effector_sha256: str
    end_effector_wrist_camera_value_status: str
    end_effector_tool_value_status: str
    T_camera_link_camera_color_optical_frame: np.ndarray
    optical_source: str
    end_effector_changed: bool = False
    end_effector_sha256_now: str | None = None


def _color_intrinsics_from_flat(d: dict) -> Intrinsics:
    """§9's flat `color_intrinsics` block, not §3.10's nested `{width,height,intrinsics:{...}}`."""
    coeffs = tuple(float(c) for c in d["coeffs"])
    if len(coeffs) != 5:
        raise CaptureRecordError(f"color_intrinsics.coeffs must have 5 values, got {len(coeffs)}")
    return Intrinsics(
        width=int(d["width"]),
        height=int(d["height"]),
        fx=float(d["fx"]),
        fy=float(d["fy"]),
        ppx=float(d["ppx"]),
        ppy=float(d["ppy"]),
        model=str(d["model"]),
        coeffs=coeffs,
    )


def _png_size(path: Path) -> tuple[int, int]:
    """`(height, width)` of a PNG, reading only the header (no pixel decode)."""
    with Image.open(path) as img:
        width, height = img.size
    return (height, width)


def _check_image_dimension_consistency(doc: dict, root: Path, image_hw: tuple[int, int]) -> None:
    """§9.4/§0.5: `image.height`/`image.width` must match every sibling artefact that exists.

    Each artefact is checked only when it is actually present for this case (a record may predate
    some of them) — but any mismatch among the ones that do exist is a hard refusal.
    """
    case_id = doc.get("case_id")

    depth_rel = doc.get("depth", {}).get("file")
    if depth_rel:
        depth_path = root / depth_rel
        if depth_path.is_file():
            actual = _png_size(depth_path)
            if actual != image_hw:
                raise CaptureRecordError(
                    f"image_hw {image_hw} does not match the aligned depth PNG {depth_path} "
                    f"dimensions {actual} (§0.5 frame invariant)"
                )

    source = doc.get("source", {}) or {}
    if source.get("kind") == "d405_metadata":
        ref_rel = source.get("ref")
        if ref_rel:
            ref_path = root / ref_rel
            if ref_path.is_file():
                try:
                    meta = json.loads(ref_path.read_text(encoding="utf-8"))
                except json.JSONDecodeError:
                    meta = {}
                color_rel = (meta.get("files") or {}).get("color")
                if color_rel:
                    color_path = root / color_rel
                    if color_path.is_file():
                        actual = _png_size(color_path)
                        if actual != image_hw:
                            raise CaptureRecordError(
                                f"image_hw {image_hw} does not match the colour image "
                                f"{color_path} dimensions {actual} (§0.5 frame invariant)"
                            )

    if case_id:
        mask_path = root / naming.mask_path(case_id)
        if mask_path.is_file():
            actual = _png_size(mask_path)
            if actual != image_hw:
                raise CaptureRecordError(
                    f"image_hw {image_hw} does not match the mask {mask_path} dimensions "
                    f"{actual} (§0.5 frame invariant)"
                )

        paths_json_path = root / "data" / "paths" / f"{case_id}_paths.json"
        if paths_json_path.is_file():
            try:
                paths_doc = json.loads(paths_json_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                paths_doc = {}
            if "image_height" in paths_doc and "image_width" in paths_doc:
                actual = (int(paths_doc["image_height"]), int(paths_doc["image_width"]))
                if actual != image_hw:
                    raise CaptureRecordError(
                        f"image_hw {image_hw} does not match {paths_json_path}'s "
                        f"image_height/image_width {actual} (§0.5 frame invariant)"
                    )


def load_capture_record(
    path: Path | str,
    *,
    root: Path,
    end_effector: kinematics.EndEffector,
    max_skew_s: float = DEFAULT_MAX_SKEW_S,
    accept_end_effector_change: bool = False,
) -> CaptureRecord:
    """Load and validate a §9 record, applying every §9.4 refusal rule.

    Raises `CaptureRecordError` on any violation, including the "2D only" case of a missing
    `robot` block (§8.4).
    """
    path = Path(path)
    root = Path(root)
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CaptureRecordError(f"could not read capture record {path}: {exc}") from exc

    if doc.get("schema") != SCHEMA:
        raise CaptureRecordError(f"unexpected schema {doc.get('schema')!r}, expected {SCHEMA!r}")

    robot = doc.get("robot")
    if not robot:
        raise CaptureRecordError(
            "capture record has no 'robot' block: valid for 2D perception only (§8.4)"
        )

    joint_names = robot.get("joint_names")
    if joint_names != JOINT_NAMES:
        raise CaptureRecordError(
            f"robot.joint_names must be exactly {JOINT_NAMES}, got {joint_names!r}"
        )

    positions_rad = robot.get("positions_rad")
    if not isinstance(positions_rad, list) or len(positions_rad) != 6:
        raise CaptureRecordError(
            f"robot.positions_rad must have exactly 6 values, got {positions_rad!r}"
        )
    positions_arr = np.asarray([float(v) for v in positions_rad], dtype=np.float64)

    kinematic_model = robot.get("kinematic_model") or {}
    urdf_path = root / kinematic_model.get("path", "")
    try:
        chain = kinematics.load_chain(urdf_path=urdf_path)
    except (OSError, KinematicsError) as exc:
        raise CaptureRecordError(f"could not load kinematic_model {urdf_path}: {exc}") from exc

    if chain.urdf_sha256 != kinematic_model.get("sha256"):
        raise CaptureRecordError(
            f"robot.kinematic_model.sha256 {kinematic_model.get('sha256')!r} does not match "
            f"the current sha256 of {urdf_path} ({chain.urdf_sha256!r})"
        )

    try:
        chain.fk(positions_arr)
    except KinematicsError as exc:
        raise CaptureRecordError(f"robot.positions_rad rejected by FK: {exc}") from exc

    end_effector_doc = doc.get("end_effector") or {}
    sha_at_capture = end_effector_doc.get("sha256")
    if not sha_at_capture:
        raise CaptureRecordError("end_effector.sha256 is missing")

    end_effector_changed = False
    sha_now = None
    if sha_at_capture != end_effector.sha256:
        if not accept_end_effector_change:
            raise CaptureRecordError(
                f"end_effector.sha256 {sha_at_capture!r} at capture time does not match the "
                f"current config/robot/end_effector.yaml sha256 {end_effector.sha256!r} "
                "(pass accept_end_effector_change to override)"
            )
        end_effector_changed = True
        sha_now = end_effector.sha256

    capture_stamp_ns = int(doc["capture_stamp_ns"])
    robot_stamp_ns = int(robot["stamp_ns"])
    skew_s = abs(capture_stamp_ns - robot_stamp_ns) / 1e9
    if skew_s > max_skew_s:
        raise CaptureRecordError(
            f"capture_stamp_ns and robot.stamp_ns differ by {skew_s:.3f}s, "
            f"exceeding max_skew_s={max_skew_s}s (the arm must be stationary at capture)"
        )

    image_hw = (int(doc["image"]["height"]), int(doc["image"]["width"]))
    _check_image_dimension_consistency(doc, root, image_hw)

    case_map_path = root / "data" / "case_map.json"
    if case_map_path.is_file():
        try:
            case_map = json.loads(case_map_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise CaptureRecordError(f"malformed {case_map_path}: {exc}") from exc
        for entry in case_map.get("cases", []):
            if entry.get("case_id") == doc.get("case_id") and entry.get("downscaled"):
                raise CaptureRecordError(
                    f"case {doc.get('case_id')!r} is downscaled (case_map.json downscaled: true): "
                    "the §0.5 pixel-index guarantee no longer holds"
                )

    image = doc["image"]
    optical = doc["camera_optical"]
    xyz = optical["T_camera_link_camera_color_optical_frame"]["xyz_m"]
    quat = optical["T_camera_link_camera_color_optical_frame"]["quat_xyzw"]
    T_optical = kinematics.transform_from_xyz_quat(xyz, quat)

    return CaptureRecord(
        schema=doc["schema"],
        case_id=doc["case_id"],
        synthetic=bool(doc["synthetic"]),
        image_hw=(int(image["height"]), int(image["width"])),
        color_intrinsics=_color_intrinsics_from_flat(doc["color_intrinsics"]),
        depth_file=doc["depth"]["file"],
        depth_scale_m_per_unit=float(doc["depth"]["depth_scale_m_per_unit"]),
        depth_aligned_to=doc["depth"]["aligned_to"],
        source_kind=doc["source"]["kind"],
        source_ref=doc["source"]["ref"],
        source_frame_index=doc["source"].get("frame_index"),
        capture_stamp_ns=capture_stamp_ns,
        clock=doc["clock"],
        joint_names=joint_names,
        positions_rad=positions_arr,
        robot_stamp_ns=robot_stamp_ns,
        robot_stamp_source=robot["stamp_source"],
        kinematic_model_path=kinematic_model["path"],
        kinematic_model_sha256=kinematic_model["sha256"],
        end_effector_path=end_effector_doc["path"],
        end_effector_sha256=sha_at_capture,
        end_effector_wrist_camera_value_status=end_effector_doc["wrist_camera_value_status"],
        end_effector_tool_value_status=end_effector_doc["tool_value_status"],
        T_camera_link_camera_color_optical_frame=T_optical,
        optical_source=optical["source"],
        end_effector_changed=end_effector_changed,
        end_effector_sha256_now=sha_now,
    )


def load_depth(record: CaptureRecord, root: Path) -> np.ndarray:
    """Load `record.depth_file`'s `uint16` z16 PNG; raises if its shape != `record.image_hw`."""
    path = Path(root) / record.depth_file
    with Image.open(path) as img:
        img.load()
        arr = np.array(img)
    if arr.dtype != np.uint16:
        arr = arr.astype(np.uint16)
    if arr.shape != record.image_hw:
        raise CaptureRecordError(
            f"depth file {path} has shape {arr.shape}, expected image_hw {record.image_hw}"
        )
    return arr


def T_base_link_optical(
    record: CaptureRecord, chain: kinematics.KinematicChain, end_effector: kinematics.EndEffector
) -> np.ndarray:
    """`FK(q) · T_gripper_link_camera_link · T_camera_link_camera_color_optical_frame` (§9.2)."""
    T_fk = chain.fk(record.positions_rad)
    return T_fk @ end_effector.T_gripper_link_camera_link @ record.T_camera_link_camera_color_optical_frame


def build_record(
    *,
    case_id: str,
    synthetic: bool,
    image_height: int,
    image_width: int,
    color_intrinsics: dict,
    depth_file: str,
    depth_scale_m_per_unit: float,
    depth_aligned_to: str,
    source_kind: str,
    source_ref: str,
    source_frame_index: int | None,
    capture_stamp_ns: int,
    joint_names: list[str],
    positions_rad: list[float],
    robot_stamp_ns: int,
    robot_stamp_source: str,
    kinematic_model_path: str,
    kinematic_model_sha256: str,
    end_effector_path: str,
    end_effector_sha256: str,
    wrist_camera_value_status: str,
    tool_value_status: str,
    optical_xyz_m: list[float],
    optical_quat_xyzw: list[float],
    optical_source: str,
) -> dict[str, Any]:
    """Pure assembly of a §9 record dict; no filesystem or validation side effects."""
    return {
        "schema": SCHEMA,
        "case_id": case_id,
        "synthetic": bool(synthetic),
        "image": {"height": int(image_height), "width": int(image_width)},
        "color_intrinsics": color_intrinsics,
        "depth": {
            "file": depth_file,
            "depth_scale_m_per_unit": float(depth_scale_m_per_unit),
            "aligned_to": depth_aligned_to,
        },
        "source": {
            "kind": source_kind,
            "ref": source_ref,
            "frame_index": source_frame_index,
        },
        "capture_stamp_ns": int(capture_stamp_ns),
        "clock": "utc_epoch_ns",
        "robot": {
            "joint_names": list(joint_names),
            "positions_rad": [float(v) for v in positions_rad],
            "stamp_ns": int(robot_stamp_ns),
            "stamp_source": robot_stamp_source,
            "kinematic_model": {"path": kinematic_model_path, "sha256": kinematic_model_sha256},
        },
        "end_effector": {
            "path": end_effector_path,
            "sha256": end_effector_sha256,
            "wrist_camera_value_status": wrist_camera_value_status,
            "tool_value_status": tool_value_status,
        },
        "camera_optical": {
            "T_camera_link_camera_color_optical_frame": {
                "xyz_m": list(optical_xyz_m),
                "quat_xyzw": list(optical_quat_xyzw),
            },
            "source": optical_source,
        },
    }


def write_record(doc: dict[str, Any], path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")


def _timestamp_utc_to_ns(timestamp_utc: str) -> int:
    text = timestamp_utc.replace("Z", "+00:00")
    dt = datetime.fromisoformat(text)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return int(dt.timestamp() * 1e9)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="capture_record.py",
        description="Assemble/validate eye-in-hand capture records (docs/INTERFACES.md §9).",
    )
    add_common_args(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    assemble = sub.add_parser("assemble", help="assemble a capture record from D405 metadata + joint state")
    assemble.add_argument("--case", required=True, metavar="CASE_ID")
    assemble.add_argument("--d405-metadata", required=True, type=Path, metavar="META.json")
    assemble.add_argument("--joint-state", required=True, type=Path, metavar="JS.json")
    assemble.add_argument(
        "--optical-source", choices=["nominal_d405", "driver_tf"], default="nominal_d405"
    )
    assemble.add_argument("--optical-tf", type=Path, default=None, metavar="TF.json")

    validate = sub.add_parser("validate", help="validate existing capture records")
    validate.add_argument("--cases", nargs="+", required=True, metavar="CASE_ID")
    validate.add_argument("--max-skew-s", type=float, default=DEFAULT_MAX_SKEW_S)
    validate.add_argument("--accept-end-effector-change", action="store_true")

    return parser


def _cmd_assemble(args: argparse.Namespace, cfg: Config, logger) -> int:
    summary = RunSummary()

    try:
        metadata = json.loads(args.d405_metadata.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        logger.error("could not read %s: %s", args.d405_metadata, exc)
        summary.write(cfg, "capture_record", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    try:
        joint_state = json.loads(args.joint_state.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        logger.error("could not read %s: %s", args.joint_state, exc)
        summary.write(cfg, "capture_record", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    if args.optical_source == "driver_tf":
        if args.optical_tf is None:
            logger.error("--optical-source driver_tf requires --optical-tf")
            summary.write(cfg, "capture_record", "failed", EXIT_USAGE)
            return EXIT_USAGE
        tf = json.loads(args.optical_tf.read_text(encoding="utf-8"))
        optical_xyz_m = tf["xyz_m"]
        optical_quat_xyzw = tf["quat_xyzw"]
        optical_source = "driver_tf"
    else:
        logger.warning(
            "no --optical-tf given: using nominal_d405 (ADR-012), not the driver's live TF"
        )
        optical_xyz_m = list(kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME_XYZ)
        _xyz, quat = kinematics.transform_to_xyz_quat(
            kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME
        )
        optical_quat_xyzw = list(quat)
        optical_source = "nominal_d405"

    end_effector = kinematics.load_end_effector()
    urdf_path = kinematics.DEFAULT_URDF
    kinematic_model_path = str(urdf_path.relative_to(cfg.root)) if urdf_path.is_relative_to(cfg.root) else str(urdf_path)

    color = metadata["color"]
    color_intrinsics = {
        "width": color["width"],
        "height": color["height"],
        **color["intrinsics"],
    }

    doc = build_record(
        case_id=args.case,
        synthetic=bool(metadata.get("synthetic", False)),
        image_height=color["height"],
        image_width=color["width"],
        color_intrinsics=color_intrinsics,
        depth_file=metadata["files"]["depth"],
        depth_scale_m_per_unit=metadata["depth_scale_m_per_unit"],
        depth_aligned_to=metadata.get("aligned_to", "color"),
        source_kind="d405_metadata",
        source_ref=str(args.d405_metadata),
        source_frame_index=metadata.get("frame_index"),
        capture_stamp_ns=_timestamp_utc_to_ns(metadata["timestamp_utc"]),
        joint_names=joint_state["joint_names"],
        positions_rad=joint_state["positions_rad"],
        robot_stamp_ns=joint_state["stamp_ns"],
        robot_stamp_source=joint_state["stamp_source"],
        kinematic_model_path=kinematic_model_path,
        kinematic_model_sha256=kinematics.file_sha256(urdf_path),
        end_effector_path="config/robot/end_effector.yaml",
        end_effector_sha256=end_effector.sha256,
        wrist_camera_value_status=end_effector.wrist_camera_value_status,
        tool_value_status=end_effector.tool_value_status,
        optical_xyz_m=optical_xyz_m,
        optical_quat_xyzw=optical_quat_xyzw,
        optical_source=optical_source,
    )

    out_path = capture_record_path(cfg.root, args.case)
    if args.dry_run:
        logger.info("dry-run: would write %s", out_path)
        summary.write(cfg, "capture_record", "ok", EXIT_OK)
        return EXIT_OK

    write_record(doc, out_path)
    logger.info("wrote %s", out_path)
    summary.increment("assembled")
    summary.write(cfg, "capture_record", "ok", EXIT_OK)
    return EXIT_OK


def _cmd_validate(args: argparse.Namespace, cfg: Config, logger) -> int:
    summary = RunSummary()
    end_effector = kinematics.load_end_effector()
    any_failed = False

    if args.dry_run:
        for case_id in args.cases:
            logger.info("dry-run: would validate case %s", case_id)
        summary.write(cfg, "capture_record", "ok", EXIT_OK)
        return EXIT_OK

    for case_id in args.cases:
        path = capture_record_path(cfg.root, case_id)
        if not path.is_file():
            logger.error("case %s: no capture record at %s", case_id, path)
            summary.add_error(f"{case_id}: no capture record at {path}")
            summary.increment("failed")
            any_failed = True
            continue
        try:
            load_capture_record(
                path,
                root=cfg.root,
                end_effector=end_effector,
                max_skew_s=args.max_skew_s,
                accept_end_effector_change=args.accept_end_effector_change,
            )
        except CaptureRecordError as exc:
            logger.error("case %s: REFUSED: %s", case_id, exc)
            summary.add_error(f"{case_id}: {exc}")
            summary.increment("failed")
            any_failed = True
        else:
            logger.info("case %s: OK", case_id)
            summary.increment("ok")

    if any_failed:
        summary.write(cfg, "capture_record", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    summary.write(cfg, "capture_record", "ok", EXIT_OK)
    return EXIT_OK


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"capture_record: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("capture_record", cfg, verbose=args.verbose, quiet=args.quiet)

    if args.command == "assemble":
        try:
            return _cmd_assemble(args, cfg, logger)
        except (KeyError, OSError) as exc:
            logger.error("assemble failed: %s: %s", type(exc).__name__, exc)
            return EXIT_RUNTIME
    elif args.command == "validate":
        return _cmd_validate(args, cfg, logger)
    return EXIT_USAGE


if __name__ == "__main__":
    sys.exit(main())
