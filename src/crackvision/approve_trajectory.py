"""crackvision.approve_trajectory — `approve_trajectory` CLI (docs/INTERFACES.md §18.2, ADR-020).

Re-verifies a `crackvision.trajectory_preview/1` manifest (and everything it names: the trajectory,
the three PNGs, the check report, and the limits/scene/end-effector bindings) directly against the
live files -- never trusting the manifest's own copy of a sha256, the same "never trust the copy"
convention `G-STALE-CONFIG`/`G-LIMITS-HASH` already use (§11.7). Only once every check passes does it
open a controlling terminal directly and ask for a typed confirmation phrase, then writes the
five-key `crackvision.execution_approval/1` file (§11.5).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)
from crackvision.preview_core import (
    PreviewRefusal,
    PreviewUsageError,
    file_sha256,
    load_manifest,
)

TOOL = "approve_trajectory"

APPROVAL_SCHEMA = "crackvision.execution_approval/1"
_APPROVED_PURPOSES = ("crack_task", "view_move")


def _iso_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _resolve(root: Path, path_str: str) -> Path:
    path = Path(path_str)
    return path if path.is_absolute() else Path(root) / path


def _normalize_path(path: Path, root: Path) -> str:
    """§11.2 path convention: repo-relative (POSIX) under `root`, else absolute."""
    resolved = Path(path).resolve()
    try:
        return resolved.relative_to(Path(root).resolve()).as_posix()
    except ValueError:
        return str(resolved)


def verify_for_approval(
    manifest_path: Path | str,
    *,
    root: Path | str,
    limits_path: Path | str,
    scene_config_path: Path | str,
    end_effector_config_path: Path | str,
) -> dict:
    """Re-verifies §18.2 checks 1-6 against live files and returns everything `build_approval` and
    the confirmation prompt need. Raises `PreviewRefusal` carrying every failing reason (joined by
    newlines) rather than stopping at the first one that fails."""
    root = Path(root)
    manifest_path = Path(manifest_path)

    try:
        manifest = load_manifest(manifest_path)
    except (PreviewRefusal, PreviewUsageError) as exc:
        raise PreviewRefusal(f"preview manifest at {manifest_path} does not parse: {exc}") from exc

    errors: list[str] = []
    warnings: list[str] = []

    traj_entry = manifest["trajectory"]
    traj_path = _resolve(root, traj_entry["path"])
    traj_sha: str | None = None
    try:
        traj_sha = file_sha256(traj_path)
    except OSError as exc:
        errors.append(f"trajectory file {traj_entry['path']} cannot be read: {exc}")
    else:
        if traj_sha != traj_entry["sha256"]:
            errors.append(
                f"trajectory file {traj_entry['path']} current sha256 {traj_sha} != "
                f"manifest.trajectory.sha256 {traj_entry['sha256']}"
            )

    for artifact in manifest["artifacts"]:
        art_path = _resolve(root, artifact["path"])
        try:
            art_sha = file_sha256(art_path)
        except OSError as exc:
            errors.append(f"artifact {artifact['kind']} ({artifact['path']}) cannot be read: {exc}")
            continue
        if art_sha != artifact["sha256"]:
            errors.append(
                f"artifact {artifact['kind']} ({artifact['path']}) current sha256 {art_sha} != "
                f"manifest sha256 {artifact['sha256']}"
            )

    check_entry = manifest["check_report"]
    check_path = _resolve(root, check_entry["path"])
    try:
        check_bytes = check_path.read_bytes()
    except OSError as exc:
        errors.append(f"check report {check_entry['path']} cannot be read: {exc}")
    else:
        check_doc: Any = None
        try:
            check_text = check_bytes.decode("utf-8")
        except UnicodeDecodeError as exc:
            errors.append(f"check report {check_entry['path']} is not valid UTF-8: {exc}")
        else:
            try:
                check_doc = json.loads(check_text)
            except json.JSONDecodeError as exc:
                errors.append(f"check report {check_entry['path']} is not valid JSON: {exc}")
                check_doc = None
            if check_doc is not None and not isinstance(check_doc, dict):
                errors.append(
                    f"check report {check_entry['path']} must be a JSON object, got "
                    f"{type(check_doc).__name__}"
                )
                check_doc = None
        if check_doc is not None:
            check_sha = file_sha256(check_path)
            if check_sha != check_entry["sha256"]:
                errors.append(
                    f"check report {check_entry['path']} current sha256 {check_sha} != "
                    f"manifest sha256 {check_entry['sha256']}"
                )
            if traj_sha is not None and check_doc.get("trajectory_sha256") != traj_sha:
                errors.append(
                    f"check report {check_entry['path']} trajectory_sha256 "
                    f"{check_doc.get('trajectory_sha256')!r} != current trajectory sha256 {traj_sha!r}"
                )
            failed = [
                c.get("check", "<unnamed check>")
                for c in (check_doc.get("checks") or [])
                if isinstance(c, dict) and c.get("outcome") == "fail"
            ]
            if failed:
                errors.append(f"check report {check_entry['path']} has failing check(s): {', '.join(failed)}")

    purpose = traj_entry["purpose"]
    if purpose not in _APPROVED_PURPOSES:
        errors.append(
            f"trajectory purpose is {purpose!r}; only {_APPROVED_PURPOSES!r} trajectories can be approved"
        )

    binding_args = {
        "limits_file_sha256": limits_path,
        "scene_config_sha256": scene_config_path,
        "end_effector_config_sha256": end_effector_config_path,
    }
    for key, cfg_path in binding_args.items():
        recorded = manifest["bindings"][key]
        if recorded is None:
            warnings.append(
                f"bindings.{key} is null; G-STALE-CONFIG/G-LIMITS-HASH will refuse this in real later"
            )
            continue
        try:
            current = file_sha256(cfg_path)
        except OSError as exc:
            errors.append(f"cannot read {cfg_path} to verify bindings.{key}: {exc}")
            continue
        if current != recorded:
            errors.append(
                f"bindings.{key} recorded {recorded} != current {cfg_path} sha256 {current} (stale binding)"
            )

    if errors:
        raise PreviewRefusal("\n".join(errors))

    manifest_sha256 = file_sha256(manifest_path)
    manifest_norm_path = _normalize_path(manifest_path, root)

    assert traj_sha is not None
    return {
        "manifest": manifest,
        "manifest_path": manifest_norm_path,
        "manifest_sha256": manifest_sha256,
        "trajectory_sha256": traj_sha,
        "warnings": warnings,
    }


def build_approval(verified: dict, operator: str, now_utc: str) -> dict:
    """Exactly the five §11.5 keys, with `preview_artifacts[]` = the manifest itself followed by
    each of its three artefacts, in the manifest's own order."""
    manifest = verified["manifest"]
    preview_artifacts = [{"path": verified["manifest_path"], "sha256": verified["manifest_sha256"]}]
    preview_artifacts.extend({"path": a["path"], "sha256": a["sha256"]} for a in manifest["artifacts"])
    return {
        "schema": APPROVAL_SCHEMA,
        "trajectory_sha256": verified["trajectory_sha256"],
        "approved_by": operator,
        "approved_utc": now_utc,
        "preview_artifacts": preview_artifacts,
    }


def _open_tty():
    return open("/dev/tty", "r+", encoding="utf-8")


def make_tty_reader(
    *, trajectory_path: str, purpose: str, duration_scaled_s: float, manifest_path: str
) -> Callable[[], str]:
    """Builds the default `confirm` reader: opens a controlling terminal directly, prints the
    trajectory path, purpose, duration at the preview speed scale and the manifest path, then
    reads one line. An `OSError` while opening means there is no controlling terminal."""

    def reader() -> str:
        tty = _open_tty()
        try:
            tty.write(f"Trajectory: {trajectory_path}\n")
            tty.write(f"Purpose: {purpose}\n")
            tty.write(f"Duration at preview speed scale: {duration_scaled_s:.3f} s\n")
            tty.write(f"Manifest: {manifest_path}\n")
            tty.write("Type the confirmation phrase: ")
            tty.flush()
            return tty.readline()
        finally:
            tty.close()

    return reader


def confirm(sha8: str, reader: Callable[[], str]) -> bool:
    """Reads one line from `reader()` and matches it against `APPROVE <sha8>` after `.strip()`.
    An `OSError` from `reader` (no controlling terminal) is a refusal."""
    try:
        line = reader()
    except OSError:
        return False
    return line.strip() == f"APPROVE {sha8}"


# ---------------------------------------------------------------------------
# CLI (§0, §18.2)
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="approve_trajectory.py",
        description="Re-verify a trajectory preview and write a crackvision.execution_approval/1 "
        "file (docs/INTERFACES.md §18.2).",
    )
    add_common_args(parser)
    parser.add_argument("--preview", type=Path, required=True, help="crackvision.trajectory_preview/1 manifest")
    parser.add_argument("--operator", type=str, required=True, help="operator identifier (approved_by)")
    parser.add_argument(
        "--out", type=Path, default=None, help="default data/approvals/{case}_{sha8}_approval.json"
    )
    parser.add_argument("--limits", type=Path, default=None, help="default config/robot/b601_dm_limits.yaml")
    parser.add_argument(
        "--end-effector-config", type=Path, default=None, help="default config/robot/end_effector.yaml"
    )
    parser.add_argument("--scene-config", type=Path, default=None, help="default config/scene/scene.yaml")
    parser.add_argument(
        "--execution-config", type=Path, default=None, help="default config/motion/execution.yaml"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"{TOOL}: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging(TOOL, cfg, verbose=args.verbose, quiet=args.quiet)
    run_summary = RunSummary()

    if not args.operator or not args.operator.strip():
        logger.error("--operator must be a non-empty string")
        run_summary.add_error("--operator must be a non-empty string")
        run_summary.write(cfg, TOOL, "failed", EXIT_USAGE)
        return EXIT_USAGE

    limits_path = args.limits or (cfg.root / "config" / "robot" / "b601_dm_limits.yaml")
    scene_path = args.scene_config or (cfg.root / "config" / "scene" / "scene.yaml")
    end_effector_path = args.end_effector_config or (cfg.root / "config" / "robot" / "end_effector.yaml")
    # Accepted per §18.2's CLI signature; no §18.2 check reads execution.yaml itself.
    _execution_path = args.execution_config or (cfg.root / "config" / "motion" / "execution.yaml")

    try:
        verified = verify_for_approval(
            args.preview,
            root=cfg.root,
            limits_path=limits_path,
            scene_config_path=scene_path,
            end_effector_config_path=end_effector_path,
        )
    except (PreviewRefusal, PreviewUsageError) as exc:
        logger.error("refused:\n%s", exc)
        run_summary.add_error(str(exc))
        if args.dry_run:
            run_summary.write(cfg, TOOL, "ok", EXIT_OK)
            return EXIT_OK
        run_summary.write(cfg, TOOL, "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    for warning in verified["warnings"]:
        logger.warning(warning)

    sha8 = verified["trajectory_sha256"][:8]
    manifest = verified["manifest"]
    case_id = manifest.get("case_id") or "view"
    out_path = args.out or (cfg.root / "data" / "approvals" / f"{case_id}_{sha8}_approval.json")

    if args.dry_run:
        if out_path.is_file():
            logger.warning("would refuse: output already exists: %s", out_path)
        logger.info(
            "dry-run: every check passed; would prompt for 'APPROVE %s' and write %s", sha8, out_path
        )
        run_summary.write(cfg, TOOL, "ok", EXIT_OK)
        return EXIT_OK

    if out_path.is_file():
        logger.error("output already exists, never overwritten: %s", out_path)
        run_summary.add_error(f"output already exists: {out_path}")
        run_summary.write(cfg, TOOL, "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    reader = make_tty_reader(
        trajectory_path=manifest["trajectory"]["path"],
        purpose=manifest["trajectory"]["purpose"],
        duration_scaled_s=manifest["summary"]["duration_scaled_s"],
        manifest_path=verified["manifest_path"],
    )
    if not confirm(sha8, reader):
        logger.error("confirmation refused or no controlling terminal")
        run_summary.add_error("confirmation refused or no controlling terminal")
        run_summary.write(cfg, TOOL, "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    approval = build_approval(verified, args.operator, _iso_now())

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(dir=out_path.parent, prefix=".approve_trajectory_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(json.dumps(approval, indent=2) + "\n")
        if out_path.exists():
            os.unlink(tmp_name)
            logger.error("output already exists, never overwritten: %s", out_path)
            run_summary.add_error(f"output already exists: {out_path}")
            run_summary.write(cfg, TOOL, "precondition", EXIT_PRECONDITION)
            return EXIT_PRECONDITION
        os.rename(tmp_name, out_path)
    except OSError:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)
        raise

    logger.info("approval written to %s (sha8=%s)", out_path, sha8)
    run_summary.write(cfg, TOOL, "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
