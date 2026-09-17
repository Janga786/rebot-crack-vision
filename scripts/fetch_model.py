"""scripts/fetch_model.py — OpenCrack nnU-Net model acquisition. Built by TC-004.

Downloads the `fadeevla/opencrack-nnunet` Hugging Face snapshot into `models/opencrack-nnunet/`,
validates that the files nnU-Net v2 needs are present with plausible sizes, and writes a sha256
manifest to `config/model_manifest.json` (docs/INTERFACES.md §3.6). Idempotent: a second run with
everything already present re-verifies without re-downloading. This script only acquires and
validates file presence/size/hash — semantic validation of dataset.json/plans.json and the
nnunet/results symlink belong to TC-005.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE, RunSummary, setup_logging

LICENSE_NOTICE = (
    "OpenCrack nnU-Net is licensed CC-BY-4.0 - attribution is required in any published work. "
    "Cite: Fadeev, V. A. (2026), OpenCrack; and Isensee et al. (2021), nnU-Net, Nature Methods 18(2)."
)

CHECKPOINT_MIN_MB = 250
CHECKPOINT_MAX_MB = 300
CHUNK_BYTES = 1024 * 1024


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _required_files(dest: Path, model: dict[str, Any]) -> dict[str, Path]:
    base = dest / model["dataset_name"] / model["trainer_config"]
    return {
        "plans.json": base / "plans.json",
        "dataset.json": base / "dataset.json",
        "checkpoint": base / f"fold_{model['fold']}" / model["checkpoint"],
    }


def _missing_required(required: dict[str, Path]) -> list[str]:
    return [label for label, path in required.items() if not path.is_file()]


def _checkpoint_size_ok(checkpoint: Path) -> tuple[bool, float]:
    size_mb = checkpoint.stat().st_size / (1024 * 1024)
    return CHECKPOINT_MIN_MB <= size_mb <= CHECKPOINT_MAX_MB, size_mb


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(CHUNK_BYTES), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_hidden(rel_parts: tuple[str, ...]) -> bool:
    return any(part.startswith(".") for part in rel_parts)


def _collect_manifest_files(dest: Path) -> list[dict[str, Any]]:
    """Hash every real snapshot file under dest, skipping hub bookkeeping (dotfiles/.cache)."""
    entries: list[dict[str, Any]] = []
    for path in sorted(dest.rglob("*")):
        if not path.is_file():
            continue
        rel = path.relative_to(dest)
        if _is_hidden(rel.parts):
            continue
        entries.append(
            {
                "path": rel.as_posix(),
                "bytes": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
        )
    return entries


def _resolve_revision(repo_id: str, logger: Any) -> str | None:
    try:
        from huggingface_hub import HfApi

        info = HfApi().model_info(repo_id)
        return info.sha
    except Exception as exc:  # noqa: BLE001 - revision is best-effort, never fatal
        logger.warning("could not resolve upstream revision sha: %s: %s", type(exc).__name__, exc)
        return None


def _write_manifest_atomic(manifest_path: Path, data: dict[str, Any]) -> None:
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(dir=str(manifest_path.parent), prefix=".model_manifest_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2)
            fh.write("\n")
        os.replace(tmp_name, manifest_path)
    except Exception:
        Path(tmp_name).unlink(missing_ok=True)
        raise


def _load_existing_manifest(manifest_path: Path) -> dict[str, Any] | None:
    if not manifest_path.is_file():
        return None
    try:
        return json.loads(manifest_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="fetch_model.py",
        description="Download and verify the OpenCrack nnU-Net snapshot (docs/INTERFACES.md §3.6).",
    )
    add_common_args(parser)
    parser.add_argument("--repo-id", default=None, help="HF repo id (default: config/project.yaml model.repo_id)")
    parser.add_argument("--dest", type=Path, default=None, help="download destination (default: paths.models)")
    parser.add_argument("--force", action="store_true", help="re-download even if files are already present")
    parser.add_argument(
        "--verify-only", action="store_true", help="skip download; validate + re-hash against the existing manifest"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"fetch_model: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("fetch_model", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    repo_id = args.repo_id or cfg.model["repo_id"]
    dest = (args.dest or cfg.paths["models"]).resolve()
    manifest_path = cfg.root / "config" / "model_manifest.json"
    required = _required_files(dest, cfg.model)

    logger.info(LICENSE_NOTICE)
    logger.info("repo_id=%s dest=%s", repo_id, dest)

    if args.dry_run:
        missing = _missing_required(required)
        if missing:
            logger.info("dry-run: would download %s (missing: %s)", repo_id, ", ".join(missing))
        else:
            logger.info("dry-run: %s already present at %s; would re-verify only", repo_id, dest)
        return EXIT_OK

    if args.verify_only:
        missing = _missing_required(required)
        if missing:
            logger.error("verify-only: missing required files: %s", ", ".join(missing))
            summary.add_error(f"missing required files: {', '.join(missing)}")
            summary.write(cfg, "fetch_model", "precondition", EXIT_PRECONDITION)
            return EXIT_PRECONDITION

        existing_manifest = _load_existing_manifest(manifest_path)
        if existing_manifest is None:
            logger.error("verify-only: no manifest at %s to verify against; run without --verify-only first", manifest_path)
            summary.add_error(f"manifest missing: {manifest_path}")
            summary.write(cfg, "fetch_model", "precondition", EXIT_PRECONDITION)
            return EXIT_PRECONDITION

        ok, size_mb = _checkpoint_size_ok(required["checkpoint"])
        if not ok:
            logger.error("checkpoint size %.1f MB outside expected %d-%d MB range", size_mb, CHECKPOINT_MIN_MB, CHECKPOINT_MAX_MB)
            summary.add_error(f"checkpoint size {size_mb:.1f} MB out of range")
            summary.write(cfg, "fetch_model", "precondition", EXIT_PRECONDITION)
            return EXIT_PRECONDITION

        manifest_by_path = {entry["path"]: entry for entry in existing_manifest.get("files", [])}
        current_files = _collect_manifest_files(dest)
        mismatches = []
        for entry in current_files:
            recorded = manifest_by_path.get(entry["path"])
            if recorded is None:
                mismatches.append(f"{entry['path']}: not in manifest")
            elif recorded.get("sha256") != entry["sha256"]:
                mismatches.append(f"{entry['path']}: sha256 mismatch (manifest={recorded.get('sha256')}, actual={entry['sha256']})")
        if mismatches:
            for m in mismatches:
                logger.error("verify-only: %s", m)
            summary.errors.extend(mismatches)
            summary.write(cfg, "fetch_model", "precondition", EXIT_PRECONDITION)
            return EXIT_PRECONDITION

        summary.increment("files_verified", len(current_files))
        logger.info("verify-only: %d files match the recorded manifest", len(current_files))
        summary.write(cfg, "fetch_model", "ok", EXIT_OK)
        return EXIT_OK

    need_download = args.force or bool(_missing_required(required))
    if need_download:
        from huggingface_hub import snapshot_download

        logger.info("downloading %s -> %s (force=%s)", repo_id, dest, args.force)
        try:
            snapshot_download(
                repo_id=repo_id,
                local_dir=dest,
                allow_patterns=["Dataset501_OpenCrack/**", "README.md", "reproducibility/**"],
            )
        except Exception as exc:  # noqa: BLE001 - network/API failure, report and stop; no manifest written
            logger.error("download failed: %s: %s", type(exc).__name__, exc)
            summary.add_error(f"download failed: {type(exc).__name__}: {exc}")
            summary.write(cfg, "fetch_model", "failed", EXIT_RUNTIME)
            return EXIT_RUNTIME
        summary.increment("downloaded", 1)
    else:
        logger.info("%s already present at %s; skipping download (idempotent)", repo_id, dest)
        summary.increment("skipped_download", 1)

    missing = _missing_required(required)
    if missing:
        logger.error("missing required files after acquisition: %s", ", ".join(missing))
        summary.add_error(f"missing required files: {', '.join(missing)}")
        summary.write(cfg, "fetch_model", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    ok, size_mb = _checkpoint_size_ok(required["checkpoint"])
    logger.info("checkpoint size: %.1f MB", size_mb)
    if not ok:
        logger.error(
            "checkpoint size %.1f MB outside expected %d-%d MB range (download truncated or upstream changed)",
            size_mb, CHECKPOINT_MIN_MB, CHECKPOINT_MAX_MB,
        )
        summary.add_error(f"checkpoint size {size_mb:.1f} MB out of range")
        summary.write(cfg, "fetch_model", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    revision = _resolve_revision(repo_id, logger)
    files = _collect_manifest_files(dest)
    manifest = {
        "schema_version": 1,
        "repo_id": repo_id,
        "downloaded_utc": _utc_iso(),
        "revision": revision,
        "files": files,
    }
    _write_manifest_atomic(manifest_path, manifest)
    summary.increment("files_hashed", len(files))
    logger.info("wrote manifest: %s (%d files, revision=%s)", manifest_path, len(files), revision)

    for entry in files:
        logger.info("  %s  %d bytes  sha256=%s", entry["path"], entry["bytes"], entry["sha256"])

    summary.write(cfg, "fetch_model", "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
