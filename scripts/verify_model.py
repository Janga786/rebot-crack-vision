"""scripts/verify_model.py — nnU-Net model wiring and semantic validation. Built by TC-005.

Validates that the tree TC-004 downloaded is a *semantically correct* nnU-Net v2 model for this
pipeline (3 RGB channels, binary crack labels, NaturalImage2DIO, a 2d config with 256x256 patches),
sanity-checks the checkpoint without ever unpickling it unsafely, and creates the relative
nnunet/results/<dataset> symlink that makes the `-d 501` fallback path work (docs/INTERFACES.md
§3.7, adr/004, adr/006). This script is read-only with respect to models/ — it diagnoses and wires,
it never edits or repairs the downloaded files.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE, RunSummary, setup_logging

STATUS_PASS = "PASS"
STATUS_WARN = "WARN"
STATUS_FAIL = "FAIL"

EXPECTED_CHANNEL_NAMES = {"0": "R", "1": "G", "2": "B"}
EXPECTED_LABELS = {"background": 0, "crack": 1}
EXPECTED_FILE_ENDING = ".png"
EXPECTED_READER_WRITER = "NaturalImage2DIO"
EXPECTED_PATCH_SIZE = [256, 256]
EXPECTED_PLANS_NAME = "nnUNetPlans"
EXPECTED_NORMALIZATION_SCHEME_COUNT = 3

TORCH_ARCHIVE_MAGIC = b"PK\x03\x04"
CHUNK_BYTES = 1024 * 1024

CORRUPT_HINT = (
    "file may be corrupt (not upstream drift); verify with --check-hashes, and if it mismatches, "
    "re-download via ./env.sh python scripts/fetch_model.py --force (TC-004) — do not edit the file"
)
DRIFT_HINT = "upstream model contract changed — this is a BLOCKER (RISKS.md R-06); do not edit the file to make it pass"


@dataclass
class Check:
    name: str
    status: str
    value: str
    hint: str = ""

    def to_dict(self) -> dict[str, str]:
        payload = {"name": self.name, "status": self.status, "value": self.value}
        if self.hint:
            payload["hint"] = self.hint
        return payload


def _load_json(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    if not path.is_file():
        return None, f"missing: {path}"
    try:
        raw = path.read_text(encoding="utf-8")
    except OSError as exc:
        return None, f"{type(exc).__name__}: {exc}"
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        return None, f"{type(exc).__name__}: {exc}"
    if not isinstance(data, dict):
        return None, f"expected a JSON object at the top level, got {type(data).__name__}"
    return data, None


def _json_load_check(name: str, path: Path, err: str | None) -> Check:
    if err is not None:
        return Check(name, STATUS_FAIL, err, CORRUPT_HINT)
    return Check(name, STATUS_PASS, f"valid JSON ({path.name})")


def _assert_field(name: str, data: dict[str, Any] | None, key: str, expected: Any) -> Check:
    if data is None:
        return Check(name, STATUS_FAIL, "not evaluated (parse failed)", "fix the underlying parse error first")
    actual = data.get(key, "<missing>")
    if actual == expected:
        return Check(name, STATUS_PASS, json.dumps(actual))
    return Check(name, STATUS_FAIL, f"expected {json.dumps(expected)}, got {json.dumps(actual)}", DRIFT_HINT)


def _check_required_files(plans_path: Path, dataset_path: Path, checkpoint_path: Path) -> Check:
    required = {"plans.json": plans_path, "dataset.json": dataset_path, "checkpoint": checkpoint_path}
    missing = [label for label, p in required.items() if not p.is_file()]
    if missing:
        return Check(
            "required_files", STATUS_FAIL, f"missing: {', '.join(missing)}", "run ./env.sh python scripts/fetch_model.py (TC-004)"
        )
    return Check(
        "required_files",
        STATUS_PASS,
        f"plans.json, dataset.json, {checkpoint_path.parent.name}/{checkpoint_path.name} present",
    )


def _check_configuration_present(plans: dict[str, Any] | None, configuration: str) -> Check:
    name = "plans_configuration_present"
    if plans is None:
        return Check(name, STATUS_FAIL, "not evaluated (parse failed)", "fix plans.json first")
    configs = plans.get("configurations")
    if isinstance(configs, dict) and configuration in configs:
        return Check(name, STATUS_PASS, f"configurations contains {configuration!r}")
    observed = sorted(configs.keys()) if isinstance(configs, dict) else type(configs).__name__
    return Check(name, STATUS_FAIL, f"configurations keys: {observed}", DRIFT_HINT)


def _configuration_block(plans: dict[str, Any] | None, configuration: str) -> dict[str, Any]:
    if plans is None:
        return {}
    configs = plans.get("configurations")
    if not isinstance(configs, dict):
        return {}
    block = configs.get(configuration)
    return block if isinstance(block, dict) else {}


def _check_patch_size(plans: dict[str, Any] | None, configuration: str) -> Check:
    name = "plans_patch_size"
    if plans is None:
        return Check(name, STATUS_FAIL, "not evaluated (parse failed)", "fix plans.json first")
    actual = _configuration_block(plans, configuration).get("patch_size")
    if actual == EXPECTED_PATCH_SIZE:
        return Check(name, STATUS_PASS, json.dumps(actual))
    return Check(name, STATUS_FAIL, f"expected {EXPECTED_PATCH_SIZE}, got {actual}", DRIFT_HINT)


def _check_normalization_schemes(plans: dict[str, Any] | None, configuration: str) -> Check:
    name = "plans_normalization_schemes"
    if plans is None:
        return Check(name, STATUS_FAIL, "not evaluated (parse failed)", "fix plans.json first")
    schemes = _configuration_block(plans, configuration).get("normalization_schemes")
    if isinstance(schemes, list) and len(schemes) == EXPECTED_NORMALIZATION_SCHEME_COUNT:
        return Check(name, STATUS_PASS, json.dumps(schemes))
    return Check(name, STATUS_FAIL, f"expected {EXPECTED_NORMALIZATION_SCHEME_COUNT} entries, got {schemes}", DRIFT_HINT)


def _check_checkpoint(checkpoint_path: Path, logger: Any) -> Check:
    name = "checkpoint_loadable"
    if not checkpoint_path.is_file():
        return Check(name, STATUS_FAIL, "missing", "run ./env.sh python scripts/fetch_model.py (TC-004)")

    import torch

    try:
        # weights_only=True is mandatory — never fall back to False, that executes arbitrary
        # pickle code from a downloaded file (docs/INTERFACES.md §3.7.3).
        ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    except Exception as exc:  # noqa: BLE001 - documented fallback path, nnU-Net ckpts carry non-tensor metadata
        logger.warning(
            "checkpoint_loadable: weights_only=True raised %s: %s; falling back to magic-byte check",
            type(exc).__name__,
            exc,
        )
        try:
            with open(checkpoint_path, "rb") as fh:
                magic = fh.read(4)
        except OSError as read_exc:
            return Check(name, STATUS_FAIL, f"cannot read file: {read_exc}", "re-download: fetch_model.py --force")
        if magic != TORCH_ARCHIVE_MAGIC:
            return Check(
                name,
                STATUS_FAIL,
                f"magic bytes {magic!r} != {TORCH_ARCHIVE_MAGIC!r} (not a valid torch archive)",
                "file is corrupt; run ./env.sh python scripts/fetch_model.py --force",
            )
        return Check(
            name,
            STATUS_WARN,
            f"weights_only=True raised {type(exc).__name__} (expected — nnU-Net checkpoints carry non-tensor "
            "metadata); magic bytes confirm a valid ZIP/torch archive",
        )

    if not isinstance(ckpt, dict):
        return Check(name, STATUS_FAIL, f"loaded object is {type(ckpt).__name__}, expected dict", "")
    keys = list(ckpt.keys())
    state_dict = ckpt.get("network_weights")
    if not isinstance(state_dict, dict):
        state_dict = ckpt.get("state_dict") if isinstance(ckpt.get("state_dict"), dict) else None
    detail = f"top-level keys: {keys}"
    if state_dict is not None:
        detail += f"; tensor count: {len(state_dict)}"
    return Check(name, STATUS_PASS, detail)


def _verify_symlink_resolves(link_path: Path, cfg: Config, name: str) -> Check:
    readlink_value = os.readlink(link_path)
    if not readlink_value.startswith(".."):
        return Check(name, STATUS_FAIL, f"symlink is not relative: {readlink_value}", "adr/004 requires a relative symlink")
    probe = link_path / cfg.model["trainer_config"] / "plans.json"
    if not probe.exists():
        return Check(name, STATUS_FAIL, f"{probe} does not resolve through the symlink", "")
    return Check(name, STATUS_PASS, f"{link_path} -> {readlink_value}; resolves to {probe}")


def _check_symlink(cfg: Config, repair: bool, dry_run: bool, logger: Any) -> Check:
    name = "nnunet_results_symlink"
    dataset_name = cfg.model["dataset_name"]
    results_dir = cfg.root / "nnunet" / "results"
    link_path = results_dir / dataset_name
    real_target = (cfg.paths["models"] / dataset_name).resolve()

    if link_path.is_symlink():
        current_rel = os.readlink(link_path)
        resolved = (link_path.parent / current_rel).resolve()
        if resolved == real_target:
            return _verify_symlink_resolves(link_path, cfg, name)
        if resolved.exists():
            return Check(
                name,
                STATUS_FAIL,
                f"symlink points to {current_rel} (resolves to {resolved}), expected a relative path to {real_target}",
                "investigate before touching it; this script never overwrites an unexpected valid symlink",
            )
        if not repair:
            return Check(
                name, STATUS_FAIL, f"broken symlink: {link_path} -> {current_rel}", "re-run with --repair-symlink"
            )
        if dry_run:
            return Check(name, STATUS_WARN, f"dry-run: would repair broken symlink {link_path} -> {current_rel}")
        link_path.unlink()
        logger.info("removed broken symlink %s -> %s (--repair-symlink)", link_path, current_rel)
    elif link_path.exists():
        kind = "directory" if link_path.is_dir() else "file"
        return Check(
            name,
            STATUS_FAIL,
            f"a real {kind} already exists at {link_path}; not deleting it",
            "move or remove it manually, then re-run",
        )

    if dry_run:
        return Check(name, STATUS_WARN, f"dry-run: would create {link_path} -> relative link to {real_target}")

    results_dir.mkdir(parents=True, exist_ok=True)
    rel_target = os.path.relpath(real_target, start=link_path.parent)
    link_path.symlink_to(rel_target)
    logger.info("created symlink %s -> %s", link_path, rel_target)

    return _verify_symlink_resolves(link_path, cfg, name)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(CHUNK_BYTES), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _check_hashes(manifest_path: Path, models_root: Path, logger: Any) -> Check:
    name = "check_hashes"
    if not manifest_path.is_file():
        return Check(name, STATUS_FAIL, f"manifest not found: {manifest_path}", "run ./env.sh python scripts/fetch_model.py (TC-004) first")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return Check(name, STATUS_FAIL, f"cannot read manifest: {type(exc).__name__}: {exc}", "")

    entries = manifest.get("files", [])
    mismatches: list[str] = []
    checked = 0
    for entry in entries:
        rel = entry.get("path", "<unknown>")
        path = models_root / rel
        expected_sha = entry.get("sha256")
        if not path.is_file():
            mismatches.append(f"{rel}: missing on disk")
            continue
        actual_sha = _sha256_file(path)
        checked += 1
        if actual_sha != expected_sha:
            mismatches.append(f"{rel}: sha256 mismatch (manifest={expected_sha}, actual={actual_sha})")

    if mismatches:
        for m in mismatches:
            logger.error("check_hashes: %s", m)
        return Check(
            name,
            STATUS_FAIL,
            "; ".join(mismatches),
            "re-download the affected file(s): ./env.sh python scripts/fetch_model.py --force",
        )
    return Check(name, STATUS_PASS, f"{checked} files match {manifest_path}")


def _print_table(checks: list[Check]) -> None:
    name_width = max(len("CHECK"), *(len(c.name) for c in checks))
    status_width = max(len("STATUS"), 4)
    print(f"{'CHECK':<{name_width}}  {'STATUS':<{status_width}}  VALUE")
    for check in checks:
        print(f"{check.name:<{name_width}}  {check.status:<{status_width}}  {check.value}")
        if check.status != STATUS_PASS and check.hint:
            print(f"   hint: {check.hint}")
    counts = Counter(c.status for c in checks)
    print()
    print(f"{counts.get(STATUS_PASS, 0)} PASS · {counts.get(STATUS_WARN, 0)} WARN · {counts.get(STATUS_FAIL, 0)} FAIL")


def _write_summary(cfg: Config, checks: list[Check], status: str, exit_code: int) -> Path:
    summary = RunSummary()
    for check_status, count in Counter(c.status for c in checks).items():
        summary.increment(check_status.lower(), count)
    for check in checks:
        if check.status == STATUS_FAIL:
            summary.add_error(f"{check.name}: {check.value}")

    latest_path = summary.write(cfg, "verify_model", status, exit_code)

    # RunSummary's schema has no room for the per-check detail this card's contract requires
    # (docs/INTERFACES.md §3.7.6: a PASS/FAIL table), so augment both files it just wrote rather
    # than duplicating its write logic here (same documented adaptation as scripts/check_env.py).
    timestamped_candidates = sorted(
        p for p in latest_path.parent.glob("verify_model_*.json") if p.name != "verify_model_latest.json"
    )
    extra = {"checks": [c.to_dict() for c in checks]}
    targets = [latest_path]
    if timestamped_candidates:
        targets.append(timestamped_candidates[-1])
    for target in targets:
        data = json.loads(target.read_text(encoding="utf-8"))
        data.update(extra)
        target.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    return latest_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="verify_model.py",
        description="nnU-Net model wiring and semantic validation (docs/INTERFACES.md §3.7).",
    )
    add_common_args(parser)
    parser.add_argument("--check-hashes", action="store_true", help="re-verify sha256 of every config/model_manifest.json file")
    parser.add_argument(
        "--repair-symlink", action="store_true", help="replace an existing *broken* symlink at nnunet/results/<dataset>"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"verify_model: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("verify_model", cfg, verbose=args.verbose, quiet=args.quiet)

    dataset_name = cfg.model["dataset_name"]
    trainer_config = cfg.model["trainer_config"]
    fold = cfg.model["fold"]
    checkpoint_name = cfg.model["checkpoint"]
    configuration = cfg.model.get("configuration", "2d")

    base = cfg.paths["models"] / dataset_name / trainer_config
    plans_path = base / "plans.json"
    dataset_path = base / "dataset.json"
    checkpoint_path = base / f"fold_{fold}" / checkpoint_name

    checks: list[Check] = []
    checks.append(_check_required_files(plans_path, dataset_path, checkpoint_path))

    dataset_json, dataset_err = _load_json(dataset_path)
    checks.append(_json_load_check("dataset_json_parse", dataset_path, dataset_err))
    checks.append(_assert_field("dataset_channel_names", dataset_json, "channel_names", EXPECTED_CHANNEL_NAMES))
    checks.append(_assert_field("dataset_labels", dataset_json, "labels", EXPECTED_LABELS))
    checks.append(_assert_field("dataset_file_ending", dataset_json, "file_ending", EXPECTED_FILE_ENDING))
    checks.append(_assert_field("dataset_reader_writer", dataset_json, "overwrite_image_reader_writer", EXPECTED_READER_WRITER))

    plans_json, plans_err = _load_json(plans_path)
    checks.append(_json_load_check("plans_json_parse", plans_path, plans_err))
    checks.append(_check_configuration_present(plans_json, configuration))
    checks.append(_check_patch_size(plans_json, configuration))
    checks.append(_check_normalization_schemes(plans_json, configuration))
    checks.append(_assert_field("plans_name", plans_json, "plans_name", EXPECTED_PLANS_NAME))
    checks.append(_assert_field("plans_dataset_name", plans_json, "dataset_name", dataset_name))

    checks.append(_check_checkpoint(checkpoint_path, logger))
    checks.append(_check_symlink(cfg, args.repair_symlink, args.dry_run, logger))

    if args.check_hashes:
        manifest_path = cfg.root / "config" / "model_manifest.json"
        checks.append(_check_hashes(manifest_path, cfg.paths["models"], logger))

    log_by_status = {STATUS_PASS: logger.info, STATUS_WARN: logger.warning, STATUS_FAIL: logger.error}
    for check in checks:
        log_by_status[check.status]("%-28s %-4s %s", check.name, check.status, check.value)

    _print_table(checks)

    hash_check = next((c for c in checks if c.name == "check_hashes"), None)
    hash_fail = hash_check is not None and hash_check.status == STATUS_FAIL
    core_fail = any(c.status == STATUS_FAIL for c in checks if c.name != "check_hashes")

    if hash_fail:
        exit_code, status = EXIT_RUNTIME, "failed"
    elif core_fail:
        exit_code, status = EXIT_PRECONDITION, "precondition"
    else:
        exit_code, status = EXIT_OK, "ok"

    if args.dry_run:
        logger.info("dry-run: logs/verify_model_latest.json not written")
        return EXIT_OK

    _write_summary(cfg, checks, status, exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
