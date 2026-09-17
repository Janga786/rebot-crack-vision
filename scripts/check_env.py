"""scripts/check_env.py — Level 1 environment smoke test. Built by TC-003.

Reports PASS/WARN/FAIL for every property this project depends on (interpreter, ROS
scrub, torch/CUDA, nnU-Net, imaging stack, project paths, model checkpoint) and writes
the reproducibility snapshot consumed by TC-016 to logs/check_env_latest.json. This
script diagnoses only — it never installs, downloads, or repairs anything.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import json
import os
import re
import shutil
import subprocess
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE, RunSummary, setup_logging

STATUS_PASS = "PASS"
STATUS_WARN = "WARN"
STATUS_FAIL = "FAIL"

GIB = 1024**3


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


def _parse_version_tuple(version: str) -> tuple[int, ...]:
    core = version.split("+")[0]
    parts = []
    for part in core.split("."):
        match = re.match(r"\d+", part)
        parts.append(int(match.group()) if match else 0)
    return tuple(parts)


# --- individual checks -------------------------------------------------------------
# Every function has the signature (cfg, versions) -> Check so they can be iterated
# uniformly, even when a given check does not need cfg. `versions` is a shared dict
# that checks populate as a side effect; it becomes the JSON summary's `versions` object.


def check_python_version(_cfg: Config, versions: dict[str, str]) -> Check:
    info = sys.version_info
    value = f"{info.major}.{info.minor}.{info.micro}"
    versions["python"] = value
    if (info.major, info.minor) == (3, 11):
        return Check("python_version", STATUS_PASS, value)
    return Check(
        "python_version", STATUS_FAIL, value, "expected 3.11 from the crackvision env; did you use ./env.sh?"
    )


def check_pythonpath_clean(_cfg: Config, _versions: dict[str, str]) -> Check:
    raw = os.environ.get("PYTHONPATH", "")
    if "/opt/ros" in raw:
        return Check(
            "pythonpath_clean", STATUS_FAIL, raw, "run via ./env.sh — ROS Humble leaks py3.10 paths (adr/007)"
        )
    return Check("pythonpath_clean", STATUS_PASS, "(no ROS entries)")


def check_syspath_clean(_cfg: Config, _versions: dict[str, str]) -> Check:
    offenders = [entry for entry in sys.path if "ros/humble" in entry]
    if offenders:
        return Check(
            "syspath_clean", STATUS_FAIL, "; ".join(offenders), "run via ./env.sh — ROS Humble leaks py3.10 paths"
        )
    return Check("syspath_clean", STATUS_PASS, "(clean)")


def check_ld_library_path_clean(_cfg: Config, _versions: dict[str, str]) -> Check:
    raw = os.environ.get("LD_LIBRARY_PATH", "")
    entries = [entry for entry in raw.split(":") if entry]
    offenders = [entry for entry in entries if "/opt/ros" in entry or "cuda-11.8" in entry]
    if offenders:
        return Check("ld_library_path_clean", STATUS_WARN, "; ".join(offenders), "env.sh should filter these")
    return Check("ld_library_path_clean", STATUS_PASS, "(clean)")


def check_torch_import(_cfg: Config, versions: dict[str, str]) -> Check:
    try:
        import torch
    except Exception as exc:  # noqa: BLE001 - report, do not crash the report
        return Check("torch_import", STATUS_FAIL, f"{type(exc).__name__}: {exc}", "run TC-002")
    versions["torch"] = torch.__version__
    return Check("torch_import", STATUS_PASS, torch.__version__)


def check_torch_version(_cfg: Config, _versions: dict[str, str]) -> Check:
    try:
        import torch
    except Exception as exc:  # noqa: BLE001
        return Check("torch_version", STATUS_FAIL, f"{type(exc).__name__}: {exc}", "run TC-002")
    value = torch.__version__
    is_excluded = value.split("+")[0].startswith("2.9.")
    if _parse_version_tuple(value) >= (2, 1, 2) and not is_excluded:
        return Check("torch_version", STATUS_PASS, value)
    return Check("torch_version", STATUS_FAIL, value, "nnunetv2 requires torch>=2.1.2,!=2.9.*")


def check_torch_cuda_build(_cfg: Config, _versions: dict[str, str]) -> Check:
    try:
        import torch
    except Exception as exc:  # noqa: BLE001
        return Check("torch_cuda_build", STATUS_FAIL, f"{type(exc).__name__}: {exc}", "run TC-002")
    cuda_version = torch.version.cuda
    value = f"{torch.__version__} (cuda={cuda_version})"
    if cuda_version is not None and "+cu" in torch.__version__:
        return Check("torch_cuda_build", STATUS_PASS, value)
    return Check(
        "torch_cuda_build", STATUS_FAIL, value, "CPU-only wheel installed; reinstall from the cu126 index"
    )


def check_cuda_available(_cfg: Config, _versions: dict[str, str]) -> Check:
    try:
        import torch
    except Exception as exc:  # noqa: BLE001
        return Check("cuda_available", STATUS_FAIL, f"{type(exc).__name__}: {exc}", "run TC-002")
    if torch.cuda.is_available():
        return Check("cuda_available", STATUS_PASS, "True")
    return Check("cuda_available", STATUS_WARN, "False", "no CUDA device visible; CPU inference is still supported")


def check_gpu_name(_cfg: Config, versions: dict[str, str]) -> Check:
    try:
        import torch
    except Exception as exc:  # noqa: BLE001
        return Check("gpu_name", STATUS_FAIL, f"{type(exc).__name__}: {exc}", "run TC-002")
    if not torch.cuda.is_available():
        return Check("gpu_name", STATUS_WARN, "N/A (no CUDA device)")
    name = torch.cuda.get_device_name(0)
    versions["gpu_name"] = name
    return Check("gpu_name", STATUS_PASS, name)


def check_gpu_memory(_cfg: Config, _versions: dict[str, str]) -> Check:
    try:
        import torch
    except Exception as exc:  # noqa: BLE001
        return Check("gpu_memory", STATUS_FAIL, f"{type(exc).__name__}: {exc}", "run TC-002")
    if not torch.cuda.is_available():
        return Check("gpu_memory", STATUS_WARN, "N/A (no CUDA device)")
    try:
        free_bytes, total_bytes = torch.cuda.mem_get_info()
    except (RuntimeError, AttributeError):
        total_bytes = torch.cuda.get_device_properties(0).total_memory
        value = f"unknown free / {total_bytes / GIB:.2f} GiB total"
        return Check(
            "gpu_memory", STATUS_WARN, value, "torch.cuda.mem_get_info() unavailable on this torch build"
        )
    free_gib = free_bytes / GIB
    total_gib = total_bytes / GIB
    value = f"{free_gib:.2f} GiB free / {total_gib:.2f} GiB total"
    if free_gib < 6:
        return Check("gpu_memory", STATUS_WARN, value, "other projects may be using the GPU; see ARCHITECTURE.md §9")
    return Check("gpu_memory", STATUS_PASS, value)


def check_nvidia_driver(_cfg: Config, versions: dict[str, str]) -> Check:
    exe = shutil.which("nvidia-smi")
    if exe is None:
        return Check("nvidia_driver", STATUS_WARN, "nvidia-smi not found", "nvidia-smi absent")
    try:
        result = subprocess.run(
            [exe, "--query-gpu=driver_version", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            timeout=10,
            check=True,
        )
    except (subprocess.SubprocessError, OSError) as exc:
        return Check("nvidia_driver", STATUS_WARN, f"{type(exc).__name__}: {exc}", "nvidia-smi errored")
    driver = result.stdout.strip().splitlines()[0] if result.stdout.strip() else ""
    if not driver:
        return Check("nvidia_driver", STATUS_WARN, "(empty output)", "nvidia-smi errored")
    versions["nvidia_driver"] = driver
    return Check("nvidia_driver", STATUS_PASS, driver)


def check_nnunetv2_import(_cfg: Config, versions: dict[str, str]) -> Check:
    try:
        import nnunetv2
    except Exception as exc:  # noqa: BLE001
        return Check("nnunetv2_import", STATUS_FAIL, f"{type(exc).__name__}: {exc}", "run TC-002")
    try:
        version = importlib.metadata.version("nnunetv2")
    except importlib.metadata.PackageNotFoundError:
        version = getattr(nnunetv2, "__version__", "unknown")
    versions["nnunetv2"] = version
    return Check("nnunetv2_import", STATUS_PASS, version)


def check_nnunet_cli(_cfg: Config, _versions: dict[str, str]) -> Check:
    path = shutil.which("nnUNetv2_predict_from_modelfolder")
    if path is None:
        return Check("nnunet_cli", STATUS_FAIL, "not found", "the console script is missing from the env")
    return Check("nnunet_cli", STATUS_PASS, path)


def check_imaging_stack(_cfg: Config, versions: dict[str, str]) -> Check:
    modules = {"numpy": "numpy", "skimage": "scikit-image", "PIL": "Pillow", "cv2": "opencv", "yaml": "pyyaml"}
    parts: list[str] = []
    missing: list[str] = []
    for mod_name, label in modules.items():
        try:
            mod = importlib.import_module(mod_name)
        except Exception as exc:  # noqa: BLE001
            missing.append(f"{mod_name}: {type(exc).__name__}: {exc}")
            continue
        version = getattr(mod, "__version__", "unknown")
        versions[label] = version
        parts.append(f"{mod_name}={version}")
    value = ", ".join(parts)
    if missing:
        value = (value + "; " if value else "") + "; ".join(missing)
        return Check("imaging_stack", STATUS_FAIL, value, "run TC-002 — one or more imaging packages missing")
    return Check("imaging_stack", STATUS_PASS, value)


def check_pyrealsense2(_cfg: Config, versions: dict[str, str]) -> Check:
    try:
        import pyrealsense2 as rs
    except Exception as exc:  # noqa: BLE001
        return Check(
            "pyrealsense2",
            STATUS_WARN,
            f"{type(exc).__name__}: {exc}",
            "optional — pip install -r requirements/requirements-realsense.txt",
        )
    version = getattr(rs, "__version__", None)
    if version is None:
        try:
            version = importlib.metadata.version("pyrealsense2")
        except importlib.metadata.PackageNotFoundError:
            version = "unknown"
    versions["pyrealsense2"] = version
    return Check("pyrealsense2", STATUS_PASS, version)


def check_project_paths(cfg: Config, _versions: dict[str, str]) -> Check:
    missing = []
    not_writable = []
    for key, path in sorted(cfg.paths.items()):
        if not Path(path).exists():
            missing.append(f"{key}={path}")
            continue
        if not os.access(path, os.W_OK):
            not_writable.append(f"{key}={path}")
    if missing or not_writable:
        parts = []
        if missing:
            parts.append("missing: " + ", ".join(missing))
        if not_writable:
            parts.append("not writable: " + ", ".join(not_writable))
        return Check("project_paths", STATUS_FAIL, "; ".join(parts), "run TC-001")
    return Check("project_paths", STATUS_PASS, f"{len(cfg.paths)} paths OK")


def check_nnunet_env_vars(cfg: Config, _versions: dict[str, str]) -> Check:
    names = ["nnUNet_raw", "nnUNet_preprocessed", "nnUNet_results"]
    root_str = str(cfg.root)
    values = {}
    problems = []
    for name in names:
        value = os.environ.get(name)
        values[name] = value
        if not value:
            problems.append(f"{name} unset")
        elif not str(Path(value)).startswith(root_str):
            problems.append(f"{name}={value} outside root")
    rendered = ", ".join(f"{k}={v}" for k, v in values.items())
    if problems:
        return Check("nnunet_env_vars", STATUS_FAIL, rendered, "run via ./env.sh (adr/004)")
    return Check("nnunet_env_vars", STATUS_PASS, rendered)


def check_model_checkpoint(cfg: Config, _versions: dict[str, str]) -> Check:
    checkpoint = (
        cfg.paths["models"]
        / cfg.model["dataset_name"]
        / cfg.model["trainer_config"]
        / f"fold_{cfg.model['fold']}"
        / cfg.model["checkpoint"]
    )
    if not checkpoint.exists():
        return Check(
            "model_checkpoint", STATUS_FAIL, "missing", "run ./env.sh python scripts/fetch_model.py (TC-004)"
        )
    size_mb = checkpoint.stat().st_size / (1024 * 1024)
    value = f"{size_mb:.1f} MB at {checkpoint}"
    if 250 <= size_mb <= 300:
        return Check("model_checkpoint", STATUS_PASS, value)
    return Check(
        "model_checkpoint",
        STATUS_FAIL,
        value,
        "checkpoint exists but its size is outside the expected 250-300 MB range (possibly corrupt/truncated); "
        "re-run ./env.sh python scripts/fetch_model.py (TC-004)",
    )


def check_model_config_files(cfg: Config, _versions: dict[str, str]) -> Check:
    base = cfg.paths["models"] / cfg.model["dataset_name"] / cfg.model["trainer_config"]
    plans = base / "plans.json"
    dataset = base / "dataset.json"
    missing = [p.name for p in (plans, dataset) if not p.exists()]
    if missing:
        return Check(
            "model_config_files",
            STATUS_FAIL,
            f"missing: {', '.join(missing)} in {base}",
            "run ./env.sh python scripts/fetch_model.py (TC-004)",
        )
    return Check("model_config_files", STATUS_PASS, f"plans.json, dataset.json present in {base}")


def check_disk_free(cfg: Config, _versions: dict[str, str]) -> Check:
    usage = shutil.disk_usage(cfg.root)
    free_gib = usage.free / GIB
    value = f"{free_gib:.2f} GiB free"
    if free_gib < 20:
        return Check("disk_free", STATUS_WARN, value, "low disk space")
    return Check("disk_free", STATUS_PASS, value)


CHECK_FUNCS: list[Callable[[Config, dict[str, str]], Check]] = [
    check_python_version,
    check_pythonpath_clean,
    check_syspath_clean,
    check_ld_library_path_clean,
    check_torch_import,
    check_torch_version,
    check_torch_cuda_build,
    check_cuda_available,
    check_gpu_name,
    check_gpu_memory,
    check_nvidia_driver,
    check_nnunetv2_import,
    check_nnunet_cli,
    check_imaging_stack,
    check_pyrealsense2,
    check_project_paths,
    check_nnunet_env_vars,
    check_model_checkpoint,
    check_model_config_files,
    check_disk_free,
]

assert len(CHECK_FUNCS) == 20, "docs/INTERFACES.md §3.5 specifies exactly 20 checks"


def _run_check(fn: Callable[[Config, dict[str, str]], Check], cfg: Config, versions: dict[str, str]) -> Check:
    name = fn.__name__[len("check_") :]
    try:
        return fn(cfg, versions)
    except Exception as exc:  # noqa: BLE001 - one check must never abort the whole report
        return Check(name, STATUS_FAIL, f"{type(exc).__name__}: {exc}", "unexpected exception in this check")


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


def _write_summary(
    cfg: Config, checks: list[Check], versions: dict[str, str], status: str, exit_code: int
) -> Path:
    summary = RunSummary()
    for check_status, count in Counter(c.status for c in checks).items():
        summary.increment(check_status.lower(), count)
    for check in checks:
        if check.status == STATUS_FAIL:
            summary.add_error(f"{check.name}: {check.value}")

    latest_path = summary.write(cfg, "check_env", status, exit_code)

    # RunSummary's schema has no room for the per-check detail this card's contract
    # requires (docs/INTERFACES.md §3.5: a `checks` array plus a `versions` object), so
    # augment both files it just wrote rather than duplicating its write logic here.
    timestamped_candidates = sorted(
        p for p in latest_path.parent.glob("check_env_*.json") if p.name != "check_env_latest.json"
    )
    extra = {"checks": [c.to_dict() for c in checks], "versions": versions}
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
        prog="check_env.py",
        description="Level 1 environment smoke test for crackvision (docs/INTERFACES.md §3.5).",
    )
    add_common_args(parser)
    parser.add_argument("--json", action="store_true", help="print the JSON summary instead of the table")
    parser.add_argument("--strict", action="store_true", help="promote WARN checks to FAIL")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"check_env: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("check_env", cfg, verbose=args.verbose, quiet=args.quiet)

    versions: dict[str, str] = {}
    checks: list[Check] = []
    log_by_status = {STATUS_PASS: logger.info, STATUS_WARN: logger.warning, STATUS_FAIL: logger.error}
    for fn in CHECK_FUNCS:
        check = _run_check(fn, cfg, versions)
        checks.append(check)
        log_by_status[check.status]("%-24s %-4s %s", check.name, check.status, check.value)

    if args.strict:
        for check in checks:
            if check.status == STATUS_WARN:
                check.status = STATUS_FAIL

    has_fail = any(c.status == STATUS_FAIL for c in checks)
    exit_code = EXIT_PRECONDITION if has_fail else EXIT_OK
    status = "precondition" if has_fail else "ok"

    if args.json:
        print(json.dumps({"checks": [c.to_dict() for c in checks], "versions": versions}, indent=2))
    else:
        _print_table(checks)

    if args.dry_run:
        logger.info("dry-run: logs/check_env_latest.json not written")
        return EXIT_OK

    _write_summary(cfg, checks, versions, status, exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
