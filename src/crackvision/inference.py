"""crackvision.inference — batch inference runner. Built by TC-008.

Invokes `nnUNetv2_predict_from_modelfolder` (or, via `--use-dataset-id`, `nnUNetv2_predict -d 501`)
against `data/nnunet_input/`, verifying every precondition before launching anything, timing the
subprocess, verifying the outputs, and handling CUDA OOM without retrying or killing anything
(docs/INTERFACES.md §3.2, ARCHITECTURE.md §9).

The released OpenCrack checkpoint's own metadata (`trainer_name`) names a custom trainer class,
`nnUNetTrainerSaveEvery10`, that nnunetv2 2.8.1 does not ship (discovered and diagnosed in TC-006,
docs/COMPLETION_LOG.md). nnU-Net resolves the trainer class via its own documented
`nnUNet_extTrainer` environment variable extension point, so this module writes a tiny inference-only
shim (inherits `nnUNetTrainer` unmodified) into `logs/ext_trainer/` on every real run and points the
subprocess's environment at it. No training is ever invoked (adr/010).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)

_OOM_RE = re.compile(r"(?i)(out of memory|outofmemoryerror|cuda error: out of memory)")

# See module docstring. Inherits nnUNetTrainer unmodified: this file exists only to satisfy a
# class-name lookup for INFERENCE (nnU-Net's recursive_find_trainer_class_by_name(), via the
# documented nnUNet_extTrainer environment variable), never to train.
_EXT_TRAINER_CLASS_NAME = "nnUNetTrainerSaveEvery10"
_EXT_TRAINER_SOURCE = f'''"""Inference-only shim — see crackvision.inference module docstring, docs/COMPLETION_LOG.md TC-006.

Recreates the class NAME the OpenCrack checkpoint's "trainer_name" metadata field requires, so
nnU-Net's recursive_find_trainer_class_by_name() can resolve it via the nnUNet_extTrainer
environment variable. Inherits nnUNetTrainer unmodified: satisfies a name lookup for INFERENCE
only, never trains (adr/010 — nnUNetv2_train is never invoked).
"""

from nnunetv2.training.nnUNetTrainer.nnUNetTrainer import nnUNetTrainer


class {_EXT_TRAINER_CLASS_NAME}(nnUNetTrainer):
    pass
'''


def _looks_like_oom(text: str) -> bool:
    return _OOM_RE.search(text) is not None


def model_folder(cfg: Config) -> Path:
    """The nnU-Net results folder for this model (docs/INTERFACES.md §3.2)."""
    return cfg.paths["models"] / cfg.model["dataset_name"] / cfg.model["trainer_config"]


def missing_model_files(cfg: Config) -> list[Path]:
    """Required files inside `model_folder(cfg)` that are absent; empty if the tree is intact."""
    folder = model_folder(cfg)
    required = [
        folder / "plans.json",
        folder / "dataset.json",
        folder / f"fold_{cfg.model['fold']}" / cfg.model["checkpoint"],
    ]
    return [p for p in required if not p.is_file()]


def prepare_ext_trainer_shim(cfg: Config) -> Path:
    """Write the nnUNet_extTrainer shim into logs/ext_trainer/ (never into models/**). Idempotent."""
    ext_dir = cfg.paths["logs"] / "ext_trainer"
    ext_dir.mkdir(parents=True, exist_ok=True)
    shim_path = ext_dir / f"{_EXT_TRAINER_CLASS_NAME}.py"
    shim_path.write_text(_EXT_TRAINER_SOURCE, encoding="utf-8")
    return ext_dir


def build_predict_command(
    cfg: Config,
    input_dir: Path,
    output_dir: Path,
    *,
    device: str,
    disable_tta: bool,
    not_on_device: bool,
    step_size: float,
    npp: int,
    nps: int,
    skip_existing: bool,
    use_dataset_id: bool,
) -> list[str]:
    """Build the nnU-Net CLI invocation as a list (never a shell string) — docs/INTERFACES.md §3.2.

    `-f` and `-chk` come from `config/project.yaml` (`model.fold`, `model.checkpoint`) but are
    always present: the defaults nnU-Net falls back to, `(0,1,2,3,4)` and `checkpoint_final.pth`,
    both fail against this model.
    """
    fold = str(cfg.model["fold"])
    checkpoint = str(cfg.model["checkpoint"])

    if use_dataset_id:
        # adr/006 fallback: works because TC-005 wired nnunet/results/Dataset501_OpenCrack.
        command = [
            "nnUNetv2_predict",
            "-i", str(input_dir),
            "-o", str(output_dir),
            "-d", str(cfg.model["dataset_id"]),
            "-c", str(cfg.model["configuration"]),
            "-f", fold,
            "-chk", checkpoint,
            "-device", device,
            "-npp", str(npp),
            "-nps", str(nps),
            "-step_size", str(step_size),
        ]
    else:
        command = [
            "nnUNetv2_predict_from_modelfolder",
            "-i", str(input_dir),
            "-o", str(output_dir),
            "-m", str(model_folder(cfg)),
            "-f", fold,
            "-chk", checkpoint,
            "-device", device,
            "-npp", str(npp),
            "-nps", str(nps),
            "-step_size", str(step_size),
        ]

    if disable_tta:
        command.append("--disable_tta")
    if not_on_device:
        command.append("--not_on_device")
    if skip_existing:
        command.append("--continue_prediction")
    return command


def _precondition(command: list[str], errors: list[str]) -> dict[str, Any]:
    return {"status": "precondition", "exit_code": EXIT_PRECONDITION, "command": command, "errors": errors}


def _log_oom_ladder(log: logging.Logger) -> None:
    log.error("CUDA out of memory. Try, in order:")
    log.error("  1. ./env.sh python -m crackvision.inference --not-on-device")
    log.error("  2. ./env.sh python -m crackvision.inference --not-on-device --npp 1 --nps 1")
    log.error("  3. ./env.sh python -m crackvision.inference --not-on-device --disable-tta")
    log.error("  4. ./env.sh python -m crackvision.prepare_inputs --max-side 1024   (then re-run inference)")
    log.error("  5. ./env.sh python -m crackvision.inference --device cpu")
    log.error("This machine is shared — other projects may be holding VRAM. Do NOT kill their processes.")


def run_inference(
    cfg: Config,
    *,
    device: str = "cuda",
    disable_tta: bool = False,
    not_on_device: bool = False,
    step_size: float = 0.5,
    npp: int = 3,
    nps: int = 3,
    skip_existing: bool = False,
    dry_run: bool = False,
    logger: logging.Logger | None = None,
    input_dir: Path | None = None,
    output_dir: Path | None = None,
    use_dataset_id: bool = False,
) -> dict[str, Any]:
    """Run one batch inference pass. Importable: also used by scripts/smoke_test.py (TC-006/TC-008).

    Returns a summary dict with at least `status` (ok/partial/failed/precondition/dry_run),
    `exit_code`, `command` and `errors`; a successful real run also carries `duration_s`, `images`
    and `seconds_per_image`. Never touches prediction pixel data — nnU-Net's output is passed
    through byte-for-byte.
    """
    log = logger or logging.getLogger("crackvision.inference")
    resolved_input_dir = Path(input_dir) if input_dir is not None else cfg.paths["nnunet_input"]
    resolved_output_dir = Path(output_dir) if output_dir is not None else cfg.paths["predictions"]

    command = build_predict_command(
        cfg,
        resolved_input_dir,
        resolved_output_dir,
        device=device,
        disable_tta=disable_tta,
        not_on_device=not_on_device,
        step_size=step_size,
        npp=npp,
        nps=nps,
        skip_existing=skip_existing,
        use_dataset_id=use_dataset_id,
    )
    log.info("nnU-Net command: %s", " ".join(command))

    if dry_run:
        log.info("dry-run: not executing")
        return {"status": "dry_run", "exit_code": EXIT_OK, "command": command, "device": device, "errors": []}

    folder = model_folder(cfg)
    if not folder.is_dir():
        msg = f"model folder missing: {folder}"
        log.error(msg)
        log.error("run: ./env.sh python scripts/fetch_model.py")
        return _precondition(command, [msg])

    missing = missing_model_files(cfg)
    if missing:
        for p in missing:
            log.error("required model file missing: %s", p)
        log.error("run: ./env.sh python scripts/verify_model.py")
        return _precondition(command, [f"missing model file: {p}" for p in missing])

    inputs = sorted(resolved_input_dir.glob("*_0000.png")) if resolved_input_dir.is_dir() else []
    if not inputs:
        msg = f"no *_0000.png files in {resolved_input_dir}"
        log.error(msg)
        log.error("run: ./env.sh python -m crackvision.prepare_inputs")
        return _precondition(command, [msg])

    if device == "cuda":
        import torch

        if not torch.cuda.is_available():
            log.error("--device cuda requested but torch.cuda.is_available() is False")
            log.error("use --device cpu, or check ./env.sh python scripts/check_env.py")
            return _precondition(command, ["cuda requested but unavailable"])

    executable = command[0]
    if shutil.which(executable) is None:
        log.error("%s not found on PATH", executable)
        log.error("run TC-002 / check ./env.sh")
        return _precondition(command, [f"{executable} not on PATH"])

    resolved_output_dir.mkdir(parents=True, exist_ok=True)
    ext_trainer_dir = prepare_ext_trainer_shim(cfg)
    env = os.environ.copy()
    env["nnUNet_extTrainer"] = str(ext_trainer_dir)
    log.info("nnUNet_extTrainer=%s", ext_trainer_dir)
    log.info("predictions are uint8 PNGs with values {0,1} (not {0,255}); see data/overlays/ for viewable renders")

    t0 = time.monotonic()
    proc = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, env=env
    )
    lines: list[str] = []
    for line in proc.stdout:  # type: ignore[union-attr]
        stripped = line.rstrip()
        lines.append(stripped)
        if stripped.strip():
            log.info("nnunet: %s", stripped)
    proc.wait()
    duration_s = time.monotonic() - t0
    combined = "\n".join(lines)

    if proc.returncode != 0:
        if _looks_like_oom(combined):
            _log_oom_ladder(log)
            errors = ["CUDA OOM during nnU-Net predict; see log for degradation ladder"]
        else:
            log.error("%s exited %d", executable, proc.returncode)
            errors = [f"nnU-Net subprocess exited {proc.returncode}"]
        return {
            "status": "failed",
            "exit_code": EXIT_RUNTIME,
            "command": command,
            "device": device,
            "duration_s": duration_s,
            "errors": errors,
        }

    expected_cases = [p.name[: -len("_0000.png")] for p in inputs]
    missing_outputs = [case for case in expected_cases if not (resolved_output_dir / f"{case}.png").is_file()]
    images = len(expected_cases) - len(missing_outputs)
    seconds_per_image = duration_s / images if images else 0.0

    result: dict[str, Any] = {
        "command": command,
        "device": device,
        "duration_s": duration_s,
        "images": images,
        "seconds_per_image": seconds_per_image,
        "errors": [],
    }

    if missing_outputs:
        for case in missing_outputs:
            log.error("missing prediction for case %s", case)
        result["status"] = "partial"
        result["exit_code"] = EXIT_RUNTIME
        result["errors"] = [f"missing prediction: {case}" for case in missing_outputs]
        result["missing"] = missing_outputs
        return result

    log.info("inference complete: %d image(s) in %.2fs (%.3fs/image)", images, duration_s, seconds_per_image)
    result["status"] = "ok"
    result["exit_code"] = EXIT_OK
    return result


def _augment_summary_files(log_dir: Path, tool: str, latest_path: Path, extra: dict[str, Any]) -> None:
    """Merge extra top-level fields into `<tool>_latest.json` and its timestamped twin.

    `RunSummary`'s schema (crackvision.logging_setup, TC-001, not this card's to modify) has no
    field for `run_inference()`'s extra keys (`images`, `inference_duration_s`, `seconds_per_image`,
    `command`, `device`) — the same documented adaptation TC-003 (`checks`/`versions`) and TC-005
    used for their own extra fields. `inference_duration_s` is named distinctly from RunSummary's
    own `duration_s` (the tool's whole-run wall time) so the two are never conflated.
    """
    extra = {k: v for k, v in extra.items() if v is not None}
    if not extra:
        return
    candidates = sorted(p for p in log_dir.glob(f"{tool}_*.json") if p.name != f"{tool}_latest.json")
    targets = [latest_path] + ([candidates[-1]] if candidates else [])
    for path in targets:
        data = json.loads(path.read_text(encoding="utf-8"))
        data.update(extra)
        path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="inference.py",
        description="Batch inference runner: invokes nnUNetv2_predict_from_modelfolder against "
        "data/nnunet_input/ (docs/INTERFACES.md §3.2).",
    )
    add_common_args(parser)
    parser.add_argument("--input-dir", type=Path, default=None, help="override data/nnunet_input")
    parser.add_argument("--output-dir", type=Path, default=None, help="override data/predictions")
    parser.add_argument(
        "--device", choices=["cuda", "cpu"], default=None, help="inference device (default: config/project.yaml)"
    )
    parser.add_argument(
        "--disable-tta", dest="disable_tta", action="store_true", default=None, help="disable mirroring TTA"
    )
    parser.add_argument(
        "--not-on-device",
        dest="not_on_device",
        action="store_true",
        default=None,
        help="move tile-aggregation buffer to CPU RAM",
    )
    parser.add_argument("--step-size", type=float, default=None, help="nnU-Net sliding-window step size")
    parser.add_argument("--npp", type=int, default=None, help="number of preprocessing processes")
    parser.add_argument("--nps", type=int, default=None, help="number of segmentation-export processes")
    parser.add_argument(
        "--use-dataset-id",
        action="store_true",
        help="use nnUNetv2_predict -d 501 -c 2d instead of -m (adr/006 fallback)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"inference: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("inference", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    device = args.device or cfg.inference["device"]
    disable_tta = cfg.inference["disable_tta"] if args.disable_tta is None else args.disable_tta
    not_on_device = cfg.inference["not_on_device"] if args.not_on_device is None else args.not_on_device
    step_size = cfg.inference["step_size"] if args.step_size is None else args.step_size
    npp = cfg.inference["npp"] if args.npp is None else args.npp
    nps = cfg.inference["nps"] if args.nps is None else args.nps
    input_dir = args.input_dir or cfg.paths["nnunet_input"]
    output_dir = args.output_dir or cfg.paths["predictions"]

    result = run_inference(
        cfg,
        device=device,
        disable_tta=disable_tta,
        not_on_device=not_on_device,
        step_size=step_size,
        npp=npp,
        nps=nps,
        skip_existing=args.skip_existing,
        dry_run=args.dry_run,
        logger=logger,
        input_dir=input_dir,
        output_dir=output_dir,
        use_dataset_id=args.use_dataset_id,
    )

    if args.dry_run:
        logger.info("dry-run: logs/inference_latest.json not written")
        return EXIT_OK

    for err in result.get("errors", []):
        summary.add_error(err)
    if "images" in result:
        summary.increment("images", result["images"])

    latest_path = summary.write(cfg, "inference", result["status"], result["exit_code"])
    _augment_summary_files(
        cfg.paths["logs"],
        "inference",
        latest_path,
        {
            "images": result.get("images"),
            "inference_duration_s": result.get("duration_s"),
            "seconds_per_image": result.get("seconds_per_image"),
            "command": result.get("command"),
            "device": result.get("device"),
        },
    )

    return result["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
