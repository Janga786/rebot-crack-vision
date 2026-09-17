"""crackvision.config — project root resolution and config/project.yaml loading. Built in TC-001."""

from __future__ import annotations

import argparse
import copy
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

DEFAULT_CONFIG: dict[str, Any] = {
    "schema_version": 1,
    "paths": {
        "input_originals": "data/input_originals",
        "nnunet_input": "data/nnunet_input",
        "predictions": "data/predictions",
        "overlays": "data/overlays",
        "comparisons": "data/comparisons",
        "skeletons": "data/skeletons",
        "smoke_test": "data/smoke_test",
        "d405": "data/d405",
        "models": "models/opencrack-nnunet",
        "logs": "logs",
    },
    "model": {
        "repo_id": "fadeevla/opencrack-nnunet",
        "dataset_name": "Dataset501_OpenCrack",
        "trainer_config": "nnUNetTrainer__nnUNetPlans__2d",
        "fold": 0,
        "checkpoint": "checkpoint_ep0500.pth",
        "configuration": "2d",
        "dataset_id": 501,
    },
    "inference": {
        "device": "cuda",
        "step_size": 0.5,
        "disable_tta": False,
        "not_on_device": False,
        "npp": 3,
        "nps": 3,
    },
    "visualization": {
        "overlay_alpha": 0.5,
        "crack_color": [255, 0, 0],
        "skeleton_color": [0, 255, 0],
        "panel_gutter_px": 8,
    },
    "skeleton": {
        "min_component_size": 64,
    },
    "realsense": {
        "color": {"width": 848, "height": 480, "fps": 30, "format": "bgr8"},
        "depth": {"width": 848, "height": 480, "fps": 30, "format": "z16"},
        "align_to": "color",
        "warmup_frames": 30,
    },
}


class ConfigError(Exception):
    """Malformed config or an unresolvable project root. Callers map this to exit code 2."""


def find_root(start: Path | None = None) -> Path:
    """Resolve the project root via $CRACKVISION_ROOT, else by walking up from `start`."""
    env_root = os.environ.get("CRACKVISION_ROOT")
    if env_root:
        candidate = Path(env_root).expanduser().resolve()
        if _looks_like_root(candidate):
            return candidate
        raise ConfigError(
            f"CRACKVISION_ROOT={env_root!r} does not contain config/project.yaml and src/crackvision"
        )

    here = (start or Path(__file__)).resolve()
    for directory in [here, *here.parents]:
        if _looks_like_root(directory):
            return directory
    raise ConfigError(
        f"could not locate project root: no ancestor of {here} contains "
        "both config/project.yaml and src/crackvision"
    )


def _looks_like_root(directory: Path) -> bool:
    return (directory / "config" / "project.yaml").is_file() and (directory / "src" / "crackvision").is_dir()


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


@dataclass
class Config:
    """Resolved project configuration: absolute paths plus the remaining sections as plain dicts."""

    root: Path
    paths: dict[str, Path]
    model: dict[str, Any]
    inference: dict[str, Any]
    visualization: dict[str, Any]
    skeleton: dict[str, Any]
    realsense: dict[str, Any]
    raw: dict[str, Any] = field(repr=False, default_factory=dict)


def load_config(path: Path | None = None, root: Path | None = None) -> Config:
    """Load config/project.yaml, deep-merged over DEFAULT_CONFIG, with paths resolved against root."""
    resolved_root = (root or find_root()).resolve()
    config_path = Path(path) if path is not None else (resolved_root / "config" / "project.yaml")

    merged = copy.deepcopy(DEFAULT_CONFIG)
    if config_path.is_file():
        try:
            with open(config_path, "r", encoding="utf-8") as fh:
                loaded = yaml.safe_load(fh) or {}
        except yaml.YAMLError as exc:
            raise ConfigError(f"malformed YAML in {config_path}: {exc}") from exc
        if not isinstance(loaded, dict):
            raise ConfigError(f"{config_path} must contain a mapping at the top level")
        merged = _deep_merge(merged, loaded)

    try:
        resolved_paths = {key: (resolved_root / value).resolve() for key, value in merged["paths"].items()}
    except (KeyError, TypeError, AttributeError) as exc:
        raise ConfigError(f"malformed 'paths' section in {config_path}: {exc}") from exc

    return Config(
        root=resolved_root,
        paths=resolved_paths,
        model=merged.get("model", {}),
        inference=merged.get("inference", {}),
        visualization=merged.get("visualization", {}),
        skeleton=merged.get("skeleton", {}),
        realsense=merged.get("realsense", {}),
        raw=merged,
    )


def ensure_dirs(cfg: Config) -> None:
    """Create every directory under cfg.paths that does not already exist."""
    for directory in cfg.paths.values():
        Path(directory).mkdir(parents=True, exist_ok=True)


def add_common_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Add the flags every crackvision CLI shares (docs/INTERFACES.md §0.3)."""
    parser.add_argument("--root", type=Path, default=None, help="project root override")
    parser.add_argument("--config", type=Path, default=None, help="config file override")
    parser.add_argument("-v", "--verbose", action="store_true", help="DEBUG logging")
    parser.add_argument("-q", "--quiet", action="store_true", help="WARNING+ only")
    parser.add_argument("--dry-run", action="store_true", help="log what would be done, write nothing, exit 0")
    parser.add_argument(
        "--skip-existing", action="store_true", help="do not regenerate outputs that already exist"
    )
    return parser
