"""retime_plan -- §16.6 CLI for the MOT-07 time-parameterization retimer (MOT-07.5).

console_script: `ros2 run crackvision_motion retime_plan`. Imports no `rclpy`/`*_msgs` type
anywhere (mirroring `retime.py`/`retime_core.py`/`tool_fk.py`), so it is also runnable, identically,
as `bash scripts/ros/env_ros.sh python3 -m crackvision_motion.retime_plan` under system python3
without a colcon build -- exactly how `crackvision_motion.execution_gate` (§11.10) is dual-runnable
today. `--dry-run` (§0.3/§16.6) runs the whole `retime_plan_doc` pipeline in-process and prints the
per-leg summary, but never calls `write_outputs`, and bypasses `cli_common.run_cli`'s own generic
dry-run shortcut (which would otherwise skip the work entirely) -- the same deliberate bypass
`execute_trajectory._offline_dry_run` uses for its own, differently-shaped, `--dry-run`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from .cartesian_plan import CartesianPlanError, load_plan, load_view_plan
from .cli_common import EXIT_OK, PreconditionError, RunSummary, add_common_args, find_root, run_cli
from .joint_trajectory import TrajectoryError
from .reachability_core import ConfigError
from .retime import RetimeError, retime_plan_doc, retime_view_plan_doc, write_outputs
from .retime_core import RetimeConfigError, load_retime_config

TOOL = "retime_plan"

DEFAULT_RETIME_CONFIG = "config/motion/retime.yaml"
DEFAULT_LIMITS = "config/robot/b601_dm_limits.yaml"
DEFAULT_END_EFFECTOR_CONFIG = "config/robot/end_effector.yaml"
DEFAULT_EXECUTION_CONFIG = "config/motion/execution.yaml"
DEFAULT_URDF = "presentation/sim/reBot_B601_DM_with_gripper.urdf"
DEFAULT_OUT_DIR = "data/trajectories"
DEFAULT_VIEW_OUT = "data/trajectories/view_trajectory.json"

# `cli_common.run_cli` hardcodes which exception types map to which exit code (§0.2): this
# module's own refusal/config-error types are not among them, so `_retime` translates them into
# `reachability_core.ConfigError` (exit 2, usage/config error) or `cli_common.PreconditionError`
# (exit 3, refusal) before they ever reach `run_cli`/`_dry_run`. A malformed `retime.yaml` is
# checked separately, first (see `_retime` below), since `retime_plan_doc` itself converts that
# `RetimeConfigError` into a `RetimeError` refusal rather than leaving it as a config error.
_CONFIG_ERRORS = (TrajectoryError,)
_PRECONDITION_ERRORS = (RetimeError, CartesianPlanError)


def _extra_args(parser: argparse.ArgumentParser) -> None:
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--plan", default=None, help="crackvision.cartesian_plan/1 file to retime")
    group.add_argument("--view-plan", default=None, help="crackvision.view_plan/1 file to retime (§17.3)")
    parser.add_argument(
        "--out", default=None,
        help=(
            f"default: {DEFAULT_OUT_DIR}/{{case_id}}_trajectory.json under --root (--plan), or "
            f"{DEFAULT_VIEW_OUT} under --root (--view-plan)"
        ),
    )
    parser.add_argument(
        "--purpose", choices=["crack_task", "test"], default=None,
        help="§11.2 purpose field; --plan only (default crack_task), exit 2 with --view-plan",
    )
    parser.add_argument("--retime-config", default=None, help=f"default: {DEFAULT_RETIME_CONFIG} under --root")
    parser.add_argument("--limits", default=None, help=f"default: {DEFAULT_LIMITS} under --root")
    parser.add_argument("--end-effector-config", default=None, help=f"default: {DEFAULT_END_EFFECTOR_CONFIG} under --root")
    parser.add_argument("--execution-config", default=None, help=f"default: {DEFAULT_EXECUTION_CONFIG} under --root")
    parser.add_argument("--urdf", default=None, help=f"default: {DEFAULT_URDF} under --root")


def _resolve(root: Path, value, default: str) -> Path:
    if value is not None:
        return Path(value).expanduser().resolve()
    return (root / default).resolve()


def _resolve_paths(args: argparse.Namespace, root: Path) -> Dict[str, Path]:
    return {
        "retime_config": _resolve(root, args.retime_config, DEFAULT_RETIME_CONFIG),
        "limits": _resolve(root, args.limits, DEFAULT_LIMITS),
        "end_effector_config": _resolve(root, args.end_effector_config, DEFAULT_END_EFFECTOR_CONFIG),
        "execution_config": _resolve(root, args.execution_config, DEFAULT_EXECUTION_CONFIG),
        "urdf": _resolve(root, args.urdf, DEFAULT_URDF),
    }


def _out_path(root: Path, args: argparse.Namespace, case_id: Optional[str]) -> Path:
    if args.out is not None:
        return Path(args.out).expanduser().resolve()
    if case_id is not None:
        return (root / DEFAULT_OUT_DIR / f"{case_id}_trajectory.json").resolve()
    return (root / DEFAULT_VIEW_OUT).resolve()


def _retime(args: argparse.Namespace, root: Path):
    """Run the whole retime pipeline for either `--plan` or `--view-plan` and return
    `(kind, doc, trajectory_doc, report_doc)`, `kind` one of `"plan"`/`"view"` and `doc` the
    loaded `cartesian_plan`/`view_plan` document (used by both `_print_leg_summary` and `_run`'s
    `--out` default)."""
    paths = _resolve_paths(args, root)
    for key in ("retime_config", "limits", "end_effector_config", "execution_config", "urdf"):
        if not paths[key].is_file():
            raise PreconditionError(f"{key} file not found: {paths[key]}")

    # `retime_plan_doc`/`retime_view_plan_doc` themselves convert a `RetimeConfigError` from
    # `load_retime_config` into a `RetimeError` (an exit-3 refusal, §16.2 Step 5's error-handling
    # convention for everything downstream of a loaded config) -- so an unknown `retime.yaml` key
    # is validated here, first, against the raw `RetimeConfigError`, to land on exit 2 (§16.6's
    # own worked example) rather than exit 3.
    try:
        load_retime_config(paths["retime_config"])
    except RetimeConfigError as exc:
        raise ConfigError(str(exc)) from exc

    if args.view_plan is not None:
        if args.purpose is not None:
            raise ConfigError("--purpose may not be given together with --view-plan (§17.3)")
        view_plan_path = Path(args.view_plan).expanduser().resolve()
        if not view_plan_path.is_file():
            raise PreconditionError(f"view plan file not found: {view_plan_path}")
        try:
            view_plan = load_view_plan(view_plan_path, paths["limits"])
            trajectory_doc, report_doc = retime_view_plan_doc(
                view_plan,
                view_plan_path,
                root,
                limits_path=paths["limits"],
                end_effector_path=paths["end_effector_config"],
                urdf_path=paths["urdf"],
                retime_config_path=paths["retime_config"],
                execution_config_path=paths["execution_config"],
            )
        except _CONFIG_ERRORS as exc:
            raise ConfigError(str(exc)) from exc
        except _PRECONDITION_ERRORS as exc:
            raise PreconditionError(str(exc)) from exc
        return "view", view_plan, trajectory_doc, report_doc

    plan_path = Path(args.plan).expanduser().resolve()
    if not plan_path.is_file():
        raise PreconditionError(f"plan file not found: {plan_path}")
    purpose = args.purpose if args.purpose is not None else "crack_task"
    try:
        plan = load_plan(plan_path, paths["limits"])
        trajectory_doc, report_doc = retime_plan_doc(
            plan,
            limits_path=paths["limits"],
            end_effector_path=paths["end_effector_config"],
            urdf_path=paths["urdf"],
            retime_config_path=paths["retime_config"],
            execution_config_path=paths["execution_config"],
            purpose=purpose,
            plan_path=plan_path,
        )
    except _CONFIG_ERRORS as exc:
        raise ConfigError(str(exc)) from exc
    except _PRECONDITION_ERRORS as exc:
        raise PreconditionError(str(exc)) from exc
    return "plan", plan, trajectory_doc, report_doc


def _print_leg_summary(kind: str, doc: Dict[str, Any], report_doc: Dict[str, Any]) -> None:
    for leg in report_doc["legs"]:
        if kind == "plan":
            n_configurations = len(doc["legs"][leg["leg_index"]]["positions"])
        else:
            n_configurations = len(doc["transit_leg"]["positions"])
        first, last = leg["point_range"]
        n_refined_points = last - first + 1
        line = (
            f"leg {leg['leg_index']} ({leg['kind']}): configurations={n_configurations} "
            f"refined_points={n_refined_points} duration={leg['duration_s']:.4f}s"
        )
        if leg["kind"] == "trace":
            line += f" v_c={leg['tool_speed']['constant']:.5f} m/s"
        print(line)


def _dry_run(args: argparse.Namespace) -> int:
    """§0.3/§16.6 `--dry-run`: run the whole retime pipeline in-process (validating paths/shas/
    schema and every per-leg tool-speed feasibility check), print the per-leg summary, and write
    nothing -- exit 0 regardless of the outcome."""
    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    try:
        kind, doc, _trajectory_doc, report_doc = _retime(args, root)
    except (PreconditionError, ConfigError) as exc:
        print(f"retime_plan --dry-run: refused: {exc}", file=sys.stderr)
        return EXIT_OK
    _print_leg_summary(kind, doc, report_doc)
    return EXIT_OK


def _run(args: argparse.Namespace, log, summary: RunSummary) -> Dict[str, str]:
    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    kind, doc, trajectory_doc, report_doc = _retime(args, root)
    out_path = _out_path(root, args, doc["case_id"] if kind == "plan" else None)

    if args.skip_existing and out_path.is_file():
        log.info("--skip-existing: %s already exists, skipping", out_path)
        summary.increment("skipped")
        return {"status": "ok"}

    write_outputs(trajectory_doc, report_doc, out_path)
    log.info("wrote trajectory to %s and report to %s.retime.json", out_path, out_path)
    summary.increment("written")
    return {"status": "ok"}


def main(argv=None) -> int:
    argv = list(argv) if argv is not None else sys.argv[1:]
    parser = argparse.ArgumentParser(prog=TOOL)
    add_common_args(parser)
    _extra_args(parser)
    args = parser.parse_args(argv)
    if args.dry_run:
        return _dry_run(args)
    return run_cli(TOOL, _run, argv, extra_args=_extra_args)


if __name__ == "__main__":
    sys.exit(main())
