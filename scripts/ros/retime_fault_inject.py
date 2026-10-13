#!/usr/bin/env python3
"""retime_fault_inject.py — MOT-07.8 fault injector for the offline retime/checker cross-check.

Stdlib only (json/argparse/copy/re/sys/pathlib; no PyYAML, no crackvision/crackvision_motion
import) so it runs identically under either interpreter without a colcon build or a conda env.
Reads one already-retimed `crackvision.joint_trajectory/1` file (§11.2) plus the
`crackvision.cartesian_plan/1` file it was retimed from (§13.11.1) and writes one mutated copy of
the trajectory per fault into `--out-dir`, each isolating a single defect that
`crackvision.trajectory_check`/`execution_gate` (MOT-07.6/.7, MOT-05.3) should catch:

  segment_overspeed.json   one segment's duration shrunk 20%   -> raw FD velocity over limit
  accel_spike.json         one interior point nudged           -> raw FD acceleration over limit
  dropped_config.json      one plan vertex removed from points -> G-PATH vertex missing
  corner_cut.json          one plan vertex replaced by the
                           chord midpoint of its neighbours     -> G-PATH distance/vertex miss
  toolspeed_dip.json       one mid-trace-leg segment slowed way
                           down (not a leading/trailing ramp)   -> G-TOOLSPEED band rule
  wrong_limits_sha.json    trajectory.limits_file.sha256 forged -> C-PROVENANCE mismatch
  t_s_gap.json             one segment's duration inflated      -> G-PERIOD (max_period_s)

`b601_dm_limits.yaml`'s `joints.<name>.velocity`/`.acceleration` are read with a minimal,
indentation-only line scanner (not a general YAML parser: this file's own fixed 2/4/6-space
layout, §16.1) so no PyYAML dependency is needed here.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]

# §16.2 Step 1 continuity tolerance (mirrors crackvision_motion.cartesian_plan.CONTINUITY_TOL_RAD /
# src/crackvision/trajectory_check.py's own hand-kept copy); not importable here by design.
_CONTINUITY_TOL_RAD = 1e-6
_BIT_EXACT_TOL = 0.0


class FaultInjectError(Exception):
    """Raised when a fault cannot be constructed against the given trajectory/plan."""


def _read_limits(path: Path) -> Dict[str, Tuple[float, float]]:
    """Minimal reader for `b601_dm_limits.yaml`'s `joints.<name>.velocity`/`.acceleration`
    (§16.1), using indentation alone: a 2-space line starts a joint block, a 4-space line is a
    direct field of that joint, anything indented 6+ spaces (the `source:` provenance block) is
    ignored. Not a general YAML parser."""
    result: Dict[str, Tuple[float, float]] = {}
    current: str | None = None
    partial: Dict[str, float] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        stripped = raw_line.strip()
        if indent == 2 and stripped.endswith(":"):
            if current is not None and "velocity" in partial and "acceleration" in partial:
                result[current] = (partial["velocity"], partial["acceleration"])
            current = stripped[:-1]
            partial = {}
        elif indent == 4 and current is not None and ":" in stripped:
            key, _, value = stripped.partition(":")
            key = key.strip()
            if key == "velocity":
                partial["velocity"] = float(value.strip())
            elif key == "acceleration":
                partial["acceleration"] = float(value.strip())
    if current is not None and "velocity" in partial and "acceleration" in partial:
        result[current] = (partial["velocity"], partial["acceleration"])
    missing = [n for n in JOINT_NAMES if n not in result]
    if missing:
        raise FaultInjectError(f"{path}: missing velocity/acceleration for joint(s) {missing}")
    return result


def _load_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise FaultInjectError(f"cannot open {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise FaultInjectError(f"malformed JSON in {path}: {exc}") from exc


def _dump_json(doc: Any, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")


def _shift_tail(points: List[dict], from_index: int, delta: float) -> None:
    """Add `delta` to every point's `t_s` from `from_index` onward -- changes only the duration of
    the segment ending at `from_index` (by `-delta`), leaving every other segment's own duration
    unchanged, and keeps `t_s` strictly increasing provided `|delta|` is smaller than that
    segment's own duration."""
    for p in points[from_index:]:
        p["t_s"] = p["t_s"] + delta


def _fd_velocities(points: List[dict], j: int) -> List[float]:
    return [
        (points[i + 1]["positions"][j] - points[i]["positions"][j])
        / (points[i + 1]["t_s"] - points[i]["t_s"])
        for i in range(len(points) - 1)
    ]


# ---------------------------------------------------------------------------
# Plan polyline (§16.2 Step 1) -- enough to locate plan vertices and trace-leg point ranges inside
# the trajectory, independently of retime_plan/trajectory_check's own copies of this logic.
# ---------------------------------------------------------------------------


def _within_continuity(a: List[float], b: List[float]) -> bool:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b)) <= _CONTINUITY_TOL_RAD


def _plan_vertices(plan: dict) -> List[Tuple[List[float], int, str]]:
    """`(vertex, leg_index, kind)` in order, dropping each leg's first configuration when within
    continuity tolerance of the previous leg's last (§16.2 Step 1)."""
    legs = sorted(plan["legs"], key=lambda leg: leg["leg_index"])
    out: List[Tuple[List[float], int, str]] = []
    prev: List[float] | None = None
    for leg in legs:
        positions = [[float(x) for x in pos] for pos in leg["positions"]]
        start = 1 if (prev is not None and _within_continuity(positions[0], prev)) else 0
        for pos in positions[start:]:
            out.append((pos, leg["leg_index"], leg["kind"]))
        prev = positions[-1]
    return out


def _match_vertices_to_trajectory(vertices: List[Tuple[List[float], int, str]], points: List[dict]) -> List[int]:
    """Index into `points` of each plan vertex, in order, by bit-exact match (mirrors
    `trajectory_check.check_path`'s own in-order scan). Raises if any vertex is not found."""
    idxs: List[int] = []
    cursor = 0
    for vertex, leg_index, _kind in vertices:
        found = None
        for i in range(cursor, len(points)):
            if all(float(a) == float(b) for a, b in zip(points[i]["positions"], vertex)):
                found = i
                break
        if found is None:
            raise FaultInjectError(f"plan vertex for leg {leg_index} not found bit-exact in trajectory")
        idxs.append(found)
        cursor = found + 1
    return idxs


# ---------------------------------------------------------------------------
# Individual faults
# ---------------------------------------------------------------------------


def fault_segment_overspeed(traj: dict, limits: Dict[str, Tuple[float, float]]) -> dict:
    """One segment 20% too fast: shrink its duration by 20% (`tau *= 0.8`), without touching any
    other segment's duration. Picked as the segment/joint already closest to its raw velocity
    bound, so a 20% speed-up reliably exceeds it (the retimer runs near `limit_fraction.velocity`,
    §16.7, so some segment is always close)."""
    d = copy.deepcopy(traj)
    points = d["points"]
    best: Tuple[float, int] | None = None
    for j, name in enumerate(JOINT_NAMES):
        v_limit, _a_limit = limits[name]
        for i, v in enumerate(_fd_velocities(points, j)):
            frac = abs(v) / v_limit if v_limit > 0 else float("inf")
            if best is None or frac > best[0]:
                best = (frac, i)
    if best is None:
        raise FaultInjectError("trajectory has fewer than 2 points: no segment to speed up")
    _frac, seg = best
    tau = points[seg + 1]["t_s"] - points[seg]["t_s"]
    _shift_tail(points, seg + 1, -0.2 * tau)
    return d


def fault_accel_spike(traj: dict, limits: Dict[str, Tuple[float, float]]) -> dict:
    """Nudge one interior point's joint1 position, chosen as the point whose two neighbouring
    segment durations are smallest (so a given position offset produces the largest finite-
    difference acceleration change), by enough to exceed joint1's raw acceleration bound with a
    comfortable (5x) margin."""
    d = copy.deepcopy(traj)
    points = d["points"]
    n = len(points)
    if n < 3:
        raise FaultInjectError("trajectory has fewer than 3 points: no interior point for an accel spike")

    best: Tuple[float, int] | None = None
    for i in range(1, n - 1):
        tau0 = points[i]["t_s"] - points[i - 1]["t_s"]
        tau1 = points[i + 1]["t_s"] - points[i]["t_s"]
        sensitivity = 1.0 / tau0 + 1.0 / tau1
        if best is None or sensitivity > best[0]:
            best = (sensitivity, i)
    sensitivity, i = best  # type: ignore[misc]

    j = 0  # joint1
    _v_limit, a_limit = limits[JOINT_NAMES[j]]
    tau0 = points[i]["t_s"] - points[i - 1]["t_s"]
    tau1 = points[i + 1]["t_s"] - points[i]["t_s"]
    denom = (tau0 + tau1) / 2.0
    delta = 5.0 * a_limit * denom / sensitivity
    points[i]["positions"][j] = points[i]["positions"][j] + delta
    return d


def fault_dropped_config(traj: dict, plan: dict) -> dict:
    """Remove one interior plan vertex (not the trajectory's first or last point) from `points`,
    breaking G-PATH's "every plan configuration appears bit-exact" guarantee."""
    d = copy.deepcopy(traj)
    points = d["points"]
    vertices = _plan_vertices(plan)
    idxs = _match_vertices_to_trajectory(vertices, points)
    interior = [i for i in idxs if 0 < i < len(points) - 1]
    if not interior:
        raise FaultInjectError("no interior plan vertex to drop")
    target = interior[len(interior) // 2]
    del points[target]
    return d


def fault_corner_cut(traj: dict, plan: dict) -> dict:
    """Replace one interior plan vertex with the midpoint of its trajectory neighbours -- cuts the
    corner instead of reaching the vertex, breaking G-PATH's distance-to-polyline and
    missing-vertex checks."""
    d = copy.deepcopy(traj)
    points = d["points"]
    vertices = _plan_vertices(plan)
    idxs = _match_vertices_to_trajectory(vertices, points)
    interior = [i for i in idxs if 0 < i < len(points) - 1]
    if not interior:
        raise FaultInjectError("no interior plan vertex to cut the corner of")
    target = interior[len(interior) // 2]
    prev_pos = points[target - 1]["positions"]
    next_pos = points[target + 1]["positions"]
    points[target]["positions"] = [(a + b) / 2.0 for a, b in zip(prev_pos, next_pos)]
    return d


def fault_toolspeed_dip(traj: dict, plan: dict) -> dict:
    """Slow one mid-trace-leg segment down by 20x (not the leading or trailing segment of the
    leg), without touching any other segment's duration: a below-band dip in the interior of the
    leg, violating G-TOOLSPEED rule 2 (only a leading/trailing run may be below band)."""
    d = copy.deepcopy(traj)
    points = d["points"]
    vertices = _plan_vertices(plan)
    idxs = _match_vertices_to_trajectory(vertices, points)

    trace_legs = sorted({leg_index for (_v, leg_index, kind), i in zip(vertices, idxs) if kind == "trace"})
    if not trace_legs:
        raise FaultInjectError("plan has no trace leg to dip the tool speed of")
    leg_index = trace_legs[0]
    leg_point_idxs = sorted(i for (_v, li, kind), i in zip(vertices, idxs) if kind == "trace" and li == leg_index)
    first, last = leg_point_idxs[0], leg_point_idxs[-1]
    if last - first < 4:
        raise FaultInjectError(f"trace leg {leg_index} has too few points for a mid-leg dip")
    mid = first + (last - first) // 2
    if mid <= first or mid >= last:
        raise FaultInjectError(f"trace leg {leg_index}: no strictly-interior segment for a mid-leg dip")

    tau = points[mid + 1]["t_s"] - points[mid]["t_s"]
    _shift_tail(points, mid + 1, 19.0 * tau)
    return d


def fault_wrong_limits_sha(traj: dict) -> dict:
    """Forge `limits_file.sha256` to a well-formed but wrong digest, breaking C-PROVENANCE."""
    d = copy.deepcopy(traj)
    real = (d.get("limits_file") or {}).get("sha256") or ("0" * 64)
    forged = ("f" if real[0] != "f" else "0") + real[1:]
    d["limits_file"]["sha256"] = forged
    return d


def fault_t_s_gap(traj: dict, max_period_s: float) -> dict:
    """Inflate one segment's duration to comfortably exceed `sampling.max_period_s`, without
    touching any other segment's duration, breaking G-PERIOD."""
    d = copy.deepcopy(traj)
    points = d["points"]
    if len(points) < 2:
        raise FaultInjectError("trajectory has fewer than 2 points: no segment to inflate")
    seg = len(points) // 2
    tau = points[seg + 1]["t_s"] - points[seg]["t_s"]
    target_tau = max(max_period_s * 5.0, tau * 5.0)
    _shift_tail(points, seg + 1, target_tau - tau)
    return d


FAULTS = (
    "segment_overspeed",
    "accel_spike",
    "dropped_config",
    "corner_cut",
    "toolspeed_dip",
    "wrong_limits_sha",
    "t_s_gap",
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, required=True, help="a retimed crackvision.joint_trajectory/1 file")
    parser.add_argument("--plan", type=Path, required=True, help="the crackvision.cartesian_plan/1 file it was retimed from")
    parser.add_argument("--limits", type=Path, required=True, help="b601_dm_limits.yaml (for the velocity/accel faults)")
    parser.add_argument("--out-dir", type=Path, required=True, help="directory to write fault_<name>.json into")
    parser.add_argument("--max-period-s", type=float, default=0.02, help="retime.yaml sampling.max_period_s (default matches §16.7)")
    return parser


def main(argv: List[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        traj = _load_json(args.trajectory)
        plan = _load_json(args.plan)
        limits = _read_limits(args.limits)
    except FaultInjectError as exc:
        print(f"retime_fault_inject: usage error: {exc}", file=sys.stderr)
        return 2

    args.out_dir.mkdir(parents=True, exist_ok=True)

    builders = {
        "segment_overspeed": lambda: fault_segment_overspeed(traj, limits),
        "accel_spike": lambda: fault_accel_spike(traj, limits),
        "dropped_config": lambda: fault_dropped_config(traj, plan),
        "corner_cut": lambda: fault_corner_cut(traj, plan),
        "toolspeed_dip": lambda: fault_toolspeed_dip(traj, plan),
        "wrong_limits_sha": lambda: fault_wrong_limits_sha(traj),
        "t_s_gap": lambda: fault_t_s_gap(traj, args.max_period_s),
    }

    failed = False
    for name in FAULTS:
        out_path = args.out_dir / f"fault_{name}.json"
        try:
            mutated = builders[name]()
        except FaultInjectError as exc:
            print(f"retime_fault_inject: FAILED to build fault '{name}': {exc}", file=sys.stderr)
            failed = True
            continue
        _dump_json(mutated, out_path)
        print(f"retime_fault_inject: wrote {out_path}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
