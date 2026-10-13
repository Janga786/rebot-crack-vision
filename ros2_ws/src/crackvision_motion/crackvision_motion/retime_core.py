"""crackvision_motion.retime_core — retime config loader, plan-polyline assembly, refinement and
limit-constrained discrete time profiling (MOT-07.3).

Stdlib + numpy + PyYAML only -- no rclpy or `*_msgs` import anywhere in this file, mirroring
`cartesian_plan.py`/`joint_trajectory.py`, so it runs standalone under the system python3 without a
colcon build. Normative source: `docs/INTERFACES.md` §16 and `docs/adr/019-time-parameterization.md`.

This module covers §16.2 Steps 1-3 and G-PERIOD only (plan-polyline assembly, linear refinement,
and the forward-backward velocity-profiling pass enforcing G-VEL/G-ACC/G-REST/the §11.6 margin rule
(M)/G-PERIOD). Step 4 (constant tool speed on trace legs, needing `tool_fk`) is MOT-07.4's job --
`time_profile`'s `caps` argument is exactly the hook that card uses to fold a per-segment tool-speed
cap into this pass without this module needing to know anything about Cartesian space. Steps 5-6
(the full-limit self-check before writing a file, and the `retime_plan`/§16.3-16.4 file writers) are
later MOT-07 cards; `time_profile`'s own internal verification (below) reuses
`joint_trajectory.check_limits` at the *configured* `f_v`/`f_a` fractions purely so the profiling
pass never returns a value it has not itself directly checked, not as a substitute for that later
full-limit G-SELFCHECK.

Algorithm (ADR-019, §16.2 Step 3), in `time_profile`:
  - Each segment `i` (between refined-polyline points `q_i`, `q_{i+1}`) gets a path speed `u_i`,
    `h_i = ||q_{i+1} - q_i||_2` (Euclidean norm over all 6 joints), direction `d_i = (q_{i+1}-q_i)/h_i`
    and duration `tau_i = h_i/u_i`.
  - A forward pass assigns each `u_i` (segment 0 first) the largest value feasible for (V) [a closed
    form per-segment cap], (R) at the first/last segment [closed form, the exact finite-difference
    rest boundary] and a closed-form (M) cap [given the already-fixed previous segment], then caps it
    further by a smooth kinematic reachability surrogate for (A) (`_kinematic_cap`): exact (A) is a
    finite-difference inequality whose feasible set in `u` is not monotone (see that function's
    docstring for why solving it directly as the per-step search target oscillates and never
    converges), so a standard, monotone `v^2 <= v_neighbour^2 + 2*alim*distance` surrogate is used as
    the candidate instead, with exact (A) compliance left entirely to the verification step below.
  - A backward pass mirrors this from the end, only ever lowering each `u_i` further to the same
    surrogate cap given the now-fixed `u_{i+1}`.
  - Because the two passes use a surrogate for (A) and are not jointly solved, the result is verified
    directly afterwards against the *exact* formulas (reusing `joint_trajectory.check_limits` for
    (V)/(A)/(M), plus a direct (R) check); any residual violation scales the implicated segments'
    caps by 0.9 and the whole pass repeats, bounded.
  - Any segment whose resulting `tau_i` exceeds `max_period_s` is subdivided (more points inserted on
    that same straight segment, preserving G-PATH) and the whole profiling pass above is re-run,
    bounded.
A bounded-iteration failure to converge on either loop is a refusal (`RetimeError`), never a
silently-non-compliant profile -- exactly ADR-019's "never clamp, never hide a slowdown" rule.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
import yaml

from .cartesian_plan import CONTINUITY_TOL_RAD
from .joint_trajectory import JOINT_NAMES, check_limits

SCHEMA = "crackvision.retime_config/1"

_N_JOINTS = len(JOINT_NAMES)

# Bounded-iteration caps (ADR-019: "never a partially-converged, silently-non-compliant file").
_MAX_CAP_RETRIES = 60
_MAX_PERIOD_ITERATIONS = 6
_ACCEL_EPS = 1e-9


class RetimeConfigError(ValueError):
    """Raised for any structurally or numerically invalid `config/motion/retime.yaml`."""


class RetimeError(ValueError):
    """Raised for an infeasible plan/polyline/profile input, or a bounded-iteration profiling pass
    that never converges on a verified, guarantee-satisfying result."""


# --------------------------------------------------------------------------------------
# load_retime_config (§16.1, schema `crackvision.retime_config/1`)
# --------------------------------------------------------------------------------------

_TOP_LEVEL_KEYS = frozenset({"schema", "limit_fraction", "sampling", "trace", "value_status", "sources"})
_LIMIT_FRACTION_KEYS = frozenset({"velocity", "acceleration"})
_SAMPLING_KEYS = frozenset({"max_joint_step_rad", "max_period_s"})
_TRACE_KEYS = frozenset({"tool_speed_m_s", "min_tool_speed_m_s", "speed_tolerance", "max_ramp_m"})


def _check_keys(d: object, allowed: frozenset, ctx: str) -> None:
    if not isinstance(d, dict):
        raise RetimeConfigError(f"'{ctx}' must be a mapping")
    unknown = set(d.keys()) - allowed
    if unknown:
        raise RetimeConfigError(f"unknown key(s) in '{ctx}': {sorted(unknown)}")


def _require(d: Dict[str, Any], key: str, ctx: str) -> Any:
    if key not in d:
        raise RetimeConfigError(f"'{ctx}' is missing required key '{key}'")
    return d[key]


def _as_float(v: object, ctx: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise RetimeConfigError(f"'{ctx}' must be numeric, got {v!r}")
    fv = float(v)
    if not math.isfinite(fv):
        raise RetimeConfigError(f"'{ctx}' must be finite, got {v!r}")
    return fv


def load_retime_config(path: Union[str, Path]) -> Dict[str, Any]:
    """Load and validate `config/motion/retime.yaml` (§16.1). Raises `RetimeConfigError` on any
    unknown/missing key or out-of-range value; strict, exactly §13.2's loader convention."""
    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict):
        raise RetimeConfigError("config root must be a mapping")
    _check_keys(raw, _TOP_LEVEL_KEYS, "<root>")

    schema = _require(raw, "schema", "<root>")
    if schema != SCHEMA:
        raise RetimeConfigError(f"'schema' must be {SCHEMA!r}, got {schema!r}")

    lf_raw = _require(raw, "limit_fraction", "<root>")
    _check_keys(lf_raw, _LIMIT_FRACTION_KEYS, "limit_fraction")
    f_v = _as_float(_require(lf_raw, "velocity", "limit_fraction"), "limit_fraction.velocity")
    f_a = _as_float(_require(lf_raw, "acceleration", "limit_fraction"), "limit_fraction.acceleration")
    if not (0.0 < f_v <= 1.0):
        raise RetimeConfigError(f"'limit_fraction.velocity' must lie in (0, 1], got {f_v}")
    if not (0.0 < f_a <= 1.0):
        raise RetimeConfigError(f"'limit_fraction.acceleration' must lie in (0, 1], got {f_a}")

    samp_raw = _require(raw, "sampling", "<root>")
    _check_keys(samp_raw, _SAMPLING_KEYS, "sampling")
    max_joint_step_rad = _as_float(
        _require(samp_raw, "max_joint_step_rad", "sampling"), "sampling.max_joint_step_rad"
    )
    if max_joint_step_rad <= 0:
        raise RetimeConfigError(f"'sampling.max_joint_step_rad' must be > 0, got {max_joint_step_rad}")
    max_period_s = _as_float(_require(samp_raw, "max_period_s", "sampling"), "sampling.max_period_s")
    if max_period_s <= 0:
        raise RetimeConfigError(f"'sampling.max_period_s' must be > 0, got {max_period_s}")

    trace_raw = _require(raw, "trace", "<root>")
    _check_keys(trace_raw, _TRACE_KEYS, "trace")
    tool_speed_m_s = _as_float(_require(trace_raw, "tool_speed_m_s", "trace"), "trace.tool_speed_m_s")
    min_tool_speed_m_s = _as_float(
        _require(trace_raw, "min_tool_speed_m_s", "trace"), "trace.min_tool_speed_m_s"
    )
    if min_tool_speed_m_s <= 0:
        raise RetimeConfigError(f"'trace.min_tool_speed_m_s' must be > 0, got {min_tool_speed_m_s}")
    if not (tool_speed_m_s > min_tool_speed_m_s):
        raise RetimeConfigError(
            f"'trace.tool_speed_m_s' ({tool_speed_m_s}) must be > 'trace.min_tool_speed_m_s' "
            f"({min_tool_speed_m_s})"
        )
    speed_tolerance = _as_float(_require(trace_raw, "speed_tolerance", "trace"), "trace.speed_tolerance")
    if not (0.0 < speed_tolerance < 1.0):
        raise RetimeConfigError(f"'trace.speed_tolerance' must lie in (0, 1), got {speed_tolerance}")
    max_ramp_m = _as_float(_require(trace_raw, "max_ramp_m", "trace"), "trace.max_ramp_m")
    if max_ramp_m <= 0:
        raise RetimeConfigError(f"'trace.max_ramp_m' must be > 0, got {max_ramp_m}")

    value_status = _require(raw, "value_status", "<root>")
    if value_status != "nominal":
        raise RetimeConfigError(f"'value_status' must be 'nominal', got {value_status!r}")

    sources = _require(raw, "sources", "<root>")
    if not isinstance(sources, dict) or not sources:
        raise RetimeConfigError("'sources' must be a non-empty mapping")
    for k, v in sources.items():
        if not isinstance(v, str) or not v:
            raise RetimeConfigError(f"'sources.{k}' must be a non-empty string")

    return {
        "schema": schema,
        "limit_fraction": {"velocity": f_v, "acceleration": f_a},
        "sampling": {"max_joint_step_rad": max_joint_step_rad, "max_period_s": max_period_s},
        "trace": {
            "tool_speed_m_s": tool_speed_m_s,
            "min_tool_speed_m_s": min_tool_speed_m_s,
            "speed_tolerance": speed_tolerance,
            "max_ramp_m": max_ramp_m,
        },
        "value_status": value_status,
        "sources": dict(sources),
    }


# --------------------------------------------------------------------------------------
# plan_polyline (§16.2 Step 1)
# --------------------------------------------------------------------------------------

LegSpan = Tuple[int, str, int, int]


def plan_polyline(plan: Dict[str, Any]) -> Tuple[np.ndarray, List[LegSpan]]:
    """Concatenate `plan["legs"][].positions` in `leg_index` order into the plan polyline `P`
    (§16.2 Step 1), dropping each leg's first configuration when it is within
    `cartesian_plan.CONTINUITY_TOL_RAD` of the previous leg's last. Every surviving row is a
    bit-exact copy of the plan's own float64 values. Returns `(configs, leg_spans)`, `leg_spans`
    a list of `(leg_index, kind, first, last)` inclusive index ranges into `configs`, per §16.4's
    shared-boundary convention (a dropped leg boundary makes `leg[i].last == leg[i+1].first`).

    Raises `RetimeError` for a plan with no `legs[]`, an empty leg, a non-6-vector position or a
    non-finite value -- this function never silently accepts an infeasible plan."""
    legs = plan.get("legs") if isinstance(plan, dict) else None
    if not legs:
        raise RetimeError("plan has no legs[]")

    configs: List[np.ndarray] = []
    spans: List[LegSpan] = []
    for leg in legs:
        positions = leg["positions"]
        if not positions:
            raise RetimeError(f"leg {leg.get('leg_index')!r} has no positions")
        leg_index = leg["leg_index"]
        kind = leg["kind"]
        first_idx: Optional[int] = None
        for j, pos in enumerate(positions):
            pos_arr = np.asarray(pos, dtype=float)
            if pos_arr.shape != (_N_JOINTS,):
                raise RetimeError(
                    f"leg {leg_index!r} position[{j}] must have {_N_JOINTS} joints, got shape {pos_arr.shape}"
                )
            if not np.all(np.isfinite(pos_arr)):
                raise RetimeError(f"leg {leg_index!r} position[{j}] is not finite: {pos!r}")
            if j == 0 and configs and np.max(np.abs(configs[-1] - pos_arr)) <= CONTINUITY_TOL_RAD:
                first_idx = len(configs) - 1
                continue
            configs.append(pos_arr)
            if first_idx is None:
                first_idx = len(configs) - 1
        spans.append((leg_index, kind, first_idx, len(configs) - 1))

    return np.array(configs, dtype=float), spans


# --------------------------------------------------------------------------------------
# refine (§16.2 Step 2, G-PATH)
# --------------------------------------------------------------------------------------

def _remap_spans(spans: Sequence[LegSpan], index_map: Sequence[int]) -> List[LegSpan]:
    return [(leg_index, kind, index_map[first], index_map[last]) for leg_index, kind, first, last in spans]


def refine(
    configs: np.ndarray, leg_spans: Sequence[LegSpan], max_step_rad: float
) -> Tuple[np.ndarray, List[LegSpan]]:
    """Build the refined polyline `Q` (§16.2 Step 2): subdivide every consecutive pair of `configs`
    with equally-spaced linear interpolation so no per-joint step exceeds `max_step_rad`, with no
    corner blending -- `Q` sits exactly on `configs`'s own straight segments (G-PATH). Every original
    row of `configs` reappears bit-exact in the output (the last interpolant of each subdivided
    segment is the original next vertex itself, never a recomputed, possibly-rounded, equal value).
    `leg_spans` are remapped to the new indices and returned alongside."""
    configs = np.asarray(configs, dtype=float)
    if configs.ndim != 2 or configs.shape[1] != _N_JOINTS:
        raise RetimeError(f"configs must be an (N, {_N_JOINTS}) array, got shape {configs.shape}")
    if configs.shape[0] < 2:
        raise RetimeError("configs must have at least 2 points")
    if not np.all(np.isfinite(configs)):
        raise RetimeError("configs contain non-finite values")
    if max_step_rad <= 0:
        raise RetimeError(f"max_step_rad must be > 0, got {max_step_rad!r}")

    m = configs.shape[0]
    new_configs: List[np.ndarray] = [configs[0]]
    index_map = [0]
    for i in range(m - 1):
        q0, q1 = configs[i], configs[i + 1]
        step = float(np.max(np.abs(q1 - q0)))
        n = max(1, math.ceil(step / max_step_rad)) if step > 0 else 1
        for k in range(1, n + 1):
            if k == n:
                new_configs.append(q1)
            else:
                frac = k / n
                new_configs.append(q0 + frac * (q1 - q0))
        index_map.append(len(new_configs) - 1)

    return np.array(new_configs, dtype=float), _remap_spans(leg_spans, index_map)


# --------------------------------------------------------------------------------------
# time_profile (§16.2 Step 3, G-VEL/G-ACC/G-REST/(M)/G-PERIOD)
# --------------------------------------------------------------------------------------

def _vel_cap(d_i: np.ndarray, vlim: np.ndarray) -> float:
    mask = np.abs(d_i) > 1e-12
    if not np.any(mask):
        return math.inf
    return float(np.min(vlim[mask] / np.abs(d_i[mask])))


def _rest_cap(h_i: float, d_i: np.ndarray, alim: np.ndarray) -> float:
    mask = np.abs(d_i) > 1e-12
    if not np.any(mask):
        return math.inf
    return float(np.min(np.sqrt(alim[mask] * h_i / np.abs(d_i[mask]))))


def _margin_cap(
    q_i: np.ndarray, u_prev: float, d_prev: np.ndarray, d_cur: np.ndarray,
    lower: np.ndarray, upper: np.ndarray, margin_rad: float,
) -> float:
    """§11.6 (M): the largest `u_cur` (this segment, leaving point `q_i`) that never speeds up,
    toward a bound `q_i` is already near, relative to the already-fixed arriving segment's speed."""
    cap = math.inf
    v_prev = u_prev * d_prev
    for sign, near in ((1.0, (upper - q_i) <= margin_rad), (-1.0, (q_i - lower) <= margin_rad)):
        w_in = sign * v_prev
        sd = sign * d_cur
        candidate_mask = near & (w_in > 0) & (sd > 1e-12)
        if np.any(candidate_mask):
            cap = min(cap, float(np.min(w_in[candidate_mask] / sd[candidate_mask])))
    return cap


def _kinematic_cap(u_neighbour: float, h_i: float, d_i: np.ndarray, alim: np.ndarray) -> float:
    """Largest `u_i` such that moving between `u_neighbour` (the already-fixed adjacent segment's
    path speed) and `u_i` over this segment's own `h_i` is reachable under the per-joint standard
    kinematic bound `v^2 <= v_neighbour^2 + 2*alim*distance` (`v_j = u*d_j`, `distance_j = h_i*|d_j|`
    for joint `j`), the smooth, closed-form, monotone-in-`u_neighbour` surrogate this module uses
    inside the forward/backward sweep instead of solving the exact finite-difference (A) inequality
    (`a_ij = (v_ij - v_{i-1,j})/avg_tau`) at every step: that exact inequality's feasible set in `u`
    is **not monotone** (`avg_tau` depends on `u` itself, so a segment can be exact-FD-feasible for a
    `u` far below its neighbour's speed -- `avg_tau` so inflated by the slow side that any jump looks
    small -- while infeasible in between), which produced a slow-fast-slow-fast oscillating profile
    that never converged on `G-PERIOD` when used directly as the per-step search target. The smooth
    surrogate is only ever a *candidate*; `_profile_once`'s own verification against
    `joint_trajectory.check_limits`'s exact finite-difference formula, with bounded cap-scaling
    retries, is what actually enforces (A), so a surrogate that is not bit-for-bit the exact formula
    is safe by construction, never silently non-compliant."""
    mask = np.abs(d_i) > 1e-12
    if not np.any(mask):
        return math.inf
    return float(np.min(np.sqrt(u_neighbour * u_neighbour + 2.0 * alim[mask] * h_i / np.abs(d_i[mask]))))


def _forward_backward_pass(
    configs: np.ndarray, h: np.ndarray, d: np.ndarray, vlim: np.ndarray, alim: np.ndarray,
    margin_rad: float, cap_ext: np.ndarray, lower: np.ndarray, upper: np.ndarray,
) -> np.ndarray:
    s = len(h)
    cap_v = np.array([_vel_cap(d[i], vlim) for i in range(s)])
    cap_r = np.array([_rest_cap(h[i], d[i], alim) for i in range(s)])

    u = np.empty(s)
    u[0] = min(cap_v[0], cap_r[0], cap_ext[0])
    for i in range(1, s):
        cap = min(cap_v[i], _margin_cap(configs[i], u[i - 1], d[i - 1], d[i], lower, upper, margin_rad), cap_ext[i])
        if i == s - 1:
            cap = min(cap, cap_r[i])
        u[i] = min(cap, _kinematic_cap(u[i - 1], h[i], d[i], alim))

    u_b = u.copy()
    for i in range(s - 2, -1, -1):
        u_b[i] = min(u_b[i], _kinematic_cap(u_b[i + 1], h[i], d[i], alim))
    return u_b


def _check_rest(u: np.ndarray, tau: np.ndarray, d: np.ndarray, alim: np.ndarray) -> List[Tuple[str, int, None, List[int]]]:
    violations: List[Tuple[str, int, None, List[int]]] = []
    v0 = u[0] * d[0]
    if np.any(np.abs(v0) > alim * tau[0] + _ACCEL_EPS):
        violations.append(("rest_start", 0, None, [0]))
    last = len(u) - 1
    v_last = u[last] * d[last]
    if np.any(np.abs(v_last) > alim * tau[last] + _ACCEL_EPS):
        violations.append(("rest_end", last, None, [last]))
    return violations


def _map_violation(v, s: int) -> Tuple[str, int, str, List[int]]:
    if v.kind == "position":
        return (v.kind, v.index, v.joint, [])
    if v.kind == "velocity":
        return (v.kind, v.index, v.joint, [v.index])
    # "acceleration" and "position_margin" are both centred on an interior point i, implicating the
    # arriving segment (i-1) and the leaving segment (i).
    return (v.kind, v.index, v.joint, [v.index - 1, v.index])


def _profile_once(
    configs: np.ndarray, caps: np.ndarray, lower: np.ndarray, upper: np.ndarray,
    velocity: np.ndarray, acceleration: np.ndarray, f_v: float, f_a: float, margin_rad: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    m = configs.shape[0]
    s = m - 1
    delta = configs[1:] - configs[:-1]
    h = np.linalg.norm(delta, axis=1)
    if np.any(h <= 0):
        raise RetimeError("degenerate zero-length segment in the refined polyline")
    d = delta / h[:, None]

    vlim = velocity * f_v
    alim = acceleration * f_a
    cur_caps = caps.copy()

    for _attempt in range(_MAX_CAP_RETRIES):
        u = _forward_backward_pass(configs, h, d, vlim, alim, margin_rad, cur_caps, lower, upper)
        tau = h / u
        t_s = np.concatenate(([0.0], np.cumsum(tau)))

        traj = {"points": [{"t_s": float(t), "positions": row.tolist()} for t, row in zip(t_s, configs)]}
        scaled_limits = {
            name: {
                "lower": float(lower[j]), "upper": float(upper[j]),
                "velocity": float(vlim[j]), "acceleration": float(alim[j]),
            }
            for j, name in enumerate(JOINT_NAMES)
        }
        violations = [_map_violation(v, s) for v in check_limits(traj, scaled_limits, position_margin_rad=margin_rad)]
        violations += _check_rest(u, tau, d, alim)

        if not violations:
            return u, tau, t_s, cur_caps

        for kind, index, _joint, segments in violations:
            if kind == "position":
                raise RetimeError(
                    f"position violation at point {index} cannot be resolved by retiming (geometry issue)"
                )
            for seg in segments:
                if 0 <= seg < s:
                    # Anchor the shrink to the speed this attempt actually achieved (always
                    # finite), not the raw cap (which is `inf` when no external cap was given --
                    # `inf * 0.9 == inf`, so scaling the cap itself would never tighten anything).
                    cur_caps[seg] = min(cur_caps[seg], u[seg] * 0.9)

    raise RetimeError(f"profiling pass did not converge within {_MAX_CAP_RETRIES} cap-scaling retries")


def _subdivide_segments(
    configs: np.ndarray, caps: np.ndarray, spans: Sequence[LegSpan], over: np.ndarray,
    tau: np.ndarray, max_period_s: float,
) -> Tuple[np.ndarray, np.ndarray, List[LegSpan]]:
    over_set = set(int(i) for i in over)
    s = configs.shape[0] - 1
    new_configs: List[np.ndarray] = [configs[0]]
    new_caps: List[float] = []
    index_map = [0]
    for i in range(s):
        q0, q1 = configs[i], configs[i + 1]
        if i in over_set:
            n = max(2, math.ceil(tau[i] / max_period_s))
            for k in range(1, n + 1):
                if k == n:
                    new_configs.append(q1)
                else:
                    frac = k / n
                    new_configs.append(q0 + frac * (q1 - q0))
                new_caps.append(caps[i])
        else:
            new_configs.append(q1)
            new_caps.append(caps[i])
        index_map.append(len(new_configs) - 1)

    return np.array(new_configs, dtype=float), np.array(new_caps, dtype=float), _remap_spans(spans, index_map)


def time_profile(
    configs: np.ndarray,
    leg_spans: Sequence[LegSpan],
    limits: Dict[str, Dict[str, float]],
    *,
    f_v: float,
    f_a: float,
    margin_rad: float,
    max_period_s: float,
    caps: Optional[Sequence[Optional[float]]] = None,
) -> Tuple[np.ndarray, np.ndarray, List[LegSpan]]:
    """§16.2 Step 3: assign every segment of the refined polyline `configs` a duration so that
    G-VEL, G-ACC, G-REST and the §11.6 margin rule (M) all hold at `f_v`/`f_a`, then subdivide and
    re-profile (bounded) until G-PERIOD (`tau_i <= max_period_s`) holds too. `caps` is an optional
    per-segment external upper bound on path speed `u_i` (rad, same units as `||delta_q||`), e.g. a
    tool-tip speed cap translated into joint-space by a caller such as MOT-07.4; entries may be
    `None` for "no external cap" on that segment. Returns `(configs, t_s, leg_spans)`; `configs` may
    have grown (G-PERIOD subdivision), with `leg_spans` updated to match.

    Raises `RetimeError` for invalid inputs or a bounded-iteration pass that never converges on a
    verified result -- this function never returns an unverified or non-compliant profile."""
    configs = np.asarray(configs, dtype=float)
    if configs.ndim != 2 or configs.shape[1] != _N_JOINTS:
        raise RetimeError(f"configs must be an (N, {_N_JOINTS}) array, got shape {configs.shape}")
    m = configs.shape[0]
    if m < 2:
        raise RetimeError("configs must have at least 2 points")
    if not np.all(np.isfinite(configs)):
        raise RetimeError("configs contain non-finite values")
    if not (0.0 < f_v <= 1.0):
        raise RetimeError(f"f_v must lie in (0, 1], got {f_v!r}")
    if not (0.0 < f_a <= 1.0):
        raise RetimeError(f"f_a must lie in (0, 1], got {f_a!r}")
    if margin_rad < 0:
        raise RetimeError(f"margin_rad must be >= 0, got {margin_rad!r}")
    if max_period_s <= 0:
        raise RetimeError(f"max_period_s must be > 0, got {max_period_s!r}")

    s = m - 1
    if caps is None:
        caps_arr = np.full(s, math.inf)
    else:
        if len(caps) != s:
            raise RetimeError(f"caps length {len(caps)} must equal the number of segments {s}")
        caps_arr = np.array([math.inf if c is None else float(c) for c in caps], dtype=float)
        if np.any(caps_arr <= 0):
            raise RetimeError("every non-null cap must be > 0")

    lower = np.array([limits[j]["lower"] for j in JOINT_NAMES])
    upper = np.array([limits[j]["upper"] for j in JOINT_NAMES])
    velocity = np.array([limits[j]["velocity"] for j in JOINT_NAMES])
    acceleration = np.array([limits[j]["acceleration"] for j in JOINT_NAMES])

    spans = list(leg_spans)
    for _period_iter in range(_MAX_PERIOD_ITERATIONS):
        u, tau, t_s, converged_caps = _profile_once(
            configs, caps_arr, lower, upper, velocity, acceleration, f_v, f_a, margin_rad
        )
        max_tau = float(np.max(tau))
        if max_tau <= max_period_s + 1e-12:
            return configs, t_s, spans

        # Subdivide *every* segment by the same uniform factor (never just the offending ones):
        # `_kinematic_cap`'s allowed per-step speed growth scales with that segment's own `h`, so a
        # freshly-subdivided (short) segment directly adjacent to an untouched (long) one lets the
        # forward/backward sweep's speed jump up at the long segment and get forced back down at
        # the next short one -- a sawtooth that strictly costs time at every such boundary, and
        # which compounded, across repeated locally-targeted subdivide/re-profile rounds, into a
        # profile over 10% slower than the analytic bound (G-NEAROPT) in development, even though a
        # uniformly-fine mesh of the very same final resolution stayed within it.
        #
        # Propagate `converged_caps` (not the original `caps_arr`) into the subdivided mesh: a sharp
        # corner can need many cap-scaling rounds to resolve, and starting the next, finer mesh's
        # own cap-scaling back over from the original (unscaled) caps every period iteration risks
        # exhausting `_MAX_CAP_RETRIES` again from scratch at every round.
        n = max(2, math.ceil(max_tau / max_period_s - 1e-9))
        all_segments = np.arange(configs.shape[0] - 1)
        synthetic_tau = np.full(configs.shape[0] - 1, n * max_period_s)
        configs, caps_arr, spans = _subdivide_segments(
            configs, converged_caps, spans, all_segments, synthetic_tau, max_period_s
        )

    raise RetimeError(f"G-PERIOD not satisfied within {_MAX_PERIOD_ITERATIONS} subdivide/re-profile iterations")
