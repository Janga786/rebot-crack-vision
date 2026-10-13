"""tests/test_trajectory_check_path.py — MOT-07.7 crackvision.trajectory_check extension.

Exercises C-PATH (G-PATH, independently recomputed), C-TOOLSPEED (G-TOOLSPEED's feasible-set
inference) and C-REPORT (retime_report cross-check), each isolating one §16.5 fault, against joint
paths and tool_tip traces built by hand from `crackvision.kinematics`'s own FK (never
`crackvision_motion.tool_fk`).
"""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

import crackvision.kinematics as kin
from crackvision.trajectory_check import (
    JOINT_NAMES,
    _build_projection,
    _interior_vertices_with_turn,
    _leg_point_ranges,
    _toolspeed_feasible_set,
    _toolspeed_rule23_ok,
    _vertex_segments_for_leg,
    build_plan_polyline,
    check_path,
    check_provenance,
    check_report,
    check_toolspeed,
    run_checks,
)

CHAIN = kin.load_chain()
END_EFFECTOR = kin.load_end_effector()

LIMITS_SHA = "l" * 64
EE_SHA = "e" * 64
SCENE_SHA = "s" * 64
PATHS3D = {"path": "data/paths/case1_paths3d.json", "sha256": "p" * 64, "execution_eligible": True}

LIMITS = {
    "joints": {
        "joint1": {"lower": -2.8, "upper": 2.8, "velocity": 1.0, "acceleration": 1.0},
        "joint2": {"lower": -3.14, "upper": 0.0, "velocity": 1.0, "acceleration": 1.0},
        "joint3": {"lower": -3.14, "upper": 0.0, "velocity": 1.0, "acceleration": 1.0},
        "joint4": {"lower": -1.87, "upper": 1.57, "velocity": 1.0, "acceleration": 1.0},
        "joint5": {"lower": -1.57, "upper": 1.57, "velocity": 1.0, "acceleration": 1.0},
        "joint6": {"lower": -3.14, "upper": 3.14, "velocity": 1.0, "acceleration": 1.0},
    }
}
MARGIN_RAD = 0.03
RETIME_CFG = {
    "limit_fraction": {"velocity": 0.95, "acceleration": 0.95},
    "sampling": {"max_joint_step_rad": 1.0, "max_period_s": 1.0},
    "trace": {
        "tool_speed_m_s": 0.05,
        "min_tool_speed_m_s": 0.005,
        "speed_tolerance": 0.05,
        "max_ramp_m": 0.005,
    },
}

HOME = [0.0] * 6
A = [0.3, -0.5, -0.4, 0.2, 0.1, 0.0]
B = [0.5, -0.65, -0.55, 0.3, 0.15, 0.1]


def _tool_tip(q) -> np.ndarray:
    T = CHAIN.fk(q) @ END_EFFECTOR.T_gripper_link_tool_tip
    return T[:3, 3]


def _joint_lerp(a, b, n: int) -> list[list[float]]:
    # Endpoints are the exact plan vertices, bit-for-bit (G-PATH requires `p_k` to reappear
    # bit-exact in the trajectory; `x + (y - x) * 1.0` is not always bit-identical to `y`).
    out = [[float(x + (y - x) * i / n) for x, y in zip(a, b)] for i in range(n + 1)]
    out[0] = [float(x) for x in a]
    out[-1] = [float(x) for x in b]
    return out


def _traj(points, purpose="test", paths3d=PATHS3D, limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA):
    return {
        "schema": "crackvision.joint_trajectory/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "test",
        "purpose": purpose,
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "points": points,
        "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": limits_sha},
        "end_effector_config_sha256": ee_sha,
        "scene_config_sha256": scene_sha,
        "source": {"paths3d": paths3d},
    }


def _plan(legs, paths3d=None, limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA):
    return {
        "schema": "crackvision.cartesian_plan/1",
        "case_id": "case1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "plan_crack_path",
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "sources": {
            "paths3d": paths3d if paths3d is not None else PATHS3D,
            "scene": {"path": "config/scene/scene.yaml", "sha256": scene_sha},
            "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": ee_sha},
            "limits": {"path": "config/robot/b601_dm_limits.yaml", "sha256": limits_sha},
            "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": "c" * 64},
        },
        "parameters": {},
        "start_state": {"joint_names": JOINT_NAMES, "positions_rad": HOME, "source": "start_joints_override"},
        "end_state": {"mode": "none", "positions_rad": None},
        "legs": legs,
        "segments": [],
        "coverage": {},
        "caveats": [],
    }


def _leg(leg_index, kind, positions):
    return {"leg_index": leg_index, "kind": kind, "positions": [list(p) for p in positions]}


def _n_segment_trace_configs(n=6):
    return _joint_lerp(A, B, n)


def _build_case(speeds=None, n=6, extra_points=()):
    """One `transit` leg HOME->A, one `trace` leg A->B (`n` joint-space segments). The trajectory
    follows the plan polyline exactly; `speeds` (len `n`, m/s) set each trace segment's tool_tip
    speed via `t_s`, independently derived from real FK distances, never from a hand-picked
    geometry."""
    configs = _n_segment_trace_configs(n)
    if speeds is None:
        speeds = [0.05] * n
    assert len(speeds) == n

    plan = _plan(
        [
            _leg(0, "transit", [HOME, A]),
            _leg(1, "trace", [A, B]),
        ]
    )

    mid_home_a = [float(h + (a - h) * 0.5) for h, a in zip(HOME, A)]
    points = [{"t_s": 0.0, "positions": HOME}, {"t_s": 1.0, "positions": mid_home_a}, {"t_s": 2.0, "positions": A}]
    t = 2.0
    tool_prev = _tool_tip(A)
    for cfg, v in zip(configs[1:], speeds):
        tool_next = _tool_tip(cfg)
        dist = float(np.linalg.norm(tool_next - tool_prev))
        tau = dist / v if v > 0 else 1.0
        t += tau
        points.append({"t_s": t, "positions": list(cfg)})
        tool_prev = tool_next

    traj = _traj(list(points) + list(extra_points))
    return plan, traj, configs


C = [0.7, -0.8, -0.7, 0.4, 0.2, 0.2]


def _build_revisit_case(trace_speeds=None, n_trace=6):
    """transit HOME->A, approach A->B, trace B->C, retract C->A: the last leg returns to `A`, an
    earlier leg's own vertex, so the plan polyline `P = [HOME, A, B, C, A]` revisits a
    configuration. Exercises the projection's tie-break at that revisited vertex (Finding
    4c399faa89247366)."""
    if trace_speeds is None:
        trace_speeds = [0.05] * n_trace
    assert len(trace_speeds) == n_trace

    plan = _plan(
        [
            _leg(0, "transit", [HOME, A]),
            _leg(1, "approach", [A, B]),
            _leg(2, "trace", [B, C]),
            _leg(3, "retract", [C, A]),
        ]
    )

    points = [{"t_s": 0.0, "positions": HOME}]
    t = 0.0

    for cfg in _joint_lerp(HOME, A, 2)[1:]:
        t += 1.0
        points.append({"t_s": t, "positions": list(cfg)})

    for cfg in _joint_lerp(A, B, 2)[1:]:
        t += 1.0
        points.append({"t_s": t, "positions": list(cfg)})

    trace_configs = _joint_lerp(B, C, n_trace)
    tool_prev = _tool_tip(B)
    for cfg, v in zip(trace_configs[1:], trace_speeds):
        tool_next = _tool_tip(cfg)
        dist = float(np.linalg.norm(tool_next - tool_prev))
        tau = dist / v if v > 0 else 1.0
        t += tau
        points.append({"t_s": t, "positions": list(cfg)})
        tool_prev = tool_next

    for cfg in _joint_lerp(C, A, 2)[1:]:
        t += 1.0
        points.append({"t_s": t, "positions": list(cfg)})

    traj = _traj(points)
    return plan, traj


def _consistent_revisit_report(traj, plan, ranges):
    def _recompute(first, last):
        t = [float(p["t_s"]) for p in traj["points"]]
        duration = t[last] - t[first]
        idxs = list(range(first, last + 1))
        speeds = []
        for a, b in zip(idxs[:-1], idxs[1:]):
            d = math_dist(_tool_tip(traj["points"][a]["positions"]), _tool_tip(traj["points"][b]["positions"]))
            speeds.append(d / (t[b] - t[a]))
        return duration, speeds

    legs = []
    for leg_index, (first, last, kind) in sorted(ranges.items()):
        duration, speeds = _recompute(first, last)
        leg_doc = {"leg_index": leg_index, "kind": kind, "point_range": [first, last], "duration_s": duration}
        if kind == "trace":
            leg_doc["tool_speed"] = {
                "target": 0.05,
                "constant": 0.05,
                "ramp_in_m": 0.0,
                "ramp_out_m": 0.0,
                "min": min(speeds),
                "max": max(speeds),
            }
        legs.append(leg_doc)
    return {"schema": "crackvision.retime_report/1", "legs": legs}


def test_projection_breaks_ties_toward_continuing_progress_at_revisited_vertex():
    plan, traj = _build_revisit_case()
    projection = _build_projection(traj, plan)
    ranges = _leg_point_ranges(projection)

    assert ranges == {
        0: (0, 2, "transit"),
        1: (2, 4, "approach"),
        2: (4, 10, "trace"),
        3: (10, 12, "retract"),
    }
    # The final trajectory point (idx 12) is bit-exact with `A` (P[1] as well as P[4]), but the
    # projection must assign it to the retract leg it actually continues from, not leg 0.
    assert traj["points"][12]["positions"] == A

    path_result = check_path(traj, plan, projection=projection)
    assert path_result["outcome"] == "pass", path_result["detail"]

    toolspeed_result = check_toolspeed(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, projection=projection)
    assert toolspeed_result["outcome"] == "pass", toolspeed_result["detail"]

    report = _consistent_revisit_report(traj, plan, ranges)
    report_result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, report, projection=projection)
    assert report_result["outcome"] == "pass", report_result["detail"]
    for leg in report["legs"]:
        assert leg["point_range"] == list(ranges[leg["leg_index"]][:2])


def _files():
    return {"trajectory_sha256": "x" * 64, "limits": {"sha256": LIMITS_SHA}, "end_effector": {"sha256": EE_SHA}}


# ---------------------------------------------------------------------------
# build_plan_polyline
# ---------------------------------------------------------------------------


def test_build_plan_polyline_drops_shared_boundary():
    plan = _plan([_leg(0, "transit", [HOME, A]), _leg(1, "trace", [A, B])])
    P, cumlen, leg_info = build_plan_polyline(plan)
    assert P == [HOME, A, B]
    assert leg_info == [
        {"leg_index": 0, "kind": "transit", "p_first": 0, "p_last": 1},
        {"leg_index": 1, "kind": "trace", "p_first": 1, "p_last": 2},
    ]
    assert cumlen[0] == 0.0
    assert cumlen[1] == pytest.approx(math_dist(HOME, A))
    assert cumlen[2] == pytest.approx(cumlen[1] + math_dist(A, B))


def math_dist(a, b):
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))


# ---------------------------------------------------------------------------
# C-PATH
# ---------------------------------------------------------------------------


def test_path_clean_trajectory_passes():
    plan, traj, _configs = _build_case()
    result = check_path(traj, plan)
    assert result["outcome"] == "pass", result["detail"]
    assert result["numbers"]["max_distance_rad"] < 1e-9


def test_path_corner_cut_fails():
    plan, traj, _configs = _build_case()
    # Nudge one interior trajectory point off the A->B segment by well over the 1e-6 rad tolerance.
    traj["points"][5]["positions"][2] += 0.01
    result = check_path(traj, plan)
    assert result["outcome"] == "fail"
    assert "tolerance" in result["detail"]


def test_path_skipped_plan_configuration_fails():
    plan, traj, _configs = _build_case()
    # Drop the trajectory point that is bit-exact with the plan's A vertex (the shared boundary).
    traj["points"] = [p for p in traj["points"] if p["positions"] != A]
    result = check_path(traj, plan)
    assert result["outcome"] == "fail"
    assert "not found bit-exact" in result["detail"]


def test_path_missing_endpoint_fails():
    plan, traj, _configs = _build_case()
    traj["points"][-1]["positions"] = list(traj["points"][-1]["positions"])
    traj["points"][-1]["positions"][0] += 0.01
    result = check_path(traj, plan)
    assert result["outcome"] == "fail"


# ---------------------------------------------------------------------------
# C-TOOLSPEED
# ---------------------------------------------------------------------------


def test_toolspeed_no_trace_legs_skips():
    plan = _plan([_leg(0, "transit", [HOME, A])])
    traj = _traj([{"t_s": 0.0, "positions": HOME}, {"t_s": 1.0, "positions": A}])
    result = check_toolspeed(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR)
    assert result["outcome"] == "skip"


def test_toolspeed_clean_constant_speed_passes():
    plan, traj, _configs = _build_case(speeds=[0.05] * 6)
    result = check_toolspeed(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR)
    assert result["outcome"] == "pass", result["detail"]
    leg1 = result["numbers"]["legs"][1]
    assert leg1["F"]  # non-empty feasible set


def test_toolspeed_numbers_record_L_v0_v_max_and_F():
    # §16.5: "numbers for C-TOOLSPEED records, per trace leg, L, v_0, v_max and F".
    plan, traj, _configs = _build_case(speeds=[0.05] * 6)
    result = check_toolspeed(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR)
    leg1 = result["numbers"]["legs"][1]
    assert isinstance(leg1["L"], float)
    assert isinstance(leg1["v_0"], float)
    assert leg1["v_max"] == RETIME_CFG["trace"]["tool_speed_m_s"]
    assert isinstance(leg1["F"], list) and leg1["F"]


def test_toolspeed_overspeed_fails():
    speeds = [0.05, 0.05, 0.05, 0.16, 0.05, 0.05]  # segment 3 far above the band for any v_c <= v_max
    plan, traj, _configs = _build_case(speeds=speeds)
    result = check_toolspeed(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR)
    assert result["outcome"] == "fail"
    assert "feasible set F is empty" in result["detail"]
    assert result["numbers"]["legs"][1]["F"] == []


def test_toolspeed_mid_trace_dip_fails():
    speeds = [0.05, 0.05, 0.001, 0.05, 0.05, 0.05]  # interior segment crawls, far below v_min
    plan, traj, _configs = _build_case(speeds=speeds)
    result = check_toolspeed(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR)
    assert result["outcome"] == "fail"
    assert result["numbers"]["legs"][1]["F"] == []


def test_toolspeed_overlong_ramp_fails():
    # Segment 0 is slow enough to count as a leading ramp at every feasible v_c, but its own real
    # tool arc length (~0.0085 m, computed from FK) exceeds max_ramp_m (0.005 m).
    speeds = [0.01, 0.05, 0.05, 0.05, 0.05, 0.05]
    plan, traj, configs = _build_case(speeds=speeds)
    seg0_len = math_dist(_tool_tip(configs[0]), _tool_tip(configs[1]))
    assert seg0_len > RETIME_CFG["trace"]["max_ramp_m"]  # the fault actually engages ramp rule 3

    result = check_toolspeed(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR)
    assert result["outcome"] == "fail"
    assert result["numbers"]["legs"][1]["F"] == []


# ---------------------------------------------------------------------------
# C-REPORT
# ---------------------------------------------------------------------------


def _recompute_report_leg(traj, plan, leg_index, first, last):
    t = [float(p["t_s"]) for p in traj["points"]]
    duration = t[last] - t[first]
    idxs = list(range(first, last + 1))
    speeds = []
    for a, b in zip(idxs[:-1], idxs[1:]):
        d = math_dist(_tool_tip(traj["points"][a]["positions"]), _tool_tip(traj["points"][b]["positions"]))
        speeds.append(d / (t[b] - t[a]))
    return duration, speeds


def _consistent_report(traj, plan):
    duration0, _ = _recompute_report_leg(traj, plan, 0, 0, 2)
    duration1, speeds1 = _recompute_report_leg(traj, plan, 1, 2, 8)
    return {
        "schema": "crackvision.retime_report/1",
        "legs": [
            {"leg_index": 0, "kind": "transit", "point_range": [0, 2], "duration_s": duration0},
            {
                "leg_index": 1,
                "kind": "trace",
                "point_range": [2, 8],
                "duration_s": duration1,
                "tool_speed": {
                    "target": 0.05,
                    "constant": 0.05,
                    "ramp_in_m": 0.0,
                    "ramp_out_m": 0.0,
                    "min": min(speeds1),
                    "max": max(speeds1),
                },
            },
        ],
    }


def test_report_missing_skips_other_checks_still_run():
    plan, traj, _configs = _build_case()
    result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, None)
    assert result["outcome"] == "skip"

    report = run_checks(
        traj, plan, limits=LIMITS, margin_rad=MARGIN_RAD, retime_cfg=RETIME_CFG, files=_files(),
        extended=True, chain=CHAIN, end_effector=END_EFFECTOR, report_doc=None,
    )
    outcomes = {c["check"]: c["outcome"] for c in report["checks"]}
    assert outcomes["C-REPORT"] == "skip"
    assert outcomes["C-PATH"] == "pass"
    assert outcomes["C-TOOLSPEED"] == "pass"


def test_report_consistent_passes():
    plan, traj, _configs = _build_case()
    report = _consistent_report(traj, plan)
    result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, report)
    assert result["outcome"] == "pass", result["detail"]


def test_report_wrong_point_range_fails():
    plan, traj, _configs = _build_case()
    report = _consistent_report(traj, plan)
    report["legs"][1]["point_range"] = [2, 7]  # wrong, should be [2, 8]
    result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, report)
    assert result["outcome"] == "fail"
    assert "point_range" in result["detail"]


def test_report_wrong_duration_fails():
    plan, traj, _configs = _build_case()
    report = _consistent_report(traj, plan)
    report["legs"][1]["duration_s"] += 10.0
    result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, report)
    assert result["outcome"] == "fail"
    assert "duration_s" in result["detail"]


def test_report_wrong_tool_speed_constant_fails():
    plan, traj, _configs = _build_case()
    report = _consistent_report(traj, plan)
    report["legs"][1]["tool_speed"]["constant"] = 1.0  # nowhere near F
    result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, report)
    assert result["outcome"] == "fail"
    assert "tool_speed.constant" in result["detail"]


def test_report_wrong_tool_speed_min_max_fails():
    plan, traj, _configs = _build_case()
    report = _consistent_report(traj, plan)
    report["legs"][1]["tool_speed"]["max"] += 1.0
    result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, report)
    assert result["outcome"] == "fail"
    assert "tool_speed.max" in result["detail"]


def test_report_missing_point_range_fails_without_crashing():
    plan, traj, _configs = _build_case()
    report = _consistent_report(traj, plan)
    del report["legs"][0]["point_range"]
    result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, report)
    assert result["outcome"] == "fail"
    assert "point_range" in result["detail"]


def test_report_non_list_point_range_fails_without_crashing():
    plan, traj, _configs = _build_case()
    report = _consistent_report(traj, plan)
    report["legs"][0]["point_range"] = "not-a-list"
    result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, report)
    assert result["outcome"] == "fail"
    assert "point_range" in result["detail"]


def test_report_non_dict_tool_speed_fails_without_crashing():
    plan, traj, _configs = _build_case()
    report = _consistent_report(traj, plan)
    report["legs"][1]["tool_speed"] = [0.05]
    result = check_report(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR, report)
    assert result["outcome"] == "fail"
    assert "tool_speed" in result["detail"]


def test_cli_malformed_report_exits_precondition_not_runtime(tmp_path):
    from crackvision.config import find_root
    from crackvision.trajectory_check import main

    plan, traj, _configs = _build_case()
    report = _consistent_report(traj, plan)
    report["legs"][1]["tool_speed"] = "oops"  # non-dict: must not crash the CLI

    traj_path = tmp_path / "traj.json"
    plan_path = tmp_path / "plan.json"
    report_path = tmp_path / "report.json"
    out_path = tmp_path / "out.check.json"
    traj_path.write_text(json.dumps(traj), encoding="utf-8")
    plan_path.write_text(json.dumps(plan), encoding="utf-8")
    report_path.write_text(json.dumps(report), encoding="utf-8")

    code = main(
        [
            "--root", str(find_root()),
            "--trajectory", str(traj_path),
            "--plan", str(plan_path),
            "--report", str(report_path),
            "--out", str(out_path),
        ]
    )
    assert code == 3  # EXIT_PRECONDITION, not EXIT_RUNTIME (1)
    written = json.loads(out_path.read_text(encoding="utf-8"))
    by_check = {c["check"]: c["outcome"] for c in written["checks"]}
    assert by_check["C-REPORT"] == "fail"


# ---------------------------------------------------------------------------
# run_checks: extended wiring and schema-fail skip cascade
# ---------------------------------------------------------------------------


def test_run_checks_default_omits_extended_checks():
    plan, traj, _configs = _build_case()
    report = run_checks(traj, plan, limits=LIMITS, margin_rad=MARGIN_RAD, retime_cfg=RETIME_CFG, files=_files())
    outcomes = {c["check"]: c["outcome"] for c in report["checks"]}
    assert "C-PATH" not in outcomes
    assert "C-TOOLSPEED" not in outcomes
    assert "C-REPORT" not in outcomes


def test_run_checks_extended_runs_all_three_new_checks():
    plan, traj, _configs = _build_case()
    report = run_checks(
        traj, plan, limits=LIMITS, margin_rad=MARGIN_RAD, retime_cfg=RETIME_CFG, files=_files(),
        extended=True, chain=CHAIN, end_effector=END_EFFECTOR, report_doc=_consistent_report(traj, plan),
    )
    outcomes = {c["check"]: c["outcome"] for c in report["checks"]}
    assert outcomes["C-PATH"] == "pass"
    assert outcomes["C-TOOLSPEED"] == "pass"
    assert outcomes["C-REPORT"] == "pass"


def test_run_checks_schema_fail_skips_new_checks_too():
    plan, traj, _configs = _build_case()
    traj["points"][1]["t_s"] = -1.0  # breaks monotone t_s -> C-SCHEMA fail
    report = run_checks(
        traj, plan, limits=LIMITS, margin_rad=MARGIN_RAD, retime_cfg=RETIME_CFG, files=_files(),
        extended=True, chain=CHAIN, end_effector=END_EFFECTOR, report_doc=_consistent_report(traj, plan),
    )
    outcomes = {c["check"]: c["outcome"] for c in report["checks"]}
    assert outcomes["C-SCHEMA"] == "fail"
    for name in ("C-PATH", "C-TOOLSPEED", "C-REPORT"):
        assert outcomes[name] == "skip"


# ---------------------------------------------------------------------------
# C-PROVENANCE: FK file shas recorded, mismatch still fails (§16.5/MOT-07.7)
# ---------------------------------------------------------------------------


def test_provenance_records_fk_file_shas():
    plan, traj, _configs = _build_case()
    files = dict(_files())
    files["fk_files"] = {"urdf_sha256": "u" * 64, "end_effector_sha256": EE_SHA}
    result = check_provenance(traj, plan, files)
    assert result["outcome"] == "pass"
    assert result["numbers"]["cli_files"]["fk_files"] == files["fk_files"]


def test_provenance_end_effector_sha_mismatch_still_fails_with_fk_files_recorded():
    plan, traj, _configs = _build_case()
    files = dict(_files())
    files["end_effector"] = {"sha256": "wrong" + "0" * 59}
    files["fk_files"] = {"urdf_sha256": "u" * 64, "end_effector_sha256": "wrong" + "0" * 59}
    result = check_provenance(traj, plan, files)
    assert result["outcome"] == "fail"
    assert "end_effector_config_sha256" in result["detail"]


# ---------------------------------------------------------------------------
# G-TOOLSPEED vertex-tied dips (MOT-07.11, ADR-019 addendum): C-TOOLSPEED mirrors retime_plan
# ---------------------------------------------------------------------------


def _sharp_turn_joint_path(n_side: int, step: float, theta_deg: float = 20.0) -> list[list[float]]:
    # Same pattern as ros2_ws/src/crackvision_motion/test/test_retime.py's helper of this name:
    # three straight runs joined by two sharp joint-space turns at indices n_side and 2*n_side.
    theta = math.radians(theta_deg)
    dirs = [
        np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([math.cos(theta), 0.0, 0.0, math.sin(theta), 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ]
    pts = [np.zeros(6)]
    for d in dirs:
        for _ in range(n_side):
            pts.append(pts[-1] + step * d)
    return [p.tolist() for p in pts]


def test_toolspeed_rule23_vertex_tied_dips_mirror_retimer_case():
    # Identical segment pattern, chords and outcomes to test_retime.py's
    # test_check_toolspeed_vertex_tied_dips_bounded_and_unrelated_dip_refused.
    configs = _sharp_turn_joint_path(n_side=60, step=0.005)
    vertices = _interior_vertices_with_turn(configs)
    assert vertices == [configs[60], configs[120]]  # straight-run rounding is not a turn
    vertex_segments = _vertex_segments_for_leg(vertices, configs)
    assert vertex_segments == frozenset({59, 60, 119, 120})

    n = len(configs) - 1
    v_c, tol, max_ramp_m = 0.02, 0.05, 0.005

    def case(dips, tol_=tol):
        speeds = [0.5 * v_c if i in set(dips) else v_c for i in range(n)]
        taus = [0.0009 / s for s in speeds]  # every segment's tool arc length is 0.9 mm
        return _toolspeed_rule23_ok(speeds, taus, tol_, v_c, max_ramp_m, vertex_segments)

    assert case([59, 60, 119, 120])
    assert not _toolspeed_rule23_ok(
        [0.5 * v_c if i in {59, 60, 119, 120} else v_c for i in range(n)],
        [0.0009 / (0.5 * v_c if i in {59, 60, 119, 120} else v_c) for i in range(n)],
        tol, v_c, max_ramp_m, frozenset(),
    )
    assert not case([89, 90])  # mid-run dip, no vertex
    assert not case(range(57, 63))  # vertex-tied but 5.4 mm > max_ramp_m
    share = list(range(58, 63)) + list(range(118, 123))
    assert not case(share)  # each 4.5 mm, together 5.6% of the leg > tol
    assert case(share, tol_=0.06)


def _build_sharp_turn_case(dip_segments, n_side=60, v=0.02):
    """transit HOME->A, then a trace leg whose plan polyline is A -> V1 -> V2 -> E with 20 deg
    joint-space turns at V1 and V2; the trajectory samples each straight run at `n_side` steps
    (plan vertices bit-exact) and runs each trace segment at tool speed `v`, except
    `dip_segments` (leg-local segment indices) at `0.5 * v`."""
    a = np.asarray(A, dtype=float)
    d0 = (np.asarray(B) - a) / np.linalg.norm(np.asarray(B) - a)
    e = np.zeros(6)
    e[5] = 1.0
    perp = e - np.dot(e, d0) * d0
    perp /= np.linalg.norm(perp)
    theta = math.radians(20.0)
    run = 0.3
    v1 = (a + run * d0).tolist()
    v2 = (np.asarray(v1) + run * (math.cos(theta) * d0 + math.sin(theta) * perp)).tolist()
    end = (np.asarray(v2) + run * d0).tolist()
    trace_plan = [list(A), v1, v2, end]
    plan = _plan([_leg(0, "transit", [HOME, A]), _leg(1, "trace", trace_plan)])

    configs = _joint_lerp(A, v1, n_side) + _joint_lerp(v1, v2, n_side)[1:] + _joint_lerp(v2, end, n_side)[1:]
    points = [{"t_s": 0.0, "positions": HOME}, {"t_s": 1.0, "positions": list(A)}]
    t = 1.0
    chords = []
    for i in range(len(configs) - 1):
        chord = float(np.linalg.norm(_tool_tip(configs[i + 1]) - _tool_tip(configs[i])))
        chords.append(chord)
        t += chord / (0.5 * v if i in set(dip_segments) else v)
        points.append({"t_s": t, "positions": configs[i + 1]})
    return plan, _traj(points), chords


def test_toolspeed_vertex_tied_dips_pass_and_mid_run_dip_fails():
    dips = [59, 60, 119, 120]  # the segments either side of V1 (row 60) and V2 (row 120)
    plan, traj, chords = _build_sharp_turn_case(dips)
    max_ramp_m = RETIME_CFG["trace"]["max_ramp_m"]
    tol = RETIME_CFG["trace"]["speed_tolerance"]
    assert chords[59] + chords[60] <= max_ramp_m and chords[119] + chords[120] <= max_ramp_m
    assert sum(chords[i] for i in dips) <= tol * sum(chords)

    result = check_toolspeed(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR)
    assert result["outcome"] == "pass", result["detail"]
    leg1 = result["numbers"]["legs"][1]
    assert leg1["vertex_segments"] == [59, 60, 119, 120]
    assert leg1["F"] and leg1["F"][0][0] <= 0.02 <= leg1["F"][-1][1]
    # The pre-MOT-07.11 rule (no vertex-tied dips) finds no feasible v_c for the same speeds.
    first, last = leg1["point_range"]
    taus = [traj["points"][i + 1]["t_s"] - traj["points"][i]["t_s"] for i in range(first, last)]
    F_old, _L, _v0 = _toolspeed_feasible_set(
        leg1["speeds"], taus, tol, RETIME_CFG["trace"]["min_tool_speed_m_s"],
        RETIME_CFG["trace"]["tool_speed_m_s"], max_ramp_m,
    )
    assert F_old == []

    plan, traj, _chords = _build_sharp_turn_case([89, 90])  # middle of the V1-V2 run
    result = check_toolspeed(traj, plan, RETIME_CFG, CHAIN, END_EFFECTOR)
    assert result["outcome"] == "fail"
    assert result["numbers"]["legs"][1]["F"] == []


def test_leg_ranges_do_not_claim_a_point_just_short_of_the_boundary():
    # MOT-07.11: the fixture's retimed trajectory has a point 9.4e-6 rad before its
    # retract->transit rest vertex; it lies on the earlier leg's own last segment only.
    a = np.asarray(A, dtype=float)
    near = (a + (np.asarray(HOME) - a) * (9.4e-6 / np.linalg.norm(np.asarray(HOME) - a))).tolist()
    plan = _plan([_leg(0, "transit", [HOME, A]), _leg(1, "trace", [A, B])])
    traj = _traj([
        {"t_s": 0.0, "positions": HOME},
        {"t_s": 1.0, "positions": near},
        {"t_s": 1.1, "positions": list(A)},
        {"t_s": 2.0, "positions": list(B)},
    ])
    ranges = _leg_point_ranges(_build_projection(traj, plan))
    assert ranges[0][:2] == (0, 2)
    assert ranges[1][:2] == (2, 3)
