"""survey_core — pure-Python validation + derivation for a `crackvision.workcell_survey/1`
record (MOT-10.2). Stdlib + PyYAML only -- no rclpy or *_msgs import anywhere in this file
(mirrors scene_core.py/reachability_core.py), so it runs standalone under the system python3
without a colcon build.

Normative source: docs/INTERFACES.md §12 (schema: §12.1-12.6; derivation + refusal rules:
§12.7). That section is the single source of truth for every formula and refusal condition
below; comments here cite rule numbers but do not restate the whole section.

Public API:
    `SurveyError`            -- raised for any structurally/numerically invalid survey record,
                                 and (from `derive_scene`) any §12.7 rule-6 refusal. Always names
                                 the offending field.
    `SCHEMA_ID`               -- "crackvision.workcell_survey/1".
    `BASE_KEEPOUT_M`          -- default base keep-out rectangle (x_min, x_max, y_min, y_max) in
                                 base_link, used by the rule-6(d) obstacle/keep-out refusal check.
                                 This mirrors config/motion/reachability.yaml's
                                 `surface_collision.base_keepout_m` (base_link.STL footprint,
                                 x +/-0.070 m / y +/-0.100 m, §12.1, plus 0.02 m clearance) for
                                 consistency with the rest of the stack; survey_core does not read
                                 that file (it is not a §12 input), so the value is a module
                                 constant here, overridable via `derive_scene`'s keyword argument.
    `load_survey(path)`       -- loads + validates the YAML file, returns the record as nested
                                  dicts/lists of plain Python values (never invents a value: every
                                  reading is the full 4-key map, unknown keys are rejected).
    `survey_sha256(path)`     -- sha256 hex digest of the raw file bytes.
    `derive_scene(survey, survey_sha, base_keepout_m=BASE_KEEPOUT_M)` -- applies §12.7's
                                  derivation + refusal rules to a validated record, returning
                                  `{'objects': [...], 'allowed_collisions': [...], 'derived': {...}}`.
                                  Every object/allowed_collisions entry is `value_status: measured`
                                  with a `source` citing `survey_sha` and the specific fields used,
                                  and is shaped to pass `scene_core.load_config`'s validation.

Uncertainty propagation: every derived scalar's 1-sigma uncertainty is computed by `_propagate`,
a first-order (linearized) central-difference Jacobian over the exact underlying readings that
feed it. This is exact for the (many) derivations that are linear in their inputs (centre,
footprint sides are not, but table/obstacle box edges, dimensions and positions are) and a
good first-order approximation for the handful that are not (yaw, footprint_m, which go through
sqrt/atan2). Passing a shared reading (e.g. `base_mounting.adapter_plate_thickness`, which
appears in both an obstacle's z_min and z_max) to `_propagate` exactly once, with the function
using it twice internally, correctly captures the resulting correlation (e.g. it cancels out of
an obstacle box's z-extent, since both faces share the same table-top datum).

NOTE on §12.7 rule 2 ("Specimen top z = adapter_plate_thickness.value_m (or 0) +
mean(thickness_readings)"): read literally, this adds the adapter thickness with the opposite
sign from rule 3's `z_table_top = -(adapter_plate_thickness.value_m or 0)` datum that the same
rule 2 result must sit on. This module implements the physically consistent reading -- "bottom on
the table top" (this card's own wording) -- i.e. `top_z = z_table_top + mean(thickness_readings)`,
which matches rule 3's datum and reduces to rule 2's literal formula whenever
`bolted_directly_to_table` is true (adapter term is 0 either way). Flagged for the §12 owner;
MOT-10.1/INTERFACES.md are out of this card's scope.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple, Union

import yaml

SCHEMA_ID = "crackvision.workcell_survey/1"

BASE_KEEPOUT_M: Tuple[float, float, float, float] = (-0.09, 0.09, -0.12, 0.12)

_TOP_LEVEL_KEYS = frozenset(
    {"schema", "survey", "derivation_params", "base_mounting", "table", "specimen", "obstacles", "acm_observations"}
)
_SURVEY_KEYS = frozenset({"operator", "date", "photos", "notes"})
_DERIVATION_PARAMS_KEYS = frozenset({"rectangularity_tolerance_m"})
_BASE_MOUNTING_KEYS = frozenset({"bolted_directly_to_table", "adapter_plate_thickness"})
_TABLE_KEYS = frozenset(
    {
        "front_face_to_far_edge",
        "front_face_to_near_edge",
        "pos_y_side_face_to_edge",
        "neg_y_side_face_to_edge",
        "thickness",
        "flatness_deviation",
        "flatness_note",
    }
)
_SPECIMEN_KEYS = frozenset({"corners", "thickness_readings", "resting_on_table"})
_CORNER_KEYS = frozenset({"x_from_front_face", "y_from_centerline"})
_OBSTACLES_KEYS = frozenset({"radius_m", "items"})
_OBSTACLE_ITEM_KEYS = frozenset({"id", "description", "x_from_front_face", "y_from_centerline", "z_from_table_top"})
_MINMAX_KEYS = frozenset({"min", "max"})
_ACM_OBS_KEYS = frozenset({"base_bolted_to_table", "specimen_resting_on_table", "notes"})
_READING_KEYS = frozenset({"value_m", "instrument", "resolution_m", "uncertainty_1sigma_m"})


class SurveyError(ValueError):
    """Raised for any structurally/numerically invalid survey record, or (from `derive_scene`)
    any §12.7 rule-6 refusal condition."""


def _check_keys(d: object, allowed: frozenset, ctx: str) -> None:
    if not isinstance(d, dict):
        raise SurveyError(f"'{ctx}' must be a mapping")
    unknown = set(d.keys()) - allowed
    if unknown:
        raise SurveyError(f"unknown key(s) in '{ctx}': {sorted(unknown)}")


def _require(d: dict, key: str, ctx: str) -> Any:
    if key not in d:
        raise SurveyError(f"'{ctx}' is missing required key '{key}'")
    return d[key]


def _require_bool(d: dict, key: str, ctx: str) -> bool:
    value = _require(d, key, ctx)
    if not isinstance(value, bool):
        raise SurveyError(f"'{ctx}.{key}' must be a bool, got {value!r}")
    return value


def _require_str(d: dict, key: str, ctx: str) -> str:
    value = _require(d, key, ctx)
    if not isinstance(value, str):
        raise SurveyError(f"'{ctx}.{key}' must be a string, got {value!r}")
    return value


def _require_float(d: dict, key: str, ctx: str) -> float:
    value = _require(d, key, ctx)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SurveyError(f"'{ctx}.{key}' must be a number, got {value!r}")
    return float(value)


def _validate_reading(raw: Any, ctx: str) -> Dict[str, Any]:
    """Validate the 4-key {value_m, instrument, resolution_m, uncertainty_1sigma_m} reading map
    (§12.2). Any of the four being null (or the map itself being null) is an incomplete reading
    (§12.7 rule 6) and blocks every derived value depending on it."""
    if raw is None:
        raise SurveyError(f"'{ctx}' is an incomplete reading (null)")
    _check_keys(raw, _READING_KEYS, ctx)
    value_m = _require_float(raw, "value_m", ctx)
    instrument = _require_str(raw, "instrument", ctx)
    if not instrument:
        raise SurveyError(f"'{ctx}.instrument' must be a non-empty string")
    resolution_m = _require_float(raw, "resolution_m", ctx)
    if resolution_m <= 0.0:
        raise SurveyError(f"'{ctx}.resolution_m' must be positive, got {resolution_m}")
    uncertainty_1sigma_m = _require_float(raw, "uncertainty_1sigma_m", ctx)
    if uncertainty_1sigma_m < 0.0:
        raise SurveyError(f"'{ctx}.uncertainty_1sigma_m' must be >= 0, got {uncertainty_1sigma_m}")
    return {
        "value_m": value_m,
        "instrument": instrument,
        "resolution_m": resolution_m,
        "uncertainty_1sigma_m": uncertainty_1sigma_m,
    }


def _validate_survey_block(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _SURVEY_KEYS, ctx)
    operator = _require_str(raw, "operator", ctx)
    date = _require_str(raw, "date", ctx)
    photos = _require(raw, "photos", ctx)
    if not isinstance(photos, list) or not all(isinstance(p, str) for p in photos):
        raise SurveyError(f"'{ctx}.photos' must be a list of strings")
    notes = _require_str(raw, "notes", ctx)
    return {"operator": operator, "date": date, "photos": list(photos), "notes": notes}


def _validate_derivation_params(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _DERIVATION_PARAMS_KEYS, ctx)
    tolerance = _require_float(raw, "rectangularity_tolerance_m", ctx)
    if tolerance <= 0.0:
        raise SurveyError(f"'{ctx}.rectangularity_tolerance_m' must be positive, got {tolerance}")
    return {"rectangularity_tolerance_m": tolerance}


def _validate_base_mounting(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _BASE_MOUNTING_KEYS, ctx)
    bolted = _require_bool(raw, "bolted_directly_to_table", ctx)
    adapter_raw = _require(raw, "adapter_plate_thickness", ctx)
    if bolted:
        # §12.7 rule 6: adapter_plate_thickness is exempt from the null-check (and, per rules
        # 2/3, treated as 0) whenever bolted_directly_to_table is true -- whatever is recorded
        # here is not used.
        adapter = None
    else:
        adapter = _validate_reading(adapter_raw, f"{ctx}.adapter_plate_thickness")
    return {"bolted_directly_to_table": bolted, "adapter_plate_thickness": adapter}


def _validate_table(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _TABLE_KEYS, ctx)
    out: Dict[str, Any] = {}
    for key in (
        "front_face_to_far_edge",
        "front_face_to_near_edge",
        "pos_y_side_face_to_edge",
        "neg_y_side_face_to_edge",
        "thickness",
        "flatness_deviation",
    ):
        out[key] = _validate_reading(_require(raw, key, ctx), f"{ctx}.{key}")
    out["flatness_note"] = _require_str(raw, "flatness_note", ctx)
    return out


def _validate_corner(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _CORNER_KEYS, ctx)
    return {
        "x_from_front_face": _validate_reading(_require(raw, "x_from_front_face", ctx), f"{ctx}.x_from_front_face"),
        "y_from_centerline": _validate_reading(_require(raw, "y_from_centerline", ctx), f"{ctx}.y_from_centerline"),
    }


def _validate_specimen(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _SPECIMEN_KEYS, ctx)
    corners_raw = _require(raw, "corners", ctx)
    if not isinstance(corners_raw, list) or len(corners_raw) != 4:
        raise SurveyError(f"'{ctx}.corners' must be a list of exactly 4 corners")
    corners = [_validate_corner(c, f"{ctx}.corners[{i}]") for i, c in enumerate(corners_raw)]
    thickness_raw = _require(raw, "thickness_readings", ctx)
    if not isinstance(thickness_raw, list) or len(thickness_raw) < 3:
        raise SurveyError(f"'{ctx}.thickness_readings' must have at least 3 readings (§12.7 rule 6)")
    thickness_readings = [_validate_reading(t, f"{ctx}.thickness_readings[{i}]") for i, t in enumerate(thickness_raw)]
    resting_on_table = _require_bool(raw, "resting_on_table", ctx)
    return {"corners": corners, "thickness_readings": thickness_readings, "resting_on_table": resting_on_table}


def _validate_minmax_reading(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _MINMAX_KEYS, ctx)
    minimum = _validate_reading(_require(raw, "min", ctx), f"{ctx}.min")
    maximum = _validate_reading(_require(raw, "max", ctx), f"{ctx}.max")
    if minimum["value_m"] > maximum["value_m"]:
        raise SurveyError(f"'{ctx}': min.value_m ({minimum['value_m']}) must be <= max.value_m ({maximum['value_m']})")
    return {"min": minimum, "max": maximum}


def _validate_obstacle_item(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _OBSTACLE_ITEM_KEYS, ctx)
    item_id = _require_str(raw, "id", ctx)
    if not item_id:
        raise SurveyError(f"'{ctx}.id' must be a non-empty string")
    description = _require_str(raw, "description", ctx)
    x_from_front_face = _validate_minmax_reading(_require(raw, "x_from_front_face", ctx), f"{ctx}.x_from_front_face")
    y_from_centerline = _validate_minmax_reading(_require(raw, "y_from_centerline", ctx), f"{ctx}.y_from_centerline")
    z_from_table_top = _validate_minmax_reading(_require(raw, "z_from_table_top", ctx), f"{ctx}.z_from_table_top")
    return {
        "id": item_id,
        "description": description,
        "x_from_front_face": x_from_front_face,
        "y_from_centerline": y_from_centerline,
        "z_from_table_top": z_from_table_top,
    }


def _validate_obstacles(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _OBSTACLES_KEYS, ctx)
    radius_m = _require_float(raw, "radius_m", ctx)
    if radius_m <= 0.0:
        raise SurveyError(f"'{ctx}.radius_m' must be positive, got {radius_m}")
    items_raw = _require(raw, "items", ctx)
    if not isinstance(items_raw, list):
        raise SurveyError(f"'{ctx}.items' must be a list")
    items = [_validate_obstacle_item(it, f"{ctx}.items[{i}]") for i, it in enumerate(items_raw)]
    seen_ids = set()
    for item in items:
        if item["id"] in seen_ids:
            raise SurveyError(f"'{ctx}.items': duplicate obstacle id {item['id']!r}")
        seen_ids.add(item["id"])
    return {"radius_m": radius_m, "items": items}


def _validate_acm_observations(raw: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(raw, _ACM_OBS_KEYS, ctx)
    base_bolted_to_table = _require_bool(raw, "base_bolted_to_table", ctx)
    specimen_resting_on_table = _require_bool(raw, "specimen_resting_on_table", ctx)
    notes = _require_str(raw, "notes", ctx)
    return {
        "base_bolted_to_table": base_bolted_to_table,
        "specimen_resting_on_table": specimen_resting_on_table,
        "notes": notes,
    }


def load_survey(path: Union[str, Path]) -> Dict[str, Any]:
    """Load and validate a `crackvision.workcell_survey/1` YAML file (§12.2-12.6). Raises
    SurveyError on any structural or numeric problem, including any reading anywhere in the file
    left incomplete (null) -- except `base_mounting.adapter_plate_thickness` when
    `bolted_directly_to_table` is true (§12.7 rule 6). Never invents a value."""
    path = Path(path)
    with path.open("r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    if not isinstance(raw, dict):
        raise SurveyError("workcell survey must be a mapping at the top level")
    _check_keys(raw, _TOP_LEVEL_KEYS, "<root>")

    schema = _require(raw, "schema", "<root>")
    if schema != SCHEMA_ID:
        raise SurveyError(f"'schema' must be {SCHEMA_ID!r}, got {schema!r}")

    return {
        "schema": schema,
        "survey": _validate_survey_block(_require(raw, "survey", "<root>"), "survey"),
        "derivation_params": _validate_derivation_params(_require(raw, "derivation_params", "<root>"), "derivation_params"),
        "base_mounting": _validate_base_mounting(_require(raw, "base_mounting", "<root>"), "base_mounting"),
        "table": _validate_table(_require(raw, "table", "<root>"), "table"),
        "specimen": _validate_specimen(_require(raw, "specimen", "<root>"), "specimen"),
        "obstacles": _validate_obstacles(_require(raw, "obstacles", "<root>"), "obstacles"),
        "acm_observations": _validate_acm_observations(_require(raw, "acm_observations", "<root>"), "acm_observations"),
    }


def survey_sha256(path: Union[str, Path]) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


_ZERO_READING = {"value_m": 0.0, "uncertainty_1sigma_m": 0.0}


def _adapter_reading(survey: Dict[str, Any]) -> Dict[str, Any]:
    adapter = survey["base_mounting"]["adapter_plate_thickness"]
    return adapter if adapter is not None else _ZERO_READING


def _propagate(func, *readings: Dict[str, Any]) -> Tuple[float, float]:
    """First-order (linearized) 1-sigma propagation of `func`, a pure function of the readings'
    `value_m`s (positional, same order as `readings`). Central-difference-free forward
    Jacobian: exact for a function linear in its inputs (most derivations in this module), a
    good first-order approximation otherwise (yaw_rad, footprint_m)."""
    values = [r["value_m"] for r in readings]
    nominal = func(*values)
    variance = 0.0
    for i, r in enumerate(readings):
        sigma = r["uncertainty_1sigma_m"]
        if sigma == 0.0:
            continue
        v = values[i]
        step = abs(v) * 1e-6 if v != 0.0 else 1e-9
        perturbed = list(values)
        perturbed[i] = v + step
        deriv = (func(*perturbed) - nominal) / step
        variance += (deriv * sigma) ** 2
    return nominal, math.sqrt(variance)


def _corner_pt(values: List[float], i: int) -> Tuple[float, float]:
    return 0.070 + values[2 * i], values[2 * i + 1]


def _dist(p: Tuple[float, float], q: Tuple[float, float]) -> float:
    return math.hypot(q[0] - p[0], q[1] - p[1])


def _centre_x_fn(*values: float) -> float:
    return sum(0.070 + values[2 * i] for i in range(4)) / 4.0


def _centre_y_fn(*values: float) -> float:
    return sum(values[2 * i + 1] for i in range(4)) / 4.0


def _yaw_fn(*values: float) -> float:
    p0, p1, p2, p3 = (_corner_pt(list(values), i) for i in range(4))
    s0 = _dist(p0, p1)
    s2 = _dist(p2, p3)
    u_a = ((p1[0] - p0[0]) / s0, (p1[1] - p0[1]) / s0)
    u_c = (-(p3[0] - p2[0]) / s2, -(p3[1] - p2[1]) / s2)
    return math.atan2(u_a[1] + u_c[1], u_a[0] + u_c[0])


def _footprint0_fn(*values: float) -> float:
    p0, p1, p2, p3 = (_corner_pt(list(values), i) for i in range(4))
    return (_dist(p0, p1) + _dist(p2, p3)) / 2.0


def _footprint1_fn(*values: float) -> float:
    p0, p1, p2, p3 = (_corner_pt(list(values), i) for i in range(4))
    return (_dist(p1, p2) + _dist(p3, p0)) / 2.0


def _source(survey_sha: str, fields: str) -> str:
    return f"{SCHEMA_ID} sha256={survey_sha}; fields={fields}"


def derive_scene(
    survey: Dict[str, Any],
    survey_sha: str,
    base_keepout_m: Tuple[float, float, float, float] = BASE_KEEPOUT_M,
) -> Dict[str, Any]:
    """Apply §12.7's derivation + refusal rules to a `load_survey`-validated record. Raises
    SurveyError (rule 6) for: a corner-rectangularity violation; `specimen.resting_on_table` is
    false; `acm_observations.base_bolted_to_table` disagrees with
    `base_mounting.bolted_directly_to_table`; or any derived obstacle box overlapping
    `base_keepout_m`. Returns `{'objects', 'allowed_collisions', 'derived'}`; every object/ACM
    entry is `value_status: measured` with a `source` citing `survey_sha` and the fields used."""
    base_mounting = survey["base_mounting"]
    table = survey["table"]
    specimen = survey["specimen"]
    acm_observations = survey["acm_observations"]
    adapter_reading = _adapter_reading(survey)

    if acm_observations["base_bolted_to_table"] != base_mounting["bolted_directly_to_table"]:
        raise SurveyError(
            "acm_observations.base_bolted_to_table "
            f"({acm_observations['base_bolted_to_table']}) disagrees with "
            f"base_mounting.bolted_directly_to_table ({base_mounting['bolted_directly_to_table']})"
        )
    if not specimen["resting_on_table"]:
        raise SurveyError("specimen.resting_on_table is false: specimen is not confirmed resting on the table")

    # --- rule 1: specimen centre/yaw/footprint, with the rectangularity check ------------------
    corners = specimen["corners"]
    pts = [(0.070 + c["x_from_front_face"]["value_m"], c["y_from_centerline"]["value_m"]) for c in corners]
    s = [_dist(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    d0 = _dist(pts[0], pts[2])
    d1 = _dist(pts[1], pts[3])
    residual_m = max(abs(d0 - d1), abs(s[0] - s[2]), abs(s[1] - s[3]))
    tolerance = survey["derivation_params"]["rectangularity_tolerance_m"]
    if residual_m > tolerance:
        raise SurveyError(
            f"specimen.corners fail the rectangularity check (§12.7 rule 1): residual {residual_m:.6f} m "
            f"exceeds rectangularity_tolerance_m {tolerance:.6f} m"
        )

    corner_readings: List[Dict[str, Any]] = []
    for c in corners:
        corner_readings.append(c["x_from_front_face"])
        corner_readings.append(c["y_from_centerline"])

    centre_x, centre_x_sigma = _propagate(_centre_x_fn, *corner_readings)
    centre_y, centre_y_sigma = _propagate(_centre_y_fn, *corner_readings)
    yaw_rad, yaw_sigma = _propagate(_yaw_fn, *corner_readings)
    footprint0, footprint0_sigma = _propagate(_footprint0_fn, *corner_readings)
    footprint1, footprint1_sigma = _propagate(_footprint1_fn, *corner_readings)

    # --- rule 2/3 shared datum: z_table_top -----------------------------------------------------
    z_table_top, z_table_top_sigma = _propagate(lambda a: -a, adapter_reading)

    # --- rule 2: specimen top z (see module docstring's note on this rule's literal wording) ---
    thickness_readings = specimen["thickness_readings"]
    thickness_mean, thickness_mean_sigma = _propagate(lambda *ts: sum(ts) / len(ts), *thickness_readings)
    top_z, top_z_sigma = _propagate(
        lambda a, *ts: -a + sum(ts) / len(ts), adapter_reading, *thickness_readings
    )
    specimen_pos_z, specimen_pos_z_sigma = _propagate(
        lambda a, *ts: -a + (sum(ts) / len(ts)) / 2.0, adapter_reading, *thickness_readings
    )

    specimen_source = _source(
        survey_sha,
        "specimen.corners[0..3].x_from_front_face,specimen.corners[0..3].y_from_centerline,"
        "specimen.thickness_readings[0..n],base_mounting.adapter_plate_thickness",
    )
    specimen_obj = {
        "id": "specimen",
        "shape": "box",
        "dimensions_m": [footprint0, footprint1, thickness_mean],
        "pose": {
            "frame": "base_link",
            "position_m": [centre_x, centre_y, specimen_pos_z],
            "rpy_rad": [0.0, 0.0, yaw_rad],
        },
        "value_status": "measured",
        "source": specimen_source,
    }

    # --- rule 3: table box -----------------------------------------------------------------------
    far = table["front_face_to_far_edge"]
    near = table["front_face_to_near_edge"]
    pos_y_edge = table["pos_y_side_face_to_edge"]
    neg_y_edge = table["neg_y_side_face_to_edge"]
    thickness = table["thickness"]

    dim_x, dim_x_sigma = _propagate(lambda f, n: (0.070 + f) - (0.070 - n), far, near)
    dim_y, dim_y_sigma = _propagate(lambda p, n: (0.100 + p) - (-0.100 - n), pos_y_edge, neg_y_edge)
    dim_z, dim_z_sigma = _propagate(lambda t: t, thickness)
    table_pos_x, table_pos_x_sigma = _propagate(lambda f, n: ((0.070 - n) + (0.070 + f)) / 2.0, far, near)
    table_pos_y, table_pos_y_sigma = _propagate(
        lambda p, n: ((-0.100 - n) + (0.100 + p)) / 2.0, pos_y_edge, neg_y_edge
    )
    table_pos_z, table_pos_z_sigma = _propagate(lambda a, t: -a - t / 2.0, adapter_reading, thickness)

    table_source = _source(
        survey_sha,
        "table.front_face_to_far_edge,table.front_face_to_near_edge,table.pos_y_side_face_to_edge,"
        "table.neg_y_side_face_to_edge,table.thickness,base_mounting.adapter_plate_thickness",
    )
    table_obj = {
        "id": "table",
        "shape": "box",
        "dimensions_m": [dim_x, dim_y, dim_z],
        "pose": {
            "frame": "base_link",
            "position_m": [table_pos_x, table_pos_y, table_pos_z],
            "rpy_rad": [0.0, 0.0, 0.0],
        },
        "value_status": "measured",
        "source": table_source,
    }

    allowed_collisions = [
        {
            "link_a": "table",
            "link_b": "base_link",
            "reason": (
                "the robot base is bolted directly to the table top"
                if base_mounting["bolted_directly_to_table"]
                else "the robot base is bolted to the table via an adapter plate; the two necessarily overlap"
            ),
            "value_status": "measured",
            "source": _source(survey_sha, "base_mounting.bolted_directly_to_table,acm_observations.base_bolted_to_table"),
        },
        {
            "link_a": "specimen",
            "link_b": "table",
            "reason": "the specimen rests directly on the table top",
            "value_status": "measured",
            "source": _source(
                survey_sha, "specimen.resting_on_table,acm_observations.specimen_resting_on_table"
            ),
        },
    ]

    # --- rule 4: obstacle boxes, with the rule-6(d) base-keep-out refusal ------------------------
    kx0, kx1, ky0, ky1 = base_keepout_m
    obstacle_objs = []
    obstacles_derived = {}
    for item in survey["obstacles"]["items"]:
        xr = item["x_from_front_face"]
        yr = item["y_from_centerline"]
        zr = item["z_from_table_top"]

        x_min_val = 0.070 + xr["min"]["value_m"]
        x_max_val = 0.070 + xr["max"]["value_m"]
        y_min_val = yr["min"]["value_m"]
        y_max_val = yr["max"]["value_m"]
        if x_min_val < kx1 and x_max_val > kx0 and y_min_val < ky1 and y_max_val > ky0:
            raise SurveyError(
                f"obstacles.items (id={item['id']!r}) overlaps the robot base keep-out "
                f"{base_keepout_m} (§12.7 rule 6): x=[{x_min_val:.4f}, {x_max_val:.4f}], "
                f"y=[{y_min_val:.4f}, {y_max_val:.4f}]"
            )

        dim_x_o, dim_x_o_sigma = _propagate(lambda vmin, vmax: (0.070 + vmax) - (0.070 + vmin), xr["min"], xr["max"])
        pos_x_o, pos_x_o_sigma = _propagate(lambda vmin, vmax: 0.070 + (vmin + vmax) / 2.0, xr["min"], xr["max"])
        dim_y_o, dim_y_o_sigma = _propagate(lambda vmin, vmax: vmax - vmin, yr["min"], yr["max"])
        pos_y_o, pos_y_o_sigma = _propagate(lambda vmin, vmax: (vmin + vmax) / 2.0, yr["min"], yr["max"])
        dim_z_o, dim_z_o_sigma = _propagate(
            lambda a, vmin, vmax: (-a + vmax) - (-a + vmin), adapter_reading, zr["min"], zr["max"]
        )
        pos_z_o, pos_z_o_sigma = _propagate(
            lambda a, vmin, vmax: -a + (vmin + vmax) / 2.0, adapter_reading, zr["min"], zr["max"]
        )

        obstacle_source = _source(
            survey_sha,
            f"obstacles.items[id={item['id']}].x_from_front_face,.y_from_centerline,.z_from_table_top,"
            "base_mounting.adapter_plate_thickness",
        )
        obstacle_objs.append(
            {
                "id": item["id"],
                "shape": "box",
                "dimensions_m": [dim_x_o, dim_y_o, dim_z_o],
                "pose": {
                    "frame": "base_link",
                    "position_m": [pos_x_o, pos_y_o, pos_z_o],
                    "rpy_rad": [0.0, 0.0, 0.0],
                },
                "value_status": "measured",
                "source": obstacle_source,
            }
        )
        obstacles_derived[item["id"]] = {
            "dimensions_m": [dim_x_o, dim_y_o, dim_z_o],
            "dimensions_sigma_m": [dim_x_o_sigma, dim_y_o_sigma, dim_z_o_sigma],
            "position_m": [pos_x_o, pos_y_o, pos_z_o],
            "position_sigma_m": [pos_x_o_sigma, pos_y_o_sigma, pos_z_o_sigma],
        }

    derived = {
        "specimen": {
            "centre_m": [centre_x, centre_y],
            "centre_sigma_m": [centre_x_sigma, centre_y_sigma],
            "yaw_rad": yaw_rad,
            "yaw_sigma_rad": yaw_sigma,
            "footprint_m": [footprint0, footprint1],
            "footprint_sigma_m": [footprint0_sigma, footprint1_sigma],
            "thickness_m": thickness_mean,
            "thickness_sigma_m": thickness_mean_sigma,
            "top_z_m": top_z,
            "top_z_sigma_m": top_z_sigma,
            "rectangularity_residual_m": residual_m,
        },
        "table": {
            "dimensions_m": [dim_x, dim_y, dim_z],
            "dimensions_sigma_m": [dim_x_sigma, dim_y_sigma, dim_z_sigma],
            "position_m": [table_pos_x, table_pos_y, table_pos_z],
            "position_sigma_m": [table_pos_x_sigma, table_pos_y_sigma, table_pos_z_sigma],
            "z_table_top_m": z_table_top,
            "z_table_top_sigma_m": z_table_top_sigma,
        },
        "obstacles": obstacles_derived,
    }

    return {
        "objects": [table_obj, specimen_obj] + obstacle_objs,
        "allowed_collisions": allowed_collisions,
        "derived": derived,
    }
