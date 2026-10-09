"""Unit tests for survey_core (MOT-10.2). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_survey_core.py -q -p no:cacheprovider
"""

import copy
import math
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crackvision_motion.survey_core import (  # noqa: E402
    BASE_KEEPOUT_M,
    SCHEMA_ID,
    SurveyError,
    derive_scene,
    load_survey,
    survey_sha256,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
TEMPLATE_PATH = REPO_ROOT / "config" / "scene" / "workcell_survey.template.yaml"
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
FIXTURE_PATH = FIXTURES_DIR / "workcell_survey_example.yaml"


def _write(tmp_path: Path, raw: dict, name: str = "survey.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


@pytest.fixture
def valid_raw():
    with FIXTURE_PATH.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# --------------------------------------------------------------------------------------
# load_survey: happy path + the fixture is clearly labelled synthetic
# --------------------------------------------------------------------------------------

def test_load_survey_valid_fixture():
    survey = load_survey(FIXTURE_PATH)
    assert survey["schema"] == SCHEMA_ID
    assert len(survey["specimen"]["corners"]) == 4
    assert len(survey["obstacles"]["items"]) == 1


def test_fixture_is_labelled_synthetic():
    raw_text = FIXTURE_PATH.read_text(encoding="utf-8")
    assert "SYNTHETIC" in raw_text or "synthetic" in raw_text
    survey = load_survey(FIXTURE_PATH)
    assert "synthetic" in survey["survey"]["operator"].lower()
    assert "synthetic" in survey["survey"]["notes"].lower()


def test_survey_sha256_deterministic():
    assert survey_sha256(FIXTURE_PATH) == survey_sha256(FIXTURE_PATH)
    other_sha = survey_sha256(TEMPLATE_PATH)
    assert other_sha != survey_sha256(FIXTURE_PATH)


# --------------------------------------------------------------------------------------
# load_survey: the all-null template is refused as incomplete
# --------------------------------------------------------------------------------------

def test_template_file_is_refused_as_incomplete():
    with pytest.raises(SurveyError):
        load_survey(TEMPLATE_PATH)


# --------------------------------------------------------------------------------------
# load_survey: rejects malformed input (§12.7 rule 6 structural refusals)
# --------------------------------------------------------------------------------------

def test_rejects_wrong_schema(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["schema"] = "crackvision.workcell_survey/2"
    with pytest.raises(SurveyError, match="schema"):
        load_survey(_write(tmp_path, raw))


def test_rejects_unknown_top_level_key(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["extra"] = 1
    with pytest.raises(SurveyError, match="unknown key"):
        load_survey(_write(tmp_path, raw))


def test_rejects_unknown_key_in_nested_block(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["table"]["extra_field"] = 1
    with pytest.raises(SurveyError, match="unknown key"):
        load_survey(_write(tmp_path, raw))


def test_rejects_missing_uncertainty_on_a_reading(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["table"]["thickness"]["uncertainty_1sigma_m"] = None
    with pytest.raises(SurveyError, match="table.thickness.uncertainty_1sigma_m"):
        load_survey(_write(tmp_path, raw))


def test_rejects_missing_value_m_on_a_corner_reading(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["specimen"]["corners"][0]["x_from_front_face"]["value_m"] = None
    with pytest.raises(SurveyError, match="specimen.corners\\[0\\]"):
        load_survey(_write(tmp_path, raw))


def test_rejects_fewer_than_3_thickness_readings(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["specimen"]["thickness_readings"] = raw["specimen"]["thickness_readings"][:2]
    with pytest.raises(SurveyError, match="thickness_readings"):
        load_survey(_write(tmp_path, raw))


def test_rejects_wrong_corner_count(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["specimen"]["corners"] = raw["specimen"]["corners"][:3]
    with pytest.raises(SurveyError, match="corners"):
        load_survey(_write(tmp_path, raw))


def test_rejects_missing_adapter_plate_thickness_when_not_bolted(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["base_mounting"]["bolted_directly_to_table"] = False
    raw["base_mounting"]["adapter_plate_thickness"] = None
    with pytest.raises(SurveyError, match="adapter_plate_thickness"):
        load_survey(_write(tmp_path, raw))


def test_adapter_plate_thickness_exempt_from_null_check_when_bolted(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    assert raw["base_mounting"]["bolted_directly_to_table"] is True
    raw["base_mounting"]["adapter_plate_thickness"] = None  # already null in the fixture
    survey = load_survey(_write(tmp_path, raw))
    assert survey["base_mounting"]["adapter_plate_thickness"] is None


def test_rejects_duplicate_obstacle_id(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["obstacles"]["items"].append(copy.deepcopy(raw["obstacles"]["items"][0]))
    with pytest.raises(SurveyError, match="duplicate obstacle id"):
        load_survey(_write(tmp_path, raw))


def test_rejects_min_greater_than_max(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["obstacles"]["items"][0]["x_from_front_face"]["min"]["value_m"] = 10.0
    with pytest.raises(SurveyError, match="min.value_m"):
        load_survey(_write(tmp_path, raw))


# --------------------------------------------------------------------------------------
# derive_scene: the worked fixture, hand-computed expected poses within 1e-9
# --------------------------------------------------------------------------------------
#
# Specimen (synthetic fixture): a 0.20 x 0.15 m rectangle centred at (0.30, 0.0), yawed by
# theta = atan2(0.28, 0.96) (an exact 7:24:25 triangle). Corners (base_link x,y), built as
# C +/- (sx/2)*u +/- (sy/2)*v with u=(0.96,0.28), v=(-0.28,0.96):
#   P0=(0.225,-0.100) P1=(0.417,-0.044) P2=(0.375,0.100) P3=(0.183,0.044)
# -> centre = mean(P0..P3) = (0.30, 0.0) exactly; s0=s2=0.20, s1=s3=0.15, d0=d1=0.25
#    (exact rectangle, residual = 0); yaw = atan2(0.28,0.96); footprint = [0.20, 0.15].
# Specimen thickness readings mean = (0.0098+0.0101+0.0100+0.0102)/4 = 0.010025 m; bolted
# directly to table (adapter treated as 0) so z_table_top = 0, specimen centre z =
# 0 + 0.010025/2 = 0.0050125.
# Table: x_far=0.070+0.800=0.870, x_near=0.070-0.300=-0.230 -> dim_x=1.100, centre_x=0.320;
#        y_pos=0.100+0.400=0.500, y_neg=-0.100-0.400=-0.500 -> dim_y=1.000, centre_y=0.0;
#        dim_z=0.045, centre_z = -0/2 - 0.045/2 = -0.0225 (bolted direct, z_table_top=0).
# Obstacle wall_east: x=[0.070+0.500,0.070+0.600]=[0.570,0.670] -> dim 0.100, centre 0.620;
#        y=[0.450,0.550] -> dim 0.100, centre 0.500; z=[0+0.000,0+0.300] -> dim 0.300, centre 0.150.

def test_derive_scene_worked_fixture_poses():
    survey = load_survey(FIXTURE_PATH)
    sha = survey_sha256(FIXTURE_PATH)
    result = derive_scene(survey, sha)

    table = next(o for o in result["objects"] if o["id"] == "table")
    assert table["dimensions_m"] == pytest.approx([1.100, 1.000, 0.045], abs=1e-9)
    assert table["pose"]["position_m"] == pytest.approx([0.320, 0.0, -0.0225], abs=1e-9)
    assert table["pose"]["rpy_rad"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    assert table["value_status"] == "measured"
    assert sha in table["source"]

    specimen = next(o for o in result["objects"] if o["id"] == "specimen")
    assert specimen["dimensions_m"] == pytest.approx([0.20, 0.15, 0.010025], abs=1e-9)
    assert specimen["pose"]["position_m"] == pytest.approx([0.30, 0.0, 0.0050125], abs=1e-9)
    assert specimen["pose"]["rpy_rad"] == pytest.approx([0.0, 0.0, math.atan2(0.28, 0.96)], abs=1e-9)
    assert specimen["value_status"] == "measured"
    assert sha in specimen["source"]

    wall = next(o for o in result["objects"] if o["id"] == "wall_east")
    assert wall["dimensions_m"] == pytest.approx([0.100, 0.100, 0.300], abs=1e-9)
    assert wall["pose"]["position_m"] == pytest.approx([0.620, 0.500, 0.150], abs=1e-9)
    assert wall["value_status"] == "measured"

    assert result["derived"]["specimen"]["rectangularity_residual_m"] == pytest.approx(0.0, abs=1e-9)
    assert result["derived"]["specimen"]["yaw_rad"] == pytest.approx(math.atan2(0.28, 0.96), abs=1e-9)


def test_derive_scene_allowed_collisions():
    survey = load_survey(FIXTURE_PATH)
    sha = survey_sha256(FIXTURE_PATH)
    result = derive_scene(survey, sha)
    pairs = {(e["link_a"], e["link_b"]) for e in result["allowed_collisions"]}
    assert ("table", "base_link") in pairs
    assert ("specimen", "table") in pairs
    for entry in result["allowed_collisions"]:
        assert entry["value_status"] == "measured"
        assert sha in entry["source"]


def test_derive_scene_uncertainty_is_propagated():
    survey = load_survey(FIXTURE_PATH)
    sha = survey_sha256(FIXTURE_PATH)
    result = derive_scene(survey, sha)
    derived = result["derived"]

    # centre is a linear mean of 4 independent corner readings (sigma 0.0005 m each on both
    # x and y) -> sigma_centre = sqrt(4 * 0.0005^2) / 4 = 0.00025, exact (finite-difference of
    # an exactly-linear function has no truncation error).
    assert derived["specimen"]["centre_sigma_m"] == pytest.approx([0.00025, 0.00025], abs=1e-9)
    # table dim_x = x_far - x_near, both readings sigma 0.001 m, independent -> sqrt(2)*0.001.
    assert derived["table"]["dimensions_sigma_m"][0] == pytest.approx(0.001 * math.sqrt(2), abs=1e-9)
    # yaw/footprint go through atan2/sqrt (nonlinear): still a finite, positive 1-sigma value.
    assert derived["specimen"]["yaw_sigma_rad"] > 0.0
    assert derived["specimen"]["footprint_sigma_m"][0] > 0.0
    assert derived["specimen"]["footprint_sigma_m"][1] > 0.0


# --------------------------------------------------------------------------------------
# derive_scene: §12.7 rule-6 refusals
# --------------------------------------------------------------------------------------

def test_refuses_non_rectangular_corners(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    # Mirrors §12.7's own worked example: push one corner 5 mm too far (> 3 mm tolerance).
    raw["specimen"]["corners"][2]["x_from_front_face"]["value_m"] += 0.005
    survey = load_survey(_write(tmp_path, raw))
    with pytest.raises(SurveyError, match="rectangularity"):
        derive_scene(survey, "irrelevant")


def test_refuses_specimen_not_resting_on_table(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["specimen"]["resting_on_table"] = False
    raw["acm_observations"]["specimen_resting_on_table"] = False
    survey = load_survey(_write(tmp_path, raw))
    with pytest.raises(SurveyError, match="resting_on_table"):
        derive_scene(survey, "irrelevant")


def test_refuses_acm_bolted_mismatch(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["acm_observations"]["base_bolted_to_table"] = False
    survey = load_survey(_write(tmp_path, raw))
    with pytest.raises(SurveyError, match="base_bolted_to_table"):
        derive_scene(survey, "irrelevant")


def test_refuses_obstacle_in_base_keepout(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    item = raw["obstacles"]["items"][0]
    item["x_from_front_face"]["min"]["value_m"] = -0.02
    item["x_from_front_face"]["max"]["value_m"] = 0.02
    item["y_from_centerline"]["min"]["value_m"] = -0.02
    item["y_from_centerline"]["max"]["value_m"] = 0.02
    survey = load_survey(_write(tmp_path, raw))
    with pytest.raises(SurveyError, match="base keep-out"):
        derive_scene(survey, "irrelevant")


def test_default_base_keepout_matches_reachability_convention():
    assert BASE_KEEPOUT_M == (-0.09, 0.09, -0.12, 0.12)


# --------------------------------------------------------------------------------------
# derive_scene: a yawed specimen (the fixture already is one, this re-checks the formula
# against a clean axis-aligned sanity case) and an adapter-plate offset
# --------------------------------------------------------------------------------------

def test_axis_aligned_specimen_has_zero_yaw(tmp_path, valid_raw):
    survey = copy.deepcopy(valid_raw)
    # Overwrite with an exact axis-aligned 0.2 x 0.2 square centred at (0.30, 0.0).
    survey["specimen"]["corners"] = [
        {
            "x_from_front_face": {"value_m": 0.130, "instrument": "caliper", "resolution_m": 0.0005, "uncertainty_1sigma_m": 0.0005},
            "y_from_centerline": {"value_m": -0.100, "instrument": "caliper", "resolution_m": 0.0005, "uncertainty_1sigma_m": 0.0005},
        },
        {
            "x_from_front_face": {"value_m": 0.330, "instrument": "caliper", "resolution_m": 0.0005, "uncertainty_1sigma_m": 0.0005},
            "y_from_centerline": {"value_m": -0.100, "instrument": "caliper", "resolution_m": 0.0005, "uncertainty_1sigma_m": 0.0005},
        },
        {
            "x_from_front_face": {"value_m": 0.330, "instrument": "caliper", "resolution_m": 0.0005, "uncertainty_1sigma_m": 0.0005},
            "y_from_centerline": {"value_m": 0.100, "instrument": "caliper", "resolution_m": 0.0005, "uncertainty_1sigma_m": 0.0005},
        },
        {
            "x_from_front_face": {"value_m": 0.130, "instrument": "caliper", "resolution_m": 0.0005, "uncertainty_1sigma_m": 0.0005},
            "y_from_centerline": {"value_m": 0.100, "instrument": "caliper", "resolution_m": 0.0005, "uncertainty_1sigma_m": 0.0005},
        },
    ]
    survey = load_survey(_write(tmp_path, survey))
    result = derive_scene(survey, "irrelevant")
    specimen = next(o for o in result["objects"] if o["id"] == "specimen")
    assert specimen["pose"]["rpy_rad"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    assert specimen["pose"]["position_m"][:2] == pytest.approx([0.30, 0.0], abs=1e-9)
    assert specimen["dimensions_m"][:2] == pytest.approx([0.20, 0.20], abs=1e-9)


def test_yawed_specimen_matches_atan2_of_corner_edge(valid_raw):
    # The fixture's own specimen is already yawed by atan2(0.28, 0.96); re-derive it in
    # isolation to pin the formula down against that exact closed form.
    survey = load_survey(FIXTURE_PATH)
    result = derive_scene(survey, "irrelevant")
    specimen = next(o for o in result["objects"] if o["id"] == "specimen")
    assert specimen["pose"]["rpy_rad"][2] == pytest.approx(math.atan2(0.28, 0.96), abs=1e-9)


def test_adapter_plate_offset_shifts_table_and_specimen_z(tmp_path, valid_raw):
    raw = copy.deepcopy(valid_raw)
    raw["base_mounting"]["bolted_directly_to_table"] = False
    raw["base_mounting"]["adapter_plate_thickness"] = {
        "value_m": 0.020,
        "instrument": "caliper",
        "resolution_m": 0.0005,
        "uncertainty_1sigma_m": 0.0005,
    }
    raw["acm_observations"]["base_bolted_to_table"] = False
    survey = load_survey(_write(tmp_path, raw))
    result = derive_scene(survey, "irrelevant")

    table = next(o for o in result["objects"] if o["id"] == "table")
    specimen = next(o for o in result["objects"] if o["id"] == "specimen")
    # z_table_top = -(adapter) = -0.020; table centre_z = -0.020 - 0.045/2 = -0.0425;
    # specimen bottom sits on z_table_top -> specimen centre_z = -0.020 + 0.010025/2 = -0.0149875.
    assert table["pose"]["position_m"][2] == pytest.approx(-0.0425, abs=1e-9)
    assert specimen["pose"]["position_m"][2] == pytest.approx(-0.020 + 0.010025 / 2.0, abs=1e-9)
    assert result["derived"]["table"]["z_table_top_m"] == pytest.approx(-0.020, abs=1e-9)
