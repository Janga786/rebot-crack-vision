"""tests/test_path_accuracy.py — path-extraction accuracy against synthetic ground truth. PERC-09.

Runs the real pipeline (`crackvision.skeleton.clean_and_skeletonize` -> `crackvision.paths.
build_case_paths`) on synthetic rasters from `tools.synth_cracks` whose centerlines are known
exactly, then scores the extracted `main_path` (and, for the branched case, the matched branch)
against ground truth with symmetric Hausdorff / mean distance / coverage / ordering-error rate.

Threshold rationale (docs/perception/PATH_ACCURACY.md):
  - Cracks are rasterized `thickness_px / 2` wide, so a perfect skeleton can sit up to roughly
    that radius off the true centerline purely from stamped-disk rounding; thresholds are set at
    ~1.5-2x that radius to catch real regressions (spur pruning, bad decomposition, junction
    bridging bugs) without being tripped by expected raster/skeletonize jitter.
  - `min_spur_length_px=3` matches the pipeline default order of magnitude (`crackvision.
    skeleton_graph.DEFAULT_MIN_SPUR_LENGTH_PX`), kept small here since these synthetic cracks are
    already spur-free by construction.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from crackvision.paths import build_case_paths  # noqa: E402
from crackvision.skeleton import clean_and_skeletonize  # noqa: E402
from synth_cracks import (  # noqa: E402
    coverage_fraction,
    generate_branched_crack,
    generate_wavy_crack,
    mean_distance_px,
    ordering_error_rate,
    symmetric_hausdorff_px,
)

HAUSDORFF_MAX_PX = 6.0
MEAN_DIST_MAX_PX = 2.5
COVERAGE_MIN_FRACTION = 0.95
COVERAGE_THRESHOLD_PX = 3.0
ORDERING_ERROR_MAX_RATE = 0.02


def _extract_main_path(mask, *, min_spur_length_px: int = 3, rdp_tolerance_px: float = 1.5):
    _, skeleton, _, _ = clean_and_skeletonize(mask, min_component_size=0)
    doc = build_case_paths(
        skeleton, "synthetic", min_spur_length_px=min_spur_length_px, rdp_tolerance_px=rdp_tolerance_px
    )
    assert doc["component_count"] >= 1
    main = doc["components"][0]["main_path"]
    return [(p["row"], p["col"]) for p in main["dense"]], doc


def _assert_accurate(gt, extracted, *, hausdorff_max=HAUSDORFF_MAX_PX, mean_max=MEAN_DIST_MAX_PX):
    hausdorff = symmetric_hausdorff_px(gt, extracted)
    mean_dist = mean_distance_px(gt, extracted)
    coverage = coverage_fraction(gt, extracted, COVERAGE_THRESHOLD_PX)
    ordering = ordering_error_rate(gt, extracted)

    assert hausdorff <= hausdorff_max, f"hausdorff {hausdorff:.2f}px exceeds {hausdorff_max}px"
    assert mean_dist <= mean_max, f"mean distance {mean_dist:.2f}px exceeds {mean_max}px"
    assert coverage >= COVERAGE_MIN_FRACTION, f"coverage {coverage:.3f} below {COVERAGE_MIN_FRACTION}"
    assert ordering <= ORDERING_ERROR_MAX_RATE, f"ordering error rate {ordering:.3f} exceeds {ORDERING_ERROR_MAX_RATE}"
    return {"hausdorff_px": hausdorff, "mean_dist_px": mean_dist, "coverage": coverage, "ordering_error_rate": ordering}


# ---------------------------------------------------------------------------
# Synthetic-generator sanity
# ---------------------------------------------------------------------------


def test_generator_produces_nonempty_connected_mask():
    crack = generate_wavy_crack(height=150, width=250, amplitude_px=15, frequency=0.03, thickness_px=3, seed=1)
    assert crack.mask.dtype == bool
    assert crack.mask.any()
    assert len(crack.centerline) > 10


def test_generator_is_deterministic_given_seed():
    a = generate_wavy_crack(height=120, width=200, thickness_px=3, salt_noise_prob=0.001, seed=42)
    b = generate_wavy_crack(height=120, width=200, thickness_px=3, salt_noise_prob=0.001, seed=42)
    assert (a.mask == b.mask).all()
    assert a.centerline == b.centerline


# ---------------------------------------------------------------------------
# Straight / low-curvature crack
# ---------------------------------------------------------------------------


def test_straight_crack_path_accuracy():
    crack = generate_wavy_crack(height=120, width=300, amplitude_px=0.0, frequency=0.0, thickness_px=3, seed=1)
    extracted, _ = _extract_main_path(crack.mask)
    _assert_accurate(crack.centerline, extracted)


# ---------------------------------------------------------------------------
# Curved crack
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("amplitude_px,frequency", [(20.0, 0.03), (35.0, 0.015)])
def test_curved_crack_path_accuracy(amplitude_px, frequency):
    crack = generate_wavy_crack(
        height=200, width=320, amplitude_px=amplitude_px, frequency=frequency, thickness_px=3, seed=2
    )
    extracted, _ = _extract_main_path(crack.mask)
    _assert_accurate(crack.centerline, extracted)


# ---------------------------------------------------------------------------
# Noisy crack (scattered salt pixels, pruned as tiny isolated components)
# ---------------------------------------------------------------------------


def test_noisy_crack_path_accuracy():
    crack = generate_wavy_crack(
        height=180, width=280, amplitude_px=18.0, frequency=0.025, thickness_px=3, salt_noise_prob=0.0008, seed=7
    )
    _, skeleton, _, _ = clean_and_skeletonize(crack.mask, min_component_size=6)
    doc = build_case_paths(skeleton, "synthetic-noisy", min_spur_length_px=3, rdp_tolerance_px=1.5)
    main_component = max(doc["components"], key=lambda c: c["main_path"]["length_px"])
    extracted = [(p["row"], p["col"]) for p in main_component["main_path"]["dense"]]
    _assert_accurate(crack.centerline, extracted)


# ---------------------------------------------------------------------------
# Branched crack: main path + one branch, both scored against their own ground truth
# ---------------------------------------------------------------------------


def test_branched_crack_main_and_branch_accuracy():
    crack = generate_branched_crack(
        height=220,
        width=320,
        amplitude_px=15.0,
        frequency=0.02,
        thickness_px=3,
        branch_frac=0.5,
        branch_length_px=60.0,
        seed=3,
    )
    _, skeleton, _, _ = clean_and_skeletonize(crack.mask, min_component_size=0)
    doc = build_case_paths(skeleton, "synthetic-branched", min_spur_length_px=3, rdp_tolerance_px=1.5)

    assert doc["component_count"] == 1
    component = doc["components"][0]
    assert len(component["branches"]) >= 1

    main_extracted = [(p["row"], p["col"]) for p in component["main_path"]["dense"]]
    branch_extracted = [(p["row"], p["col"]) for p in component["branches"][0]["dense"]]

    # The main-path decomposition picks the graph *diameter*, which may run from the crack's start
    # through into the branch rather than along the originally-drawn main crack if the branch is
    # long enough — so score each extracted polyline against whichever ground-truth centerline
    # (main or branch) it lies closest to, rather than assuming a fixed main<->main pairing.
    candidates = [crack.centerline, *crack.branch_centerlines]
    for extracted in (main_extracted, branch_extracted):
        best_gt = min(candidates, key=lambda gt: symmetric_hausdorff_px(gt, extracted))
        _assert_accurate(best_gt, extracted, hausdorff_max=10.0, mean_max=4.0)


# ---------------------------------------------------------------------------
# Regression guard: a corrupted extraction must fail these thresholds
# ---------------------------------------------------------------------------


def test_metrics_reject_a_badly_wrong_path():
    crack = generate_wavy_crack(height=120, width=250, amplitude_px=15.0, frequency=0.03, thickness_px=3, seed=5)
    wrong = [(r + 20, c) for r, c in crack.centerline]  # shifted 20px off the true centerline
    hausdorff = symmetric_hausdorff_px(crack.centerline, wrong)
    mean_dist = mean_distance_px(crack.centerline, wrong)
    coverage = coverage_fraction(crack.centerline, wrong, COVERAGE_THRESHOLD_PX)
    assert hausdorff > HAUSDORFF_MAX_PX
    assert mean_dist > MEAN_DIST_MAX_PX
    assert coverage < COVERAGE_MIN_FRACTION
