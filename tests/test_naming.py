"""tests/test_naming.py — the case_id contract (docs/INTERFACES.md §1). Built by TC-007.

Every row of the worked-example table in docs/INTERFACES.md §1.1 is implemented here, using the
literal input filenames and expected case_ids from that table.

Two rows in that table contradict their own stated algorithm / parenthetical explanation; both are
documented at the point of use below rather than silently "fixed" or silently followed. See this
card's completion report (docs/COMPLETION_LOG.md, TC-007) ISSUES for the full write-up:

  * "A.png" / "a!.png": the table's answer column prints "A, then a__2", but its own parenthetical
    says "distinct stems -> no collision: A vs a", and the normative algorithm (this same section,
    and the task card body) is case-preserving with no case-folding step, so "A" and "a" are
    different case_ids and there is no collision. Implemented per the algorithm + parenthetical.
  * "x y.png" / "x-y.png" / "x_y.png": the table's answer column says "no collision", but the
    normative sanitisation rule maps both the space in "x y.png" and the underscore in "x_y.png" to
    "_", so both stems become "x_y" -- that IS the collision case, per the very rule that defines
    collisions. Implemented per the algorithm: "x y.png" (sorts first) keeps the bare id "x_y";
    "x-y.png" is untouched by the collision ("-" is an allowed character, so it stays "x-y"); "x_y.png"
    (sorts last of the two "x_y" producers) becomes "x_y__2".
"""

from __future__ import annotations

from pathlib import Path

import pytest

from crackvision.naming import (
    assign_case_ids,
    comparison_path,
    derive_case_id,
    mask_path,
    nnunet_input_path,
    overlay_path,
    prediction_path,
    skeleton_overlay_path,
    skeleton_path,
    skeleton_stats_path,
)

# docs/INTERFACES.md §1.1 worked-example table, single-file rows.
CASE_ID_TABLE = [
    ("Deck Crack 01.JPG", "Deck_Crack_01"),
    ("IMG_2043.png", "IMG_2043"),
    ("bridge-pier.7.jpeg", "bridge-pier_7"),
    ("crack (copy).png", "crack_copy"),
    ("  spaced  .bmp", "spaced"),
    ("sample_0000.png", "sample"),
    ("2026-09-16_run1.tif", "2026-09-16_run1"),
    ("....png", "image"),
]


@pytest.mark.parametrize("filename,expected", CASE_ID_TABLE)
def test_derive_case_id_table(filename: str, expected: str) -> None:
    assert derive_case_id(Path(filename)) == expected


def test_derive_case_id_is_pure_and_case_preserving() -> None:
    # No case-folding step anywhere in the algorithm.
    assert derive_case_id(Path("A.png")) == "A"
    assert derive_case_id(Path("a!.png")) == "a"


def test_assign_case_ids_A_vs_a_no_collision() -> None:
    """"A.png" and "a!.png" sanitise to different strings ("A" vs "a") -> no collision, no suffix."""
    sources = [Path("A.png"), Path("a!.png")]
    assigned = assign_case_ids(sources)
    assert assigned[Path("A.png")] == "A"
    assert assigned[Path("a!.png")] == "a"


def test_assign_case_ids_space_and_underscore_do_collide() -> None:
    """"x y.png" and "x_y.png" both sanitise to "x_y" -> that IS a collision; "x-y.png" is untouched."""
    sources = [Path("x y.png"), Path("x-y.png"), Path("x_y.png")]
    assigned = assign_case_ids(sources)
    assert assigned[Path("x y.png")] == "x_y"  # sorts first among the "x_y" producers -> bare id
    assert assigned[Path("x-y.png")] == "x-y"  # "-" is an allowed char, never touched
    assert assigned[Path("x_y.png")] == "x_y__2"  # second "x_y" producer, in sorted order


def test_assign_case_ids_collision_suffix_order() -> None:
    """"a b.png" then "a_b.png" -> "a_b", "a_b__2" (docs/INTERFACES.md §1.1)."""
    sources = [Path("a b.png"), Path("a_b.png")]
    assigned = assign_case_ids(sources)
    assert assigned[Path("a b.png")] == "a_b"
    assert assigned[Path("a_b.png")] == "a_b__2"


def test_assign_case_ids_three_way_collision_is_sorted_path_order() -> None:
    sources = [Path("crack!.png"), Path("crack.png"), Path("crack?.png")]
    assigned = assign_case_ids(sources)
    # sorted(str) order: "crack!.png" < "crack.png" < "crack?.png" ('!' 0x21 < '.' 0x2e < '?' 0x3f)
    assert assigned[Path("crack!.png")] == "crack"
    assert assigned[Path("crack.png")] == "crack__2"
    assert assigned[Path("crack?.png")] == "crack__3"


def test_assign_case_ids_is_deterministic_regardless_of_input_order() -> None:
    forward = [Path("a b.png"), Path("a_b.png")]
    backward = list(reversed(forward))
    assert assign_case_ids(forward) == assign_case_ids(backward)


def test_assign_case_ids_empty() -> None:
    assert assign_case_ids([]) == {}


@pytest.mark.parametrize(
    "fn,case,expected",
    [
        (nnunet_input_path, "Deck_Crack_01", Path("data/nnunet_input/Deck_Crack_01_0000.png")),
        (prediction_path, "Deck_Crack_01", Path("data/predictions/Deck_Crack_01.png")),
        (mask_path, "Deck_Crack_01", Path("data/overlays/Deck_Crack_01_mask.png")),
        (overlay_path, "Deck_Crack_01", Path("data/overlays/Deck_Crack_01_overlay.png")),
        (comparison_path, "Deck_Crack_01", Path("data/comparisons/Deck_Crack_01_comparison.png")),
        (skeleton_path, "Deck_Crack_01", Path("data/skeletons/Deck_Crack_01_skeleton.png")),
        (
            skeleton_overlay_path,
            "Deck_Crack_01",
            Path("data/skeletons/Deck_Crack_01_skeleton_overlay.png"),
        ),
        (
            skeleton_stats_path,
            "Deck_Crack_01",
            Path("data/skeletons/Deck_Crack_01_skeleton_stats.json"),
        ),
    ],
)
def test_path_helpers(fn, case: str, expected: Path) -> None:
    assert fn(case) == expected
