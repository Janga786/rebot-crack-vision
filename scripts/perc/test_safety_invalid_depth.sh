#!/usr/bin/env bash
# scripts/perc/test_safety_invalid_depth.sh — invalid-depth failure-injection test (INT-03.2).
#
# Reuses tools/synth_scene3d.py + the run_synthetic_perception.sh case-fixup pattern (INT-01.1) to
# produce one synthetic case with a real crack, runs it through crackvision.skeleton ->
# crackvision.paths exactly as an operator would, then -- before crackvision.path3d -- overwrites
# the §9 capture record's aligned depth PNG with the D405's documented uint16 "no return" sentinel
# (0, geometry.classify_depth's REASON_ZERO): once for 100% of pixels, once for a partial band
# (~50%, every other row). Each corrupted depth image is run through crackvision.path3d
# independently, under its own scratch root, and the written paths3d.json is checked against
# docs/INTERFACES.md §10 and docs/geometry/PATH3D_VERIFICATION.md's §10.3 gap policy. See
# docs/integration/safety/INVALID_DEPTH.md for the full writeup.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CASE_ID="safety_invalid_depth"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BASE_ROOT="$REPO_ROOT/logs/safety_invalid_depth/$STAMP"
mkdir -p "$BASE_ROOT"

fail() {
    echo "test_safety_invalid_depth.sh: FAIL: $*" >&2
    exit 1
}

echo "test_safety_invalid_depth.sh: scratch root $BASE_ROOT"

# 1. Build one normal synthetic case with a real crack (GEOM-08.8 generator, unmodified): writes
#    the GT skeleton, the dilated mask at naming.mask_path, data/synthetic/{case}_color.png/
#    _depth.png/_gt.json, the §9 capture record and a minimal case_map.json entry.
SEED_ROOT="$BASE_ROOT/seed"
mkdir -p "$SEED_ROOT"
"$REPO_ROOT/env.sh" python "$REPO_ROOT/tools/synth_scene3d.py" --out "$SEED_ROOT" --seed 0 --case-id "$CASE_ID" \
    || fail "tools/synth_scene3d.py failed"

# 2. Stand-in for real OpenCrack segmentation (out of scope, same as run_synthetic_perception.sh)
#    and complete the case_map.json entry to the full docs/INTERFACES.md §2 schema.
"$REPO_ROOT/env.sh" python - "$SEED_ROOT" "$CASE_ID" <<'PYEOF' || fail "case fixup failed"
import hashlib
import json
import shutil
import sys
from pathlib import Path

from crackvision import naming
from PIL import Image

root = Path(sys.argv[1])
case_id = sys.argv[2]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relposix(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


mask_src = root / naming.mask_path(case_id)
prediction_dst = root / naming.prediction_path(case_id)
prediction_dst.parent.mkdir(parents=True, exist_ok=True)
shutil.copyfile(mask_src, prediction_dst)

color_src = root / "data" / "synthetic" / f"{case_id}_color.png"
nnunet_dst = root / naming.nnunet_input_path(case_id)
nnunet_dst.parent.mkdir(parents=True, exist_ok=True)
shutil.copyfile(color_src, nnunet_dst)

with Image.open(color_src) as img:
    width, height = img.size

case_map_path = root / "data" / "case_map.json"
doc = json.loads(case_map_path.read_text(encoding="utf-8"))
doc["cases"] = [c for c in doc.get("cases", []) if c.get("case_id") != case_id]
doc["cases"].append(
    {
        "case_id": case_id,
        "source_path": relposix(color_src, root),
        "source_name": color_src.name,
        "width": width,
        "height": height,
        "downscaled": False,
        "sha256_source": sha256(color_src),
        "sha256_nnunet_input": sha256(nnunet_dst),
    }
)
doc["cases"].sort(key=lambda c: c["case_id"])
case_map_path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
print(f"test_safety_invalid_depth.sh: case_map.json entry for {case_id} written ({width}x{height})")
PYEOF

# 3. crackvision.skeleton + crackvision.paths over the real stand-in mask: produces a normal,
#    valid skeleton/paths.json for the uncorrupted case, shared by both corruption runs below.
"$REPO_ROOT/env.sh" python -m crackvision.skeleton --root "$SEED_ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.skeleton failed"
SKELETON_STATS="$SEED_ROOT/data/skeletons/${CASE_ID}_skeleton_stats.json"
[ -f "$SKELETON_STATS" ] || fail "expected skeleton stats missing: $SKELETON_STATS"
"$REPO_ROOT/env.sh" python - "$SKELETON_STATS" <<'PYEOF' || fail "expected a non-empty skeleton for this synthetic case"
import json, sys
stats = json.load(open(sys.argv[1], encoding="utf-8"))
assert stats["skeleton_pixels"] > 0, f"skeleton_pixels is not > 0: {stats}"
PYEOF

"$REPO_ROOT/env.sh" python -m crackvision.paths --root "$SEED_ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.paths failed"
PATHS_JSON="$SEED_ROOT/data/paths/${CASE_ID}_paths.json"
[ -f "$PATHS_JSON" ] || fail "expected paths output missing: $PATHS_JSON"

# 4. Fork the seed root into two independent scratch roots -- one per corruption variant -- so the
#    only difference between the two path3d runs is the depth PNG, never the paths.json/mask.
FULL_ROOT="$BASE_ROOT/full"
PARTIAL_ROOT="$BASE_ROOT/partial"
cp -r "$SEED_ROOT" "$FULL_ROOT"
cp -r "$SEED_ROOT" "$PARTIAL_ROOT"

# 5. Fault injection: overwrite the §9 capture record's depth.file PNG, same uint16 dtype and
#    (height, width), with the device's documented 0 "no return" sentinel (geometry.classify_depth
#    REASON_ZERO) -- (a) 100% of pixels in $FULL_ROOT, (b) a contiguous band of rows crossing the
#    crack path in $PARTIAL_ROOT, sized to exceed max_gap_px (default 5) so it exercises the §10.3
#    split policy as well as the gap-interpolation policy, rather than being silently absorbed by
#    annulus sampling's own min_fraction_valid tolerance (0.3).
"$REPO_ROOT/env.sh" python - "$FULL_ROOT" "$PARTIAL_ROOT" "$CASE_ID" <<'PYEOF' || fail "depth corruption injection failed"
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

full_root, partial_root, case_id = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]

# Locate the crack path's row span so the partial corruption band actually intersects it (zeroing
# unrelated background rows would never touch an annulus sample and would be a no-op test).
paths_doc = json.loads((partial_root / "data" / "paths" / f"{case_id}_paths.json").read_text(encoding="utf-8"))
dense = paths_doc["components"][0]["main_path"]["dense"]
rows = sorted(p["row"] for p in dense)
row_mid = rows[len(rows) // 2]

for root, mode in [(full_root, "full"), (partial_root, "partial")]:
    capture_path = root / "data" / "captures" / f"{case_id}_capture.json"
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    depth_rel = capture["depth"]["file"]
    depth_path = root / depth_rel

    with Image.open(depth_path) as img:
        assert img.mode in ("I;16", "I;16B", "I"), f"unexpected depth PNG mode: {img.mode}"
        depth = np.array(img, dtype=np.uint16)

    if mode == "full":
        corrupted = np.zeros_like(depth)
        frac = 1.0
    else:
        # A 24-row band centred on the crack's middle row: wide enough (> annulus outer radius 8
        # px on each side, > max_gap_px 5 px of path row-run) to invalidate a real run of path
        # points rather than being absorbed by annulus sampling's min_fraction_valid tolerance.
        corrupted = depth.copy()
        r0 = max(0, row_mid - 12)
        r1 = min(depth.shape[0], row_mid + 12)
        corrupted[r0:r1, :] = 0
        frac = float(np.mean(corrupted == 0))

    assert corrupted.dtype == np.uint16
    assert corrupted.shape == depth.shape
    Image.fromarray(corrupted, mode="I;16").save(depth_path, format="PNG")
    print(f"test_safety_invalid_depth.sh: {mode}: zeroed {frac:.3f} of depth pixels in {depth_path}")
PYEOF

# 6. Run crackvision.path3d (docs/INTERFACES.md §10) against each corrupted depth image.
FULL_LOG="$FULL_ROOT/path3d.stderr.log"
"$REPO_ROOT/env.sh" python -m crackvision.path3d --root "$FULL_ROOT" --cases "$CASE_ID" -q 2>"$FULL_LOG"
FULL_EXIT=$?
cat "$FULL_LOG" >&2

PARTIAL_LOG="$PARTIAL_ROOT/path3d.stderr.log"
"$REPO_ROOT/env.sh" python -m crackvision.path3d --root "$PARTIAL_ROOT" --cases "$CASE_ID" -q 2>"$PARTIAL_LOG"
PARTIAL_EXIT=$?
cat "$PARTIAL_LOG" >&2

# 7. No unhandled exception / traceback in either run's stderr.
for log in "$FULL_LOG" "$PARTIAL_LOG"; do
    if grep -qE "Traceback \(most recent call last\)" "$log"; then
        fail "unhandled exception found in $log"
    fi
done

FULL_PATHS3D="$FULL_ROOT/data/paths3d/${CASE_ID}_paths3d.json"
PARTIAL_PATHS3D="$PARTIAL_ROOT/data/paths3d/${CASE_ID}_paths3d.json"
[ -f "$FULL_PATHS3D" ] || fail "expected paths3d output missing: $FULL_PATHS3D"
[ -f "$PARTIAL_PATHS3D" ] || fail "expected paths3d output missing: $PARTIAL_PATHS3D"

# 8. path3d.py's own contract (src/crackvision/path3d.py main()): a case with zero valid points
#    is never counted among `failed_count` (build_case_paths3d only returns status "failed" for a
#    missing/malformed capture/paths.json/case_map entry, not for no_valid_points -- it still
#    writes a valid paths3d.json with execution_eligible=false). So both runs are expected to exit
#    0 (EXIT_OK), read here from the module's own source rather than assumed.
"$REPO_ROOT/env.sh" python -c "
import inspect
from crackvision import path3d
src = inspect.getsource(path3d.build_case_paths3d)
assert '\"no_valid_points\"' in src, 'expected no_valid_points handling in build_case_paths3d'
" || fail "could not confirm path3d.py's no_valid_points contract from its own source"

[ "$FULL_EXIT" -eq 0 ] || fail "100%-invalid run: crackvision.path3d exited $FULL_EXIT, expected 0 (EXIT_OK per path3d.py's own contract: no_valid_points makes the case execution_eligible=false, never a runtime/precondition failure)"
[ "$PARTIAL_EXIT" -eq 0 ] || fail "partial run: crackvision.path3d exited $PARTIAL_EXIT, expected 0 (EXIT_OK)"

# 9. Validate the written JSON against docs/INTERFACES.md §10 and the §10.3 gap policy.
"$REPO_ROOT/env.sh" python - "$FULL_PATHS3D" "$PARTIAL_PATHS3D" <<'PYEOF' || fail "output validation failed"
import json
import math
import sys

full_path, partial_path = sys.argv[1:3]

# -- 100%-invalid run -------------------------------------------------------
doc = json.load(open(full_path, encoding="utf-8"))
assert doc["schema"] == "crackvision.paths3d/1", f"unexpected schema: {doc.get('schema')}"
assert doc["counts"]["valid"] == 0, f"expected 0 valid points, got {doc['counts']}"
assert doc["counts"]["points"] > 0, "expected a non-empty skeleton to have fed non-zero points in"
assert doc["execution_eligible"] is False, "execution_eligible is not False for an all-invalid-depth case"
assert "no_valid_points" in doc["ineligible_reasons"], (
    f"ineligible_reasons does not contain no_valid_points: {doc['ineligible_reasons']}"
)

geometry_fields = ("depth_m", "p_optical_m", "p_base_m", "n_base", "normal_rms_m",
                    "sigma_normal_rad", "cov_base_m2", "sigma_base_m")
n_points_checked = 0
for comp in doc["components"]:
    for polyline in comp["polylines"]:
        for point in polyline["points"]:
            n_points_checked += 1
            assert point["valid"] is False, f"point is valid=True in the all-invalid-depth run: {point}"
            assert point["reason"], f"invalid point has no reason: {point}"
            for field in geometry_fields:
                value = point[field]
                assert value is None, f"invalid point has non-null {field}={value!r} (expected null)"
assert n_points_checked > 0, "no points were checked in the 100%-invalid run"
print(f"test_safety_invalid_depth.sh: 100%-invalid run: {n_points_checked} points, all valid=False with a "
      f"reason and null geometry; execution_eligible=False; ineligible_reasons={doc['ineligible_reasons']}")

# -- partial (banded) run ----------------------------------------------------
pdoc = json.load(open(partial_path, encoding="utf-8"))
assert pdoc["schema"] == "crackvision.paths3d/1", f"unexpected schema: {pdoc.get('schema')}"
assert pdoc["counts"]["invalid_by_reason"], (
    "partial run left zero invalid points -- the injected band did not actually corrupt enough "
    f"of the annulus to invalidate any path point, counts={pdoc['counts']}"
)
# The injected band (24 rows) is wider than max_gap_px (5), so §10.3 requires a split, not a
# bridge, across the corrupted run.
total_segments = sum(len(pl["segments"]) for comp in pdoc["components"] for pl in comp["polylines"])
assert total_segments > 1, f"expected the oversized invalid run to split the polyline, got {total_segments} segment(s)"

# §10.3: invalid points carry null geometry, never a fabricated coordinate; interpolated points
# (filled by the gap policy) are the only invalid-adjacent points allowed non-null geometry, and
# they are marked valid=True, interpolated=True.
n_partial_points = 0
n_interpolated = 0
for comp in pdoc["components"]:
    for polyline in comp["polylines"]:
        for point in polyline["points"]:
            n_partial_points += 1
            if point["interpolated"]:
                n_interpolated += 1
                assert point["valid"] is True, f"interpolated point is not valid=True: {point}"
                for field in ("p_base_m", "n_base"):
                    vec = point[field]
                    assert vec is not None, f"interpolated point has null {field}: {point}"
                    assert all(math.isfinite(c) for c in vec), f"interpolated point has non-finite {field}: {point}"
            elif not point["valid"]:
                assert point["reason"], f"invalid point has no reason: {point}"
                for field in ("p_optical_m", "p_base_m", "n_base"):
                    assert point[field] is None, (
                        f"non-interpolated invalid point has non-null {field}={point[field]!r} "
                        f"(fabricated coordinate for an invalid pixel): {point}"
                    )
assert n_partial_points > 0, "no points were checked in the partial run"
print(f"test_safety_invalid_depth.sh: partial run: {n_partial_points} points, {n_interpolated} interpolated "
      f"by the §10.3 gap policy, counts={pdoc['counts']}, execution_eligible={pdoc['execution_eligible']}")
PYEOF

echo "test_safety_invalid_depth.sh: OK ($BASE_ROOT) full_path3d=$FULL_EXIT partial_path3d=$PARTIAL_EXIT"
exit 0
