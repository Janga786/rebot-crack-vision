#!/usr/bin/env bash
# scripts/perc/run_synthetic_perception.sh — synthetic segmentation-to-paths-to-3D run (INT-01.1).
#
# tools/synth_scene3d.py (GEOM-08.8, unmodified) --> crackvision.visualize --> crackvision.skeleton
# --> crackvision.paths --> crackvision.visualize_paths --> crackvision.path3d, every perception/
# geometry step invoked exactly as an operator would via `./env.sh python -m crackvision.*`, under
# one scratch root logs/synthetic_perception/<stamp>/ (never data/ or config/ in the repo tree).
# No ROS, no MoveIt, no hardware; scripts/ros/run_cartesian_synthetic.sh (MOT-06.10) covers the
# ROS/MoveIt-mock half of INT-01 and never exercises visualize/skeleton/visualize_paths itself,
# since its generator-produced case feeds crackvision.paths directly from a hand-written
# ground-truth skeleton. This script closes that gap: the generator's own dilated mask
# (naming.mask_path) is copied to naming.prediction_path as the one deliberate stand-in for the
# real OpenCrack segmentation step, which is out of scope here (ADR-010, accepted PERC cards) --
# everything downstream of that one substitution is the real CLI surface.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CASE_ID="synth_int01"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ROOT="$REPO_ROOT/logs/synthetic_perception/$STAMP"
mkdir -p "$ROOT"

fail() {
    echo "run_synthetic_perception.sh: FAIL: $*" >&2
    exit 1
}

echo "run_synthetic_perception.sh: scratch root $ROOT"

# 1. Synthetic scene (GEOM-08.8 generator, unmodified): writes the GT skeleton, the dilated mask
#    at naming.mask_path, data/synthetic/{case}_color.png/_depth.png/_gt.json, the §9 capture
#    record and a minimal case_map.json entry.
"$REPO_ROOT/env.sh" python "$REPO_ROOT/tools/synth_scene3d.py" --out "$ROOT" --seed 0 --case-id "$CASE_ID" \
    || fail "tools/synth_scene3d.py failed"

# 2. Stand in for the real OpenCrack segmentation step (out of scope here) and complete the
#    case_map.json entry to the full docs/INTERFACES.md §2 schema -- the generator itself only
#    writes {case_id, downscaled}.
"$REPO_ROOT/env.sh" python - "$ROOT" "$CASE_ID" <<'PYEOF' || fail "case fixup (prediction/case_map) failed"
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


# The one deliberate stand-in for real OpenCrack/nnU-Net segmentation (ADR-010, accepted PERC
# cards: real inference is out of scope for this card): copy the generator's own dilated mask to
# the path crackvision.skeleton reads a prediction from.
mask_src = root / naming.mask_path(case_id)
prediction_dst = root / naming.prediction_path(case_id)
prediction_dst.parent.mkdir(parents=True, exist_ok=True)
shutil.copyfile(mask_src, prediction_dst)

# data/synthetic/{case}_color.png never goes near nnU-Net (no nnUNetv2_* call anywhere in this
# script) -- it exists only so case_map.json's sha256_nnunet_input field means something.
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
print(f"run_synthetic_perception.sh: case_map.json entry for {case_id} rewritten ({width}x{height})")
PYEOF

# 3. crackvision.visualize (PERC-01-adjacent): mask/overlay/comparison from the stand-in prediction.
"$REPO_ROOT/env.sh" python -m crackvision.visualize --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.visualize failed"
for f in "data/overlays/${CASE_ID}_mask.png" "data/overlays/${CASE_ID}_overlay.png" "data/comparisons/${CASE_ID}_comparison.png"; do
    [ -f "$ROOT/$f" ] || fail "expected visualize output missing: $ROOT/$f"
done

# 4. crackvision.skeleton (TC-010/PERC-02): real remove_small_objects + skimage.skeletonize over
#    the stand-in prediction -- the one step MOT-06.10's own synthetic script never exercises.
"$REPO_ROOT/env.sh" python -m crackvision.skeleton --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.skeleton failed"
SKELETON_PNG="$ROOT/data/skeletons/${CASE_ID}_skeleton.png"
SKELETON_STATS="$ROOT/data/skeletons/${CASE_ID}_skeleton_stats.json"
[ -f "$SKELETON_PNG" ] || fail "expected skeleton output missing: $SKELETON_PNG"
[ -f "$SKELETON_STATS" ] || fail "expected skeleton stats missing: $SKELETON_STATS"

# 5. crackvision.paths (PERC-03): pixel graph -> ordered polylines.
"$REPO_ROOT/env.sh" python -m crackvision.paths --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.paths failed"
PATHS_JSON="$ROOT/data/paths/${CASE_ID}_paths.json"
[ -f "$PATHS_JSON" ] || fail "expected paths output missing: $PATHS_JSON"

# 6. crackvision.visualize_paths (PERC-04).
"$REPO_ROOT/env.sh" python -m crackvision.visualize_paths --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.visualize_paths failed"
PATHS_OVERLAY="$ROOT/data/paths/${CASE_ID}_paths_overlay.png"
[ -f "$PATHS_OVERLAY" ] || fail "expected paths overlay missing: $PATHS_OVERLAY"

# 7. crackvision.path3d (docs/INTERFACES.md §10): 3D lift + waypoints over the nominal capture.
"$REPO_ROOT/env.sh" python -m crackvision.path3d --root "$ROOT" --cases "$CASE_ID" -q \
    || fail "crackvision.path3d failed"
PATHS3D_JSON="$ROOT/data/paths3d/${CASE_ID}_paths3d.json"
[ -f "$PATHS3D_JSON" ] || fail "expected paths3d output missing: $PATHS3D_JSON"

# 8. Validate the skeleton and paths3d outputs with their own on-disk JSON, independent of each
#    CLI's own exit code / log lines.
"$REPO_ROOT/env.sh" python - "$SKELETON_STATS" "$PATHS3D_JSON" <<'PYEOF' || fail "output validation failed"
import json
import sys

skeleton_stats_path, paths3d_path = sys.argv[1:3]

stats = json.load(open(skeleton_stats_path, encoding="utf-8"))
assert stats["skeleton_pixels"] > 0, f"skeleton_pixels is not > 0: {stats}"

doc = json.load(open(paths3d_path, encoding="utf-8"))
assert doc["execution_eligible"] is False, "paths3d execution_eligible is not false"
required_reasons = {"wrist_camera_nominal", "tool_nominal", "optical_frames_nominal", "synthetic_capture"}
missing = required_reasons - set(doc["ineligible_reasons"])
assert not missing, f"missing expected ineligible reasons: {missing}"

component_count = len(doc["components"])
segment_count = sum(len(polyline["segments"]) for comp in doc["components"] for polyline in comp["polylines"])
print(
    f"run_synthetic_perception.sh: paths3d components={component_count} segments={segment_count} "
    f"points={doc['counts']['points']} valid={doc['counts']['valid']}"
)
PYEOF

echo "run_synthetic_perception.sh: OK ($ROOT)"
exit 0
