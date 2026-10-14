#!/usr/bin/env bash
# scripts/perc/test_safety_empty_mask.sh — empty-mask failure-injection test (INT-03.1).
#
# Reuses tools/synth_scene3d.py + the run_synthetic_perception.sh case-fixup pattern (INT-01.1) to
# produce one synthetic case, then overwrites the stand-in prediction mask at naming.prediction_path
# with an all-zero PNG of the same (height, width) -- a legitimate "no crack detected" input, not a
# malformed file. Runs crackvision.skeleton -> crackvision.paths -> crackvision.path3d exactly as an
# operator would via `./env.sh python -m crackvision.<module>`, under a fresh logs/ scratch root, and
# asserts the chain reaches a clean, schema-valid empty result: skeleton_pixels=0, paths.json
# component_count=0, paths3d.json components=[] with execution_eligible=false and no_valid_points in
# ineligible_reasons. See docs/integration/safety/EMPTY_MASK.md for the full writeup.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CASE_ID="safety_empty_mask"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ROOT="$REPO_ROOT/logs/safety_empty_mask/$STAMP"
mkdir -p "$ROOT"

fail() {
    echo "test_safety_empty_mask.sh: FAIL: $*" >&2
    exit 1
}

echo "test_safety_empty_mask.sh: scratch root $ROOT"

# 1. Synthetic scene (GEOM-08.8 generator, unmodified): writes the GT skeleton, the dilated mask at
#    naming.mask_path, data/synthetic/{case}_color.png/_depth.png/_gt.json, the §9 capture record and
#    a minimal case_map.json entry.
"$REPO_ROOT/env.sh" python "$REPO_ROOT/tools/synth_scene3d.py" --out "$ROOT" --seed 0 --case-id "$CASE_ID" \
    || fail "tools/synth_scene3d.py failed"

# 2. Complete the case_map.json entry to the full docs/INTERFACES.md §2 schema (the generator itself
#    only writes {case_id, downscaled}), then overwrite naming.prediction_path with an all-zero mask
#    of the same dimensions -- the fault being injected. This is a legitimate "no crack detected"
#    prediction, never a malformed/wrong-shaped file.
"$REPO_ROOT/env.sh" python - "$ROOT" "$CASE_ID" <<'PYEOF' || fail "case fixup / fault injection failed"
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
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


color_src = root / "data" / "synthetic" / f"{case_id}_color.png"
nnunet_dst = root / naming.nnunet_input_path(case_id)
nnunet_dst.parent.mkdir(parents=True, exist_ok=True)
import shutil
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

# Fault injection: an all-zero prediction mask of the same (height, width) -- a legitimate
# crack-free result, not a malformed file.
prediction_dst = root / naming.prediction_path(case_id)
prediction_dst.parent.mkdir(parents=True, exist_ok=True)
zero_mask = np.zeros((height, width), dtype=np.uint8)
Image.fromarray(zero_mask, mode="L").save(prediction_dst, format="PNG")

print(f"test_safety_empty_mask.sh: case_map.json entry + all-zero prediction for {case_id} written ({width}x{height})")
PYEOF

# 3. crackvision.skeleton (TC-010/PERC-02) over the all-zero prediction.
SKELETON_LOG="$ROOT/skeleton.stderr.log"
"$REPO_ROOT/env.sh" python -m crackvision.skeleton --root "$ROOT" --cases "$CASE_ID" -q 2>"$SKELETON_LOG"
SKELETON_EXIT=$?
cat "$SKELETON_LOG" >&2
[ "$SKELETON_EXIT" -eq 0 ] || fail "crackvision.skeleton exited $SKELETON_EXIT, expected 0 (EXIT_OK; a crack-free mask is not a runtime/precondition failure per skeleton.py's own contract)"

SKELETON_STATS="$ROOT/data/skeletons/${CASE_ID}_skeleton_stats.json"
[ -f "$SKELETON_STATS" ] || fail "expected skeleton stats missing: $SKELETON_STATS"

# 4. crackvision.paths (PERC-03) over the empty skeleton.
PATHS_LOG="$ROOT/paths.stderr.log"
"$REPO_ROOT/env.sh" python -m crackvision.paths --root "$ROOT" --cases "$CASE_ID" -q 2>"$PATHS_LOG"
PATHS_EXIT=$?
cat "$PATHS_LOG" >&2
[ "$PATHS_EXIT" -eq 0 ] || fail "crackvision.paths exited $PATHS_EXIT, expected 0 (EXIT_OK; zero components is not a runtime/precondition failure per paths.py's own contract)"

PATHS_JSON="$ROOT/data/paths/${CASE_ID}_paths.json"
[ -f "$PATHS_JSON" ] || fail "expected paths output missing: $PATHS_JSON"

# 5. crackvision.path3d (docs/INTERFACES.md §10) over the zero-component paths.json.
PATH3D_LOG="$ROOT/path3d.stderr.log"
"$REPO_ROOT/env.sh" python -m crackvision.path3d --root "$ROOT" --cases "$CASE_ID" -q 2>"$PATH3D_LOG"
PATH3D_EXIT=$?
cat "$PATH3D_LOG" >&2
[ "$PATH3D_EXIT" -eq 0 ] || fail "crackvision.path3d exited $PATH3D_EXIT, expected 0 (EXIT_OK; no_valid_points makes the case execution_eligible=false, never a runtime/precondition failure, per path3d.py's own contract)"

PATHS3D_JSON="$ROOT/data/paths3d/${CASE_ID}_paths3d.json"
[ -f "$PATHS3D_JSON" ] || fail "expected paths3d output missing: $PATHS3D_JSON"

# 6. No traceback / unhandled exception in any captured stderr.
for log in "$SKELETON_LOG" "$PATHS_LOG" "$PATH3D_LOG"; do
    if grep -qE "Traceback \(most recent call last\)" "$log"; then
        fail "unhandled exception found in $log"
    fi
done

# 7. Validate the written JSON against docs/INTERFACES.md #10 and the acceptance criteria.
"$REPO_ROOT/env.sh" python - "$SKELETON_STATS" "$PATHS_JSON" "$PATHS3D_JSON" <<'PYEOF' || fail "output validation failed"
import json
import sys

skeleton_stats_path, paths_json_path, paths3d_path = sys.argv[1:4]

stats = json.load(open(skeleton_stats_path, encoding="utf-8"))
assert stats["skeleton_pixels"] == 0, f"skeleton_pixels is not 0: {stats}"

paths_doc = json.load(open(paths_json_path, encoding="utf-8"))
assert paths_doc["component_count"] == 0, f"paths.json component_count is not 0: {paths_doc}"
assert paths_doc["components"] == [], f"paths.json components is not []: {paths_doc}"

doc = json.load(open(paths3d_path, encoding="utf-8"))
assert doc["schema"] == "crackvision.paths3d/1", f"unexpected schema: {doc.get('schema')}"
assert doc["components"] == [], f"paths3d.json components is not []: {doc}"
assert doc["counts"]["points"] == 0, f"paths3d.json counts.points is not 0: {doc['counts']}"
assert doc["counts"]["valid"] == 0, f"paths3d.json counts.valid is not 0: {doc['counts']}"
assert doc["execution_eligible"] is False, "paths3d.json claims execution_eligible=true for a crack-free case"
assert "no_valid_points" in doc["ineligible_reasons"], (
    f"paths3d.json does not list no_valid_points among ineligible_reasons: {doc['ineligible_reasons']}"
)

print(
    f"test_safety_empty_mask.sh: validated skeleton_pixels=0, component_count=0, "
    f"paths3d components=[] execution_eligible=False ineligible_reasons={doc['ineligible_reasons']}"
)
PYEOF

echo "test_safety_empty_mask.sh: OK ($ROOT) skeleton=$SKELETON_EXIT paths=$PATHS_EXIT path3d=$PATH3D_EXIT"
exit 0
