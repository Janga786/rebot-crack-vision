#!/usr/bin/env python3
"""Verify that config/llm_models.json entries exist on disk with matching
size and sha256 against config/llm_candidates.json.

Exit codes follow docs/INTERFACES.md §0:
  0 - all listed models present and verified (may be zero models)
  1 - one or more listed models missing or hash/size mismatch
  2 - usage/config error (missing or malformed config files)
"""
import hashlib
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CANDIDATES_PATH = REPO_ROOT / "config" / "llm_candidates.json"
MODELS_JSON_PATH = REPO_ROOT / "config" / "llm_models.json"
CHUNK_SIZE = 8 * 1024 * 1024


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(CHUNK_SIZE), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    if not CANDIDATES_PATH.exists():
        print(f"[error] missing {CANDIDATES_PATH}", file=sys.stderr)
        return 2
    candidates = {c["file"]: c for c in json.loads(CANDIDATES_PATH.read_text())["candidates"]}

    if not MODELS_JSON_PATH.exists():
        print(f"[info] {MODELS_JSON_PATH} does not exist yet; nothing to verify")
        return 0

    try:
        manifest = json.loads(MODELS_JSON_PATH.read_text())
    except json.JSONDecodeError as exc:
        print(f"[error] malformed {MODELS_JSON_PATH}: {exc}", file=sys.stderr)
        return 2

    models = manifest.get("models", [])
    if not models:
        print("[info] no models listed in manifest; nothing to verify")
        return 0

    ok = True
    for entry in models:
        filename = entry["file"]
        candidate = candidates.get(filename)
        if candidate is None:
            print(f"[fail] {filename}: not present in llm_candidates.json shortlist", file=sys.stderr)
            ok = False
            continue

        path = Path(entry["path"])
        if not path.exists():
            print(f"[fail] {filename}: {path} does not exist", file=sys.stderr)
            ok = False
            continue

        actual_size = path.stat().st_size
        if actual_size != candidate["size_bytes"]:
            print(
                f"[fail] {filename}: size {actual_size} != expected {candidate['size_bytes']}",
                file=sys.stderr,
            )
            ok = False
            continue

        actual_sha = sha256_file(path)
        if actual_sha != candidate["sha256"]:
            print(
                f"[fail] {filename}: sha256 {actual_sha} != expected {candidate['sha256']}",
                file=sys.stderr,
            )
            ok = False
            continue

        print(f"[ok] {filename}: size and sha256 verified")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
