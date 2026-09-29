#!/usr/bin/env python3
"""Bounded, sequential downloader for the LLM-01 shortlisted GGUF candidates.

Downloads one file at a time from config/llm_candidates.json into
~/models/llm, verifying sha256 after each download and refusing to
start any download that would drop free disk below the configured
floor. Writes config/llm_models.json with the resulting local paths,
sizes, hashes and verification timestamps.
"""
import hashlib
import json
import os
import shutil
import sys
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CANDIDATES_PATH = REPO_ROOT / "config" / "llm_candidates.json"
MODELS_JSON_PATH = REPO_ROOT / "config" / "llm_models.json"
MODELS_DIR = Path(os.environ.get("LLM_MODELS_DIR", str(Path.home() / "models" / "llm")))
MIN_FREE_DISK_GB = 40
CHUNK_SIZE = 8 * 1024 * 1024


def free_disk_gb(path: Path) -> float:
    path.mkdir(parents=True, exist_ok=True)
    usage = shutil.disk_usage(path)
    return usage.free / (1024 ** 3)


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(CHUNK_SIZE), b""):
            h.update(chunk)
    return h.hexdigest()


def hf_url(repo: str, filename: str) -> str:
    return f"https://huggingface.co/{repo}/resolve/main/{filename}"


def download_one(candidate: dict) -> dict:
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    filename = candidate["file"]
    dest = MODELS_DIR / filename
    partial = dest.with_suffix(dest.suffix + ".part")
    expected_size = candidate["size_bytes"]
    expected_sha = candidate["sha256"]

    if dest.exists():
        if dest.stat().st_size == expected_size and sha256_file(dest) == expected_sha:
            print(f"[skip] {filename} already verified on disk")
            return {
                "name": candidate["name"],
                "file": filename,
                "path": str(dest),
                "size_bytes": expected_size,
                "sha256": expected_sha,
                "verified_at": datetime.now(timezone.utc).isoformat(),
            }
        # Stale/corrupt full-size file: remove it, never leave as "verified".
        dest.unlink()

    free_before = free_disk_gb(MODELS_DIR)
    projected_free = free_before - (expected_size / (1024 ** 3))
    if projected_free < MIN_FREE_DISK_GB:
        print(
            f"[refuse] downloading {filename} would leave {projected_free:.1f} GB free "
            f"(< {MIN_FREE_DISK_GB} GB floor); skipping",
            file=sys.stderr,
        )
        if partial.exists():
            partial.unlink()
        return None

    resume_from = partial.stat().st_size if partial.exists() else 0
    url = hf_url(candidate["repo"], filename)
    req = urllib.request.Request(url)
    if resume_from:
        req.add_header("Range", f"bytes={resume_from}-")
        print(f"[resume] {filename} from byte {resume_from}")
    else:
        print(f"[start] {filename} ({expected_size / 1e9:.2f} GB)")

    mode = "ab" if resume_from else "wb"
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            with open(partial, mode) as out:
                downloaded = resume_from
                last_report = time.monotonic()
                while True:
                    chunk = resp.read(CHUNK_SIZE)
                    if not chunk:
                        break
                    out.write(chunk)
                    downloaded += len(chunk)
                    now = time.monotonic()
                    if now - last_report > 10:
                        print(f"  {filename}: {downloaded / 1e9:.2f} / {expected_size / 1e9:.2f} GB")
                        last_report = now
    except Exception as exc:
        print(f"[error] download of {filename} failed: {exc}; leaving partial for resume", file=sys.stderr)
        return None

    if partial.stat().st_size != expected_size:
        print(
            f"[error] {filename}: size mismatch after download "
            f"({partial.stat().st_size} != {expected_size}); deleting partial",
            file=sys.stderr,
        )
        partial.unlink()
        return None

    actual_sha = sha256_file(partial)
    if actual_sha != expected_sha:
        print(
            f"[error] {filename}: sha256 mismatch ({actual_sha} != {expected_sha}); deleting",
            file=sys.stderr,
        )
        partial.unlink()
        return None

    partial.rename(dest)
    print(f"[ok] {filename} verified sha256={actual_sha}")
    return {
        "name": candidate["name"],
        "file": filename,
        "path": str(dest),
        "size_bytes": expected_size,
        "sha256": expected_sha,
        "verified_at": datetime.now(timezone.utc).isoformat(),
    }


def main() -> int:
    candidates = json.loads(CANDIDATES_PATH.read_text())["candidates"]
    if MODELS_JSON_PATH.exists():
        existing = json.loads(MODELS_JSON_PATH.read_text())
    else:
        existing = {"min_free_disk_gb": MIN_FREE_DISK_GB, "models_dir": str(MODELS_DIR), "models": []}
    by_file = {m["file"]: m for m in existing.get("models", [])}

    for candidate in candidates:
        result = download_one(candidate)
        if result is None:
            continue
        by_file[result["file"]] = result
        existing["models"] = list(by_file.values())
        existing["min_free_disk_gb"] = MIN_FREE_DISK_GB
        existing["models_dir"] = str(MODELS_DIR)
        MODELS_JSON_PATH.write_text(json.dumps(existing, indent=2) + "\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
