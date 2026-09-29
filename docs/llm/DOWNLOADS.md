# LLM-03 — Download + verify shortlisted models

## Scripts

- `scripts/llm/download_models.py` — downloads the 3 candidates from
  `config/llm_candidates.json` one at a time into `~/models/llm`
  (override with `LLM_MODELS_DIR`). Before each file it checks that
  free disk on that filesystem would stay >= 40 GB after the download
  and refuses to start otherwise. Resumes a partial (`.part`) file via
  HTTP `Range`, or deletes it if the resulting size/sha256 doesn't
  match the candidate record. Only renames `.part` -> the final
  filename, and only appends to `config/llm_models.json`, after the
  sha256 has matched — a file is never recorded as verified until it
  actually is.
- `scripts/llm/verify_models.py` — for every entry currently in
  `config/llm_models.json`, re-checks the file exists, its size, and
  its sha256 against `config/llm_candidates.json`. Exit 0 if all
  listed entries verify (including the case of an empty/absent
  manifest — nothing to verify yet), exit 1 on any mismatch/missing
  file, exit 2 on a missing/malformed config file, per
  docs/INTERFACES.md §0.

## Source verification (done, no download required)

Before writing the downloader, each candidate's repo/file/size/sha256
in `config/llm_candidates.json` (written in LLM-01) was re-checked
live against the Hugging Face tree API, e.g.:

```
curl -s https://huggingface.co/api/models/unsloth/Qwen3-Coder-30B-A3B-Instruct-GGUF/tree/main
```

confirms `Qwen3-Coder-30B-A3B-Instruct-IQ4_XS.gguf`, size
16378076320 bytes, `lfs.oid` (sha256)
`26cd4fef3ada9de84c6c59c4b93f9bc5df4d69a71e18cfe8a17369c2f8374ce7` —
matching the candidates file exactly. The other two repos
(`unsloth/Qwen3.6-27B-GGUF`, `bartowski/Laguna-XS-2.1-GGUF`) also
returned HTTP 200 from the HF API.

## Blocked: downloads not run in this environment

This card's declared extra writable path, `~/models/llm`, is **not
actually writable** in the sandbox this attempt ran in:

```
$ mkdir -p ~/models/llm
mkdir: cannot create directory '/home/boosterk1/models/llm': Read-only file system
$ touch ~/models/testfile
touch: cannot touch '/home/boosterk1/models/testfile': Read-only file system
```

`~/models` itself shows `drwxrwxr-x` ownership but every write into it
fails with EROFS. `mount` inside the sandbox lists explicit rw bind
mounts for `~/.cache`, `~/.claude`, this repo, etc., but has no entry
for `~/models` — it is part of the base read-only home overlay, and no
one ever bind-mounted it read-write despite the card's scope. This
session has no sudo, so it cannot add that mount itself.

Downloading the ~48 GB of candidate GGUF files therefore did not
happen in this attempt. `scripts/llm/download_models.py` is written,
disk-budget-checked, resumable, and hash-verifying, and is ready to
run as soon as `~/models/llm` (or `LLM_MODELS_DIR` pointed at another
writable, >=64 GB-free path) is actually writable — at that point:

```
python3 scripts/llm/download_models.py   # populates config/llm_models.json
python3 scripts/llm/verify_models.py     # exit 0 once all 3 verify
```

`config/llm_models.json` does not exist yet (no downloads occurred),
so `verify_models.py` currently exits 0 trivially (nothing listed to
verify yet) — it will start reporting real per-file size/sha256
results once a download actually lands a file.
