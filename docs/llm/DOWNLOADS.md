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

## Downloads completed (2026-10-09)

The sandbox fix noted in the card feedback held: `~/models/llm` is
writable, and all 3 shortlisted candidates were downloaded there,
one at a time, via `scripts/llm/download_models.py`:

```
$ python3 scripts/llm/download_models.py
...
$ python3 scripts/llm/verify_models.py
[ok] Qwen3.6-27B-Q4_K_M.gguf: size and sha256 verified
[ok] Laguna-XS-2.1-Q3_K_M.gguf: size and sha256 verified
[ok] Qwen3-Coder-30B-A3B-Instruct-IQ4_XS.gguf: size and sha256 verified
$ echo $?
0
```

All three files' size and sha256 match `config/llm_candidates.json`
exactly and are recorded with local path and verification timestamp
in `config/llm_models.json`. Total on-disk size is ~46 GiB
(16817244384 + 15575904128 + 16378076320 bytes), leaving 52 GB free
on the `~/models/llm` filesystem — above the 40 GB floor.

The downloader was interrupted twice by this session's own command
timeouts mid-run (not by the script itself) and resumed cleanly both
times from the `.part` file via HTTP `Range`, re-verifying the
already-completed file(s) first and never recording a partial file
as verified. The final run completed with exit code 0.

Per LLM-01's `deletion_policy`, only the model selected by LLM-05's
benchmark will be kept under `~/models/llm`; the other two candidates'
GGUF files are deleted from local disk (not from Hugging Face) once
that benchmark completes.
