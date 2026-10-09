# HOST-06.1 — Fresh-clone + from-lock reproducibility audit, with RUNBOOK conformance fixes

```json card
{
  "kind": "integration",
  "depends_on": [
    "HOST-01",
    "HOST-02",
    "LLM-02",
    "TC-016"
  ],
  "requirements": [
    "REQ-HOST-1",
    "REQ-OPS-2"
  ],
  "scope": {
    "write": [
      "scripts/audit_fresh_clone.sh",
      "docs/host/REPRO_AUDIT.md",
      "docs/RUNBOOK.md",
      "README.md",
      "docs/ENVIRONMENT_SNAPSHOT.md"
    ]
  },
  "inputs": [
    "docs/RUNBOOK.md",
    "docs/host/HOST_MANIFEST.md",
    "docs/llm/RUNTIME.md",
    "docs/INTERFACES.md#0",
    "requirements/crackvision.conda-explicit.txt",
    "requirements/crackvision.pip-freeze.txt",
    "requirements/requirements-core.txt",
    "config/host_versions.json",
    "config/llm_runtime.json",
    "scripts/check_host.py",
    "scripts/verify_lock.py",
    "scripts/llm/build_llama_cpp.sh"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "script-exists",
        "cmd": "test -x scripts/audit_fresh_clone.sh",
        "timeout_s": 10,
        "expect_exit": 0
      },
      {
        "id": "audit-doc",
        "cmd": "test -s docs/host/REPRO_AUDIT.md",
        "timeout_s": 10,
        "expect_exit": 0
      },
      {
        "id": "no-stray-worktree",
        "cmd": "! git worktree list | grep -vi \"$(pwd)\" | grep -qi crackvision-audit",
        "timeout_s": 15,
        "expect_exit": 0
      },
      {
        "id": "no-stray-conda-env",
        "cmd": "! conda env list | grep -q crackvision-audit",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "host-manifest",
        "cmd": "/usr/bin/python3 -sE scripts/check_host.py --manifest config/host_versions.json",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "lock-drift",
        "cmd": "./env.sh python scripts/verify_lock.py",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "unit-suite",
        "cmd": "./env.sh pytest tests/ -q",
        "timeout_s": 1800,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "scripts/audit_fresh_clone.sh is idempotent, non-interactive, and non-destructive: it creates its own `git worktree` at a path outside the repo and (separately) a temporary conda env (e.g. `crackvision-audit`) built only from the committed lock files, exercises the reproducibility checks from/against them, and removes both in a trap that fires on success AND failure — it never leaves either behind, never passes `--record` to check_host.py, and never installs into or otherwise mutates the real `crackvision` conda env, config/host_versions.json, or config/llm_runtime.json.",
      "The worktree half of the script re-runs, from the relocated path and reusing the existing `crackvision` env / already-fetched model weights / already-built llama.cpp (no redundant re-download or CUDA rebuild): `check_env.py`, `verify_model.py --check-hashes`, `smoke_test.py`, `pytest tests/`, `check_host.py`, `verify_lock.py`, and `scripts/llm/build_llama_cpp.sh`'s documented no-op path — proving path-relocatability.",
      "The from-lock half actually creates a new conda env from `requirements/crackvision.conda-explicit.txt` + `requirements/crackvision.pip-freeze.txt`, confirms it installs without error, then tears it down — proving the committed lock is sufficient on its own, not just consistent with the live env (which is all `verify_lock.py` on its own proves).",
      "docs/host/REPRO_AUDIT.md is a real, dated transcript of an actual run (UTC timestamp, exact commands, real exit codes/output excerpts) — not a template — covering both halves above plus the RUNBOOK cross-check below, and states plainly if anything failed and why.",
      "Every command/flag/exit-code claim in docs/RUNBOOK.md was checked against the current `--help`/behaviour of the script it names (check_env.py, fetch_model.py, verify_model.py, smoke_test.py, check_realsense.py, scripts/llm/build_llama_cpp.sh) during this audit; any drift found is fixed in the same change and listed in docs/host/REPRO_AUDIT.md's findings, not silently folded in.",
      "docs/ENVIRONMENT_SNAPSHOT.md is only touched if the audit surfaced an actual, real version drift worth recording — not regenerated speculatively.",
      "README.md's existing Reproducibility doc-index section links docs/host/REPRO_AUDIT.md.",
      "This card does not attempt the requirements→evidence export or final readiness report — that is OPS-03's job, which depends on this card; HOST-06.1 only has to prove and document reproducibility."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "repro-audit+docs",
    "local_ok": false
  },
  "id": "HOST-06.1",
  "title": "Fresh-clone + from-lock reproducibility audit, with RUNBOOK conformance fixes",
  "parent": "HOST-06",
  "outcome": "A non-destructive audit script proves (a) nothing in this repo is hard-coded to this checkout's absolute path, and (b) the committed conda/pip lock files alone stand up a working environment that passes check_host, verify_lock, the model/smoke checks and the full test suite; RUNBOOK.md is corrected wherever it no longer matches the scripts it documents, and the whole run is recorded as evidence."
}
```

HOST-01, HOST-02, LLM-02 and TC-016 are all accepted, so this card can now be fully specified (the parent's `refine_after` condition is satisfied) and is itself the only leaf HOST-06 needs — it is already narrowly scoped to one bounded outcome, matching the precedent of other single-card host audits (HOST-01, HOST-02, TC-016) in this project.

Two things are conflated in REQ-OPS-2's "reproducible install" language and both need real (not just documented) evidence, so do both in `scripts/audit_fresh_clone.sh`:

1. **Path-relocatability.** Nothing about this repo should assume it lives at its current checkout path. Add a `git worktree add <tmp-path> HEAD` (or `mktemp -d` + local `git clone`), `cd` into it, and re-run the fast reproducibility surface from there: `./env.sh python scripts/check_env.py`, `./env.sh python scripts/verify_model.py --check-hashes`, `./env.sh python scripts/smoke_test.py`, `./env.sh pytest tests/ -v`, `/usr/bin/python3 -sE scripts/check_host.py --manifest config/host_versions.json`, `./env.sh python scripts/verify_lock.py`, and `scripts/llm/build_llama_cpp.sh` (expect its documented no-op path, not a rebuild). Deliberately reuse the existing `crackvision` conda env, the already-fetched model weights under `models/`, and the already-built `~/opt/llama.cpp` — those artefacts' own reproducibility is HOST-02's and LLM-02's job, already proven; redownloading/rebuilding them here would just waste time without adding evidence. `env.sh` already claims to resolve its own root from `BASH_SOURCE` (see its header comment) — this is the first real test of that claim. Grep the repo first for the current checkout's literal absolute path and for `$HOME`-relative paths to tell which hits are legitimate fixed user-space locations (e.g. `~/opt/llama.cpp` by LLM-02's own design) versus checkout-path leaks that would actually break here.

2. **Lock sufficiency.** `scripts/verify_lock.py` (HOST-02) only proves the *live* env matches the lock files — it can't tell you whether the lock files alone would stand up a *working* env from scratch (ordering/transitive-dependency issues in a from-scratch install wouldn't show up in a live-vs-lock diff). Add a second, independent step: `conda create --name crackvision-audit --file requirements/crackvision.conda-explicit.txt -y`, then `conda run -n crackvision-audit pip install -r requirements/crackvision.pip-freeze.txt`, confirm it installs cleanly (packages should already be in the local conda/pip cache, so this shouldn't need new downloads), then `conda env remove --name crackvision-audit -y` once confirmed. Never leave this env behind, on success or failure — wrap the whole script in a cleanup trap.

While doing this, read `docs/RUNBOOK.md` top to bottom against the actual current `--help`/behaviour of every script it names and fix anything that has drifted (renamed flags, changed exit-code meaning, a step that's now missing or redundant). This is the audit's documentation-correctness half, not just script execution — the outcome text says "documented steps reproduce," and that has to be checked, not assumed.

Write `docs/host/REPRO_AUDIT.md` as the evidence artefact: UTC timestamp, exact commands run, real exit codes and relevant output excerpts (not a template with placeholders), covering both halves of the script above and the RUNBOOK cross-check, including anything that failed and how/whether it was fixed. Link it from README.md's existing Reproducibility section. Leave `docs/ENVIRONMENT_SNAPSHOT.md` untouched unless the audit actually surfaces a real drift worth recording.

Do not build the requirements→evidence map or a final readiness report here — `plan/cards/OPS-03.md` ("Handoff package + final readiness report") already lists HOST-06 as a dependency for exactly that; this card's job is strictly narrower: prove, and durably document, that a fresh clone plus the documented steps reproduce the environment checks and test suite.
