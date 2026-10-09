# HOST-06.1 — Fresh-clone + from-lock reproducibility audit, with RUNBOOK conformance fixes

**Accepted:** 2026-10-09 · **Work package:** Host platform & reproducibility (L-HOST) · **Track:** software · **Verified commit:** [`4126105`](https://github.com/Janga786/rebot-crack-vision/commit/4126105bf73a4bc6d89ee16b5e664b3243a6ca65)

## What was done

Added an automated audit script that proves this project's environment setup is truly portable: it spins up a full copy of the repo in a scratch location and a brand-new throwaway Python environment built only from the committed lock files, runs the whole test/verification suite from both, then cleans up automatically either way. Running it for real found and fixed two genuine bugs: the lock files alone couldn't install PyTorch without an extra download URL, and the setup guide wrongly claimed one test script auto-detects CPU vs GPU when it doesn't. The project's 358 unit tests and all host/lock consistency checks still pass after the fixes.

Files changed:
- [README.md](https://github.com/Janga786/rebot-crack-vision/blob/4126105bf73a4bc6d89ee16b5e664b3243a6ca65/README.md) (+2 / −0)
- [docs/RUNBOOK.md](https://github.com/Janga786/rebot-crack-vision/blob/4126105bf73a4bc6d89ee16b5e664b3243a6ca65/docs/RUNBOOK.md) (+6 / −2)
- [docs/host/REPRO_AUDIT.md](https://github.com/Janga786/rebot-crack-vision/blob/4126105bf73a4bc6d89ee16b5e664b3243a6ca65/docs/host/REPRO_AUDIT.md) (+262 / −0)
- [scripts/audit_fresh_clone.sh](https://github.com/Janga786/rebot-crack-vision/blob/4126105bf73a4bc6d89ee16b5e664b3243a6ca65/scripts/audit_fresh_clone.sh) (+168 / −0)

Commits: [`4126105`](https://github.com/Janga786/rebot-crack-vision/commit/4126105bf73a4bc6d89ee16b5e664b3243a6ca65)

## Why it was done

A non-destructive audit script proves (a) nothing in this repo is hard-coded to this checkout's absolute path, and (b) the committed conda/pip lock files alone stand up a working environment that passes check_host, verify_lock, the model/smoke checks and the full test suite; RUNBOOK.md is corrected wherever it no longer matches the scripts it documents, and the whole run is recorded as evidence.

- Requirement **REQ-HOST-1**: Reproducible host setup with pinned, recorded versions and an automated drift check
- Requirement **REQ-OPS-2**: Reproducible install/config/tests/evaluation results, handoff docs, requirements→evidence map, honest readiness report

## How it moves the project forward

- Host platform & reproducibility: **6/10** tasks accepted; whole project: **50/93**.
- REQ-HOST-1: 3/6 contributing tasks done
- REQ-OPS-2: 3/6 contributing tasks done
- Verification: automated checks run by the pipeline itself (script-exists ✔, audit-doc ✔, no-stray-worktree ✔, no-stray-conda-env ✔, host-manifest ✔, lock-drift ✔, unit-suite ✔); independent audit accepted it (criteria: 10 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The audit script, docs, and RUNBOOK fixes all hold up under independent re-verification. I ran scripts/audit_fresh_clone.sh myself twice on this host and reproduced the exact same two FAIL lines (build_llama_cpp.sh's git fetch into ~/opt, and `conda create --name` into ~/miniconda3/envs) that REPRO_AUDIT.md attributes to a read-only-mount restriction — confirming this is a real, host-wide sandbox characteristic (findmnt shows / is `ro`) and not a fabricated excuse specific to the implementer's own session. All six relocated-copy checks (check_env, verify_model, smoke_test -
…[920 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.
