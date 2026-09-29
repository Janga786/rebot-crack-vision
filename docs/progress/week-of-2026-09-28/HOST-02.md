# HOST-02 — Lock the crackvision environment + verify-lock script

**Accepted:** 2026-09-29 · **Work package:** Host platform & reproducibility (L-HOST) · **Track:** software · **Verified commit:** [`8c9719c`](https://github.com/Janga786/rebot-crack-vision/commit/8c9719c6292cce48b531738989116bad503bd04f)

## What was done

The crackvision perception environment can now be locked and its drift automatically checked: `verify_lock.py` correctly reports a clean match against the committed lock files, and `rebuild_env.sh --dry-run` prints the exact commands to recreate the environment under a new name without touching the existing one. The prior false-alarm (the checker flagging drift every time a git commit was made, because the repo's own editable install pin naturally changes with each commit) is fixed by ignoring that self-referential commit hash while still comparing every real dependency version exactly.</presentation_summary>

Files changed:
- [docs/host/ENV_LOCK.md](https://github.com/Janga786/rebot-crack-vision/blob/8c9719c6292cce48b531738989116bad503bd04f/docs/host/ENV_LOCK.md) (+71 / −0)
- [requirements/crackvision.conda-explicit.txt](https://github.com/Janga786/rebot-crack-vision/blob/8c9719c6292cce48b531738989116bad503bd04f/requirements/crackvision.conda-explicit.txt) (+41 / −0)
- [requirements/crackvision.pip-freeze.txt](https://github.com/Janga786/rebot-crack-vision/blob/8c9719c6292cce48b531738989116bad503bd04f/requirements/crackvision.pip-freeze.txt) (+110 / −0)
- [scripts/rebuild_env.sh](https://github.com/Janga786/rebot-crack-vision/blob/8c9719c6292cce48b531738989116bad503bd04f/scripts/rebuild_env.sh) (+92 / −0)
- [scripts/verify_lock.py](https://github.com/Janga786/rebot-crack-vision/blob/8c9719c6292cce48b531738989116bad503bd04f/scripts/verify_lock.py) (+163 / −2)

Commits: [`24f774c`](https://github.com/Janga786/rebot-crack-vision/commit/24f774cb196ed7560112870a397c4afb3e94fed5), [`8c9719c`](https://github.com/Janga786/rebot-crack-vision/commit/8c9719c6292cce48b531738989116bad503bd04f)

## Why it was done

The perception environment can be recreated exactly from committed lock files, and drift is detectable.

- Requirement **REQ-HOST-1**: Reproducible host setup with pinned, recorded versions and an automated drift check

## How it moves the project forward

- Host platform & reproducibility: **5/9** tasks accepted; whole project: **34/74**.
- REQ-HOST-1: 2/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (verify ✔, dry-run ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The fix narrowly normalizes only the repo's self-referential editable-install commit-hash line (via an exact-URL regex) before diffing, on both locked and live sides, while comparing every other package pin verbatim. Both acceptance checks pass on re-run.”

## What it unlocks next

- Closer: HOST-06 — Host reproducibility audit (fresh clone) (still needs LLM-02, LLM-02)
