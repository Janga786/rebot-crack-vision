# LLM-03 — Download + verify shortlisted models (bounded, sequential)

**Accepted:** 2026-10-09 · **Work package:** Local coding model (L-LLM) · **Track:** automation · **Verified commit:** [`4dcd5c0`](https://github.com/Janga786/rebot-crack-vision/commit/4dcd5c005ded15cf214ebbe3f7f2e59eb54e65cd)

## What was done

All three shortlisted local coding-model GGUF files remain downloaded and verified; re-running the verification script still exits 0 with no issues.

Files changed:
- [config/llm_models.json](https://github.com/Janga786/rebot-crack-vision/blob/4dcd5c005ded15cf214ebbe3f7f2e59eb54e65cd/config/llm_models.json) (+30 / −0)
- [docs/llm/DOWNLOADS.md](https://github.com/Janga786/rebot-crack-vision/blob/4dcd5c005ded15cf214ebbe3f7f2e59eb54e65cd/docs/llm/DOWNLOADS.md) (+99 / −27)
- [scripts/llm/download_models.py](https://github.com/Janga786/rebot-crack-vision/blob/4dcd5c005ded15cf214ebbe3f7f2e59eb54e65cd/scripts/llm/download_models.py) (+161 / −0)
- [scripts/llm/verify_models.py](https://github.com/Janga786/rebot-crack-vision/blob/4dcd5c005ded15cf214ebbe3f7f2e59eb54e65cd/scripts/llm/verify_models.py) (+89 / −0)

Commits: [`f56f306`](https://github.com/Janga786/rebot-crack-vision/commit/f56f3060080110b39e1ce720080730d63d72fdac), [`4dcd5c0`](https://github.com/Janga786/rebot-crack-vision/commit/4dcd5c005ded15cf214ebbe3f7f2e59eb54e65cd)

## Why it was done

Shortlisted GGUF files are on disk with verified sha256, within the disk budget.

- Requirement **REQ-LLM-1**: Local coding model chosen from current primary sources + a project benchmark (quantization, context, tool reliability, latency, license, VRAM coexistence) with bounded downloads

## How it moves the project forward

- Local coding model: **3/6** tasks accepted; whole project: **63/104**.
- REQ-LLM-1: 2/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (verify ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “All 3 shortlisted GGUF files are present in ~/models/llm with sha256/size matching config/llm_candidates.json, config/llm_models.json records paths/sizes/hashes/verification times, verify_models.py exits 0, and free disk (52G) remains above the 40G floor. This revision only updated docs/config vs. the prior reviewed state; scripts are unchanged.”

## What it unlocks next

- **Ready to start:** LLM-04 — Project benchmark harness for local coding models
