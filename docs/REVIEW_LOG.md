# REVIEW_LOG.md — append-only audit history

One entry per independent review. Written by the Opus auditor at the review gate, never by an
implementation agent. Never edit or delete an earlier entry.

---

## 2026-09-17 · TC-001 — Project scaffolding and Git hygiene · **APPROVED**

- **Review id:** `TC-001-rev1-45a88930e080` · **Reviewer:** opus
- **Implementation run:** `TC-001-r0-06a169d4e5f5` (sonnet) · **Commit:** `3025616`
- **Criteria:** 9 / 9 PASS · **Full report:** `.task_orchestrator/reviews/TC-001-rev1-45a88930e080/report.md`

Run state was recovered by `cv-recover`, not witnessed, so every claim was re-derived from the
filesystem and git and the card's verification commands were re-executed.

**Verified:** commit contains exactly the card's 27 files and nothing else (no `git add -A`, no
co-author trailer, one commit on `main`, author `Gavinw575`, largest blob 5 943 B). `config/project.yaml`
is byte-identical to `INTERFACES.md` §4 (`diff` clean). `.gitignore` proof re-run verbatim: 4 ignored,
0 visible. `config.py` / `logging_setup.py` re-exercised with 39 assertions — all pass, including
deep-merge over `DEFAULT_CONFIG`, `ConfigError` on malformed YAML, the six §0.3 flags, the 0/1/2/3
exit codes, UTC log format, and the §0.4 summary schema in both `<tool>_<ts>.json` and
`<tool>_latest.json`. Planning docs and ADRs untouched (mtimes pre-date the run); `~/.bashrc`,
`~/.gitconfig`, global git config, conda envs and the orchestration gate all unmodified.

**Non-blocking observations:** `load_config` silently defaults when an explicit `--config` path is
missing (a usage error that §0.2 maps to exit 2); `find_root` raises instead of falling back on an
invalid `$CRACKVISION_ROOT` (accepted — the safer reading); completion report says "25 named files"
where 27 were committed.

**Integration risk to watch:** the pre-existing planning docs (`docs/*.md`, `docs/adr/`, `task_cards/`)
are untracked and not ignored, so `git status` shows ten `??` entries for every later card — a human
decision, not TC-001 rework, but **TC-002** is the first card to meet it. Also: `DEFAULT_CONFIG`
(`config.py:14-60`) duplicates `config/project.yaml` by design; any card editing the YAML must edit
both.

**Next:** TC-002 unblocked.

---

## 2026-09-17 · TC-002 — Python environment, dependencies and `env.sh` · **APPROVED**

- **Review id:** `TC-002-rev1-5d6d11f000c3` · **Reviewer:** opus
- **Implementation run:** `TC-002-r0-875b881027d5` (sonnet) · **Commit:** `3d8247f`
- **Criteria:** 13 / 13 PASS · **Full report:** `.task_orchestrator/reviews/TC-002-rev1-5d6d11f000c3/report.md`

Every acceptance command was re-executed independently; the report's pasted outputs match the
filesystem value for value.

**Verified:** commit contains exactly the card's five files (`env.sh`, `requirements/*`), author and
committer `Gavinw575`, no co-author trailer, no `git add -A`; `docs/COMPLETION_LOG.md` correctly left
uncommitted per the card's commit guidance, and the ten pre-existing untracked planning paths were
not swept in. The critical ROS scrub was proven under a **forced** `PYTHONPATH=/opt/ros/humble/...3.10...`
and `LD_LIBRARY_PATH` containing `/opt/ros` + `/usr/local/cuda-11.8` — output `python 3.11.16`,
`ROS leaks: []`, `LD_LIBRARY_PATH` filtered to the surviving entry (not blanked). torch
`2.14.0+cu126` / CUDA 12.6 / `True` / `NVIDIA GeForce RTX 3090`; nnunetv2 `2.8.1` imports and
`nnUNetv2_predict_from_modelfolder --help` exits 0; `import crackvision` resolves to `src/` (real
editable install); all three `nnUNet_*` roots inside `$CRACKVISION_ROOT` with `nnUNet_compile=f`.
`requirements-core.txt` is 83/83 `==`-pinned and **regenerates byte-identically** from its own
documented command. `env.sh` has zero literal `/home/`, works sourced and exec'd, under `set -u`,
from a foreign cwd and via a relative path, and fails loudly with a clear message (rc 1, command not
run) when the env is absent. ADR-004/005/007 obeyed; no training, no depth projection, no skeleton
traversal, no ROS import; `nnunet/`, `models/`, `data/` still hold only TC-001's `.gitkeep` files.
The declared deviation (excluding `cuda-bindings` / `cuda-pathfinder` / `cuda-toolkit`) checks out —
`pip show` reports all three as torch-owned.

**System safety:** no `sudo`, no `/usr/local/cuda*`, `/opt/ros/**` or driver touch. `~/.bashrc`
unmodified (content *and* mtime, 2026-06-03). File-level scan of the run window shows **zero**
changes in `base`, `isaaclab`, `lerobot`, `navila`, `navila-vila`, `vlnce-isaac`; `isaaclab` torch
still `2.7.0+cu128`. Only shared-cache writes under `~/miniconda3/pkgs`, inherent to `conda create`.
Orchestration gate intact — all `cv-*` commands predate the run; `.task_orchestrator/` untouched by
the agent.

**Non-blocking findings:** (F-1, MINOR) `env.sh:92` uses `[ "$#" -gt 0 ]` as the proxy for "not
sourced", so sourcing it from a script that itself has positional parameters `exec`s the *host
script's* arguments (reproduced: host exits 127). This is the card's own prescribed logic, not agent
error, and harmless for every documented usage — but **TC-011** owns `run_test.sh`, which ADR-007
routes through `env.sh`; write it as `./env.sh pytest …`, or gate on `[ "${BASH_SOURCE[0]}" = "$0" ]`.
(F-2, cosmetic) report says mode `0755`, on-disk is `0775`; committed git mode is `100755`.
(F-3, info) `setuptools` omitted from the pins by `pip freeze`'s default exclusion; covered by the
conda `pip` dependency.

**Integration risks to watch:** **TC-016** — `environment.yml` pulls `nnunetv2==2.8.1`, which depends
on torch, so a clean `conda env create -f environment.yml` *without* the cu126 step first would
silently land a CPU wheel; the two-step order must be documented. **TC-012** — `pyrealsense2
2.58.4.10922` is already installed and importable in the env (correctly excluded from the pins); do
not assume a pristine state. **TC-007/009/010** — the stack resolved to `opencv-python-headless
5.0.0.93` (OpenCV **5.0**) and `numpy 2.4.6`; I functionally exercised `imwrite`/`imread`/`cvtColor`/
`addWeighted`/`connectedComponents`/`skeletonize` and instantiated `nnUNetPredictor` on CUDA — all
fine, but check OpenCV-4-era snippets against 5.0. **TC-003** — `env.sh` exports exactly the names
INTERFACES.md §310-330 asks `check_env.py` to assert.

**Pinned facts for later cards:** Python `3.11.16` · torch `2.14.0+cu126` (CUDA 12.6) · torchvision
`0.29.0+cu126` · nnunetv2 `2.8.1` · numpy `2.4.6` · scikit-image `0.26.0` · Pillow `12.3.0` · cv2
`5.0.0` · PyYAML `6.0.3` · huggingface_hub `1.31.0` · pytest `9.1.1` · pyrealsense2 `2.58.4.10922`.

**Next:** TC-003, TC-004, TC-007, TC-010, TC-012, TC-015 unblocked.

---

## 2026-09-17 · TC-003 — Environment verification script (Level 1) · **APPROVED**

- **Review id:** `TC-003-rev1-c81f17aad71e` · **Reviewer:** opus
- **Implementation run:** `TC-003-r0-ac25430f86e6` (sonnet) · **Commit:** `859859e`
- **Criteria:** 8 / 8 PASS · **Full report:** `.task_orchestrator/reviews/TC-003-rev1-c81f17aad71e/report.md`

Every acceptance command was re-executed independently; the report's pasted outputs match the
filesystem value for value.

**Verified:** commit contains exactly three files — `scripts/check_env.py` (the card's only creation)
plus the two allowed doc/index files. Twenty-six untracked planning paths were present at commit time
and none were swept in, so `git add -A` was provably not used; no co-author trailer; author and
committer `Gavinw575`. All forbidden files (`config.py`, `logging_setup.py`, `project.yaml`, `env.sh`,
`pyproject.toml`, every `docs/**` and `task_cards/TC-0NN-*.md`) carry mtimes predating the 21:33–21:58
run window. All **20 checks** present in the card's prescribed order; via `env.sh` →
`17 PASS · 0 WARN · 3 FAIL`, exit 3. The ROS guard was proven, not assumed: under a forced
`PYTHONPATH=/opt/ros/humble/...python3.10...` outside `env.sh`, `pythonpath_clean` **FAIL**,
`syspath_clean` **FAIL**, `ld_library_path_clean` WARN on the real inherited `/opt/ros` +
`cuda-11.8` entries, exit 3. `--strict` re-verified independently of the agent's method (module
loaded, `CHECK_FUNCS` replaced with one PASS + one synthetic WARN): exit 0 → exit **3**.
Exception-safety **exercised**, not just inspected — an injected `ZeroDivisionError` became
`boom FAIL ZeroDivisionError: …`, the remaining checks still ran and the table still printed.
`check_env_latest.json` **and** its timestamped twin both carry the §0.4 envelope plus `checks`
(20 entries) and `versions`; `--json` still writes the file. `grep "/home/"` → no match. adr/002,
/004, /006, /007, /008, /009, /010 all obeyed — ROS appears only as a *string being searched for*,
`pyrealsense2` is import-probed only, no skeleton traversal, no depth projection, no training.

**System safety:** no `sudo`, no `shell=True`, no `rm -rf`/`rmtree`, no `weights_only`; `nvidia-smi`
invoked read-only via list-form `subprocess.run` with a timeout. `~/.bashrc` unmodified (content and
mtime `2026-06-03`). No other conda env, system CUDA, ROS tree or driver touched. Orchestration gate
intact — `.task_orchestrator/` absent from the commit, all six `cv-*` commands predate the run.
The agent's disclosed throwaway 260 MB model fixture was verified fully reverted: `models/` holds
only TC-001's 0-byte `.gitkeep`, and rows 16/18/19 are honestly still FAIL.

**Non-blocking findings:** (F-1, MINOR) `check_env.py:470-472` — `--dry-run` returns exit **0** even
with FAILs on the board (reproduced); per INTERFACES §0.3's literal wording and outside the card's
`[--json] [--strict]` signature, so not a contract breach, but it makes that invocation useless as a
gate. (F-2) `--dry-run` still writes the `.log` file, inherited from TC-001's `setup_logging`.
(F-3) `--json` stdout carries only `{checks, versions}`; the written file carries strictly more.
(F-4) `_write_summary` (`:411-413`) finds the timestamped twin by globbing and taking the
lexicographically last name, because `RunSummary.write()` returns only the `latest` path — correct in
practice, fragile if the clock moves backwards; the clean fix belongs in `logging_setup.py`, which
this card was rightly forbidden to touch. (F-5) the `len(CHECK_FUNCS) == 20` assert is elided under
`python -O`. (F-6) `nnunet_env_vars` uses a bare `startswith` prefix test — *verbatim as the card
prescribes*, so a sibling `<root>_scratch` would pass; spec issue, not agent error. (F-7)
`project_paths` FAILs with the hint "run TC-001" when the real remedy is TC-004, because
`paths.models = models/opencrack-nnunet` is only created by `fetch_model.py`; the agent implemented
the card's fixed hint literally and flagged the mismatch rather than improvising — correct under
AGENT_INSTRUCTIONS rules 3 and 4.

**Integration risks to watch:** **TC-011** — `run_test.sh` uses `check_env.py` as its Level-1 gate;
it must invoke it *without* `--dry-run` and branch on the exit code, or F-1 lets a broken environment
through green. **TC-016** — read the snapshot from `logs/check_env_latest.json`, not from `--json`
stdout, or lose `tool`/`status`/`exit_code`/`counts`/`errors`. **TC-004** — it is the only card that
can turn rows 16/18/19 green, so it should assert that a clean `check_env.py` run exits 0 afterwards;
until then **exit 3 is the correct expected state** and no card should gate on a green check_env.
**TC-012** — `pyrealsense2 2.58.4` already PASSes; do not assume a RealSense-free env. `logs/` gains
one `.log`+`.json` pair per invocation and is gitignored — worth a note for TC-016.

**Next:** TC-004 unblocked.

---

## 2026-09-17 — TC-004 — OpenCrack model acquisition from Hugging Face — **APPROVED**

**Review:** `TC-004-rev1-873850003d43` (opus) · **Run:** `TC-004-r0-4cb47a6c3f8c` (sonnet) ·
**Commit:** `e118073` *feat: add OpenCrack model acquisition and sha256 manifest* ·
**Full report:** `.task_orchestrator/reviews/TC-004-rev1-873850003d43/report.md`

**Criteria: 9 / 9 PASS**, every one re-run by the reviewer rather than read off the report. The
268 091 674-byte checkpoint is byte-exact to `ARCHITECTURE.md` §4.1; all six manifest entries were
independently re-hashed outside the script and every `sha256` **and** `bytes` field matches
(checkpoint `dcba82012874682c84387c68a2b69f64eddef07e84ae7d813844cdf6396a1a5f`). Re-run is genuinely
idempotent (`real 0m1.611s`, "skipping download", exit 0, vs ~12.4 s for the real fetch);
`--verify-only` exits 0; `check_env.py` is now **20 PASS · 0 WARN · 0 FAIL, exit 0** — `project_paths`
flipped green too, exactly as TC-003's review predicted. Exit-code contract exercised directly:
missing files → 3, malformed config → 2, unknown flag → 2, `--dry-run` → 0, and a deliberately
corrupted `plans.json` → 3 with a sha256 mismatch, which proves the verifier detects drift rather
than merely existing. Scope is exact (4 files, all on the card's list), the commit holds only the two
source files with no `Co-Authored-By` trailer, `models/` is gitignored and uncommitted, and
`.gitignore`/`src/crackvision/**` are byte-identical to baseline. ADRs 001/002/003/004/005/008/009/010
all clear; no `sudo`, no `subprocess`, no `shell=True`, no `weights_only`, `~/.bashrc` mtime still
2026-06-03, no other conda env or system CUDA/ROS path touched, `.task_orchestrator/` and the `cv-*`
commands untouched.

**Non-blocking findings:** (F-1, MINOR) `fetch_model.py:187-201` — `--verify-only` walks disk→manifest
only, never manifest→disk; **reproduced** on a scratch copy, deleting `README.md` and
`reproducibility/splits_final.json` yields "4 files match" and **exit 0**. Bounded (the three required
files *are* existence-checked, corruption *is* caught) but a real hole in RISKS R-06. (F-2, MINOR)
`:248-256` rewrites the tracked `config/model_manifest.json` with a fresh `downloaded_utc` even when
nothing downloaded — semantically wrong field on that path, and every run dirties a committed file
(reproduced; restored). (F-3, MINOR) `:53-55` divides by 1024² while labelling the unit "MB", so
268.09 MB logs as `255.7 MB` and the 250–300 MB band is really 262–315 MB decimal; harmless and
consistent with `check_env.py`, but the report's `ISSUES:` note misreads it as upstream "rounding
variance" when the file is byte-exact. (F-4) `--force` silently ignored alongside `--verify-only`.
(F-5) `--skip-existing` accepted but a no-op, as in TC-003. (F-6, INFO) `_is_hidden` (`:66-67`)
correctly filters `huggingface_hub`'s `.cache/huggingface/**` bookkeeping out of the manifest —
disclosed by the agent, not a silent deviation. (F-7, INFO) the report's `du -sh ~/.cache/huggingface`
→ "17G before and after" cannot resolve 268 MB against 17 GB; the reviewer confirmed the criterion
properly — `hub/models--fadeevla--opencrack-nnunet` is **12 K**, holding only a 40-byte `refs/main`.
Weak evidence for a true claim.

**Integration risks to watch:** **TC-005** — `verify_model.py --check-hashes` (INTERFACES §3.7.3) must
**not** copy F-1's one-directional loop, or a deleted model file passes its gate; and
`models/opencrack-nnunet/` now holds a `.cache/huggingface/` sibling next to `Dataset501_OpenCrack/`,
so any "this directory is an nnU-Net results tree" assertion must tolerate it (the `Dataset501_OpenCrack/`
subtree itself is clean, so the §3.7.2 relative symlink is unaffected — it does **not** exist yet, so
no card may use the `-d 501` fallback until TC-005 lands). **TC-016** — F-2 means the manifest is not a
stable tracked file; anchor reproducibility on `revision`
(`1198179e893f5f6eb0dd3eae2d8de5f1adf85afc`), not `downloaded_utc`, and do not assume a clean
`git status` after a pipeline run. **TC-011** — if `run_test.sh` gates on `fetch_model.py` it will dirty
the manifest every run (F-2); prefer `--verify-only`, weighing F-1. **TC-006 / TC-008** — the
`nnUNetv2_predict_from_modelfolder` model folder (ADR-006) is now materially present, and
`check_env.py` exits 0, superseding TC-003's note that exit 3 was the correct expected state.

**Next:** TC-005 unblocked.

---

## 2026-09-18 — TC-005 — nnU-Net model wiring and semantic validation — **CHANGES_REQUIRED**

**Review:** `TC-005-rev1-76de7a27a2fe` (opus) · **Run:** `TC-005-r0-8fb8bea46eb1` (sonnet, state
recovered by `cv-recover --accept-report`, not a witnessed exit) · **Commit:** `14c05b4`
*feat: add nnU-Net model wiring and semantic validation* ·
**Full report:** `.task_orchestrator/reviews/TC-005-rev1-76de7a27a2fe/report.md` ·
**Issues:** `.task_orchestrator/reviews/TC-005-rev1-76de7a27a2fe/issues.json`

**Criteria: 7 / 10 PASS.** The verdict is about the repository's state, **not the agent's work** —
`scripts/verify_model.py` is correct and needs **no code change**. The next session must not treat
this as a code review of that file.

**The blocker, and who caused it.** `models/…/nnUNetTrainer__nnUNetPlans__2d/plans.json` is corrupt:
6362 bytes vs the manifest's 6361, hex tail `7d 0a 7d 58` (`}\n}X`), one stray `X` appended. So
`plans_json_parse` FAILs with `JSONDecodeError: Extra data: line 218 column 2 (char 6361)`, the five
`plans_*` assertions all read `not evaluated (parse failed)`, and `--check-hashes` exits 1 on
`manifest=08f03a8e…, actual=68f97544…`. **This is not upstream drift and R-06 is not triggered:**
truncating a real copy to 6361 bytes reproduces the manifest sha256 `08f03a8e…` exactly, so the
upstream content is intact. **The mechanism is a hardlink and it was found, not inferred** —
`find -samefile` still shows the project file sharing one inode with
`…/scratchpad/**c366e593-…**/fakeroot/models/…/plans.json`, TC-004's *reviewer* scratch tree, built
with a hardlinking copy. That review's own "deliberately corrupted `plans.json` → 3 with a sha256
mismatch" test therefore wrote **through the link into the real model file** at 05:39:01Z and was
never reverted. The TC-005 run started 05:41:16Z — **2 min 15 s later**. The agent is exonerated.

**Why not APPROVED.** Three criteria genuinely do not hold on the delivered tree (the five
`plans_*` assertions, `--check-hashes`, and idempotent exit 0 — the symlink half of idempotency does
hold: inode `2360123` and ctime unchanged across runs). APPROVED would flip TC-005 to COMPLETE and
unblock **TC-006**, whose smoke test runs `nnUNetv2_predict_from_modelfolder` against this exact
model folder and would crash inside nnU-Net's own `plans.json` parse. The gate stays shut until the
tree is repaired and the nine assertions pass for real.

**Verified good — the whole rest of the card.** On an isolated **real-copy** tree (`find -samefile`
→ 0 shared inodes) with the stray byte removed, the card's own commands give
**14 PASS · 1 WARN · 0 FAIL, exit 0**, including all five `plans_*` values (`'2d'`, `[256, 256]`,
3 × `ZScoreNormalization`, `"nnUNetPlans"`, `"Dataset501_OpenCrack"`) and `check_hashes PASS 6 files
match`. **Contract values confirmed for later cards:** `channel_names={"0":"R","1":"G","2":"B"}`,
`labels={"background":0,"crack":1}`, `file_ending=".png"`,
`overwrite_image_reader_writer="NaturalImage2DIO"`, `patch_size=[256,256]`. All four exit codes
exercised directly (0/1/2/3). `weights_only=False` appears **nowhere**; the real checkpoint raises
`UnpicklingError: Unsupported global: GLOBAL numpy._core.multiarray.scalar` and correctly falls back
to the `PK\x03\x04` magic-byte WARN. Symlink is relative (`../../models/opencrack-nnunet/
Dataset501_OpenCrack`) and resolves; all three edge cases re-exercised by the reviewer — a broken
link is refused without `--repair-symlink` and left untouched, repaired with it, and a **real
directory survives with its `CANARY` file intact**. Scope is exact (3 files, all on the card's list),
26 untracked planning paths present at commit time and none swept in, no co-author trailer, author
`Gavinw575`. `src/`, `config/`, `env.sh`, `scripts/check_env.py`, `scripts/fetch_model.py` are
byte-identical to baseline with mtimes predating the run window; `find models -newer
config/model_manifest.json` empty. ADRs 002/004/005/006/008/009/010 clear — no skeleton traversal, no
depth→3D, no ROS, no training. No `sudo`, `subprocess`, `shell=True` or `rm -rf`; the only `unlink`
is reachable only for a broken symlink under `--repair-symlink`. `~/.bashrc` mtime still
`2026-06-03`. Orchestration gate **intact** — `.task_orchestrator/` absent from the commit, all six
`cv-*` commands and `orchestrator.py` predate the run by hours. Every claim in the agent's
`COMPLETION_LOG.md` entry checked out against the filesystem; it reported BLOCKED honestly rather
than editing the model to go green, which is precisely AGENT_INSTRUCTIONS rules 3/5/9/11.

**Credit where due:** `_check_hashes` (`:273-283`) walks **manifest → disk** and flags a missing file
as a mismatch — a direct fix for TC-004 review **F-1**, the exact integration risk that review raised
against this card. The agent read the prior review and acted on it.

**Non-blocking findings:** (F-1, MINOR) `:404-413` a hash mismatch masks core precondition failures —
exit 1 wins over 3 when both are present; defensible under the card's literal exit-1 rule.
(F-2, MINOR) `:415` `--dry-run` exits 0 with FAILs on the board, same as `check_env.py` F-1.
(F-3, MINOR) `:184-193` the `weights_only=True` **success** branch is unreachable on this checkpoint,
so its "log keys + tensor count" code has never executed. (F-4, MINOR) `:159` the broad
`except Exception` means a truncated-but-`PK`-prefixed checkpoint would WARN rather than FAIL;
bounded by `--check-hashes`. (F-5, MINOR) `:323-329` timestamped-twin discovery by lexicographic
glob, inherited from `check_env.py` (TC-003 F-4) because `logging_setup.py` is not this card's to
change. (F-6/F-7, INFO) the real-file branch and the absolute-symlink rejection are both correct.

**Integration risks to watch:** **TC-006** and **TC-008** both load this model folder and both break
until `plans.json` is clean. **TC-011** — `run_test.sh` must call `verify_model.py` *without*
`--dry-run` (F-2) and treat **both 1 and 3** as failure, not just 3 (F-1). **TC-016** — read
`logs/verify_model_latest.json`, not stdout. **TC-014** — `--check-hashes` only becomes a usable
regression gate once the tree is repaired.

**Process finding for the orchestrator (chargeable to no card):** review scratch trees must be built
with **real copies** (`cp -r --no-preserve=links`), never a hardlinking `cp -al`. One hardlinked
fixture wrote into the live model tree and has now cost a full card cycle. This audit's own scratch
tree was verified hardlink-free (`find -samefile` → 0 shared inodes) before anything was written to it.

**Next:** TC-005 revision 1 — repair the model tree via `./env.sh python scripts/fetch_model.py
--force` (authorised for that revision as a scoped exception), then re-verify. TC-006 stays blocked.

---

## 2026-09-19 · TC-005 (revision 1) — nnU-Net model wiring and semantic validation · **APPROVED**

**Review:** `TC-005-rev2-afe6e985afe5` (opus) · **Run:** `TC-005-r1-6bb1438f24a4` (sonnet) ·
**Baseline** `14c05b4` → **commit** `4424e0e` · **Criteria 10/10 PASS.** Exit was unwitnessed
(watchdog-recorded); every claim was re-derived from the filesystem and git, not from the report.

**Both prior BLOCKERs fixed, both MINORs addressed.** (1) `plans.json` is now **6361 bytes**,
sha256 `08f03a8e…dc01eeb` — byte-for-byte the manifest value — and parses; hex tail is `7d 0a 7d`,
the stray `0x58` is gone. Repaired with `fetch_model.py --force`, **not hand-edited**: `revision`
stayed `1198179e…85afc` and all six `bytes`/`sha256` values are unchanged, so no upstream drift —
exactly the stop-condition the rev-1 review set. Only `downloaded_utc` moved. (2) Re-run by this
auditor: `verify_model.py` → `13 PASS · 1 WARN · 0 FAIL`, `exit=0`; `--check-hashes` →
`14 PASS · 1 WARN · 0 FAIL`, `exit=0`, `check_hashes PASS 6 files match …`; symlink inode+ctime
(`20879111 1789624174`) identical across runs; `find models -newer config/model_manifest.json`
empty; `logs/verify_model_latest.json` → `"exit_code": 0`. (3) and (4) are recorded in the
completion report's ISSUES as their acceptance conditions required, naming **TC-011** for the
`--dry-run` hazard.

**No test was weakened to go green.** `scripts/verify_model.py` is byte-identical to `14c05b4`
(`sha256 a6fe6f66…86a0f` both sides). The card passes because the data was repaired.

**Scope exact.** Only `docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md` (both on the card's
allowed list) and `config/model_manifest.json` changed in the run window; the commit carries the
first two only. 26 untracked planning paths present and none swept in — no `git add -A`, no
co-author trailer, author `Gavinw575`. The manifest and `models/**` writes were pre-authorised in
writing by the rev-1 review as a revision-only exception, and the manifest was correctly left
uncommitted. ADRs 002/004/005/006/008/009/010 clear — relative symlink verified, `-d 501` glob
discoverable, no skeleton traversal, no depth→3D, no ROS, no training. No `sudo`, `shell=True`,
`subprocess` or `rm -rf`; `weights_only=False` appears nowhere. `~/.bashrc` still `2026-06-03`;
`/opt/ros` and `/usr/local/cuda*` untouched. **Orchestration gate intact** — `.task_orchestrator/`
never committed, `cards.json` unchanged since Sep 16, the 19:30 `state.json`/`history.log` writes
are the watchdog's own `SESSION_STOPPED_BY_WATCHDOG` entry, all six `cv-*` commands predate the run.

**Credit:** the agent flagged that the rev-1 review's own acceptance text was off by one (it said a
"14-entry `checks` array"; the array correctly has **15**, one per printed row) instead of silently
conforming to the reviewer's wrong number.

**Live hazard, chargeable to no card:** three model files are *still* hardlinked into TC-004's
review scratch tree — `dataset.json` (inode `20879027`), `dataset_fingerprint.json` (`20879030`)
and the 268 MB `checkpoint_ep0500.pth` (`20879022`) all show `nlink=2`, and the 263 MB
`…/c366e593-…/scratchpad/fakeroot` still exists. Only `plans.json` was un-linked, as a side effect
of the re-download. Any write into that tree corrupts the live model again. Recommended before
TC-006: `rm -rf` that fakeroot, and build every future review fixture with
`cp -r --no-preserve=links`, never `cp -al`.

**Other integration risks:** `config/model_manifest.json` is left dirty (`downloaded_utc` only —
harmless, both versions pass `--check-hashes`; TC-016 should decide whether that field belongs in a
tracked file). **TC-011** must call `verify_model.py` without `--dry-run` and treat **both 1 and 3**
as failure. **TC-006/TC-008** are unblocked and should run `--check-hashes` first as the cheapest
detector for a hardlink recurrence. **TC-014** can now use `--check-hashes` as a regression gate.

**Contract confirmed against the live model:** `channel_names={"0":"R","1":"G","2":"B"}`,
`labels={"background":0,"crack":1}`, `file_ending=".png"`,
`overwrite_image_reader_writer="NaturalImage2DIO"`, `patch_size=[256,256]`, 3 ×
`ZScoreNormalization`, `plans_name="nnUNetPlans"`, `dataset_name="Dataset501_OpenCrack"`,
`configurations=["2d"]`.

**Next:** TC-005 COMPLETE. TC-006 (synthetic end-to-end smoke test, Level 2) is READY.

---

## 2026-09-19 · TC-006 — Synthetic end-to-end smoke test (Level 2) · **CHANGES_REQUIRED**

**Review:** `TC-006-rev1-8d7946ef31e0` (opus) · **Run:** `TC-006-r0-0fa83c7e8f96` (sonnet) ·
**Baseline** `4424e0e` → **commit** `6edcbfd` · **Criteria 10/10 PASS.** Exit was unwitnessed
(watchdog-recorded); every claim was re-derived from the filesystem and git, and every acceptance
command was re-executed by this auditor rather than read from the report.
**Full report:** `.task_orchestrator/reviews/TC-006-rev1-8d7946ef31e0/report.md` ·
**Issues:** `.../issues.json` (1 BLOCKER, 1 MAJOR, 4 MINOR).

**All ten acceptance checkboxes genuinely hold — reproduced, not taken on trust.** `--device cuda`
→ `SMOKE TEST PASSED`, exit 0, 15.0 s wall; `--device cpu` → PASSED, exit 0, 19.2 s. Predictions:
`synthetic_crack dtype uint8 shape (512,512) unique [0 1]`, `synthetic_blank … unique [0]` — the
`{0,1}` convention (R-03) is pinned by a real assertion. `pred.shape == input.shape[:2]` → True for
both, so the frame invariant (R-14, `adr/002`) holds with no resize/crop/pad anywhere. The command
line carries **`-f 0`** and **`-chk checkpoint_ep0500.pth`**, built as a list with no `shell=True`.
Mask/overlay/skeleton all 512×512; `595 ≤ 3112` and `0 ≤ 0`. No assertion touches IoU, Dice, recall
or crack-pixels>0 — the counts are logged only, exactly as the card's most important constraint
demands. VRAM `178 MiB` before and after; no GPU process killed. Fixtures byte-identical across two
runs. `verify_model.py --check-hashes` after two real inference passes → `14 PASS · 1 WARN · 0 FAIL`.

**Sent back for two reproducible contract violations outside the checkbox list, neither disclosed.**
(1) **BLOCKER** — the missing-model precondition never yields exit 3. Only the CUDA branch returns
`EXIT_PRECONDITION`; with no model the nnU-Net `FileNotFoundError` propagates as rc=1 → exit **1**
plus a 20-line traceback. Reproduced against a scratch root. Violates `INTERFACES.md` §0.2 ("3 |
precondition not met | **model folder missing**"), the card's §5 Output, the card's Failure-handling
row ("Model missing → exit 3 pointing at TC-004/TC-005"), `AGENT_INSTRUCTIONS.md` § Exit codes, and
the stated consequence of `adr/006` ("an existence check gives a precise error **before anything is
launched**"). (2) **MAJOR** — `--dry-run --clean` deletes
`data/smoke_test/{input,nnunet_input,predictions,overlays,skeletons}` and then reports success, because
the `shutil.rmtree` loop (`smoke_test.py:292`) precedes the dry-run guard (`:300`). Reproduced.
Violates §0.3 ("write nothing, exit 0"). Four MINORs: blank fixture does not share the crack
fixture's noise (card §1 says it must); assertions run *after* rendering so a shape mismatch crashes
instead of failing cleanly; `--clean` misses `ext_trainer/` and its stale `__pycache__`; hardcoded
assertion count and an undisclosed `--skip-existing` no-op.

**Scope exact, gate intact.** Commit = exactly `scripts/smoke_test.py`, `docs/COMPLETION_LOG.md`,
`task_cards/TASK_INDEX.md` — all on the card's own lists. No `git add -A`, no co-author trailer,
author `Gavinw575`. `config/model_manifest.json` appears in the run diff but its only change
(`downloaded_utc → 2026-09-19T01:26:14Z`) was written **16 minutes before the run began** and was
correctly left uncommitted. `models/**` untouched and hash-clean; the six sibling `data/` pipeline
dirs still hold only their TC-001 `.gitkeep`. No `sudo`, `rm -rf` on a variable, `os.system`,
`weights_only=False` or literal `/home/`. `~/.bashrc`, `/opt/ros`, `/usr/local/cuda*`, the driver and
every other conda env untouched. `.task_orchestrator/` never committed; all six `cv-*` commands still
carry Sep 16 mtimes. No test weakened or skipped.

**The `nnUNet_extTrainer` shim is accepted — verified independently, not taken on trust.** The
checkpoint's `trainer_name` really is `nnUNetTrainerSaveEvery10`, a class the OpenCrack authors never
published; `nnUNet_extTrainer` really is upstream's own documented extension point
(`nnunetv2/paths.py:74`, `utilities/find_objects.py:24-52`, with upstream tests). The `pass` subclass
is architecturally inert — `predict_from_raw_data.py:99-124` builds the network from `plans_manager`,
and the strict `network.load_state_dict()` at `:131` succeeded on both devices, which is hard proof of
architecture match rather than a silent fallback. The shim is written only under
`data/smoke_test/ext_trainer/` (card-allowed), never into `models/**`, and invokes no training
(`adr/010` intact). `AGENT_INSTRUCTIONS.md` expressly sanctions adapting to current upstream API and
recording it under `ISSUES:` — the agent did exactly that, in verifiable detail. Credit also for
declining to "fix" the scikit-image `min_size` deprecation that belongs to TC-010, and for declining
to `rm -rf` a hazard outside its scope. ADRs 002/006/008/009/010 all clear.

**Integration risks.** **TC-008** is the sharp one: its card tells it to refactor this module's
subprocess logic into `crackvision.inference`, so both the exit-3 gap and the destructive dry-run
would propagate into the production path — cheaper to fix here. TC-008 must also carry
`nnUNet_extTrainer`, or inference fails with `RuntimeError: Could not find requested nnunet trainer
nnUNetTrainerSaveEvery10`; **TC-016** should put that fact in the README, since a fresh clone hits it
on first inference. **TC-010** must resolve the `remove_small_objects(min_size=…)` deprecation in
`INTERFACES.md` §3.4. **TC-014** will fail the §0.2 exit-code contract as things stand. **TC-011**
must not combine `--dry-run` with clean/reset behaviour until fixed.

**Live hazard, chargeable to no card — still not cleared.** `dataset.json` (inode 20879027),
`dataset_fingerprint.json` (20879030) and `checkpoint_ep0500.pth` (20879022) all still show
`nlink=2` into TC-004's review scratch tree; only `plans.json` is clean. TC-006 only ever read
`models/**` and did not touch it. A human should still clear that fakeroot, and future review
fixtures must use `cp -r --no-preserve=links`, never `cp -al`.

**Next:** TC-006 returns for revision 1. The rework is narrow — one precondition check, one statement
reordering and four small cleanups, all inside `scripts/smoke_test.py`. Nothing upstream needs
re-running and no design decision is reopened.

---

## 2026-09-19 · TC-006 (revision 1) — Synthetic end-to-end smoke test (Level 2) · **APPROVED**

**Reviewer:** opus · **Review id:** `TC-006-rev2-f8a0efcb0b57` · **Run:** `TC-006-r1-47cb7c0542f2`
(sonnet) · **Commit:** `7d6183c` · **Baseline:** `6edcbfd` · **Criteria: 10 / 10 PASS** ·
**Prior findings: 6 / 6 fixed.** Exit unwitnessed (watchdog transition); every claim re-derived from
the filesystem and `git`, nothing accepted on the report's word.

**Both gating rev-1 findings are closed, verified by re-execution, not by reading the diff.**
[BLOCKER] `check_model_folder()` (`scripts/smoke_test.py:134-147`) now runs at `:406` — after the
CUDA check, before fixture generation and before the subprocess. Against a scratch root holding only
`config/project.yaml` the script exits **3**, names all three missing paths, points at
`fetch_model.py` (TC-004) then `verify_model.py` (TC-005), emits no traceback, and writes
`"status": "precondition", "exit_code": 3`. `INTERFACES` §0.2 and `adr/006`'s "existence check
before anything is launched" are satisfied. [MAJOR] The `--dry-run` guard now precedes the `--clean`
`rmtree` (`:384-395`): with sentinels planted in `input/` and `ext_trainer/`, `--dry-run --clean`
logged what it *would* remove, exited 0 and deleted **nothing**; a real `--clean` then removed both
sentinels and the stale `__pycache__` and still passed. `INTERFACES` §0.3 honoured.

**The four MINORs likewise verified.** The fixtures are now a genuine matched pair — the auditor's
own measurement shows `synthetic_crack` outside the polyline is **bit-for-bit identical** to
`GaussianBlur(0.6)` of `synthetic_blank` (residual max 0), so the whole background difference is the
crack image's own blur on the *same* noise draw; md5s reproduce across runs. `evaluate_case()`
asserts dtype / `unique ⊆ {0,1}` / shape-vs-the-real-fixture *before* rendering — all four injected
malformations (256×256, all-255, uint16, absent) returned clean failure strings with `result=None`
and no traceback. `--clean` now covers `ext_trainer/`. The banner count is computed from a live
per-case counter and correctly reads **7**, not the old hardcoded 6; `--skip-existing` is disclosed
under `ISSUES:` as an accepted no-op, per the precedent the rev-1 review itself named.

**All ten criteria re-run by the auditor.** `--device cuda` → `SMOKE TEST PASSED (7 assertions x 2
images, 12.3 s)`, exit 0; `--device cpu` → same banner, 18.6 s, exit 0. Independent reads off disk:
`synthetic_crack uint8 (512,512) [0 1]`, `synthetic_blank uint8 (512,512) [0]` — R-03 pinned, R-14
frame invariant held and asserted against the fixture's real dimensions rather than a literal. Mask,
overlay and skeleton all 512×512; `657 ≤ 4577` and `0 ≤ 0`. Command line carried `-f 0 -chk
checkpoint_ep0500.pth` exactly (R-05), built as a list, `shell=False`. `synthetic_blank` yielded zero
crack pixels and still passed — the card's "all-zero is acceptable" branch demonstrably works, and
`grep -inE "iou|dice|recall"` hits only the docstring saying these are never asserted (`adr/001`).
`--device banana` → 2. VRAM `178 MiB` before and after both runs; no GPU process killed.
`verify_model.py --check-hashes` after all inference → `14 PASS · 1 WARN · 0 FAIL`, exit 0.

**Scope and safety clean.** `git show --stat HEAD` touches exactly `scripts/smoke_test.py` and
`docs/COMPLETION_LOG.md` — both on the card's lists. No `git add -A`, no co-author trailer, author
`Gavinw575`. `config/model_manifest.json` appears in the run diff only as pre-existing dirt
(`downloaded_utc` from TC-005's repair, written before this run) and was correctly left uncommitted.
`models/**`, `src/crackvision/**`, `scripts/check_env.py`, `scripts/verify_model.py` and the six
sibling `data/` pipeline dirs are all untouched — the latter still hold only their TC-001 `.gitkeep`.
Nothing outside `data/smoke_test/` and `logs/`. No `sudo`, `shell=True`, `os.system`, `/home/`
literal, `weights_only=False` or `rm -rf` on a variable; `~/.bashrc`, `/opt/ros`, `/usr/local/cuda*`,
the driver and other conda envs untouched; `.task_orchestrator/` and the `cv-*` commands unmodified.
ADRs 001/002/006/008/009/010 all clear. The `nnUNet_extTrainer` shim was **re-verified from source**
rather than carried over: `nnUNetTrainerSaveEvery10` is genuinely unshipped, and `nnUNet_extTrainer`
is genuinely upstream's own extension point (`nnunetv2/paths.py:74`,
`utilities/find_objects.py:24-51`, with an upstream test). Credit for declining to fix TC-010's
scikit-image deprecation and for leaving `models/**` strictly read-only.

**Integration risks.** **TC-008** is the sharp one: it refactors this subprocess logic into
`crackvision.inference` and must carry `nnUNet_extTrainer` across, or inference dies with
`Could not find requested nnunet trainer nnUNetTrainerSaveEvery10`; the shim presently lives only in
`smoke_test.py`, writing into a gitignored dir, and is documented in neither `ARCHITECTURE.md` nor
`INTERFACES.md` — TC-006's file scope did not permit adding it there. **TC-016** should put that in
the README. **TC-010** must resolve the `remove_small_objects(min_size=…)` deprecation and decide
whether the `min_size=64` pre-skeleton filter is production semantics. **TC-014** can now rely on the
§0.2 exit-code contract. **TC-011** may safely compose `--dry-run` with `--clean`.

**Live hazard, chargeable to no card — still not cleared.** `dataset.json` (inode 20879027),
`dataset_fingerprint.json` (20879030) and `checkpoint_ep0500.pth` (20879022) still show `nlink=2`
into TC-004's review scratch tree; only `plans.json` (20879174) is clean. TC-006 read `models/**`
only. A human should delete that scratch tree; future review fixtures must use
`cp -r --no-preserve=links`, never `cp -al`.

**Next:** TC-007 (input preparation + naming contract).

---

## 2026-09-19 · TC-007 — Input preparation utility and the naming contract · **APPROVED**

**Reviewer:** opus (`TC-007-rev1-e769d682fde1`) · **Run:** `TC-007-r0-c2554d585317` (sonnet) ·
**Commit:** `845227b` · **Baseline:** `7d6183c` · **Criteria:** 13 / 13 PASS.

This run's exit was unwitnessed — the watchdog transitioned it on its own evidence — so every claim
was re-verified against the filesystem and a live re-run of the card's own commands. **No
discrepancy between the completion report and the filesystem was found**: the test count, flag list,
exit codes, case_map contents, idempotency hashes and the agent's own cleanup all reproduce exactly.

**All thirteen criteria re-run by the auditor.** `./env.sh pytest tests/ -v` → **37 passed in
0.32s**, every test name matching the report. `--help` prints exactly the §3.1 signature plus all
six §0.3 common flags. Empty `data/input_originals/` → `exit=3` with a message naming both the
directory and all six accepted extensions; missing directory likewise. End-to-end on TC-006's two
synthetic fixtures → `exit=0`, `all RGB PNG OK`, `counts:{"found":2,"converted":2}`, `status:"ok"`,
and a `case_map.json` I validated key-by-key against §2 — sorted by `case_id`, project-relative
POSIX paths, both sha256s independently confirmed with `sha256sum`. Re-run produced byte-identical
output hashes (idempotent). `data/input_originals/` content hash `7e1555c5…c239` identical before
and after, including across a `--max-side` run. All seven Pillow modes (RGB JPEG, RGBA, `L`, `P`,
CMYK TIFF, `I;16`, BMP) come out `RGB`/PNG/uint8/3-channel; the orientation-6 JPEG comes out upright
with `(100,60) → (60,100)`. A zero-byte plus a half-truncated PNG alongside one good file →
`failed:2, converted:1, status:"partial"`, exit 1, good case still mapped. `--dry-run` writes zero
files into `data/`. No `/home/` literal.

**The one deviation is correct and was declared three times over.** Two cells of the `INTERFACES.md`
§1.1 worked-example table contradict themselves: `A.png`/`a!.png` prints "`A`, then `a__2`" while its
own parenthetical says "distinct stems → no collision", and `x y`/`x-y`/`x_y` says "no collision"
while the normative rule maps both a space and an underscore to `_`. I re-derived both by hand — the
normative algorithm (stated identically in §1.1, `ARCHITECTURE.md:307-312` and the card body) has no
case-folding step, so the agent's reading is the only one consistent with the rule. It implemented
the algorithm, recorded the contradiction under `ISSUES:`, in `test_naming.py`'s docstring and at
each assertion, and correctly did **not** edit the planning-owned `docs/INTERFACES.md`. Criterion 1
scored PASS on that basis. **A human should repair those two cells**, which are now contradicted by
a green suite.

**Scope and safety clean.** `git show --stat HEAD` touches exactly the card's five *Files to create*
plus its two *allowed to modify* — nothing else. `data/input_originals/`, `config.py`,
`logging_setup.py`, `scripts/**` and `models/**` unchanged since the baseline. No `git add -A`, no
co-author trailer, author `Gavinw575`. `config/model_manifest.json` appears in the run diff only as
pre-existing dirt (mtime 80 minutes before the run began) and was correctly left uncommitted. No
`sudo`, `shell=True`, `subprocess`, `os.system`, `pkill`, `rm -rf` or `weights_only=False`; no
OpenCV. `~/.bashrc`, `/opt/ros`, `/usr/local/cuda*`, the driver and all six sibling conda envs carry
mtimes predating the run. `.task_orchestrator/**` and the `cv-*` commands unmodified. ADRs
001/002/008/009/010 all clear — no skeleton traversal, no depth→3D, no ROS, no training. Credit for
the `img.load()` placement (forces a truncation error inside the per-file `try`), for the
`exif_transpose(...) or img` guard, and for `--max-side <= 0` → exit 2.

**Integration risks.** **TC-009** is the sharp one: it loads the original via `source_path` and makes
a shape mismatch a hard per-case failure, but an EXIF-rotated or `--max-side` source on disk no
longer matches the prepared frame — TC-009 must `exif_transpose` the original itself or compare
against `case_map.json`'s `width`/`height` (which TC-007 correctly records post-transpose).
**TC-010** carries the same exposure. **Measured, and flagged by no card:** `convert("RGB")` from
`I;16` *clips* rather than rescales — `[0, 1000, 30000, 65535] → [0, 255, 255, 255]` — so a real
16-bit TIFF arrives at nnU-Net almost entirely white. The card mandates that exact call and only
requires 8-bit output, so the criterion passes, but **TC-016** should document it and **TC-015**
should know before feeding real 16-bit imagery through. **TC-011/TC-014** must read `counts`
defensively: zero-valued keys are omitted (`{"found":2,"converted":2}`), matching the `check_env`
and `smoke_test` precedent. **TC-011** should also note that `--clean` deletes `*.png` in whatever
`--output-dir` names, unguarded. `naming.py`'s eight path helpers are config-independent literals by
design — TC-009/TC-010 must treat their own CLI flag, not `naming.*_path()`, as the runtime source
of truth.

**Next:** TC-008 (batch inference runner) and TC-009 (visualization), both now unblocked.

---

## 2026-09-19 · TC-008 — Batch inference runner · **CHANGES_REQUIRED**

**Reviewer:** opus (`TC-008-rev1-1bcf09a35ee4`) · **Run:** `TC-008-r0-23c1b2473efc` (sonnet) ·
**Baseline:** `845227b` · **Commit:** `acf8767` (4 files: `src/crackvision/inference.py` new,
`scripts/smoke_test.py`, `docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`) ·
**Full report:** `.task_orchestrator/reviews/TC-008-rev1-1bcf09a35ee4/report.md` ·
**Issues:** `…/issues.json` (1 MAJOR, 1 MINOR)

**All 13 acceptance criteria re-run by the auditor and all PASS.** `--dry-run -v` carries both
`-f 0` and `-chk checkpoint_ep0500.pth`; all five preconditions exit **3** with their exact
actionable messages (empty input and missing-model against a scratch `--root` for real; the
`cuda`-unavailable and not-on-PATH pair by patching `torch.cuda.is_available`/`shutil.which`); real
CUDA run → `2 image(s) in 11.81s (5.903s/image)`, exit 0; `--device cpu` → `18.08s (9.040s/image)`,
exit 0; `--use-dataset-id` → `nnUNetv2_predict -d 501 -c 2d -f 0 -chk checkpoint_ep0500.pth`, exit 0,
**proving TC-005's `nnunet/results` symlink**. Predictions are `uint8` with `unique ⊆ {0,1}` and the
**frame invariant holds on a non-square case too** (640×480 in → `(480, 640)` out). The `{0,1}`
advisory appears exactly once per run. The OOM ladder and the `partial`/exit-1 path were both forced
via a patched `subprocess.run` and behave exactly as specified — no retry, no process kill. The
refactored `scripts/smoke_test.py` calls `run_inference()` and still prints
`SMOKE TEST PASSED (7 assertions x 2 images, 12.3 s)` with `evaluate_case()` byte-unchanged; `pytest`
→ 37 passed; `verify_model.py --check-hashes` → `6 files match` after every run; GPU left at 178 MiB
with no compute processes. Scope, ADRs (002/006/008/009/010), exit-code contract, `pathlib` use and
the no-post-processing rule are all clean; no co-author trailer; `.task_orchestrator/` and the `cv-*`
commands untouched.

**Blocking (MAJOR), `src/crackvision/inference.py:255`:** the card requires *"Stream the subprocess's
stdout/stderr into the logger line by line — do **not** capture silently."* The implementation uses
`subprocess.run(..., capture_output=True)` and replays the text only after the process exits —
measurably so: every `nnunet:` line in a real run's log carries one identical timestamp. The
completion report's claim that the module "streams … line by line" is contradicted by the source.
The code was inherited verbatim from TC-006's approved smoke test, whose card used the weaker
wording; TC-008 sharpened it deliberately, and its own failure handling (*"Subprocess hangs > 30 min
… likely `-npp`/`-nps` worker deadlock"*) is undiagnosable without live output. Fix is narrow:
`Popen(stdout=PIPE, stderr=STDOUT)` + iterate, still accumulating the combined text for the OOM regex.

**MINOR:** `logs/inference_latest.json` carries `duration_s` = RunSummary's tool wall time (13.191 s)
next to a `seconds_per_image` derived from the 11.81 s subprocess, and `images` only as
`counts.images` — the three numbers cannot be reconciled and `summary["images"]` raises `KeyError`.

**Integration risks to watch:** **TC-011** — the one-command runner will show a frozen terminal for
the whole inference stage until Finding 1 is fixed. **TC-014 / TC-015** — read the summary JSON
defensively (`counts.images`, not `images`) and do not assume `duration_s ≈ images × seconds_per_image`.
**TC-016** — the `nnUNet_extTrainer` shim is *still* undocumented in `ARCHITECTURE.md`/`INTERFACES.md`
and has now moved from `data/smoke_test/ext_trainer/` (TC-006) to `logs/ext_trainer/`, a disposable
gitignored directory regenerated per run; `scripts/smoke_test.py:276-277`'s `ext_trainer_dir` is now
vestigial (correctly left alone — outside this card's one authorised edit).

**Next:** TC-008 returns for revision 1. The rework is one function body plus one summary-JSON merge;
nothing else in the card needs revisiting.

---

## 2026-09-19 · TC-008 (revision 1) — Batch inference runner · **APPROVED**

**Reviewer:** opus (independent) · **Review id:** `TC-008-rev2-148b743f74b8` ·
**Run:** `TC-008-r1-4465a711e62c` (sonnet) · **Commit:** `a2b944f` on baseline `acf8767`
**Full audit:** `.task_orchestrator/reviews/TC-008-rev2-148b743f74b8/report.md`

The run's exit was not witnessed — the harness watchdog moved the gate on its own evidence. Every
claim was therefore re-verified from scratch: I generated my own synthetic inputs (512×512 PNG and
480×640 JPEG), re-ran the whole pipeline, and exercised each branch myself. Nothing below rests on
the agent's pasted output.

**Both rev-0 findings are genuinely fixed.**

*Finding 1 (MAJOR — captured instead of streamed).* `inference.py:254-266` now uses
`Popen(stdout=PIPE, stderr=STDOUT, text=True, bufsize=1)` and logs each line as it arrives, still
accumulating the combined text for the unchanged OOM regex. My run: **21 distinct `nnunet:`
timestamps** (required > 1), first at `19:38:09,886` against `inference complete:` at `19:38:17,462`
— **7.58 s apart** (required ≥ 1 s). tqdm's `\r` progress updates are split by universal-newline
translation, so per-tile progress is genuinely live. The OOM branch still fires under the new path:
with `Popen` selectively patched to yield `CUDA out of memory` and `returncode=1`, `run_inference()`
returned `status='failed'`, `exit_code=1` and logged all five ladder steps plus the
"Do NOT kill their processes" line.

*Finding 2 (MINOR — unreconcilable summary JSON).* `main()` now also merges `images` and
`inference_duration_s`. The issue's exact check passes: `2 11.9806 5.9903 13.353`, no `KeyError`,
and `abs(images × seconds_per_image − inference_duration_s) = 0.0`. `RunSummary` and
`logging_setup.py` were correctly left alone.

The agent also **retracted its earlier inaccurate "streams … line by line" claim** in a new
append-only completion-log entry rather than quietly editing the old one. That is the right way to
handle a false claim, and it counted in its favour.

**All 13 acceptance criteria PASS under independent re-execution.** Highlights: `--dry-run -v`
carries both `-f 0` and `-chk checkpoint_ep0500.pth`; predictions are `uint8` with `unique ⊆ {0,1}`
and shapes identical to their inputs for both the square and non-square case (frame invariant held);
`--device cpu` → exit 0 (19.32 s, 9.660 s/image); `--use-dataset-id` → exit 0, proving TC-005's
symlink; `smoke_test.py` still passes with `evaluate_case()` **byte-identical** to its pre-TC-008
form; `pytest tests/ -q` → 37 passed. I exercised **all five** preconditions for real (the card
required two) — every one exits **3** with its actionable message — plus the `"partial"` path, which
returns `exit_code=1` and names each missing case rather than succeeding silently.

**Scope, ADRs, safety:** the commit touches exactly `src/crackvision/inference.py` and
`docs/COMPLETION_LOG.md`; no forbidden file, no `git add -A`, **no Claude co-author trailer**.
adr/002/006/008/009/010 all honoured; no post-processing of prediction pixels; no hard-coded
`/home/`; no `shell=True`. Model tree verified intact (`14 PASS · 1 WARN · 0 FAIL`, hashes match),
GPU back at its **178 MiB** idle baseline with no compute processes after four real passes, and the
orchestration gate untouched.

**Integration risks carried forward (documentation, not defects):** **TC-014 / TC-015** must read
`inference_duration_s`, not `duration_s`, for inference timing — the file now legitimately carries
both, but `INTERFACES.md` §3.2 line 229 still names only `duration_s`. **TC-014** should also not
assume `logs/` holds only `*.log`/`*.json`: `logs/ext_trainer/` is a live Python package directory
(plus `__pycache__/`) regenerated on every run. **TC-016** should document both, along with the
`nnUNet_extTrainer` shim itself, still absent from `ARCHITECTURE.md`/`INTERFACES.md`.

**Next:** TC-008 closes. TC-009 (visualization) and TC-010 (skeletonization) are unblocked.

---

## 2026-09-19 · TC-009 — Visualization pipeline · **APPROVED**

**Review** `TC-009-rev1-da025cee09d2` (opus) · **Run** `TC-009-r0-0b57b0cabd63` (sonnet) ·
**Commit** `6302bfc feat: add mask, overlay and comparison visualization` · baseline `a2b944f`.
This run's exit was **not witnessed** (watchdog transition), so every claim was re-derived from the
filesystem and from commands re-run in an isolated scratch root — nothing taken on trust.

**All 13 acceptance criteria PASS under independent re-execution.** I rebuilt a throwaway project
root and drove five synthetic cases plus a 400×300 legibility case through the real CLI. Masks are
mode-`L` uint8 with `np.unique ⊆ {0,255}`; the `{0,1}` and `{0,255}` predictions produce
`np.array_equal` masks at 54 crack px each, proving the `pred > 0` guard (RISKS R-03). Frame
invariant holds exactly — `orig(40,30) → mask(40,30) → overlay(40,30)`, comparison `(136,30)` vs
`3·40+2·8`, and the gutter columns are pure 255. The all-zero case emits all three files, logs
`0 crack pixels (0.000%)` and exits 0. The 11×13-vs-30×40 case is refused with
`ERROR case cbad: shape mismatch: prediction (11, 13) != original (30, 40)`, writes **nothing**, and
the four healthy siblings still complete — process exit **1**, summary `status=partial`. No
`resize`/`crop`/`pad` call exists anywhere in the module, so the invariant is enforced rather than
evaded. `--alpha 0.8 --color 0,255,255` reproduces my independently computed float blend
**bit-exactly**, and unmasked pixels are byte-identical to the original — the blend really is done
in float32 and cast once. `--dry-run` left both output directories empty. Missing `case_map.json`
exits **3** with the actionable message in the scratch root *and* in the live repo. Captions were
rendered and read back: `ORIGINAL`, `MASK 0.250%`, `OVERLAY`, white-on-black plates, legible, with
no filesystem TTF (`ImageFont.load_default(size=16)`, `TypeError` fallback). `pytest tests/ -q` →
**46 passed**, exactly as claimed, no skips or xfails. Captions are drawn on copies, so the on-disk
`_mask.png` and `_overlay.png` stay uncontaminated — verified by re-reading both.

**Scope, ADRs, safety:** the commit touches exactly `src/crackvision/visualize.py`,
`tests/test_visualize.py`, `docs/COMPLETION_LOG.md` (append-only) and `task_cards/TASK_INDEX.md`.
No forbidden file, no `git add -A`, **no Claude co-author trailer**. `config/model_manifest.json`
appears in `diff.patch` but is in this run's `baseline.json` `dirty[]` — not attributable.
adr/002/008/009/010 all honoured: no skeleton, no depth projection, no ROS, no training — no
`torch` import at all. No `subprocess`, `shell=True`, `sudo`, `rm -rf` or `unlink`; nothing outside
the project tree; no GPU context opened. Live `data/overlays/` and `data/comparisons/` hold only
`.gitkeep`, confirming the agent's real end-to-end run was genuinely isolated. Orchestration gate
intact: `.task_orchestrator/**` untouched and all six `cv-*` commands still carry their 2026-09-16
mtimes.

**Judgement calls that went the agent's way.** Resolving directories from `cfg.paths` instead of
`naming.prediction_path()` produces identical paths and matches the approved precedent in
`prepare_inputs.py:146-147` and `inference.py:190-191` — using the `naming.*` literals would have
made `visualize` the only module to ignore a user's `paths:` override. Skipping the summary JSON on
`--dry-run` is likewise the established pattern in both sibling modules.

**Integration risks carried forward (documentation, not defects):** (1) `prepare_inputs --max-side`
downscales the nnU-Net input, so the prediction returns smaller than the original and *every* case
hard-fails in `visualize` — which is exactly what the card demands, but `case_map.json`'s
`downscaled`/`scale_factor` fields are consumed by nobody. **TC-011** and **TC-014** must not
exercise `--max-side` expecting visualization to work, and **TC-016** should document that it
disables the visualize/skeleton stages. (2) **TC-010** should resolve its output directories from
`cfg.paths` like this module, not from `naming.skeleton_path()` literals, or a non-default `paths:`
config would split outputs across two trees. (3) **TC-010** must apply the same
`exif_transpose` + `convert("RGB")` pair to the original before drawing its skeleton overlay —
`visualize.py:46-51` mirrors `prepare_inputs.py:53-54`, which is what closes TC-007's EXIF
integration risk; omitting it would reintroduce a shape mismatch. (4) An unknown `--cases` id is
counted failed and exits **1**, not 2 — harmless, but a harness passing a stale id sees a runtime
failure rather than a usage error.

**Non-blocking observations:** the suite's only `--alpha`/`--color` test uses the config defaults,
so criterion 9 rests on my direct verification rather than a regression guard; `--alpha` is not
range-validated (output merely clips); a hypothetical 3-channel prediction would pass the
`shape[:2]` check and then raise uncaught inside `Image.fromarray(..., mode="L")` — unreachable via
`NaturalImage2DIO.write_seg`, which writes single-channel.

**Next:** TC-009 closes. TC-010 (skeletonization) is READY; TC-011 stays BLOCKED on TC-010.
