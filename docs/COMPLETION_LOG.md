# COMPLETION_LOG.md — append-only build history

Every task card appends its completion report here, in the fixed block format defined in
`task_cards/AGENT_INSTRUCTIONS.md`. Never edit or delete an earlier entry.

---

## TC-001 — Project scaffolding and Git hygiene

```
TASK: TC-001
STATUS: COMPLETE
CHANGES:
  - .gitignore — model/data/log ignore rules per ARCHITECTURE.md §5 source-control policy
  - pyproject.toml — setuptools packaging for src/crackvision, pytest markers (gpu/camera/model)
  - config/project.yaml — created verbatim from docs/INTERFACES.md §4
  - src/crackvision/__init__.py — package init, __version__ = "0.1.0"
  - src/crackvision/config.py — find_root, Config, load_config, ConfigError, add_common_args, ensure_dirs
  - src/crackvision/logging_setup.py — EXIT_* codes, setup_logging, RunSummary, utc_stamp
  - README.md — short placeholder (TC-016 writes the real one)
  - docs/COMPLETION_LOG.md — this file (header + this entry)
  - data/input_originals/.gitkeep
  - data/nnunet_input/.gitkeep
  - data/predictions/.gitkeep
  - data/overlays/.gitkeep
  - data/comparisons/.gitkeep
  - data/skeletons/.gitkeep
  - data/smoke_test/.gitkeep
  - data/d405/color/.gitkeep
  - data/d405/depth/.gitkeep
  - data/d405/metadata/.gitkeep
  - models/.gitkeep
  - nnunet/raw/.gitkeep
  - nnunet/preprocessed/.gitkeep
  - nnunet/results/.gitkeep
  - logs/.gitkeep
  - scripts/.gitkeep
  - tools/.gitkeep
  - tests/.gitkeep
  - requirements/.gitkeep
  - git repository initialised at ~/Projects/rebot_crack_vision, branch main, one commit
TESTS:
  - `python3 -c "import yaml,sys; yaml.safe_load(open('config/project.yaml')); print('project.yaml parses')"`
    -> `project.yaml parses`
  - gitignore proof:
    `touch models/DUMMY.pth data/input_originals/DUMMY.png logs/DUMMY.log nnunet/results/DUMMY`
    `git status --porcelain --ignored | grep -c DUMMY` -> `4` (all four ignored, as required)
    `git status --porcelain | grep -c DUMMY` -> `0` (none shown as tracked/untracked)
    dummy files removed afterward; not part of any commit.
  - Module surface self-test (`sys.path.insert(0, "src")`, no editable install yet — that is TC-002):
    - `find_root(...)` resolves to the project root via the config/project.yaml + src/crackvision markers -> PASS
    - `load_config(root=root)` returns a `Config` whose `.paths`, `.model`, `.inference`,
      `.visualization`, `.skeleton`, `.realsense` match `docs/INTERFACES.md` §4 key-for-key -> PASS
    - deep-merge over `DEFAULT_CONFIG` verified with a partial YAML (missing keys fall back
      to defaults) -> PASS
    - malformed YAML raises `ConfigError` (`while parsing a flow sequence ... expected ',' or
      ']', but got '<stream end>'`) -> PASS
    - `add_common_args` adds `--root --config -v/--verbose -q/--quiet --dry-run --skip-existing`
      and they parse correctly -> PASS
    - `ensure_dirs(cfg)` creates missing directories idempotently -> PASS
    - `EXIT_OK/EXIT_RUNTIME/EXIT_USAGE/EXIT_PRECONDITION == 0/1/2/3` -> PASS
    - `utc_stamp()` returns `YYYYmmddTHHMMSSZ` (e.g. `20260917T001444Z`) -> PASS
    - `setup_logging(...)` writes to stderr and to `logs/<tool>_<UTC>.log` with the specified
      format -> PASS
    - `RunSummary().write(cfg, tool, "ok", 0)` writes `logs/<tool>_<UTC>.json` and
      `logs/<tool>_latest.json` matching the schema in `docs/INTERFACES.md` §0.4
      (`tool, schema_version, started_utc, finished_utc, duration_s, status, exit_code, counts,
      errors`) -> PASS
    - All self-test log/summary artefacts were deleted after verification; they are not part of
      this card's committed output.
  - `git init -b main` -> `Initialized empty Git repository in
    /home/boosterk1/Projects/rebot_crack_vision/.git/`
  - `git add <25 named files>` (no `-A`, no `-.`) followed by
    `git commit -m "chore: project scaffolding, config and logging primitives"`
    -> committed (see `git log --oneline` / `git log -1` in the session's shell transcript for the
    resulting hash; exactly one commit, branch `main`, author `Gavinw575 <gavinw575@gmail.com>`,
    no `Co-Authored-By` trailer).
  - `git ls-files -s` / `git cat-file -s <blob>` on every tracked blob -> largest tracked file is
    `docs/INTERFACES.md`-scale text, all committed files are well under 1 MB (all `.gitkeep` files
    are 0 bytes; source files are a few KB each).
  - Directory tree diffed against `docs/ARCHITECTURE.md` §5 -> every listed directory exists, with
    `.gitkeep` in each one that would otherwise be empty.
ISSUES:
  - `python3` used for the acceptance checks and the module self-test is the base conda
    interpreter (3.13.13, with PyYAML 6.0.3 pre-installed), not the (not-yet-created) `crackvision`
    conda env — per the card's own "Failure handling" note, TC-002 creates that env. The self-test
    added `src` to `sys.path` manually since the editable install (`pip install -e .`) happens in
    TC-002; this is expected and does not indicate a problem with `pyproject.toml`.
  - None otherwise.
NEXT CARD: TC-002
```

---

## TC-002 — Python environment, dependencies and the `env.sh` launcher

```
TASK: TC-002
STATUS: COMPLETE
CHANGES:
  - conda env `crackvision` created (Python 3.11.16), via `conda create -n crackvision python=3.11 -y`
  - torch 2.14.0+cu126 + torchvision 0.29.0+cu126 installed from the PyTorch cu126 index (first,
    per the load-bearing install order); verified CUDA-enabled before and after nnunetv2 install
  - nnunetv2 2.8.1 installed; did not disturb the CUDA torch build
  - pyyaml (6.0.3), pillow (12.3.0), scikit-image (0.26.0), opencv-python-headless (5.0.0.93),
    pytest (9.1.1), huggingface_hub (1.31.0) installed
  - pyrealsense2 2.58.4.10922 installed (optional hardware extra) and verified importable
  - `pip install -e .` — crackvision 0.1.0 editable-installed into the env
  - env.sh — new, executable (0755). Scrubbing launcher: resolves `CRACKVISION_ROOT` from its own
    path, unsets `PYTHONPATH`/`PYTHONHOME`, filters `/opt/ros` and `/usr/local/cuda-11.8` entries
    out of `LD_LIBRARY_PATH`, sets `PYTHONNOUSERSITE=1`, activates the `crackvision` conda env
    (guarded with `set +u` around the conda hook), exports `nnUNet_raw`/`nnUNet_preprocessed`/
    `nnUNet_results` (all under `$CRACKVISION_ROOT/nnunet/`) and `nnUNet_compile=f`, then either
    `exec`s the given command or returns/exits cleanly for the sourced case. No literal `/home/`
    path anywhere in the file.
  - requirements/environment.yml — new: conda-forge, python=3.11, pip -r requirements-core.txt
  - requirements/requirements-torch.txt — new: documented (non-pip-parsable) install command,
    cu126 primary / cu130,cu129 fallbacks, `torch>=2.1.2,!=2.9.*` constraint noted
  - requirements/requirements-realsense.txt — new: `pyrealsense2>=2.55,<3`, installed separately
  - requirements/requirements-core.txt — new: pinned (`==`) from `./env.sh pip freeze`, 83 packages,
    excludes torch/torchvision/nvidia-*/triton/pyrealsense2 and also `cuda-bindings`,
    `cuda-pathfinder`, `cuda-toolkit` (see ISSUES) and the project's own `-e` self-install
  - git commit `3d8247f` on `main`: "chore: crackvision python 3.11 environment, pinned deps and
    env.sh launcher" — exactly the 5 files above, author `Gavinw575 <gavinw575@gmail.com>`, no
    Claude co-author trailer
TESTS:
  - `conda env list` ->
    ```
    base                 *   /home/boosterk1/miniconda3
    crackvision              /home/boosterk1/miniconda3/envs/crackvision
    isaaclab                 /home/boosterk1/miniconda3/envs/isaaclab
    lerobot                  /home/boosterk1/miniconda3/envs/lerobot
    navila                   /home/boosterk1/miniconda3/envs/navila
    navila-vila              /home/boosterk1/miniconda3/envs/navila-vila
    vlnce-isaac              /home/boosterk1/miniconda3/envs/vlnce-isaac
    ```
    All six pre-existing envs present plus the new `crackvision` env.
  - `./env.sh python -c "import sys; r=[p for p in sys.path if 'ros/humble' in p]; print('python',
    sys.version.split()[0]); print('ROS leaks:', r); assert not r, 'SCRUB FAILED'"` ->
    `python 3.11.16` / `ROS leaks: []` — exit 0. **The critical check passes.**
  - `./env.sh python -c "import torch; print(torch.__version__, torch.version.cuda,
    torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else
    '-')"` -> `2.14.0+cu126 12.6 True NVIDIA GeForce RTX 3090`
  - `./env.sh python -c "import nnunetv2; print('nnunetv2', nnunetv2.__file__)"` ->
    `nnunetv2 /home/boosterk1/miniconda3/envs/crackvision/lib/python3.11/site-packages/nnunetv2/__init__.py`
  - `./env.sh which nnUNetv2_predict_from_modelfolder` ->
    `/home/boosterk1/miniconda3/envs/crackvision/bin/nnUNetv2_predict_from_modelfolder`
  - `./env.sh nnUNetv2_predict_from_modelfolder --help` -> exit 0, prints the nnU-Net citation
    banner and argparse help.
  - `./env.sh python -c "import numpy,skimage,PIL,cv2,yaml; print(numpy.__version__,
    skimage.__version__, PIL.__version__, cv2.__version__)"` -> `2.4.6 0.26.0 12.3.0 5.0.0`
  - `./env.sh python -c "import crackvision, os; print(crackvision.__version__);
    print({k:os.environ[k] for k in ['CRACKVISION_ROOT','nnUNet_raw','nnUNet_results',
    'nnUNet_compile']})"` -> `0.1.0` /
    `{'CRACKVISION_ROOT': '/home/boosterk1/Projects/rebot_crack_vision', 'nnUNet_raw':
    '/home/boosterk1/Projects/rebot_crack_vision/nnunet/raw', 'nnUNet_results':
    '/home/boosterk1/Projects/rebot_crack_vision/nnunet/results', 'nnUNet_compile': 'f'}`
  - `~/miniconda3/envs/isaaclab/bin/python -c "import torch; print('isaaclab torch still',
    torch.__version__)"` -> `isaaclab torch still 2.7.0+cu128` (unchanged, as recorded in
    `MACHINE_STATE.md`).
  - `grep -c "/home/" env.sh` -> `0`.
  - Sourced-mode test: `bash -c "source env.sh && python -c \"...\""` -> `sourced OK, python 3.11.16`,
    `CRACKVISION_ROOT= /home/boosterk1/Projects/rebot_crack_vision`.
  - `set -u` compatibility, both modes: `bash -c 'set -u; source env.sh; ...'` and
    `bash -uc './env.sh python -c "print(1)"'` -> both completed without an unbound-variable error.
  - `LD_LIBRARY_PATH` scrub, simulated ROS+CUDA-11.8 value
    (`/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/usr/local/cuda-11.8/lib64:/some/other/lib`)
    -> scrubbed to `/some/other/lib`. Real inherited shell value (which does contain the same
    ROS/cuda-11.8 entries — confirms F-2 empirically) -> scrubbed to empty string.
  - `grep -c crackvision ~/.bashrc` -> `0`; `stat -c '%y %n' ~/.bashrc` -> mtime `2026-06-03`,
    predating this session — `~/.bashrc` untouched.
  - `grep -iE '^(torch|torchvision|nvidia-|pyrealsense2)' requirements/requirements-core.txt` ->
    no match (PASS). Every non-comment line matches `==` (83/83 pinned).
  - `git status` before commit showed only the pre-existing untracked planning docs (`docs/*.md`,
    `docs/adr/`, `task_cards/` — not owned by this card) plus this card's 5 new files; `git add`
    named exactly those 5; `git commit` succeeded as `3d8247f`, author `Gavinw575
    <gavinw575@gmail.com>`, no co-author trailer.
ISSUES:
  - `requirements-core.txt` excludes three packages beyond the card's literal grep pattern:
    `cuda-bindings`, `cuda-pathfinder`, `cuda-toolkit`. These are new (upstream drift since the
    architecture was planned) transitive pip metapackages that the torch 2.14 cu126 wheel now
    pulls in to provide its bundled CUDA runtime — same role as the already-excluded `nvidia-*-cu12`
    packages, just not matching that name prefix. Left in, they would falsely suggest these are
    project-chosen "core" dependencies rather than torch-owned runtime pieces, and would need
    re-pinning every time the torch wheel changes. Excluded them and documented why in the file's
    header comment; this is a minor, recorded adaptation, not a silent deviation.
  - The `-e /home/boosterk1/Projects/rebot_crack_vision` editable-install self-reference that
    `pip freeze` emits for `crackvision` itself was also excluded from `requirements-core.txt` — it
    is not a dependency (it is installed via `pip install -e .`, a separate documented step), and
    including it verbatim would have put a hard-coded `/home/` path in a committed file.
  - `pyrealsense2` install succeeded on the first attempt (2.58.4.10922, matching
    `requirements-realsense.txt`'s `>=2.55,<3` constraint) — no fallback or WARN needed.
  - cu126 wheel resolved on the first attempt; no fallback to cu130/cu129 was needed.
  - No other conda env, `~/.bashrc`, system CUDA, or ROS files were touched. `pyproject.toml`,
    `config/project.yaml` and the directory tree from TC-001 were read but not modified.
  - The pre-existing dirty planning docs (`docs/ARCHITECTURE.md`, `docs/INTERFACES.md`, etc. and
    `task_cards/`) noted in the session's working-tree-was-already-dirty list remain untracked and
    were not touched or committed by this card, as expected.
NEXT CARD: TC-003
```

---

## TC-003 — Environment verification script (Level 1 smoke test)

```
TASK: TC-003
STATUS: COMPLETE
CHANGES:
  - scripts/check_env.py — new. Implements the 20 checks from docs/INTERFACES.md §3.5 as a `Check`
    dataclass (name/status/value/hint) produced by 20 individually try/except-wrapped check
    functions, run via a `_run_check` wrapper that is itself a second layer of exception safety (an
    unexpected exception in any check becomes a FAIL with the exception text as the value, never
    aborting the report). Supports `--json`, `--strict` (promotes WARN→FAIL before computing the
    exit code), and the universal common flags (`--root --config -v/--verbose -q/--quiet --dry-run
    --skip-existing`) via `crackvision.config.add_common_args`. Uses `crackvision.logging_setup
    .setup_logging` for the per-run human log and `RunSummary` for the base JSON summary schema,
    then augments both the timestamped and `_latest.json` files with the `checks` array and
    `versions` object the card's contract requires (RunSummary's dataclass has no field for these;
    see ISSUES). Exit 0 if no FAIL, exit 3 if any FAIL, exit 2 on a malformed config (`ConfigError`).
    No hard-coded `/home/` path (grep-verified).
TESTS:
  - `cd ~/Projects/rebot_crack_vision && ./env.sh python scripts/check_env.py ; echo "exit=$?"` ->
    full 20-row table printed (python_version PASS 3.11.16 ... disk_free PASS 114.23 GiB free),
    `17 PASS · 0 WARN · 3 FAIL` (project_paths, model_checkpoint, model_config_files — all FAIL
    because the model has not been fetched yet, which is the expected pre-TC-004 state), `exit=3`.
  - `./env.sh python scripts/check_env.py --json | python3 -m json.tool | head -30` -> valid JSON,
    `{"checks": [{"name": "python_version", "status": "PASS", "value": "3.11.16"}, ...`.
  - `cat logs/check_env_latest.json | python3 -m json.tool | head -20` ->
    ```
    {
      "tool": "check_env", "schema_version": 1, "started_utc": "...", "finished_utc": "...",
      "duration_s": 0.0, "status": "precondition", "exit_code": 3,
      "counts": {"pass": 17, "fail": 3},
      "errors": ["project_paths: missing: models=.../models/opencrack-nnunet",
                 "model_checkpoint: missing",
                 "model_config_files: missing: plans.json, dataset.json in .../nnUNetTrainer__nnUNetPlans__2d"],
      "checks": [ ...20 entries... ], "versions": {"python": "3.11.16", "torch": "2.14.0+cu126",
      "nnunetv2": "2.8.1", "numpy": "2.4.6", "scikit-image": "0.26.0", "Pillow": "12.3.0",
      "opencv": "5.0.0", "pyyaml": "6.0.3", "pyrealsense2": "2.58.4", "nvidia_driver": "580.173.02",
      "gpu_name": "NVIDIA GeForce RTX 3090"}
    }
    ```
    Verified programmatically: `checks` has exactly 20 entries, both required top-level keys present.
  - Poisoned-`PYTHONPATH` proof (bypassing `env.sh`, exactly as the card specifies):
    `PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages ~/miniconda3/envs/crackvision/bin/python
    scripts/check_env.py ; echo "dirty exit=$?"` ->
    `pythonpath_clean       FAIL    /opt/ros/humble/lib/python3.10/site-packages`,
    `syspath_clean          FAIL    /opt/ros/humble/lib/python3.10/site-packages`, `dirty exit=3`.
    This proves the guard actually detects a dirty environment, not merely that the check exists.
  - `--strict` WARN→FAIL promotion: on this machine every WARN-eligible check (`ld_library_path_clean`,
    `cuda_available`, `gpu_name`, `gpu_memory`, `nvidia_driver`, `pyrealsense2`, `disk_free`) currently
    reads PASS, and the only non-PASS rows are the pre-TC-004 model FAILs — so a genuine WARN-only run
    could not occur without either fetching the model (out of scope for this card) or forcing a WARN.
    Temporarily created a throwaway model tree (`models/opencrack-nnunet/.../fold_0/checkpoint_ep0500.pth`
    via `truncate -s 260M`, plus empty `plans.json`/`dataset.json`) so the three model/`project_paths`
    checks read PASS, then ran with `PATH` narrowed to exclude every directory containing `nvidia-smi`
    (`/usr/bin` and its `/bin` alias) to force exactly one WARN (`nvidia_driver`) and zero FAILs:
    - without `--strict`: `19 PASS · 1 WARN · 0 FAIL`, exit=0
    - with `--strict`: `nvidia_driver FAIL nvidia-smi not found`, `19 PASS · 0 WARN · 1 FAIL`, exit=3
    Confirms `--strict` turns a WARN-only run into exit 3. The throwaway model tree was deleted
    immediately afterward (`rm -rf models/opencrack-nnunet`); re-ran the plain command and confirmed
    `model_checkpoint`/`model_config_files`/`project_paths` are back to FAIL — the delivered repo state
    is unaffected by this demonstration.
  - Pre-TC-004 state re-confirmed after cleanup: `model_checkpoint FAIL missing`, hint names
    `fetch_model.py`; `model_config_files FAIL missing: plans.json, dataset.json in ...`; script still
    printed the full 20-row table and exited 3 (no crash).
  - `grep -n "/home/" scripts/check_env.py` -> no match (exit 1).
  - `./env.sh python -m py_compile scripts/check_env.py` -> compiles cleanly.
  - `--dry-run`: `sha256sum logs/check_env_latest.json` unchanged after
    `./env.sh python scripts/check_env.py --dry-run`; exit=0; logged
    `"dry-run: logs/check_env_latest.json not written"`.
  - `./env.sh pytest tests/ -v` -> `collected 0 items` / `no tests ran` (TC-014 adds the suite;
    `tests/` currently holds only TC-001's `.gitkeep`). Confirms this card introduced no regression.
  - Code inspection for "a single failing check never aborts the report": every one of the 20 check
    functions is called through `_run_check`, which wraps the call in `try/except Exception` and
    converts any escape into `Check(name, FAIL, f"{type(exc).__name__}: {exc}", ...)`; several checks
    additionally catch their own expected failure mode (e.g. `ImportError`) internally to attach a
    more specific hint before that outer net would ever trigger.
ISSUES:
  - `crackvision.logging_setup.RunSummary.write()` has a fixed dataclass schema (`tool,
    schema_version, started_utc, finished_utc, duration_s, status, exit_code, counts, errors`) with
    no field for the per-check `checks` array or `versions` object this card's contract requires.
    `logging_setup.py` is explicitly not-mine-to-modify (TC-001 owns it), so `check_env.py` calls
    `RunSummary.write()` unmodified for the base schema and the dual-file/timestamp mechanics, then
    reads back both the `_latest.json` and its timestamped twin (located by globbing
    `check_env_*.json` and taking the lexicographically-last, which sorts chronologically since the
    stamp is `YYYYmmddTHHMMSSZ`) and merges in `checks`/`versions` before rewriting them. Flagging
    this as a documented adaptation, not a silent deviation — a future card that wants generic
    extra-fields support in `RunSummary` would need to get that added by whichever card owns
    `logging_setup.py`.
  - `project_paths` (check #16) legitimately FAILs right now, not just `model_checkpoint`/
    `model_config_files`: `config/project.yaml`'s `paths.models` is `models/opencrack-nnunet`
    (INTERFACES.md §4), and TC-001 only created the placeholder `models/.gitkeep`, not the
    `opencrack-nnunet` subdirectory — that subdirectory is created by `fetch_model.py` (TC-004). The
    card's own check table gives `project_paths` a fixed hint of "run TC-001", which is not the
    actually-correct remedy in this specific case (TC-001 already ran correctly; TC-004 is what's
    missing). Implemented the hint text exactly as specified rather than improvising a different one,
    per "do not redesign the architecture" — flagging the mismatch here rather than silently
    patching the hint table.
  - `--dry-run`/`--skip-existing` are accepted (via the shared `add_common_args`) for CLI-surface
    consistency with docs/INTERFACES.md §0.3 ("every CLI accepts all of these"), even though the
    card's own signature for this script only lists `[--json] [--strict]` and neither flag is
    exercised by the card's acceptance criteria. `--dry-run` skips the one side effect this tool has
    (writing `logs/check_env_latest.json`) and always exits 0, per the flag's documented contract
    ("log what would be done, write nothing, exit 0"). `--skip-existing` is accepted but a no-op:
    check_env has no generated artifact to conditionally skip regenerating.
  - `docs/COMPLETION_LOG.md` already contained TC-002's completion report, uncommitted, when this
    session started (TC-002 correctly left it uncommitted per its own commit guidance — confirmed in
    `docs/REVIEW_LOG.md`'s TC-002 entry). This card's commit of `docs/COMPLETION_LOG.md` necessarily
    carries that pre-existing TC-002 append along with this entry, since git diffs the whole file and
    the two entries were never committed separately — not a scope expansion, just how the shared
    append-only log accumulates between review gates.
  - None otherwise.
NEXT CARD: TC-004
```

---

## TC-004 — OpenCrack model acquisition from Hugging Face

```
TASK: TC-004
STATUS: COMPLETE
CHANGES:
  - scripts/fetch_model.py — new. Downloads fadeevla/opencrack-nnunet via
    huggingface_hub.snapshot_download(local_dir=..., allow_patterns=[...]) (no
    local_dir_use_symlinks kwarg — confirmed absent from huggingface_hub 1.31.0's signature, so
    nothing needed dropping), validates the three required files (plans.json, dataset.json,
    fold_0/checkpoint_ep0500.pth) exist with the checkpoint in the 250-300 MB range, computes
    sha256 in 1 MiB chunks over every real snapshot file (excluding huggingface_hub's
    `.cache/huggingface/**` resume bookkeeping), and writes config/model_manifest.json atomically
    (tempfile + os.replace). Idempotent by default (skips the network call entirely when the three
    required files are already present); `--force` re-downloads; `--verify-only` re-hashes the
    files on disk against the existing manifest without touching the network for the model itself
    (still makes one best-effort HfApi call only in the non-verify-only path, to capture the
    upstream revision sha). `--dry-run` reports the plan and writes nothing. Logs the CC-BY-4.0
    licence + citation line at INFO on every invocation. Uses crackvision.config/logging_setup
    exactly as TC-003 does (RunSummary + setup_logging); no hard-coded `/home/` path (grep-verified).
  - config/model_manifest.json — generated by running the script (see TESTS for full contents).
  - models/opencrack-nnunet/** — the downloaded snapshot (268 MB checkpoint + 4 smaller files),
    gitignored per TC-001's .gitignore, not committed.
  - task_cards/TASK_INDEX.md — TC-004 row COMPLETE, TC-005 row READY, progress line updated.
TESTS:
  - `./env.sh python -m py_compile scripts/fetch_model.py` -> compiles cleanly.
  - `grep -n "/home/" scripts/fetch_model.py` -> no match (exit 1).
  - `./env.sh python scripts/fetch_model.py --dry-run` -> logs the licence line, then
    `dry-run: would download fadeevla/opencrack-nnunet (missing: plans.json, dataset.json,
    checkpoint)`, exit=0. Nothing written (confirmed no models/ dir existed yet at this point).
  - `./env.sh python scripts/fetch_model.py ; echo "exit=$?"` -> downloaded 6 files in ~12.4s
    wall-clock (`Fetching 6 files: 100%|...`), logged checkpoint size `255.7 MB`, wrote the
    manifest, logged all 6 file paths/sizes/sha256 values, `exit=0`.
  - `du -sh models/opencrack-nnunet` -> `266M`.
  - `find models/opencrack-nnunet -type f | sort` -> exactly the 4 Dataset501_OpenCrack files
    (dataset.json, dataset_fingerprint.json, fold_0/checkpoint_ep0500.pth, plans.json) +
    README.md + reproducibility/splits_final.json, plus huggingface_hub's own
    `.cache/huggingface/**` resume-lock/metadata files (excluded from the manifest and from git;
    matches the exact 7-file upstream tree from ARCHITECTURE.md §4.1 minus `.gitattributes`, which
    `allow_patterns` correctly excludes, exactly as the card's own snapshot_download example omits it).
  - `python3 -m json.tool config/model_manifest.json` -> valid JSON:
    ```json
    {
      "schema_version": 1, "repo_id": "fadeevla/opencrack-nnunet",
      "downloaded_utc": "2026-09-17T04:49:49Z",
      "revision": "1198179e893f5f6eb0dd3eae2d8de5f1adf85afc",
      "files": [
        {"path": "Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/dataset.json", "bytes": 400,
         "sha256": "2b8c47ce6a12fe5d3469437df4efe58b970dbf2a0b45d1c83393c3f5ca61dc08"},
        {"path": "Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/dataset_fingerprint.json",
         "bytes": 6738791, "sha256": "0990276837260c7c600eca25189503dd0be33ce6cb944c1614a72f7cac3e6b7f"},
        {"path": "Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/fold_0/checkpoint_ep0500.pth",
         "bytes": 268091674, "sha256": "dcba82012874682c84387c68a2b69f64eddef07e84ae7d813844cdf6396a1a5f"},
        {"path": "Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json", "bytes": 6361,
         "sha256": "08f03a8e0f2312d9756e6637a2b8e115a5e27c898e93fe218784d0a69dc01eeb"},
        {"path": "README.md", "bytes": 6067,
         "sha256": "26b9df32e6c1e162a409f00ff8bd06480d340e0c40c4e078222c21b4c4ccfda6"},
        {"path": "reproducibility/splits_final.json", "bytes": 3035124,
         "sha256": "990ed5aca9e9dce4ba2edab68438a13a5f6d16ca17f5bcf01e68a3f2e14c419d"}
      ]
    }
    ```
    Checkpoint sha256 independently cross-checked with `sha256sum` on the file directly -> identical
    value (`dcba82012874682c84387c68a2b69f64eddef07e84ae7d813844cdf6396a1a5f`).
  - `./env.sh python scripts/fetch_model.py ; echo "second run exit=$?"` -> `fadeevla/opencrack-nnunet
    already present at .../models/opencrack-nnunet; skipping download (idempotent)`, manifest
    rewritten (fresh hashes, same values), `real 0m1.548s` (vs 12.374s for the actual download —
    no re-download occurred), `exit=0`.
  - `./env.sh python scripts/fetch_model.py --verify-only ; echo "verify exit=$?"` -> `verify-only:
    6 files match the recorded manifest`, `exit=0`.
  - `./env.sh python scripts/check_env.py ; echo "exit=$?"` -> **20 PASS · 0 WARN · 0 FAIL**, `exit=0`
    — `model_checkpoint` (`255.7 MB at .../fold_0/checkpoint_ep0500.pth`) and `model_config_files`
    (`plans.json, dataset.json present in ...`) both PASS, and `project_paths` (which was also FAILing
    pre-TC-004 per TC-003's own report) now PASSes too, as TC-003's review anticipated.
  - `du -sh ~/.cache/huggingface` before and after -> `17G` both times (unchanged; `local_dir`
    download confirmed not to duplicate into the hub cache).
  - `git status --porcelain | grep -c '^?? models'` -> `0`.
  - `git status --porcelain --ignored` -> only `config/model_manifest.json` and
    `scripts/fetch_model.py` appear as new tracked-candidate files (plus the pre-existing modified
    `task_cards/TASK_INDEX.md`); `models/opencrack-nnunet/`, all `logs/**`, `.task_orchestrator/`,
    `scripts/__pycache__/`, `src/crackvision.egg-info/`, `src/crackvision/__pycache__/` all show as
    `!!` (ignored), confirming `.gitignore` holds and nothing outside this card's file list changed.
  - Licence line: `grep -l "CC-BY-4.0" logs/fetch_model_20260917T044952Z.log` -> match found; the
    line appears in every run's log, not just the download run.
ISSUES:
  - `huggingface_hub` 1.31.0's `snapshot_download` signature has no `local_dir_use_symlinks`
    parameter at all (confirmed via `inspect.signature`) — modern versions removed it outright
    rather than merely deprecating it, exactly as the card warned. No kwarg was ever passed, so
    there was nothing to drop at runtime; noted here only because the card asked for this to be
    recorded if encountered.
  - The checkpoint downloaded at 255.7 MB (268 091 674 bytes), inside the card's 250-300 MB
    acceptance band but below the 268.09 MB the architecture doc quotes from planning-time
    metadata — within normal rounding/reporting variance for the same file (the card's own PASS
    band already anticipates exactly this), not a truncation: independently re-hashed via
    `sha256sum` outside the script and it matches the manifest exactly.
  - `local_dir` downloads (this huggingface_hub version) create a `.cache/huggingface/` bookkeeping
    subdirectory inside `models/opencrack-nnunet/` (lock/metadata files for resumable downloads,
    plus a `trees/<sha>.json`). This is upstream `huggingface_hub` behaviour, not something the card
    anticipated by name. It is harmless (fully covered by the existing `models/*` gitignore rule, so
    it was never at risk of being committed) but would have polluted `config/model_manifest.json`
    with irrelevant hash entries if left unfiltered, so `_collect_manifest_files` explicitly skips
    any path with a dot-prefixed path component. Flagging as a documented adaptation, not a
    silent deviation.
  - Per the card's explicit scope boundary, semantic validation of `dataset.json`/`plans.json`
    content (channel_names, labels, patch_size, etc.) and the `nnunet/results` symlink were
    deliberately NOT implemented here — that is TC-005. This card only confirmed file
    existence/size/hash, as instructed.
  - `--force` (re-download) was not exercised against the live network a second time in this
    session — its download branch is code-path-identical to the first real download already
    exercised above (`need_download=True` triggers the same `snapshot_download(...)` call
    regardless of why); re-running it here would only have re-spent ~12s and bandwidth without
    adding coverage, so it was reasoned about rather than re-run.
NEXT CARD: TC-005
```

---

## TC-005 — nnU-Net model wiring and semantic validation

```
TASK: TC-005
STATUS: BLOCKED
CHANGES:
  - scripts/verify_model.py — new. Implements docs/INTERFACES.md §3.7: structural check for the
    three required files; loads dataset.json/plans.json defensively (a JSON parse failure produces
    a FAIL check plus "not evaluated" FAILs for the dependent semantic assertions, never a crash);
    the four dataset.json assertions (channel_names, labels, file_ending,
    overwrite_image_reader_writer) and five plans.json assertions ("2d" in configurations,
    patch_size==[256,256], len(normalization_schemes)==3, plans_name, dataset_name), each compared
    against the exact literals in the card and logged with the observed value; checkpoint sanity via
    `torch.load(weights_only=True)` with the documented magic-byte (`PK\x03\x04`) fallback on
    failure — `weights_only=False` appears nowhere in the file (grep-verified); creates/repairs the
    relative `nnunet/results/Dataset501_OpenCrack` symlink (refuses to touch a real file/directory,
    only replaces a *broken* symlink and only with `--repair-symlink`, verifies it resolves through
    to `plans.json`); `--check-hashes` recomputes sha256 for every file **named in the manifest**
    (manifest-driven, not disk-driven, per TC-004 review F-1's warning about a one-directional
    loop — a manifest entry whose file is missing on disk is reported as a mismatch, not silently
    skipped) and exits 1 naming any mismatch, independent of the 0/3 exit code used by the other
    checks. `--dry-run` performs all read-only checks but skips symlink creation/repair and the log
    write, exiting 0 per docs/INTERFACES.md §0.3 (same literal-contract interpretation TC-003's
    review accepted for check_env.py). Writes `logs/verify_model_latest.json` (RunSummary's base
    schema augmented with a `checks` array, same documented pattern as check_env.py, since
    logging_setup.py is not mine to modify). No hard-coded `/home/` path (grep-verified).
  - docs/COMPLETION_LOG.md — this entry.
  - task_cards/TASK_INDEX.md — TC-005 row set to `BLOCKED` with a one-line reason; TC-006 left
    `BLOCKED` (its prerequisite is not COMPLETE).
TESTS:
  - `./env.sh python -m py_compile scripts/verify_model.py` -> compiles cleanly.
  - `grep -n "/home/" scripts/verify_model.py` -> no match (exit 1).
  - `grep -n "weights_only=False\|weights_only = False" scripts/verify_model.py` -> no match (exit 1).
  - `./env.sh python scripts/verify_model.py ; echo "exit=$?"` ->
    `required_files PASS`, `dataset_json_parse PASS`, all four `dataset_*` assertions **PASS** with
    observed values `{"0": "R", "1": "G", "2": "B"}` / `{"background": 0, "crack": 1}` / `".png"` /
    `"NaturalImage2DIO"` — **byte-identical to the card's contract**. `plans_json_parse` **FAIL**
    `JSONDecodeError: Extra data: line 218 column 2 (char 6361)`, and the five dependent
    `plans_*` checks **FAIL** `not evaluated (parse failed)` (see ISSUES — this is real file
    corruption in `models/`, not an implementation defect). `checkpoint_loadable` **WARN**:
    `weights_only=True` raised `UnpicklingError: ... Unsupported global: GLOBAL
    numpy._core.multiarray.scalar ...` (exactly the non-tensor-metadata case the card
    anticipated), fell back to the magic-byte check, which passed. `nnunet_results_symlink`
    **PASS**, symlink created. `7 PASS · 1 WARN · 6 FAIL`, `exit=3`.
  - `./env.sh python scripts/verify_model.py --check-hashes ; echo "hashes exit=$?"` -> same table
    plus `check_hashes FAIL Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json: sha256
    mismatch (manifest=08f03a8e0f2312d9756e6637a2b8e115a5e27c898e93fe218784d0a69dc01eeb,
    actual=68f9754477c76e07a28e4bfc091d81506766e6d539e0d0b1cb3a92d492cc51dd)`; `hashes exit=1`
    (not 3 — matches the card's distinct exit-1 contract for a hash mismatch). The other five
    manifest files (dataset.json, dataset_fingerprint.json, the 268 091 674-byte checkpoint,
    README.md, splits_final.json) all matched.
  - `ls -la nnunet/results/` -> `Dataset501_OpenCrack -> ../../models/opencrack-nnunet/Dataset501_OpenCrack`.
  - `readlink nnunet/results/Dataset501_OpenCrack` -> `../../models/opencrack-nnunet/Dataset501_OpenCrack`
    (starts with `../` — relative, per adr/004).
  - `test -f nnunet/results/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json && echo
    "symlink resolves"` -> `symlink resolves`.
  - `./env.sh python -c "import os,glob; print(sorted(glob.glob(os.path.join(os.environ['nnUNet_results'],'Dataset501*'))))"`
    -> `['/home/boosterk1/Projects/rebot_crack_vision/nnunet/results/Dataset501_OpenCrack']` — the
    `-d 501` fallback path (adr/006) is now discoverable.
  - Idempotency: re-ran `./env.sh python scripts/verify_model.py` a second time -> `exit=3`
    (unchanged — same plans.json corruption), `readlink` unchanged, no error or symlink recreation
    attempted; `nnunet_results_symlink` stayed **PASS** both times.
  - `find models -newer config/model_manifest.json` -> empty, confirmed **before and after** every
    test in this session, including the deliberate symlink-edge-case tests below (which only write
    under `nnunet/results/`) — `models/**` was never modified.
  - Symlink edge cases, exercised directly (each restored to the passing state immediately after):
    - Pointed the symlink at a nonexistent target, ran without `--repair-symlink` -> `FAIL "broken
      symlink: .../Dataset501_OpenCrack -> ../../models/opencrack-nnunet/NONEXISTENT_TARGET"`, hint
      `"re-run with --repair-symlink"`, symlink left untouched (not silently fixed).
    - Same broken symlink, ran **with** `--repair-symlink` -> repaired, `nnunet_results_symlink`
      **PASS**, correct target restored.
    - Replaced the path with a real empty directory, ran the script -> `FAIL "a real directory
      already exists at .../Dataset501_OpenCrack; not deleting it"`; `ls -la` confirmed the
      directory was still there afterward, unmodified — the script never deletes it.
    - Restored the correct relative symlink afterward and re-verified `nnunet_results_symlink`
      **PASS** and `readlink` correct (see the final run above) — the repo is left in the same
      passing symlink state these edge-case tests started from.
  - `./env.sh python scripts/verify_model.py --dry-run` -> full table printed (same PASS/FAIL
    pattern), logged `"dry-run: logs/verify_model_latest.json not written"`, `exit=0`; confirmed no
    new `logs/verify_model_*.json` timestamp was written for that invocation and the symlink was
    not touched (already correct at the time).
  - `cat logs/verify_model_latest.json` -> valid JSON with the `tool/schema_version/started_utc/
    finished_utc/duration_s/status/exit_code/counts/errors` envelope plus a `checks` array of all
    check dicts (`name/status/value/hint`) — `status: "precondition"`, `exit_code: 3`,
    `counts: {"pass": 7, "fail": 6, "warn": 1}`.
  - `./env.sh pytest tests/ -v` -> `collected 0 items` / `no tests ran` — no regression (TC-005 adds
    no test file per its own scope; `tests/` still only holds TC-001's `.gitkeep`).
  - `git status --porcelain` before and after -> only `scripts/verify_model.py` added beyond the
    pre-existing dirty state (`docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md` modified;
    planning docs/`task_cards/TC-0NN-*.md` untracked) that was present before this session started;
    `models/opencrack-nnunet/` and `nnunet/results/Dataset501_OpenCrack` both show as `!!` (ignored).
ISSUES:
  - **Root-cause finding, not an implementation defect:**
    `models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json` is
    corrupted on disk — it is 6362 bytes where `config/model_manifest.json` (committed, byte-for-byte
    the same as git's `e118073` content) records 6361, and a raw hex dump shows the file ends
    `7d 0a 7d 58` (`}\n}X`): a single stray `X` byte appended after the final `}`. Its sha256
    (`68f9754477c76e07a28e4bfc091d81506766e6d539e0d0b1cb3a92d492cc51dd`) does not match the
    manifest's recorded value (`08f03a8e0f2312d9756e6637a2b8e115a5e27c898e93fe218784d0a69dc01eeb`).
    Stripping that one trailing byte and parsing the rest (done only in-memory, for diagnosis — the
    file on disk was never written to) shows every other field intact and exactly matching
    `ARCHITECTURE.md` §4.1's quoted values: `dataset_name="Dataset501_OpenCrack"`,
    `plans_name="nnUNetPlans"`, `configurations` has exactly one key `"2d"`,
    `patch_size=[256,256]`, `normalization_schemes` has 3 `"ZScoreNormalization"` entries. This is
    **not upstream drift** — it is a corrupted local copy of an otherwise-correct file.
    `stat` shows `plans.json`'s mtime (`2026-09-16 23:39:01`) is ~49 minutes after every sibling
    file from TC-004's original download (`22:49:2x`) and within 8 seconds of
    `config/model_manifest.json`'s mtime (`23:39:09`, though that file's *content* is unmodified
    relative to its committed `e118073` blob) — consistent with `docs/REVIEW_LOG.md`'s TC-004 entry,
    which describes the reviewer exercising "a deliberately corrupted `plans.json` -> 3 with a sha256
    mismatch" to prove `verify_model.py`'s (this card's) hash check would work, apparently on the
    real file rather than a scratch copy, and apparently never reverted — plausible because
    `models/**` is gitignored, so a `git checkout` restoring the tracked `config/model_manifest.json`
    would not have restored a hand-edited, untracked `plans.json`. I did not corrupt this file and
    did not modify anything under `models/` at any point this session (verified repeatedly via
    `find models -newer config/model_manifest.json` -> empty, before and after every test).
  - Per this card's own Failure handling section ("A semantic assertion fails -> BLOCKED... Do not
    edit dataset.json or plans.json to make it pass — that is falsifying the model") and its file
    list ("Files that must NOT be modified: models/** — read-only... config/model_manifest.json
    (TC-004 owns it)"), I did not edit `plans.json` and did not run
    `./env.sh python scripts/fetch_model.py --force` myself — that command would rewrite both
    `models/**` and `config/model_manifest.json`, both outside this card's scope (TC-004 owns
    acquisition). I only *diagnosed* the corruption using read-only operations.
  - **Recommended remedy (not performed by this session):** re-run
    `./env.sh python scripts/fetch_model.py --force` (TC-004) to get a clean `plans.json`, then
    re-run `./env.sh python scripts/verify_model.py --check-hashes`. Once that passes, TC-005's
    acceptance criteria should all read PASS with no further code changes needed — the five
    `plans_*` checks and `check_hashes` are the only ones affected by this corruption; every other
    check (structural, all four `dataset_*` assertions, checkpoint sanity, the symlink and every one
    of its edge cases) already passes against the current model tree.
  - `torch.load(..., weights_only=True)` raises `_pickle.UnpicklingError` on this real checkpoint
    (`Unsupported global: GLOBAL numpy._core.multiarray.scalar`) exactly as the card's §3
    anticipated for nnU-Net checkpoints carrying non-tensor metadata (`init_args`, `trainer_name`,
    etc.); the magic-byte fallback (`PK\x03\x04`) correctly identifies it as a valid ZIP/torch
    archive, logged as WARN, not FAIL, per the card's documented fallback path.
  - `dataset.json`'s exact printed values (documented per the card's "Documentation update"
    requirement): `channel_names={"0": "R", "1": "G", "2": "B"}`,
    `labels={"background": 0, "crack": 1}`, `file_ending=".png"`,
    `overwrite_image_reader_writer="NaturalImage2DIO"`. `plans.json`'s values, recovered only for
    diagnosis from the corrupted file's otherwise-intact content (not asserted PASS by the tool,
    since the file does not currently parse): `patch_size=[256, 256]`,
    `normalization_schemes` has 3 entries (all `ZScoreNormalization`), `plans_name="nnUNetPlans"`,
    `dataset_name="Dataset501_OpenCrack"`, `configurations` keys `["2d"]`.
NEXT CARD: TC-005 (re-run after `fetch_model.py --force` restores a clean `plans.json`; TC-006 stays
BLOCKED until then)
```

---

## TC-005 (revision 1) — nnU-Net model wiring and semantic validation

```
TASK: TC-005
STATUS: COMPLETE
CHANGES:
  - No code changes. scripts/verify_model.py is byte-identical to the version committed in 14c05b4
    (`git diff --stat scripts/verify_model.py` -> empty) — the audit's finding was that the script
    is correct and the *model tree* was corrupt, not the script.
  - models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json — repaired
    by re-running `./env.sh python scripts/fetch_model.py --force` (TC-004's own tool), per the
    review's explicit, scoped, revision-1-only authorisation to write models/** and
    config/model_manifest.json to undo review-tooling damage (see ISSUES). Not hand-edited.
  - config/model_manifest.json — rewritten by the same `fetch_model.py --force` run (TC-004 owns
    this file; the write is the authorised side effect of the repair, not a TC-005 edit). Only
    `downloaded_utc` changed (`2026-09-17T04:49:49Z` -> `2026-09-19T01:26:14Z`); `revision` and all
    six files' `bytes`/`sha256` are byte-identical to the pre-repair manifest — confirmed by
    `git diff config/model_manifest.json` before running any further checks. Left uncommitted per
    the review's commit instructions (see ISSUES) — TC-004 owns this file, TC-005 does not commit it.
  - docs/COMPLETION_LOG.md — this entry.
  - task_cards/TASK_INDEX.md — TC-005 row set to `COMPLETE`; TC-006 row set to `READY` (TC-003 is
    COMPLETE); Progress section narrative updated to 5/16 and to describe the repair instead of the
    block.
TESTS:
  - Pre-repair confirmation (read-only, matching the audit's diagnosis exactly):
    `sha256sum .../plans.json` -> `68f9754477c76e07a28e4bfc091d81506766e6d539e0d0b1cb3a92d492cc51dd`
    (6362 bytes); `tail -c 20 .../plans.json | xxd` -> hex tail `...7d0a 7d58` (`}\n}X`, one stray
    `X` byte after the final `}`), matching the manifest's expected 6361 bytes /
    `08f03a8e0f2312d9756e6637a2b8e115a5e27c898e93fe218784d0a69dc01eeb` mismatch reported by the audit.
  - `./env.sh python scripts/fetch_model.py --force ; echo "exit=$?"` -> re-downloaded 6 files,
    logged `plans.json 6361 bytes sha256=08f03a8e0f2312d9756e6637a2b8e115a5e27c898e93fe218784d0a69dc01eeb`,
    wrote the manifest, `exit=0`.
  - Post-repair verification: `wc -c .../plans.json` -> `6361`; `sha256sum .../plans.json` ->
    `08f03a8e0f2312d9756e6637a2b8e115a5e27c898e93fe218784d0a69dc01eeb` (matches manifest exactly);
    `python3 -m json.tool .../plans.json > /dev/null` -> parses cleanly, no error.
  - `git diff config/model_manifest.json` -> only the `downloaded_utc` line changed; `revision`
    (`1198179e893f5f6eb0dd3eae2d8de5f1adf85afc`) and every file's `bytes`/`sha256` unchanged ->
    confirms this is repair of a corrupted local copy, not upstream drift, per the review's own
    stop-condition.
  - `./env.sh python scripts/verify_model.py --check-hashes ; echo "exit=$?"` ->
    `required_files PASS`, `dataset_json_parse PASS`, all four `dataset_*` PASS (values below),
    `plans_json_parse PASS`, `plans_configuration_present PASS configurations contains '2d'`,
    `plans_patch_size PASS [256, 256]`, `plans_normalization_schemes PASS ["ZScoreNormalization",
    "ZScoreNormalization", "ZScoreNormalization"]`, `plans_name PASS "nnUNetPlans"`,
    `plans_dataset_name PASS "Dataset501_OpenCrack"`, `checkpoint_loadable WARN` (documented
    magic-byte fallback, `weights_only=True` raised `UnpicklingError` for
    `numpy._core.multiarray.scalar`, exactly as anticipated), `nnunet_results_symlink PASS`,
    `check_hashes PASS 6 files match .../config/model_manifest.json`. `14 PASS · 1 WARN · 0 FAIL`,
    `exit=0`. Matches the review's accept condition for issue 1 exactly.
  - `./env.sh python scripts/verify_model.py ; echo "exit=$?"` (no `--check-hashes`, the card's own
    first command) -> same 13 non-hash checks, all PASS except the same documented
    `checkpoint_loadable` WARN. `13 PASS · 1 WARN · 0 FAIL`, `exit=0`.
  - Idempotency: `stat -c '%i %Z' nnunet/results/Dataset501_OpenCrack` before and after a second
    consecutive `./env.sh python scripts/verify_model.py` run -> `20879111 1789624174` both times
    (byte-identical inode and ctime); second run's table and exit code identical to the first
    (`13 PASS · 1 WARN · 0 FAIL`, `exit=0`).
  - `ls -la nnunet/results/` -> `Dataset501_OpenCrack -> ../../models/opencrack-nnunet/Dataset501_OpenCrack`.
  - `readlink nnunet/results/Dataset501_OpenCrack` -> `../../models/opencrack-nnunet/Dataset501_OpenCrack`
    (relative, starts with `../`, per adr/004).
  - `test -f nnunet/results/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json && echo
    "symlink resolves"` -> `symlink resolves`.
  - `./env.sh python -c "import os,glob; print(sorted(glob.glob(os.path.join(os.environ['nnUNet_results'],'Dataset501*'))))"`
    -> `['/home/boosterk1/Projects/rebot_crack_vision/nnunet/results/Dataset501_OpenCrack']` — the
    `-d 501` fallback path (adr/006) is discoverable.
  - `find models -newer config/model_manifest.json` -> empty (run after the repair and after every
    verification pass in this session) — confirms `models/**` was not touched again after the
    authorised repair, and that the repair itself left the manifest no older than the model files.
  - `cat logs/verify_model_latest.json` -> valid JSON envelope (`tool/schema_version/started_utc/
    finished_utc/duration_s/status/exit_code/counts/errors`) plus a `checks` array of 15 entries
    (all 14 core checks + `check_hashes`, from the `--check-hashes` run that ran last), `status:
    "ok"`, `exit_code: 0`, `counts: {"pass": 14, "warn": 1}`. (Note: the review's accept text said
    "a 14-entry checks array once --check-hashes is included" — the actual array has 15 entries,
    one per check row printed in the table, including `check_hashes` itself; the 14 PASS + 1 WARN =
    15 total is what the printed table's own summary line also shows. Flagging the discrepancy
    between the review's wording and the observed array length rather than silently matching it.)
  - `git diff --stat scripts/verify_model.py` -> empty (no code change).
  - `git status --porcelain` -> `config/model_manifest.json` and `task_cards/TASK_INDEX.md` modified
    (both expected — see CHANGES); the same pre-existing untracked planning docs from session start;
    no other file touched.
  - `./env.sh pytest tests/ -v` -> `collected 0 items` / `no tests ran` (unchanged from TC-005
    revision 0 and TC-003; `tests/` still holds only TC-001's `.gitkeep` — TC-005 owns no test file).
  - Dataset.json / plans.json contract values, printed and PASS-verified this run (per the card's
    Documentation-update requirement): `channel_names={"0": "R", "1": "G", "2": "B"}`,
    `labels={"background": 0, "crack": 1}`, `file_ending=".png"`,
    `overwrite_image_reader_writer="NaturalImage2DIO"`, `patch_size=[256, 256]`.
ISSUES:
  - **Root cause (established by the review, re-confirmed here, not re-litigated):** the corrupt
    `plans.json` was a hardlink write-through from TC-004's own review scratch tree (built with
    hardlinks; `find -samefile` showed the scratch copy and the real model file shared one inode) —
    the reviewer's deliberate "corrupt a scratch plans.json" test at `2026-09-17T05:39:01Z` wrote
    through into the real file and was never reverted, 2m15s before the original TC-005 attempt began.
    Not caused by any implementation card. Repaired per the review's explicit, scoped exception for
    this revision only (`fetch_model.py --force`, authorised to touch `models/**` and
    `config/model_manifest.json` despite TC-005's normal file-ownership list).
  - `config/model_manifest.json` is left **modified but uncommitted** in the working tree
    (`downloaded_utc` only). The review's commit instructions for this revision name exactly three
    files to commit — `scripts/verify_model.py` (unchanged, so nothing to commit there),
    `docs/COMPLETION_LOG.md`, and `task_cards/TASK_INDEX.md` — and do not include
    `config/model_manifest.json`, which TC-004 owns. I did not commit it. The next session/reviewer
    should be aware `git status` will show this file as dirty; it reflects the sanctioned repair, not
    an unreviewed change, and its content (aside from the timestamp) is identical to what TC-004
    originally produced and already committed in `e118073`.
  - Per issue 3 (minor, optional) in the review report: `verify_model.py`'s exit-code precedence
    (`hash_fail` short-circuits to exit 1 even when core FAILs are also present) is left as-is. This
    is the intentional, literal reading of the card's own contract ("`--check-hashes` re-verifies
    sha256... Mismatch -> exit 1 naming the file", `docs/INTERFACES.md` §3.7.5, distinct from the
    0/3 exit codes used by every other check) — changing it would mean guessing a new, undocumented
    precedence rule the card never specified. Not applicable to the currently-delivered tree (no
    core FAILs are present after the repair), so it could not be re-reproduced this run in any case.
  - Per issue 4 (minor, optional): `verify_model.py --dry-run` exits 0 unconditionally, even if the
    read-only checks it ran would otherwise report FAILs. This mirrors `check_env.py`'s accepted
    literal reading of `docs/INTERFACES.md` §0.3 ("log what would be done, write nothing, exit 0")
    and is unchanged. **TC-011's `run_test.sh` must not call `verify_model.py --dry-run` as a
    pass/fail gate** — it must call it without `--dry-run` and branch on the real exit code (0/1/3).
  - `checkpoint_loadable` still WARNs on every run (`weights_only=True` raises `UnpicklingError:
    Unsupported global: GLOBAL numpy._core.multiarray.scalar`), falling back to the magic-byte check
    as the card's §3 explicitly anticipates for nnU-Net checkpoints carrying non-tensor metadata.
    This is unchanged from revision 0 and is not a defect — `weights_only=False` appears nowhere in
    the file (`grep -n "weights_only=False\|weights_only = False" scripts/verify_model.py` -> no
    match).
NEXT CARD: TC-006
```

---

## TC-006 — Synthetic end-to-end smoke test (Level 2 core)

```
TASK: TC-006
STATUS: COMPLETE
CHANGES:
  - scripts/smoke_test.py — new. Implements docs/INTERFACES.md §3.8: generates two deterministic
    (numpy.random.default_rng(1234)) 512x512 synthetic fixtures into data/smoke_test/input/
    (synthetic_crack.png: mid-grey + Gaussian noise + a dark 2-3px wandering polyline with one
    branch, Gaussian-blurred; synthetic_blank.png: same base, no crack), force-converts each to
    3-channel RGB PNG at data/smoke_test/nnunet_input/<case>_0000.png, builds the
    nnUNetv2_predict_from_modelfolder command as a list (never shell=True) from config/project.yaml
    values (-i/-o/-m/-f 0/-chk checkpoint_ep0500.pth/-device/-npp 3/-nps 3), streams its stdout+stderr
    into the log, and on success renders a minimal mask (x255)/red overlay/skimage skeleton into
    data/smoke_test/{overlays,skeletons}/ for eyeballing. Asserts (never accuracy): subprocess exit 0,
    prediction files exist and non-empty, dtype==uint8, unique values subset of {0,1}, shape==(512,512)
    == input shape, mask/overlay/skeleton files exist at 512x512, skeleton_pixels <= mask_pixels.
    Prints the SMOKE TEST PASSED/FAILED banner, per-stage timings, and the exact nnU-Net command line.
    `--device {cuda,cpu}` (default cuda; exits 3 if cuda requested but torch.cuda.is_available() is
    False), `--clean` (removes prior data/smoke_test/{input,nnunet_input,predictions,overlays,
    skeletons} first), `--dry-run` (writes nothing, exits 0, never imports torch). CUDA-OOM detection
    on the subprocess output prints the ARCHITECTURE.md §9 degradation ladder and exits 1 (never
    retries automatically, never kills a GPU process). Uses crackvision.config/logging_setup exactly
    as TC-003/004/005 do (RunSummary + setup_logging). No hard-coded `/home/` path (grep-verified).
  - docs/COMPLETION_LOG.md — this entry.
  - task_cards/TASK_INDEX.md — TC-006 row set to COMPLETE; progress narrative updated to 6/16. No
    other row flips: TC-008 (the only card whose deps include TC-006) still needs TC-007, which is
    unaffected by this card and was already READY.
TESTS:
  - `./env.sh python -m py_compile scripts/smoke_test.py` -> compiles cleanly.
  - `grep -n "/home/" scripts/smoke_test.py` -> no match. `grep -n "shell=True"` -> only inside a
    docstring sentence ("never shell=True"), not actual usage. `grep -n "weights_only"` -> no match
    (this script never touches the checkpoint directly; verify_model.py already validated it).
  - Pre-flight model integrity check (per TC-005 review's own recommendation to run --check-hashes
    first as the cheapest detector for a hardlink recurrence):
    `./env.sh python scripts/verify_model.py --check-hashes` -> `14 PASS · 1 WARN · 0 FAIL`, exit=0,
    `check_hashes PASS 6 files match .../config/model_manifest.json` — model tree confirmed healthy
    before this card touched anything.
  - `./env.sh python scripts/smoke_test.py --dry-run` -> logs the dry-run lines, writes nothing,
    exit=0.
  - `./env.sh python scripts/smoke_test.py --clean ; echo "exit=$?"` (device defaults to cuda) ->
    real nnU-Net subprocess ran, banner:
    ```
    ============================================================
      SMOKE TEST PASSED   (6 assertions x 2 images, 12.3 s)
      predictions: /home/boosterk1/Projects/rebot_crack_vision/data/smoke_test/predictions/
      crack pixels: synthetic_crack 1.19%   synthetic_blank 0.00%
      NOTE: crack-pixel counts are informational only; this test
            verifies execution, not accuracy.
    ============================================================
    ```
    `exit=0`. Per-stage timings printed: generate_fixtures 0.151s, convert_inputs 0.103s,
    nnunet_predict 11.880s, render_synthetic_crack 0.086s, render_synthetic_blank 0.061s.
  - **Exact nnU-Net command line used (pasted from the run log, per acceptance criterion):**
    `nnUNetv2_predict_from_modelfolder -i .../data/smoke_test/nnunet_input -o
    .../data/smoke_test/predictions -m .../models/opencrack-nnunet/Dataset501_OpenCrack/
    nnUNetTrainer__nnUNetPlans__2d -f 0 -chk checkpoint_ep0500.pth -device cuda -npp 3 -nps 3` —
    confirms `-f 0` and `-chk checkpoint_ep0500.pth` are present exactly as mandated.
  - `ls -la data/smoke_test/predictions/` -> `synthetic_blank.png` (334 B), `synthetic_crack.png`
    (2766 B), plus nnU-Net's own copied dataset.json/plans.json/predict_from_raw_data_args.json.
  - `./env.sh python -c "from PIL import Image; import numpy as np; a=np.array(Image.open(
    'data/smoke_test/predictions/synthetic_crack.png')); print('dtype',a.dtype,'shape',a.shape,
    'unique',np.unique(a))"` -> `dtype uint8 shape (512, 512) unique [0 1]` — **pins the {0,1}
    convention, printed in this report as required.** Same check on synthetic_blank.png -> `dtype
    uint8 shape (512, 512) unique [0]`.
  - `ls -la data/smoke_test/overlays/ data/smoke_test/skeletons/` -> `synthetic_crack_mask.png`,
    `synthetic_crack_overlay.png`, `synthetic_blank_mask.png`, `synthetic_blank_overlay.png`,
    `synthetic_crack_skeleton.png`, `synthetic_blank_skeleton.png` all present; opened
    programmatically and confirmed 512x512 for every file (this is also asserted by the script
    itself, which passed).
  - `synthetic_crack`: mask_pixels=3114, skeleton_pixels=595 -> `595 <= 3114` holds.
    `synthetic_blank`: mask_pixels=0, skeleton_pixels=0 -> `0 <= 0` holds. Both printed in the run log
    (`case synthetic_crack: 3114 crack pixels (1.188%), 595 skeleton pixels`, `case synthetic_blank:
    0 crack pixels (0.000%), 0 skeleton pixels`).
  - `nvidia-smi --query-gpu=memory.used --format=csv` -> `178 MiB` before, during-run peak not
    captured, and `178 MiB` again immediately after the run completed — VRAM fully released, no
    leaked process, matching the acceptance criterion.
  - CPU path: `./env.sh python scripts/smoke_test.py --device cpu ; echo "cpu exit=$?"` -> same
    `SMOKE TEST PASSED (6 assertions x 2 images, 18.3 s)` banner, `cpu exit=0`. nnU-Net logged
    `perform_everything_on_device=True is only supported for cuda devices! Setting this to False`
    (expected, upstream's own message, not an error) and `perform_everything_on_device: False`.
    Predictions: `synthetic_crack` dtype=uint8, unique=[0,1], shape=(512,512); `synthetic_blank`
    dtype=uint8, unique=[0], shape=(512,512).
  - `grep -inE "iou|dice|recall" scripts/smoke_test.py` -> only the module docstring's explicit
    statement that these are *never* asserted; no match in any assertion/comparison code — confirms
    the card's most important constraint (no accuracy assertion anywhere).
  - Nothing-written-outside-scope check: `git status --porcelain` before and after both runs shows
    only `scripts/smoke_test.py` as a new tracked-candidate file beyond the pre-existing dirty state
    from session start; `git status --porcelain --ignored` shows `data/smoke_test/{ext_trainer,
    input,nnunet_input,overlays,predictions,skeletons}/` all as `!!` (gitignored, as expected) and
    `data/predictions/`, `data/overlays/`, `data/comparisons/`, `data/skeletons/` (the *real* pipeline
    directories) still hold only their TC-001 `.gitkeep` — confirmed via `ls -la` on all four,
    nothing else present.
  - `find models -newer config/model_manifest.json` -> empty, run immediately after both smoke-test
    invocations — confirms `models/**` was not modified by this card.
  - Re-ran `./env.sh python scripts/verify_model.py --check-hashes` after both smoke-test runs ->
    identical `14 PASS · 1 WARN · 0 FAIL`, exit=0, `check_hashes PASS 6 files match` — model tree
    still intact after two real inference passes.
  - `./env.sh pytest tests/ -v` -> `collected 0 items` / `no tests ran` — no regression (this card
    owns no test file per its own scope; `tests/` still holds only TC-001's `.gitkeep`).
  - Malformed-config exit code: `./env.sh python scripts/smoke_test.py --config
    /tmp/bad_config_smoketest.yaml` (file containing `bad: yaml: [`) -> `smoke_test: malformed YAML
    in /tmp/bad_config_smoketest.yaml: ...`, `exit=2` (scratch file outside the repo, removed after).
ISSUES:
  - **Significant, disclosed deviation — a genuine upstream fact not anticipated by ARCHITECTURE.md
    or the card's own Failure-handling table.** The first run of the real nnU-Net command failed
    (exit 1) with `RuntimeError: Could not find requested nnunet trainer nnUNetTrainerSaveEvery10 in
    nnunetv2.training.nnUNetTrainer`. Diagnosed via
    `torch.load(checkpoint_ep0500.pth, weights_only=False)['trainer_name']` ==
    `"nnUNetTrainerSaveEvery10"`: the checkpoint's own internal metadata (read by nnU-Net's
    `initialize_from_trained_model_folder`, `nnunetv2/inference/predict_from_raw_data.py:85`) names a
    custom trainer class that the OpenCrack authors used during their own training run but never
    published — only the checkpoint and planner-generated `plans.json`/`dataset.json` were released
    (confirmed against `models/opencrack-nnunet/README.md`, which documents "no early stopping" and a
    fixed 500-epoch single-fold budget with the architecture "the one the nnU-Net planner derives" —
    consistent with a trainer subclass whose only special behaviour is checkpoint-saving cadence, not
    network construction). I verified this class does not exist anywhere in the installed nnunetv2
    2.8.1 package (`find .../nnUNetTrainer -iname "*SaveEvery*"` -> no match; only `variants/
    training_length/nnUNetTrainer_Xepochs.py` and `variants/benchmarking/*5epochs*.py` exist, neither
    matching). This is not a bug in my invocation — the model card's own suggested command
    (`nnUNetv2_predict -d 501 -c 2d -f 0 -chk checkpoint_ep0500.pth`) hits the identical
    `initialize_from_trained_model_folder` code path and would fail identically.
    **Fix applied, using nnU-Net's own documented, supported extension point** (confirmed by reading
    `nnunetv2/utilities/find_class_by_name.py`'s `recursive_find_trainer_class_by_name`, and the
    error message itself: *"If the trainer is located elsewhere ... specify the external path via the
    `nnUNet_extTrainer` environment variable"*): `scripts/smoke_test.py` now writes a tiny shim file
    (`class nnUNetTrainerSaveEvery10(nnUNetTrainer): pass` — inherits everything unmodified, overrides
    nothing) to `data/smoke_test/ext_trainer/nnUNetTrainerSaveEvery10.py` at runtime, and sets
    `nnUNet_extTrainer` in the subprocess's environment to that directory. **This never touches
    `models/**`, `src/crackvision/**`, or any file outside `data/smoke_test/`** (this card's own
    allowed-files list) and requires no change to the mandated command line (`-f 0 -chk
    checkpoint_ep0500.pth` etc. are unchanged and unaffected). It is inference-only: no
    `nnUNetv2_train` or optimiser step is ever invoked (adr/010 unaffected). Confirmed to be the
    correct fix, not a masking hack, because `network.load_state_dict(parameters[0])` succeeded
    cleanly on both CUDA and CPU (a real architecture mismatch would have raised a key-mismatch error
    here, not silently produced plausible output) and the model then genuinely detected the synthetic
    crack (1.19% crack pixels on `synthetic_crack`, 0.00% on `synthetic_blank` — informational only,
    not asserted, but strong circumstantial confirmation the network loaded its real trained weights
    correctly rather than random-init fallback weights).
    **This is a new fact for later cards to know:** any future direct use of this checkpoint (e.g. if
    TC-008's `crackvision.inference` module ever bypasses `smoke_test.py`'s command-building and
    reimplements it independently) must also set `nnUNet_extTrainer` or predictions will fail with the
    same RuntimeError. TC-008's own card already says it refactors `smoke_test.py`'s subprocess logic
    into a reusable function — as long as it reuses `run_inference()`/`prepare_ext_trainer_shim()`
    from this file rather than rewriting the command construction from scratch, this is inherited
    automatically.
  - `skimage.morphology.remove_small_objects(binary, min_size=64)` (used in this card's *minimal*
    inline skeleton rendering, exactly as `docs/INTERFACES.md` §3.4 specifies for the real
    `crackvision.skeleton` module TC-010 will build) emits `FutureWarning: Parameter min_size is
    deprecated since version 0.26.0 ... use max_size instead` under the pinned scikit-image 0.26.0.
    Non-fatal, does not affect this card's assertions (all passed), and I did not change the call —
    §3.4's algorithm block says "exactly this, nothing more" (adr/008) and changing the parameter
    name/semantics is TC-010's decision to make, not mine to improvise here. **Flagging for TC-010**:
    the exact literal algorithm in `docs/INTERFACES.md` §3.4 will need to address this deprecation
    (rename to `max_size`, note the semantic difference the warning describes — "or equal to" vs
    "smaller than" — or explicitly pin a warnings filter) when it builds the real module.
  - Confirmed, per TC-005's rev-2 review note, the live hardlink-fixture hazard
    (`.../scratchpad/.../fakeroot` sharing inodes with three model files) was **not** triggered or
    touched by this card — this card only reads `models/**` (via the CLI's `-m` argument) and never
    writes into it; `find models -newer config/model_manifest.json` stayed empty across the whole
    session. I did not attempt to `rm -rf` that external fakeroot myself since it is outside
    `~/Projects/rebot_crack_vision` and outside this card's file list — flagging it again here since
    the hazard is still live per the last review, in case the orchestrator wants a human to clear it
    before further cards touch `models/`.
  - `nnUNetTrainer` version note: `checkpoint_loadable` in `verify_model.py` already WARNs (not
    FAILs) on `weights_only=True` for this exact checkpoint — that WARN is about a different, already
    -documented issue (`numpy._core.multiarray.scalar` in the pickle, TC-005's finding) and is
    unrelated to the trainer-name issue found here, which only surfaces when nnU-Net actually tries to
    *construct* the network from the trainer name, something `verify_model.py` never does (by design
    — it only sanity-checks the checkpoint is a loadable archive, it doesn't run inference).
  - None otherwise.
NEXT CARD: TC-007
```

---

## TC-006 (revision 1) — Synthetic end-to-end smoke test (Level 2 core)

```
TASK: TC-006
STATUS: COMPLETE
CHANGES:
  - scripts/smoke_test.py — revised per the rev-1 Opus review (docs/REVIEW_LOG.md,
    .task_orchestrator/reviews/TC-006-rev1-8d7946ef31e0/{report.md,issues.json}: 1 BLOCKER, 1 MAJOR,
    4 MINOR). Fixed exactly these six findings, nothing else:
    1. [BLOCKER] Added `check_model_folder(cfg)`, called in `main()` right after the CUDA
       precondition check and before any fixture generation or subprocess launch. It checks
       `plans.json`, `dataset.json`, and `fold_<fold>/<checkpoint>` under the model dir (same
       derivation as `build_predict_command`); if any is missing it logs each missing path, logs
       "run: ./env.sh python scripts/fetch_model.py (TC-004), then: ./env.sh python
       scripts/verify_model.py (TC-005)", writes a `"precondition"` summary, and returns
       EXIT_PRECONDITION (3) — for both `--device cuda` and `--device cpu`, before nnU-Net's own
       FileNotFoundError can surface as an exit-1 traceback. The mandated `-f 0`/`-chk
       checkpoint_ep0500.pth` command line is unchanged; nothing is downloaded.
    2. [MAJOR] Moved the `--dry-run` guard above the `--clean` block (previously `--clean`'s
       `shutil.rmtree` ran unconditionally before the dry-run check, so `--dry-run --clean` deleted
       real output directories). `--dry-run --clean` now only logs
       "dry-run: would remove <dirs>" and deletes nothing; plain `--clean` (no dry-run) still
       actually removes the directories. Moved `import shutil` to the module-level import block.
    3. [MINOR] `generate_fixtures` now draws one noisy base from a single seeded
       `np.random.default_rng(1234)` call, reuses it untouched as `synthetic_blank`, and draws the
       polyline onto a *copy* of it for `synthetic_crack` (a second, separately seeded generator
       supplies the polyline jitter so its geometry stays reproducible independent of the base
       generator's consumed stream). `_draw_blank` removed; `_draw_crack(base, rng)` now takes the
       shared base explicitly. Seed 1234, image size, base grey level, noise sigma, polyline
       endpoints/branch/width and the 0.6-radius Gaussian blur are all unchanged.
    4. [MINOR] Split the per-case prediction check out of the main loop into a new
       `evaluate_case(...)` function: it now reads the prediction and asserts dtype ==
       uint8 / unique-values ⊆ {0,1} / shape == the actual original fixture's (height, width)
       *before* calling `render_visuals()`; a failure short-circuits (returns `result=None`) and
       skips rendering entirely, instead of letting a malformed prediction crash inside the overlay
       blend. `render_visuals()` now takes the already-loaded `pred` ndarray as a parameter instead
       of re-reading the file, and no longer computes dtype/unique/shape itself (the caller sets
       those on the result dict). The shape check compares against the real fixture's own
       `Image.open(original_path).height/.width`, not the `IMAGE_SIZE` literal.
    5. [MINOR] `--clean`'s directory list now includes `ext_trainer/` alongside
       `input/nnunet_input/predictions/overlays/skeletons`, so a stale shim `__pycache__` cannot
       outlive a change to `_EXT_TRAINER_SOURCE`. `prepare_ext_trainer_shim()` already rewrites the
       shim unconditionally on every run, so removing the directory first is safe.
    6. [MINOR] The `n_assertions_per_image` banner value is no longer a hardcoded `6`; each call to
       `evaluate_case` now returns the number of assertions it actually ran for that case
       (`checks`, incremented once per assertion evaluated: dtype, unique-values, shape, then — if
       those pass — mask/overlay/skeleton existence+size, then skeleton≤mask), collected into
       `assertions_per_case`, and the banner prints `max(assertions_per_case.values())` — the true
       count is 7, not 6 (dtype, unique, shape, 3×file-check, skeleton≤mask), which the hardcoded
       literal had silently undercounted. `--skip-existing` remains an accepted no-op (see ISSUES),
       matching TC-003/TC-004's disclosed precedent rather than inventing new semantics here.
  - task_cards/TASK_INDEX.md — TC-006 row set back to COMPLETE (was flipped to IN PROGRESS by the
    orchestrator for this revision). No other rows changed.
  - docs/COMPLETION_LOG.md — this entry.
TESTS:
  - `./env.sh python -m py_compile scripts/smoke_test.py` -> compiles cleanly.
  - **Finding 1 (BLOCKER) — model-missing precondition, exit 3, no traceback:**
    `./env.sh python scripts/smoke_test.py --root <scratch-root-with-only-config/project.yaml>
    --device cpu` ->
    ```
    ERROR   smoke_test: model file missing: <root>/models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json
    ERROR   smoke_test: model file missing: <root>/models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/dataset.json
    ERROR   smoke_test: model file missing: <root>/models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/fold_0/checkpoint_ep0500.pth
    ERROR   smoke_test: run: ./env.sh python scripts/fetch_model.py (TC-004), then: ./env.sh python scripts/verify_model.py (TC-005)
    ```
    `exit=3`, no Python traceback anywhere in the output. `<root>/logs/smoke_test_latest.json` ->
    `"status": "precondition", "exit_code": 3`, `errors` names all three missing paths. Re-verified
    the model-present path is unaffected below (both `--device cuda` and `--device cpu` still PASS).
  - **Finding 2 (MAJOR) — `--dry-run --clean` no longer destructive:** created
    `data/smoke_test/input/SENTINEL` and `data/smoke_test/ext_trainer/SENTINEL_EXT`, then
    `./env.sh python scripts/smoke_test.py --dry-run --clean` -> logged
    `"dry-run: would remove <root>/data/smoke_test/{input,nnunet_input,predictions,overlays,skeletons,ext_trainer}"`,
    `exit=0`; `ls -a data/smoke_test/ data/smoke_test/input data/smoke_test/ext_trainer` afterwards
    showed every directory **and both sentinels** still present — nothing was deleted. Then
    `./env.sh python scripts/smoke_test.py --clean --device cuda` (no `--dry-run`) -> real run,
    `SMOKE TEST PASSED (7 assertions x 2 images, 12.4 s)`, `exit=0`; `ls -a` afterwards showed both
    sentinels **gone** (real `--clean` still actually cleans, including the previously-missed
    `ext_trainer/__pycache__` — finding 5).
  - **Finding 3 (MINOR) — fixture reproducibility and matched-pair noise:**
    Two independent in-process calls to `generate_fixtures()` into separate temp dirs, and two
    on-disk `--clean`-free regenerations into `data/smoke_test/input/`, all produced byte-identical
    files: `synthetic_crack.png` md5 `b64873df0e19524292613851b1e1d410` (both runs),
    `synthetic_blank.png` md5 `447a8a2371f32ad108f91930feb7a26a` (both runs). Matched-pair check:
    `diff = |crack - blank|` outside a 4px-dilated "diff>30" crack mask has `max=30, mean=8.73`;
    applying the *same* 0.6-radius Gaussian blur directly to the shared unblurred base (no polyline
    at all) gives `max=31, mean=8.73` against that base — i.e. the entire background difference
    between the two fixtures is explained by the crack image's own blur smoothing the *same* shared
    noise, not by a different noise draw. The two fixtures are a genuine matched pair.
  - **Finding 4 (MINOR) — malformed prediction fails cleanly, no crash:** unit-level exercise of
    `evaluate_case()` (docs/REVIEW_LOG.md accepts this form of proof) with a hand-written 256×256
    uint8 all-zero PNG placed at `<tmp>/predictions/synthetic_crack.png` against the real 512×512
    `synthetic_crack.png` fixture as the original ->
    `failures=['synthetic_crack: prediction shape (256, 256) != original image shape (512, 512)']`,
    `result is None` (rendering skipped, no exception raised), `checks=3`. No traceback.
  - **Finding 5 (MINOR) — `--clean` now covers `ext_trainer/`:** demonstrated together with finding
    2's test above (`ext_trainer/SENTINEL_EXT` and its old `__pycache__` were both removed by a real
    `--clean`, and the directory + shim were correctly regenerated by the same run).
  - **Finding 6 (MINOR) — assertion count is computed, not literal:** both the CUDA and CPU full
    runs below print `(7 assertions x 2 images, ...)`, not the old hardcoded `6`; confirmed by
    reading `main()`'s use of `max(assertions_per_case.values(), default=0)` fed by
    `evaluate_case`'s real per-case `checks` counter (3 pre-render + 3 file checks + 1
    skeleton≤mask = 7 on the success path). `--skip-existing`: `grep -n "skip_existing"
    scripts/smoke_test.py` -> no match outside `add_common_args` itself — confirmed still an
    accepted no-op (see ISSUES), not silently broken.
  - **Full re-run of all ten original acceptance criteria on the revised file:**
    `./env.sh python scripts/smoke_test.py --clean --device cuda` ->
    ```
    ============================================================
      SMOKE TEST PASSED   (7 assertions x 2 images, 12.3 s)
      predictions: /home/boosterk1/Projects/rebot_crack_vision/data/smoke_test/predictions/
      crack pixels: synthetic_crack 1.75%   synthetic_blank 0.00%
      NOTE: crack-pixel counts are informational only; this test
            verifies execution, not accuracy.
    ============================================================
    ```
    `exit=0`. Exact nnU-Net command line from the log: `nnUNetv2_predict_from_modelfolder -i
    .../data/smoke_test/nnunet_input -o .../data/smoke_test/predictions -m
    .../models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d -f 0 -chk
    checkpoint_ep0500.pth -device cuda -npp 3 -nps 3` — `-f 0` and `-chk checkpoint_ep0500.pth`
    present exactly as mandated.
  - `./env.sh python -c "from PIL import Image; import numpy as np; a=np.array(Image.open(
    'data/smoke_test/predictions/synthetic_crack.png')); print(a.dtype, a.shape, np.unique(a))"` ->
    `uint8 (512, 512) [0 1]`. Same check on `synthetic_blank.png` -> `uint8 (512, 512) [0]`.
  - `./env.sh python scripts/smoke_test.py --device cpu` (no `--clean`, fixtures regenerated
    in-place) -> `SMOKE TEST PASSED (7 assertions x 2 images, 18.3 s)`, `cpu exit=0`. Same
    dtype/shape/unique values as the CUDA run.
  - `nvidia-smi --query-gpu=memory.used --format=csv,noheader` -> `178 MiB` before, `178 MiB`
    immediately after both the CUDA and CPU runs — VRAM fully released, nothing leaked.
  - Mask/overlay/skeleton sizes: `Image.open(...).size` -> `(512, 512)` for all six files
    (`synthetic_{crack,blank}_{mask,overlay}.png`, `synthetic_{crack,blank}_skeleton.png`).
    Pixel counts: `synthetic_crack` mask_pixels=4577, skeleton_pixels=657 (`657 <= 4577` holds);
    `synthetic_blank` mask_pixels=0, skeleton_pixels=0 (`0 <= 0` holds).
  - `grep -inE "iou|dice|recall" scripts/smoke_test.py` -> only the module docstring's explicit
    statement that these are never asserted; no match in any assertion/comparison code.
  - `grep -n "/home/" scripts/smoke_test.py` -> no match. `grep -n "shell=True"` -> only inside a
    docstring sentence. `grep -n "weights_only"` -> only inside the trainer-shim explanatory
    comment (this script never touches the checkpoint directly).
  - `./env.sh python scripts/verify_model.py --check-hashes` (re-run after every real inference
    pass above) -> `14 PASS · 1 WARN · 0 FAIL`, exit=0, `check_hashes PASS 6 files match` — model
    tree still intact; `find models -newer config/model_manifest.json` -> empty.
  - `git status --porcelain` -> only `scripts/smoke_test.py` (mine), plus the two pre-existing dirty
    files from session start (`config/model_manifest.json`, `task_cards/TASK_INDEX.md`, the latter
    now correctly re-set to COMPLETE by me). `git status --porcelain --ignored | grep smoke_test`
    shows only gitignored `data/smoke_test/**` and `logs/smoke_test_*` — nothing written outside
    `data/smoke_test/` and `logs/`. `data/predictions/`, `data/overlays/`, `data/comparisons/`,
    `data/skeletons/`, `data/input_originals/` all still hold only their TC-001 `.gitkeep`.
  - `./env.sh pytest tests/ -v` -> `collected 0 items` / `no tests ran` — unchanged, no regression
    (this card owns no test file).
ISSUES:
  - `--skip-existing` remains an accepted no-op in this file (accepted by `add_common_args`, has no
    effect anywhere in `smoke_test.py`) — this is the rev-1 review's own suggested resolution
    ("either implement meaningfully or record as an accepted no-op, matching how TC-003/TC-004
    disclosed the same situation"). Implementing real skip-existing semantics for a two-fixture,
    sub-second-to-generate synthetic smoke test would add complexity with no practical benefit and
    was not requested by any of the six findings, so it is disclosed here rather than built.
  - `skimage.morphology.remove_small_objects(binary, min_size=64)` still emits the same
    `FutureWarning` about `min_size` -> `max_size` under scikit-image 0.26.0 noted in the original
    TC-006 completion report. Unchanged and out of scope for this revision (none of the six findings
    touch it; it remains flagged for TC-010, which owns the real `crackvision.skeleton` module).
  - The `nnUNet_extTrainer` shim from the original submission (checkpoint's embedded `trainer_name`
    is `nnUNetTrainerSaveEvery10`, not shipped with nnunetv2 2.8.1) is unchanged and was already
    independently verified and accepted by the rev-1 review; not touched in this revision beyond
    ensuring `--clean` now also removes `ext_trainer/` between runs (finding 5).
  - `config/model_manifest.json` shows as modified in `git status` (only `downloaded_utc` differs
    from the committed value) — this predates this session (confirmed via `git diff`, matches the
    state already disclosed in the TC-005 rev-2 and original TC-006 completion reports) and was not
    touched or committed by this revision.
  - The live hardlink hazard from TC-004's review scratch tree, flagged again in the rev-1 review,
    was not touched by this card (read-only access to `models/**` throughout, confirmed by `find
    models -newer config/model_manifest.json` staying empty across every run in this session).
  - None otherwise.
NEXT CARD: TC-007
```

---

## TC-007 — Input preparation utility and the naming contract

```
TASK: TC-007
STATUS: COMPLETE
CHANGES:
  - src/crackvision/naming.py — new. Pure functions, no I/O: `derive_case_id` (the sanitisation
    rule from docs/INTERFACES.md §1.1, applied via two compiled regexes), `assign_case_ids`
    (collision resolution in `sorted(sources, key=str)` order, first occurrence bare, later ones
    `__2`/`__3`/…), plus the eight path helpers from §1.2 (`nnunet_input_path`, `prediction_path`,
    `mask_path`, `overlay_path`, `comparison_path`, `skeleton_path`, `skeleton_overlay_path`,
    `skeleton_stats_path`), each returning the literal `data/<subdir>/{case}<suffix>` relative
    `Path` from the table, independent of `crackvision.config` (matches the component table's
    "Reads —, Writes —" contract for the naming module).
  - src/crackvision/prepare_inputs.py — new. CLI + `main()` per docs/INTERFACES.md §3.1: discovers
    supported-extension files non-recursively in `--input-dir` (default `cfg.paths["input_originals"]`),
    assigns case_ids via `naming.assign_case_ids`, then per file: `Image.open` -> `img.load()` (so a
    truncated file raises here, not deep in the pipeline) -> `ImageOps.exif_transpose(img) or img`
    (guarded per the card's own noted Pillow-version gotcha) -> `.convert("RGB")` (mandatory,
    unconditional) -> optional `--max-side` LANCZOS downscale (off by default, records
    `downscaled`/`scale_factor`) -> `img.save(dst, format="PNG", compress_level=6)`. A single
    corrupt/unreadable file is caught, logged at ERROR, counted in `counts.failed`, and the batch
    continues (no case_map entry is written for it). Writes `data/case_map.json` in full every run
    (schema_version 1, `generated_utc`, absolute `root`, `cases` sorted by `case_id`, project-relative
    POSIX `source_path`/`nnunet_input`, sha256 of both source and output). `--dry-run` writes nothing;
    `--clean` deletes existing `*.png` in the output dir first; `--skip-existing` decodes+converts
    in-memory for accurate case_map metadata but skips the `save()` call when the destination already
    exists. Exit 3 (naming the directory + accepted extensions) when zero supported files are found;
    exit 1 (`status: "partial"`) when `counts.failed > 0`; exit 0 otherwise. No hard-coded `/home/`
    path (grep-verified).
  - tests/conftest.py — new. `tmp_project` fixture: copies the real `config/project.yaml` into a
    pytest `tmp_path`, calls `load_config(root=tmp_path)`, creates every `cfg.paths` directory, and
    returns the `Config` — no test ever reads or writes this repo's real `data/`.
  - tests/test_naming.py — new. Every single-file row of the docs/INTERFACES.md §1.1 worked-example
    table, parametrised; collision-suffix ordering (`a b.png`/`a_b.png` -> `a_b`/`a_b__2`); a
    three-way collision proving sorted-path order (`__2`, `__3`); order-independence of
    `assign_case_ids`; the empty-input case; and all eight path helpers. Two of the table's
    multi-file rows are implemented per the *algorithm* rather than per their own literal answer
    column, which contradicts itself — see ISSUES below; both are documented in the test file's
    module docstring and at each assertion site, not silently "corrected" without a trace.
  - tests/test_prepare_inputs.py — new. In-test PIL fixtures for every required mode (RGB JPEG,
    RGBA PNG, grayscale `L` PNG, palette `P` PNG, CMYK TIFF, 16-bit `I;16` PNG via
    `Image.new("I;16", size).frombytes(...)` to avoid a Pillow 13 deprecation on the `fromarray(...,
    mode=...)` path, EXIF-orientation-6 JPEG, BMP) all asserted to come out `mode=="RGB"`,
    `format=="PNG"`, 3-channel uint8; the EXIF fixture asserted upright with width/height swapped;
    `data/case_map.json` schema/sort-order/relative-POSIX-path/sha256 round-trip; `--dry-run` writes
    zero files; `--clean` removes a manually-planted stale output; re-running twice yields identical
    `sha256_nnunet_input`; `--skip-existing` leaves the existing file's mtime untouched and reports
    it under `counts.skipped`, not `counts.converted`; a zero-byte file plus a half-truncated real
    PNG alongside one good file yields `counts.failed == 2`, `counts.converted == 1`, `status:
    "partial"`, exit 1, and the good case still lands in `case_map.json`; empty and missing input
    dirs both exit 3; `data/input_originals/` content-hash is unchanged after a run (including one
    that passes `--max-side`); `--max-side` downscales and records `scale_factor`; an unsupported
    extension (`.txt`) is excluded from `counts.found` without error.
TESTS:
  - `./env.sh python -m py_compile src/crackvision/naming.py src/crackvision/prepare_inputs.py
    tests/conftest.py tests/test_naming.py tests/test_prepare_inputs.py` -> compiles cleanly.
  - `./env.sh pytest tests/test_naming.py tests/test_prepare_inputs.py -v` -> **37 passed** (8
    parametrised case-id-table rows + 7 more naming tests + 8 path-helper rows + 15 prepare_inputs
    tests), `0.35s`. Full transcript included every individual test name, all `PASSED`.
  - `./env.sh pytest tests/ -v` -> same **37 passed** (no other test files exist yet in `tests/` —
    TC-001 through TC-006 own no test files of their own), `0.31s`.
  - `./env.sh python -m crackvision.prepare_inputs --help` -> prints full usage: `--root --config -v
    -q --dry-run --skip-existing --input-dir --output-dir --max-side --clean`, exit 0.
  - `./env.sh python -m crackvision.prepare_inputs` (real `data/input_originals/` empty except
    `.gitkeep`) -> `ERROR ... no supported images found in .../data/input_originals (accepted
    extensions: .bmp, .jpeg, .jpg, .png, .tif, .tiff)`, `empty-dir exit=3 (expect 3)` -> **matched**.
  - Real end-to-end run, per the card's own command list: copied
    `data/smoke_test/input/{synthetic_blank,synthetic_crack}.png` (TC-006's leftover fixtures) into
    `data/input_originals/`, then `./env.sh python -m crackvision.prepare_inputs` -> `exit=0`, logged
    `converted .../synthetic_blank.png -> .../synthetic_blank_0000.png (RGB PNG -> RGB, 512x512)` and
    the same for `synthetic_crack`, `wrote .../data/case_map.json (2 case(s))`.
    `ls data/nnunet_input/` -> `synthetic_blank_0000.png synthetic_crack_0000.png`.
    `python3 -m json.tool data/case_map.json` -> valid JSON matching the §2 schema exactly (both
    entries: `schema_version:1`, absolute `root`, `source_path`/`nnunet_input` as project-relative
    POSIX strings, `source_mode:"RGB"`, `source_format:"PNG"`, `converted:true`, `downscaled:false`,
    `scale_factor:1.0`, matching `sha256_source`/`sha256_nnunet_input` — the synthetic PNGs were
    already RGB, so source and output hashes are identical, as expected for a lossless no-op
    re-encode).
    `./env.sh python -c "from PIL import Image; import glob; ... assert im.mode=='RGB' and
    im.format=='PNG' ..."` -> `all RGB PNG OK`.
    `sha256sum data/input_originals/*.png` recorded before cleanup for the idempotency/read-only
    checks below.
    `python3 -m json.tool logs/prepare_inputs_latest.json` -> `status:"ok"`, `exit_code:0`,
    `counts:{"found":2,"converted":2}`.
    `grep -n "/home/" src/crackvision/naming.py src/crackvision/prepare_inputs.py tests/conftest.py
    tests/test_naming.py tests/test_prepare_inputs.py` -> no match (exit 1 from grep, i.e. clean).
  - Idempotency re-run (same real fixtures, no `--clean`): `./env.sh python -m
    crackvision.prepare_inputs` again -> `exit=0`; `sha256sum data/nnunet_input/*.png` -> **identical
    hashes** to the first run for both files.
  - Cleanup: removed the copied `data/input_originals/{synthetic_blank,synthetic_crack}.png`, the
    generated `data/nnunet_input/*_0000.png`, and `data/case_map.json` after the demonstration above,
    restoring `data/input_originals/` and `data/nnunet_input/` to `.gitkeep`-only (this repo's `data/`
    tree is entirely gitignored runtime scratch, not a tracked fixture — see docs/ARCHITECTURE.md §5
    source-control policy — so this is housekeeping for the next card's session, not a git operation).
    `ls -la data/input_originals/ data/nnunet_input/` after cleanup -> only `.gitkeep` in each.
  - `git status --porcelain` before committing -> exactly the 5 new files this card owns
    (`src/crackvision/naming.py`, `src/crackvision/prepare_inputs.py`, `tests/conftest.py`,
    `tests/test_naming.py`, `tests/test_prepare_inputs.py`) plus this entry's edit to
    `docs/COMPLETION_LOG.md` and the status-board edit to `task_cards/TASK_INDEX.md`, on top of the
    pre-existing dirty state (`config/model_manifest.json`, the untracked planning `docs/*.md` /
    `docs/adr/` / `task_cards/TC-0NN-*.md` / `task_cards/AGENT_INSTRUCTIONS.md`) that was already
    present before this session started and is not owned by this card.
ISSUES:
  - **Two rows of the docs/INTERFACES.md §1.1 worked-example table are internally inconsistent; both
    were implemented per the normative algorithm (verified computationally against every other row
    in the same table, which all check out), not per the table's own literal answer text.** Neither
    is a redesign — the algorithm itself is unambiguous and is spelled out twice (INTERFACES.md §1.1
    and the task card body), and it is not something I invented a fix for; I just followed it where
    two of its own worked examples disagree with themselves:
    1. `A.png` then `a!.png` — the answer column prints "`A`, then `a__2`", but the same cell's own
       parenthetical says "distinct stems → no collision: `A` vs `a`". The algorithm has no
       case-folding step anywhere, so `A.png` → `A` and `a!.png` → `a` are two different strings and
       there is, in fact, no collision — the parenthetical is correct and the leading answer text
       is not (it reads like a copy/paste leftover from the actual-collision row two lines below,
       `a b.png`/`a_b.png` → `a_b`/`a_b__2`). Implemented and tested as: `A.png` → `A`, `a!.png` →
       `a`, no suffix on either.
    2. `x y.png` then `x-y.png` then `x_y.png` — the table says "no collision", but the normative
       sanitisation rule maps both the space in `x y.png` and the underscore in `x_y.png` to `_`, so
       both stems sanitise to the identical string `x_y` — that is the literal definition of a
       collision this same section gives two rows earlier. `x-y.png` is untouched (`-` is an
       allowed character) and stays `x-y`. In `sorted(str)` order the three filenames sort as
       `x y.png` < `x-y.png` < `x_y.png` (space 0x20 < hyphen 0x2D < underscore 0x5F), so the
       deterministic result is `x y.png` → `x_y` (bare, first `x_y`-producer), `x-y.png` → `x-y`
       (never collided), `x_y.png` → `x_y__2` (second `x_y`-producer). Implemented and tested exactly
       this way, with the reasoning recorded at the point of use in `tests/test_naming.py`'s module
       docstring and at each assertion.
    I did not treat this as a BLOCKER per AGENT_INSTRUCTIONS.md rule 4 — the algorithm is not
    "impossible", it is fully specified and testable, and only two illustrative example cells
    disagree with their own stated rule. Flagging here per rule 3 ("write it under ISSUES: — do not
    fix it") since fixing the *documentation* is out of this card's file-ownership list
    (`docs/INTERFACES.md` is planning-session-owned, "nobody" may modify it per
    `task_cards/TASK_INDEX.md` §File ownership).
  - `naming.py`'s eight path helpers return fixed literal `data/<subdir>/...` relative paths, per
    docs/INTERFACES.md §1.2's own literal table, independent of `crackvision.config`. This means a
    future card's `--output-dir`/config override for a given stage would not automatically flow
    through `naming.<x>_path()` — `prepare_inputs.py` itself does not have this problem (it builds
    its actual output path from `args.output_dir` and only borrows `nnunet_input_path(case).name` for
    the filename suffix), but a later consumer (TC-009/TC-010) that called e.g.
    `naming.prediction_path(case)` directly and expected it to honour a non-default `--output-dir`
    would need to resolve it against the same directory it configured separately. This is exactly
    what the component table specifies (naming.py: "Reads —, Writes —", pure functions with no cfg
    dependency) and is not something this card's scope covers changing — flagging it only so TC-008
    (which resolves `data/nnunet_input` for `-i`) and TC-009/TC-010 (which build predictions/overlay
    paths) are aware their own CLI's `--output-dir`/`--input-dir` flag, not `naming.py`, is the source
    of truth for the actual runtime location.
  - `PIL.Image.fromarray(arr, mode="I;16")` (the natural way to build a 16-bit test fixture) emits a
    `DeprecationWarning` under Pillow 12.3.0 ("'mode' parameter for changing data types is deprecated
    and will be removed in Pillow 13 (2026-10-15)"). Used `Image.new("I;16", size).frombytes(...)`
    instead in `tests/test_prepare_inputs.py`, which round-trips through a real PNG save/reopen with
    no warning — confirmed with `warnings.simplefilter("error")` before committing to that approach.
    Not an issue in `prepare_inputs.py` itself (which never constructs an `I;16` image, only opens
    ones already on disk), but worth noting for any future card that builds synthetic 16-bit fixtures.
  - `config/model_manifest.json` continues to show as modified in `git status` (pre-existing dirty
    state carried from TC-005's repair; not touched by this session — confirmed untouched throughout).
  - None otherwise.
NEXT CARD: TC-008
```
