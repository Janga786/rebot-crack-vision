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
