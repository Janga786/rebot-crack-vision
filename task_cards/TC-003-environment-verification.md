# TC-003 — Environment verification script (Level 1 smoke test)

**Task ID:** TC-003 · **Complexity:** SMALL–MEDIUM · **Prerequisites:** TC-001, TC-002

## Objective
Build `scripts/check_env.py`: a single command that reports PASS/WARN/FAIL for every property this
project depends on, and writes a machine-readable reproducibility snapshot.

## Why this matters
Every later card starts by running this. It converts "something is broken somewhere" into a named
row, and it is the automated guard that the `env.sh` scrub has not regressed (`RISKS.md` R-01).

## Prerequisites
TC-002 COMPLETE — `./env.sh python -c "import torch, nnunetv2"` works.

## Scope
Implement `scripts/check_env.py` per `docs/INTERFACES.md` §3.5.

## Out of scope
- Fixing anything it finds. This script **diagnoses only** — it never installs, creates, or repairs.
- Downloading the model.
- Any RealSense probing beyond "does `import pyrealsense2` work" (TC-012 does the real probe).

## Files to create
`scripts/check_env.py`

## Files allowed to modify
`docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`.

## Files that must NOT be modified
Everything else, especially `src/crackvision/config.py` and `logging_setup.py` (TC-001 owns them —
if you need a change there, report it, do not make it).

## Implementation requirements

`./env.sh python scripts/check_env.py [--json] [--strict]`

Implement a small `Check` record (`name`, `status` ∈ `PASS|WARN|FAIL`, `value`, `hint`) and a list of
check functions. **Every check must be individually exception-safe** — one failure must not abort the
report. Wrap each in try/except and turn an unexpected exception into a `FAIL` with the exception
text as the value.

### Required checks, in this order

| # | Name | PASS when | On failure, `hint` should say |
|---|---|---|---|
| 1 | `python_version` | `sys.version_info[:2] == (3, 11)` | "expected 3.11 from the crackvision env; did you use ./env.sh?" |
| 2 | `pythonpath_clean` | no `/opt/ros` in `os.environ.get("PYTHONPATH","")` | "run via ./env.sh — ROS Humble leaks py3.10 paths (adr/007)" |
| 3 | `syspath_clean` | no `ros/humble` in any `sys.path` entry | same |
| 4 | `ld_library_path_clean` | no `/opt/ros` or `cuda-11.8` entry | "env.sh should filter these" — **WARN**, not FAIL |
| 5 | `torch_import` | imports | "run TC-002" |
| 6 | `torch_version` | `>= 2.1.2` and not `2.9.*` | nnunetv2 constraint |
| 7 | `torch_cuda_build` | `torch.version.cuda` is not None and `+cu` in version | "CPU-only wheel installed; reinstall from the cu126 index" |
| 8 | `cuda_available` | `torch.cuda.is_available()` | **WARN** not FAIL — CPU inference is supported |
| 9 | `gpu_name` | device 0 name reported | WARN if no CUDA |
| 10 | `gpu_memory` | report total **and free** VRAM in GiB via `torch.cuda.mem_get_info()` | WARN below 6 GiB free: "other projects may be using the GPU; see ARCHITECTURE.md §9" |
| 11 | `nvidia_driver` | parsed from `nvidia-smi --query-gpu=driver_version --format=csv,noheader` | WARN if `nvidia-smi` absent |
| 12 | `nnunetv2_import` | imports; report `version("nnunetv2")` | "run TC-002" |
| 13 | `nnunet_cli` | `shutil.which("nnUNetv2_predict_from_modelfolder")` is not None | "the console script is missing from the env" |
| 14 | `imaging_stack` | numpy, skimage, PIL, cv2, yaml all import; report versions | |
| 15 | `pyrealsense2` | imports; report version | **WARN** only: "optional — `pip install -r requirements/requirements-realsense.txt`" |
| 16 | `project_paths` | every path in `cfg` exists and is writable | "run TC-001" |
| 17 | `nnunet_env_vars` | all three set **and** `str(path).startswith(str(root))` | "run via ./env.sh (adr/004)" |
| 18 | `model_checkpoint` | `models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/fold_0/checkpoint_ep0500.pth` exists, size within 250–300 MB | "run `./env.sh python scripts/fetch_model.py` (TC-004)" — message must distinguish *not yet fetched* from *corrupt* |
| 19 | `model_config_files` | `plans.json` and `dataset.json` exist alongside | same |
| 20 | `disk_free` | `shutil.disk_usage(root).free` reported in GiB | **WARN** below 20 GiB |

### Output

Human table to stdout, aligned, e.g.:
```
CHECK                   STATUS  VALUE
python_version          PASS    3.11.9
pythonpath_clean        PASS    (no ROS entries)
...
model_checkpoint        FAIL    missing
   hint: run ./env.sh python scripts/fetch_model.py (TC-004)

18 PASS · 2 WARN · 1 FAIL
```
`--json` prints the JSON instead of the table (still writes the file).

Always write `logs/check_env_latest.json` (and the timestamped twin) via
`crackvision.logging_setup.RunSummary`, with a `checks` array of every `Check` plus a `versions`
object holding every version string gathered. This file is the reproducibility snapshot TC-016 uses.

Exit `0` if no FAIL, `3` if any FAIL. `--strict` promotes WARN → FAIL.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh python scripts/check_env.py ; echo "exit=$?"
./env.sh python scripts/check_env.py --json | python3 -m json.tool | head -30
cat logs/check_env_latest.json | python3 -m json.tool | head -20

# prove check 2/3 actually detect a dirty environment:
PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages \
  ~/miniconda3/envs/crackvision/bin/python scripts/check_env.py ; echo "dirty exit=$?"
# expect: pythonpath_clean FAIL, exit=3
```

## Acceptance criteria
- [ ] All 20 checks are implemented and each appears in the output.
- [ ] Run via `./env.sh`: `pythonpath_clean` and `syspath_clean` are **PASS**.
- [ ] Run with a deliberately poisoned `PYTHONPATH` (bypassing `env.sh`): `pythonpath_clean` is
      **FAIL** and the exit code is **3**. *(This proves the guard works, not just that it exists.)*
- [ ] Before TC-004 runs, `model_checkpoint` is `FAIL` with a hint naming `fetch_model.py`, and the
      script still completes and prints the full table (no crash).
- [ ] `--strict` turns a WARN-only run into exit 3.
- [ ] `logs/check_env_latest.json` exists, is valid JSON, and contains `checks` and `versions`.
- [ ] A single failing check never aborts the report — verify by temporarily renaming nothing;
      instead confirm by code inspection that every check is wrapped in try/except.
- [ ] No hard-coded `/home/` path in the script.

## Failure handling
- **`nvidia-smi` missing or errors** → WARN, keep going.
- **`torch.cuda.mem_get_info()` unavailable** on this torch version → fall back to
  `torch.cuda.get_device_properties(0).total_memory` and report free as `unknown` (WARN).
- **`importlib.metadata.version("nnunetv2")` raises** → fall back to `nnunetv2.__version__` if
  present, else report `"unknown"` and still PASS on the import itself.
- **Model not yet downloaded** → this is the *expected* state when this card runs. FAIL rows 18–19
  are correct; do not weaken them, and do not download the model to make them green.

## Documentation update
Append the completion report plus the **full check table output** to `docs/COMPLETION_LOG.md`.
Update `TASK_INDEX.md`: TC-003 → `COMPLETE`.

## Commit guidance
`feat: add environment verification script (level 1 smoke test)`

## Completion report format
```
TASK: TC-003
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-004
```
