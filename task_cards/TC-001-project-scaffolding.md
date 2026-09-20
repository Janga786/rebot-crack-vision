# TC-001 — Project scaffolding and Git hygiene

**Task ID:** TC-001 · **Complexity:** SMALL · **Prerequisites:** None

## Objective
Create the full directory tree, `.gitignore`, `pyproject.toml`, `config/project.yaml`, the shared
`crackvision.config` / `crackvision.logging_setup` modules, and initialise the git repository.

## Why this matters
Every later card writes into this tree and imports these two shared modules. Getting `.gitignore`
right now is what prevents a 268 MB checkpoint from being committed to a repo that has no git-lfs.

## Prerequisites
None. `docs/` and `task_cards/` already exist from the planning session — **do not delete or
rewrite anything in them.**

## Scope
1. Create the directory tree from `docs/ARCHITECTURE.md` §5, with `.gitkeep` in every directory that
   would otherwise be empty.
2. Write `.gitignore`.
3. Write `pyproject.toml` (project metadata + setuptools config exposing `src/crackvision`).
4. Write `config/project.yaml` **exactly** as given in `docs/INTERFACES.md` §4.
5. Write `src/crackvision/__init__.py` (version string only).
6. Write `src/crackvision/config.py`.
7. Write `src/crackvision/logging_setup.py`.
8. `git init -b main`, first commit.

## Out of scope
- Creating the conda environment or installing anything (TC-002).
- `env.sh` (TC-002).
- `naming.py` (TC-007) — do not create it here.
- Any pipeline module, script, or test.
- Downloading the model.
- Changing `init.defaultBranch` globally, or any other global git config.

## Files to create
```
.gitignore
pyproject.toml
config/project.yaml
src/crackvision/__init__.py
src/crackvision/config.py
src/crackvision/logging_setup.py
README.md                     (short placeholder; TC-016 writes the real one)
docs/COMPLETION_LOG.md        (header + your first entry)
data/input_originals/.gitkeep
data/nnunet_input/.gitkeep
data/predictions/.gitkeep
data/overlays/.gitkeep
data/comparisons/.gitkeep
data/skeletons/.gitkeep
data/smoke_test/.gitkeep
data/d405/color/.gitkeep
data/d405/depth/.gitkeep
data/d405/metadata/.gitkeep
models/.gitkeep
nnunet/raw/.gitkeep
nnunet/preprocessed/.gitkeep
nnunet/results/.gitkeep
logs/.gitkeep
scripts/.gitkeep
tools/.gitkeep
tests/.gitkeep
requirements/.gitkeep
```

## Files allowed to modify
Only the files above.

## Files that must NOT be modified
`docs/**` (except creating `COMPLETION_LOG.md`), `task_cards/**`, anything outside
`~/Projects/rebot_crack_vision`, `~/.bashrc`, any conda env.

## Implementation requirements

### `.gitignore`
```gitignore
# Model weights and nnU-Net roots — 268 MB checkpoint, and git-lfs is NOT installed here
models/*
!models/.gitkeep
nnunet/raw/*
nnunet/preprocessed/*
nnunet/results/*
!nnunet/raw/.gitkeep
!nnunet/preprocessed/.gitkeep
!nnunet/results/.gitkeep

# Data — imagery is never committed
data/**
!data/**/
!data/**/.gitkeep

# Logs
logs/*
!logs/.gitkeep

# Generated model manifest holds hashes only, but is regenerated — keep it tracked:
!config/model_manifest.json

# Python
__pycache__/
*.py[cod]
*.egg-info/
.eggs/
build/
dist/
.venv/
.pytest_cache/
.coverage
htmlcov/

# Editors / OS
.vscode/
.idea/
*.swp
.DS_Store
```

### `pyproject.toml`
```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "crackvision"
version = "0.1.0"
description = "OpenCrack nnU-Net v2 crack segmentation front-end for reBot B601-DM inspection"
requires-python = ">=3.11"
dependencies = []          # managed via requirements/ — see TC-002

[tool.setuptools.packages.find]
where = ["src"]

[tool.pytest.ini_options]
testpaths = ["tests"]
markers = [
  "gpu: requires a CUDA GPU",
  "camera: requires an attached RealSense device",
  "model: requires the downloaded OpenCrack checkpoint",
]
```
Dependencies are intentionally empty here so `pip install -e .` never fights the pinned
`requirements/` files (`RISKS.md` R-16).

### `src/crackvision/config.py`
Must provide:
- `find_root(start: Path | None = None) -> Path` — use `$CRACKVISION_ROOT` if set and valid;
  otherwise walk up from this file until a directory containing both `config/project.yaml` and
  `src/crackvision` is found. Raise `ConfigError` if not found.
- `@dataclass Config` exposing `root`, the resolved absolute paths for every entry under `paths:`,
  and the `model` / `inference` / `visualization` / `skeleton` / `realsense` sections as plain dicts.
- `load_config(path: Path | None = None, root: Path | None = None) -> Config` — loads
  `config/project.yaml`, **deep-merges over built-in defaults** so a missing key never crashes,
  resolves every path against `root`, and raises `ConfigError` (→ exit code 2) on malformed YAML.
- `class ConfigError(Exception)`.
- `add_common_args(parser)` — adds `--root`, `--config`, `--verbose/-v`, `--quiet/-q`, `--dry-run`,
  `--skip-existing` (`docs/INTERFACES.md` §0.3), so every CLI gets them identically.
- `ensure_dirs(cfg)` — `mkdir(parents=True, exist_ok=True)` for every output directory.
- Built-in defaults must be a literal dict matching `docs/INTERFACES.md` §4, so the package works
  even if `project.yaml` is deleted.

Use `yaml.safe_load` (PyYAML; it is a transitive dependency of the stack and is added explicitly in
TC-002). Never `yaml.load`.

### `src/crackvision/logging_setup.py`
Must provide:
- `EXIT_OK = 0`, `EXIT_RUNTIME = 1`, `EXIT_USAGE = 2`, `EXIT_PRECONDITION = 3`.
- `setup_logging(tool: str, cfg, verbose=False, quiet=False) -> logging.Logger` — logs to stderr and
  to `logs/<tool>_<YYYYmmddTHHMMSSZ>.log`, format
  `%(asctime)s %(levelname)-7s %(name)s: %(message)s`, UTC timestamps.
- `class RunSummary` — accumulates `counts` and `errors`; `.write(cfg, tool, status, exit_code)`
  emits both `logs/<tool>_<ts>.json` and `logs/<tool>_latest.json` in the schema of
  `docs/INTERFACES.md` §0.4. Include `started_utc`, `finished_utc`, `duration_s`.
- `def utc_stamp() -> str` returning `YYYYmmddTHHMMSSZ`.

### git
```bash
cd ~/Projects/rebot_crack_vision
git init -b main
git add <the named files>          # NEVER git add -A
git commit -m "chore: project scaffolding, config and logging primitives"
```
If `git init -b main` fails on this git version (2.34.1 supports `-b`), fall back to
`git init && git symbolic-ref HEAD refs/heads/main`. Do **not** set any `--global` config.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
python3 -c "import yaml,sys; yaml.safe_load(open('config/project.yaml')); print('project.yaml parses')"
git status --short
# gitignore proof:
touch models/DUMMY.pth data/input_originals/DUMMY.png logs/DUMMY.log nnunet/results/DUMMY
git status --porcelain --ignored | grep -c DUMMY     # expect 4 (all ignored)
git status --porcelain | grep -c DUMMY               # expect 0
rm -f models/DUMMY.pth data/input_originals/DUMMY.png logs/DUMMY.log nnunet/results/DUMMY
git log --oneline
```

## Acceptance criteria
- [ ] Every directory in `docs/ARCHITECTURE.md` §5 exists, with `.gitkeep` where it would be empty.
- [ ] `config/project.yaml` parses and matches `docs/INTERFACES.md` §4 key-for-key.
- [ ] `src/crackvision/config.py` and `logging_setup.py` exist with every function/class listed above.
- [ ] `.gitignore` proof: a dummy file in each of `models/`, `data/input_originals/`, `logs/`,
      `nnunet/results/` is ignored — `git status --porcelain` shows **zero** of them.
- [ ] `git log` shows exactly one commit, on branch `main`, authored by `Gavinw575`.
- [ ] The commit contains **no** file over 1 MB (`git cat-file` / `git ls-files -s` check).
- [ ] No Claude co-author trailer in the commit message.
- [ ] `docs/` and `task_cards/` planning files are byte-identical to before this card ran
      (except the newly created `COMPLETION_LOG.md` and the `TASK_INDEX.md` status update).
- [ ] Nothing outside `~/Projects/rebot_crack_vision` was modified.

## Failure handling
- **PyYAML not importable** (likely — no env yet): run the YAML check with any interpreter that has
  it, or defer that single check to TC-003 and say so under `ISSUES:`. Do **not** pip-install
  anything in this card.
- **`git init` fails**: report BLOCKED with the error; do not work around it by committing elsewhere.
- **A directory already exists with content**: inspect it, do not overwrite; report under `ISSUES:`.

## Documentation update
Create `docs/COMPLETION_LOG.md` with a one-line header, then append your completion report.
Update `task_cards/TASK_INDEX.md`: TC-001 → `COMPLETE`, TC-002 → `READY`.

## Commit guidance
`chore: project scaffolding, config and logging primitives`

## Completion report format
```
TASK: TC-001
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-002
```
