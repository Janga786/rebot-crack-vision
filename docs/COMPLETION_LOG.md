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
