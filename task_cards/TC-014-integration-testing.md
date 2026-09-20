# TC-014 — Integration test suite and project hygiene checks

**Task ID:** TC-014 · **Complexity:** MEDIUM · **Prerequisites:** TC-010, TC-011, TC-013

## Objective
Complete the `tests/` suite: a full end-to-end integration test, cross-component contract tests, and
the repo-hygiene checks that guard the architecture's invariants.

## Why this matters
The individual cards each tested their own module. This card tests the *seams* — where contracts are
actually broken — and makes the invariants (frame size, no hard-coded paths, no scope creep)
enforceable by a command rather than by good intentions.

## Prerequisites
TC-010, TC-011, TC-013 COMPLETE (so every component exists).

## Scope
1. `tests/test_integration_smoke.py` — end-to-end + hygiene.
2. `tests/test_contracts.py` — schema validation for `case_map.json`, `_skeleton_stats.json`,
   D405 metadata, and the `logs/*_latest.json` summary shape.
3. Make `./env.sh pytest tests/ -v` green **with no GPU, no camera, and no downloaded model**.

## Out of scope
- Modifying any component to make a test pass. If a test reveals a real bug, **report it** under
  `ISSUES:` and mark `PARTIAL` — the owning card gets a follow-up. Only trivially-obvious fixes
  within a single file you already own are permitted, and each must be called out.
- Any accuracy/model-quality assertion (`docs/VALIDATION_PLAN.md` §L2).
- CI configuration, GitHub Actions, pre-commit hooks.
- Performance benchmarking.

## Files to create
```
tests/test_integration_smoke.py
tests/test_contracts.py
```

## Files allowed to modify
`tests/conftest.py` (add shared fixtures), `docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`.

## Files that must NOT be modified
`src/crackvision/**` and `scripts/**` — **except** for a trivially-obvious bug fix, which must be
named explicitly in the completion report. Never weaken or delete an existing test to go green.

## Implementation requirements

### `tests/test_integration_smoke.py`

**A. End-to-end with a stub predictor (no GPU, no model).**
Build a temp project, drop 3 fixture images (RGB JPEG, RGBA PNG, grayscale PNG) into
`input_originals/`, run `prepare_inputs`, then write fake `{0,1}` predictions directly into
`predictions/` (monkeypatching or simply writing the files — do **not** shell out to nnU-Net), then
run `visualize` and `skeleton`. Assert every expected file exists with the expected name and size.

**B. Frame invariant, end to end** (`RISKS.md` R-14):
for each case, assert
`original.size == nnunet_input.size == prediction.size == mask.size == overlay.size == skeleton.size`.
Include a **non-square** fixture (e.g. 640×360) — square images hide transpose bugs.

**C. Naming round-trip:** for every case, `derive_case_id(source_path)` reproduces the `case_id` in
`case_map.json`, and every path helper points at a file that exists.

**D. No hard-coded machine paths** (`RISKS.md` R-15):
```python
for path in Path("src").rglob("*.py") + Path("scripts")... + Path("tools")... + Path("tests")...:
    assert "/home/" not in path.read_text(), f"hard-coded home path in {path}"
```
Also check `env.sh` and `run_test.sh`. Exempt: nothing. (Docs may contain them; source may not.)

**E. Scope guards** — cheap greps that keep the deferred decisions deferred (`RISKS.md` R-18):
- `src/crackvision/skeleton.py` contains no `networkx|skan|traverse|ordered_path|prune|spur|longest_path|polyline|spline` (`adr/008`)
- no source file imports `rclpy`, `rospy`, `moveit`, or `sensor_msgs` (`adr/009`)
- no source file calls `nnUNetv2_train` or `nnUNetv2_plan_and_preprocess` (`adr/010`)
- no source file contains `weights_only=False` (TC-005 safety rule)
- no source file contains `shell=True`
- no source file calls `os.system`, `subprocess.call("kill"...)`, `pkill`, or `nvidia-smi --gpu-reset`
  (`AGENT_INSTRUCTIONS.md` rule 13)

**F. CLI surface:** for each of the 5 `crackvision.*` CLIs and 5 `scripts/*.py`, assert
`--help` exits 0, and that each accepts `--dry-run` and `--verbose`.

**G. Markers behave:** `@pytest.mark.gpu` / `camera` / `model` tests are **skipped**, not errored,
when the resource is absent.

### `tests/test_contracts.py`

Validate, with hand-written schema checks (no `jsonschema` dependency unless already installed):
- `case_map.json` — `INTERFACES.md` §2: required keys, types, POSIX-relative paths, sorted by `case_id`,
  64-hex sha256s.
- `{case}_skeleton_stats.json` — §3.4: required keys, `mask_fraction == mask_pixels/(H*W)` within
  float tolerance, `skeleton_pixels <= mask_pixels`, `components_after <= components_before`.
- D405 per-frame metadata — §3.10: required keys including `depth_scale_m_per_unit` (a **float, > 0**,
  and explicitly **not** absent or null), both intrinsics blocks, `aligned_to`.
  Use `--synthetic` output as the fixture.
- `logs/*_latest.json` — §0.4: `tool`, `started_utc`, `finished_utc`, `duration_s`, `status`,
  `exit_code`, `counts`, `errors`; `status` in the allowed set.

### Suite must be hardware-free
`./env.sh pytest tests/ -v` must exit **0** on a machine with no GPU, no camera and no model, with
the marked tests **skipped**. Verify by running with `CUDA_VISIBLE_DEVICES=""`.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh pytest tests/ -v ; echo "exit=$?"
CUDA_VISIBLE_DEVICES="" ./env.sh pytest tests/ -v ; echo "no-gpu exit=$?"
./env.sh pytest tests/ -v -m "not gpu and not camera and not model" ; echo "exit=$?"
./env.sh pytest tests/ --collect-only -q | tail -5
# hygiene checks standalone:
grep -rn "/home/" src/ scripts/ tools/ tests/ env.sh run_test.sh || echo "no hard-coded home paths"
grep -rn "weights_only=False\|shell=True\|pkill\|gpu-reset" src/ scripts/ tools/ || echo "no unsafe calls"
```

## Acceptance criteria
- [ ] `./env.sh pytest tests/ -v` exits **0**.
- [ ] `CUDA_VISIBLE_DEVICES="" ./env.sh pytest tests/ -v` exits **0** (GPU tests skipped, not errored).
- [ ] The suite passes with no camera and with no downloaded model.
- [ ] The end-to-end stub test covers RGB JPEG, RGBA PNG and grayscale PNG inputs.
- [ ] The frame-invariant assertion covers a **non-square** image.
- [ ] Naming round-trip passes for every case.
- [ ] The hard-coded-`/home/` check runs over `src/`, `scripts/`, `tools/`, `tests/`, `env.sh`,
      `run_test.sh` and **passes**.
- [ ] All six scope-guard greps pass.
- [ ] All ten CLIs respond to `--help` with exit 0.
- [ ] All four JSON contracts validate.
- [ ] Marked tests skip cleanly rather than erroring.
- [ ] **No component source file was modified** — or, if a trivial bug fix was unavoidable, it is
      named explicitly in the completion report with the reason.

## Failure handling
- **A test reveals a genuine bug in another card's component** → do **not** silently fix it beyond a
  one-line obvious correction. Record it under `ISSUES:` with the failing assertion, mark `PARTIAL`,
  and name the card that owns it.
- **A scope guard fires** (e.g. traversal code found in `skeleton.py`) → that is a real finding:
  report it, do not delete the guard.
- **The suite is slow (>2 min)** → acceptable; do not add parallel test execution.
- **A fixture needs the real model** → mark it `@pytest.mark.model` so it skips. The suite must never
  require a 268 MB download.

## Documentation update
Append the report with the full pytest summary line and the test count. Update `TASK_INDEX.md`:
TC-014 → `COMPLETE`, TC-015/TC-016 → `READY`.

## Commit guidance
`test: add integration, contract and project-hygiene test suite`

## Completion report format
```
TASK: TC-014
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-015
```
