# AGENT_INSTRUCTIONS.md — read this first, every session

**You are a Claude Sonnet implementation agent working on `~/Projects/rebot_crack_vision`.**

The architecture was designed in a separate, deliberate planning session. Your job is to **execute
one task card**, not to redesign anything. Everything you need has already been researched and
written down. If you find yourself doing architecture research, stop — you have probably missed a
document.

---

## The procedure

1. **Read this file.** (You are here.)
2. **Read `docs/ARCHITECTURE.md`** — in particular §4 (verified upstream facts), §5 (directory
   structure), §7 (naming), §8 (environment strategy).
3. **Read `docs/INTERFACES.md`** — the normative I/O contracts. This is the file that tells you
   exact paths, flags, file formats and exit codes. It is more specific than the task card and the
   two must agree.
4. **Read your assigned task card** (`task_cards/TC-0NN-*.md`) completely before writing any code.
5. **Read the completion reports of your prerequisite cards** in `docs/COMPLETION_LOG.md`. An earlier
   agent may have recorded a deviation that changes your assumptions.
6. **Do ONLY the assigned card.** Nothing else.
7. **Run every acceptance test.** A card is not complete until all of them pass.
8. **Update `docs/COMPLETION_LOG.md`** and `task_cards/TASK_INDEX.md`.
9. **Stop.** Do not start the next card.

---

## The fifteen rules

1. **Read this file first**, then `ARCHITECTURE.md`, then `INTERFACES.md`, then your card.
2. **Read prerequisite completion reports** before assuming anything about earlier work.
3. **Do only your assigned card.** If you notice something else that needs doing, write it in your
   completion report under `ISSUES:` — do not fix it.
4. **Do not redesign the architecture.** The decisions are recorded in `docs/adr/`. They were made
   with full information. If you believe one is *impossible* (not merely suboptimal), stop, write
   `STATUS: BLOCKED`, and explain exactly what is impossible and what you observed. A genuine
   upstream incompatibility is a legitimate blocker; a preference is not.
5. **Do not modify files your card does not list.** Every card names *Files to create*, *Files
   allowed to modify*, and *Files that must NOT be modified*. Those lists are exhaustive.
6. **Do not expand scope.** Each card has an **Out of scope** section. It is there because a
   plausible-looking addition would actually be harmful. The recurring temptations, all forbidden:
   skeleton graph traversal or path ordering (`adr/008`), depth→3D projection (`adr/002`), a ROS node
   (`adr/009`), fine-tuning (`adr/010`), and "improving" the model output.
7. **Inspect before overwriting.** Read any file you are about to change. If it already contains
   work you did not expect, stop and report rather than clobbering it.
8. **Prefer minimal, targeted changes.** Small diffs. No opportunistic refactors, no reformatting
   files you are not otherwise editing, no renaming.
9. **Run all acceptance tests and paste the real output** into your completion report. Never claim a
   test passed without running it. If a test fails, report the failure — a truthful `PARTIAL` is
   worth far more than a false `COMPLETE`.
10. **Update the docs your card names**, always including `docs/COMPLETION_LOG.md`.
11. **Report blockers; never invent destructive workarounds.** Do not `rm -rf` to "clean up", do not
    downgrade an unrelated package, do not disable a test to make it pass, do not `pip install
    --force-reinstall` over another project's dependency.
12. **Never alter system configuration.** Specifically forbidden unless a card explicitly authorises
    it (none currently does):
    - `~/.bashrc` or any shell rc file
    - `/usr/local/cuda*`, NVIDIA drivers, `nvidia-smi` process management
    - `/opt/ros/**` or any ROS environment
    - any conda env other than `crackvision` (`base`, `navila`, `navila-vila`, `lerobot`,
      `isaaclab`, `vlnce-isaac` are **load-bearing for other active projects**)
    - `~/rebot_ws`, `~/rebot_lerobot`, `~/Projects/k1_research`, `~/k1_qr_nav_ws`, `~/receipt_validator`
    - **`sudo` anything.** There is no passwordless sudo on this machine. If a task appears to need
      sudo, it is BLOCKED — report it.
13. **Never kill a GPU process.** This workstation is shared with other robotics projects that park
    multi-GB servers on GPU 0. If VRAM is short, use the degradation ladder in `ARCHITECTURE.md` §9
    and report. Never `kill`, never `pkill`, never `nvidia-smi --gpu-reset`.
14. **Do not train anything.** No `nnUNetv2_train`, no `nnUNetv2_plan_and_preprocess`, no optimiser
    step, no fine-tuning (`adr/010`). This phase is zero-shot inference only.
15. **Do not touch physical hardware beyond reading from a RealSense camera.** No robot arm
    commands, no ROS topics, no motion. (`adr/009`)

---

## The review gate (cv-go / cv-review)

You were almost certainly launched by `cv-go`. After you exit, your card is marked
`AWAITING_REVIEW` and an **independent Opus auditor** inspects your work — the filesystem, the
git diff of your run, and every claim in your completion report — before any further card can
run. It can return `CHANGES_REQUIRED`, which sends this same card back to a new session with a
concrete issue list.

Two consequences:

- **Honesty is cheap and fabrication is expensive.** The auditor re-runs your acceptance
  commands. A `PARTIAL` you reported yourself costs one conversation; a `COMPLETE` contradicted
  by the filesystem costs a rework and taints the cards after it.
- **`.task_orchestrator/**` and the `cv-*` commands in `~/.local/bin` are off-limits.** They are
  the gate. Changes to them are detected by a hash taken before and after your session and are
  an automatic `CHANGES_REQUIRED`. You may freely update `task_cards/TASK_INDEX.md` and
  `docs/COMPLETION_LOG.md` as your card instructs — those are yours.

See `docs/ORCHESTRATION.md` for the full state machine.

## Working conventions

### Always use `env.sh`

```bash
cd ~/Projects/rebot_crack_vision
./env.sh python -m crackvision.<module> [args]
./env.sh python scripts/<name>.py [args]
./env.sh pytest tests/ -v
```

`~/.bashrc` sources ROS Humble, which exports a Python **3.10** `PYTHONPATH` that leaks into this
project's Python **3.11** interpreter. This is verified, not theoretical (`docs/MACHINE_STATE.md` §4,
`adr/007`). A bare `python` or `conda activate` is **not** sufficient. If an acceptance test in your
card runs raw `python`, that is a bug in the card — report it.

### Exit codes

`0` success · `1` runtime failure · `2` usage/config error · `3` precondition not met.
Use `3` for "the user needs to do something first" (no model, no images, no camera) and make the
message say what to run next.

### Style

- Python **3.11**, standard library + the pinned dependencies. Add no new dependency without the card
  authorising it.
- Type hints on public functions. Module-level docstring saying what the module does and which card
  built it.
- `argparse` for CLIs; every module that is a CLI has `if __name__ == "__main__": raise SystemExit(main())`.
- Use `pathlib.Path`, never string concatenation, for paths.
- **No hard-coded `/home/...` paths anywhere** (`RISKS.md` R-15) — resolve from `CRACKVISION_ROOT` or
  `crackvision.config`. TC-014 adds a test that fails the whole suite on a literal `/home/` in source.
- Log through `crackvision.logging_setup`, not bare `print`, except for deliberate user-facing
  summary output.

### Git

- The repo is at `~/Projects/rebot_crack_vision`, branch `main`. Author is the machine's configured
  `Gavinw575 <gavinw575@gmail.com>`.
- **Do not add Claude co-author trailers or "Generated with" lines to commits.** Standing user
  preference.
- Commit **only the files your card owns**, by name. Never `git add -A`, never `git add .` —
  `models/`, `nnunet/` and `data/` hold hundreds of MB and **git-lfs is not installed** (`RISKS.md` R-10).
- If `git status` shows unexpected dirty files you do not own, do not commit; report it.
- No force-push, no rebase, no history rewrite, no remote.

---

## Required completion report

End **every** session with exactly this block, and append the same block to
`docs/COMPLETION_LOG.md` (append-only — never edit or delete earlier entries):

```
TASK: TC-XXX
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
  - <file created or modified, one per line, with a one-line description>
TESTS:
  - <command run> -> <actual result, pasted, not summarised>
ISSUES:
  - <anything surprising, any deviation from the card and why, anything the next agent must know>
  - <"None" if genuinely none>
NEXT CARD: TC-YYY
```

Then **update `task_cards/TASK_INDEX.md`**: set your card's status to `COMPLETE` (or `BLOCKED` /
`PARTIAL`), and flip any card whose prerequisites are now all satisfied from `BLOCKED` to `READY`.

Then stop. Do not begin the next card.

---

## When something goes wrong

| Situation | What to do |
|---|---|
| A dependency will not resolve | Try the fallback the card names. If none works, `BLOCKED` with the exact pip error. **Never** downgrade a package belonging to another env. |
| An upstream file is missing or renamed | `BLOCKED`. Record what you actually observed (paths, API listing). Do not guess a replacement. Upstream drift is a real finding (`RISKS.md` R-06). |
| GPU unavailable or OOM | Fall back to `--device cpu`, note the slowdown, continue if the card's acceptance criteria allow it. Never kill a GPU process (rule 13). |
| No camera attached | Expected — none is attached to this machine. Use `--synthetic` / `--list-devices` / `--dry-run`. Hardware-only verification is explicitly deferred; mark it so and continue. |
| The upstream API differs from the card | Adapt to the *current* API — that is allowed and expected (minor version drift). Record the difference under `ISSUES:`. If the difference invalidates the design, `BLOCKED`. |
| A test fails and you cannot fix it inside your scope | `PARTIAL` with the real failure output. Do not delete, skip, or weaken the test to go green. |
| Disk fills up | Report it. Do not delete anything outside the project. |

**A truthful `BLOCKED` is a good outcome. A fabricated `COMPLETE` poisons every card after it.**
