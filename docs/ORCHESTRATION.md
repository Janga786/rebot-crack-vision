# ORCHESTRATION.md — claude-auto (current) and the retired cv-harness

## Current (since 2026-09-28): claude-auto
The cv-go/cv-review gate is **retired**. Work now runs through `claude-auto` (continuous impl → independent review loop
with model/effort routing, repair cards, staleness tracking, a sandbox for every worker, an Opus-5.5-xhigh-gated privileged
path, and operator-only physical motion). Plan: `plan/`. Runtime state: `.claude-auto/` (git-excluded).
Operator guide: `~/claude-auto/docs/OPERATOR_GUIDE.md`. Architecture decisions: `~/claude-auto/setup/decisions/ARCHITECTURE.md`.

    claude-auto status        claude-auto start | stop [--now] | resume        claude-auto-setup --goal "…"

Everything below is the historical design of the retired harness, kept for the record (`.task_orchestrator/` still holds
its full history; TC-001…009 were imported as accepted after their checks were re-run on 2026-09-28).

---

# ORCHESTRATION.md — the cv-go / cv-review review gate

**Harness owner:** the user. **Implementation agents must not modify any of it.**
Added 2026-09-16, after the planning session, alongside the task cards.

---

## What it is

A durable state machine that lets a cheap Sonnet agent implement one card at a time while an
independent Opus auditor validates each card before the next one can start.

```
        NOT_READY  (prerequisites not yet approved)
            │  all prerequisites APPROVED
            ▼
          READY
            │  cv-go
            ▼
      IMPLEMENTING ──── session exits non-zero ──► INTERRUPTED ──┐
            │  session exits 0                                    │ cv-go resumes
            ▼                                                     │
     AWAITING_REVIEW  ◄───────────────────────────────────────────┘
            │  cv-review  →  cv-verdict
            ├──────────────► APPROVED ──► next dependency-valid card becomes READY
            └──────────────► CHANGES_REQUIRED
                                   │  cv-go  (SAME card, revision n+1)
                                   ▼
                            IMPLEMENTING …
```

**The gate:** while any card is `AWAITING_REVIEW` or `CHANGES_REQUIRED`, `cv-go` refuses to
start any other card. There is no normal path to card N+1 until card N is `APPROVED`.

---

## Commands

| Command | Model | What it does |
|---|---|---|
| `cv-go` | **Sonnet** | Runs the one eligible card (next READY, or the current CHANGES_REQUIRED / INTERRUPTED one). Marks it `AWAITING_REVIEW` on clean exit. Never runs a second card. |
| `cv-review` | **Opus** | Audits the single `AWAITING_REVIEW` card independently. Refuses if none, or if more than one, awaits review. |
| `cv-verdict <TC-0NN> APPROVED\|CHANGES_REQUIRED "summary"` | — | Records the verdict and moves the gate. The review session runs this; you can also run it yourself to override. |
| `cv-status` | — | Board and next action. Launches nothing. |
| `cv-unlock [--force]` | — | Clears a stale lock. Refuses to steal a live one without `--force`. |

Useful flags: `cv-go --dry-run [--show-prompt]`, `cv-review --dry-run [--show-prompt]`,
`cv-status -v` (per-card review history).

Implementation permission mode (verified against Claude Code CLI **2.1.273**, 2026-09-16):
`--dangerously-skip-permissions`. Both sessions start with cwd = the project root.

---

## Where state lives

```
.task_orchestrator/          ← machine-canonical, git-excluded via .git/info/exclude
├── state.json                  per-card state, revision_count, runs[], reviews[]
├── cards.json                  card catalogue + dependency graph, parsed ONCE from the cards
├── active_review.json          the review in flight
├── lock/owner.json             atomic mkdir lock: command, pid, host, start time
├── runs/<run_id>/              prompt.txt, baseline.json, diff.patch,
│                               changed_files.txt, preexisting_dirty.txt, diff_summary.json
└── reviews/<review_id>/        prompt.txt, report.md, issues.json
```

`task_cards/TASK_INDEX.md` stays human-readable but is a **mirror**: the orchestrator rewrites
its Status column from canonical state after every transition. Agents may still update it as
their cards instruct; canonical state wins on the next transition.

---

## Guarantees the harness provides

1. **Review gate.** No card N+1 while card N is unreviewed or rejected. Enforced in `gate()`
   before any dependency logic runs.
2. **Single verdict.** `cv-review` refuses if zero or more than one card awaits review.
3. **Mandatory verdict.** A review session that exits without running `cv-verdict` leaves the
   card `AWAITING_REVIEW` and the gate locked.
4. **Locking.** Directory-based, atomic. A second `cv-go` fails cleanly with the holder's pid.
   Locks whose pid is dead, or older than 24 h, are cleared automatically; a live lock is
   never stolen.
5. **Git isolation.** The repo is snapshotted before each run. Files already dirty are recorded
   in `preexisting_dirty.txt` and separated from `attributable` in `diff_summary.json`, so the
   auditor never blames the agent for your work. **Nothing is ever reset, stashed or discarded.**
6. **Structured rework.** A `CHANGES_REQUIRED` verdict carries an `issues.json` where every
   issue has `severity`, `path`, `problem`, `expected_correction`, `acceptance_condition`.
   The next `cv-go` pastes that list into the implementation prompt.
7. **Tamper detection.** `state.json` is hashed before and after each session; a change is
   recorded on the run and surfaced to the auditor as a hard `CHANGES_REQUIRED` trigger.
8. **No auto-advance.** An `APPROVED` verdict prints the next card and stops. You run `cv-go`.

## Models

```
Opus plans  →  Sonnet builds  →  Opus validates  →  Sonnet builds next card
```

Sonnet may delegate narrow, bounded support work (file searches, scope greps, log inspection,
collecting command output) to **Haiku** subagents. A subagent may never choose a card, alter
the architecture, or touch orchestration state. Overrides: `CV_IMPL_MODEL`, `CV_REVIEW_MODEL`.
`CV_NO_LAUNCH=1` exercises the state machine without spawning anything (used to test the harness).
