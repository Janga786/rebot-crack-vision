# ADR-021: Operator CLI workflow — stage model, run-state schema, resumability

**Status:** accepted · **Date:** 2026-10-14 · **Supersedes:** — · **Superseded by:** —

## Context

REQ-OPS-1 asks for "clean launch → capture → plan → preview → controlled execution, with failure
recovery." Every piece of machinery that sequence needs already exists and is already independently
gated: ADR-014/§8–§9 (capture), §3.13/§10 (perception/lifting), §13/§16 (planning/retiming/
checking), §17 (view-move trajectories), §18/ADR-020 (preview and approval), §11/ADR-016
(commissioning-gated execution). What does not exist yet is the thing that strings them together for
an operator who should not have to remember fifteen CLI invocations in the right order, or re-run a
40-minute capture because `plan` failed on a typo.

OPS-01 is eight implementer cards building that operator-facing wrapper. Without a fixed contract
first, each sub-card would invent its own idea of "what stage comes next", "what counts as done" and
"what happens to `--mode`" — exactly the kind of drift this project's append-only `ADR` +
`INTERFACES.md` discipline exists to prevent. This card is the decision: the stage list and order, the
on-disk run-state file and its freshness rule, and — the one rule that actually matters for safety —
that nothing OPS-01 builds may ever shorten the existing `retime_plan → trajectory_check →
trajectory_preview → approve_trajectory → execute_trajectory` gate sequence for any trajectory
execution, including the capture-time view move §17 already defined for exactly this workflow.

OPS-01 adds no new planning, perception or execution logic. Every stage below is a thin sequencer
over a CLI or ROS entry point this project has already specified and reviewed. The only new
artefact is the run-state file that lets an operator stop and resume a case without re-running work
that is still valid, and without silently reusing work that is not.

## Decision

1. **Fixed stage list and order**, exactly as `docs/INTERFACES.md` §19 states: `doctor` (global, no
   `case_id`) then, per `case_id`: `capture → segment → paths → plan → preview → execute`. Each
   stage delegates to an existing contract/CLI; OPS-01 writes no new segmentation, lifting, planning
   or execution logic of its own — only sequencing, status tracking and operator-facing messaging.

2. **One run-state file per case**, `data/operator_runs/{case_id}_run_state.json`, schema
   `crackvision.operator_run/1` (§19.2). It is git-ignored, the same way `data/captures/` already
   is (§9) — it is a live run record, not a build artefact.

3. **Resumability is mechanical, not advisory.** A stage is treated as already done, and skipped,
   only if: its recorded status is `done`; every output file it declared still exists with a
   matching sha256; and every stage it names as a dependency is *itself* still fresh by this same
   rule, recursively. A stale upstream output invalidates every stage downstream of it, even if that
   downstream stage's own output files are untouched — re-running `capture` always invalidates the
   cached `done` status of `segment`, `paths`, `plan`, `preview` and `execute` for that case, because
   each one depends (directly or transitively) on `capture`'s output. `--force` always re-runs a
   named stage regardless of its recorded status. `--stage NAME` runs exactly that one stage and
   exits 3 if a prerequisite stage is not currently fresh — it never silently runs prerequisites for
   the operator.

4. **No OPS-01 code path may ever skip a trajectory execution gate.** Every trajectory this workflow
   ever runs — the capture-time view move (§17, when the camera is not already positioned) and the
   final crack-trace trajectory alike — goes through `retime_plan → trajectory_check →
   trajectory_preview → approve_trajectory → execute_trajectory`, in that order, with no flag that
   collapses or reorders any of those five steps. OPS-01 never sets `CRACKVISION_ARM_REAL` itself
   (ADR-016 §11.7 `G-ARM`: only an operator's own shell environment may set it), and never defaults
   `--mode` to anything but `mock` on any `execute_trajectory`/`retime_plan`/`trajectory_check`
   invocation it issues. The G-\* gate catalogue (§11.7, amended by §17.2/§18.3) remains the only
   authority on whether real motion happens; OPS-01's job is sequencing already-reviewed CLIs in the
   order `docs/motion/EXECUTION.md`/`PREVIEW_APPROVAL.md` already specify, never adding a shortcut
   around them.

5. **`doctor` is case-independent, read-only, and exits only 0 or 3.** It aggregates
   `scripts/check_env.py`, `scripts/check_host.py`, `scripts/check_realsense.py` and a
   `ros2_ws/install/setup.bash` presence check — every one of those is itself read-only — so its own
   exit code is restricted to `{0, 3}` (pass / not-ready); it has no usage-error or runtime-failure
   mode of its own because it performs no action beyond running those checks and reporting.

Full field-level schema, the freshness algorithm and the stage-to-CLI delegation table are appended
as `docs/INTERFACES.md` §19, the same append-only pattern §15–§18 already established for this file.

## Alternatives considered

- **No run-state file; always re-run every stage from `capture` on.** Rejected: a capture-to-execute
  cycle includes a camera session and, per REQ-OPS-1, a human-reviewed preview/approval — re-running
  all of that after a transient failure in `execute` (e.g. a dropped ROS graph) would throw away a
  still-valid capture, segmentation and plan for no reason, and would punish exactly the failure
  mode ("failure recovery") REQ-OPS-1 calls out.
- **A single global run-state file for all cases.** Rejected: cases are processed independently
  (§1), and a shared file would force every case's stage history to serialize through one
  read-modify-write, with no benefit — nothing in this workflow ever needs cross-case state.
- **Trust each stage's own exit code as the freshness signal, without checking output file
  hashes.** Rejected: a stage can exit 0 but have its output file edited or deleted afterward (by a
  later stage's `--clean`, an operator's manual edit, or a different tool entirely); checking sha256
  against the recorded value is the same "never trust the copy, re-check the live file" discipline
  §11.7's `G-STALE-CONFIG`/`G-LIMITS-HASH` already use, applied here to the operator's own bookkeeping
  instead of to a config file.
- **A `--skip-gates`/`--yes` convenience flag for repeated test runs.** Rejected outright, for the
  same reason ADR-020 rejected a `--yes` flag on `approve_trajectory`: the entire value of the
  preview/approval/confirmation chain is that it cannot be short-circuited by a flag that ends up
  baked into a script or an operator's habit. OPS-01 gets no exemption no other card gets.

## Consequences

**Makes easy:** every OPS-01.2–.8 implementer card has one stage list, one run-state schema and one
freshness rule to build against, instead of each re-deriving "what does resumable mean here"; an
operator can interrupt a long capture-to-execute run at any point and resume without re-doing
finished, still-valid work; a reviewer auditing an OPS-01 script only has to check that it calls the
named existing CLIs in the named order, never that it reimplements any safety-relevant logic itself.

**Makes hard / costs:** every stage's "outputs" and "depends_on" list must be kept accurate as the
underlying CLIs evolve — a stage that starts writing an extra output file, or an upstream card that
renames a default output path, needs its OPS-01 declaration updated too, or the freshness rule stops
being trustworthy. The cascade-invalidation rule is also strict in the conservative direction: it
will re-run more than is strictly required whenever any upstream output's sha256 changes for a
reason unrelated to correctness (e.g. a timestamp field embedded in an otherwise-identical file),
which this project accepts as the safer failure mode over silently reusing stale data.

**Forecloses:** any operator convenience that trades away the preview/approval chain's integrity for
speed — this ADR states plainly that OPS-01 is never the place that gap gets opened, matching
ADR-016/ADR-020's existing stance.

## Revisit when

- MOT-09 commissioning is complete and `real`-mode runs through this workflow for the first time,
  which may surface friction in the `--stage`/`--force` UX that this contract did not anticipate.
- A future stage needs state that spans more than one `case_id` (e.g. a batch run across many
  cases), at which point the "one run-state file per case" decision (point 2) needs revisiting
  alongside whatever batch-level status format that would require.
