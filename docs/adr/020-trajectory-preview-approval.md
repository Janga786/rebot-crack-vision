# ADR-020: Trajectory preview manifest and operator approval binding

**Status:** accepted · **Date:** 2026-10-13 · **Supersedes:** — · **Superseded by:** —

## Context

§11.5 already defines `crackvision.execution_approval/1`: five keys, including a `preview_artifacts[]`
list the card never specified the shape or provenance of, and `G-APPROVAL` (§11.7) only ever checks that
*some* approval file names the current trajectory sha256 and is fresh enough. Nothing today requires that
file to have been produced from, or even to agree with, anything the operator actually looked at — an
operator (or a script) can hand-write an approval file naming the right sha256 and `G-APPROVAL` passes.
REQ-OPS-1 ("capture → plan → preview → controlled execution") and REQ-INT-2 ("operator-sourced evidence")
both require the approval to be tied to a specific, reviewable rendering of the trajectory the executor is
about to run — not just to the trajectory's own sha256.

MOT-08 has to build: something that renders a trajectory so a human can actually see it, a CLI that turns
that rendering plus a typed confirmation into the §11.5 file, and a `G-APPROVAL` amendment that refuses an
approval not demonstrably tied to a preview the operator could have reviewed. This card is the decision and
contract only; MOT-08.2–.9 implement to it.

Two preview surfaces are useful for different reasons. A human reviewing a trajectory before approving a
real-mode run needs a deterministic, offline artefact that becomes *the record* of what was reviewed —
something a reviewer can replay later and get the same picture. RViz is good for *looking* at a trajectory
interactively (3D, can be rotated, shows the robot and scene together) but is unsuitable as that record: it
is live, stateful, and — critically — ships a `MotionPlanning` display with its own Plan/Execute buttons,
which is a motion-capable surface this project has spent ADR-016 keeping out of every code path that is not
`execute_trajectory` itself behind `G-CONFIRM`.

## Decision

1. **Two preview surfaces, one of which is the approval artefact.**
   - **The approved surface**: a headless, deterministic render — `./env.sh python -m
     crackvision.trajectory_preview` — producing a JSON manifest (`crackvision.trajectory_preview/1`,
     INTERFACES §18) plus three PNGs (joint tracking, tool path, tool speed), computed with
     `crackvision.kinematics` FK (the same independent FK lineage `trajectory_check`, §16.5, already uses —
     never `crackvision_motion.tool_fk`, so the preview and the retimer can never silently agree on a wrong
     number together). This is what `approve_trajectory` requires and what `G-APPROVAL` binds to.
   - **An optional convenience**: `ros2 run crackvision_motion preview_trajectory`, an RViz publisher that
     is motion-free *by construction* — no `ActionClient`, no service client, no publisher on
     `/joint_states` or any `/rebotarm/**`/vendor command topic, ever, checkable by grep over the node's
     own source the way `trajectory_check`'s no-`crackvision_motion`-import rule is checked (§16.5). The
     launch file's RViz config excludes the `MotionPlanning` display entirely — not merely "please don't
     press the button" but the button is not present. This surface is never read by `approve_trajectory` or
     `G-APPROVAL`; it exists only so an operator can look at the motion in 3D before running the thing that
     actually produces the approval record.

2. **The manifest is the thing approved, not the trajectory file directly.** `approve_trajectory` takes
   `--preview PATH`, not `--trajectory PATH`. This makes "what did the operator review" a first-class,
   re-checkable object (sha256-pinned PNGs plus the summary numbers that drove them) rather than an
   assertion that the operator looked at *a* trajectory file, trusted on their word. `G-APPROVAL`'s new
   checks (§18) then close the loop at execution time: the approval's `preview_artifacts[]` must include a
   manifest that itself names the current trajectory's sha256, so a stale or substituted preview cannot
   carry an old approval onto a new trajectory.

3. **Preview refusal is narrow; a failing check report is not a refusal.** `trajectory_preview` refuses
   only on integrity problems (bad schema, sha mismatch between the reports and the trajectory) — never
   because the trajectory itself fails its checks. An operator specifically needs to be able to render and
   look at a *failing* trajectory to understand why it fails; refusing to show it would remove the one tool
   that explains a refusal. The red banner (§18) carries that information into the artefact itself, so a
   reviewer looking at the PNG later (not just the live CLI output) still sees it failed.

4. **`approve_trajectory` re-verifies everything at approval time, from scratch, against live files** —
   never trusting the manifest's own claims about artefact or binding shas. This mirrors `G-STALE-CONFIG`/
   `G-LIMITS-HASH`'s existing "never trust the copy, re-check the live file" convention (§11.7) and closes
   the gap where a manifest rendered correctly an hour ago could be approved against files that have since
   changed underneath it.

5. **The typed phrase is `APPROVE <sha8>`, asked only after every non-interactive check passes**, on a
   directly-opened `/dev/tty`, with no flag/env/stdin substitute — the same shape as `G-CONFIRM`'s
   `EXECUTE <sha8>` (§11.8), deliberately reusing a pattern operators are already trained on rather than
   inventing new confirmation UX, while keeping the two phrases textually distinct so an operator's muscle
   memory for one can't typo into the other mid-shift.

## Alternatives considered

- **A `--yes`/`--force` flag on `approve_trajectory`.** Rejected: it is exactly the kind of
  flag/env/stdin substitute for a typed confirmation that ADR-016 §... (§11.8) already refuses to allow for
  `G-CONFIRM`, for the same reason — a flag can end up baked into a script, a CI job, or an operator's shell
  history, silently turning "a human reviewed this" into "a human ran a command once." The whole point of
  the typed, sha-specific phrase is that it cannot be copy-pasted forward to a different trajectory without
  the operator noticing the sha8 changed.
- **Folding approval into `execute_trajectory` itself** (prompt for approval inline, at run time, instead
  of a separate pre-recorded file). Rejected: it collapses "reviewed the preview" and "about to run it" into
  one moment under time pressure at the robot, which is exactly what §11.5's design (an approval file,
  checked for freshness by `G-APPROVAL`, §11.7) already deliberately separates — review happens earlier, at
  a desk, against a rendered artefact; execution happens later, at the arm, gated on that review still being
  fresh. Inlining it would also mean every `execute_trajectory --mode real` invocation needs its own
  `/dev/tty` prompt UX duplicated from `approve_trajectory`, instead of one approval artefact being reusable
  across retries of the same run within `approval_max_age_s`.
- **RViz's `MotionPlanning` panel as (part of) the approval surface.** Rejected outright: it carries Plan
  and Execute buttons wired to live planning/execution services. Including it anywhere in a preview-only
  code path would reintroduce exactly the motion-capable surface ADR-016 keeps behind commissioning and
  `G-CONFIRM`, for a tool whose entire job is to be safe to run with no commissioning at all.

## Consequences

This makes a real-mode approval demonstrably tied to artefacts a reviewer can re-open and inspect, and
makes "the trajectory changed after approval" or "the limits/scene/end-effector file changed after
approval" both refuse at `G-APPROVAL` time in `real` (warn in `mock`/`dry`), not just at the weaker
sha-and-age check §11.5 had before. It makes forging an approval require either a correctly-rendered
preview with matching artefact shas, or bypassing `approve_trajectory` entirely to hand-write both the
approval file and a passing manifest — strictly harder than today's single sha256 string.

It forecloses, for now, any workflow where an approval does not correspond to a rendered PNG triple sitting
on disk: a fully text-only/headless approval path (no matplotlib, no PNG) is not supported by this contract.
It also means every already-issued approval becomes **invalid** against the new `G-APPROVAL` rule the
moment this contract ships, because no approval existing before this card names any `preview_artifacts[]`
entry that parses as a `crackvision.trajectory_preview/1` manifest — stated explicitly in §18, since no
approvals exist in this repository yet, this is not a migration concern, only a forward-looking fact for the
first operator who approves something after MOT-08 lands.

**Residual risk.** An approval certifies only that the operator was shown this rendering, with these
derived numbers, for this exact trajectory/plan/report sha triple — never that the rendering was itself an
accurate or sufficient preview of what the arm will physically do, never that the operator actually looked
at it carefully rather than typing the phrase from habit, and never that the scene or specimen has not
moved since the render was produced (`G-STALE-CONFIG`/`G-LIMITS-HASH` catch *file* changes; nothing catches
an un-recorded physical rearrangement of the cell, exactly the same residual risk `approval_max_age_s`
already documents at §11.12). The red failing-check banner makes a failing trajectory hard to approve by
accident, but a determined operator can still type `APPROVE <sha8>` at a manifest whose `check_report.
all_pass` is `false` as long as every individual `approve_trajectory` precondition (§18) happens to pass —
this card does not add a check-report-must-pass precondition to `approve_trajectory` itself beyond the ones
listed in §18, by design (an operator may have a documented reason to approve a `warn`-only failing run);
only `G-APPROVAL`'s own `check_report.all_pass == true` requirement (§18) is the hard backstop, and it
applies at execution time, not at approval time.

## Revisit when

MOT-09 commissioning puts a real approval through this path for the first time and finds the typed-phrase
or re-verification UX too heavy for routine operation, or a future card wants multi-operator sign-off or an
approval revocation list (both explicitly out of scope here, §18) — either would need a new ADR, not a
silent amendment of this one.
