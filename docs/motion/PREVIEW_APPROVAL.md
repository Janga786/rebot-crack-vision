# PREVIEW_APPROVAL.md — operator runbook for preview → approval → dry/real execution

**Status:** operator-facing. Normative source for behaviour: `docs/adr/020-trajectory-preview-approval.md`
(ADR-020) and `docs/INTERFACES.md` §18 (amending §11.5/§11.7 `G-APPROVAL`). This document explains how to
run `trajectory_preview`, inspect its output, run `approve_trajectory`, and then rehearse/execute with the
resulting approval; it adds nothing to, and never relaxes, those two sources. If anything here disagrees
with them, they win.

## Purpose

An approval must be demonstrably tied to a specific, reviewable rendering of the trajectory the executor
is about to run (ADR-020). This doc walks the exact sequence: `trajectory_check` → `trajectory_preview`
(renders the manifest + three PNGs, §18.1) → inspect the PNGs → optionally look at the trajectory in RViz
(§18.4, a convenience only) → `approve_trajectory` (§18.2, typed `APPROVE <sha8>`) → `execute_trajectory
--mode dry --approval` (and, eventually, the gated `real` path, `docs/motion/EXECUTION.md`).

## Inputs

- A trajectory file (`crackvision.joint_trajectory/1`, §11.2) from `retime_plan`.
- Its plan file (`crackvision.cartesian_plan/1` or `crackvision.view_plan/1`, whichever produced it).
- Its `*.retime.json` sidecar (retime report, §16.4) and `*.check.json` sidecar (check report, §16.5) —
  `trajectory_preview` defaults to `<trajectory>.retime.json` / `<trajectory>.check.json`, the same
  sidecar-naming convention `trajectory_check` already uses.

## Step 1: `trajectory_check`

Run the independent re-check before previewing — `trajectory_preview` reads this report rather than
recomputing it, and refuses if it does not parse or does not name the trajectory's current sha256 (§18.1):

```bash
./env.sh python -m crackvision.trajectory_check --trajectory PATH --plan PATH --report PATH --root REPO_ROOT \
    [--limits PATH] [--end-effector-config PATH] [--execution-config PATH] [--retime-config PATH] \
    --out CHECK_OUT [-v|-q]
```

Confirm every entry in `CHECK_OUT`'s `checks[]` is `pass` or `skip` before moving on. A failing check does
not block previewing (Step 2) — `trajectory_preview` renders a failing trajectory on purpose, so an
operator can see *why* it fails — but `approve_trajectory` (Step 4) refuses to approve one.

## Step 2: `trajectory_preview`

```bash
./env.sh python -m crackvision.trajectory_preview --trajectory PATH --plan PATH \
    [--retime-report PATH] [--check-report PATH] [--execution-config PATH] \
    [--limits PATH] [--end-effector-config PATH] [--scene-config PATH] \
    [--out-dir PATH] [--root PATH] [--dry-run] [-v|-q]
```

`--trajectory` and `--plan` are required; the rest default exactly as §11.10/§16.1 already default them.
Output directory defaults to `data/previews/{case}_{sha8}/` (`{sha8}` = first 8 hex chars of the
trajectory's sha256; `{case}` = the plan's `case_id`, or the literal `view` for a `view_plan`). Inside it,
exactly four files are always written: `preview.json` (the `crackvision.trajectory_preview/1` manifest),
`joints.png`, `toolpath.png`, `toolspeed.png`.

Refuses (exit 3, nothing written) only on integrity problems — a file that does not parse as its expected
schema, or a retime/check report whose `trajectory_sha256` does not match the trajectory's current
sha256 — never because the trajectory's own checks fail (§18.1).

## Step 3: inspecting the PNGs

Before approving, open all three PNGs and check:

- **`joints.png`**: each of the six subplots stays clear of the horizontal `lower`/`upper` reference
  lines — that is the joint's limit headroom; any trace that hugs a line leaves little margin.
- **`toolpath.png`**: the left (x/y) and right (z vs arc length) subplots show the tool path going where
  it should relative to the scene and the crack — specifically that the trace legs (colour-coded by leg
  `kind`: `transit`/`approach`/`trace`/`retract`) actually sit over the crack/specimen, not somewhere
  unexpected, and that `z` stays sane (no unexpected dip or climb).
- **`toolspeed.png`**: the plotted tool speed is the **unscaled** file's own speed; read it together with
  the manifest's `summary.tool.max_speed_scaled_m_s` / `summary.duration_scaled_s` (`preview.json`) to see
  the tool speed and duration a `real`/`dry` run would actually have at `execution.yaml`'s
  `speed_scale.real_default` — the scale the manifest was rendered at.
- **A red `"CHECK FAILED: ..."` banner on any of the three PNGs** means `check_report.all_pass` is
  `false` — read the listed failed checks; `approve_trajectory` will refuse this preview outright (Step 4).

## Step 3b (optional): RViz preview

On a machine with a display:

```bash
source scripts/ros/env_ros.sh
set +u; source ros2_ws/install/setup.bash
ros2 launch crackvision_motion preview_rviz.launch.py trajectory:=PATH
```

This launches the same headless mock planning stack (`mock_planning.launch.py`, MOT-02) purely so RViz has
a `RobotModel`/`PlanningScene` to render against, plus `preview_trajectory` (publishes
`/display_planned_path` and `/crackvision/preview_markers`, both motion-free by construction — no
`ActionClient`, no service client, no publisher on `/joint_states` or any vendor command topic) and
`rviz2` with `rviz/preview.rviz`. That RViz config has **no `MotionPlanning` display and therefore no
Plan/Execute button at all** — not merely one the operator shouldn't press. This surface is never read by
`approve_trajectory` or `G-APPROVAL`; it is only for looking at the motion in 3D before running the thing
that actually produces the approval record (Step 4).

## Step 4: `approve_trajectory`

```bash
./env.sh python -m crackvision.approve_trajectory --preview PATH --operator ID \
    [--out PATH] [--limits PATH] [--scene-config PATH] [--end-effector-config PATH] \
    [--execution-config PATH] [--root PATH] [--dry-run] [-v|-q]
```

`--preview` points at `preview.json` (Step 2's manifest), not at the trajectory file — the manifest is
re-verified from scratch against the live trajectory/artefact/check-report/config files, never trusting
any sha256 the manifest itself recorded (§18.2). On success it writes the five `crackvision.execution_
approval/1` keys (§11.5) — `schema`, `trajectory_sha256`, `approved_by`, `approved_utc`, and
`preview_artifacts[]` (the manifest itself plus its three PNGs, four entries) — to
`data/approvals/{case}_{sha8}_approval.json` by default. **An existing file at the output path is never
overwritten, even with `--skip-existing`** — exit 3 unconditionally; an approval is a one-time record.

Only after every refusal check below passes does it open `/dev/tty` directly and prompt for the exact
phrase `APPROVE <sha8>` (`{sha8}` = the trajectory's current sha256, first 8 hex chars, case-sensitive,
matched after `.strip()`). No flag, environment variable or stdin path substitutes for this prompt.

**What an approval certifies, and what it does not.** It certifies only that the named operator was shown
this exact rendering — these PNGs, these derived numbers, for this trajectory/plan/report sha triple —
at `approved_utc`, and that the check report it was rendered from passed every check. It never certifies
that the rendering was an accurate or sufficient preview of what the arm will physically do, that the
operator scrutinised it rather than typing the phrase from habit, or that the scene/specimen has not
physically moved since the render (ADR-020, "Residual risk"). It expires after `approval_max_age_s`
(ships 3600 s, `execution.yaml`) and is invalidated — at `G-APPROVAL` time — by any change to the
trajectory file, to any of the four `preview_artifacts[]` files, or to the limits/scene/end-effector
configs the manifest's `bindings.*` named (§18.3). **Approval alone never moves the arm**: see Step 5.

## Step 5: execution

```bash
ros2 run crackvision_motion execute_trajectory --trajectory PATH --mode dry --approval APPROVAL_PATH \
    [--profile moveit_mock|vendor_mock|vendor] [other flags per docs/motion/EXECUTION.md §4]
```

`dry` never sends a `FollowJointTrajectory` goal in any profile; it only evaluates every gate (including
the amended `G-APPROVAL`, §18.3) and reports what `real` would do. The gated real path — commissioning
(MOT-09), measured scene/end-effector configs, `CRACKVISION_ARM_REAL=1`, and the typed `EXECUTE <sha8>`
confirmation (`G-CONFIRM`) — is unchanged by this document and is `docs/motion/EXECUTION.md`'s subject in
full; an approval only ever satisfies `G-APPROVAL`, one row among many, and never substitutes for any of
the others.

## Troubleshooting

| Refusal | Remedy |
|---|---|
| `trajectory_preview` exit 3: trajectory/plan/report does not parse, or a report's `trajectory_sha256` mismatches | Re-run the producing step (`retime_plan`/`trajectory_check`) against the current trajectory file so the sidecars agree with it |
| `approve_trajectory` exit 3, check 1–2 (manifest/trajectory unreadable or sha mismatch) | Re-render `trajectory_preview` against the current trajectory |
| `approve_trajectory` exit 3, check 3 (an artefact PNG missing or tampered) | Re-render `trajectory_preview`; do not hand-edit anything under `data/previews/` |
| `approve_trajectory` exit 3, check 4 (check report stale, mismatched, or any check `fail`) | Re-run `trajectory_check`, fix the trajectory until every check is `pass`/`skip`, then re-preview |
| `approve_trajectory` exit 3, check 5 (`purpose: "test"`) | Only `crack_task`/`view_move` trajectories can be approved; a `test` trajectory already refuses unconditionally in `real` |
| `approve_trajectory` exit 3, check 6 (`bindings.*` mismatch) | The limits/scene/end-effector config changed since the preview was rendered; re-preview against the current configs |
| `approve_trajectory` exit 3, wrong phrase or no controlling tty | Type the exact `APPROVE <sha8>` phrase on a real terminal; no scripted substitute exists |
| `approve_trajectory` exit 3, output path exists | Approvals are never overwritten; pick a new `--out` path or re-run with a fresh preview |
| `execute_trajectory` `G-APPROVAL` warn/fail | Read the gate's `detail` (§18.3 checks 1–5 plus the pre-existing sha/age checks); get a fresh approval naming the current trajectory sha, or re-run `trajectory_preview`/`approve_trajectory` so the approval lists exactly one bound, passing manifest |

## Verification

- `scripts/ros/test_preview_approve.sh` — headless mock run of the whole sequence (view plan → `retime_plan
  --view-plan` → `trajectory_check` → `trajectory_preview` → `approve_trajectory` via a pseudo-tty →
  `execute_trajectory --mode dry --approval`, asserting `G-APPROVAL` pass), plus the tamper/refusal paths:
  a wrong approval phrase, a byte-tampered preview artefact, an approval bound to a different trajectory,
  and the unavoidable `real`-mode-without-commissioning refusal.
- `scripts/ros/test_preview_rviz.sh` — headless proof that `preview_rviz.launch.py`'s node graph
  (`mock_planning.launch.py` + `preview_trajectory`) publishes `/display_planned_path` and
  `/crackvision/preview_markers` without moving the arm (`/joint_states` unchanged); `rviz2` itself is not
  exercised on a host with no display — see that script's own comment for the manual RViz step.
