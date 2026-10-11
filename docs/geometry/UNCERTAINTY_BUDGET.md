# Pixel-to-tool-pose uncertainty budget (`crackvision.uncertainty_budget/1`)

## Purpose

GEOM-09 built the pieces that compute, per-term, how much of a `paths3d` point's or waypoint's
uncertainty comes from pixel localization, depth noise, hand-eye calibration, TCP calibration and
(new in GEOM-09.4) joint-encoder/FK error, and cross-checks that first-order decomposition against
a Monte-Carlo resample of the same geometry (ADR-018). `crackvision.uncertainty_budget` (GEOM-09.4.1)
is the CLI that assembles all of that, for an already-written §10 `paths3d` file, into one report:
`data/uncertainty/{case}_uncertainty_budget.json` (schema `crackvision.uncertainty_budget/1`, full
field-by-field definition in `docs/INTERFACES.md` §14).

## What the report contains

For every valid dense point and every segment waypoint in the source `paths3d` file, the report
gives:

- A **named breakdown** of the base_link-frame position covariance into exactly five terms:
  `pixel`, `depth`, `handeye`, `tcp` (waypoints only), and `fk` — the one term this card adds,
  capturing joint-encoder/FK uncertainty that the earlier `paths3d` covariance never included.
  `contributions_m2` sums exactly to the report's own new total (`sigma_total_m` /
  `sigma_along_normal_m`), not to the source file's original `cov_base_m2` / `sigma_along_normal_m`.
- An **MC-vs-analytic agreement check** per point/waypoint: a Monte-Carlo resample of the same
  pixel→base_link chain is compared against the linearized (analytic) total via a ratio of
  covariance traces/eigenvalues (points) or along-normal sigmas (waypoints). `flagged: true` means
  that ratio fell outside the configured agreement band (default `[0.5, 2.0]`) — i.e. the
  linearization and the resample disagree enough to be worth a human look.
- `summary.overall_pass`: `true` iff nothing anywhere in the report was flagged.

**How to use it:** if a waypoint comes back `within_budget: false` (that flag is copied unchanged
from the source `paths3d` file — this report never recomputes or overrides it), look at its
`contributions_m2` to see which named term dominates the budget, and check whether its
`mc_analytic_agreement.flagged` is also `true` — if so, the linearized number driving that
`within_budget` call may itself be unreliable for that waypoint, not just large.

## Running it

```
./env.sh python -m crackvision.uncertainty_budget --cases demo_case
```

With no `--cases`, it discovers and processes every `data/paths3d/*_paths3d.json` case. Key flags
(shared with the rest of the CLI family, `docs/INTERFACES.md` §0):

- `--cases CASE_ID [CASE_ID ...]` — restrict to specific cases; default is to discover all.
- `--root PATH` / `--config PATH` — project root / config file, as elsewhere.
- `--skip-existing` — leave a case alone if its output file already exists.
- `--dry-run` — log which cases would be processed and write nothing.

Optional config file `config/calibration/uncertainty_budget.yaml` (schema
`crackvision.uncertainty_budget_config/1`, §14.6) can override the Monte-Carlo sample count, seed,
subsampling and agreement band, and the joint-encoder sigma; it need not exist — the defaults
(`mc.n_samples: 2000`, `mc.seed: 0`, `mc.subsample: "all"`, `mc.agreement_band: [0.5, 2.0]`,
`sigma_joint_rad`: from `config/robot/b601_dm_limits.yaml`'s `repeatability_rad` if present, else
the ADR-018 §3 nominal `0.001`) apply otherwise. An unknown key or an out-of-range value in that
file is a usage error (exit 2).

## Output and exit codes

The report is written to `data/uncertainty/{case}_uncertainty_budget.json` for every case that
successfully processes — **including** when `summary.overall_pass` is `false`. A flagged
MC/analytic disagreement is a diagnostic result, not a precondition failure, so the file is never
withheld because of it.

Exit codes (`docs/INTERFACES.md` §0, as implemented in `uncertainty_budget.py`):

- **0** — every requested case processed and every case's `summary.overall_pass` is `true`.
- **1** (runtime) — either some/all requested cases failed to process (e.g. a missing `paths3d`
  file or capture record), or every case processed but at least one case's `summary.overall_pass`
  is `false`. In the latter situation the report file is still written; the non-zero exit is the
  signal to go look at it, not evidence it's missing.
- **2** (usage) — malformed `--config` or a malformed/invalid `config/calibration/uncertainty_budget.yaml`.
- **3** (precondition) — no `--cases` were given and no `data/paths3d/*_paths3d.json` files exist,
  or every requested case is missing a required input file.

## Scope of what this verifies, and its limits

This report budgets only the **modelled** uncertainty sources that the `paths3d` chain already
carries plus the one new FK term this card adds: pixel localization, D405 stereo depth noise,
hand-eye calibration, TCP/pivot calibration, and joint-encoder/FK. It is **not** a measurement of
physical accuracy, and it is not a substitute for:

- the `docs/INTERFACES.md` §10.4 / ADR-018 **`unmodelled`** list: distortion Jacobian, depth bias,
  capture-time skew/motion, and thermal drift are not budgeted by any term here, exactly as
  `paths3d`'s own `uncertainty_model.unmodelled` already states;
- real hardware verification — GEOM-05/07 (camera-to-robot extrinsic, TCP calibration) and
  INT-04/05 (end-to-end accuracy on the physical robot/camera) are the steps that check whether
  the *models* this budget decomposes are themselves correct, not just internally consistent.

Camera intrinsics are a special case, stated verbatim in every report's `caveats[]`
(`docs/INTERFACES.md` §14.2/§14.3): intrinsics uncertainty is **not** a sixth additive term — it
is only indirectly exercised through the Monte-Carlo resample's nonlinear deprojection/annulus
geometry, never given its own `contributions_m2` entry.

As with `docs/geometry/PATH3D_VERIFICATION.md`'s own caveat: a clean (`overall_pass: true`) report
means the decomposition and the Monte-Carlo resample agree about what the *declared* chain's
uncertainty should be, under the nominal/measured sigmas fed into it — it says nothing about
whether those declared sigmas, or the chain's transforms themselves, match the real robot.
