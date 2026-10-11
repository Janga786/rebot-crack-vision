# LLM local-route canary: re-scan (attempt 2) still finds zero currently-qualifying real candidates

LLM-05 qualified exactly one task class, `script`, for routing to the local model
(Laguna-XS-2.1-Q3_K_M.gguf, served per `evidence/llm/qualification.json`'s `serving`
block). No card has ever been attempted through the local route yet: grepping
`.claude-auto/ledger.jsonl` for `laguna`/`llama-server`/`local_ok.*true` only turns
up this card's own debugger action, never an actual attempt run against the
local model/runtime.

This is attempt 2 of this card. Attempt 1 (commit `e777ba5`) re-scanned per
debugger D0002's note (HOST-05 now accepted, GEOM-04.1 re-checked) and concluded
zero of 8 enumerated unaccepted/non-hardware/non-motion/`consequence<=3` cards
qualified. That check failed its own acceptance script (which requires 2-3
`selected_cards`), per this card's rule 3: a truthful blocked/zero result still
has to pass the mechanical check or be reported honestly as blocked rather than
forced. This attempt independently redid the full scan (including one candidate,
`MOT-06.10`, that attempt 1's enumeration missed) and reaches the same
conclusion for an expanded, stronger-justified reason set.

## Scan method

`.claude-auto/state.json` was read directly for every card's `approval`/`status`.
Every one of the 152 files under `plan/cards/*.md` was parsed for its `routing`,
`kind`, and `motion` fields. Filtering to unaccepted (`approval == null` and
`status != 'accepted'`), `kind != 'hardware'`, `motion is not True`, and
`routing.consequence <= 3` leaves exactly nine candidates (branch/decision/draft
stub cards such as `OPS-01..03`, `IL-01`, `INT-01..03`, `PERC-06`, `PERC-08` have
no routing dimensions assigned yet and are excluded on that basis alone):

`CAM-01`, `CAM-04`, `GEOM-04.1`, `GEOM-04.2`, `GEOM-04.4`, `GEOM-09.2.1`,
`LLM-06.1`, `LLM-06.2`, and `MOT-06.10`.

- **`CAM-01`, `CAM-04`** — `kind: hardware`, require a physically connected D405.
  Excluded (hardware operator step).
- **`LLM-06.1`, `LLM-06.2`** — this canary's own parent chain (`LLM-06.1` is this
  very card, mid-session; `LLM-06.2` is still `draft` with no finished spec and
  depends on this card). Excluded (self-referential).
- **`GEOM-04.1`** — status `blocked`. Its only action is a `./env.sh pip
  uninstall/install` swap of the pinned OpenCV build *inside the shared
  `crackvision` conda env itself*, not a bounded file edit — a sandboxed session
  cannot durably alter that env, and its one check runs the entire test suite, a
  whole-environment blast radius for a first, unproven local-model trial.
  Excluded (privileged host action + environment-wide blast radius).
- **`GEOM-04.2`, `GEOM-04.4`** — real from-scratch computer-vision/calibration
  algorithm work (ChArUco detector wrapper with a PnP pose solve and a synthetic
  perspective-warp renderer; hand-eye capture-pose assembly). Both also
  `depends_on` the still-unaccepted, currently-`blocked` `GEOM-04.1`, so neither
  is even genuinely startable yet. Despite `routing.consequence<=3`, the actual
  work is real geometry/vision judgement, not the generic deterministic
  string/path-matching logic LLM-05's `script` benchmark tasks tested. Excluded
  (not ready; not script-like).
- **`GEOM-09.2.1`** — the only one of the nine that is truly unblocked
  (`status: ready`, dependency `GEOM-09.1` accepted). Its body asks for an
  analytic uncertainty-propagation library: closed-form pixel/depth/hand-eye
  covariance decomposition and a central-finite-difference FK Jacobian, matched
  exactly against existing `lift3d`/`tool_waypoints` internals per ADR-018 §3.
  `routing.task_class` is `numerical_library`, not `script`, and the actual work
  is applied-math/robotics derivation, not script-like. Excluded.
- **`MOT-06.10`** (missed by attempt 1's enumeration; added here) — `status:
  pending`, `depends_on: [MOT-06.7, MOT-06.9, GEOM-08.8]`, of which `MOT-06.7` and
  `MOT-06.9` are only `ready` (not yet accepted) — so this card is not actually
  startable real work yet either. Even setting that aside, its task is a
  cross-domain ROS 2 + motion-planning + perception end-to-end integration run
  (build the ROS workspace, drive a mock robot/perception stack, assert plan
  coverage/eligibility semantics, then write up the observed run) — exactly the
  kind of cross-domain judgement call this card's own instructions exclude, not
  the small deterministic script/data-wrangling/report work LLM-05 benchmarked.
  Excluded (dependencies not accepted; not script-like).

`GEOM-09.2` (`status: active` — someone else may be mid-session on it; also the
same non-script-like `numerical_library` uncertainty-math work as `GEOM-09.2.1`)
and `GEOM-09.3`/`GEOM-09.4` (both `spec_state: draft`, no finished spec, same
`numerical_library`/math-judgement nature) were also checked and excluded for the
same combination of reasons, for completeness, even though their
`routing.consequence` (2) would otherwise pass the filter.

## Conclusion

Zero of the nine (now ten, counting `GEOM-09.2`) candidates examined are both (a)
genuinely ready/unblocked, unaccepted, real work and (b) a small, deterministic,
script-like task matching what LLM-05 actually qualified. Per this card's own
instructions ("If you cannot find 2 qualifying real candidates anywhere in the
backlog, stop and report that honestly... do not force-fit a bad candidate or
invent a synthetic card"), no `routing.task_class`/`routing.local_ok` edit is
requested by this attempt. This is reported as a blocked outcome rather than a
false "complete": the honest answer does not satisfy the acceptance script's
`2 <= len(selected_cards) <= 3` assertion, and forcing a selection to pass that
assertion would mean routing real, unproven-locally work to the local model
against a card the backlog does not actually support yet — the exact risk this
canary exists to avoid on the local route's first real trial.

```json canary
{
  "qualification_ref": "evidence/llm/qualification.json",
  "selected_cards": [],
  "selection_rationale": "Re-scanned all 152 plan/cards/*.md entries against .claude-auto/state.json; of the unaccepted, non-hardware, motion!=true, routing.consequence<=3 cards (CAM-01, CAM-04, GEOM-04.1, GEOM-04.2, GEOM-04.4, GEOM-09.2.1, LLM-06.1, LLM-06.2, MOT-06.10, plus GEOM-09.2/09.3/09.4 checked for completeness), every one is excluded for at least one of: requires a physically connected D405 (CAM-01/CAM-04); is this canary's own self-referential parent chain (LLM-06.1/LLM-06.2); requires a privileged, environment-wide conda-env package swap rather than a bounded file edit (GEOM-04.1, status blocked); depends on that still-blocked GEOM-04.1 so is not yet startable, and/or is real from-scratch computer-vision/calibration or uncertainty-propagation algorithm work rather than the generic deterministic script logic LLM-05 benchmarked (GEOM-04.2, GEOM-04.4, GEOM-09.2.1, GEOM-09.2, GEOM-09.3, GEOM-09.4); or has unaccepted dependencies plus is a cross-domain ROS/motion-planning/perception integration run rather than a small script/report task (MOT-06.10). No card currently meets this canary's bar of being both genuinely ready and genuinely script-like/low-judgement, so none is selected; forcing one would risk the exact unproven-local-model failure mode this first real trial is supposed to catch safely.",
  "pre_state": [],
  "prior_selection_now_invalid": {
    "HOST-05": "accepted since the earlier (pre-attempt-1) LLM-06.1 selection; no longer an unaccepted card and therefore out of scope for any routing edit here",
    "GEOM-04.1": "re-examined per debugger D0002; status is blocked and its action is a privileged, environment-wide conda package swap inside the shared crackvision env, not a bounded file edit a first unproven local-model trial should attempt"
  }
}
```
