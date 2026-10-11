# LLM local-route canary: re-scan finds zero currently-qualifying real candidates

LLM-05 qualified exactly one task class, `script`, for routing to the local model
(Laguna-XS-2.1-Q3_K_M.gguf, served per `evidence/llm/qualification.json`'s `serving`
block). No card has ever been attempted through the local route yet (grep of
`.claude-auto/state.json` / `.claude-auto/ledger.jsonl` shows no attempt naming the
local model or runtime).

An earlier LLM-06.1 attempt (commit `04551f9`) selected `GEOM-04.1` and `HOST-05`.
Debugger D0002 directed this re-scan because `HOST-05` is now **accepted** (no
longer eligible — this card only touches currently-unaccepted cards) and because
`GEOM-04.1` needed re-checking as a canary. This attempt re-ran the full scan below
and concludes neither of those two, nor any other card in the backlog, currently
qualifies as a genuinely mechanical, bounded, low-risk real canary for the local
route.

## Scan method

Every card in `plan/cards/*.md` whose `.claude-auto/state.json` entry has
`approval == null` and `status != 'accepted'` was read directly (152 card files
total; the unaccepted subset is listed below with its own routing fields). From
that subset, the ones already excluded on their face were: every `hardware`-kind
card and every card with `motion == true` (per this card's own instructions), every
card whose `routing.consequence` is `> 3` or unset, and every `branch`/`decision`
placeholder card with no `scope`/`acceptance` of its own (`OPS-01..03`, `IL-01`,
`INT-01..05` (except `INT-04`/`INT-05`, excluded as `motion: true` anyway),
`PERC-06`, `PERC-08` are all `spec_state: draft` stubs — no routing dimensions
assigned yet, so `routing.consequence<=3` can't even be asserted).

That left eight cards with real `scope`/`acceptance` and `routing.consequence<=3`,
`motion!=true`, unaccepted: `CAM-01`, `CAM-04`, `GEOM-04.1`, `GEOM-04.2`,
`GEOM-04.4`, `GEOM-09.2.1`, `LLM-06.1`, `LLM-06.2`. Each was read in full:

- **`CAM-01`, `CAM-04`** — both `kind: hardware`, require a physically connected
  D405 camera. Excluded (hardware operator step).
- **`LLM-06.1`, `LLM-06.2`** — this canary's own parent chain. Excluded
  (self-referential; `LLM-06.2` is also still `draft` with no finished spec of its
  own, and depends on this very card).
- **`GEOM-04.1`** — re-checked per the debugger note. Its only action is
  `./env.sh pip uninstall -y opencv-python-headless && ./env.sh pip install
  opencv-contrib-python-headless==4.11.0.86` *inside the `crackvision` conda env*,
  followed by regenerating lock files and running the full suite. The prior
  attempt on this exact card (visible in its own history) was blocked because a
  sandboxed session cannot write into that conda env — i.e. completing it for real
  requires a privileged host action, not a bounded script edit a local model could
  safely attempt unsupervised. It is also not low-blast-radius in practice: it
  swaps the OpenCV build used by the whole `crackvision` env and its one acceptance
  check is the *entire* test suite (`expect_exit: 0` on `pytest tests/ -q`) — a
  wide, environment-level blast radius for a first, as-yet-unproven local-model
  trial, not the narrow single-file edit the canary concept calls for. Excluded.
- **`GEOM-04.2`, `GEOM-04.4`** — real calibration-library implementations (ChArUco
  board/detector wrapper with a layer-1 detection gate; capture-pose assembly +
  hand-eye solve CLI). Each carries genuine algorithmic/cross-domain judgement
  (3D pose geometry, hand-eye calibration math) despite `routing.consequence<=3`
  — the "small, deterministic script/data-wrangling/report task" bar this canary
  requires is not met by a from-scratch calibration algorithm implementation.
  Excluded, consistent with the prior attempt's own reasoning for the same two
  cards.
- **`GEOM-09.2.1`** — the only one of the eight that is actually unblocked right
  now (`status: ready`, its one dependency `GEOM-09.1` is accepted). But its body
  (`plan/cards/GEOM-09.2.1.md`) asks for an analytic uncertainty-propagation
  library: closed-form pixel/depth/hand-eye covariance decomposition, a
  central-finite-difference FK Jacobian per ADR-018 §3, and exact-matching
  waypoint-level variance bookkeeping against existing `lift3d`/`tool_waypoints`
  internals. `routing.task_class` is `numerical_library`, not `script`, and the
  actual work is real applied-math/robotics judgement (deriving and verifying
  closed-form covariance terms, handling interpolated-point edge cases per
  §10.3/§14.5) — not script-like in the sense LLM-05 benchmarked (`script`'s 5
  benchmark tasks were generic deterministic Python logic: case-ID string mapping,
  glob-path matching, etc., not domain-specific uncertainty-propagation math).
  Letting an as-yet-unproven-on-real-work local model attempt this for its first
  real trial risks a subtly wrong closed-form derivation that passes a shallow
  review — the exact failure mode a canary is supposed to avoid. Excluded.

## Conclusion

Zero of the eight candidates are both (a) truly ready/unblocked and unaccepted and
(b) a genuinely mechanical, bounded, low-blast-radius script/report task matching
what LLM-05 actually qualified. Per this card's own instructions ("If you cannot
find 2 qualifying real candidates anywhere in the backlog, stop and report that
honestly... do not force-fit a bad candidate or invent a synthetic card"), **no
`routing.task_class`/`routing.local_ok` edit is made by this attempt**. No
`plan/cards/*.md` routing was changed.

```json canary
{
  "qualification_ref": "evidence/llm/qualification.json",
  "selected_cards": [],
  "selection_rationale": "Re-scanned all 152 plan/cards/*.md entries; 144 are already accepted. Of the remaining 8 unaccepted, non-motion, non-hardware, routing.consequence<=3 cards with a real scope/acceptance (CAM-01, CAM-04, GEOM-04.1, GEOM-04.2, GEOM-04.4, GEOM-09.2.1, LLM-06.1, LLM-06.2), CAM-01/CAM-04 need a physically connected D405 (excluded), LLM-06.1/LLM-06.2 are this canary's own parent chain (excluded), GEOM-04.1 requires a privileged conda-env package swap plus a whole-suite-wide blast radius (excluded), GEOM-04.2/GEOM-04.4 are real calibration-algorithm implementations carrying cross-domain judgement (excluded, as the prior LLM-06.1 attempt also found), and GEOM-09.2.1 -- though actually unblocked -- is a from-scratch analytic uncertainty-propagation math library (task_class numerical_library, not script-like in the sense LLM-05 benchmarked) rather than a small deterministic script/report task (excluded). The previously-selected HOST-05 is now accepted and therefore no longer eligible. No card currently meets this canary's bar, so none is selected; forcing one would risk the exact unproven-local-model failure mode this first real trial is supposed to catch safely.",
  "pre_state": [],
  "prior_selection_now_invalid": {
    "HOST-05": "accepted since the prior LLM-06.1 attempt; no longer an unaccepted card and therefore out of scope for any routing edit here",
    "GEOM-04.1": "re-examined per debugger D0002; still excluded, now for two independent reasons (privileged conda-env action; whole-suite blast radius) rather than only the sandbox write-access limitation noted previously"
  }
}
```
