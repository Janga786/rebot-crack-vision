# LLM local-route canary: first two real cards selected

LLM-05 qualified exactly one task class, `script`, for routing to the local model
(Laguna-XS-2.1-Q3_K_M.gguf, served per `evidence/llm/qualification.json`'s `serving`
block). No card has ever been attempted through the local route yet (grep of
`.claude-auto/state.json` / `.claude-auto/ledger.jsonl` shows no attempt naming the
local model or runtime). This card selects the first real cards to try it on.

Every pending/ready/draft leaf card in `plan/cards/*.md` with `motion != true` and
`routing.consequence <= 3` was scanned (8 total: CAM-01, CAM-04, GEOM-04.1, GEOM-04.2,
GEOM-04.4, HOST-05, LLM-06.1, LLM-06.2). LLM-06.1/LLM-06.2 are this canary's own
parent chain and excluded. CAM-01/CAM-04 require a physically connected D405 camera
(`hardware: ["d405_connected", ...]`) — a hardware operator step, excluded by this
card's own instructions. GEOM-04.2/GEOM-04.4 are full calibration-library
implementations (ChArUco pose solving, perspective-warp synthetic rendering, hand-eye
CLI assembly) carrying real algorithmic/cross-domain judgement despite a routing
complexity of 3 — not the "small, deterministic script/data-wrangling/report task"
this canary is meant to risk; excluded.

That leaves exactly two real, qualifying candidates:

- **GEOM-04.1** — swap one pinned pip package (`opencv-python-headless` →
  `opencv-contrib-python-headless==4.11.0.86`), regenerate the two documented lock
  files via the already-specified `./env.sh pip freeze` procedure, append one dated
  note to `docs/host/ENV_LOCK.md` and one line to `docs/COMPLETION_LOG.md`, and run
  the existing test suite. Every step is mechanical and deterministic: no new design,
  no camera/robot hardware, `routing.consequence=3`. Its one past attempt was blocked
  on a privileged-action denial (the sandboxed session can't write into the
  `crackvision` conda env), not on the task's design, so it is still real, unaccepted,
  low-risk work — a reasonable first local-route trial even if it re-hits the same
  sandbox limitation and reports blocked again.
- **HOST-05** — the operator already recorded an authoritative decision in this
  card's feedback (`leave K1_Storage COMPLETELY UNTOUCHED ... record this decision
  ... in docs/host/STORAGE.md`). What remains is exactly the "small, deterministic
  ... report task" this canary wants: write the already-decided facts (disk sizes,
  free space, the LLM-03 40 GB floor) into one markdown file, with no further
  judgement call and no privileged action. `routing.consequence=3`.

For each, `routing.task_class` is set to `script` (the only LLM-05-qualified class)
and `routing.local_ok` to `true`; nothing else in either card file changes. This is
safe because both tasks are fully mechanical/deterministic, both are flagged
low-risk (`consequence<=3`) under the project's own routing dimensions, and the
`pre_state` below lets `local_ok` and `task_class` be reverted to their prior values
in one step if this first real local-route trial surfaces a problem.

## Known blocker — reported, not silently worked around

This sandbox session mounts `plan/` (and `task_cards/`, `.claude-auto/`) read-only
regardless of a card's declared `scope.write`, per this session's own environment
instructions ("Writable: the project repo (except .git/, plan/, task_cards/ and
.claude-auto/)"). `plan/cards/GEOM-04.1.md` and `plan/cards/HOST-05.md` could
therefore **not** be edited from here even though this card's scope grants
`plan/cards/*.md`. This file (`docs/llm/CANARY.md`, under `docs/`, which is
writable) records the intended selection and the exact `routing.task_class`/
`routing.local_ok` edits below so the scheduler or a session with `plan/` write
access can apply them.

```json canary
{
  "qualification_ref": "evidence/llm/qualification.json",
  "selected_cards": ["GEOM-04.1", "HOST-05"],
  "selection_rationale": "Of 8 pending/ready/draft leaf cards with motion!=true and routing.consequence<=3, CAM-01/CAM-04 need a physically connected D405 (hardware operator step, excluded), GEOM-04.2/GEOM-04.4 are real algorithmic calibration-library implementations (not a bounded script/report task despite complexity=3, excluded), and LLM-06.1/LLM-06.2 are this canary's own parent chain (excluded). GEOM-04.1 (swap one pinned pip package, regenerate lock files, run the suite -- fully mechanical, consequence=3) and HOST-05 (write an already operator-decided storage outcome into one doc -- a pure report task now that the judgement call is made, consequence=3) are the only two real, currently-unaccepted, low-risk mechanical candidates found anywhere in the backlog.",
  "pre_state": [
    {"id": "GEOM-04.1", "prior_task_class": "dependency-fix", "prior_local_ok": false, "status": "blocked"},
    {"id": "HOST-05", "prior_task_class": null, "prior_local_ok": null, "status": "ready"}
  ],
  "intended_edit": {
    "GEOM-04.1": {"routing.task_class": "script", "routing.local_ok": true},
    "HOST-05": {"routing.task_class": "script", "routing.local_ok": true}
  },
  "blocker": "This sandbox session mounts plan/ read-only (per its own environment instructions) regardless of this card's declared scope.write=plan/cards/*.md, so plan/cards/GEOM-04.1.md and plan/cards/HOST-05.md could not actually be edited from here. The edits above are the ones that still need to be applied, by the scheduler or by a session with plan/ write access, to complete this card."
}
```
