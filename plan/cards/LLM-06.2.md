# LLM-06.2 — Verify the canary cards' real outcomes: local routing, acceptance and failure demotion

```json card
{
  "kind": "integration",
  "depends_on": [
    "LLM-06.1"
  ],
  "requirements": [
    "REQ-LLM-2"
  ],
  "spec_state": "draft",
  "refine_after": [
    "LLM-06.1"
  ],
  "scope": {
    "write": [
      "docs/llm/CANARY.md",
      "evidence/llm/canary/**"
    ]
  },
  "inputs": [
    "docs/llm/CANARY.md#selected_cards",
    ".claude-auto/state.json",
    ".claude-auto/ledger.jsonl",
    "config/llm_runtime.json",
    "evidence/llm/qualification.json"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "canary-result",
        "cmd": "python3 -c \"\nimport json, pathlib\ndoc = pathlib.Path('docs/llm/CANARY.md').read_text()\nfence = chr(96)*3\nstart = doc.index(fence + 'json canary-result') + len(fence + 'json canary-result')\nend = doc.index(fence, start)\ndata = json.loads(doc[start:end])\nresults = data['results']\nassert 2 <= len(results) <= 3, results\nstate = json.load(open('.claude-auto/state.json'))['cards']\nfor res in results:\n    cid = res['card_id']\n    assert cid in state, cid\n    entry = state[cid]\n    assert res['final_status'] == entry.get('status'), (cid, res['final_status'], entry.get('status'))\n    assert isinstance(res.get('attempt_ids'), list) and len(res['attempt_ids']) >= 1, cid\n    assert res.get('observed_local_route') in (True, False), cid\n    assert res.get('evidence_path'), cid\n    assert pathlib.Path(res['evidence_path']).exists(), res['evidence_path']\nprint('ok')\n\"",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Every claim of 'ran on the local model' or 'fell back to a non-local model' cites a specific attempt id and an evidence artefact under evidence/llm/canary/, not just a bare status field.",
      "Each result's final_status matches the live .claude-auto/state.json status at the time of writing — no stale, anticipated, or guessed results.",
      "If none of the canaries happened to fail locally, the report says so plainly rather than claiming the demotion half of REQ-LLM-2 is proven; demotion is never simulated or forced to manufacture evidence.",
      "The report gives a plain yes/no on whether the qualified local route is safe to rely on more broadly, with the specific evidence behind that answer.",
      "If the canary cards have not yet reached a terminal/near-terminal state, the card reports that honestly as a stalled canary rather than fabricating a result."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 4,
    "consequence": 3,
    "task_class": "observability-report",
    "local_ok": false
  },
  "track": "automation",
  "id": "LLM-06.2",
  "parent": "LLM-06",
  "title": "Verify the canary cards' real outcomes: local routing, acceptance and failure demotion",
  "outcome": "For each card LLM-06.1 routed to the local model, evidence is produced showing which attempt(s) actually ran on the local runtime, their final status, and — if any local attempt failed — whether the system fell back to a non-local model and still reached a sane terminal state. docs/llm/CANARY.md's canary-result block gives a sourced, honest answer on whether REQ-LLM-2's local route works in production, including what (if anything) about the failure/demotion path remains unverified."
}
```

LLM-06.1 selected 2–3 real cards and flipped their routing to the local model. Run this card only after those cards have actually been attempted by the scheduler — wait for their .claude-auto/state.json status to move past pending/ready/draft (accepted, reviewing, changes_requested, blocked, etc.) before writing results; if after a reasonable wait none have moved, report that truthfully as a stalled canary rather than inventing results.

For each selected card, using .claude-auto/state.json (cards[id].attempts[], approval, status) and .claude-auto/ledger.jsonl, determine:
- which attempt(s) ran for it, and whether any attempt actually used the local runtime (config/llm_runtime.json's llama-server / the model name from evidence/llm/qualification.json, e.g. 'Laguna-XS-2.1') rather than claude-sonnet-5/haiku/opus — cite the exact attempt id and the field(s) you read as evidence, and copy or excerpt the relevant record (ledger line, attempt log, llama-server log if one exists) into evidence/llm/canary/<card-id>.json or .log so the claim is checkable later;
- the final status/outcome (accepted, still cycling, demoted, etc.);
- if a local attempt failed, or a later attempt on the same card used a different (non-local) model, whether that was an automatic fallback that let the card still reach a sane terminal state — this is the 'demotes itself on failures' half of LLM-06's outcome. If none of the 2–3 canaries happened to fail locally, say so plainly; do not simulate or force a failure to manufacture evidence that wasn't observed, and do not claim the demotion path is verified when it wasn't exercised.

Append a fenced ```json canary-result block to docs/llm/CANARY.md: {"results": [{"card_id":, "attempt_ids": [...], "observed_local_route": true|false, "final_status":, "evidence_path":, "notes": "..."}, ...], "demotion_observed": true|false, "demotion_notes": "..."}. Add prose summarizing, for the REQ-LLM-2 record, whether the local route as deployed produces accepted work on real cards, and what (if anything) remains unverified about the failure/demotion path.
