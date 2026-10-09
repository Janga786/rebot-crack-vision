# LLM-06.1 — Select real low-risk mechanical cards and flip their routing to the qualified local model

```json card
{
  "kind": "impl",
  "depends_on": [
    "LLM-05"
  ],
  "requirements": [
    "REQ-LLM-2"
  ],
  "scope": {
    "write": [
      "docs/llm/CANARY.md",
      "plan/cards/*.md"
    ]
  },
  "inputs": [
    "evidence/llm/qualification.json",
    "docs/llm/BENCHMARK_RESULTS.md",
    ".claude-auto/state.json"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "selection",
        "cmd": "python3 -c \"\nimport json, pathlib\ndoc = pathlib.Path('docs/llm/CANARY.md').read_text()\nfence = chr(96)*3\nstart = doc.index(fence + 'json canary') + len(fence + 'json canary')\nend = doc.index(fence, start)\ndata = json.loads(doc[start:end])\nids = data['selected_cards']\nassert isinstance(ids, list) and 2 <= len(ids) <= 3, ids\nstate = json.load(open('.claude-auto/state.json'))['cards']\nfor cid in ids:\n    text = pathlib.Path(f'plan/cards/{cid}.md').read_text()\n    cstart = text.index(fence + 'json card') + len(fence + 'json card')\n    cend = text.index(fence, cstart)\n    card = json.loads(text[cstart:cend])\n    r = card.get('routing', {})\n    assert r.get('task_class') == 'script', (cid, 'task_class', r)\n    assert r.get('local_ok') is True, (cid, 'local_ok', r)\n    assert r.get('consequence', 99) <= 3, (cid, 'consequence', r)\n    assert card.get('motion') is not True, (cid, 'motion excluded')\n    entry = state.get(cid, {})\n    assert entry.get('approval') is None, f'{cid} already accepted'\n    assert entry.get('status') != 'accepted', f'{cid} already accepted'\nprint('ok')\n\"",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "routing-only-diff",
        "cmd": "bash -c 'set -e; for f in $(git diff --name-only -- plan/cards/*.md); do bad=$(git diff -- \"$f\" | grep -E \"^[+-][^+-]\" | grep -v -E \"task_class|local_ok\" || true); if [ -n \"$bad\" ]; then echo \"disallowed change in $f: $bad\"; exit 1; fi; done; echo ok'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "docs/llm/CANARY.md names exactly 2 or 3 selected cards, each currently unaccepted and genuinely mechanical/low-risk/real (not manufactured just for this test), with routing.consequence<=3 and motion!=true as the card already stood before this card touched it.",
      "The diff to plan/cards/*.md touches only the routing.task_class and routing.local_ok keys of the selected cards' files; nothing else in any card file changed (not outcome, scope, acceptance, other routing dimensions, priority, etc.).",
      "docs/llm/CANARY.md's pre_state records each selected card's prior task_class/local_ok/status so the change is revertible if LLM-06.2 finds a problem.",
      "No already-accepted card's routing was altered.",
      "If fewer than 2 real qualifying candidates exist anywhere in the backlog, the card reports that honestly instead of force-fitting a bad candidate or inventing a synthetic one."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 3,
    "context": 3,
    "consequence": 3,
    "task_class": "card-routing-edit",
    "local_ok": false
  },
  "track": "automation",
  "id": "LLM-06.1",
  "parent": "LLM-06",
  "title": "Select real low-risk mechanical cards and flip their routing to the qualified local model",
  "outcome": "2–3 real, currently-unaccepted, low-risk mechanical leaf cards elsewhere in the backlog are selected; their routing.task_class is set to 'script' (the only class LLM-05 qualified) and routing.local_ok flipped to true, with nothing else in those cards changed. docs/llm/CANARY.md records the selection, rationale and pre-change state so the change is revertible."
}
```

LLM-05 qualified exactly one task class, 'script' (evidence/llm/qualification.json), for routing to the local model (Laguna-XS-2.1, served per that file's 'serving' block). No card has ever actually been attempted through the local route yet — grep .claude-auto/state.json / .claude-auto/ledger.jsonl and confirm no attempt anywhere currently names the local model or runtime. This card is the first real trial.

Scan plan/cards/*.md for leaf cards that are:
- not accepted yet: .claude-auto/state.json cards[id].approval is null AND status != 'accepted' (pending/ready/draft is fine; avoid a card someone is already mid-session on),
- genuinely mechanical and bounded: a small, deterministic script/data-wrangling/report task, motion != true, no hardware operator step, no cross-domain judgement call, routing.consequence <= 3 and routing.complexity <= 3 as the card is already written,
- real project work, not something manufactured only to exercise this canary.

Pick 2 or 3 such cards. For each: set routing.task_class = 'script' and routing.local_ok = true; change nothing else in that card file. If a candidate's existing task_class differs from 'script' (e.g. 'technical-docs'), only pick it if the actual work is genuinely script-like; otherwise keep looking. If you cannot find 2 qualifying real candidates anywhere in the backlog, stop and report that honestly — that is an acceptable, informative outcome; do not force-fit a bad candidate or invent a synthetic card.

Write docs/llm/CANARY.md with a fenced ```json canary block: {"qualification_ref": "evidence/llm/qualification.json", "selected_cards": [...], "selection_rationale": "...", "pre_state": [{"id":, "prior_task_class":, "prior_local_ok":, "status":}, ...]}. The pre_state entries let LLM-06.2 (or anyone) revert local_ok to its prior value if the canary reveals a problem. Add prose above the block explaining why each chosen card is real, mechanical, low-risk, and why it is safe to let the as-yet-untested local route attempt it for real.

Do not touch any other file, and do not start/stop the llama-server yourself — that happens automatically when the scheduler later runs the chosen cards' impl attempts.
