# LLM-06 — Local-route canary on real low-risk cards

```json card
{
  "kind": "integration",
  "depends_on": [
    "LLM-05"
  ],
  "requirements": [
    "REQ-LLM-2"
  ],
  "spec_state": "draft",
  "refine_after": [
    "LLM-05"
  ],
  "track": "automation",
  "id": "LLM-06",
  "title": "Local-route canary on real low-risk cards",
  "parent": "L-LLM",
  "outcome": "Confirm the qualified local route produces accepted work on real cards and demotes itself on failures."
}
```

Draft: after qualification, pick 2–3 real mechanical cards and verify acceptance + ledger attribution.
