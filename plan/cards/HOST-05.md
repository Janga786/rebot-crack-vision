# HOST-05 — Storage decision for the unmounted 1.9 TB NVMe (K1_Storage)

```json card
{
  "kind": "decision",
  "requirements": [
    "REQ-HOST-1"
  ],
  "scope": {
    "write": [
      "docs/host/STORAGE.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "doc",
        "cmd": "test -f docs/host/STORAGE.md"
      }
    ],
    "criteria": [
      "STORAGE.md states facts (sizes, free space, label, not in fstab, possible K1-project ownership) and the operator's recorded decision",
      "If mounting was chosen, it was done via the privileged path with a reversible method"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 3,
    "context": 2,
    "consequence": 3
  },
  "priority": 15,
  "id": "HOST-05",
  "title": "Storage decision for the unmounted 1.9 TB NVMe (K1_Storage)",
  "parent": "L-HOST",
  "outcome": "An operator-approved decision on where datasets, rosbags and LLM weights live, applied safely."
}
```

Collect facts read-only (lsblk -f, df -h, du of data/models dirs). nvme0n1 is labelled K1_Storage and may hold another project's data:
do NOT mount or modify it on your own. Write the options into docs/host/STORAGE.md, then return status "blocked" with blocker type
"operator_decision" asking whether and where to mount it. When resumed with the operator's note, apply that decision.
