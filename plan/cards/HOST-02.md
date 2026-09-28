# HOST-02 — Lock the crackvision environment + verify-lock script

```json card
{
  "kind": "impl",
  "depends_on": [
    "TC-002"
  ],
  "requirements": [
    "REQ-HOST-1"
  ],
  "scope": {
    "write": [
      "requirements/crackvision.conda-explicit.txt",
      "requirements/crackvision.pip-freeze.txt",
      "scripts/verify_lock.py",
      "scripts/rebuild_env.sh",
      "docs/host/ENV_LOCK.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "verify",
        "cmd": "./env.sh python scripts/verify_lock.py"
      },
      {
        "id": "dry-run",
        "cmd": "bash scripts/rebuild_env.sh --dry-run"
      }
    ],
    "criteria": [
      "Lock files generated from the live env (conda list --explicit; pip freeze) with a header noting date and source",
      "verify_lock.py exits 0 when the env matches, 1 with a diff otherwise",
      "rebuild_env.sh --dry-run prints the exact commands to recreate the env under a NEW name; it never touches existing envs"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 2,
    "task_class": "script"
  },
  "priority": 40,
  "extra_rw": [
    "~/.conda"
  ],
  "id": "HOST-02",
  "title": "Lock the crackvision environment + verify-lock script",
  "parent": "L-HOST",
  "outcome": "The perception environment can be recreated exactly from committed lock files, and drift is detectable."
}
```


