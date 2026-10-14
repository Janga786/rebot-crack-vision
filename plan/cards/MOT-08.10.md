# MOT-08.10 — approve_trajectory: open /dev/tty as separate read/write handles so the typed APPROVE prompt works on a real (pseudo-)terminal

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-08.4"
  ],
  "requirements": [
    "REQ-OPS-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/approve_trajectory.py",
      "tests/test_approve_trajectory.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "src/crackvision/approve_trajectory.py",
    "tests/test_approve_trajectory.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_approve_trajectory.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "no-bypass",
        "cmd": "bash -c '! grep -nE \"(--yes|--force|--non-interactive|sys\\.stdin|input\\()\" src/crackvision/approve_trajectory.py && grep -q \"/dev/tty\" src/crackvision/approve_trajectory.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "no-r-plus-tty",
        "cmd": "bash -c '! grep -nE \"open\\([^)]*/dev/tty[^)]*r\\+\" src/crackvision/approve_trajectory.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "no-tty-refuses",
        "cmd": "bash -c 'setsid -w ./env.sh python -m crackvision.approve_trajectory --preview /nonexistent/preview.json --operator ci </dev/null; rc=$?; test $rc -eq 2 -o $rc -eq 3'",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The default tty reader opens /dev/tty directly as two separate text handles (one for reading, one for writing) instead of a single 'r+' handle. Both are closed in a finally block, and the prompt text stays the same (trajectory path, purpose, duration at the preview speed scale, manifest path).",
      "A new test runs make_tty_reader's reader in a child process whose controlling terminal is a real pseudo-terminal (pty.fork or util-linux `script`), with no monkeypatching of _open_tty. It writes 'APPROVE <sha8>' to the pty master and asserts that the reader returns that line and that confirm() is True. A second real-pty case with a wrong phrase asserts confirm() is False.",
      "Having no controlling terminal is still a refusal: main() exits 3 and writes no file. The confirmation is still read only from /dev/tty, and no flag, env var or stdin path bypasses it. All existing MOT-08.4 tests still pass unchanged."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 1,
    "context": 2,
    "consequence": 5,
    "task_class": "python-cli-safety",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.10",
  "title": "approve_trajectory: open /dev/tty as separate read/write handles so the typed APPROVE prompt works on a real (pseudo-)terminal",
  "parent": "MOT-08",
  "outcome": "`crackvision.approve_trajectory` can read the typed APPROVE <sha8> phrase from a real terminal or pseudo-terminal. A real-pty regression test proves this, and every §18 refusal still holds.",
  "source": "debugger D0013"
}
```

Bug found by MOT-08.8 (attempt 01320): src/crackvision/approve_trajectory.py:228 `_open_tty()` does `open("/dev/tty", "r+", encoding="utf-8")`. Text-mode 'r+' builds an io.BufferedRandom, which requires a seekable raw stream, and tty/pty devices are not seekable. So the open always raises io.UnsupportedOperation (an OSError subclass). confirm() turns that into a refusal, which means approval can never succeed on any terminal. Reproduced with `script -qec "./env.sh python -c 'open(\"/dev/tty\",\"r+\",encoding=\"utf-8\")'" /dev/null`.

Fix: in make_tty_reader's reader, open `/dev/tty` once with 'r' and once with 'w' (both encoding='utf-8'). Write the prompt to the write handle and flush, readline from the read handle, and close both in `finally`. If you keep `_open_tty`, make it return a (reader, writer) pair, and update test_no_tty_refuses so it still raises OSError from it. An OSError from either open must still be a refusal.

Regression test: use `pty.fork()` (or `os.openpty()` plus a child that calls `os.setsid()` and opens the slave as its controlling tty). In the child, import the module by file path, build the reader with make_tty_reader(...), call confirm(sha8, reader) and exit 0 or 1 based on the result. In the parent, write `APPROVE <sha8>\n` to the master fd (drain the prompt output first so the buffer cannot fill) and assert the child's exit status. Add a second case with a wrong phrase. Keep it tmp_path-only and add a timeout so the test cannot hang.

Do not add any stdin/flag/env bypass. Do not touch scripts/ros/test_preview_approve.sh (MOT-08.8 owns it).
