"""python -m arm_guard {check|approve|install-hook|uninstall-hook|hook-status|events}"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
from pathlib import Path

from . import config as C
from . import core
from .errors import GuardConfigError

SRC_DIR = Path(__file__).resolve().parents[1]
PTH_NAME = "arm_guard.pth"
KNOWN_PYTHONS = [Path.home() / "miniconda3/envs" / e / "bin/python" for e in ("rebot", "lerobot", "crackvision")]


def _summary(cfg) -> None:
    print(f"config  {cfg.path}\n  sha256 {cfg.sha256}\nurdf    {cfg.urdf_path}\n  sha256 {cfg.urdf_sha256}")
    print(f"{'joint':6} {'id':>4} {'model':6} {'lower':>7} {'upper':>7} {'vcap':>5} {'step':>5} {'kp_max':>6} "
          f"{'tau_ff':>6} {'tq_abort':>8}")
    for j in cfg.joints.values():
        print(f"{j.name:6} {j.motor_id:>#4x} {j.model:6} {j.lower:>7.3f} {j.upper:>7.3f} {j.vel_cap:>5.2f} "
              f"{j.max_step:>5.2f} {j.kp_max:>6.1f} {j.tau_ff_max:>6.1f} {j.torque_abort:>8.1f}"
              + (f"  BEYOND URDF: {j.beyond_urdf_reason}" if j.beyond_urdf_reason else ""))
    rest = {j.urdf_joint: 0.0 for j in cfg.arm_joints}
    worst = min(core.pose_margins(cfg, rest).items(), key=lambda kv: kv[1])
    print(f"keep-out at the fold (all zero): tightest {worst[0]} {worst[1] * 1000:+.1f} mm")


def cmd_check(a) -> int:
    try:
        cfg = C.load(a.config)
    except GuardConfigError as e:
        print(f"FAIL: {e}")
        return 1
    _summary(cfg)
    print(f"approved: {json.dumps(C.check_approved(cfg))}")
    return 0


def cmd_approve(a) -> int:
    cfg = C.load(a.config, check_approval=False)
    _summary(cfg)
    if not a.yes:
        if input("\nType 'approve' to approve exactly this config + URDF: ").strip() != "approve":
            print("not approved")
            return 1
    rec = {"config_sha256": cfg.sha256, "urdf_sha256": cfg.urdf_sha256, "approved_by": a.by,
           "approved_at": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"), "note": a.note}
    path = C.approval_path(Path(cfg.path))
    path.write_text(json.dumps(rec, indent=2) + "\n")
    print(f"wrote {path}")
    return 0


def _purelib(python: str) -> Path:
    out = subprocess.run([python, "-I", "-c", "import sysconfig;print(sysconfig.get_paths()['purelib'])"],
                         capture_output=True, text=True, check=True)
    return Path(out.stdout.strip())


def cmd_install_hook(a) -> int:
    for py in a.python:
        pth = _purelib(py) / PTH_NAME
        pth.write_text(f"{SRC_DIR}\nimport arm_guard_boot\n")
        print(f"installed {pth}")
    return cmd_hook_status(a)


def cmd_uninstall_hook(a) -> int:
    for py in a.python:
        pth = _purelib(py) / PTH_NAME
        if pth.exists():
            pth.unlink()
            print(f"removed {pth}")
    return 0


PROBE = r"""
import json, sys
r = {"python": sys.executable}
try:
    import motorbridge.core as mc
    r["motorbridge"] = getattr(__import__("motorbridge"), "__version__", "?")
    r["patched"] = bool(getattr(mc.Motor, "_arm_guard_patched", False) and getattr(mc.Controller, "_arm_guard_patched", False))
    import arm_guard
    s = arm_guard.status()
    r["config_ok"] = s["config"] is not None
    r["config_error"] = s["config_error"]
except Exception as e:
    r["error"] = repr(e)
print(json.dumps(r))
"""


def cmd_hook_status(a) -> int:
    bad = 0
    for py in a.python:
        if not Path(py).exists():
            continue
        pth = _purelib(py) / PTH_NAME
        out = subprocess.run([py, "-c", PROBE], capture_output=True, text=True, env={"PATH": "/usr/bin:/bin",
                             "HOME": str(Path.home())})
        try:
            r = json.loads(out.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            r = {"error": out.stderr.strip()[-300:]}
        has_mb = "error" not in r or "motorbridge" not in r.get("error", "")
        ok = pth.exists() and r.get("patched") and r.get("config_ok")
        bad += 0 if ok else 1
        print(f"{'OK  ' if ok else 'WARN'} {py}  hook={'yes' if pth.exists() else 'no'}  {json.dumps(r)}"
              + ("" if has_mb else "  (no motorbridge here)"))
    return 0 if bad == 0 else 2


def cmd_events(a) -> int:
    from .runtime import EVENT_LOG
    if not EVENT_LOG.exists():
        print(f"no events yet ({EVENT_LOG})")
        return 0
    for line in EVENT_LOG.read_text().splitlines()[-a.n:]:
        print(line)
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m arm_guard")
    p.add_argument("--config", default=None, help=f"default: ${C.CONFIG_ENV} or {C.DEFAULT_CONFIG}")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="validate config + approval, print the enforced limits")
    ap = sub.add_parser("approve", help="(operator) approve the current config + URDF hashes")
    ap.add_argument("--by", required=True)
    ap.add_argument("--note", default="")
    ap.add_argument("--yes", action="store_true", help="skip the typed confirmation")
    for name in ("install-hook", "uninstall-hook", "hook-status"):
        sp = sub.add_parser(name)
        sp.add_argument("--python", nargs="+", default=[str(x) for x in KNOWN_PYTHONS])
    ev = sub.add_parser("events", help="tail the guard event log")
    ev.add_argument("-n", type=int, default=30)
    a = p.parse_args(argv)
    return {"check": cmd_check, "approve": cmd_approve, "install-hook": cmd_install_hook,
            "uninstall-hook": cmd_uninstall_hook, "hook-status": cmd_hook_status, "events": cmd_events}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
