"""crackvision.first_run — the operator's first crack run on the powered arm (docs/motion/FIRST_CRACK_RUN.md).

Four commands, all reached through scripts/crack_pipeline.sh:

  preflight         one go/no-go at the desk: project env, model, guard approval and hooks, the runner files (sha256
                    recorded), nothing holding the D405 or the arm, stale pid/state files cleaned, disk, the D405 on
                    USB 3 with a short validated capture, search.json/scan.json regenerated and rehearsed x1 and x2
                    with the real guard config, the print PDF, the crack gate switched on. Prints PASS/WARN/FAIL with
                    a reason and a fix per row, writes data/crack_trace/preflight_<UTC>.json; exit 0 only with no FAIL.
  bench [CASE]      the print under the camera, ARM OFF: the same burst + crack_filter path as crack_trace capture
                    (crackvision.camera_pipeline.filter_burst), captured now, with no state file and no settle wait.
                    Prints the component table in plain words and a verdict, writes an overlay.
                    `bench --live`: the filtered live view (camera.py live --filter) for aiming.
  session CASE [slow]  search -> scan-plan --from-search -> scan -> plan -> (the operator looks at the preview) ->
                    trace, a typed `yes` before each step. It only ever runs the existing crack_pipeline.sh
                    subcommands; every arm step goes through ~/rebot_setup/run-trajectory.sh as before.
  archive CASE      copies (never moves) the case's files, the runner's run CSV/JSON, the preflight record and the
                    guard events into evidence/operator/first-crack-run-<date>/ with sha256s and a README template.

Nothing here opens the CAN bus or talks to a motor. The only runner call is `run-trajectory.sh --rehearse`
(simulation) in preflight; session hands the terminal to crack_pipeline.sh, which calls the runner as it always has.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable

import numpy as np

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE, setup_logging, utc_stamp

PREFLIGHT_SCHEMA = "crackvision.first_run_preflight/1"
BENCH_SCHEMA = "crackvision.first_run_bench/1"
SESSION_SCHEMA = "crackvision.first_run_session/1"
ARCHIVE_SCHEMA = "crackvision.first_run_archive/1"
D405_USB_ID = ("8086", "0b5b")
DEFAULT_DEMO_EXTRA = "--gains j2=100/4,j3=80/4"
DEFAULT_SLOW = "2"
RUNNER_FILES = ("run-trajectory.sh", "demo_move.py", "first_move_j1.py", "gravity.py", "fakebus.py")
PRINT_PDFS = ("crack_prints.pdf", "crack_test_prints.pdf")      # the file on disk / the name the tool writes today
PREFLIGHT_MAX_AGE_H = 12.0
DISK_WARN_GB, DISK_FAIL_GB = 10.0, 2.0
RUNNER_PROC_HINTS = ("demo_move.py", "run-trajectory.sh", "basic-demo.sh", "motor_cli", "ws_gateway", "-m lerobot",
                     "lerobot-", "rebotarmcontroller")


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def repo_root() -> Path:
    return Path(os.environ.get("CRACKVISION_ROOT") or Path(__file__).resolve().parents[2])


def runner_dir() -> Path:
    return Path(os.environ.get("REBOT_RUNNER") or Path.home() / "rebot_setup")


def trace_dir(root: Path) -> Path:
    return Path(root) / "data" / "crack_trace"


# ---- preflight ---------------------------------------------------------------------------------------------------

@dataclass
class Check:
    name: str
    status: str                       # PASS | WARN | FAIL
    reason: str
    fix: str = ""
    data: dict = field(default_factory=dict)


def _pass(name, reason, **data) -> Check:
    return Check(name, "PASS", reason, "", data)


def _warn(name, reason, fix="", **data) -> Check:
    return Check(name, "WARN", reason, fix, data)


def _fail(name, reason, fix="", **data) -> Check:
    return Check(name, "FAIL", reason, fix, data)


def run_cmd(cmd: list[str], cwd: Path | None = None, timeout: float = 600.0) -> tuple[int, str]:
    """(exit code, stdout+stderr). Never raises for a failing command."""
    try:
        p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 127, f"{type(exc).__name__}: {exc}"
    return p.returncode, (p.stdout or "") + (p.stderr or "")


def proc_list(proc_root: Path = Path("/proc")) -> list[tuple[int, str]]:
    """(pid, command line) of every process we can read."""
    out = []
    for d in proc_root.iterdir():
        if not d.name.isdigit():
            continue
        try:
            cmd = (d / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
        if cmd:
            out.append((int(d.name), cmd))
    return out


def pid_alive(pid: int, proc_root: Path = Path("/proc")) -> str | None:
    """The command line of pid if it is running, else None."""
    try:
        return (proc_root / str(pid) / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip() or "?"
    except OSError:
        return None


def d405_nodes(sys_root: Path = Path("/sys"), usb_id: tuple[str, str] = D405_USB_ID) -> list[str]:
    """The D405's /dev/videoN nodes and its /dev/bus/usb/BBB/DDD node, from sysfs (empty when it is not attached)."""
    nodes: set[str] = set()
    v4l = Path(sys_root) / "class" / "video4linux"
    if not v4l.is_dir():
        return []
    for v in sorted(v4l.iterdir()):
        try:
            dev = (v / "device").resolve()
        except OSError:
            continue
        for p in (dev, *dev.parents):
            vid = p / "idVendor"
            if vid.is_file():
                try:
                    ids = (vid.read_text().strip().lower(), (p / "idProduct").read_text().strip().lower())
                    if ids == usb_id:
                        nodes.add(f"/dev/{v.name}")
                        nodes.add(f"/dev/bus/usb/{int((p / 'busnum').read_text()):03d}/"
                                  f"{int((p / 'devnum').read_text()):03d}")
                except (OSError, ValueError):
                    pass
                break
    return sorted(nodes)


def device_holders(nodes: Iterable[str], proc_root: Path = Path("/proc"),
                   exclude: Iterable[int] = ()) -> list[dict]:
    """Processes with one of `nodes` open: [{pid, cmd, node}]. Processes we cannot inspect are skipped."""
    nodes = set(nodes)
    skip = set(exclude) | {os.getpid()}
    found = []
    if not nodes:
        return found
    for d in Path(proc_root).iterdir():
        if not d.name.isdigit() or int(d.name) in skip:
            continue
        try:
            fds = list((d / "fd").iterdir())
        except OSError:
            continue
        for fd in fds:
            try:
                target = os.readlink(fd)
            except OSError:
                continue
            if target in nodes:
                found.append({"pid": int(d.name), "cmd": pid_alive(int(d.name), proc_root) or "?", "node": target})
                break
    return found


def camera_holder_text(holders: list[dict]) -> str:
    return "; ".join(f"pid {h['pid']} ({h['cmd'][:70]}) has {h['node']} open" for h in holders)


def parse_rehearsal(out: str, rc: int) -> dict:
    m = re.search(r"keep-out margin >= (\d+) mm", out)
    gap = re.search(r"max gap ([\d.]+) ms", out)
    summ = re.search(r"^summary: (\S+)", out, re.M)
    outcome = re.search(r"^outcome: (\S+)", out, re.M)
    refused = [ln.strip() for ln in out.splitlines() if re.search(r"REFUS|GUARD FAULT", ln)]
    return {"rc": rc, "outcome": outcome.group(1) if outcome else None,
            "completed": bool(outcome and outcome.group(1) == "completed" and rc == 0),
            "keepout_mm": int(m.group(1)) if m else None, "max_gap_ms": float(gap.group(1)) if gap else None,
            "summary": summ.group(1) if summ else None, "refusals": refused[:5]}


def looks_like_operator_name(approved_by: str) -> bool:
    """An approval record names a person: short, no 'not reviewed / offline / test / pending / agent' wording."""
    s = (approved_by or "").strip()
    if not s or len(s) > 40 or len(s.split()) > 3:
        return False
    return not re.search(r"\b(not|no|offline|test|testing|pending|review|reviewed|unreviewed|claude|agent|auto|"
                         r"ci|bot|tbd|todo|unknown)\b", s.lower())


@dataclass
class PreflightCtx:
    """Everything preflight touches, injectable for tests."""
    root: Path
    runner: Path
    cfg: Any = None
    python: str = sys.executable
    run: Callable[..., tuple[int, str]] = run_cmd
    procs: Callable[[], list[tuple[int, str]]] = proc_list
    pid_alive: Callable[[int], str | None] = pid_alive
    d405_nodes: Callable[[], list[str]] = d405_nodes
    holders: Callable[[list[str]], list[dict]] = device_holders
    lsusb_sees_d405: Callable[[], bool | None] = None          # type: ignore[assignment]
    list_devices: Callable[[], list[dict]] = None              # type: ignore[assignment]
    capture: Callable[[Path], dict] = None                     # type: ignore[assignment]
    disk_free_gb: Callable[[Path], float] = None               # type: ignore[assignment]
    tty_acm: Callable[[], list[str]] = None                    # type: ignore[assignment]
    camera: bool = True
    rehearse: bool = True
    stamp: str = field(default_factory=utc_stamp)

    def __post_init__(self):
        self.root, self.runner = Path(self.root), Path(self.runner)
        if self.lsusb_sees_d405 is None:
            self.lsusb_sees_d405 = _lsusb_sees_d405
        if self.list_devices is None:
            self.list_devices = _list_devices
        if self.capture is None:
            self.capture = lambda out: _short_capture(self.cfg, out)
        if self.disk_free_gb is None:
            self.disk_free_gb = lambda p: shutil.disk_usage(p).free / 1e9
        if self.tty_acm is None:
            self.tty_acm = lambda: sorted(str(p) for p in Path("/dev").glob("ttyACM*"))

    @property
    def state_file(self) -> Path:
        return self.runner / "runs" / "arm_state.json"

    @property
    def guard_src(self) -> Path:
        return self.root / "packages" / "arm_guard" / "src"


def _lsusb_sees_d405() -> bool | None:
    exe = shutil.which("lsusb")
    if not exe:
        return None
    rc, out = run_cmd([exe, "-d", ":".join(D405_USB_ID)], timeout=20)
    return rc == 0 and ":".join(D405_USB_ID) in out.lower()


def _list_devices() -> list[dict]:
    from crackvision.d405_live import list_devices
    return list_devices()


def _short_capture(cfg: Config, out_dir: Path, n: int = 30) -> dict:
    """30 frames from the D405 into a validated recording; {fps, validation, median_depth_m, ...}."""
    from crackvision import camera_pipeline as cp
    from crackvision.d405_live import LiveD405Source
    rs_cfg = cfg.realsense
    w, h, fps = int(rs_cfg["color"]["width"]), int(rs_cfg["color"]["height"]), int(rs_cfg["color"]["fps"])
    with LiveD405Source(w, h, fps, warmup=int(rs_cfg.get("warmup_frames", 30))) as cam:
        info = cam.session_info
        frames, secs = cp.capture_frames(cam, n)
    cp.write_recording(info, frames, out_dir)
    v = cp.validate(out_dir)
    scale = float(info["depth_scale_m_per_unit"])
    z, valid = scene_depth_m([f.depth_u16 for f in frames], scale)
    return {"frames": len(frames), "seconds": round(secs, 2), "fps": round(cp.measured_fps(frames), 2),
            "resolution": f"{w}x{h}@{fps}", "validation": v["status"],
            "validation_problems": [f"{c['status']} {c['name']}: {c['value']}" for c in v["checks"]
                                    if c["status"] != "PASS"],
            "median_depth_m": None if z is None else round(z, 3),
            "valid_depth_fraction": round(valid, 3), "recording": str(out_dir)}


def scene_depth_m(depths: list, scale: float) -> tuple[float | None, float]:
    """(camera-to-scene distance in m = the median of the per-frame medians of the valid depth, over the frames that
    have any; None when no frame has depth) and the mean valid-depth fraction over all frames. Every frame counts:
    one all-zero frame (e.g. the middle one) no longer reads as 'no depth' when the other 29 are fine."""
    meds = [float(np.median(d[d > 0])) * scale for d in depths if (d > 0).any()]
    valid = float(np.mean([(d > 0).mean() for d in depths])) if depths else 0.0
    return (float(np.median(meds)) if meds else None), valid


def check_env(ctx: PreflightCtx) -> list[Check]:
    rc, out = ctx.run([ctx.python, str(ctx.root / "scripts" / "check_env.py"), "--json", "--dry-run", "-q"],
                      cwd=ctx.root, timeout=300)
    try:
        m = re.search(r"^\{", out, re.M)
        doc, _ = json.JSONDecoder().raw_decode(out[m.start():]) if m else ({}, 0)
        checks = doc["checks"]
    except (ValueError, KeyError):
        return [_fail("env", f"check_env.py gave no result (exit {rc})", "./env.sh python scripts/check_env.py"),
                _fail("model", "unknown (check_env.py gave no result)", "./env.sh python scripts/check_env.py")]
    model_names = {"model_checkpoint", "model_config_files"}
    rows = []
    for title, sel in (("env", [c for c in checks if c["name"] not in model_names | {"disk_free"}]),
                       ("model", [c for c in checks if c["name"] in model_names])):
        bad = [c for c in sel if c["status"] == "FAIL"]
        warn = [c for c in sel if c["status"] == "WARN"]
        if not sel:
            rows.append(_fail(title, "check_env.py did not report it", "./env.sh python scripts/check_env.py"))
        elif bad:
            rows.append(_fail(title, "; ".join(f"{c['name']}: {c['value']}" for c in bad),
                              "./env.sh python scripts/check_env.py (and docs/OPERATOR.md)"))
        elif warn:
            rows.append(_warn(title, "; ".join(f"{c['name']}: {c['value']}" for c in warn),
                              "./env.sh python scripts/check_env.py"))
        else:
            vals = {c["name"]: c["value"] for c in sel}
            reason = (f"{len(sel)} checks pass (python {vals.get('python_version', '?')}, torch "
                      f"{vals.get('torch_version', '?')}, {vals.get('gpu_name', '?')})" if title == "env" else
                      vals.get("model_checkpoint", "checkpoint present").split(" at ")[0] + " checkpoint + plans/dataset")
            rows.append(_pass(title, reason))
    return rows


def check_guard_approval(ctx: PreflightCtx) -> Check:
    rc, out = ctx.run([ctx.python, "-m", "arm_guard", "check"], cwd=ctx.guard_src, timeout=60)
    fix_approve = ("cd packages/arm_guard/src && ../../../env.sh python -m arm_guard check   # read the limits; then\n"
                   "../../../env.sh python -m arm_guard approve --by <your name> --note \"reviewed for the first "
                   "crack run\"")
    if rc != 0:
        lines = out.strip().splitlines() or ["?"]
        why = next((ln for ln in lines if "FAIL" in ln), lines[-1])
        return _fail("guard approval", f"the guard refuses its config: {why[:160]}", fix_approve)
    m = re.search(r"^approved: (\{.*\})\s*$", out, re.M)
    try:
        rec = json.loads(m.group(1)) if m else {}
    except ValueError:
        rec = {}
    by = rec.get("approved_by", "")
    data = {"approval": rec}
    if looks_like_operator_name(by):
        return _pass("guard approval", f"approved by {by} at {rec.get('approved_at', '?')}", **data)
    return _warn("guard approval", f"config approved, but not by an operator: \"{by[:100]}\"", fix_approve, **data)


def check_guard_hooks(ctx: PreflightCtx) -> Check:
    envs = [str(Path.home() / "miniconda3" / "envs" / e / "bin" / "python") for e in ("rebot", "lerobot")]
    rc, out = ctx.run([ctx.python, "-m", "arm_guard", "hook-status", "--python", *envs], cwd=ctx.guard_src,
                      timeout=120)
    status = {}
    for ln in out.splitlines():
        for e, py in zip(("rebot", "lerobot"), envs):
            if py in ln:
                status[e] = ln.split()[0]
    fix = "cd packages/arm_guard/src && ../../../env.sh python -m arm_guard install-hook   (then hook-status)"
    if status.get("rebot") != "OK":
        return _fail("guard hooks", f"rebot env (the runner's) is not guarded: {status.get('rebot', 'no answer')}",
                     fix, hook_status=status)
    if status.get("lerobot") != "OK":
        return _warn("guard hooks", f"rebot OK; lerobot {status.get('lerobot', 'no answer')}", fix,
                     hook_status=status)
    return _pass("guard hooks", "rebot OK, lerobot OK (motorbridge patched, config loads)", hook_status=status)


def check_runner_files(ctx: PreflightCtx) -> Check:
    need = [ctx.runner / "run-trajectory.sh", ctx.runner / "demo_move.py"]
    missing = [p.name for p in need if not p.is_file()]
    if missing:
        return _fail("runner files", f"missing in {ctx.runner}: {', '.join(missing)}",
                     "the runner lives in ~/rebot_setup (or set REBOT_RUNNER)")
    sums = {n: sha256(ctx.runner / n) for n in RUNNER_FILES if (ctx.runner / n).is_file()}
    src = (ctx.runner / "demo_move.py").read_text(errors="replace")
    ver = re.search(r'^SCRIPT_VERSION\s*=\s*"([^"]+)"', src, re.M)
    version = ver.group(1) if ver else "unknown"
    data = {"sha256": sums, "demo_move_version": version, "runner_dir": str(ctx.runner)}
    if '"--state-file"' not in src:
        return _fail("runner files", f"demo_move.py {version} has no --state-file (the capture needs it)",
                     "use demo_move.py 2026-10-23.1 or later", **data)
    if not os.access(ctx.runner / "run-trajectory.sh", os.X_OK):
        return _fail("runner files", "run-trajectory.sh is not executable", "chmod +x ~/rebot_setup/run-trajectory.sh",
                     **data)
    return _pass("runner files", f"demo_move.py {version}; sha256 run-trajectory.sh "
                 f"{sums['run-trajectory.sh'][:12]}, demo_move.py {sums['demo_move.py'][:12]} (all in the JSON)", **data)


def check_runner_idle(ctx: PreflightCtx) -> Check:
    me = os.getpid()
    busy = [(pid, cmd) for pid, cmd in ctx.procs() if pid != me and any(h in cmd for h in RUNNER_PROC_HINTS)
            and "first_run" not in cmd and "pytest" not in cmd]
    if busy:
        return _fail("arm idle", "something already drives (or may drive) the arm: "
                     + "; ".join(f"pid {p} {c[:60]}" for p, c in busy[:3]),
                     "let it finish or stop it (stop + Enter in its terminal); only one program may own the arm",
                     processes=[{"pid": p, "cmd": c} for p, c in busy])
    acm = ctx.tty_acm()
    where = (f"USB2CAN connected ({', '.join(acm)}; not opened here)" if acm
             else "USB2CAN not connected (expected at the desk: plug it in after the 24 V is on)")
    return _pass("arm idle", f"no runner, demo, LeRobot or motor tool running; {where}", tty_acm=acm)


def check_stale_files(ctx: PreflightCtx) -> Check:
    notes, data = [], {"removed": []}
    pidf = trace_dir(ctx.root) / "capture.pid"
    if pidf.is_file():
        try:
            pid = int(pidf.read_text().split()[0])
        except (ValueError, IndexError):
            pid = None
        cmd = ctx.pid_alive(pid) if pid else None
        if cmd and "crack_trace" in cmd:
            return _fail("stale files", f"a capture is still running (pid {pid}: {cmd[:60]})",
                         f"wait for it, or stop it: kill {pid}")
        pidf.unlink()
        data["removed"].append(str(pidf))
        notes.append(f"removed capture.pid (pid {pid} not running)")
    for f in (ctx.state_file, ctx.state_file.with_name(ctx.state_file.name + ".tmp")):
        if f.is_file():
            if any("demo_move.py" in c for _, c in ctx.procs()):
                return _fail("stale files", f"{f} is live: demo_move.py is running", "let the run finish")
            f.unlink()
            data["removed"].append(str(f))
            notes.append(f"removed {f.name}")
    return _pass("stale files", "; ".join(notes) if notes else "no stale capture.pid or arm_state.json", **data)


def check_disk(ctx: PreflightCtx) -> Check:
    gb = ctx.disk_free_gb(ctx.root / "data")
    if gb < DISK_FAIL_GB:
        return _fail("disk", f"{gb:.1f} GB free under data/", "free space (the runner and camera write here)",
                     free_gb=round(gb, 1))
    if gb < DISK_WARN_GB:
        return _warn("disk", f"{gb:.1f} GB free under data/ (under {DISK_WARN_GB:.0f} GB)",
                     "free some space before a long session", free_gb=round(gb, 1))
    return _pass("disk", f"{gb:.0f} GB free", free_gb=round(gb, 1))


def check_print(ctx: PreflightCtx) -> Check:
    d = ctx.root / "data" / "crack_targets"
    pdf = next((d / n for n in PRINT_PDFS if (d / n).is_file()), None)
    if pdf is None:
        return _warn("print PDF", "no crack test print PDF under data/crack_targets/",
                     "./env.sh python tools/make_crack_targets.py --no-model   (then print #1 fit to page, matte)")
    img = d / "images" / "01_crack.jpg"
    page = 1 if pdf.name == "crack_prints.pdf" else 2          # crack_test_prints.pdf starts with a cover page
    return _pass("print PDF", f"{pdf.relative_to(ctx.root)} ({pdf.stat().st_size / 1e6:.1f} MB); print #1 (the "
                 f"single crack) = page {page}, fit to page, matte" + ("" if img.is_file() else
                                                                      "; images/01_crack.jpg missing"),
                 path=str(pdf), sha256=sha256(pdf))


def check_crack_gate(ctx: PreflightCtx) -> Check:
    raw = getattr(ctx.cfg, "raw", None) or {}
    section = raw.get("crack_filter") or {}
    if section.get("crack_trace", True) is not True:
        return _fail("crack gate", "config/project.yaml crack_filter.crack_trace is not true: capture would use the "
                     "whole raw mask", "set crack_filter: crack_trace: true in config/project.yaml")
    return _pass("crack gate", f"crack_filter.crack_trace: true (mean p >= {section.get('min_mean_p', 0.8)}, "
                 f"length >= {section.get('min_length_mm', 10)} mm, width <= {section.get('max_width_mm', 10)} mm, "
                 f"{section.get('working_range_m', [0.07, 0.5])} m)")


def check_camera_free(ctx: PreflightCtx) -> Check:
    nodes = ctx.d405_nodes()
    if not nodes:
        return _warn("camera free", "no D405 device nodes to inspect (see the D405 row)", "")
    held = ctx.holders(nodes)
    if held:
        return _fail("camera free", camera_holder_text(held),
                     "stop that program (Ctrl-C in its terminal, e.g. `bench --live`, realsense-viewer)",
                     holders=held)
    return _pass("camera free", f"no other process has the D405 open ({len(nodes)} device nodes checked)")


def check_d405(ctx: PreflightCtx) -> tuple[Check, bool]:
    seen = ctx.lsusb_sees_d405()
    try:
        devs = ctx.list_devices()
    except Exception as exc:  # noqa: BLE001 — pyrealsense2 missing or broken
        return _fail("D405", f"pyrealsense2 cannot list devices: {exc}",
                     "./env.sh python scripts/check_realsense.py"), False
    d405 = [d for d in devs if str(d.get("product_id", "")).upper() == "0B5B"]
    if not d405:
        hint = ("lsusb sees it but librealsense cannot open it: another program holds it, or the udev rule is "
                "missing (docs/host/DEVICE_ACCESS.md)" if seen else
                "plug it into a rear USB 3 port, cable into the camera first, then the host end quickly and firmly")
        return _fail("D405", f"not found (lsusb: {'yes' if seen else 'no' if seen is False else 'n/a'}; "
                     f"pyrealsense2: {len(devs)} device(s))", hint), False
    dev = d405[0]
    from crackvision.d405_live import usb_is_3
    data = {"device": dev, "lsusb": seen}
    if not usb_is_3(dev.get("usb_type", "")):
        return _fail("D405", f"on a USB {dev.get('usb_type')} link: the D405 needs USB 3",
                     "re-plug: rear USB 3 port, no hub, push the host end in quickly and firmly "
                     "(docs/camera/D405_SETUP.md §2)", **data), False
    return _pass("D405", f"serial {dev.get('serial')}, USB {dev.get('usb_type')}, firmware {dev.get('firmware')} "
                 f"(5.17 not needed for this run)", **data), True


D405_RANGE_M = (0.07, 0.50)   # the D405's working range (short-range camera): outside it depth is mostly invalid by design


def check_d405_capture(ctx: PreflightCtx) -> Check:
    out = ctx.root / "data" / "recordings" / f"preflight_{ctx.stamp}"
    try:
        r = ctx.capture(out)
    except Exception as exc:  # noqa: BLE001 — D405Error, USB hiccup
        return _fail("D405 capture", f"capture failed: {exc}"[:200],
                     "re-plug the camera; close any program using it; ./env.sh python scripts/camera.py check")
    z = r.get("median_depth_m")
    reason = (f"{r['frames']} frames {r.get('resolution', '')}, {r['fps']:.1f} fps, validate_recording "
              f"{r['validation']}; scene at {'%.0f cm' % (100 * z) if z else 'no depth'}")
    problems = list(r.get("validation_problems", []))
    if r["validation"] == "FAIL":
        names = [str(pr).split(None, 1)[-1] if str(pr).startswith(("FAIL ", "WARN ")) else str(pr) for pr in problems]
        depth_only = bool(names) and all(nm.startswith(("invalid_depth", "depth")) for nm in names)
        lo, hi = D405_RANGE_M
        out_of_range = z is None or z < lo or z > hi
        if depth_only and out_of_range and r.get("fps", 0.0) >= 20.0:
            # the link streams fine; the depth is invalid because the D405 (working range ~7-50 cm) is looking at
            # something too close, too far, or at a dark room: a desk-scene condition, not a camera fault
            why = ("no depth at all: dark scene, or nothing in range" if z is None else
                   f"too close: {100 * z:.0f} cm < {100 * lo:.0f} cm" if z < lo else
                   f"too far: {100 * z:.0f} cm > {100 * hi:.0f} cm")
            return _warn("D405 capture", reason + f" ({why}, outside the D405 range): " + "; ".join(problems)[:200],
                         "point the camera at a surface 20-50 cm away (the print, your hand) in normal light and "
                         "run preflight again; at the arm the table is in range", **r)
        return _fail("D405 capture", reason + ": " + "; ".join(problems)[:200],
                     "./env.sh python scripts/camera.py check (USB 3 port, cable)"
                     + ("; the scene is in range, so the invalid depth is not the desk scene" if depth_only
                        and not out_of_range else ""), **r)
    if r["validation"] == "WARN":
        return _warn("D405 capture", reason + ": " + "; ".join(r.get("validation_problems", []))[:200],
                     "see the validator's hints: ./env.sh python scripts/validate_recording.py " + str(out), **r)
    return _pass("D405 capture", reason, **r)


def check_plans(ctx: PreflightCtx) -> list[Check]:
    out = trace_dir(ctx.root)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    plans = [("search.json", ["search-plan", "--out", str(out / "search.json")]),
             ("scan.json", ["scan-plan", "--out", str(out / "scan.json")])]
    for name, args in plans:
        rc, txt = ctx.run([ctx.python, "-m", "crackvision.crack_trace", "-q", *args], cwd=ctx.root, timeout=600)
        path = out / name
        if rc != 0 or not path.is_file():
            last = (txt.strip().splitlines() or ["?"])[-1]
            rows.append(_fail(f"{name}", f"regenerating failed (exit {rc}): {last[:150]}",
                              f"bash scripts/crack_pipeline.sh {args[0]}"))
            continue
        if not ctx.rehearse:
            rows.append(_warn(name, "regenerated; rehearsal skipped (--no-rehearse)", "run preflight without "
                              "--no-rehearse before the session", sha256=sha256(path)))
            continue
        res = {}
        for slow in ("1", "2"):
            rc2, txt2 = ctx.run([str(ctx.runner / "run-trajectory.sh"), "--rehearse", str(path), slow],
                                cwd=ctx.runner, timeout=900)
            res[f"x{slow}"] = parse_rehearsal(txt2, rc2)
        bad = [k for k, v in res.items() if not v["completed"]]
        margin = min((v["keepout_mm"] for v in res.values() if v["keepout_mm"] is not None), default=None)
        data = {"sha256": sha256(path), "rehearsals": res}
        if bad:
            why = "; ".join(f"{k}: {res[k]['outcome'] or 'exit ' + str(res[k]['rc'])} "
                            + " ".join(res[k]["refusals"][:1]) for k in bad)
            rows.append(_fail(name, f"regenerated, rehearsal failed ({why})"[:220],
                              f"~/rebot_setup/run-trajectory.sh --rehearse {path} 1   (read the refusal)", **data))
        else:
            rows.append(_pass(name, f"regenerated; rehearsed x1 and x2: completed, no guard fault, keep-out >= "
                              f"{margin} mm", **data))
    return rows


def run_preflight(ctx: PreflightCtx) -> list[Check]:
    rows: list[Check] = []

    def safe(fn, *a):
        try:
            r = fn(*a)
        except Exception as exc:  # noqa: BLE001 — a broken check is a FAIL row, never a crash
            r = _fail(getattr(fn, "__name__", "check").replace("check_", ""), f"check crashed: {exc}"[:200])
        return r if isinstance(r, list) else [r]

    rows += safe(check_env, ctx)
    rows += safe(check_guard_approval, ctx)
    rows += safe(check_guard_hooks, ctx)
    rows += safe(check_runner_files, ctx)
    rows += safe(check_runner_idle, ctx)
    rows += safe(check_stale_files, ctx)
    rows += safe(check_disk, ctx)
    if ctx.camera:
        rows += safe(check_camera_free, ctx)
        d405_ok = False
        try:
            row, d405_ok = check_d405(ctx)
        except Exception as exc:  # noqa: BLE001
            row = _fail("D405", f"check crashed: {exc}"[:200])
        rows.append(row)
        if d405_ok and not any(r.name == "camera free" and r.status == "FAIL" for r in rows):
            rows += safe(check_d405_capture, ctx)
        else:
            rows.append(_fail("D405 capture", "not attempted (the D405 row or the camera-free row failed)",
                              "fix the rows above, then run preflight again"))
    else:
        rows.append(_warn("D405", "skipped (--no-camera)", "run preflight with the camera before the session"))
    rows += safe(check_plans, ctx)
    rows += safe(check_print, ctx)
    rows += safe(check_crack_gate, ctx)
    return rows


def preflight_doc(ctx: PreflightCtx, rows: list[Check]) -> dict:
    fails = [r.name for r in rows if r.status == "FAIL"]
    warns = [r.name for r in rows if r.status == "WARN"]
    return {"schema": PREFLIGHT_SCHEMA, "created_utc": _utc_iso(), "stamp": ctx.stamp,
            "verdict": "NO-GO" if fails else "GO", "fails": fails, "warns": warns,
            "root": str(ctx.root), "runner": str(ctx.runner), "camera_checked": ctx.camera,
            "rehearsed": ctx.rehearse, "checks": [asdict(r) for r in rows]}


def print_table(rows: list[Check], out=None) -> None:
    out = out or sys.stdout
    w = max(len(r.name) for r in rows)
    for r in rows:
        print(f"  {r.status:<4}  {r.name:<{w}}  {r.reason}", file=out)
        if r.fix and r.status != "PASS":
            for i, ln in enumerate(r.fix.splitlines()):
                print(f"        {'':<{w}}  {'fix: ' if i == 0 else '     '}{ln}", file=out)


def cmd_preflight(cfg: Config, args: argparse.Namespace) -> int:
    ctx = PreflightCtx(root=cfg.root, runner=runner_dir(), cfg=cfg, camera=not args.no_camera,
                       rehearse=not args.no_rehearse)
    print(f"preflight {_utc_iso()}: the desk checks before the first crack run (nothing moves; the runner is only "
          "called with --rehearse)", flush=True)
    rows = run_preflight(ctx)
    doc = preflight_doc(ctx, rows)
    out = trace_dir(cfg.root) / f"preflight_{ctx.stamp}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, indent=1, default=str) + "\n")
    print_table(rows)
    n_w = len(doc["warns"])
    if doc["fails"]:
        print(f"NO-GO: {len(doc['fails'])} FAIL ({', '.join(doc['fails'])}), {n_w} WARN. Fix the FAIL rows and run "
              f"preflight again.")
    else:
        print(f"GO: no FAIL, {n_w} WARN" + (f" ({', '.join(doc['warns'])}): read them, they are decisions for you"
                                            if n_w else ""))
    print(f"record: {out.relative_to(cfg.root)}")
    return EXIT_RUNTIME if doc["fails"] else EXIT_OK


def latest_preflight(root: Path) -> tuple[Path | None, dict | None]:
    files = sorted(trace_dir(root).glob("preflight_*.json"))
    if not files:
        return None, None
    try:
        return files[-1], json.loads(files[-1].read_text())
    except (OSError, ValueError):
        return files[-1], None


# ---- bench -------------------------------------------------------------------------------------------------------

REASON_WORDS = {
    "low_confidence": "the model is not confident enough (mean p < {min_mean_p})",
    "too_short_px": "too short in the image (< {min_length_px:.0f} px)",
    "too_short_mm": "shorter than {min_length_mm:.0f} mm",
    "too_wide_mm": "wider than {max_width_mm:.0f} mm (a blob or a shadow, not a crack)",
    "out_of_working_range": "outside the {lo:.0f}-{hi:.0f} cm working distance",
    "no_surface_depth": "no valid depth around it (too close, too far, glare, or no texture)",
    "depth_step": "on a depth step or an object edge, not on a flat surface",
    "not_a_dark_ridge": "not a dark line on its background",
    "one_sided_edge": "a one-sided edge (a shadow or an object's outline)",
    "joint_like": "straight and even like a joint or a seam",
}


def reason_words(code: str, params) -> str:
    p = asdict(params)
    p["lo"], p["hi"] = 100 * params.working_range_m[0], 100 * params.working_range_m[1]
    try:
        return REASON_WORDS.get(code, code).format(**p)
    except (KeyError, ValueError):
        return code


def _fmt(v, fmt: str, none: str = "-") -> str:
    return none if v is None else format(v, fmt)


def component_lines(result, params) -> list[str]:
    rows = sorted(result.components, key=lambda c: (not c["accepted"], -c["score"]))
    lines = [f"  {'#':>3}  {'verdict':<8} {'length':>8} {'width':>8} {'depth':>7} {'mean_p':>6}  why"]
    for c in rows:
        why = ("; ".join(reason_words(r, params) for r in c["reject_reasons"]) if not c["accepted"] else "a crack")
        if "joint_like" in c.get("flags", []):
            why += " (flagged: straight and even, could be a joint)"
        if "touches_border" in c.get("flags", []):
            why += " (touches the image edge)"
        length = (f"{c['length_mm']:.0f} mm" if c.get("length_mm") is not None else f"{c['skeleton_length_px']:.0f} px")
        width = f"{c['width_mm']:.1f} mm" if c.get("width_mm") is not None else f"{c['width_px']:.1f} px"
        depth = f"{100 * c['depth_m']:.1f} cm" if c.get("depth_m") is not None else "-"
        lines.append(f"  {c['id']:>3}  {'accepted' if c['accepted'] else 'rejected':<8} {length:>8} {width:>8} "
                     f"{depth:>7} {c['mean_p']:>6.2f}  {why}")
    return lines


def frame_distance_m(frame, info: dict) -> float | None:
    scale = info.get("depth_scale_m_per_unit")
    d = getattr(frame, "depth_u16", None)
    if not scale or d is None or not d.any():
        return None
    return round(float(np.median(d[d > 0])) * float(scale), 3)


def bench_verdict(result, params, distance_m: float | None, depth_used: bool) -> tuple[bool, str]:
    best = result.best
    where = f"camera to scene {100 * distance_m:.0f} cm" if distance_m else "no depth"
    if best is not None:
        n = result.summary["accepted_components"]
        L = f"{best['length_mm']:.0f} mm long" if best.get("length_mm") is not None else \
            f"{best['skeleton_length_px']:.0f} px long (no depth: no mm)"
        W = f", {best['width_mm']:.1f} mm wide" if best.get("width_mm") is not None else ""
        z = f", at {100 * best['depth_m']:.1f} cm" if best.get("depth_m") else ""
        msg = (f"VERDICT: CRACK ACCEPTED: component #{best['id']}, {L}{W}{z}, mean p {best['mean_p']:.2f}"
               + (f" ({n} accepted; plan traces the best one)" if n > 1 else "") + f"; {where}")
        if "touches_border" in best.get("flags", []):
            msg += ". It touches the image edge: move the camera or the print so the whole crack is in view"
        if not depth_used:
            msg += ". No depth was used: the mm and working-distance gates were skipped"
        return True, msg
    cand = result.summary["candidate_components"]
    if cand == 0:
        msg = f"VERDICT: NO ACCEPTED COMPONENT: the model found nothing crack-like in the view; {where}"
    else:
        why = sorted({r for c in result.components for r in c["reject_reasons"]})
        msg = (f"VERDICT: NO ACCEPTED COMPONENT: {cand} candidate(s), all rejected ("
               + "; ".join(reason_words(r, params) for r in why) + f"); {where}")
    if distance_m and not params.working_range_m[0] <= distance_m <= params.working_range_m[1]:
        msg += (f". The scene is outside the {100 * params.working_range_m[0]:.0f}-"
                f"{100 * params.working_range_m[1]:.0f} cm working distance: hold the camera about 25 cm over the print")
    return False, msg


def bench_overlay(rgb: np.ndarray, result, verdict: str) -> np.ndarray:
    from PIL import Image, ImageDraw, ImageFont
    from crackvision import crack_filter as cf
    img = Image.fromarray(cf.overlay(rgb, result))
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.load_default(size=16)
        small = ImageFont.load_default(size=13)
    except TypeError:                                  # Pillow < 10.1
        font = small = ImageFont.load_default()
    for c in result.components:
        x, y = c["centroid_px"]
        col = (255, 60, 60) if c["accepted"] else (255, 220, 0)
        draw.text((x + 6, y - 8), f"#{c['id']}", fill=col, font=font, stroke_width=2, stroke_fill=(0, 0, 0))
    import textwrap
    lines = textwrap.wrap(verdict, width=max(40, img.width // 8))[:5]
    canvas = Image.new("RGB", (img.width, img.height + 8 + 16 * len(lines)), (0, 0, 0))   # below: hides nothing
    canvas.paste(img, (0, 0))
    d2 = ImageDraw.Draw(canvas)
    for i, ln in enumerate(lines):
        d2.text((6, img.height + 4 + 16 * i), ln, fill=(255, 255, 255), font=small)
    return np.asarray(canvas)


def bench_frames(cfg: Config, case: str, frames: list, info: dict, predictor, params, out_dir: Path,
                 source: str) -> dict:
    """The capture path's burst + crack gate on frames already in hand. Writes color/depth/overlay/components and
    bench.json into out_dir; returns the bench document (with `lines`, the printed table)."""
    from PIL import Image
    from crackvision import camera_pipeline as cp
    from crackvision import crack_filter as cf
    from crackvision.crack_trace import sharpness
    from crackvision.recording import _write_depth_png
    if not frames:
        raise ValueError("no frames")
    scores = [sharpness(f.color_rgb) for f in frames]
    k, prob, res, binfo = cp.filter_burst(predictor, frames, scores, info, params)
    f = frames[k]
    dist = frame_distance_m(f, info)
    accepted, verdict = bench_verdict(res, params, dist, res.summary["depth"] == "used")
    out_dir.mkdir(parents=True, exist_ok=True)
    Image.fromarray(f.color_rgb).save(out_dir / "color.png")
    if f.depth_u16 is not None and f.depth_u16.any():
        _write_depth_png(f.depth_u16, out_dir / "depth.png")
    Image.fromarray(bench_overlay(f.color_rgb, res, verdict)).save(out_dir / "overlay.png")
    (out_dir / "components.json").write_text(json.dumps(
        cf.to_doc(res, params, case_id=case, extra={"burst": binfo, "source": source}), indent=1) + "\n")
    lines = component_lines(res, params) if res.components else []
    doc = {"schema": BENCH_SCHEMA, "created_utc": _utc_iso(), "case": case, "source": source,
           "frames": len(frames), "chosen_frame": int(k), "burst": binfo, "distance_m": dist,
           "accepted": accepted, "verdict": verdict, "summary": res.summary,
           "best": res.best, "components": res.components,
           "files": {n: str((out_dir / n).relative_to(cfg.root)) if (out_dir / n).is_relative_to(cfg.root)
                     else str(out_dir / n) for n in ("color.png", "depth.png", "overlay.png", "components.json")
                     if (out_dir / n).is_file()}}
    (out_dir / "bench.json").write_text(json.dumps(doc, indent=1, default=str) + "\n")
    doc["lines"] = lines
    return doc


def _image_frames(path: Path) -> tuple[list, dict]:
    from crackvision.camera_pipeline import _image_frame
    frame, info = _image_frame(path)
    return [frame], info


LIVE_HELP = """bench --live: the crack gate on the live D405 view, for aiming (arm off).
  open  http://127.0.0.1:8090  in a browser on this machine
  red = a component the gate accepts, yellow = one it rejects; the top line shows 'accepted N/M' and 'dist .. cm'
  hold the camera about 25 cm over the print (the scan view distance); the whole crack inside the picture
  STOP with Ctrl-C in this terminal. The camera must be free again before bench, search or scan."""


def cmd_bench(cfg: Config, args: argparse.Namespace, log) -> int:
    if args.live:
        print(LIVE_HELP, flush=True)
        held = device_holders(d405_nodes())
        if held and not args.replay:
            print(f"the camera is busy: {camera_holder_text(held)}. Stop that first.")
            return EXIT_PRECONDITION
        from crackvision import camera_pipeline
        argv = ["live", "--filter"] + (["--replay", str(args.replay)] if args.replay else []) + \
               (["--hd"] if args.hd else []) + (["--port", str(args.port)] if args.port else [])
        return camera_pipeline.main(argv)
    from crackvision import crack_filter as cf
    from crackvision import naming
    case = naming.derive_case_id(Path(f"{args.case or 'bench_' + utc_stamp()}.png"))
    out_dir = trace_dir(cfg.root) / "bench" / case
    if out_dir.exists() and not args.force:
        print(f"{out_dir} exists: pick another CASE or pass --force")
        return EXIT_USAGE
    params = cf.FilterParams.from_config(cfg)
    if args.image:
        frames, info = _image_frames(Path(args.image))
        source = f"image {args.image}"
        predictor = _predictor(cfg, args)
    else:
        if not args.replay:
            held = device_holders(d405_nodes())
            if held:
                print(f"the camera is busy: {camera_holder_text(held)}. Stop that (e.g. Ctrl-C in the `bench --live` "
                      "terminal) and run bench again.")
                return EXIT_PRECONDITION
        print("loading the crack model ...", flush=True)
        predictor = _predictor(cfg, args)
        from crackvision import camera_pipeline as cp
        from crackvision.d405_live import D405Error
        ns = argparse.Namespace(replay=str(args.replay) if args.replay else None, hd=args.hd, width=None,
                                height=None, fps=None, warmup=None, serial=None)
        try:
            with cp.open_camera(cfg, ns, log) as cam:
                info = cam.session_info
                frames = list(cam.frames(args.burst))
        except D405Error as exc:
            print(f"camera: {exc}")
            return EXIT_PRECONDITION
        source = f"replay {args.replay}" if args.replay else f"d405 {info.get('device', {}).get('serial', '?')}"
    if not frames:
        print("the source delivered no frames")
        return EXIT_RUNTIME
    doc = bench_frames(cfg, case, frames, info, predictor, params, out_dir, source)
    s = doc["summary"]
    print(f"bench {case}: {doc['frames']} frame(s) from {source}; model on {len(doc['burst']['frames'])} sharpest "
          f"(vote >= {doc['burst']['min_votes']}), {s['argmax_components']} raw component(s), "
          f"{s['candidate_components']} candidate(s), {s['accepted_components']} accepted; depth {s['depth']}"
          + (f"; camera to scene {100 * doc['distance_m']:.1f} cm (median of the frame)" if doc["distance_m"] else ""))
    for ln in doc["lines"]:
        print(ln)
    print(doc["verdict"])
    print(f"  overlay    {doc['files'].get('overlay.png')}   (accepted red, rejected yellow, #id at each centroid)")
    print(f"  table      {doc['files'].get('components.json')}")
    return EXIT_OK if doc["accepted"] else EXIT_PRECONDITION


def _predictor(cfg: Config, args):
    from crackvision import crack_filter as cf
    from crackvision.crack_trace import warm_up
    from crackvision.live_predictor import LivePredictor
    p = LivePredictor(cfg, tta=args.tta or cf.config_flag(cfg, "tta", False))
    warm_up(cfg, p, True)
    return p


# ---- session -----------------------------------------------------------------------------------------------------

@dataclass
class Step:
    key: str
    argv: list[str]                   # crack_pipeline.sh subcommand + args
    arm: bool
    title: str
    happens: list[str]
    you_see: list[str]
    runner_stem: str | None = None    # traj_<stem>_*.json the runner writes for an arm step
    log: str | None = None
    camera: bool = False


def session_steps(case: str, slow: str) -> list[Step]:
    sl = f"x{slow}" if slow not in ("1", "") else "its own speed"
    return [
        Step("search", ["search", case, slow], True, f"1/5  SEARCH (arm + camera), {sl}",
             ["search-capture starts in the background and loads the crack model (log data/crack_trace/"
              f"{case}_search.log); it waits up to 300 s at each stop for the arm to rest",
              "run-trajectory.sh: all 7 motors enable, the arm moves to the fold over >= 3 s and HOLDS",
              "you press Enter at its prompt; the camera then visits 6 stops about 25 cm above the table "
              "(x 0.26 / 0.36 m, y -0.15 / 0 / +0.15 m), resting 3 s at each (about 34 s at x1, 70 s at x2), returns "
              "to the fold, holds 1 s and every motor is switched off",
              "no Enter within 5 minutes: switched off at the fold"],
             ["the runner's lines, then one 'stop N: ...' line per stop and a 'search:' line: where the crack is "
              "(base x, y) or 'no crack found'", "the wrist turning to look straight down; small pauses at each stop"],
             runner_stem="search", log=f"data/crack_trace/{case}_search.log", camera=True),
        Step("scan-plan", ["scan-plan", "--from-search", case], False, "2/5  SCAN-PLAN (no arm)",
             ["the scan pose is solved with the camera 25 cm above the crack the search found, looking straight "
              "down, held 15 s", "it is rehearsed once in simulation with the real guard config"],
             ["'outcome: completed' from the rehearsal; nothing moves"]),
        Step("scan", ["scan", case, slow], True, f"3/5  SCAN (arm + camera), {sl}",
             ["capture starts in the background, opens the camera and loads the model (log data/crack_trace/"
              f"{case}_capture.log); it waits up to 600 s for the arm to rest at the scan pose",
              "run-trajectory.sh: enable, fold, HOLD; you press Enter",
              "the arm moves to the scan pose and holds it 15 s at x1 (30 s at x2); capture grabs 10 frames once it "
              "has rested and runs the crack gate", "the arm returns to the fold, holds 1 s, every motor is switched off"],
             ["'captured case ...', the crack filter table (accepted / rejected), 'perception: N crack component(s)'",
              "the arm still over the crack, camera about 25 cm above it"],
             runner_stem="scan", log=f"data/crack_trace/{case}_capture.log", camera=True),
        Step("plan", ["plan", case], False, "4/5  PLAN (no arm)",
             ["path3d lifts the accepted crack to the table plane; the trace is planned for the best accepted "
              "component; it is rehearsed x1 and x2 in simulation with the real guard config"],
             ["'crack selection: ... component #N', 'trace for ...: ... hover ~14 cm ...', two rehearsals "
              "'outcome: completed'", "then the preview path and the accepted component numbers"]),
        Step("trace", ["trace", case, slow], True, f"5/5  TRACE (arm), {sl}",
             ["run-trajectory.sh: enable, fold, HOLD; you press Enter",
              "approach to 4 cm above the hover height, then the tool tip follows the crack about 14 cm above the "
              "table (tilted up to 30 deg where the wrist needs it), at 2 cm/s (x1), retract, return to the fold, "
              "every motor switched off"],
             ["the tool tip drawing the crack's shape in the air, about a hand's width above the print",
              "a few cm of offset from the crack is expected (calibration, GEOM-05/07): do not lower the hover"],
             runner_stem=f"{case}_trace"),
    ]


def find_runner_summary(runner: Path, stem: str, since: float) -> dict | None:
    """The newest runs/traj_<stem>_<UTC>.json written at or after `since` (a real run, not _SIM)."""
    best = None
    for p in (runner / "runs").glob(f"traj_{stem}_*.json"):
        if p.name.endswith("_SIM.json") or not re.fullmatch(rf"traj_{re.escape(stem)}_\d{{8}}T\d{{6}}Z\.json", p.name):
            continue
        try:
            mt = p.stat().st_mtime
        except OSError:
            continue
        if mt >= since - 1 and (best is None or mt > best[0]):
            best = (mt, p)
    if best is None:
        return None
    try:
        d = json.loads(best[1].read_text())
    except (OSError, ValueError):
        d = {}
    return {"path": str(best[1]), "outcome": d.get("outcome"), "csv": d.get("csv"),
            "refusal": d.get("refusal"), "guard_faults": [e for e in (d.get("guard") or {}).get("events", [])
                                                          if isinstance(e, dict) and e.get("kind") == "fault"]}


def components_report(root: Path, case: str) -> list[str]:
    """Plain lines: the case's accepted components and the one the plan traces."""
    from crackvision import crack_filter as cf
    table, _ = cf.find_case_table(root, case)
    lines = []
    if table:
        acc = [c for c in table.get("components", []) if c.get("accepted")]
        lines.append("accepted components: " + (", ".join(
            f"#{c['id']} ({_fmt(c.get('length_mm'), '.0f')} mm, p {c['mean_p']:.2f})" for c in acc) or "none"))
    plan = trace_dir(root) / f"{case}_trace_plan.json"
    if plan.is_file():
        sel = json.loads(plan.read_text()).get("selection", {})
        if sel.get("mode") == "best_crack":
            lines.append(f"the trace follows component #{sel.get('filter_component')} ({sel.get('length_mm')} mm)")
        else:
            lines.append(f"selection mode: {sel.get('mode')}")
    return lines


@dataclass
class SessionIO:
    """Terminal + process seams, injectable for tests."""
    ask: Callable[[str], str] = input
    say: Callable[[str], None] = print
    run: Callable[[list[str], dict], int] = None           # type: ignore[assignment]
    camera_holders: Callable[[], list[dict]] = None        # type: ignore[assignment]
    now: Callable[[], float] = time.time

    def __post_init__(self):
        if self.run is None:
            self.run = run_interactive
        if self.camera_holders is None:
            self.camera_holders = lambda: device_holders(d405_nodes())


def flush_tty_input() -> None:
    """Drop anything typed ahead on the terminal (e.g. a second Enter after 'yes') so it never reaches the runner's
    hold prompt as a go. No-op when stdin is not a terminal (tests, pipes)."""
    try:
        import termios
        if sys.stdin is not None and sys.stdin.isatty():
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except (ImportError, OSError, ValueError, AttributeError):
        pass


def run_interactive(cmd: list[str], env: dict) -> int:
    """Run with the terminal handed over. Ctrl-C goes to the child (run-trajectory.sh parks the arm); this process
    only waits, and never kills the child. Pending terminal input is flushed first (flush_tty_input)."""
    old = signal.signal(signal.SIGINT, lambda *_: None)
    try:
        flush_tty_input()
        p = subprocess.Popen(cmd, env=env)
        while True:
            try:
                return p.wait()
            except InterruptedError:
                continue
    finally:
        signal.signal(signal.SIGINT, old)


def _write_session(path: Path, doc: dict) -> None:
    path.write_text(json.dumps(doc, indent=1, default=str) + "\n")


def merge_demo_extra(extra: str | None) -> str:
    """The runner extras for a session. Unset -> the kd 4 default. Set without --gains -> the default gains are
    prepended (the runner would otherwise replace them: DEMO_EXTRA is one word-split string, not merged). Set with
    --gains -> exactly as given. Set to empty -> empty (the stock tuned gains, an explicit opt-out)."""
    if extra is None:
        return DEFAULT_DEMO_EXTRA
    if not extra.strip() or any(w == "--gains" or w.startswith("--gains=") for w in extra.split()):
        return extra
    return f"{DEFAULT_DEMO_EXTRA} {extra.strip()}"


def run_session(root: Path, case: str, slow: str, io: SessionIO, *, runner: Path | None = None,
                env: dict | None = None, max_preflight_age_h: float = PREFLIGHT_MAX_AGE_H) -> int:
    from crackvision import naming
    runner = runner or runner_dir()
    env = dict(os.environ if env is None else env)
    say = io.say
    if naming.derive_case_id(Path(f"{case}.png")) != case:
        say(f"session: CASE must be a plain name (letters, digits, _ -); try "
            f"{naming.derive_case_id(Path(f'{case}.png'))}")
        return EXIT_USAGE
    if not re.fullmatch(r"\d+(\.\d+)?", slow) or float(slow) < 1:
        say(f"session: slow must be a number >= 1 (got {slow!r})")
        return EXIT_USAGE
    pf_path, pf = latest_preflight(root)
    if pf is None:
        say("session: no preflight record. Run first:  bash scripts/crack_pipeline.sh preflight")
        return EXIT_PRECONDITION
    age_h = (io.now() - pf_path.stat().st_mtime) / 3600
    if pf.get("fails"):
        say(f"session: the last preflight ({pf_path.name}) is NO-GO ({', '.join(pf['fails'])}). Fix it and run "
            "preflight again.")
        return EXIT_PRECONDITION
    if age_h > max_preflight_age_h:
        say(f"session: the last preflight is {age_h:.0f} h old (limit {max_preflight_age_h:.0f} h). Run preflight "
            "again.")
        return EXIT_PRECONDITION
    if not pf.get("camera_checked", True) or not pf.get("rehearsed", True):
        say("session: the last preflight skipped the camera or the rehearsals. Run the full preflight.")
        return EXIT_PRECONDITION
    env["DEMO_EXTRA"] = merge_demo_extra(env.get("DEMO_EXTRA"))
    pipeline = [str(root / "scripts" / "crack_pipeline.sh")]
    out = trace_dir(root)
    out.mkdir(parents=True, exist_ok=True)
    spath = out / f"{case}_session.json"
    doc = {"schema": SESSION_SCHEMA, "case": case, "slow": slow, "demo_extra": env["DEMO_EXTRA"],
           "started_utc": _utc_iso(), "started_unix": io.now(), "preflight": str(pf_path), "steps": [],
           "outcome": "running"}
    _write_session(spath, doc)
    say(f"session {case}: search -> scan-plan -> scan -> plan -> trace at 1/{slow} speed; runner extras "
        f"DEMO_EXTRA=\"{env['DEMO_EXTRA']}\"" + (" (default kd 4 override)" if env["DEMO_EXTRA"] == DEFAULT_DEMO_EXTRA
                                                 else " (yours, default gains prepended)" if
                                                 env["DEMO_EXTRA"].startswith(DEFAULT_DEMO_EXTRA + " ") else " (yours)"))
    if any(w == "--ff-bias" or w.startswith("--ff-bias=") for w in env["DEMO_EXTRA"].split()):
        say("WARNING: DEMO_EXTRA has --ff-bias: the bias keeps J2 moving above the capture's settle limit "
            "(|v| < 0.05 rad/s), so search/scan captures may never settle. Not recommended for the session "
            "(docs/motion/FIRST_CRACK_RUN.md).")
    say(f"preflight: {pf_path.name}, {pf['verdict']}, {age_h:.1f} h old"
        + (f", WARN: {', '.join(pf.get('warns', []))}" if pf.get("warns") else ""))
    say("Before the first arm step: 24 V on, USB2CAN plugged in, the read-only check passed "
        "(~/rebot_setup/run-trajectory.sh --check ...), base clamped, area clear, your hand on the 24 V switch.")

    def finish(outcome: str, rc: int) -> int:
        doc["outcome"], doc["ended_utc"], doc["ended_unix"] = outcome, _utc_iso(), io.now()
        _write_session(spath, doc)
        say(f"session record: {spath.relative_to(root) if spath.is_relative_to(root) else spath}")
        return rc

    for step in session_steps(case, slow):
        say("")
        say(f"== {step.title}:  crack_pipeline.sh {' '.join(step.argv)}")
        say("   what happens:")
        for ln in step.happens:
            say(f"     - {ln}")
        say("   what you will see:")
        for ln in step.you_see:
            say(f"     - {ln}")
        if step.arm:
            say("   stop + Enter = park to the fold and switch off; off + Enter = switch off NOW (the arm drops); "
                "the 24 V switch is the e-stop")
        try:
            ans = io.ask(f"   type yes to run {step.key} (anything else stops the session): ").strip()
        except (EOFError, KeyboardInterrupt):
            ans = ""
        if ans != "yes":
            say(f"stopped before {step.key} (no 'yes'). Nothing more runs.")
            doc["steps"].append({"step": step.key, "skipped": True})
            return finish(f"stopped_before_{step.key}", EXIT_OK)
        if step.camera:
            held = io.camera_holders()
            if held:
                say(f"the camera is busy: {camera_holder_text(held)}. Stop that (e.g. Ctrl-C in the bench --live "
                    f"terminal) and run the session again (from this step, use crack_pipeline.sh {step.argv[0]}).")
                doc["steps"].append({"step": step.key, "refused": "camera busy", "holders": held})
                return finish(f"camera_busy_before_{step.key}", EXIT_PRECONDITION)
        t0 = io.now()
        rec: dict[str, Any] = {"step": step.key, "argv": step.argv, "started_utc": _utc_iso(), "started_unix": t0}
        flush_tty_input()                  # typed-ahead input never reaches the runner's hold prompt
        rc = io.run(["bash", *pipeline, *step.argv], env)
        rec |= {"rc": rc, "ended_utc": _utc_iso()}
        problem = None
        if step.runner_stem:
            summ = find_runner_summary(runner, step.runner_stem, t0)
            rec["runner"] = summ
            if summ is None:
                problem = "no runner summary was written (the runner did not start?)"
            elif summ["outcome"] != "completed":
                problem = f"the arm run's outcome is '{summ['outcome']}'" + (
                    f" ({summ['refusal']})" if summ.get("refusal") else "")
        doc["steps"].append(rec)
        _write_session(spath, doc)
        if rc != 0 or problem:
            say("")
            say(f"STOPPED at {step.key}: " + (f"exit {rc}" if rc != 0 else "") + ("; " if rc and problem else "")
                + (problem or ""))
            if step.log:
                say(f"   log: {step.log}")
            if rec.get("runner"):
                say(f"   runner summary: {rec['runner']['path']}")
            say("   read 'What can happen' in docs/motion/FIRST_CRACK_RUN.md; nothing more runs.")
            return finish(f"failed_at_{step.key}", rc or EXIT_RUNTIME)
        if step.key == "plan":
            prev = out / f"{case}_trace_preview.png"
            say("")
            say(f"PREVIEW: {prev.relative_to(root) if prev.is_relative_to(root) else prev}")
            for ln in components_report(root, case):
                say(f"   {ln}")
            say("   Open it. Green (the planned path) must sit on red (an accepted crack), on the crack you laid "
                "down; the circle marks the start. Yellow = rejected. If not, do not trace.")
    say("")
    say(f"session {case} done. Next: bash scripts/crack_pipeline.sh archive {case}, then write what you saw into "
        "its README.")
    return finish("completed", EXIT_OK)


def cmd_session(cfg: Config, args: argparse.Namespace) -> int:
    return run_session(cfg.root, args.case, str(args.slow), SessionIO())


# ---- archive -----------------------------------------------------------------------------------------------------

CASE_FILES = [  # relative to data/; {c} = the case id
    "camera/{c}", "captures/{c}_capture.json", "comparisons/{c}_comparison.png", "crack_trace/{c}",
    "crack_trace/{c}_search", "crack_trace/{c}_search.log", "crack_trace/{c}_capture.log",
    "crack_trace/{c}_trace.json", "crack_trace/{c}_trace_plan.json", "crack_trace/{c}_trace_preview.png",
    "crack_trace/{c}_session.json", "nnunet_input/{c}_0000.png", "operator_runs/{c}_run_state.json",
    "overlays/{c}_mask.png", "overlays/{c}_overlay.png", "paths3d/{c}_paths3d.json", "paths/{c}_paths.json",
    "paths/{c}_paths_overlay.png", "predictions/{c}.png", "skeletons/{c}_skeleton.png",
    "skeletons/{c}_skeleton_overlay.png", "skeletons/{c}_skeleton_stats.json",
]

README_TEMPLATE = """# First crack run on the powered arm, case `{case}`, {date}

Archived {created} by `scripts/crack_pipeline.sh archive {case}` (copies; the originals are untouched). Every file
and its sha256 is in `MANIFEST.json` and `SHA256SUMS`.

| | |
|---|---|
| Case | `{case}` |
| Session | {session_line} |
| Preflight | {preflight_line} |
| Runner | demo_move.py {runner_version}; sha256 run-trajectory.sh `{rt_sha}`, demo_move.py `{dm_sha}` |
| Guard approval | {approval_line} |
| Arm steps | {arm_line} |
| Search | {search_line} |
| Scan / crack gate | {scan_line} |
| Trace plan | {plan_line} |

## What the operator saw (fill in)

- Print used (number, paper, where it lay: x, y on the table, taped?):
- Lighting (glare? shadows from the arm?):
- Camera mount side confirmed by eye (up-facing gripper face at the fold)? yes / no:
- Search: did it find the print? which stop?
- Scan: did the overlay show the crack in red? anything accepted that is not the crack?
- Preview: did green sit on red?
- **Trace offset from the crack** (measure: along x, along y, roughly in cm; a few cm is expected until GEOM-05/07):
- Hover height above the print (about 14 cm expected):
- **Wobble** (which joint, how much, when: approach / trace / return):
- **Anything refused or stopped** (REFUSED, GUARD FAULT, ABORT, stop/off typed, 24 V cut):
- kd 4 override used? (DEMO_EXTRA above):
- Anything else:

## Files

{files_block}
"""


def _copy(src: Path, dst: Path, manifest: list, root: Path) -> None:
    if src.is_dir():
        for p in sorted(src.rglob("*")):
            if p.is_file():
                _copy(p, dst / p.relative_to(src), manifest, root)
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    manifest.append({"path": None, "source": str(src), "sha256": sha256(dst), "bytes": dst.stat().st_size,
                     "_dst": dst})


def _guard_events(path: Path, t0: float | None, t1: float | None) -> list[str]:
    if not path.is_file():
        return []
    lines = []
    for ln in path.read_text(errors="replace").splitlines():
        try:
            t = float(json.loads(ln).get("t_wall", 0))
        except (ValueError, AttributeError):
            continue
        if (t0 is None or t >= t0 - 60) and (t1 is None or t <= t1 + 60):
            lines.append(ln)
    return lines


def runner_runs_for(runner: Path, case: str, session: dict | None, root: Path) -> list[Path]:
    """The runner's run JSON (and CSV) files for this case: those the session recorded, else the real runs of
    search.json / scan.json / <case>_trace.json since the case's earliest file."""
    out: list[Path] = []
    if session:
        for st in session.get("steps", []):
            r = st.get("runner") or {}
            if r.get("path"):
                out.append(Path(r["path"]))
                if r.get("csv"):
                    out.append(Path(r["csv"]))
        if out:
            return [p for p in out if p.is_file()]
    mtimes = [p.stat().st_mtime for pat in CASE_FILES for p in [root / "data" / pat.format(c=case)] if p.exists()]
    since = min(mtimes) - 3600 if mtimes else 0
    targets = {str(trace_dir(root) / n) for n in ("search.json", "scan.json", f"{case}_trace.json")}
    for p in sorted((runner / "runs").glob("traj_*.json")):
        if p.name.endswith("_SIM.json") or p.stat().st_mtime < since:
            continue
        try:
            d = json.loads(p.read_text())
        except (OSError, ValueError):
            continue
        tpath = ((d.get("trajectory") or {}).get("path")) if isinstance(d.get("trajectory"), dict) else None
        if tpath in targets:
            out.append(p)
            if d.get("csv") and Path(d["csv"]).is_file():
                out.append(Path(d["csv"]))
    return out


def archive_case(root: Path, case: str, *, runner: Path | None = None, events: Path | None = None,
                 out_base: Path | None = None, date: str | None = None) -> Path:
    root = Path(root)
    runner = runner or runner_dir()
    events = events or Path.home() / ".local" / "state" / "arm_guard" / "events.jsonl"
    present = [(pat, root / "data" / pat.format(c=case)) for pat in CASE_FILES]
    present = [(pat, p) for pat, p in present if p.exists()]
    if not present:
        raise FileNotFoundError(f"no files for case {case} under {root / 'data'}")
    spath = trace_dir(root) / f"{case}_session.json"
    session = json.loads(spath.read_text()) if spath.is_file() else None
    date = date or ((session or {}).get("started_utc", "") or _utc_iso())[:10]
    base = Path(out_base) if out_base else root / "evidence" / "operator"
    dest = base / f"first-crack-run-{date}"
    if dest.exists():
        k = 2
        while (base / f"first-crack-run-{date}-{k}").exists():
            k += 1
        dest = base / f"first-crack-run-{date}-{k}"
    tmp = dest.with_name(dest.name + ".partial")
    if tmp.exists():
        shutil.rmtree(tmp)
    manifest: list[dict] = []
    for pat, p in present:
        _copy(p, tmp / "data" / pat.format(c=case), manifest, root)
    for name in ("search.json", "scan.json"):
        p = trace_dir(root) / name
        if p.is_file():
            _copy(p, tmp / "plans_at_archive_time" / name, manifest, root)
    pf = None
    if session and session.get("preflight") and Path(session["preflight"]).is_file():
        pf = Path(session["preflight"])
    else:
        pf, _ = latest_preflight(root)
    if pf:
        _copy(pf, tmp / "preflight" / pf.name, manifest, root)
    bench = trace_dir(root) / "bench"
    if bench.is_dir():
        for b in sorted(bench.iterdir()):
            bj = b / "bench.json"
            try:
                if bj.is_file() and json.loads(bj.read_text()).get("created_utc", "")[:10] == date:
                    _copy(b, tmp / "bench" / b.name, manifest, root)
            except (OSError, ValueError):
                pass
    runs = runner_runs_for(runner, case, session, root)
    for p in runs:
        _copy(p, tmp / "runner_runs" / p.name, manifest, root)
    t0 = (session or {}).get("started_unix")
    t1 = (session or {}).get("ended_unix")
    if t0 is None and runs:
        ts = [p.stat().st_mtime for p in runs]
        t0, t1 = min(ts) - 600, max(ts) + 60
    ev = _guard_events(events, t0, t1) if (t0 is not None) else []
    (tmp / "guard").mkdir(parents=True, exist_ok=True)
    (tmp / "guard" / "events.jsonl").write_text("\n".join(ev) + ("\n" if ev else ""))
    manifest.append({"path": None, "source": f"{events} (lines in the session window)",
                     "sha256": sha256(tmp / "guard" / "events.jsonl"),
                     "bytes": (tmp / "guard" / "events.jsonl").stat().st_size, "_dst": tmp / "guard" / "events.jsonl"})
    for cfgf in ("config/robot/arm_guard.approved.json",):
        if (root / cfgf).is_file():
            _copy(root / cfgf, tmp / "guard" / Path(cfgf).name, manifest, root)
    runner_sums = {n: sha256(runner / n) for n in RUNNER_FILES if (runner / n).is_file()}
    for m in manifest:
        m["path"] = m.pop("_dst").relative_to(tmp).as_posix()
    # the README is written last, so it can say what was found
    rt, dm = runner_sums.get("run-trajectory.sh", "?")[:16], runner_sums.get("demo_move.py", "?")[:16]
    ver = "?"
    if (runner / "demo_move.py").is_file():
        m_ = re.search(r'^SCRIPT_VERSION\s*=\s*"([^"]+)"', (runner / "demo_move.py").read_text(errors="replace"), re.M)
        ver = m_.group(1) if m_ else "?"
    pfd = json.loads(pf.read_text()) if pf else {}
    appr = next((c.get("data", {}).get("approval") for c in pfd.get("checks", []) if c.get("name") == "guard approval"),
                None) or {}
    sr = trace_dir(root) / f"{case}_search" / "search_result.json"
    srd = json.loads(sr.read_text()) if sr.is_file() else {}
    from crackvision import crack_filter as cf
    table, _ = cf.find_case_table(root, case)
    plan = trace_dir(root) / f"{case}_trace_plan.json"
    pld = json.loads(plan.read_text()) if plan.is_file() else {}
    arm = [f"{st['step']}: {(st.get('runner') or {}).get('outcome', '-')}" for st in (session or {}).get("steps", [])
           if st.get("runner") is not None]
    segs = pld.get("segments", [])
    readme = README_TEMPLATE.format(
        case=case, date=date, created=_utc_iso(),
        session_line=(f"{session.get('started_utc')} to {session.get('ended_utc', '?')}, slow {session.get('slow')}, "
                      f"DEMO_EXTRA `{session.get('demo_extra')}`, outcome **{session.get('outcome')}**"
                      if session else "no session record (steps run by hand)"),
        preflight_line=(f"`{pf.name}`: {pfd.get('verdict')}" + (f", WARN {', '.join(pfd.get('warns', []))}"
                                                                  if pfd.get("warns") else "") if pf else "none found"),
        runner_version=ver, rt_sha=rt, dm_sha=dm,
        approval_line=(f"{appr.get('approved_by')} ({appr.get('approved_at')})" if appr else "see guard/"),
        arm_line=", ".join(arm) if arm else (f"{len(runs)} runner file(s) in runner_runs/" if runs else "none recorded"),
        search_line=(f"found at stop {srd.get('best_stop')}, centre {srd.get('target_base_m')}"
                     if srd.get("found") else ("no crack found" if srd else "no search result")),
        scan_line=(f"{table['summary']['accepted_components']} of {table['summary']['candidate_components']} "
                   f"component(s) accepted, best #{table.get('best_id')}" if table else "no component table"),
        plan_line=("; ".join(f"{s.get('length_m', 0) * 1000:.0f} mm, hover {s.get('hover_m', 0) * 100:.1f} cm, "
                             f"tilt {s.get('tilt_deg', 0):.0f} deg" for s in segs) + (
            f", traced component #{pld.get('selection', {}).get('filter_component')}" if pld.get("selection") else "")
                   if segs else "no plan"),
        files_block="\n".join(f"- `{m['path']}` ({m['bytes']} B)" for m in manifest[:400])
        + (f"\n- ... {len(manifest) - 400} more (MANIFEST.json)" if len(manifest) > 400 else ""))
    (tmp / "README.md").write_text(readme)
    (tmp / "MANIFEST.json").write_text(json.dumps(
        {"schema": ARCHIVE_SCHEMA, "case": case, "created_utc": _utc_iso(), "root": str(root),
         "runner_sha256": runner_sums, "files": manifest}, indent=1) + "\n")
    (tmp / "SHA256SUMS").write_text("".join(f"{m['sha256']}  {m['path']}\n" for m in manifest))
    tmp.rename(dest)
    return dest


def cmd_archive(cfg: Config, args: argparse.Namespace) -> int:
    try:
        dest = archive_case(cfg.root, args.case, out_base=args.out)
    except FileNotFoundError as exc:
        print(f"archive: {exc}")
        return EXIT_PRECONDITION
    n = len(json.loads((dest / "MANIFEST.json").read_text())["files"])
    print(f"archived case {args.case}: {n} files copied (originals untouched) -> "
          f"{dest.relative_to(cfg.root) if dest.is_relative_to(cfg.root) else dest}")
    print(f"  next: write what you saw into {dest.name}/README.md (offset, wobble, anything refused); "
          f"file it with `claude-auto evidence add <card> <file>` if wanted")
    return EXIT_OK


# ---- CLI ---------------------------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m crackvision.first_run", description=__doc__.split("\n\n")[0])
    add_common_args(p)
    sub = p.add_subparsers(dest="cmd", required=True)
    pf = sub.add_parser("preflight", help="go/no-go at the desk (no arm motion; rehearsals only)")
    pf.add_argument("--no-camera", action="store_true", help="skip the D405 rows (they become WARN)")
    pf.add_argument("--no-rehearse", action="store_true", help="regenerate the plans but skip the rehearsals (WARN)")
    b = sub.add_parser("bench", help="the print under the camera, arm off: crack gate + verdict")
    b.add_argument("case", nargs="?", default=None)
    b.add_argument("--live", action="store_true", help="the filtered live view in the browser (camera.py live --filter)")
    b.add_argument("--replay", type=Path, default=None, help="a recording instead of the live D405")
    b.add_argument("--image", type=Path, default=None, help="an image file instead of the camera (no depth)")
    b.add_argument("--burst", type=int, default=10)
    b.add_argument("--hd", action="store_true")
    b.add_argument("--tta", action="store_true")
    b.add_argument("--port", type=int, default=None)
    b.add_argument("--force", action="store_true", help="overwrite an existing bench case")
    s = sub.add_parser("session", help="search -> scan-plan -> scan -> plan -> trace, a typed yes before each step")
    s.add_argument("case")
    s.add_argument("slow", nargs="?", default=DEFAULT_SLOW)
    a = sub.add_parser("archive", help="copy the case's evidence into evidence/operator/first-crack-run-<date>/")
    a.add_argument("case")
    a.add_argument("--out", type=Path, default=None, help="base directory (default evidence/operator)")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        cfg = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"first_run: {exc}", file=sys.stderr)
        return EXIT_USAGE
    log = setup_logging("first_run", cfg, verbose=args.verbose, quiet=args.quiet)
    if args.cmd == "preflight":
        return cmd_preflight(cfg, args)
    if args.cmd == "bench":
        return cmd_bench(cfg, args, log)
    if args.cmd == "session":
        return cmd_session(cfg, args)
    return cmd_archive(cfg, args)


if __name__ == "__main__":
    raise SystemExit(main())
