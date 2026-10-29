"""crackvision.first_run: preflight checks (with injected fakes), bench on frames, the session's step logic, archive.

Nothing here touches the arm, the CAN bus or the runner: the runner, /proc, sysfs, the camera and the terminal are all
fakes. The one real-model test (bench on the 2026-10-22 desk recording) is marked gpu/model and skips without them.
"""

from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from crackvision import first_run as fr
from crackvision.config import load_config
from crackvision.crack_filter import FilterParams
from crackvision.recording import CaptureFrame

REPO = Path(__file__).resolve().parents[1]
INTR = {"width": 848, "height": 480, "fx": 436.0, "fy": 436.0, "ppx": 424.0, "ppy": 240.0}
INFO = {"depth_scale_m_per_unit": 1e-4, "color_intrinsics": INTR, "device": {"serial": "fake"}}

GUARD_CHECK_OUT = """config  /x/arm_guard.toml
  sha256 abc
approved: {"config_sha256": "abc", "urdf_sha256": "def", "approved_by": "%s", "approved_at": "2026-10-22T04:52:33+00:00", "note": ""}
"""

REHEARSAL_OK = """pre-checks OK; path OK: trajectory search.json
arm_guard pre-flight OK: 9000 command cycles x 7 motors accepted; keep-out margin >= 31 mm
all motors DISABLED (confirmed): j1, j2, j3, j4, j5, j6, j7
outcome: completed
max |ref-pos| per joint: j1 0.009; max gap 14.0 ms; csv /r/runs_sim/x_SIM.csv
summary: /r/runs_sim/traj_search_20261029T000000Z_SIM.json
"""


# ---- small pure pieces -------------------------------------------------------------------------------------------

def test_operator_name_heuristic():
    assert fr.looks_like_operator_name("Janga786")
    assert fr.looks_like_operator_name("Jane Doe")
    assert not fr.looks_like_operator_name("not yet reviewed by the operator (approved for offline testing, 2026-10-21)")
    assert not fr.looks_like_operator_name("claude agent")
    assert not fr.looks_like_operator_name("")
    assert not fr.looks_like_operator_name("pending review")


def test_parse_rehearsal():
    r = fr.parse_rehearsal(REHEARSAL_OK, 0)
    assert r["completed"] and r["keepout_mm"] == 31 and r["max_gap_ms"] == 14.0
    assert r["summary"].endswith("_SIM.json")
    bad = fr.parse_rehearsal("REFUSED (arm_guard pre-flight: keep-out) nothing was enabled\noutcome: refused\n", 3)
    assert not bad["completed"] and bad["outcome"] == "refused" and bad["refusals"]
    assert not fr.parse_rehearsal(REHEARSAL_OK, 1)["completed"]          # a non-zero exit is never "completed"


# ---- preflight ---------------------------------------------------------------------------------------------------

class FakeRun:
    """Answers ctx.run by the command's shape; records every call."""

    def __init__(self, approved_by="Janga786", hooks=("OK", "OK"), guard_rc=0, rehearse=(0, REHEARSAL_OK),
                 env_checks=None):
        self.calls = []
        self.approved_by, self.hooks, self.guard_rc, self.rehearse = approved_by, hooks, guard_rc, rehearse
        self.env_checks = env_checks or [
            {"name": "python_version", "status": "PASS", "value": "3.11"},
            {"name": "torch_version", "status": "PASS", "value": "2.x"},
            {"name": "gpu_name", "status": "PASS", "value": "RTX 3090"},
            {"name": "model_checkpoint", "status": "PASS", "value": "255.7 MB at /m/ckpt.pth"},
            {"name": "model_config_files", "status": "PASS", "value": "plans.json"},
            {"name": "disk_free", "status": "PASS", "value": "80 GiB"}]

    def __call__(self, cmd, cwd=None, timeout=0):
        self.calls.append(list(map(str, cmd)))
        s = " ".join(map(str, cmd))
        if "check_env.py" in s:
            return 0, "log line {not json}\n" + json.dumps({"checks": self.env_checks}, indent=2) + "\nstderr {x}\n"
        if "arm_guard check" in s:
            if self.guard_rc:
                return self.guard_rc, "FAIL: config sha256 does not match the approval record"
            return 0, GUARD_CHECK_OUT % self.approved_by
        if "hook-status" in s:
            envs = cmd[cmd.index("--python") + 1:]
            return 0, "\n".join(f"{st:<4} {py}  hook=yes  {{}}" for st, py in zip(self.hooks, envs))
        if "crackvision.crack_trace" in s:
            Path(cmd[cmd.index("--out") + 1]).write_text("{}")
            return 0, ""
        if "--rehearse" in s:
            return self.rehearse
        raise AssertionError(f"unexpected command {s}")


def make_runner(tmp_path: Path, state_file_flag=True) -> Path:
    r = tmp_path / "rebot_setup"
    (r / "runs").mkdir(parents=True, exist_ok=True)
    (r / "run-trajectory.sh").write_text("#!/bin/sh\necho fake\n")
    (r / "run-trajectory.sh").chmod(0o755)
    (r / "demo_move.py").write_text('SCRIPT_VERSION = "2026-10-25.1"\n'
                                    + ('ap.add_argument("--state-file")\n' if state_file_flag else ""))
    return r


def make_ctx(tmp_path: Path, **kw) -> fr.PreflightCtx:
    root = tmp_path / "repo"
    (root / "data" / "crack_trace").mkdir(parents=True, exist_ok=True)
    (root / "data" / "crack_targets").mkdir(parents=True, exist_ok=True)
    (root / "data" / "crack_targets" / "crack_prints.pdf").write_bytes(b"%PDF-1.4 fake")
    runner = kw.pop("runner", None) or make_runner(tmp_path)
    defaults = dict(
        root=root, runner=runner, cfg={"crack_filter": {"crack_trace": True}}, run=FakeRun(),
        procs=lambda: [], pid_alive=lambda pid: None, d405_nodes=lambda: ["/dev/video0"], holders=lambda n: [],
        lsusb_sees_d405=lambda: True,
        list_devices=lambda: [{"product_id": "0B5B", "usb_type": "3.2", "serial": "123", "firmware": "5.15.1.55"}],
        capture=lambda out: {"frames": 30, "fps": 30.0, "resolution": "848x480@30", "validation": "PASS",
                             "validation_problems": [], "median_depth_m": 0.6},
        disk_free_gb=lambda p: 80.0, tty_acm=lambda: [], stamp="20261029T000000Z")
    defaults.update(kw)
    ctx = fr.PreflightCtx(**defaults)
    if isinstance(ctx.cfg, dict):                    # check_crack_gate reads cfg.raw
        ctx.cfg = type("Cfg", (), {"raw": ctx.cfg})()
    return ctx


def by_name(rows):
    return {r.name: r for r in rows}


def test_preflight_all_good_is_go_and_records_every_row(tmp_path):
    ctx = make_ctx(tmp_path)
    rows = fr.run_preflight(ctx)
    doc = fr.preflight_doc(ctx, rows)
    assert doc["verdict"] == "GO" and not doc["fails"] and not doc["warns"], [(r.name, r.status, r.reason) for r in rows]
    names = [r.name for r in rows]
    for n in ("env", "model", "guard approval", "guard hooks", "runner files", "arm idle", "stale files", "disk",
              "camera free", "D405", "D405 capture", "search.json", "scan.json", "print PDF", "crack gate"):
        assert n in names
    rf = by_name(rows)["runner files"]
    assert set(rf.data["sha256"]) == {"run-trajectory.sh", "demo_move.py"} and rf.data["demo_move_version"] == "2026-10-25.1"
    rehearsals = [c for c in ctx.run.calls if "--rehearse" in c]
    assert [c[-1] for c in rehearsals] == ["1", "2", "1", "2"]                     # search x1 x2, scan x1 x2
    assert all(c[0].endswith("run-trajectory.sh") and c[1] == "--rehearse" for c in rehearsals)
    # the only runner invocation preflight ever makes is --rehearse
    assert not [c for c in ctx.run.calls if c[0].endswith("run-trajectory.sh") and "--rehearse" not in c]
    json.dumps(doc, default=str)


def test_guard_approval_warns_until_an_operator_approves(tmp_path):
    row = fr.check_guard_approval(make_ctx(tmp_path, run=FakeRun(approved_by="not yet reviewed by the operator")))
    assert row.status == "WARN" and "approve --by" in row.fix
    assert fr.check_guard_approval(make_ctx(tmp_path, run=FakeRun(guard_rc=1))).status == "FAIL"


def test_guard_hooks(tmp_path):
    assert fr.check_guard_hooks(make_ctx(tmp_path, run=FakeRun(hooks=("OK", "WARN")))).status == "WARN"
    assert fr.check_guard_hooks(make_ctx(tmp_path, run=FakeRun(hooks=("WARN", "OK")))).status == "FAIL"


def test_runner_files(tmp_path):
    assert fr.check_runner_files(make_ctx(tmp_path)).status == "PASS"
    t2 = tmp_path / "b"
    t2.mkdir()
    old = fr.check_runner_files(make_ctx(t2, runner=make_runner(t2, state_file_flag=False)))
    assert old.status == "FAIL" and "--state-file" in old.reason
    t3 = tmp_path / "c"
    t3.mkdir()
    r = make_runner(t3)
    (r / "demo_move.py").unlink()
    assert fr.check_runner_files(make_ctx(t3, runner=r)).status == "FAIL"


def test_arm_idle_refuses_a_running_runner(tmp_path):
    row = fr.check_runner_idle(make_ctx(tmp_path, procs=lambda: [(4242, "python -u demo_move.py --trajectory x")]))
    assert row.status == "FAIL" and "4242" in row.reason
    ok = fr.check_runner_idle(make_ctx(tmp_path, tty_acm=lambda: ["/dev/ttyACM0"]))
    assert ok.status == "PASS" and "ttyACM0" in ok.reason


def test_stale_files_are_cleaned_and_live_ones_refused(tmp_path):
    ctx = make_ctx(tmp_path)
    pidf = ctx.root / "data" / "crack_trace" / "capture.pid"
    pidf.write_text("3663555\n")
    ctx.state_file.write_text("{}")
    row = fr.check_stale_files(ctx)
    assert row.status == "PASS" and not pidf.exists() and not ctx.state_file.exists()
    assert len(row.data["removed"]) == 2
    pidf.write_text("77\n")
    live = make_ctx(tmp_path, pid_alive=lambda pid: "python -m crackvision.crack_trace capture --case x")
    assert fr.check_stale_files(live).status == "FAIL" and pidf.exists()
    pidf.unlink()
    ctx.state_file.write_text("{}")
    running = make_ctx(tmp_path, procs=lambda: [(9, "python demo_move.py --state-file x")])
    assert fr.check_stale_files(running).status == "FAIL" and ctx.state_file.exists()


def test_disk_thresholds(tmp_path):
    assert fr.check_disk(make_ctx(tmp_path, disk_free_gb=lambda p: 50)).status == "PASS"
    assert fr.check_disk(make_ctx(tmp_path, disk_free_gb=lambda p: 5)).status == "WARN"
    assert fr.check_disk(make_ctx(tmp_path, disk_free_gb=lambda p: 1)).status == "FAIL"


def test_camera_rows(tmp_path):
    held = [{"pid": 99, "cmd": "python scripts/camera.py live --filter", "node": "/dev/video2"}]
    ctx = make_ctx(tmp_path, holders=lambda n: held)
    rows = by_name(fr.run_preflight(ctx))
    assert rows["camera free"].status == "FAIL" and "camera.py live" in rows["camera free"].reason
    assert rows["D405 capture"].status == "FAIL" and "not attempted" in rows["D405 capture"].reason
    usb2 = make_ctx(tmp_path, list_devices=lambda: [{"product_id": "0B5B", "usb_type": "2.1"}])
    assert fr.check_d405(usb2)[0].status == "FAIL"
    none = make_ctx(tmp_path, list_devices=lambda: [], lsusb_sees_d405=lambda: False)
    row, ok = fr.check_d405(none)
    assert row.status == "FAIL" and not ok and "USB 3 port" in row.fix
    bad = make_ctx(tmp_path, capture=lambda out: {"frames": 30, "fps": 12.0, "validation": "FAIL",
                                                  "validation_problems": ["FAIL dropped_frames: 40"]})
    assert fr.check_d405_capture(bad).status == "FAIL"
    far = make_ctx(tmp_path, capture=lambda out: {"frames": 30, "fps": 30.0, "validation": "FAIL", "median_depth_m": 1.51,
                                                  "validation_problems": ["FAIL invalid_depth_fraction: mean=79.9%"]})
    row = fr.check_d405_capture(far)          # a far / dark desk scene is not a camera fault
    assert row.status == "WARN" and "20-50 cm" in row.fix
    near_bad = make_ctx(tmp_path, capture=lambda out: {"frames": 30, "fps": 30.0, "validation": "FAIL", "median_depth_m": 0.3,
                                                       "validation_problems": ["FAIL invalid_depth_fraction: mean=79.9%"]})
    assert fr.check_d405_capture(near_bad).status == "FAIL"   # in range and still no depth: a real problem
    def cap(z, fps=30.0):
        return make_ctx(tmp_path, capture=lambda out: {"frames": 30, "fps": fps, "validation": "FAIL", "median_depth_m": z,
                                                       "validation_problems": ["FAIL invalid_depth_fraction: mean=85%"]})
    row = fr.check_d405_capture(cap(0.55))                    # audit 2026-10-29 item 8: 55 cm is beyond the D405 range
    assert row.status == "WARN" and "too far" in row.reason and "20-50 cm" in row.fix
    row = fr.check_d405_capture(cap(0.04))                    # face down on the desk: a scene hint, not a cable hint
    assert row.status == "WARN" and "too close" in row.reason and "cable" not in row.fix
    row = fr.check_d405_capture(cap(None))
    assert row.status == "WARN" and "no depth" in row.reason
    row = fr.check_d405_capture(cap(0.30))                    # in range and invalid: a real problem
    assert row.status == "FAIL" and "not the desk scene" in row.fix
    assert fr.check_d405_capture(cap(0.55, fps=12.0)).status == "FAIL"   # the link does not stream: not the scene
    boom = make_ctx(tmp_path, capture=lambda out: (_ for _ in ()).throw(RuntimeError("Frame didn't arrive")))
    assert fr.check_d405_capture(boom).status == "FAIL"
    nocam = make_ctx(tmp_path, camera=False)
    assert by_name(fr.run_preflight(nocam))["D405"].status == "WARN"


def test_a_failed_rehearsal_is_no_go(tmp_path):
    ctx = make_ctx(tmp_path, run=FakeRun(rehearse=(3, "REFUSED (arm_guard pre-flight)\noutcome: refused\n")))
    rows = fr.run_preflight(ctx)
    doc = fr.preflight_doc(ctx, rows)
    assert doc["verdict"] == "NO-GO" and {"search.json", "scan.json"} <= set(doc["fails"])


def test_crack_gate_off_is_a_fail(tmp_path):
    assert fr.check_crack_gate(make_ctx(tmp_path, cfg={"crack_filter": {"crack_trace": False}})).status == "FAIL"


def test_missing_print_is_a_warning(tmp_path):
    ctx = make_ctx(tmp_path)
    (ctx.root / "data" / "crack_targets" / "crack_prints.pdf").unlink()
    assert fr.check_print(ctx).status == "WARN"


def test_env_failure_is_reported(tmp_path):
    checks = [{"name": "cuda_available", "status": "FAIL", "value": "False"},
              {"name": "model_checkpoint", "status": "FAIL", "value": "missing"},
              {"name": "model_config_files", "status": "PASS", "value": "ok"}]
    rows = by_name(fr.check_env(make_ctx(tmp_path, run=FakeRun(env_checks=checks))))
    assert rows["env"].status == "FAIL" and "cuda_available" in rows["env"].reason
    assert rows["model"].status == "FAIL"


def test_a_crashing_check_becomes_a_fail_row(tmp_path):
    ctx = make_ctx(tmp_path, disk_free_gb=lambda p: 1 / 0)
    row = by_name(fr.run_preflight(ctx))["disk"]
    assert row.status == "FAIL" and "crashed" in row.reason


def test_d405_nodes_and_holders_from_fake_sys_and_proc(tmp_path):
    sysr = tmp_path / "sys"
    usbdev = sysr / "devices" / "usb2" / "2-4"
    iface = usbdev / "2-4:1.0"
    iface.mkdir(parents=True)
    for k, v in (("idVendor", "8086"), ("idProduct", "0b5b"), ("busnum", "2"), ("devnum", "11")):
        (usbdev / k).write_text(v + "\n")
    other = sysr / "devices" / "usb1" / "1-1"
    (other / "1-1:1.0").mkdir(parents=True)
    for k, v in (("idVendor", "046d"), ("idProduct", "0825"), ("busnum", "1"), ("devnum", "3")):
        (other / k).write_text(v + "\n")
    v4l = sysr / "class" / "video4linux"
    for name, target in (("video0", iface), ("video1", iface), ("video9", other / "1-1:1.0")):
        (v4l / name).mkdir(parents=True)
        (v4l / name / "device").symlink_to(target)
    nodes = fr.d405_nodes(sysr)
    assert nodes == ["/dev/bus/usb/002/011", "/dev/video0", "/dev/video1"]
    proc = tmp_path / "proc"
    for pid, cmd, links in ((101, b"python\0scripts/camera.py\0live", ["/dev/video1"]),
                            (102, b"firefox", ["/dev/video9", "/dev/null"])):
        (proc / str(pid) / "fd").mkdir(parents=True)
        (proc / str(pid) / "cmdline").write_bytes(cmd)
        for i, ln in enumerate(links):
            os.symlink(ln, proc / str(pid) / "fd" / str(i))
    held = fr.device_holders(nodes, proc)
    assert [(h["pid"], h["node"]) for h in held] == [(101, "/dev/video1")] and "camera.py live" in held[0]["cmd"]
    assert fr.device_holders(nodes, proc, exclude=[101]) == []
    assert fr.device_holders([], proc) == []


def test_print_table_shows_fix_only_for_problems(capsys):
    fr.print_table([fr._pass("a", "fine"), fr._warn("bbb", "hmm", "do this\nthen that")])
    out = capsys.readouterr().out
    assert "PASS  a" in out and "WARN  bbb" in out and "fix: do this" in out and "then that" in out


# ---- bench -------------------------------------------------------------------------------------------------------

def _frame(rgb, depth, i=0):
    return CaptureFrame(frame_index=i, color_rgb=rgb, depth_u16=depth, color_timestamp=float(i),
                        color_timestamp_domain="none", depth_timestamp=float(i), depth_timestamp_domain="none",
                        color_frame_number=i, depth_frame_number=i, host_monotonic_ns=time.monotonic_ns())


class LinePredictor:
    """A 'model' that marks the dark pixels of the image as crack (p 0.95)."""

    def predict_probs(self, rgb):
        return np.where(rgb.mean(axis=2) < 60, 0.95, 0.02).astype(np.float32)


def _scene(with_crack=True, z_m=0.25):
    rng = np.random.default_rng(0)
    rgb = np.clip(rng.normal(170, 6, (480, 848, 3)), 0, 255).astype(np.uint8)
    if with_crack:
        for c in range(250, 600):                      # a gently wavy, 3 px wide dark crack, 350 px long
            r = int(240 + 12 * np.sin(c / 40))
            rgb[r - 1:r + 2, c] = 20
    depth = np.full((480, 848), int(z_m / 1e-4), np.uint16)
    return rgb, depth


def test_bench_accepts_a_crack_and_writes_its_files(tmp_project, tmp_path):
    rgb, depth = _scene()
    frames = [_frame(rgb, depth, i) for i in range(4)]
    out = tmp_path / "bench" / "b1"
    doc = fr.bench_frames(tmp_project, "b1", frames, INFO, LinePredictor(), FilterParams(), out, "test")
    assert doc["accepted"] and doc["verdict"].startswith("VERDICT: CRACK ACCEPTED")
    best = doc["best"]
    assert 150 < best["length_mm"] < 260 and best["width_mm"] < 5 and abs(best["depth_m"] - 0.25) < 0.01
    assert doc["distance_m"] == pytest.approx(0.25, abs=1e-3)
    for n in ("color.png", "depth.png", "overlay.png", "components.json", "bench.json"):
        assert (out / n).is_file(), n
    assert any("accepted" in ln and "a crack" in ln for ln in doc["lines"])
    saved = json.loads((out / "bench.json").read_text())
    assert saved["accepted"] and "lines" not in saved


def test_bench_rejects_out_of_range_and_says_why(tmp_project, tmp_path):
    rgb, depth = _scene(z_m=1.9)
    doc = fr.bench_frames(tmp_project, "far", [_frame(rgb, depth, i) for i in range(3)], INFO, LinePredictor(),
                          FilterParams(), tmp_path / "far", "test")
    assert not doc["accepted"] and "NO ACCEPTED COMPONENT" in doc["verdict"]
    assert "working distance" in doc["verdict"] and "hold the camera about 25 cm" in doc["verdict"]
    assert any("rejected" in ln and "working distance" in ln for ln in doc["lines"])


def test_bench_with_nothing_in_view(tmp_project, tmp_path):
    rgb, depth = _scene(with_crack=False)
    doc = fr.bench_frames(tmp_project, "empty", [_frame(rgb, depth)], INFO, LinePredictor(), FilterParams(),
                          tmp_path / "e", "test")
    assert not doc["accepted"] and "nothing crack-like" in doc["verdict"] and doc["lines"] == []


def test_bench_without_depth_says_the_mm_gates_were_skipped(tmp_project, tmp_path):
    rgb, _ = _scene()
    frames = [_frame(rgb, np.zeros((480, 848), np.uint16))]
    doc = fr.bench_frames(tmp_project, "nod", frames, {"depth_scale_m_per_unit": None, "color_intrinsics": None},
                          LinePredictor(), FilterParams(), tmp_path / "n", "image")
    assert doc["accepted"] and "No depth was used" in doc["verdict"] and doc["distance_m"] is None


def test_reason_words_cover_every_gate():
    p = FilterParams()
    for code in fr.REASON_WORDS:
        w = fr.reason_words(code, p)
        assert w and "{" not in w
    assert "7-50 cm" in fr.reason_words("out_of_working_range", p)


def _model_ready() -> bool:
    try:
        import torch
        from crackvision.inference import missing_model_files
        return torch.cuda.is_available() and not missing_model_files(load_config(root=REPO))
    except Exception:  # noqa: BLE001
        return False


DESK = REPO / "data/recordings/cam01_20261022T174624Z"


@pytest.mark.gpu
@pytest.mark.model
@pytest.mark.skipif(not _model_ready(), reason="needs CUDA and the OpenCrack checkpoint")
@pytest.mark.skipif(not (DESK / "session.json").is_file(), reason="the 2026-10-22 desk recording is not here")
def test_bench_on_the_real_desk_recording_accepts_nothing(tmp_path):
    from crackvision.camera_pipeline import _ReplayCamera
    from crackvision.live_predictor import LivePredictor
    cfg = load_config(root=REPO)
    cam = _ReplayCamera(DESK)
    frames = list(cam.frames(10))
    doc = fr.bench_frames(cfg, "desk", frames, cam.session_info, LivePredictor(cfg), FilterParams.from_config(cfg),
                          tmp_path / "desk", "replay")
    assert not doc["accepted"] and doc["summary"]["depth"] == "used"
    assert "working distance" in doc["verdict"]


# ---- session -----------------------------------------------------------------------------------------------------

class FakeTerminal:
    def __init__(self, answers):
        self.answers, self.said, self.asked = list(answers), [], []

    def ask(self, prompt):
        self.asked.append(prompt)
        return self.answers.pop(0) if self.answers else ""

    def say(self, s):
        self.said.append(s)

    @property
    def text(self):
        return "\n".join(self.said)


def session_root(tmp_path, preflight=None, age_s=0):
    root = tmp_path / "repo"
    (root / "data" / "crack_trace").mkdir(parents=True)
    if preflight is not False:
        p = root / "data" / "crack_trace" / "preflight_20261029T000000Z.json"
        p.write_text(json.dumps(preflight or {"verdict": "GO", "fails": [], "warns": ["guard approval"],
                                              "camera_checked": True, "rehearsed": True}))
        if age_s:
            t = time.time() - age_s
            os.utime(p, (t, t))
    return root


class FakePipeline:
    """Stands in for `bash crack_pipeline.sh SUB ...`: records the call and writes what the runner would."""

    def __init__(self, runner: Path, root: Path, rc=None, outcome=None):
        self.runner, self.root, self.calls = runner, root, []
        self.rc, self.outcome = rc or {}, outcome or {}

    def __call__(self, cmd, env):
        sub, args = cmd[2], cmd[3:]
        self.calls.append((sub, args, env.get("DEMO_EXTRA")))
        assert cmd[0] == "bash" and cmd[1].endswith("scripts/crack_pipeline.sh")
        stem = {"search": "search", "scan": "scan", "trace": f"{args[0]}_trace"}.get(sub)
        if stem:
            stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
            (self.runner / "runs" / f"traj_{stem}_{stamp}.json").write_text(json.dumps(
                {"outcome": self.outcome.get(sub, "completed"), "csv": None}))
        if sub == "plan":
            case = args[0]
            (self.root / "data" / "crack_trace" / f"{case}_trace_plan.json").write_text(json.dumps(
                {"selection": {"mode": "best_crack", "filter_component": 3, "length_mm": 212.0}}))
            cam = self.root / "data" / "camera" / case
            cam.mkdir(parents=True, exist_ok=True)
            (cam / f"{case}_components.json").write_text(json.dumps({"components": [
                {"id": 3, "accepted": True, "length_mm": 212.0, "mean_p": 0.91},
                {"id": 5, "accepted": False, "length_mm": 8.0, "mean_p": 0.6}]}))
        return self.rc.get(sub, 0)


def run_session(tmp_path, answers, env=None, root=None, **kw):
    root = root or session_root(tmp_path)
    runner = make_runner(tmp_path)
    term = FakeTerminal(answers)
    pipe = kw.pop("pipeline", None) or FakePipeline(runner, root, **{k: kw.pop(k) for k in ("rc", "outcome") if k in kw})
    io = fr.SessionIO(ask=term.ask, say=term.say, run=pipe, camera_holders=kw.pop("holders", lambda: []))
    rc = fr.run_session(root, "crack01", kw.pop("slow", "2"), io, runner=runner, env=env if env is not None else {})
    return rc, term, pipe, root


def test_session_runs_the_five_steps_in_order_with_the_defaults(tmp_path):
    rc, term, pipe, root = run_session(tmp_path, ["yes"] * 5)
    assert rc == 0
    assert [(c[0], c[1]) for c in pipe.calls] == [("search", ["crack01", "2"]), ("scan-plan", ["--from-search", "crack01"]),
                                                  ("scan", ["crack01", "2"]), ("plan", ["crack01"]),
                                                  ("trace", ["crack01", "2"])]
    assert {c[2] for c in pipe.calls} == {fr.DEFAULT_DEMO_EXTRA}
    assert len(term.asked) == 5 and all("type yes" in a for a in term.asked)
    assert "PREVIEW: data/crack_trace/crack01_trace_preview.png" in term.text
    assert "#3 (212 mm" in term.text and "follows component #3" in term.text
    assert "what you will see" in term.text and "about 14 cm above" in term.text
    doc = json.loads((root / "data" / "crack_trace" / "crack01_session.json").read_text())
    assert doc["outcome"] == "completed" and [s["step"] for s in doc["steps"]] == ["search", "scan-plan", "scan",
                                                                                   "plan", "trace"]
    assert all(s["runner"]["outcome"] == "completed" for s in doc["steps"] if s["step"] in ("search", "scan", "trace"))


def test_session_keeps_the_operators_demo_extra_even_when_empty(tmp_path):
    _, _, pipe, _ = run_session(tmp_path, ["yes"], env={"DEMO_EXTRA": ""})
    assert pipe.calls[0][2] == ""
    t2 = tmp_path / "b"
    t2.mkdir()
    _, _, pipe2, _ = run_session(t2, ["yes"], env={"DEMO_EXTRA": "--gains j4=40/3"})
    assert pipe2.calls[0][2] == "--gains j4=40/3"


def test_session_merges_the_default_gains_into_an_extra_without_gains(tmp_path):
    """audit 2026-10-29 item 9: DEMO_EXTRA replaces the runner's default, so a session extra without --gains would
    silently drop kd 4; the default gains are prepended."""
    _, term, pipe, _ = run_session(tmp_path, ["yes"], env={"DEMO_EXTRA": "--friction j4=0.18"})
    assert pipe.calls[0][2] == fr.DEFAULT_DEMO_EXTRA + " --friction j4=0.18"
    assert any("default gains prepended" in ln for ln in term.said)
    assert fr.merge_demo_extra(None) == fr.DEFAULT_DEMO_EXTRA
    assert fr.merge_demo_extra("") == ""                                   # explicit opt-out: stock gains
    assert fr.merge_demo_extra("--gains=j2=100/2 --friction j4=0.2") == "--gains=j2=100/2 --friction j4=0.2"


def test_session_warns_about_ff_bias(tmp_path):
    _, term, _, _ = run_session(tmp_path, ["no"], env={"DEMO_EXTRA": "--ff-bias j2=1.4"})
    assert any("--ff-bias" in ln and "WARNING" in ln for ln in term.said)


def test_session_stops_without_yes(tmp_path):
    rc, term, pipe, root = run_session(tmp_path, ["yes", "no"])
    assert rc == 0 and [c[0] for c in pipe.calls] == ["search"]
    assert "stopped before scan-plan" in term.text
    assert json.loads((root / "data/crack_trace/crack01_session.json").read_text())["outcome"] == \
        "stopped_before_scan-plan"


def test_session_stops_on_a_failed_step_with_the_log(tmp_path):
    rc, term, pipe, _ = run_session(tmp_path, ["yes"] * 5, rc={"search": 3})
    assert rc == 3 and [c[0] for c in pipe.calls] == ["search"]
    assert "STOPPED at search: exit 3" in term.text and "data/crack_trace/crack01_search.log" in term.text


def test_session_stops_when_the_arm_run_did_not_complete(tmp_path):
    rc, term, pipe, _ = run_session(tmp_path, ["yes"] * 5, outcome={"scan": "aborted"})
    assert rc != 0 and [c[0] for c in pipe.calls] == ["search", "scan-plan", "scan"]
    assert "outcome is 'aborted'" in term.text and "runner summary" in term.text


def test_session_stops_when_no_runner_summary_appears(tmp_path):
    root = session_root(tmp_path)
    runner = make_runner(tmp_path)
    term = FakeTerminal(["yes"] * 5)
    calls = []
    io = fr.SessionIO(ask=term.ask, say=term.say, run=lambda cmd, env: calls.append(cmd) or 0,
                      camera_holders=lambda: [])
    rc = fr.run_session(root, "crack01", "2", io, runner=runner, env={})
    assert rc != 0 and len(calls) == 1 and "no runner summary" in term.text


def test_session_refuses_a_busy_camera(tmp_path):
    held = [{"pid": 7, "cmd": "python scripts/camera.py live --filter", "node": "/dev/video0"}]
    rc, term, pipe, _ = run_session(tmp_path, ["yes"], holders=lambda: held)
    assert rc == fr.EXIT_PRECONDITION and pipe.calls == [] and "camera is busy" in term.text


@pytest.mark.parametrize("pf,age,msg", [
    (False, 0, "no preflight record"),
    ({"verdict": "NO-GO", "fails": ["D405"], "warns": []}, 0, "NO-GO"),
    (None, 13 * 3600, "h old"),
    ({"verdict": "GO", "fails": [], "warns": [], "camera_checked": False, "rehearsed": True}, 0, "skipped the camera"),
])
def test_session_needs_a_recent_go_preflight(tmp_path, pf, age, msg):
    root = session_root(tmp_path, preflight=pf, age_s=age)
    rc, term, pipe, _ = run_session(tmp_path, ["yes"] * 5, root=root)
    assert rc == fr.EXIT_PRECONDITION and pipe.calls == [] and msg in term.text


def test_session_rejects_bad_names_and_speeds(tmp_path):
    root = session_root(tmp_path)
    term = FakeTerminal([])
    io = fr.SessionIO(ask=term.ask, say=term.say, run=lambda c, e: 0, camera_holders=lambda: [])
    assert fr.run_session(root, "crack 01/../x", "2", io, runner=tmp_path, env={}) == fr.EXIT_USAGE
    assert fr.run_session(root, "crack01", "0.5", io, runner=tmp_path, env={}) == fr.EXIT_USAGE


def test_find_runner_summary_ignores_sim_and_old_runs(tmp_path):
    runner = make_runner(tmp_path)
    runs = runner / "runs"
    old = runs / "traj_scan_20261020T000000Z.json"
    old.write_text(json.dumps({"outcome": "completed"}))
    t = time.time() - 3600
    os.utime(old, (t, t))
    (runs / "traj_scan_20261029T000001Z_SIM.json").write_text(json.dumps({"outcome": "completed"}))
    (runs / "traj_scan_extra_20261029T000001Z.json").write_text(json.dumps({"outcome": "completed"}))
    assert fr.find_runner_summary(runner, "scan", time.time() - 60) is None
    (runs / "traj_scan_20261029T000002Z.json").write_text(json.dumps({"outcome": "refused", "refusal": "x"}))
    s = fr.find_runner_summary(runner, "scan", time.time() - 60)
    assert s["outcome"] == "refused" and s["refusal"] == "x"


def test_run_interactive_does_not_kill_the_child_on_ctrl_c(tmp_path):
    """SIGINT to this process while the child runs: the child finishes on its own and its exit code comes back."""
    import signal
    import threading
    marker = tmp_path / "done"
    script = f"trap 'echo got-int' INT; sleep 0.6; echo ok > {marker}; exit 5"
    threading.Timer(0.2, lambda: os.kill(os.getpid(), signal.SIGINT)).start()
    rc = fr.run_interactive(["bash", "-c", script], dict(os.environ))
    assert rc == 5 and marker.read_text().strip() == "ok"


# ---- archive -----------------------------------------------------------------------------------------------------

def make_case(root: Path, runner: Path, case="crack01"):
    d = root / "data"
    files = {
        f"camera/{case}/input/{case}.png": b"png", f"camera/{case}/{case}_components.json": json.dumps(
            {"summary": {"accepted_components": 1, "candidate_components": 2}, "best_id": 3, "components": []}).encode(),
        f"captures/{case}_capture.json": b"{}", f"paths3d/{case}_paths3d.json": b"{}",
        f"crack_trace/{case}_search/search_result.json": json.dumps(
            {"found": True, "best_stop": 5, "target_base_m": [0.33, -0.11, 0.0]}).encode(),
        f"crack_trace/{case}_search/stop_05_overlay.png": b"png",
        f"crack_trace/{case}_search.log": b"stop 5: crack\n", f"crack_trace/{case}_capture.log": b"captured\n",
        f"crack_trace/{case}_trace.json": b"{}", f"crack_trace/{case}_trace_preview.png": b"png",
        f"crack_trace/{case}_trace_plan.json": json.dumps({"segments": [{"length_m": 0.209, "hover_m": 0.138,
                                                                         "tilt_deg": 10}],
                                                           "selection": {"filter_component": 3}}).encode(),
        "crack_trace/search.json": b"{}", "crack_trace/scan.json": b"{}",
        f"paths/{case}_other_paths.json": b"not mine",            # another case sharing the prefix
    }
    for rel, data in files.items():
        (d / rel).parent.mkdir(parents=True, exist_ok=True)
        (d / rel).write_bytes(data)
    (root / "config" / "robot").mkdir(parents=True, exist_ok=True)
    (root / "config" / "robot" / "arm_guard.approved.json").write_text('{"approved_by": "Janga786"}')
    pf = d / "crack_trace" / "preflight_20261029T000000Z.json"
    pf.write_text(json.dumps({"verdict": "GO", "warns": [], "checks": [
        {"name": "guard approval", "data": {"approval": {"approved_by": "Janga786", "approved_at": "t"}}}]}))
    now = time.time()
    csv = runner / "runs" / "traj_scan_20261029T010000Z.csv"
    csv.write_text("t,q\n")
    rj = runner / "runs" / "traj_scan_20261029T010000Z.json"
    rj.write_text(json.dumps({"outcome": "completed", "csv": str(csv)}))
    (d / "crack_trace" / f"{case}_session.json").write_text(json.dumps({
        "case": case, "slow": "2", "demo_extra": fr.DEFAULT_DEMO_EXTRA, "started_utc": "2026-10-29T01:00:00Z",
        "started_unix": now - 100, "ended_utc": "2026-10-29T01:10:00Z", "ended_unix": now, "outcome": "completed",
        "preflight": str(pf), "steps": [{"step": "scan", "runner": {"path": str(rj), "outcome": "completed",
                                                                    "csv": str(csv)}}]}))
    ev = root / "events.jsonl"
    ev.write_text(json.dumps({"t_wall": now - 50, "kind": "startup", "msg": "ok"}) + "\n"
                  + json.dumps({"t_wall": now - 99999, "kind": "fault", "msg": "old"}) + "\n")
    return files, ev


def test_archive_copies_with_sha256s_and_a_readme(tmp_path):
    root = tmp_path / "repo"
    runner = make_runner(tmp_path)
    files, ev = make_case(root, runner)
    before = {p: p.read_bytes() for p in (root / "data").rglob("*") if p.is_file()}
    dest = fr.archive_case(root, "crack01", runner=runner, events=ev, out_base=tmp_path / "evidence")
    assert dest.name == "first-crack-run-2026-10-29"
    assert {p: p.read_bytes() for p in (root / "data").rglob("*") if p.is_file()} == before     # copy, never move
    man = json.loads((dest / "MANIFEST.json").read_text())
    paths = {m["path"] for m in man["files"]}
    assert "data/crack_trace/crack01_search/stop_05_overlay.png" in paths
    assert "data/camera/crack01/input/crack01.png" in paths
    assert "runner_runs/traj_scan_20261029T010000Z.json" in paths and "runner_runs/traj_scan_20261029T010000Z.csv" in paths
    assert "preflight/preflight_20261029T000000Z.json" in paths and "guard/events.jsonl" in paths
    assert "plans_at_archive_time/search.json" in paths
    assert not any("other_paths" in p for p in paths)
    for line in (dest / "SHA256SUMS").read_text().splitlines():
        digest, rel = line.split("  ", 1)
        assert fr.sha256(dest / rel) == digest
    events = (dest / "guard" / "events.jsonl").read_text()
    assert "startup" in events and "old" not in events
    readme = (dest / "README.md").read_text()
    for s in ("Trace offset from the crack", "Wobble", "Anything refused", "found at stop 5", "1 of 2",
              "209 mm, hover 13.8 cm", "Janga786", "outcome **completed**"):
        assert s in readme, s
    again = fr.archive_case(root, "crack01", runner=runner, events=ev, out_base=tmp_path / "evidence")
    assert again.name == "first-crack-run-2026-10-29-2" and dest.is_dir()       # never overwrites


def test_archive_without_a_session_finds_the_runs_by_trajectory_path(tmp_path):
    root = tmp_path / "repo"
    runner = make_runner(tmp_path)
    make_case(root, runner)
    (root / "data" / "crack_trace" / "crack01_session.json").unlink()
    trace = root / "data" / "crack_trace" / "crack01_trace.json"
    (runner / "runs" / "traj_crack01_trace_20261029T020000Z.json").write_text(json.dumps(
        {"outcome": "completed", "trajectory": {"path": str(trace)}}))
    (runner / "runs" / "traj_heart_20261029T020000Z.json").write_text(json.dumps(
        {"outcome": "completed", "trajectory": {"path": "/elsewhere/heart.json"}}))
    dest = fr.archive_case(root, "crack01", runner=runner, events=root / "none.jsonl", out_base=tmp_path / "ev",
                           date="2026-10-30")
    names = {p.name for p in (dest / "runner_runs").iterdir()}
    assert names == {"traj_crack01_trace_20261029T020000Z.json"}
    assert "no session record" in (dest / "README.md").read_text()


def test_archive_refuses_an_unknown_case(tmp_path):
    root = tmp_path / "repo"
    (root / "data").mkdir(parents=True)
    with pytest.raises(FileNotFoundError):
        fr.archive_case(root, "nope", runner=make_runner(tmp_path), out_base=tmp_path / "ev")


# ---- the entry point -----------------------------------------------------------------------------------------------

def test_crack_pipeline_script_parses_and_dispatches_the_new_commands():
    script = REPO / "scripts" / "crack_pipeline.sh"
    assert subprocess.run(["bash", "-n", str(script)]).returncode == 0
    text = script.read_text()
    assert "preflight|bench|session|archive)" in text and "crackvision.first_run" in text
    usage = subprocess.run(["bash", str(script)], capture_output=True, text=True)
    assert usage.returncode == 2
    for cmd in ("search-plan", "search CASE", "scan CASE", "plan CASE", "trace CASE", "demo", "preflight",
                "bench [CASE]", "session CASE", "archive CASE"):
        assert cmd in usage.stdout, cmd
    assert "set -uo pipefail" not in usage.stdout


@pytest.mark.parametrize("sub", ["search", "scan"])
@pytest.mark.parametrize("rrc,crc,want", [(0, 0, 0), (3, 0, 3), (1, 0, 1), (0, 5, 5), (3, 5, 3)])
def test_search_and_scan_exit_with_the_arm_runs_code_even_when_the_capture_succeeded(tmp_path, sub, rrc, crc, want):
    """audit 2 item 5: a failed arm run (runner exit != 0) fails search/scan even when the capture exited 0; the
    "next:" hint only on success. A copy of the script in a stand-in repo: env.sh = the capture, a stub runner."""
    repo, runner = tmp_path / "repo", tmp_path / "runner"
    (repo / "scripts").mkdir(parents=True)
    (repo / "data" / "crack_trace").mkdir(parents=True)
    (runner / "runs").mkdir(parents=True)
    script = repo / "scripts" / "crack_pipeline.sh"
    script.write_text((REPO / "scripts" / "crack_pipeline.sh").read_text())
    (repo / "env.sh").write_text(f"#!/usr/bin/env bash\necho 'search: stub capture'\nexit {crc}\n")
    (runner / "run-trajectory.sh").write_text(f"#!/usr/bin/env bash\necho \"stub runner $*\"\nexit {rrc}\n")
    for f in (repo / "env.sh", runner / "run-trajectory.sh"):
        f.chmod(0o755)
    (repo / "data" / "crack_trace" / ("search.json" if sub == "search" else "scan.json")).write_text("{}")
    env = {k: v for k, v in os.environ.items() if k != "DEMO_EXTRA"} | {"REBOT_RUNNER": str(runner)}
    r = subprocess.run(["bash", str(script), sub, "case1", "2"], capture_output=True, text=True, env=env, timeout=60)
    assert r.returncode == want, r.stdout + r.stderr
    assert ("next:" in r.stdout) == (want == 0), r.stdout
    assert ("the arm run ended with exit" in r.stdout) == (rrc != 0), r.stdout


def test_cli_parser():
    p = fr.build_parser()
    a = p.parse_args(["session", "crack01"])
    assert a.slow == fr.DEFAULT_SLOW
    assert p.parse_args(["bench", "--live"]).live
    assert p.parse_args(["preflight", "--no-camera"]).no_camera


def test_scene_depth_uses_every_frame():
    """audit 2026-10-29 item 8: the distance is the median of the per-frame medians over all frames; one all-zero
    middle frame among 29 good in-range frames is not 'no depth' (it read as WARN before; it must FAIL)."""
    good = np.full((48, 64), 3000, np.uint16)
    depths = [good.copy() for _ in range(30)]
    depths[15][:] = 0
    z, valid = fr.scene_depth_m(depths, 1e-4)
    assert z == pytest.approx(0.30) and valid == pytest.approx(29 / 30)
    assert fr.scene_depth_m([np.zeros((4, 4), np.uint16)] * 3, 1e-4) == (None, 0.0)
