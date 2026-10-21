"""The interpreter-start hook: importing motorbridge yields guarded classes, or fails — never unguarded."""

import json
import os
import pathlib
import subprocess
import sys

import pytest

from conftest import HERE, SRC

PROBE = ("import json, motorbridge.core as mc, arm_guard; "
         "print(json.dumps({'patched': bool(getattr(mc.Motor, '_arm_guard_patched', False)), "
         "'config_ok': arm_guard.status()['config'] is not None}))")


def _run(code, extra_path=(), env=None):
    e = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH",)}
    e["PYTHONPATH"] = os.pathsep.join([str(SRC), str(HERE / "fake_mb"), *map(str, extra_path)])
    e["PYTHONNOUSERSITE"] = "1"
    e.update(env or {})
    return subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=e)


def test_boot_patches_motorbridge_on_import():
    r = _run("import arm_guard_boot; " + PROBE)
    assert r.returncode == 0, r.stderr
    assert json.loads(r.stdout.strip().splitlines()[-1]) == {"patched": True, "config_ok": True}


def test_boot_after_motorbridge_already_imported():
    r = _run("import motorbridge.core; import arm_guard_boot; " + PROBE)
    assert json.loads(r.stdout.strip().splitlines()[-1])["patched"] is True


def test_broken_guard_blocks_motorbridge(tmp_path):
    broken = tmp_path / "arm_guard"
    broken.mkdir()
    (broken / "__init__.py").write_text("raise RuntimeError('simulated broken install')\n")
    # a copy of the boot module beside the broken package
    (tmp_path / "arm_guard_boot.py").write_text((SRC / "arm_guard_boot.py").read_text())
    r = _run("import arm_guard_boot\nimport motorbridge", env={
        "PYTHONPATH": os.pathsep.join([str(tmp_path), str(HERE / "fake_mb")])})
    assert r.returncode != 0 and "motorbridge is blocked" in r.stderr


def test_patch_failure_fails_the_import(tmp_path):
    mb = tmp_path / "motorbridge"
    mb.mkdir()
    (mb / "__init__.py").write_text("from .core import Controller\n")
    (mb / "core.py").write_text("class Controller:\n    pass\n")       # no Motor class: cannot be guarded
    r = _run("import arm_guard_boot; import motorbridge", env={
        "PYTHONPATH": os.pathsep.join([str(SRC), str(tmp_path)])})
    assert r.returncode != 0 and "refusing to provide an unguarded" in r.stderr


def test_unapproved_config_refuses_motion(tmp_path):
    bad = tmp_path / "arm_guard.toml"
    bad.write_text("[guard]\n")
    r = _run("import arm_guard_boot; " + PROBE, env={"ARM_GUARD_CONFIG": str(bad)})
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert out == {"patched": True, "config_ok": False} and "CONFIG ERROR" in r.stderr


REAL_ENVS = [pathlib.Path.home() / "miniconda3/envs" / e / "bin/python" for e in ("rebot", "lerobot")]


@pytest.mark.parametrize("python", [p for p in REAL_ENVS if p.exists()], ids=lambda p: p.parts[-3])
def test_real_motorbridge_fully_classified(python):
    """Against the installed motorbridge (no bus opened): every public method is pass-through, guarded or
    refused, so a motorbridge upgrade cannot slip in an unguarded call."""
    code = (
        "import sys, json; sys.path.insert(0, %r)\n"
        "import motorbridge, motorbridge.core as mc\n"
        "from arm_guard.runtime import Guard\n"
        "from arm_guard import patch as P\n"
        "P.patch(mc.Controller, mc.Motor, Guard(None, config_error='probe'))\n"
        "bad = [n for n in dir(mc.Motor) if not n.startswith('_') and callable(getattr(mc.Motor, n))\n"
        "       and n not in P.MOTOR_PASSTHROUGH and n not in P.MOTOR_GUARDED\n"
        "       and not getattr(getattr(mc.Motor, n), '_arm_guard_refuser', False)]\n"
        "print(json.dumps({'version': motorbridge.VERSION if hasattr(motorbridge, 'VERSION') else '?', 'bad': bad,\n"
        "  'guarded': sorted(n for n in P.MOTOR_GUARDED if hasattr(mc.Motor, n))}))\n"
    ) % str(SRC)
    r = subprocess.run([str(python), "-I", "-c", code], capture_output=True, text=True)
    if "No module named 'motorbridge'" in r.stderr:
        pytest.skip("no motorbridge in this env")
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    assert out["bad"] == []
    assert {"send_mit", "send_pos_vel", "enable", "set_zero_position"} <= set(out["guarded"])
