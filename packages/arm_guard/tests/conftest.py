"""Shared fixtures: the real repo config, a fake motorbridge with simulated joints, and an isolated guard."""

import os
import pathlib
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent
SRC = pathlib.Path(os.environ.get("ARM_GUARD_TEST_SRC") or HERE.parent / "src")   # override: mutation testing
REPO = HERE.parents[2]
sys.path.insert(0, str(SRC))
sys.path.insert(0, str(HERE / "fake_mb"))

from arm_guard import config as C  # noqa: E402
from arm_guard import runtime  # noqa: E402
from arm_guard.patch import patch  # noqa: E402
from motorbridge import core as fake_core  # noqa: E402

CONFIG = REPO / "config" / "robot" / "arm_guard.toml"


@pytest.fixture(scope="session")
def cfg():
    return C.load(CONFIG, check_approval=False)


@pytest.fixture(autouse=True)
def _isolate(tmp_path, monkeypatch):
    monkeypatch.setattr(runtime, "LOCK_DIR", tmp_path / "locks")
    monkeypatch.setattr(runtime, "EVENT_LOG", tmp_path / "events.jsonl")


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Rig:
    def __init__(self, guard, ctl, motors, clock, Controller, Motor):
        self.guard, self.ctl, self.m, self.clock = guard, ctl, motors, clock
        self.Controller, self.Motor = Controller, Motor

    def step(self, dt=0.002):
        self.clock.t += dt
        for m in self.m.values():
            m._sim_step(dt)

    def enable_all(self):
        for m in self.m.values():
            m.enable()


def make_rig(cfg, start=None, joints=None, channel="/dev/fake-arm", guard_cfg="same"):
    Controller, Motor = fake_core.make_classes()
    clock = Clock()
    g = runtime.Guard(cfg if guard_cfg == "same" else guard_cfg, clock=clock,
                      config_error="" if guard_cfg == "same" or guard_cfg else "test: no config")
    patch(Controller, Motor, g)
    Controller.start_positions = {cfg.joints[n].motor_id: q for n, q in (start or {}).items()}
    ctl = Controller.from_dm_serial(channel)
    names = joints or list(cfg.joints)
    motors = {n: ctl.add_damiao_motor(cfg.joints[n].motor_id, cfg.joints[n].feedback_id, cfg.joints[n].model)
              for n in names}
    return Rig(g, ctl, motors, clock, Controller, Motor)


@pytest.fixture
def rig(cfg):
    return make_rig(cfg)
