"""tools/analyze_run_gains.py: the error, torque-offset and ripple statistics on a synthetic run CSV."""

from __future__ import annotations

import csv
import importlib.util
import math
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("analyze_run_gains", REPO / "tools" / "analyze_run_gains.py")
arg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arg)


def write_run(path: Path, *, lag_rad: float, torque_offset: float, ripple: float, n: int = 4000) -> None:
    names = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]
    cols = ["t_s", "mode", "seg"] + [f"{j}_{c}" for j in names for c in ("ref", "pos", "vel", "torq", "ff")]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i in range(n):
            t = i * 0.005
            row = [f"{t:.4f}", "path", i * 3 // n]
            for j in names:
                ref = 0.3 * math.sin(0.5 * t)
                pos = ref - lag_rad + ripple * math.sin(2 * math.pi * 3.0 * t)
                ff = -2.0
                row += [f"{ref:.6f}", f"{pos:.6f}", "0", f"{ff + torque_offset:.3f}", f"{ff:.3f}"]
            w.writerow(row)


def test_statistics_recover_the_synthetic_lag_offset_and_ripple(tmp_path):
    p = tmp_path / "run.csv"
    write_run(p, lag_rad=0.012, torque_offset=-0.9, ripple=0.0005)
    r = arg.analyse(arg.load(p))["j3"]
    assert r["mean_err_mrad"] == 12.0
    assert abs(r["torque_minus_ff_mean_nm"] + 0.9) < 1e-6
    assert abs(r["ripple_hz"] - 3.0) < 0.5
    assert 0.30 < r["ripple_mrad"] < 0.40                      # 0.5 mrad amplitude -> 0.354 mrad rms, not attenuated at 3 Hz
    assert set(r["by_segment_rms_mrad"]) == {0, 1, 2}


def test_gzip_and_short_runs(tmp_path):
    import gzip
    p = tmp_path / "run.csv"
    write_run(p, lag_rad=0.0, torque_offset=0.0, ripple=0.0)
    gz = tmp_path / "run.csv.gz"
    gz.write_bytes(gzip.compress(p.read_bytes()))
    assert len(arg.load(gz)) == 4000
    assert arg.main([str(tmp_path / "run.csv")]) == 0


def test_band_rms_does_not_attenuate_low_frequencies():
    dt = 0.005
    t = np.arange(0, 20, dt)
    for f in (3.0, 10.0):
        rms, hz = arg.band_rms(0.001 * np.sin(2 * math.pi * f * t) + 0.2 * np.sin(2 * math.pi * 0.4 * t), dt)
        assert abs(rms - 0.001 / math.sqrt(2)) < 0.05e-3 and abs(hz - f) < 0.2
