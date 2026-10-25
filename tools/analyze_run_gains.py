"""tools/analyze_run_gains.py — what the gains and the feed-forward did in a demo_move.py run (offline, from its CSV).

For each arm joint, over the moving path ('path' rows of the run CSV): the mean and rms tracking error (ref - pos, so
+ = the arm lags behind the command), its peak, the 2.5-40 Hz ripple, how much more torque than the feed-forward the
P term had to supply (mean torque - ff), and the same split by segment (approach / trajectory / return). With
--directions it also splits the near-stationary samples by the direction of the preceding move, which separates a
gravity-model bias (the same in both directions) from friction (opposite signs). A steady error with a matching
torque offset is a feed-forward problem, not a gain problem; ripple is the oscillation the encoders can see.

  ./env.sh python tools/analyze_run_gains.py RUN.csv [RUN2.csv ...] [--directions]    (also reads .csv.gz)
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
from pathlib import Path

import numpy as np

JOINTS = ["j1", "j2", "j3", "j4", "j5"]


def load(path: Path) -> list[dict]:
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt", newline="") as fh:
        return [x for x in csv.DictReader(fh) if x["mode"] == "path" and x["j2_pos"] != ""]


def band_rms(x: np.ndarray, dt: float, lo: float = 2.5, hi: float = 40.0) -> tuple[float, float]:
    """(rms of x in the lo..hi Hz band, the frequency of the strongest line in it). A Hann-windowed FFT with Parseval's
    window correction, so a sinusoid of amplitude A reads A/sqrt(2) wherever it sits in the band (a moving-average
    high-pass would attenuate the lowest frequencies, a 3 Hz line by about 6x)."""
    x = np.asarray(x, dtype=float) - np.mean(x)
    w = np.hanning(len(x))
    X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(len(x), dt)
    P = np.abs(X) ** 2
    band = (f >= lo) & (f <= hi)
    rms = float(np.sqrt(2.0 * P[band].sum() / (len(x) * np.sum(w**2))))
    return rms, float(f[band][P[band].argmax()])


def analyse(rows: list[dict]) -> dict:
    t = np.array([float(x["t_s"]) for x in rows])
    dt = float(np.median(np.diff(t)))
    seg = np.array([int(x["seg"]) for x in rows])
    out = {}
    for j in JOINTS:
        g = lambda c: np.array([float(x[f"{j}_{c}"]) for x in rows])
        e, tq, ff = g("ref") - g("pos"), g("torq"), g("ff")
        ripple_mrad, ripple_hz = band_rms(e, dt)
        out[j] = {"mean_err_mrad": round(float(e.mean() * 1000), 1), "rms_err_mrad": round(float(e.std() * 1000), 1),
                  "max_err_mrad": round(float(abs(e).max() * 1000), 1),
                  "ripple_mrad": round(ripple_mrad * 1000, 2), "ripple_hz": round(ripple_hz, 1),
                  "torque_minus_ff_mean_nm": round(float((tq - ff).mean()), 2),
                  "peak_torque_nm": round(float(abs(tq).max()), 1), "peak_ff_nm": round(float(abs(ff).max()), 1),
                  "by_segment_rms_mrad": {int(s): round(float(e[seg == s].std() * 1000), 1) for s in np.unique(seg)}}
    return out


def directions(rows: list[dict]) -> dict:
    t = np.array([float(x["t_s"]) for x in rows])
    dt = float(np.median(np.diff(t)))
    k, K = max(2, int(0.2 / dt)), int(1.0 / dt)
    out = {}
    for j in JOINTS[:4]:
        g = lambda c: np.array([float(x[f"{j}_{c}"]) for x in rows])
        ref, pos, tq, ff = g("ref"), g("pos"), g("torq"), g("ff")
        vref = np.gradient(np.convolve(ref, np.ones(k) / k, mode="same"), dt)
        acc = {1: [], -1: []}
        for i in range(K, len(rows) - k):
            if abs(vref[i]) < 0.01 and abs(ref[i] - ref[i - K]) > 0.01:
                acc[1 if ref[i] > ref[i - K] else -1].append((tq[i] - ff[i], ref[i] - pos[i]))
        out[j] = {("up" if d > 0 else "down"): {"n": len(a), "torque_minus_ff_nm": round(float(np.mean([v[0] for v in a])), 2),
                                                 "static_err_mrad": round(float(np.mean([v[1] for v in a]) * 1000), 1)}
                  for d, a in acc.items() if a}
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("csv", nargs="+", type=Path)
    ap.add_argument("--directions", action="store_true", help="split near-stationary samples by approach direction")
    ap.add_argument("--json", action="store_true", help="print JSON instead of a table")
    args = ap.parse_args(argv)
    result = {}
    for p in args.csv:
        rows = load(p)
        if len(rows) < 500:
            print(f"{p}: only {len(rows)} path rows (an aborted or refused run?): skipped")
            continue
        result[p.name] = {"joints": analyse(rows)} | ({"directions": directions(rows)} if args.directions else {})
    if args.json:
        print(json.dumps(result, indent=1))
        return 0
    for name, r in result.items():
        print(f"== {name}")
        print("  joint  mean   rms   max  ripple@Hz     tq-ff   peak tq  peak ff   rms by segment (0 approach, 1 path, 2 return)")
        for j, v in r["joints"].items():
            print(f"  {j}  {v['mean_err_mrad']:+6.1f} {v['rms_err_mrad']:5.1f} {v['max_err_mrad']:5.1f}  "
                  f"{v['ripple_mrad']:4.2f}@{v['ripple_hz']:4.1f}  {v['torque_minus_ff_mean_nm']:+6.2f} Nm  {v['peak_torque_nm']:5.1f}  "
                  f"{v['peak_ff_nm']:5.1f}   {v['by_segment_rms_mrad']}")
        for j, d in r.get("directions", {}).items():
            print(f"  {j} stationary after moving " + ", ".join(
                f"{k}: tq-ff {v['torque_minus_ff_nm']:+.2f} Nm, err {v['static_err_mrad']:+.1f} mrad (n={v['n']})" for k, v in d.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
