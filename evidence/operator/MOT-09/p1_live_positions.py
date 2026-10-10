"""Phase 1 step 3 — READ-ONLY live position table for all 7 motors (~5 Hz). Never enables anything.

Each cycle sends one Damiao refresh (0xCC) per motor, then reads the decoded feedback.
Stops on Ctrl-C, after --duration seconds, or when the file ~/rebot_setup/STOP_LIVE exists.
Writes every sample to CSV; prints a table line per cycle and a per-joint summary at the end.
"""
from __future__ import annotations

import argparse
import csv
import os
import time

from readonly_bus import STATUS, close_readonly, open_readonly

STOP_FILE = os.path.expanduser("~/rebot_setup/STOP_LIVE")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--duration", type=float, default=900.0)
    ap.add_argument("--csv", default=os.path.expanduser("~/rebot_setup/p1_live_positions.csv"))
    args = ap.parse_args()
    if os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)

    ctrl, motors = open_readonly()
    names = [m[0] for m in motors]
    stats = {n: {"min": float("inf"), "max": float("-inf"), "status": set(), "missing": 0} for n in names}
    period = 1.0 / args.hz
    t_start = time.monotonic()
    n = 0
    try:
        with open(args.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s"] + [f"{x}_pos" for x in names] + [f"{x}_status" for x in names])
            print("    t |" + "".join(f"{x:>9s}" for x in names) + " | status (all should be DISABLED)", flush=True)
            while True:
                t_cycle = time.monotonic()
                t = t_cycle - t_start
                if t > args.duration or os.path.exists(STOP_FILE):
                    break
                for *_, m in motors:
                    m.request_feedback()
                time.sleep(0.02)  # replies arrive in ~2 ms; background RX thread decodes them
                pos, sts = [], []
                for name, *_, m in motors:
                    st = m.get_state()
                    if st is None:
                        stats[name]["missing"] += 1
                        pos.append(float("nan"))
                        sts.append(-1)
                        continue
                    pos.append(st.pos)
                    sts.append(st.status_code)
                    s = stats[name]
                    s["min"], s["max"] = min(s["min"], st.pos), max(s["max"], st.pos)
                    s["status"].add(st.status_code)
                w.writerow([f"{t:.3f}"] + [f"{p:+.5f}" for p in pos] + sts)
                flag = "ok" if all(s == 0 for s in sts) else "!! " + ",".join(
                    f"{nm}={STATUS.get(s, s)}" for nm, s in zip(names, sts) if s != 0)
                print(f"{t:5.1f} |" + "".join(f"{p:+9.3f}" for p in pos) + f" | {flag}", flush=True)
                n += 1
                if n % 25 == 0:
                    fh.flush()
                time.sleep(max(0.0, period - (time.monotonic() - t_cycle)))
    except KeyboardInterrupt:
        print("\n[stopped by Ctrl-C]")
    finally:
        close_readonly(ctrl, motors)

    print(f"\nsummary over {n} samples ({time.monotonic() - t_start:.0f} s), csv: {args.csv}")
    print(f"{'jnt':3s} {'min':>8s} {'max':>8s}  peak (signed)  statuses seen")
    for name in names:
        s = stats[name]
        peak = s["max"] if abs(s["max"]) >= abs(s["min"]) else s["min"]
        seen = ",".join(STATUS.get(c, hex(c)) for c in sorted(s["status"]))
        print(f"{name:3s} {s['min']:+8.3f} {s['max']:+8.3f}  {peak:+8.3f}       {seen}"
              + (f"  missing={s['missing']}" if s["missing"] else ""))


if __name__ == "__main__":
    main()
