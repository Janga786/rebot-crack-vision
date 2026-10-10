"""Phase 1 step 2 — READ-ONLY register + state dump for all 7 motors (no enable, no writes).

Reads per motor: CTRL_MODE (rid 10), TIMEOUT (rid 9), PMAX/VMAX/TMAX (rid 21/22/23, model check),
one refreshed feedback frame (status, pos, vel, torque, temps), and the register round-trip time.
"""
from __future__ import annotations

import time

from readonly_bus import STATUS, close_readonly, open_readonly

MODE_NAMES = {1: "MIT", 2: "POS_VEL", 3: "VEL", 4: "FORCE_POS"}
MODEL_LIMITS = {"4340P": (12.5, 10.0, 28.0), "4310": (12.5, 30.0, 10.0)}  # motorbridge catalog


def main() -> None:
    ctrl, motors = open_readonly()
    try:
        print(f"{'jnt':3s} {'id':4s} {'model':5s} | {'CTRL_MODE':12s} | {'TIMEOUT(raw→ms)':16s} | "
              f"{'PMAX/VMAX/TMAX':20s} {'model?':6s} | {'status':9s} {'pos rad':>8s} {'vel':>7s} "
              f"{'torq':>7s} {'Tmos':>4s} {'Trot':>4s} | rtt ms")
        for name, mid, _fid, model, m in motors:
            t0 = time.perf_counter()
            mode = m.get_register_u32(10, 200)
            rtt_ms = (time.perf_counter() - t0) * 1e3
            timeout_raw = m.get_register_u32(9, 200)
            pmax, vmax, tmax = (m.get_register_f32(r, 200) for r in (21, 22, 23))
            exp = MODEL_LIMITS[model]
            ok = all(abs(a - b) <= 0.01 for a, b in zip((pmax, vmax, tmax), exp))

            m.request_feedback()
            st = None
            for _ in range(50):
                time.sleep(0.004)
                st = m.get_state()
                if st is not None:
                    break
            if st is None:
                state = f"{'NO FEEDBACK':9s}"
            else:
                state = (f"{STATUS.get(st.status_code, hex(st.status_code)):9s} {st.pos:+8.4f} "
                         f"{st.vel:+7.3f} {st.torq:+7.3f} {st.t_mos:4.0f} {st.t_rotor:4.0f}")
            print(f"{name:3s} 0x{mid:02X} {model:5s} | {mode} {MODE_NAMES.get(mode, '??'):10s} | "
                  f"{timeout_raw:>7d} → {timeout_raw / 20:6.0f} | "
                  f"{pmax:5.2f}/{vmax:5.1f}/{tmax:5.1f}  {'OK' if ok else 'MISMATCH':6s} | {state} | {rtt_ms:5.1f}")
    finally:
        close_readonly(ctrl, motors)


if __name__ == "__main__":
    main()
