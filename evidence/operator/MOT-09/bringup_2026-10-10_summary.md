# reBot B601-DM bring-up evidence — 2026-10-10 (scoped summary)

**Status of this file:** a summary of operator-sourced raw evidence, filed into the repo unmodified alongside it.
The overnight Claude Code session of 2026-10-10/11 filed it on the operator's written instruction. It adds no
measurement of its own. Every number below is either read from the raw files listed at the end (sha256 given),
or marked **operator statement**. An operator statement comes from the operator's written instructions for
the overnight session and does not appear in the raw logs.

The raw evidence comes from the operator's attended bring-up sessions on 2026-10-10 (sessions 1–3, about
14:00–16:45 MDT), assisted by a Claude Code session that wrote the read-only scripts and `SETUP_LOG.md` in
`~/rebot_setup/`.

## Scope: what this evidence establishes

1. **Bus and adapter.** A Damiao USB2CAN serial bridge, USB `2e88:4603` (manufacturer `HDSC`, product
   `CDC Device`, serial `00000000050C`). It enumerated as `/dev/ttyACM0` at 14:06:10 and again at 15:51:48
   (`journalctl -k` excerpts, `SETUP_LOG.md` §1.3 and S2 Phase 1). Transport `dm-serial` at 921600 baud,
   motorbridge 0.5.6 in conda env `rebot` (Python 3.12), commands run with `env -u PYTHONPATH`.
   CAN bitrate 1 Mbps and the 24 V supply: **operator statement**.
2. **Motor IDs.** Motor N answers at ESC_ID `0x0N` with MST_ID (feedback) `0x1N`, for N = 1…7. Nothing answers
   at 0x08–0x20 (`scan_s2_phase2.txt`, 15:54; `scan_s2_phase3_post_zero.txt`, 16:20). The two scan files are
   byte-identical because the CLI prints no timestamps; the times come from `SETUP_LOG.md` and file mtimes.
   The operator wrote the IDs with DMTool (Windows, 3-pin cable), one motor at a time, and verified each there.
   The scan reads only registers 7/8. It does not prove which physical joint carries which ID. That rests on
   the operator's per-motor DMTool session, and the sign check (item 5) corroborates it: each hand-moved joint
   changed the expected ID's reading, apart from the cross-talk noted there.
3. **Models and registers** (`p1_registers.txt`, read-only register reads `0x33`, all motors DISABLED):
   - M1–M3: PMAX/VMAX/TMAX = 12.5 / 10.0 / 28.0 (motorbridge `4340P`).
   - M4–M7: PMAX/VMAX/TMAX = 12.5 / 30.0 / 10.0 (`4310`).
   - CTRL_MODE = 1 (MIT) on all 7.
   - TIMEOUT = 0 on all 7, so the motor-side CAN-timeout alarm is off. An enabled motor keeps executing its
     last command if the host stops sending.
   - Register round trip about 2.1–2.2 ms.

   Motor types (M1–M3 J4340P, gear ratio 40; M4–M7 J4310, gear ratio 10) and motor 7 firmware sub-version
   **06** versus **09** on M1–M6 (seen in DMTool): **operator statement**.
4. **Zero reference.** The operator set the zero in MotorBridge Studio (Enable → Zero+Save → Disable, per
   motor) with the arm in Seeed's folded "sit-down" pose. The operator described the pose as "base slot on
   the robot's left, gripper pointing forward, gripper fully closed" (**operator statement**). `SETUP_LOG.md`
   paraphrases it as "folded sit-down, base forward, gripper closed".
   - The operator verified the zero after a power cycle (Studio; **operator statement**, `SETUP_LOG.md` §3.3).
     Software did not read the zero back at that point.
   - The read-only logger's first samples at about 16:33 read −0.00019 rad on every joint. That is the
     encoder's mid-scale code, 1 LSB of the 16-bit ±12.5 rad mapping, and is consistent with zero.
   - With joint2/joint3 resting at about 0 in the fold, the zero agrees with the URDF all-zero `home`, which
     puts joint2/joint3 on their upper limit 0.0 (`config/robot/b601_dm_limits.yaml`).
5. **Joint sign conventions** (`p1_live_positions.csv`). This was a readback-only check: 2766 samples at
   5 Hz over 553 s, every sample with status 0 = DISABLED. The only frames sent were `0xCC` refresh requests.
   The operator moved one joint at a time by hand.

   | Joint | Hand motion (operator) | Peak reading, rad (t, s) | Expected sign from URDF FK at q = 0 | Verdict |
   |---|---|---|---|---|
   | joint1 | arm swung to the robot's left | +0.41218 (130.1) | + | matches |
   | joint2 | upper arm lifted out of the fold | −0.62886 (231.3) | − (limits −3.14…0) | matches; max in fold +0.00172 |
   | joint3 | forearm lifted off the upper arm | −0.36717 (247.9) | − (limits −3.14…0) | matches; max −0.00019 |
   | joint4 | gripper nose tipped up | −0.94587 (272.3) | − for nose-up | matches |
   | joint5 | gripper turned to the robot's right | +1.09007 (343.6) | + | matches |
   | joint6 | top of the gripper rolled to the robot's right | +1.29568 (368.8) | + | matches |
   | gripper (M7) | gripper opened | −1.77596 (383.2), "partly open" | no URDF joint | **opening = negative motor angle** |

   Cross-talk during the hand moves: grabbing the wrist for joint5 moved joint6 between −0.077 and +0.064.
   Joint3's lift sagged joint4 to +0.076. Both returned to rest. No joint contradicts its URDF axis or limits.
6. **Gripper mapping.** Opening reads a **negative** motor angle. The URDF models the fingers as prismatic
   `gripper_joint1/2`, 0…0.0715 m, so any code that maps motor angle to finger width must treat open as
   negative. The full-open motor angle was **not** measured; −1.776 rad was a partial opening.
7. **Final rest pose** (last 10 s of the log, 50 samples, zero spread, all DISABLED), in rad:

   | j1 | j2 | j3 | j4 | j5 | j6 | j7 |
   |---|---|---|---|---|---|---|
   | +0.01698 | +0.00172 | −0.00134 | −0.00210 | +0.03147 | +0.00019 | +0.00782 |

   Every |q| is below 0.05 rad. At the first rest check, joint5 sat at +0.057 rad and was nudged by hand.
8. **What software sent to the motors that day.** The motorbridge CLI scan read registers 7/8. The Python
   scripts sent only `0xCC` refresh requests and `0x33` register reads (`readonly_bus.py` whitelists exactly
   these). The motors were enabled only inside MotorBridge Studio, by the operator, for Zero+Save. No script
   enabled a motor, no powered motion was commanded, and no register was written from Linux. Phase 2
   (`first_move_j1.py`) was deliberately skipped (`SETUP_LOG.md`, "S3 END").

## Scope: what this evidence does NOT establish

- **E-stop.** No hardware e-stop exists or was tested. The 24 V supply switch is the only power cut, and there
  is no inline e-stop (**operator statement**). `commissioning.yaml` `estop.verified` stays false.
- **Powered motion.** None was tested. `disable` behaviour was not measured, so ADR-016 §7's possible-fall
  assumption stands.
- **Joint-limit and range verification.** None was done. The sign check covered only partial ranges:
  j1 0…+0.41, j2 −0.63…0, j3 −0.37…0, j4 −0.95…+0.08, j5 −0.02…+1.09, j6 −0.08…+1.30, j7 −1.78…+0.01 rad.
- **Vendor driver.** `reBotArmController` and ROS were not run against the arm.
- **Commissioning.** Nothing is commissioned. `config/robot/commissioning.yaml`,
  `config/robot/b601_dm_limits.yaml` and `config/motion/execution.yaml` are unchanged.
- **Motor 7 firmware.** The 06 versus 09 difference was not evaluated.

## Raw files (copied unmodified from `~/rebot_setup/`)

| File | sha256 | What it is |
|---|---|---|
| `SETUP_LOG.md` | `c697d9e2c751d07f7d4d97ffe337cd70838688277b101ec879a73170c6ae2bae` | narrative log of sessions 1–3, commands and outputs |
| `scan_phase2.txt` | `4a5e239141b570d8760331371c9ee4d3314b0810d815d50bf60737751f2285e5` | session 1 scan: 1 hit at 0x01 / MST 0x00 (factory-default collision) |
| `scan_s2_phase2.txt` | `bb0cc14af5d72e6e78d4f1a2ce9e5dc8e4c8fb74ef1beae985dcf1de34171a3e` | session 2 scan, 7/7 (15:54) |
| `scan_s2_phase3_post_zero.txt` | `bb0cc14af5d72e6e78d4f1a2ce9e5dc8e4c8fb74ef1beae985dcf1de34171a3e` | post-zero scan, 7/7 (16:20) |
| `gateway_phase3.log` | `c47f087ebe098dc1c1b03d4a7e50e24c05903d8875583b0c15aab43ea4bd6251` | ws_gateway log during the Studio zeroing |
| `p1_registers.txt` | `1de8365021fbe5aa1999dfb5d4166344d296f26fe6771d2e0f2de36b0c0ef422` | read-only register and state dump |
| `p1_live_positions.csv` | `3dc01ff79f8ef8895869f5732a570d288cac345f5ce30af3d24ebfbbe3aa87c8` | sign check, 2766 samples |
| `p1_live_positions.log` | `b3cae7d0dd06db6595bad8831c7463bc4859b63e3ac9166c3f4bfdbe1b592efa` | console log of the sign check, with summary |
| `readonly_bus.py` | `f30f27b33eb05ac0fa4eed71f5b31db989312cb9a8a75e43c70c085eb57e53e6` | read-only motor wrapper used by the scripts |
| `p1_registers.py` | `4bb3453f6019e2e689da6209e09c17ab06f6804fa70f786954b27fba91931c0f` | produced `p1_registers.txt` |
| `p1_live_positions.py` | `216fa0a2e31cbc49c0670f2564380c7cce046c9b9963fb464bfcff3118929850` | produced `p1_live_positions.*` |

The machine-readable transcription of this summary is `bringup_2026-10-10_facts.yaml`, in the same directory.
