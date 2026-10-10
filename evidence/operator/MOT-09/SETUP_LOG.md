# reBot Arm B601-DM — setup log

Machine: HP Z8 G4 (Ubuntu, kernel 6.8.0-124-generic). Started 2026-10-10.
Arm: 7 Damiao motors (M1–M3 J4340P "4340P", M4–M7 J4310 "4310", M7 = gripper).
Link: Damiao USB2CAN (serial bridge) → signal/power separation board → XT30 2+2 harness.
Expected IDs: Motor N → CAN ID 0x0N, feedback/master ID 0x1N.

---

## Phase 1 — Environment

### 1.1 Conda
```
$ ls -d ~/miniforge3 ~/anaconda3 ~/miniconda3; which conda; conda --version
~/miniconda3 exists; /home/boosterk1/miniconda3/bin/conda; conda 26.3.2
```
→ Conda already installed (Miniconda). Miniforge install SKIPPED (not needed).
Existing envs: base, crackvision, isaaclab, lerobot, navila, navila-vila, vlnce-isaac (no `rebot` yet).

### 1.3 Adapter
```
$ ls -l /dev/ttyACM* /dev/ttyUSB*
crw-rw---- 1 root dialout 166, 0 Oct 10 14:06 /dev/ttyACM0      (no ttyUSB*)
$ ls -l /dev/serial/by-id/
usb-HDSC_CDC_Device_00000000050C-if00 -> ../../ttyACM0
$ dmesg | tail -30
dmesg: read kernel buffer failed: Operation not permitted   (dmesg_restrict; used journalctl -k instead)
$ journalctl -k -n 40 | grep -Ei 'usb|acm|cdc|disconnect'
Oct 10 14:06:10 usb 1-7: New USB device found, idVendor=2e88, idProduct=4603
Oct 10 14:06:10 usb 1-7: Product: CDC Device / Manufacturer: HDSC / SerialNumber: 00000000050C
Oct 10 14:06:10 cdc_acm 1-7:1.0: ttyACM0: USB ACM device
```
→ Damiao USB2CAN (HDSC 2e88:4603) enumerated once as /dev/ttyACM0, no disconnect messages.
```
$ dpkg -l brltty ; systemctl is-active brltty brltty-udev
rc  brltty 6.4-4ubuntu3   (removed, config files only) ; inactive / inactive
```
→ brltty not installed/active — no action needed.
```
$ id -nG
boosterk1 adm cdrom sudo dip plugdev lpadmin lxd sambashare     (NOT in dialout)
$ fuser -v /dev/ttyACM0
(nothing holds the port)
```
→ Port is root:dialout 0660 and user is not in dialout → needs `sudo chmod 666 /dev/ttyACM*` (user to run).
No motorbridge gateway / ws_gateway / rebot / lerobot process running.
Note: an unrelated detached Claude session (tmux `claude-auto-fix`, software-only, no hardware) is running.

### 1.2 `rebot` env + motorbridge
```
$ conda create -y -n rebot python=3.12          → created ~/miniconda3/envs/rebot
$ env -u PYTHONPATH conda run -n rebot pip install -U motorbridge
Successfully installed motorbridge-0.5.6
$ ls ~/miniconda3/envs/rebot/bin | grep motor
motorbridge  motorbridge-cli  motorbridge-gateway  motorbridge-install-dm-device
```
Note: ~/.bashrc exports ROS Humble's py3.10 PYTHONPATH; commands are run with `env -u PYTHONPATH`
so it can't leak into the py3.12 env. ~/.local/lib/python3.12 does not exist (no user-site shadowing).

### 1.4 CLI check
```
$ motorbridge-cli --version            → motorbridge 0.5.6
$ motorbridge-cli scan --help          → OK; flags --vendor/--transport dm-serial/--serial-port/--serial-baud
                                          exist as expected. Defaults: --model 4340, --start-id 0x01,
                                          --end-id 0x10, --feedback-base 0x10, --timeout-ms 80
```
Read-only check of the scan (source: motorbridge/cli/scan.py `_scan_damiao`): for transport dm-serial it
reads register 8 (ESC_ID) and register 7 (MST_ID) per probe ID, then frees the handle and closes the bus.
It does not enable, move, zero, or write anything. It expects replies on feedback ID 0x10 + (id & 0x0F), so a
motor with a non-standard MST_ID could show "no reply".
FOOTGUN noted: `run` is the CLI's DEFAULT subcommand, so a mistyped subcommand could fall through to `run`.
Always type the subcommand explicitly.

Phase 1 status: complete except the port permission (user to run `sudo chmod 666 /dev/ttyACM*`).

### 1.3b Port permission attempt (user, via `!` prefix)
```
$ sudo chmod 666 /dev/ttyACM*
sudo: a terminal is required to read the password; ... sudo: a password is required
```
→ The `!` shell has no TTY, so sudo can't prompt for the password. Asked the user to run it in a separate terminal instead.

### 1.3c Port permission (done)
The user supplied the sudo password in chat and told Claude to use it (password NOT recorded here). Claude ran exactly:
```
$ sudo -S chmod 666 /dev/ttyACM0        (password piped on stdin)   → exit 0
$ ls -l /dev/ttyACM0
crw-rw-rw- 1 root dialout 166, 0 Oct 10 14:06 /dev/ttyACM0
```
No other sudo commands were run.

---

## Phase 2 — Scan (read-only)

Pre-check: `pgrep -ax ws_gateway` / `pgrep -a motorbridge` → none; `fuser -v /dev/ttyACM0` → port free.
```
$ motorbridge-cli scan --vendor damiao --transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600
command=scan vendor=damiao transport=dm-serial serial_port=/dev/ttyACM0 serial_baud=921600 model=4340 id_range=[0x1,0x10] timeout_ms=80
[hit] vendor=damiao probe=0x01 esc_id=0x1 mst_id=0x0
[.. ] probe=0x02 … 0x10  no reply
scan done: 1 motor(s) found
  probe=0x01 vendor=damiao esc_id=0x1 mst_id=0x0
```
(full output: ~/rebot_setup/scan_phase2.txt)

| Motor | Expected CAN / MST | Found | Status |
|---|---|---|---|
| M1 (4340P) | 0x01 / 0x11 | something answered at 0x01 with MST_ID **0x00** | wrong MST_ID (0x00 = Damiao factory default) |
| M2 (4340P) | 0x02 / 0x12 | no reply | missing |
| M3 (4340P) | 0x03 / 0x13 | no reply | missing |
| M4 (4310)  | 0x04 / 0x14 | no reply | missing |
| M5 (4310)  | 0x05 / 0x15 | no reply | missing |
| M6 (4310)  | 0x06 / 0x16 | no reply | missing |
| M7 (4310, gripper) | 0x07 / 0x17 | no reply | missing |

**Verdict: CASE B** (1 of 7 found, and the one found has the factory-default master ID 0x00).
Most likely all 7 motors are still at factory defaults (ESC_ID 0x01, MST_ID 0x00) and answer together at
0x01. The read-only scan cannot tell one reply from seven colliding ones. A CAN break after M1 is the
other possibility. Either way: STOPPED. No ID writes from Linux. Recommendation: set IDs with DM_Tools on
Windows over the 3-pin debug cable, one motor at a time. Phase 3 is blocked until a rescan shows all 7.

---
---

# Session 2 — after DMTool ID writes (2026-10-10, ~15:5x)

User wrote all 7 IDs in DMTool on Windows (3-pin cable) and verified each one there.
Expected now: Motor N → CAN ID 0x0N, MST/feedback ID 0x1N (0x01/0x11 … 0x07/0x17).
USB2CAN replugged into this PC; arm powered (red lights = powered, disabled).
Rules for this session: Phases 1–2 read-only only; no enable/move/zero/calibrate/ID-or-param write without
explicit per-command approval; no password handling (user runs sudo); never run 2_zero_and_read.py;
stop motorbridge-gateway before any CLI scan.

## S2 Phase 1 — Environment re-check (read-only)

```
$ env -u PYTHONPATH conda run -n rebot motorbridge-cli --version
motorbridge 0.5.6
$ env -u PYTHONPATH conda run -n rebot motorbridge-cli scan --help
OK — --vendor damiao, --transport dm-serial, --serial-port, --serial-baud, --start-id, --end-id,
--feedback-base, --timeout-ms all present.
```
→ `rebot` env + motorbridge still work.

```
$ ls -l /dev/ttyACM* /dev/ttyUSB*
crw-rw---- 1 root dialout 166, 0 Oct 10 15:51 /dev/ttyACM0        (no ttyUSB*)
$ ls -l /dev/serial/by-id/
usb-HDSC_CDC_Device_00000000050C-if00 -> ../../ttyACM0
$ dmesg | tail -20
dmesg: read kernel buffer failed: Operation not permitted          (dmesg_restrict)
$ journalctl -k -n 60 | grep -Ei 'usb|acm|cdc|disconnect'
15:09:23 usb 1-7: USB disconnect, device number 4                  (adapter unplugged for DMTool work)
15:51:48 usb 1-7: New USB device found, idVendor=2e88, idProduct=4603 (HDSC CDC Device, SN 00000000050C)
15:51:48 cdc_acm 1-7:1.0: ttyACM0: USB ACM device
```
→ Same Damiao USB2CAN (same serial number) re-enumerated as /dev/ttyACM0 at 15:51:48.
→ Permission RESET to root:dialout 0660 by the replug (as expected).

```
$ pgrep -ax ws_gateway ; ps … | grep -Ei 'motorbridge|ws_gateway|lerobot|reBotArm'   → none
$ fuser -v /dev/ttyACM*                                                              → nothing holds the port
$ id -nG   → boosterk1 adm cdrom sudo dip plugdev lpadmin lxd sambashare           (still NOT in dialout)
```
→ No gateway running, port free. Waiting on user to run `sudo chmod 666 /dev/ttyACM0`.

### S2 1.3 Port permission (user ran it themselves)
User ran `sudo chmod 666 /dev/ttyACM0` (no password seen or handled by Claude).
```
$ ls -l /dev/ttyACM0
crw-rw-rw- 1 root dialout 166, 0 Oct 10 15:51 /dev/ttyACM0
```
Permanent fix suggested to user (NOT run): `sudo usermod -aG dialout $USER` (takes effect after re-login).

## S2 Phase 2 — Scan (read-only)

Pre-check: `pgrep -ax ws_gateway` / ps grep motorbridge|ws_gateway → none; `fuser -v /dev/ttyACM0` → port free.
```
$ env -u PYTHONPATH conda run -n rebot motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600 --start-id 1 --end-id 32
command=scan vendor=damiao transport=dm-serial serial_port=/dev/ttyACM0 serial_baud=921600 model=4340 id_range=[0x1,0x20] timeout_ms=80
[hit] probe=0x01 esc_id=0x1 mst_id=0x11
[hit] probe=0x02 esc_id=0x2 mst_id=0x12
[hit] probe=0x03 esc_id=0x3 mst_id=0x13
[hit] probe=0x04 esc_id=0x4 mst_id=0x14
[hit] probe=0x05 esc_id=0x5 mst_id=0x15
[hit] probe=0x06 esc_id=0x6 mst_id=0x16
[hit] probe=0x07 esc_id=0x7 mst_id=0x17
[.. ] probe=0x08 … 0x20  no reply
scan done: 7 motor(s) found
```
(full output: ~/rebot_setup/scan_s2_phase2.txt)

| Motor | Model | Expected CAN / MST | Found CAN / MST | Status |
|---|---|---|---|---|
| M1 | 4340P | 0x01 / 0x11 | 0x01 / 0x11 | ✅ |
| M2 | 4340P | 0x02 / 0x12 | 0x02 / 0x12 | ✅ |
| M3 | 4340P | 0x03 / 0x13 | 0x03 / 0x13 | ✅ |
| M4 | 4310  | 0x04 / 0x14 | 0x04 / 0x14 | ✅ |
| M5 | 4310  | 0x05 / 0x15 | 0x05 / 0x15 | ✅ |
| M6 | 4310  | 0x06 / 0x16 | 0x06 / 0x16 | ✅ |
| M7 | 4310 (gripper) | 0x07 / 0x17 | 0x07 / 0x17 | ✅ |

No replies at 0x08–0x20 → no stray/duplicate IDs (incl. nothing left at a feedback-range ID 0x11–0x17).
**Verdict: CASE A** — all 7 found with correct IDs. The session-1 factory-default collision is resolved.
Scope note: the scan reads only ESC_ID (reg 8) and MST_ID (reg 7); it doesn't confirm motor model or which
physical joint carries which ID (that relies on the user's per-motor DMTool verification).
STOPPED at checkpoint 2 — awaiting user go for Phase 3.

## S2 Phase 3 — Zero calibration

User go (2026-10-10): "Posed. Go ahead with Phase 3." → arm posed at zero (folded sit-down, base forward,
gripper closed) per b601dm_zeroposition.jpg. No Studio Help command pasted → used the planned command.

### 3.2 Gateway start
Read-only pre-check of what it does: `motorbridge-gateway --help` → "Router mode … web UI/WS messages choose
vendor, model, IDs … --vendor/--serial-port etc. are optional defaults only used when WS message omits target
fields". Wrapper motorbridge/gateway.py just preflights and execs the packaged Rust bin motorbridge/bin/ws_gateway.
```
$ env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/motorbridge-gateway -- --bind 127.0.0.1:9002 --vendor damiao \
    --transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600 --dt-ms 20   (background; log: gateway_phase3.log)
ws_gateway listening on ws://127.0.0.1:9002 (router_mode=standby, dynamic_target=true, dt_ms=20)
$ pgrep -ax ws_gateway         → pid 7308
$ ss -ltnp | grep 9002         → LISTEN 127.0.0.1:9002 ws_gateway pid 7308 (loopback only)
$ fuser -v /dev/ttyACM0        → (no holder yet — standby; serial port opened on demand by the UI)
```
→ Handed off to user: Studio → Scan Damiao → Enable → Zero+Save (user clicks; Claude sends nothing).

### 3.3 Zero (user, in motorbridge-studio)
User clicked Scan Damiao → Enable → Zero+Save in https://motorbridge.github.io/motorbridge-studio/ and reports:
"Zeroing done and verified after a power cycle." Claude sent no commands to the arm during this step.

### 3.4 Gateway stop + post-zero rescan (read-only)
```
$ tail ~/rebot_setup/gateway_phase3.log
ws_gateway listening on ws://127.0.0.1:9002 (router_mode=standby, dynamic_target=true, dt_ms=20)
[ws_gateway] session closed: WebSocket protocol error: Handshake not finished        (15:58:11)
$ pkill -x ws_gateway          → exit 0   (used -x, not -f: `pkill -f ws_gateway` self-matches the
                                           harness shell whose argv contains the pattern; same target)
background task exit 241       = wrapper returning -15 (SIGTERM) — expected
$ pgrep -ax ws_gateway         → none ; fuser /dev/ttyACM0 → free ; no USB events since 15:51:48
$ stat /dev/ttyACM0            → mtime 16:19:36 (tty mtime has ~8 s granularity), ctime 15:54:04 (= chmod)
```
Gateway-log note: the only logged session is one failed handshake at 15:58:11 — the gateway does NOT log
successful sessions or commands. Independent evidence the Studio did reach the bus through it: right after
start the port had no holder (standby), but /dev/ttyACM0 was last written at ~16:19:36, i.e. up to the moment
the gateway was stopped (no other process had the port; Phase-2 scan was at 15:54).

```
$ env -u PYTHONPATH conda run -n rebot motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600 --start-id 1 --end-id 32
[hit] probe=0x01 esc_id=0x1 mst_id=0x11   …   [hit] probe=0x07 esc_id=0x7 mst_id=0x17
[.. ] probe=0x08 … 0x20  no reply
scan done: 7 motor(s) found
```
(full output: ~/rebot_setup/scan_s2_phase3_post_zero.txt) → all 7 still respond, IDs unchanged.

---

## SUMMARY — reBot Arm B601-DM bring-up (2026-10-10)

**What was done**
1. Session 1: Miniconda already present → created conda env `rebot` (py3.12), `pip install motorbridge` (0.5.6).
   First scan found only 1 motor (0x01 / MST 0x00 = all motors at factory defaults, colliding).
2. User set all 7 IDs in DMTool on Windows over the 3-pin cable, one motor at a time.
3. Session 2: env + adapter re-checked; user ran `sudo chmod 666 /dev/ttyACM0`; read-only scan → 7/7 correct.
4. User posed the arm at zero (folded sit-down, base forward, gripper closed); Claude started the gateway;
   user ran Scan → Enable → Zero+Save in motorbridge-studio and verified the zero after a power cycle.
5. Gateway stopped; read-only rescan → 7/7 still respond with correct IDs.

**Final IDs (verified by scan 15:54 and again post-zero 16:20)**

| Motor | Model | CAN (ESC_ID) | Feedback (MST_ID) |
|---|---|---|---|
| M1 | J4340P "4340P" | 0x01 | 0x11 |
| M2 | J4340P "4340P" | 0x02 | 0x12 |
| M3 | J4340P "4340P" | 0x03 | 0x13 |
| M4 | J4310 "4310"   | 0x04 | 0x14 |
| M5 | J4310 "4310"   | 0x05 | 0x15 |
| M6 | J4310 "4310"   | 0x06 | 0x16 |
| M7 | J4310 "4310" (gripper) | 0x07 | 0x17 |

**Verification scope:** the CLI scan reads only ESC_ID/MST_ID registers. It confirms IDs and that every motor
answers; it does NOT read joint position, so the zero itself is user-verified (Studio, after power cycle),
not independently checked by Claude.

**Start the gateway again later** (from any shell; `env -u PYTHONPATH` keeps ~/.bashrc's ROS py3.10 path out):
```
env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/motorbridge-gateway -- --bind 127.0.0.1:9002 --vendor damiao --transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600 --dt-ms 20
```
Wait for `ws_gateway listening on ws://127.0.0.1:9002`, then open motorbridge-studio. Stop it with Ctrl-C
(foreground) or `pkill -x ws_gateway`.

**Before each session / gotchas**
- After any adapter replug: `sudo chmod 666 /dev/ttyACM0` (until `sudo usermod -aG dialout $USER` + re-login
  is done — suggested, not run). The `!` prompt shell has no TTY, so run sudo in a real terminal.
- The gateway holds the serial port: stop it before any `motorbridge-cli scan` (else 0 motors).
- Re-check scan anytime (read-only):
  `env -u PYTHONPATH conda run -n rebot motorbridge-cli scan --vendor damiao --transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600 --start-id 1 --end-id 32`
- `motorbridge-cli`'s default subcommand is `run` → always type `scan` explicitly.
- Never run reBotArm_control_py `2_zero_and_read.py` (re-zeros every joint automatically).
- Don't run this stack alongside ~/rebot_lerobot or ~/rebot_ws drivers against the arm (same serial port).
- dmesg is restricted on this box → use `journalctl -k`.

---
---

# Session 3 — first powered movement, joint1 only (2026-10-10, from ~16:30)

Rules: Phase 1 read-only (no enable); no zero/ID/param/register writes, calibration, gravity comp, or mode change
without asking; nothing enables until the user has seen the exact script and replied "go"; ONLY motor 1 may ever
be enabled; repo ~/Projects/rebot_crack_vision is read-only context (clean tree @ 5b82e4e); scripts/logs here.
Pre-check: no ws_gateway, port free, claude-auto daemon stopped.

Context read: ADR-016 §3/§7/§8 (MOT-09 = operator bring-up; executor never auto-disables because of a gravity-fall
risk; this joint1 test is outside execute_trajectory), MOT-09 card, b601_dm_limits.yaml, ROBOT_MODEL.md.
Vendor config ~/rebot_ws/third_party/reBotArm_control_py/config/rebotarm_dm.yaml: no direction flags; joint1 MIT
kp=120 kd=8 (our cap kp≤10/kd≤1 is ~12x softer).

## S3 Phase 1 — read-only

### 1.1 SDK read-without-enable (verified in motorbridge v0.5.6 source, tag c652ce4, shallow clone in scratchpad)
- `Motor.request_feedback()` → `encode_feedback_request_cmd` = `[id,0,0xCC,0,0,0,0,0]` on 0x7FF = Damiao
  "refresh status". The motor replies with a normal feedback frame; no state change.
- `get_state()` returns the last decoded frame (local cache; sends nothing). Decoding uses the model's PMAX/VMAX/TMAX.
- Register read = `[id,0,0x33,rid,0..]` on 0x7FF. Enable `…FC`, disable `…FD`, zero `…FE`, write `0x55`, store `0xAA`.
- `enable()` = 1 frame; `send_mit()` = 1 frame. Neither calls ensure_mode or auto-enables.
- `add_damiao_motor` sends nothing (starts a receive-only background RX thread).
- `ctrl.shutdown()` = disable_all + close; `ctrl.close_bus()` = close only (no frames). Read-only scripts use close_bus.
- Feedback has NO timestamp and the ABI exposes no fresh-state call → Phase 2 needs its own staleness check.
- Scripts: readonly_bus.py (ReadOnlyMotor whitelist: request_feedback/get_state/get_register_* only).

### 1.2 Register dump (p1_registers.py → p1_registers.txt)
```
jnt id   model | CTRL_MODE | TIMEOUT | PMAX/VMAX/TMAX      model? | status    pos rad   vel    torq  Tmos Trot | rtt ms
j1  0x01 4340P | 1 MIT     | 0       | 12.50/10.0/28.0     OK     | DISABLED  -0.0002 -0.002 -0.007  27   25  | 2.1
j2  0x02 4340P | 1 MIT     | 0       | 12.50/10.0/28.0     OK     | DISABLED  -0.0002 -0.002 -0.034  26   24  | 2.1
j3  0x03 4340P | 1 MIT     | 0       | 12.50/10.0/28.0     OK     | DISABLED  -0.0002 -0.002 -0.007  26   24  | 2.2
j4  0x04 4310  | 1 MIT     | 0       | 12.50/30.0/10.0     OK     | DISABLED  -0.0002 -0.007 +0.002  28   25  | 2.1
j5  0x05 4310  | 1 MIT     | 0       | 12.50/30.0/10.0     OK     | DISABLED  -0.0002 -0.007 -0.007  28   26  | 2.2
j6  0x06 4310  | 1 MIT     | 0       | 12.50/30.0/10.0     OK     | DISABLED  -0.0002 -0.007 +0.002  28   27  | 2.2
j7  0x07 4310  | 1 MIT     | 0       | 12.50/30.0/10.0     OK     | DISABLED  -0.0002 -0.007 +0.017  29   26  | 2.2
```
→ All in MIT mode (no mode change needed). Models match. All DISABLED at 0 (−0.0002 = 1 LSB of the 16-bit
±12.5 rad encoding). TIMEOUT=0 on all → the motor-side CAN watchdog is OFF (an enabled motor holds its last
command if the host dies). Not changed (no register writes).

### 1.3 Sign check — live logger started
`p1_live_positions.py --hz 5 --duration 900` (background) → p1_live_positions.log / .csv. First samples: all 0.000, all DISABLED.
Expected directions from URDF FK at q=0 (base faces +x, z up, robot's left = +y):
j1 + = swing arm to robot's LEFT (CCW from above) | j2 + = elbow DOWN into fold → lifting upper arm = − |
j3 + = wrist end DOWN onto upper arm → raising forearm = − | j4 + = gripper nose pitches DOWN |
j5 + = gripper yaws to robot's RIGHT | j6 + = top of gripper rolls to robot's RIGHT | j7: no URDF joint
(fingers 0..0.0715 m, vendor close_gripper → 0) → opening presumably +.

### 1.3b Sign check result (p1_live_positions.csv, 5 Hz, all samples DISABLED; analysis in scratchpad sign_analysis.py)
User moved one joint at a time (J4 tipped UP instead of nose-down, as allowed).

| Joint | Hand motion | Peak reading | URDF prediction | Limits | Verdict |
|---|---|---|---|---|---|
| j1 | arm swung to robot's left | +0.412 rad (+23.6°) | + | ±2.8 | ✅ matches |
| j2 | upper arm lifted out of fold | −0.629 rad (−36.0°) | − | −3.14..0 | ✅ matches limits (max seen +0.002 = fold contact noise) |
| j3 | forearm lifted off upper arm | −0.367 rad (−21.0°) | − | −3.14..0 | ✅ matches limits |
| j4 | gripper nose tipped UP | −0.946 rad (−54.2°) | − (nose-up) | −1.87..1.57 | ✅ matches |
| j5 | gripper turned to robot's right | +1.090 rad (+62.5°) | + | ±1.57 | ✅ matches |
| j6 | gripper top rolled to robot's right | +1.296 rad (+74.2°) | + | ±3.14 | ✅ matches |
| j7 | gripper opened | −1.776 rad (−101.8°) | (no URDF joint; guessed +) | — | ⚠ opening = NEGATIVE motor angle (informational) |

Cross-talk: grabbing the wrist for j5 nudged j6 −0.054→+0.064 (it later returned to 0); j3's lift sagged j4 +0.076. No joint
contradicts its URDF limits/axis.
Rest check at t≈428 s: j1 +0.017, j2 +0.002, j3 −0.001, j4 −0.002, **j5 +0.057 (> 0.05)**, j6 +0.000, j7 +0.008
→ j5 not back within 0.05 rad; asked user to nudge it ~3° toward robot's left.

### 1.4 Rest check after the user's j5 nudge — PASS
Logger stopped via STOP_LIVE (close_bus, no frames); exit 0; 2766 samples over 553 s, every one DISABLED; port free.
Last 10 s (50 samples, zero spread on every joint):
j1 0.0170 | j2 0.0017 | j3 0.0013 | j4 0.0021 | j5 0.0315 | j6 0.0002 | j7 0.0078   (max |pos|, rad)
→ all |pos| < 0.05 rad. Phase 1 COMPLETE. STOPPED at the Phase-1 checkpoint, waiting for the user's "go" for Phase 2.
(Note: `pgrep -f '<pattern>'` self-matches the tool's own bash argv, like pkill -f; it falsely printed "STILL RUNNING".)

## S3 END — user: "Done for today, skip Phase 2." (2026-10-10)
Phase 2 NOT started: first_move_j1.py not written, no motor was ever enabled this session, no register written.
State left: all 7 motors DISABLED, at zero (≤0.032 rad), CTRL_MODE=1 (MIT), TIMEOUT=0; no process holds /dev/ttyACM0.
Files: readonly_bus.py, p1_registers.py(.txt), p1_live_positions.py(.csv/.log).
To resume Phase 2 later: re-run `p1_registers.py` (read-only) to re-confirm DISABLED + zero, then write
first_move_j1.py → show the user in full → run only on their "go". Open design points carried forward:
(1) staleness: no feedback timestamp in the SDK → fresh = feedback tuple changed OR a recent successful read-only
register round trip to motor 1 (~2 ms); (2) TIMEOUT=0 → motor 1 would hold its last target if the host died,
so use try/finally disable + the user's 24 V switch; set a motor-side timeout only if the user explicitly asks
(that's a register write).
