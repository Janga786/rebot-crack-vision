# Controller tuning of the B601-DM arm (week of 2026-10-12)

**What this is:** how the per-joint MIT controller gains, gravity compensation and friction compensation of the
physical reBot B601-DM were found on **2026-10-14** (first powered moves) and **2026-10-17** (tuning, sweeps,
multi-joint moves, two all-joint demos). Every number below comes from a logged hardware run; the raw logs,
code snapshot, analysis script and all figures are in
[`evidence/operator/controller-tuning-2026-10/`](../../evidence/operator/controller-tuning-2026-10/README.md).

**What this is not:** commissioning. Nothing in `config/robot/*` was changed; the only e-stop is the 24 V supply
switch; joint limits were not measured (vendor URDF values); all runs were operator-attended, each started on the
operator's explicit "go". The project's commissioning gate (MOT-05 / MOT-09) is unchanged.

**Result in one table** (each joint: hold error of a 0.10 rad move, gripper: 1.0 rad opening):

| Joint | Motor | kp (Nm/rad) | kd (Nm·s/rad) | Gravity scale | Friction ff (Nm) | Hold error (rad) | Before tuning (2026-10-14) |
|---|---|---|---|---|---|---|---|
| J1 | DM4340P | 100 | 4.0 | – (vertical axis) | 1.0 | 0.0012 | 0.045 (kp 25) |
| J2 | DM4340P | 100 | 1.0 | 0.6 | 1.0 | 0.0042 | 0.040 (kp 40) |
| J3 | DM4340P | 80 | 1.0 | 2.5 | 2.0 | 0.0031 | 0.050 (kp 100, no ff) |
| J4 | DM4310 | 40 | 1.0 | 2.5 | 0.5 | 0.0050 | 0.037 (kp 30) |
| J5 | DM4310 | 40 | 1.0 | 1.0 | 0.3 | 0.0037 | 0.0036 (kp 30) |
| J6 | DM4310 | 40 | 1.0 | 1.0 | 0.1 (0.3 swept) | 0.0019 | 0.0023 (kp 30) |
| J7 gripper | DM4310 | 40 | 0.5 | – | – | 0.0047 (1 rad open) | aborted (velocity spike) |

With these gains the 7-joint "show" demo (52 s, peak 0.56 rad/s on J1–J3) tracked every joint within
**0.025 rad** (J3 worst) and the gripper frame within **12.9 mm** of its planned Cartesian path.

---

## Contents

1. [Hardware and signal chain](#1-hardware-and-signal-chain)
2. [Control law](#2-control-law)
3. [Reference trajectories](#3-reference-trajectories)
4. [Safety envelope (the abort mathematics)](#4-safety-envelope)
5. [Gravity model from the URDF](#5-gravity-model-from-the-urdf)
6. [Identification of the real loads](#6-identification-of-the-real-loads)
7. [Feed-forward composition](#7-feed-forward-composition)
8. [Gain sweeps and the scoring function](#8-gain-sweeps-and-the-scoring-function)
9. [Multi-joint moves and the demos](#9-multi-joint-moves-and-the-demos)
10. [Offline simulator](#10-offline-simulator)
11. [Chronology, incidents and lessons](#11-chronology-incidents-and-lessons)
12. [Limitations and open items](#12-limitations-and-open-items)
13. [Figure catalogue](#13-figure-catalogue)
14. [Reproducing everything](#14-reproducing-everything)

---

## 1. Hardware and signal chain

| | |
|---|---|
| Motors | 7 × Damiao: J1–J3 **DM4340P** (40:1, rated 9 Nm, peak 27 Nm), J4–J6 + gripper J7 **DM4310** (10:1, rated 3 Nm, peak 7 Nm) |
| Bus | one USB-CAN serial bridge, `dm-serial` at 921 600 baud, `/dev/ttyACM0`; IDs motor N = 0x0N, feedback 0x1N |
| Host library | `motorbridge` 0.5.6 (conda env `rebot`) |
| Control mode | Damiao **MIT** mode (CTRL_MODE = 1) on every motor, checked before each enable, never written |
| Host loop | 200 Hz: one MIT frame per enabled motor per cycle; each frame is answered by a feedback frame |
| Motor zero | encoder zero = the folded pose (set by the vendor, never changed; scripts block `set_zero_position`) |

**Feedback resolution, measured from the logs** (smallest step between distinct reported values,
[`analysis/feedback_resolution.md`](../../evidence/operator/controller-tuning-2026-10/analysis/feedback_resolution.md)).
It matches the MIT frame encoding exactly, with the PMAX/VMAX/TMAX registers read before every run
(4340P: 12.5 rad / 10 rad/s / 28 Nm; 4310: 12.5 / 30 / 10):

| Quantity | Encoding | DM4340P LSB | DM4310 LSB |
|---|---|---|---|
| position | 16 bit over ±PMAX | 25/65535 = **0.000381 rad** | 0.000381 rad |
| velocity | 12 bit over ±VMAX | 20/4095 = **0.00488 rad/s** | 60/4095 = **0.01465 rad/s** |
| torque | 12 bit over ±TMAX | 56/4095 = **0.0137 Nm** | 20/4095 = **0.0049 Nm** |

**What the resolution means** (a sanity check, not a design choice — the encoding is fixed by the Damiao firmware
and motorbridge):
- *The logs are raw motor feedback.* Every logged value sits exactly on the MIT grid, which also confirms that the
  PMAX/VMAX/TMAX ranges read from the registers are the ones in use.
- *Sensing is not what limits the tuning.* Position resolution is 0.00038 rad (0.02°); the tuned tracking errors are
  10–60× larger. Friction and gravity-model error dominate (§2, §6).
- *Velocity is coarse on the DM4310s.* 0.0147 rad/s per step against moves of 0.1–1.2 rad/s, which is part of why
  their velocity read-outs are noisy. The aborting spikes (§11) were **19 and 26 steps** above the neighbouring
  samples (0.549 vs 0.271 and 0.506 vs 0.125 rad/s), so they are genuine jumps in the motor's velocity estimate, not
  rounding.
- *Possible improvement (not done):* the DM4310s encode velocity over ±30 rad/s while the arm never exceeds about
  1.2 rad/s, so ~96 % of the range is unused; a lower VMAX would give proportionally finer velocity feedback. It is a
  motor register write (commissioning-level), and the scripts deliberately refuse to run when PMAX/VMAX/TMAX differ
  from the per-model values the host is configured with, so it would also need confirming that motorbridge scales
  its MIT frames from the registers rather than from the model table (not verified).

**Loop timing** (figure f11): mean cycle 5.02 ms, 99th percentile 5.10 ms, worst 13 ms (single-joint sweep with 7
motors live) / 15 ms (7-joint demo). The slow cycles are most likely register "heartbeat" reads (§4) — not
verified; the 50 ms stream-gap abort never came close.

## 2. Control law

Every motor runs the MIT impedance law internally; the host only streams set-points:

$$
\tau_{\text{motor}} = k_p\,(q_{\text{ref}} - q) + k_d\,(\dot q_{\text{ref}} - \dot q) + \tau_{\text{ff}}
$$

- $q_{\text{ref}}, \dot q_{\text{ref}}$: the min-jerk reference (§3), sent every 5 ms.
- $k_p$ [Nm/rad], $k_d$ [Nm·s/rad]: the gains tuned here (units as the Damiao MIT convention; not independently
  verified).
- $\tau_{\text{ff}}$: gravity + friction feed-forward computed on the host (§7).

**Static error.** At rest ($\dot q = \dot q_{\text{ref}} = 0$) with gravity holding torque $G$ (the torque the motor
must supply to hold still) and Coulomb/static friction of magnitude up to $F_s$:

$$
k_p e + \tau_{\text{ff}} = G + f, \quad |f| \le F_s
\quad\Rightarrow\quad
e = \frac{G - \tau_{\text{ff}} + f}{k_p}, \qquad |e| \le \frac{|G - \tau_{\text{ff}}| + F_s}{k_p}
$$

This one inequality explains the whole tuning story: before compensation ($\tau_{\text{ff}} = 0$) J3 at kp 100
sat 0.05 rad short because $|G| + F_s \approx 6.6 + 2.3$ Nm; a correct gravity term removes $|G-\tau_{\text{ff}}|$
and friction feed-forward along the motion direction cancels most of $f$ while moving, leaving millimetre errors.

## 3. Reference trajectories

**Min-jerk** (figure f01) between $q_0$ and $q_1$ over $T$, with $s = t/T \in [0,1]$ and $\Delta = q_1 - q_0$:

$$
q(t) = q_0 + \Delta\,(10s^3 - 15s^4 + 6s^5), \qquad
\dot q(t) = \frac{\Delta}{T}\,(30s^2 - 60s^3 + 30s^4), \qquad
\ddot q(t) = \frac{\Delta}{T^2}\,(60s - 180s^2 + 120s^3)
$$

Peak speed $\dot q_{\max} = 1.875\,\Delta/T$ at $s = 0.5$; peak acceleration $\approx 5.77\,\Delta/T^2$. Position,
velocity and acceleration are continuous and zero at both ends, so the motor never sees a velocity step.

**Single-joint test move** (`first_move_j1.py`): settle 0.5 s → ramp to $q_0+\Delta$ → hold 1 s → ramp back → hold
1 s. A **sweep** repeats this block once per $(k_p, k_d)$ pair inside a single enable; gains switch only at block
boundaries, where the joint is at rest at $q_0$.

**Demo paths** (`demo_move.py`): piecewise min-jerk through absolute joint waypoints, all 7 joints simultaneously
with a common segment duration; a segment may be marked "auto", in which case

$$
T = \max\!\Big(2\text{ s},\ \max_j \frac{1.875\,|\Delta_j|}{0.9\,v_{\text{plan},j}}\Big)
$$

with the planned-speed caps $v_{\text{plan}}$ = 0.6 rad/s (DM4340P) and 1.2 rad/s (DM4310).

## 4. Safety envelope

Every condition below aborts a run; all were exercised offline (53 + 12 unit tests against the simulator) and
several fired for real (§11).

| Check | Single-joint script | Demo script |
|---|---|---|
| tracking $\lvert q_{\text{ref}} - q\rvert$ | > 0.10 rad (moving joint), > 0.15 rad (held joints) | > 0.10 rad (any joint) |
| torque $\lvert\tau\rvert$ | $> k_p e_{\max} + k_d v_{\max} + 0.5 + \lvert\tau_{\text{ff}}\rvert$ | same formula, per joint |
| velocity $\lvert\dot q\rvert$ | 0.5 rad/s (DM4340P), 1.0 (DM4310, see §11) | 1.0 / 1.6 rad/s |
| feedback freshness | no new frame for 0.2 s (a frame counts as new if any field changed, else a register read must succeed — "heartbeat") | same |
| stream gap | > 50 ms between command cycles | same |
| motor status | anything but ENABLED (over-current, over-temp, lost comm …) | same |
| feed-forward cap | $\lvert\tau_{\text{ff}}\rvert$ > 10 Nm (4340P) / 2.5 Nm (4310) | same |
| pre-enable | bus answers 7/7, all DISABLED, MIT mode, PMAX/VMAX/TMAX match the model, motorbridge 0.5.6, no other process on the port | + start in the fold (each joint within 0.10 rad of 0; gripper 0.40) |
| path (demo only) | – | every waypoint inside the limits; planned peak speed per joint ≤ cap; **gripper_link z ≥ 0.12 m** at every 0.05 s of the path (forward kinematics, §5) |

**Torque bound.** The abort level is the largest torque a healthy loop can command inside the other bounds:
$k_p$ times the tracking bound, plus $k_d$ times the velocity bound, plus 0.5 Nm margin, plus the feed-forward. A
torque reading above it means the motor is pushing against something (a jam, a person, the table).

**Abort behaviour.**
- *Single-joint script*: hold the measured position 0.5 s, then disable every motor and confirm each disable from
  its feedback frame.
- *Demo script* (the arm may be raised): **park** — a min-jerk return of all joints (except the base, which carries
  no gravity) from the measured pose to the fold over
  $T_{\text{park}} = \max(3\text{ s},\ 1.875\,\max_j\lvert\Delta_j\rvert / 0.25)$, tolerant of velocity/torque
  read-outs but stopped by tracking > 0.25 rad, bus or freshness failures; then disable all. Bus loss, stale
  feedback or a motor fault cannot be parked through and go straight to hold-and-disable.

The 24 V switch remains the e-stop; cutting it while the arm is raised drops it.

## 5. Gravity model from the URDF

Source: the vendor URDF `reBot_B601_DM_with_gripper.urdf` (copied into the evidence `code/model/`), 10 links,
total mass 2.744 kg. Implementation: [`code/gravity.py`](../../evidence/operator/controller-tuning-2026-10/code/gravity.py),
pure Python, 0.32 ms per evaluation (fast enough to recompute every 5 ms cycle from the measured joint angles).

**Forward kinematics.** For each joint $i$ (topological order from `base_link`), with origin $(\mathbf{x}_i,
\text{rpy}_i)$ and unit axis $\hat{\mathbf{a}}_i$ in its own frame:

$$
{}^{0}T_{\text{child}(i)} = {}^{0}T_{\text{parent}(i)}\ \cdot\ T(\mathbf{x}_i, R_{\text{rpy}}(\text{rpy}_i))\ \cdot\ \operatorname{Rot}(\hat{\mathbf{a}}_i, q_i)
$$

$$
R_{\text{rpy}}(r,p,y) = R_z(y)\,R_y(p)\,R_x(r), \qquad
\operatorname{Rot}(\hat{\mathbf a}, q) = I + \sin q\,[\hat{\mathbf a}]_\times + (1-\cos q)\,[\hat{\mathbf a}]_\times^2 \ \ \text{(Rodrigues)}
$$

The joint's world axis and position are taken *before* its own rotation:
$\mathbf a_i = {}^0R_i^{\text{pre}}\hat{\mathbf a}_i$, $\mathbf p_i = {}^0\mathbf t_i^{\text{pre}}$.
Each link's centre of mass is $\mathbf c_k = {}^0T_k \cdot \mathbf c_k^{\text{local}}$ from its `<inertial><origin>`.

**Gravity torque and holding torque.** With $\mathbf g = (0, 0, -9.81)$ m/s² and $\mathcal S(i)$ the set of links
distal to joint $i$:

$$
\tau_{g,i}(q) = \mathbf a_i \cdot \sum_{k \in \mathcal S(i)} (\mathbf c_k - \mathbf p_i) \times m_k\,\mathbf g,
\qquad
G_i(q) = -\,\tau_{g,i}(q)
$$

$G_i$ is the torque the motor must apply to hold the arm still, i.e. what the feed-forward should be. Extra point
masses (camera, motors missing from the URDF) are added as $(\text{link}, \mathbf r^{\text{local}}, m)$ and enter
$\mathcal S(i)$ exactly like links.

**Values at the fold ($q = 0$):** $G = (0, -1.114, -2.789, -0.673, 0, 0)$ Nm. URDF masses: base 0.799, link1 0.084,
link2 0.710, link3 0.525, link4 0.191, link5 0.150, link6 0.019, gripper 0.182 (+2 × 0.042 fingers) kg.

**Sign check.** A negative J3 torque lifts the arm away from the fold, and a real J3 hold at the fold measured
−5.1 Nm with $\tau_{\text{ff}} = 0$ → the model's sign (−2.79) is right; the magnitude is not (§6). Note the URDF
gives J2 axis $(0,0,-1)$ and J3 axis $(0,0,+1)$ after their frames, so the forearm's absolute orientation depends
on $q_3 - q_2$: lifting J2 alone swings the gripper *down* (gripper_link z 0.192 → 0.070 m at $q_2 = -0.5$), which
is why every demo rise moves J2 and J3 together and why the path check exists.

Figure f04 shows $G$ along the demo's rise for J2–J4: URDF, the scaled model that is actually used, and the
extra-mass hypothesis of §6.4.

## 6. Identification of the real loads

### 6.1 J3: torque–position hysteresis (run `first_move_j3_20261017T191042Z`, figure f03)

With every other joint held and the URDF gravity ×2.5 on J3, J3 was ramped 0.10 rad up and back at kp 100. At
standstill the motor torque must satisfy $\lvert\tau - G\rvert \le F_s$, so the joint sticks at the two edges of a
band centred on the true gravity torque:

| | Torque (Nm) |
|---|---|
| stuck while going up (end of the up-hold) | −8.87 |
| stuck while coming down (final hold) | −4.33 |
| **band centre ⇒ gravity** $G_3 \approx \tfrac{1}{2}(-8.87 - 4.33)$ | **−6.60** |
| **half width ⇒ static friction** $F_s \approx \tfrac12\lvert -8.87 + 4.33\rvert$ | **±2.27** |
| URDF model at the same pose | −2.72 |
| ⇒ scale $s_3 = 6.60 / 2.72$ | **2.42 → 2.5 used** |

The ×2.5 feed-forward (−6.79 Nm) sits 0.19 Nm from the identified centre. The earlier, un-identified runs are
consistent with it: with $\tau_{\text{ff}} = 0$ the hold needed −5.1 Nm at 0.050 rad error (inside the band), with
the URDF feed-forward (−2.8) the error was 0.022 rad.

### 6.2 J4

Held at kp 20 while J3 lifted, J4 sagged +0.05 rad and needed −1.94 Nm against the model's −0.67 (×2.9). In its own
sweep with ×2.5 (−1.68 Nm) the lifted hold settled at −1.90 Nm with $k_p e = 40 \times 0.005 = 0.2$ Nm making up the
rest, confirming ×2.5. (Coming back down J4 stopped ~0.02 rad above its start because the wrist lands on its rest
on the J2 motor — a contact, not a tracking error.)

### 6.3 J2

J2 went the other way. With URDF ×1 (−1.05 Nm) and 1.5 Nm friction help it **overshot** the lift target by
~0.007 rad on every gain pair and held at −0.42…−0.61 Nm, i.e. it needs *less* than the model. ×0.6 and 1.0 Nm
friction removed the overshoot (0 in both pairs) and halved tracking error (0.0063/0.0052 vs 0.0094/0.0076 rad).
J2's load near the fold is confounded by the support the base sits on and the forearm resting on the J2 motor, so
0.6 is a local, empirical value.

### 6.4 Why the real arm is heavier than the URDF (hypothesis, not yet measured)

A camera alone cannot explain it: the D405 + mount (~0.1 kg) at its CAD pose adds only −0.42 Nm at J3. But the URDF
link masses are too light to contain the wrist motors (link6 19 g, link4/5 150–190 g, while a DM4310 is roughly
0.3 kg — estimate, not weighed). Adding 0.3 kg at the link5, link6 and gripper origins plus the camera gives:

| | J2 | J3 | J4 |
|---|---|---|---|
| extra torque predicted at the fold | +1.27 | −3.86 | −1.48 Nm |
| extra torque measured (identified − URDF) | +0.45 | −4.01 | −1.01 Nm |

J3 and J4 agree well; J2 does not (support contact, §6.3). Weighing the camera+mount and a spare DM4310 would let
the scale factors return to ≈1. Along the whole demo path the scaled model and this hypothesis differ by at most
0.83 Nm (J2), i.e. ≤ 0.012 rad at the tuned gains — which is why the demos tracked well despite the model gap.

## 7. Feed-forward composition

For joint $i$ at every 5 ms cycle (using the **measured** angles of all joints):

$$
\tau_{\text{ff},i}(t) = \underbrace{\big(\min(s_i,1) + \phi(t)\,\max(s_i - 1, 0)\big)\, G_i(q_{\text{meas}})}_{\text{gravity}}
\;+\; \underbrace{F_{c,i}\,\tanh\!\big(\dot q_{\text{ref},i}/\varepsilon\big)}_{\text{friction}}
\;+\; \tau_{\text{manual}}
$$

- $s_i$: per-joint gravity scale (§6); $\phi(t) = \min(1, t/1\,\text{s})$ fades in only the part of the scale above 1
  after enable. Reason: at the fold the forearm rests on the J2 motor, so J3 initially carries much less than its
  "lifted" load; the full ×2.5 step at enable kicked J3 to 0.535 rad/s within 15 ms (run `…190919Z`, abort). The
  plain URDF part is applied immediately so nothing sags at enable.
- $F_{c,i}$: Coulomb friction compensation (J1 1.0, J2 1.0, J3 2.0, J4 0.5, J5 0.3, J6 0.1–0.3 Nm), switched by the
  *commanded* velocity through $\tanh(\dot q_{\text{ref}}/\varepsilon)$, $\varepsilon = 0.02$ rad/s (figure f13). Using the
  reference instead of the noisy measured velocity avoids chatter at standstill: when $\dot q_{\text{ref}} = 0$ the term
  is exactly zero, so holds are not biased.
- Caps: total $\lvert\tau_{\text{ff}}\rvert$ ≤ 10 Nm (4340P) / 2.5 Nm (4310), checked after the friction term (the
  first J3 friction simulation hit the old 7 Nm cap: 6.8 gravity + 2 friction).

## 8. Gain sweeps and the scoring function

Each sweep runs $N$ back-and-forth blocks (§3), one per gain pair, in one enable, with all other joints held at their
hold gains (J1–J3 kp 60/kd 1, J4–J6 kp 30/kd 0.5, gripper kp 8/kd 0.3) and gravity feed-forward. Per block $k$ with
tracking error $e = q_{\text{ref}} - q$ and move direction $\sigma = \operatorname{sgn}(\Delta)$:

$$
\text{RMS}_k = \sqrt{\tfrac{1}{|R|}\textstyle\sum_{R} e^2}, \quad R = \text{ramp samples}
\qquad
\bar e_k = \text{mean}_{H}(e), \quad H = \text{hold samples}
$$

$$
O_k = \max\big(0,\ \max_H(-\sigma e)\big) \ \text{(overshoot past the target)}, \qquad
N_k = \#\{\text{sign flips of } \dot q \text{ with } |\dot q| > 0.05 \text{ rad/s in holds}\}
$$

$$
J_k = \text{RMS}_k + |\bar e_k| + 2\,O_k + 0.01\,N_k \qquad (J_k = \infty \text{ if the block did not finish})
$$

Overshoot is weighted ×2 because for crack tracing an overshoot is worse than lagging; oscillation flips are a
tie-breaker. All tables: [`analysis/SWEEPS.md`](../../evidence/operator/controller-tuning-2026-10/analysis/SWEEPS.md);
plots f05 (one per sweep) and f06 (summary).

**Findings**
- **kp:** tracking improved monotonically with kp on every joint (f06); the best kp was the top of the tested range
  (100 on J1/J2, script cap; 40 on J4–J6). J3 picked 80 over 100 on hold error inside its friction band —
  effectively a tie.
- **kd** (cap raised 1 → 5 for this): J1 RMS 0.0029 → 0.0019 → **0.0013** rad for kd 1/2/4; J2 and J3 showed no
  improvement (scores 0.0069/0.0072/0.0078 and 0.0133/0.0157/0.0158). J1 rotates about the vertical with only
  ~1 Nm friction, so damping shapes its response; J2/J3 are friction-dominated (§2), so damping is irrelevant there.
  The vendor's kd 10 (J1–J3) was not needed.
- **Wrist (J4–J6):** kd 0.5 at kp 30 showed a small oscillation on J4 (1 flip) and J6 at kp 20; kd 1.0 removed it.
  J6 overshot 0.002 rad with 0.3 Nm friction ff → 0.1 Nm adopted.

## 9. Multi-joint moves and the demos

**First multi-joint move** (`first_move_j1_20261017T192751Z`): J1 +0.10 with J2, J3, J4 each −0.10 (`--also`, tuned
gains): max errors J1 0.0035, J2 0.0123, J3 0.0204, J4 0.0189 rad; J3's hold error grew to 0.0096 rad (vs 0.003
alone) because its gravity load changes when J2 moves at the same time.

**Demos** (`demo_move.py`, all 7 joints, figures f09 tracking and f10 3-D gripper path from forward kinematics):

| Max / RMS tracking error (rad) | J1 | J2 | J3 | J4 | J5 | J6 | J7 | gripper path max / RMS |
|---|---|---|---|---|---|---|---|---|
| "wave", 38 s, peak 0.375 rad/s | 0.0078 / 0.0056 | 0.0196 / 0.0133 | 0.0212 / 0.0132 | 0.0160 / 0.0103 | 0.0097 / 0.0061 | 0.0083 / 0.0010 | 0.0130 / 0.0033 | 12.3 / 8.6 mm |
| "show", 52 s, peak 0.56 rad/s, facing the audience (J1 −3.76) | 0.0074 / 0.0032 | 0.0245 / 0.0109 | 0.0251 / 0.0131 | 0.0173 / 0.0054 | 0.0108 / 0.0025 | 0.0199 / 0.0022 | 0.0184 / 0.0041 | 12.9 / 7.4 mm |

50 % faster moves cost only ~0.004 rad of extra worst-case error. The operator flow was: first "go" → enable, move
every joint to 0 and hold; second "go" (passed to the running process as a file) → run; no second go within 5 min
→ disable at the fold.

## 10. Offline simulator

`code/fakebus.py` simulates the seven motors on a fake bus in simulated time (no hardware, no motorbridge). Each
joint is a rigid body with Coulomb + viscous friction and stiction, integrated with explicit Euler at 0.5 ms:

$$
J\,\ddot q = \operatorname{clip}_{\pm T_{\max}}\!\big(k_p(q_{\text{ref}}-q) + k_d(\dot q_{\text{ref}}-\dot q) + \tau_{\text{ff}}\big) + \tau_{\text{load}} - b\,\dot q - F_c\operatorname{sgn}(\dot q),
\qquad \dot q \leftarrow 0 \ \text{ if } |\dot q| < 10^{-4} \text{ and } |\tau| \le F_c
$$

Defaults $J = 0.07$ kg·m², $b = 0.5$ Nm·s/rad, $F_c = 0.3$ Nm; for J3 the identified $F_c = 2.3$ Nm and $J = 0.3$
were used, and $\tau_{\text{load}} = -s\,G(q_{\text{rest}})$ (applied only while the motor is enabled — when disabled
the joint rests on its stop). Faults can be injected (jam, torque spike, frozen feedback, other motor enabled or
moved, bus error, Ctrl-C, e-stop).

Figure f14 replays the J3 sweep in the simulator with these parameters next to the real run: the steady-state hold
errors match in sign and size (−0.004 … −0.009 rad), the real arm shows sharper stick-slip on the ramps, and the
simulator's first block starts with a dip that the real arm does not have (the simulated J3 is not resting on its
stop at enable). Two simulator bugs were found and fixed during the week: enabling a motor integrated the whole
disabled interval as if it had been powered (a fake 30 ms "fall"), and a park loop that skipped its pacing sleep
spun forever in simulated time (on hardware it would have busy-looped).

## 11. Chronology, incidents and lessons

Full list with outcome and notes for every run: [`analysis/RUNS.md`](../../evidence/operator/controller-tuning-2026-10/analysis/RUNS.md)
(41 runs: 28 completed, 6 aborted, 5 refused before enable, 2 read-only checks).

**2026-10-14 — first powered moves** (one joint at a time, others disabled, no feed-forward; figure f16):
J1 kp 10 did not break static friction (reached 0.083 of 0.15 rad), kp 25 reached +0.105; J2 kp 40 −0.06 of −0.10;
J3 kp 60 failed to lift, kp 100 reached −0.06 with ~5.2 Nm; J4 out to +0.075, back only to +0.058 (gravity);
J5, J6 kp 30 tracked well; J7 gripper aborted on a velocity reading. One run aborted when the unpowered J4 swung
0.106 rad as J3 lifted.

**2026-10-17 — tuning.** In order:

1. **J7 velocity spikes.** Two aborts at 0.549 and 0.505 rad/s while the gripper actually moved ~0.17 rad/s (f07):
   single samples, jumps of 19 and 26 LSB (0.0147 rad/s) above their neighbours — not quantisation. ⇒ per-joint velocity abort (DM4310 1.0 rad/s), tracking
   error stays the real guard. *Process error:* after this code change one run (`…184241Z`) was started without a
   fresh operator go; it completed safely; the rule "never re-run after a code change without a new go" was
   re-affirmed.
2. **Manual J3 feed-forward −2.8 Nm** — the first attempt (`…184744Z`) changed nothing because the motor wrapper
   still hard-coded $\tau_{\text{ff}} = 0$; after the fix (`…184845Z`) the hold error went 0.050 → 0.022 rad.
3. **Other joints not stiff** (operator: "J4 is falling when J3 lifts") ⇒ `--hold-others`: every other motor
   enabled and held where it is; per-held-joint aborts; all disabled on any abort.
4. **URDF gravity on every motor** ⇒ J3 needed ~−7 Nm vs model −2.8 ⇒ identification (§6.1) ⇒ ×2.5. A full ×2.5 step
   at enable kicked J3 (abort) ⇒ fade-in of the excess (§7).
5. **Friction feed-forward** ⇒ J3 hold error 0.020 → 0.003 rad.
6. **Sweeps** J3, J2 (twice), J1, J4 (one abort: J4 velocity spike at enable ⇒ DM4310 limit 1.0 for J4–J6), J5,
   J6; then **kd sweeps** J1–J3.
7. **Multi-joint** (`--tuned --also`), then the **demos** with path checks, park-on-abort, the fold start check and
   the two-go flow; then the faster "show" facing the audience.

**Recurring harmless refusal:** six times a run was refused before enabling because one motor (J2, J3+J5, J5, J6
twice, J7) missed a single 200 ms register read during the pre-enable re-check; an immediate retry always worked.
Three of these are in the logs (`…164908Z`, `…190515Z`, `…191359Z`); the other three were seen in the session
output but their summaries were overwritten by the retry, which started within the same second (log names have 1 s
resolution). The two other refusals of the week (`…164229Z`, `…164348Z`) were the start-at-rest check, since
removed. Worth investigating (bus timing after the previous run's disable), not a safety issue.

**Lessons**
- A correct feed-forward is worth far more than gain: J3 went from 0.050 to 0.003 rad at the *same* kp.
- Identify loads on the real arm (the hysteresis band) before trusting a CAD/URDF mass model.
- Velocity read-outs of the DM4310 are not trustworthy sample-to-sample; abort logic must not hinge on them.
- Contacts (forearm resting on the J2 motor, J2 on its support) dominate near the fold and make "gravity scale"
  pose-dependent; tuning should eventually be repeated away from the fold.
- Every safety mechanism needed its own test in the simulator — two simulator bugs and one real control bug
  (the dropped feed-forward) were found that way.

## 12. Limitations and open items

- **Not commissioning.** No hardware e-stop beyond the 24 V switch; joint limits are vendor URDF values, not
  measured; MOT-09's commissioning steps (switch tests, limit finding) remain open.
- **J1 "audience" angle −3.76 rad is beyond the URDF ±2.8.** Allowed in the demo script only (to −3.9) at the
  operator's request; watch the base cable harness. Multi-turn position after a power cycle with J1 turned has not
  been verified — return J1 near 0 before powering off.
- **Gravity scales are local** (identified near the fold) and stand in for missing masses (§6.4).
- **Gains are for 0.1 rad test moves and the two demos**; crack-tracing speeds and contact forces were not tested.
- **kp/kd units** are taken as Nm/rad, Nm·s/rad per the Damiao MIT convention; not independently verified.
- **Missed register reads** before enable (§11) are unexplained.
- **DM4310 velocity resolution** (0.0147 rad/s) could be improved ~25× by a lower VMAX (§1); not attempted.

## 13. Figure catalogue

All in [`evidence/operator/controller-tuning-2026-10/figures/`](../../evidence/operator/controller-tuning-2026-10/figures/),
regenerated by `analysis/analyze.py`.

| Figure | Shows |
|---|---|
| f01_min_jerk_profile | normalised min-jerk position / velocity / acceleration |
| f02_j3_compensation_progression | J3 lift: no ff → URDF ff → held + gravity ×1 → ×2.5 → + friction (position and error) |
| f03_j3_torque_position_hysteresis | the identification plot: stick-up/stick-down band, gravity centre, friction half-width |
| f04_gravity_model_along_rise | G for J2–J4 along the rise: URDF, scaled (used), extra-mass hypothesis |
| f05_sweep_<joint>_<run> | one per sweep: position, error and score per gain pair |
| f06_sweep_summary | RMS and hold error vs kp for all joints; RMS vs kd for J1–J3 |
| f07_velocity_spikes_j7 | reported vs planned vs numerically differentiated velocity; the abort sample |
| f09_demo_{wave,show}_tracking | all 7 joints: reference vs measured, and error |
| f10_demo_{wave,show}_gripper_path_3d | gripper_link path by forward kinematics, measured vs reference |
| f11_loop_timing | cycle-period histograms (sweep with 7 motors live; demo) |
| f12_held_j4_during_j3_lift | held-wrist sag before/after stiffer holds and the ×2.5 scale |
| f13_friction_feedforward | the tanh switching function; J3 error with/without friction ff |
| f14_j3_sweep_sim_vs_real | the same J3 sweep on the real arm and in the simulator |
| f16_first_powered_moves_2026-10-14 | every first single-joint move of 2026-10-14 |

## 14. Reproducing everything

```bash
# figures, RUNS.md, SWEEPS.md, metrics.json, feedback_resolution.md (system python3 with numpy + matplotlib)
python3 evidence/operator/controller-tuning-2026-10/analysis/analyze.py

# offline tests of the motion scripts (no hardware)
cd evidence/operator/controller-tuning-2026-10/code
python3 -m unittest test_first_move_j1 test_demo_move

# simulated runs (no hardware)
python3 first_move_j1.py --simulate --joint j3 --move -0.10 --hold-others --gravity \
    --gravity-joint-scale j3=2.5,j4=2.5 --friction 2.0 --sweep 60:0.5,80:1.0,100:1.0 --ramp-s 1
python3 demo_move.py --simulate --choreo show
```

On the arm (operator present, each run on the operator's go; the scripts live in the operator workspace
`~/rebot_setup/`, the copies here are the exact versions used):

```bash
env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python first_move_j1.py --operator-go "go" --joint j3 --tuned \
    --move -0.10 --hold-others --gravity
env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python demo_move.py --operator-go "go" --choreo show \
    --wait-go ~/rebot_setup/runs/DEMO_GO      # second go: echo go > ~/rebot_setup/runs/DEMO_GO
```
