# J4 friction / gain analysis of the powered show runs — 2026-10-29

`tools/analyze_run_gains.py --friction-fit` over the seven powered show runs of 2026-10-23/25 (the logs in
[../powered-show-runs-2026-10-23_25/runs](../powered-show-runs-2026-10-23_25/README.md), byte-identical to
`~/rebot_setup/runs/traj_*.csv`). Workstation only: nothing here ran the arm. Plan:
[docs/motion/J4_TUNING_PLAN.md](../../../docs/motion/J4_TUNING_PLAN.md); write-up and predictions:
[docs/motion/J4_TUNING.md](../../../docs/motion/J4_TUNING.md).

| File | What |
|---|---|
| `analysis.txt` | the table output: the existing per-joint statistics, then per run the friction fit of J1-J5 on segment 1 (the trajectory) and per segment, with the what-if `--what-if-friction j4=0.18`; at the end the across-run implied Coulomb |
| `analysis.json` | the same as JSON (`--json`), every coefficient, all three segments |
| `analysis_whatif_kp50.txt` | the same with `--what-if-friction j4=0.18 --what-if-kp j4=50` (the what-if column only changes) |

Reproduce:

```bash
E=evidence/operator/powered-show-runs-2026-10-23_25/runs
RUNS="$E/traj_heart_20261023T205033Z.csv.gz $E/traj_heart_20261023T205318Z.csv.gz $E/traj_heart_20261025T214843Z.csv.gz \
      $E/traj_figure8_20261023T204854Z.csv.gz $E/traj_showreel_20261025T213800Z.csv.gz $E/traj_box_20261025T215102Z.csv.gz \
      $E/traj_figure8_20261023T204705Z.csv.gz"
./env.sh python tools/analyze_run_gains.py $RUNS --friction-fit --what-if-friction j4=0.18 > evidence/operator/j4-tuning-2026-10-29/analysis.txt
./env.sh python tools/analyze_run_gains.py $RUNS --friction-fit --what-if-friction j4=0.18 --json > evidence/operator/j4-tuning-2026-10-29/analysis.json
./env.sh python tools/analyze_run_gains.py $RUNS --friction-fit --what-if-friction j4=0.18 --what-if-kp j4=50 > evidence/operator/j4-tuning-2026-10-29/analysis_whatif_kp50.txt
```

## What the fit is

For each joint (J1-J5) and each segment of the moving part of a run (`mode == path`: 0 = approach from the fold,
1 = the trajectory, 2 = return), two least-squares fits on the same regressors:

- the tracking error $e = q_{ref} - q$ (+ = lagging), and
- the torque the P/D terms supplied, $\tau - \tau_{ff}$ (motor-reported torque minus the logged feed-forward),

on $[\,1,\ \tanh(\dot q_{ref}/0.02),\ \dot q_{ref},\ \ddot q_{ref}\,]$, with $\dot q_{ref}$ and $\ddot q_{ref}$ by
`np.gradient` of the logged `ref` (200 Hz). The tanh is the runner's own smooth sign (`FRICTION_VEL_EPS` 0.02 rad/s),
so the sign coefficient is directly "how much the Coulomb feed-forward is off":

- **sign term** of the error, mrad, and the same x the run's kp (Nm);
- **sign term** of the torque fit, Nm. Negative = the feed-forward pushes harder than the joint's friction (the joint
  runs ahead of the reference);
- **implied Coulomb** = the run's Coulomb feed-forward (from its summary; tuned 0.5 Nm on J4) + the torque sign term;
  also split by direction (`[up, down]`: separate sign columns for positive and negative velocity);
- constant (the pose-dependent gravity error, the gravity model's job), viscous and inertia coefficients, R²;
- **VIF** of the sign column (how well the other columns predict it): > 15 is flagged `*` (sign and constant confounded,
  e.g. a segment that moves in one direction only);
- **coherence** of the joint's error with J3's (Welch, Hann, 1024 samples = 5.1 s, 50 % overlap): its largest value in
  2.5-3.5 Hz and where, the level a zero true coherence stays under 95 % of the time with that many segments
  ($1 - 0.05^{1/(n-1)}$), and the joint's 2.5-40 Hz band rms split into the part linearly shared with J3
  ($\sum C\,P_{xx}$) and the rest;
- **what-if**: the error had the run used Coulomb X (each sample moved by $(F_{run} - X)/k_p \cdot \tanh(\dot q_{ref}/0.02)$)
  and/or kp K (the whole error x $k_{p,run}/K$, quasi-static). "rms" is the standard deviation about the mean, as in the
  tool's other columns.

## Result (J4, segment 1)

| run | sgn (mrad) | sgn x kp (Nm) | sgn tq-ff (Nm) | viscous (Nm·s/rad) | inertia (Nm·s²/rad) | R² (e) | implied Coulomb (up / down) | corr e3 | coherence e3 (n seg, 95 % zero) | band rms shared / own (mrad) | rms → what-if 0.18 (mrad) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| heart 10-23 205033 (kd 4 J2/J3) | −8.1 | −0.32 | −0.324 | −0.17 | −0.002 | 0.93 | 0.18 (0.21 / 0.15) | −0.40 | 0.56 @ 3.32 Hz (7, 0.39) | 0.81 / 0.95 | 7.6 → 2.1 |
| heart 10-23 205318 (kd 5) | −7.9 | −0.32 | −0.321 | −0.14 | −0.001 | 0.93 | 0.18 (0.21 / 0.15) | −0.54 | 0.62 @ 3.32 (7, 0.39) | 0.77 / 0.93 | 7.5 → 2.0 |
| heart 10-25 214843 (ff-bias) | −8.5 | −0.34 | −0.346 | +0.45 | −0.001 | 0.91 | 0.15 (0.15 / 0.16) | −0.55 | 0.47 @ 2.54 (7, 0.39) | 0.92 / 1.05 | 7.4 → 2.3 |
| figure-8 10-23 x1 | −7.6 | −0.30 | −0.320 | −0.27 | −0.012 | 0.91 | 0.18 (0.23 / 0.12) | −0.35 | 0.91 @ 2.73 (3, 0.78) | 1.44 / 0.90 | 7.7 → 2.4 |
| showreel 10-25 | −8.4 | −0.34 | −0.338 | −0.15 | 0.000 | 0.90 | 0.16 (0.17 / 0.16) | −0.35 | 0.12 @ 2.54 (30, 0.10) | 0.61 / 1.05 | 7.5 → 2.4 |
| box 10-25 (\|v\| ≤ 0.07) | −4.7 | −0.19 | −0.190 | −2.20 | −0.001 | 0.78 | 0.31 (0.33 / 0.29) | +0.36 | 0.53 @ 3.32 (8, 0.35) | 0.47 / 0.48 | 4.8 → 2.4 |
| figure-8 10-23 x2 | −7.9 | −0.32 | −0.318 | −0.56 | −0.001 | 0.92 | 0.18 (0.13 / 0.25) | −0.13 | 0.22 @ 2.93 (8, 0.35) | 0.61 / 0.94 | 7.7 → 2.2 |

**Implied J4 Coulomb, across the 7 runs: 0.192 ± 0.053 Nm (sd; range 0.154-0.310); without the box 0.172 ± 0.011 Nm
(6 runs).** The runner feeds forward 0.5 Nm. The plan's §2 table (six runs, without the figure-8 x2) is reproduced
to the digit: sign term −8.1/−7.9/−8.5/−7.6/−8.4/−4.7 mrad, −0.32/−0.32/−0.35/−0.32/−0.34/−0.19 Nm, viscous
−0.16/−0.13/+0.46/−0.27/−0.15/−2.2 (here −0.17/−0.14/+0.45, a rounding of the same fit), inertia, R² 0.93/0.93/0.91/
0.91/0.90/0.78, corr −0.40/−0.54/−0.55/−0.35/−0.35/+0.36. Coherence: the plan's "0.5-0.56 at 3.3 Hz in the hearts and
the box, 0.12 in the showreel" is this tool's 0.56/0.62/0.47/0.53 and 0.12.

Other joints, same fit (segment 1, across runs): **J5 is also over-compensated**: implied 0.116 ± 0.013 Nm against
0.3 fed forward, sign term −4.5 mrad, R² 0.86-0.89 (the gravity fit found 0.12, GRAVITY_MODEL §5.2). J1 0.99 ± 0.07
(1.0 fed forward), J2 1.16 ± 0.15 (1.0), J3 1.99 ± 0.14 (2.0): right on; their errors are not friction-dominated
(R² 0.01-0.33: gravity and the rest at the fold dominate).

## Caveats

- `ref` is logged with 4 decimals (0.1 mrad), so `np.gradient` gives velocity steps of 0.01-0.02 rad/s and the
  acceleration column is mostly quantisation noise (|a_ref| up to 3-9 rad/s² in the figure-8/showreel). The inertia
  coefficient is therefore not meaningful, but bounded: < 0.01 Nm either way. On the hearts, figure-8s and showreel
  the sign and viscous terms are insensitive to this (a 21- or 41-sample Savitzky-Golay derivative instead moves the
  sign term by <= 0.15 mrad and the viscous term by <= 0.08 Nm·s/rad); on the box it does not (sign −4.7 → −6.5 mrad,
  viscous −2.2 → −0.9), one more sign that the box does not determine them.
- Approach and return (segments 0, 2) move J4 in one direction while the pose (and the gravity error) changes
  monotonically, so their sign terms absorb pose-dependent gravity (J4 approach: implied −0.16..+0.10 Nm, return
  0.36-0.54 Nm; physically one joint). Trust segment 1, where J4 reverses many times. Consequence for the what-if: the
  approach improves (5.4-7.6 → 3.6-5.9 mrad) but the return gets slightly worse (3.0-4.8 → 3.4-5.6) with 0.18 Nm:
  the return's small error is partly a gravity error cancelling the friction over-compensation.
- The box (|v_ref| ≤ 0.07 rad/s, tanh saturated almost everywhere) cannot separate the sign and viscous terms well
  (viscous −2.2 Nm·s/rad, R² 0.78), which is why it is the outlier in the implied Coulomb.
- Coherence with 3-8 segments is biased upward (a zero true coherence reads up to the "95 % zero" column); the
  figure-8 x1 (3 segments) 0.91 is barely above its 0.78 threshold, the figure-8 x2 0.22 is below its 0.35.
- The what-if is quasi-static (a torque deficit d gives e = d / kp; no dynamics). Checked on the fake plant
  (`fakebus` J4: Coulomb 0.3 Nm, viscous 0.5, inertia 0.07) with the j4_tune rehearsal: the fit recovered the plant's
  Coulomb (implied 0.31, up/down 0.33/0.29) from the default rehearsal and again (0.30) from a rehearsal with
  `--friction j4=0.3`; the predicted sign term with 0.3 Nm was +0.3 mrad, the simulated +0.0. The predicted rms was
  **conservative**: 3.5 → simulated 1.7 mrad at the heart pose, 5.8 → 4.0 at the crack pose (the fake plant's fit has
  R² 0.41: part of the over-compensation shows up as dynamics the linear term does not hold). On the real arm R² is
  0.9, so most of it is in the linear term.
