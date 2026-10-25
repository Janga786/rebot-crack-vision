# Powered show runs on the B601-DM — 2026-10-23 and 2026-10-25

Seven runs of the show trajectories from `tools/make_fancy_trajectory.py` (and the box from
`tools/make_box_trajectory.py`) on the powered arm, each on the operator's go (a hold at the fold, then a second go),
through `~/rebot_setup/demo_move.py` behind arm_guard. **All seven completed, with no guard fault, and every motor was
switched off and confirmed disabled at the end.** Each run has its summary (`runs/*.json`) and its 200 Hz log
(`runs/*.csv.gz`). `analysis.txt` is `tools/analyze_run_gains.py` over all of them.

| Run | Shape | Speed | J2 / J3 gains (kp/kd) | `--ff-bias` | Worst |ref-pos| J1 / J2 / J3 / J4 (rad) | Peak J3 torque |
|---|---|---|---|---|---|---|
| `figure8_20261023T204705Z` | figure-8 | x2 slower | 100/4, 80/4 | – | 0.009 / 0.034 / 0.023 / 0.020 | 10.4 Nm |
| `figure8_20261023T204854Z` | figure-8 | x1 | 100/4, 80/4 | – | 0.007 / 0.029 / 0.020 / 0.020 | 10.3 Nm |
| `heart_20261023T205033Z` | heart | x2 | 100/4, 80/4 | – | 0.006 / 0.030 / 0.025 / 0.019 | 10.5 Nm |
| `heart_20261023T205318Z` | heart | x2 | 100/5, 80/5 | – | 0.009 / 0.030 / 0.025 / 0.020 | 10.3 Nm |
| `showreel_20261025T213800Z` | figure-8, star, heart | x2 | 100/4, 80/4 | – | 0.006 / 0.032 / 0.026 / 0.022 | 10.9 Nm |
| `heart_20261025T214843Z` | heart | x2 | 100/4, 80/4 | j2 +1.4, j3 -0.6, j4 -0.2 Nm | 0.009 / 0.025 / 0.022 / 0.017 | 10.5 Nm |
| `box_20261025T215102Z` | box 15 cm | x2 | 100/4, 80/4 | j2 +1.4, j3 -0.6, j4 -0.2 Nm | 0.010 / 0.024 / 0.026 / 0.016 | 10.2 Nm |

The worst loop gap in every run was 13.9–16.0 ms (200 Hz stream held). The abort limit for tracking is 0.1 rad.

## What the logs say about the gains (the operator's "something is a little off")

1. **The ripple the encoders see is small, and kd does reduce it.** Band power of the tracking error at 2.5-40 Hz
   (`analysis.txt`; FFT band rms): J2 0.6-0.95 mrad, J3 0.6-1.1 mrad, J4 0.7-1.6 mrad, all concentrated at 2.5-3.3 Hz,
   the roughly 3 Hz J2/J3 mode of 2026-10-22. At 0.5 m that is about half a millimetre per mrad. On the same heart,
   kd 5 against kd 4 cut J2 0.95 -> 0.70 mrad and J3 1.07 -> 0.80 (about 25 %); on 2026-10-22 kd 4 against kd 1 cut J2
   2.25 -> 0.54 and J3 1.62 -> 0.80. **J4, still at kp 40 / kd 1, has the largest ripple of the three** (1.1-1.6 mrad
   in the figure-8, heart and showreel).
   *Correction:* a first pass of this analysis used a 0.1 s moving-average high-pass, which attenuates a 3 Hz line
   about 6x; it understated the ripple about 3x and wrongly concluded that kd 5 made no difference. The tool now uses
   an FFT band measure (tested: a sinusoid reads A/sqrt(2) anywhere in the band). What the encoders cannot see (link
   flex, backlash, sway of the base) is still open: the operator's report of an oscillation is what to chase.
2. **There is a steady, pose-dependent sag, and it is a feed-forward problem.** Without bias, the P term had to supply
   a constant extra torque while the arm moved: J2 +1.0 Nm (mean error +10.6 mrad), J3 -0.85 Nm (-10.0 mrad), J4
   -0.24 Nm (-5.4 mrad). Splitting the near-stationary samples by the direction of the preceding move (`--directions`)
   separates bias from friction: J2 needs about +1.4 Nm in both directions (+1.64 after moving up, +1.20 down: a real
   gravity-model bias), J3 about -0.9 Nm plus about ±0.6 Nm of friction (-0.3 up, -1.5 down), J4 about -0.2 Nm.
   The tuned gravity scales (J2 0.6, J3 2.5, J4 2.5) were fitted on small hold moves; the real arm is heavier distally
   than the URDF (camera and mount).
3. **A constant bias helps but is crude.** `--ff-bias j2=1.4,j3=-0.6,j4=-0.2` (J3 held back to -0.6: its feed-forward
   peaks at -8.98 Nm in the unbiased runs, and the cap is 10 Nm) halved the offsets on the same heart: J2 mean error
   +10.6 -> -2.8 mrad, J3 -10.0 -> -5.3, J4 -5.4 -> -2.4; peak J3 feed-forward 9.5 Nm. Rms and peak error barely moved
   (J2 9.7 -> 8.6 mrad rms, 30 -> 25 mrad peak): the approach and return from the fold dominate (J2/J3 rms 7-10 mrad,
   mean -7 to -11 mrad there, against 3.7-4.1 mrad inside the heart itself), because the gravity error depends on
   pose and a constant is wrong where the arm is near the fold.
4. **J4 has a velocity-dependent error inside the path** (correlation with the commanded speed -0.89, rms 7.4 mrad
   in the heart) and the largest 3 Hz ripple: a gain question (kd 1 now; the guard allows kd up to 5), not gravity.

## Next (workstation-only first)

- A pose-dependent gravity correction per joint, fitted on the stationary samples of all these runs, then an A/B on the
  heart (operator's go for the arm).
- J4: try kd 3-4 (and kp), the joint nobody has damped yet; kd 5 on J2/J3 is also worth adopting: it cut their ripple
  about 25 % with no other change.
- The bias is a per-run option only (`demo_move.py --ff-bias`); `tuned_gains.yaml` and the defaults are unchanged.
