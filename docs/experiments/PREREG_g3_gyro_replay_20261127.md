# Pre-registration: does SLAM help when heading comes from a gyro? (G3, offline replay). Written 2026-11-27, before any corrupted replay was run

## Why

The veto/symmetry flights (`PREREG_veto_symmetry_20261116.md`) found the fused estimate worse than
dead reckoning (DR) in 24 of 24 missions at mission end, and in 23 of 24 on median error. Measured
since, on `a50_stkL0_seed7_20261127_010854`:
- the simulated IMU's yaw is truth plus 0.1 deg white noise, with a trend of -0.001 deg/h over
  1276 s. Both filters fuse it as absolute yaw: a perfect compass;
- the DVL already carries datasheet A50 errors (1.01 % scale, 0.5 deg misalignment per unit).

A vehicle working next to a steel jacket cannot trust a magnetometer. Its heading is integrated gyro, which drifts
by grade. This replay asks the G3 question under that condition.

## Question

With the IMU heading replaced by an integrated gyro of a stated grade, is the fused estimate
(SLAM through the bridge) more accurate than DR?

## Method

`eval/replay_nav.py` (fidelity checked before this was written, uncorrupted, on
`a50_stkL0_seed7_20261127_010854`: replayed against recorded DR 1.8 mm median and 4.2 mm max; bridge 2.6 cm
median, with the same 11 064 poses; fused 2.3 cm median and 0.32 m max; error against truth reproduced: DR median 0.264 m
vs 0.264 m, fused median 0.433 m vs 0.424 m). Three passes, in the order the system computes them:
the real `ekf_dr` node, the real bridge class against that DR, then the real fused `ekf` node.

Gyro model, per flight: yaw' = yaw_imu + s (yaw_imu - yaw_imu(t0)) + b t + W(t); wz' = wz (1 + s) + b.
Roll and pitch are unchanged. b ~ N(0, sigma_b) and s ~ N(0, sigma_s) are drawn from the flight's DVL seed, and W is
angle random walk. Grades are from `eval/match_gap.py` GRADES, unchanged. The draws, fixed by seed and
listed here before any replay:

| seed | MEMS, in-water cal. (PRIMARY): bias deg/h, scale | tactical MEMS: bias deg/h | MEMS, on-deck cal.: bias deg/h |
|---|---|---|---|
| 0 | +4.5, -0.0013 | +0.1 | +45.3 |
| 1 | +12.4, +0.0082 | +0.3 | +124.4 |
| 2 | +6.8, -0.0052 | +0.2 | +68.1 |
| 29 | -14.1, +0.0005 | -0.3 | -141.1 |
| 3 | +73.5, -0.0256 | +1.6 | +734.7 |
| 26 | -69.3, -0.0302 | -1.5 | -693.0 |
| 5 | -28.9, -0.0132 | -0.6 | -288.7 |
| 7 | +0.0, +0.0030 | +0.0 | +0.4 |

**Flights.** The 24 valid veto/symmetry flights (8 seeds by 3 arms). All arms are used. The arm changes the SLAM
track, not the navigation question.

**Grades.**
- PRIMARY: "MEMS, bias cal. in water (dT 1 C)" (sigma_b 36 deg/h). This is a BlueROV2-class
  consumer IMU (ICM-20602) with its bias calibrated in water.
- Secondary: "tactical MEMS (ADIS16495-class)" (0.8 deg/h). Where SLAM should stop mattering.
- Secondary: "MEMS, bias cal. on deck (dT 10 C)" (360 deg/h). The stock BlueROV2 behaviour: ArduSub
  calibrates bias at boot.

## Endpoints

Per flight and grade: horizontal error against truth of DR' and fused'. Primary is the median from
the truth start to MISSION COMPLETE (as `eval/prereg_score.py` derives it). Secondary is the error at
MISSION COMPLETE.

**Primary test**, PRIMARY grade only: one-sided Wilcoxon signed-rank on DR' median minus fused' median across the 24
flights, alpha 0.05.

**"SLAM helps" is claimed only if** p < 0.05 AND fused' is better in more than half of the flights.
Otherwise the result is reported as "not shown" with the numbers. The secondary grades are descriptive, with the same
statistics reported.

Scored by `eval/g3_replay.py score`, committed with this file. Missing replays are listed, never
imputed. A replay that fails, for example a filter that does not start, is re-run once, and the fact is reported.

## Limits, stated before the result

- **Open loop.** The vehicle flies the recorded path. A degrading estimate would, in flight, steer
  the vehicle somewhere new; that is not modelled. A confirmatory flight is only considered if
  the replay shows an effect.
- **ORB-SLAM3 is not re-run.** Its heading-check decisions are those made in flight with the perfect
  compass, so the veto under a drifting gyro is not tested here.
- **The bridge aligns SLAM against DR'.** Its alignment therefore inherits the DR' heading error. This is the
  system as designed, not a choice of this replay.

## Results log

(append only)
- 2026-11-27, IMPLEMENTATION, before any corrupted replay finished: playback raised from 4x to 16x (`REPLAY_RATE=16`), 6 parallel workers. Fidelity re-checked at 16x on `a50_stkL0_seed7_20261127_010854` uncorrupted: DR' vs recorded 1.8 mm median / 4.5 mm max, fused' 2.3 cm / 0.32 m, error against truth fused' median 0.435 m vs recorded 0.424 m, identical to 4x; one 0.3 ms overrun of the DR filter's 50 Hz tick, 0.3 % fewer DR outputs. A first 2-worker run at 4x was stopped after its first two replays started and before either finished; nothing from it is used. Each replay's filter warnings are kept in its summary.json and reported.
