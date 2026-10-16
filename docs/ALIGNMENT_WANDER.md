# The alignment wander: what it is, and what was actually wrong

2026-10-16. Follows the degraded-heading run, which measured the bridge's
fitted rotation wandering 3.83 deg sd about its trend and attributed the
5x SLAM penalty to it. That attribution was **wrong**, and this is the
correction.

All numbers are offline replays over `/bluerov2/robot_pose_slam`
(converted ENU->NED on read), fitting against `/bluerov2/odometry/dr`
exactly as the bridge does, and scored against `/bluerov2/odometry`.
Ground truth is used only to score. Four bags.

## The wander is not a defect

Sweeping the trailing-window width, with scale fitted, scoring the
aligned pose against truth from t=220 s:

| bag | as flown | 75 s | 150 s | 300 s | 600 s | all history |
|---|---|---|---|---|---|---|
| slamdrift | 0.40 | **0.63** / sd 5.0 | 0.79 / 4.8 | 0.95 / 1.7 | 1.19 / 1.0 | 1.31 / 0.8 |
| slamned | 2.32 | **3.48** / 79.6 | 6.04 / 63.6 | 8.96 / 47.4 | 11.55 / 14.4 | 11.81 / 7.2 |
| slamb30 | 2.83 | **1.08** / 8.9 | 1.23 / 2.2 | 1.18 / 1.5 | 1.21 / 1.3 | 1.24 / 1.2 |
| slamref12 | 2.84 | **0.82** / 5.1 | 0.87 / 1.5 | 0.84 / 0.7 | 0.84 / 0.5 | 0.84 / 0.5 |

(m RMSE / sd of theta in deg. slamb30 and slamref12 predate the ENU->NED
fix, so their *as flown* column is not comparable; the replays are.)

**Longer windows reduce the wander and increase the error, monotonically,
on every bag.** Theta's scatter and the pose's accuracy move in opposite
directions. The map frame genuinely moves -- ORB-SLAM3 re-optimises it --
and a transform that refuses to follow is just wrong for longer. The
sliding window, the EMA and the refit cadence were all left alone.

A single transform locked once and held is worse still: 0.95-1.12 m on
slamdrift against 0.40 m as flown, and 14-18 m on slamned.

So "fix the wander" had no fix in it. Two other things did.

## Defect 1: scale was assumed to be 1.0

The fit was rotation-only, documented as "stereo SLAM's metric scale is
already correct from the calibrated baseline". The baseline **is**
correct -- `eval/check_stereo_baseline.py` re-derives it from the .scn
and it matches the SLAM yaml to four figures, and fx = (640/2)/tan(75/2)
= 417.03 matches `Camera1.fx` exactly. The *realised* scale is not:

| bag | global k | per-window k, median |
|---|---|---|
| slamdrift | 0.967 | 0.956 |
| slamned | 0.946 | 0.578 |
| slamb30 | 0.953 | 0.967 |
| slamref12 | 0.928 | 0.942 |

ORB-SLAM3's own scale bias on a far-dominated scene -- mThDepth = 5.8 m
with 13% of returns inside it, so most points carry scale-free
constraints. On the clean bag, fitting scale halves the global residual
(0.67 -> 0.34 m). At the bridge's own window and damping it is worth
5-16% on three of four bags and -2% on the fourth.

## Defect 2: the published covariance was a constant, and a lie

The bridge declared `0.05 m^2` (sigma = 0.22 m) on every pose it ever
published. Measured per-axis sigma of the pose it actually published:

| bag | true sigma | declared | overconfident by | window residual |
|---|---|---|---|---|
| slamdrift | 0.28 m | 0.22 m | 2x in variance | 0.18 m |
| slamned | 1.64 m | 0.22 m | **54x in variance** | 1.94 m |

This is the arithmetic behind SLAM making the filter worse. An EKF told
that a 1.64 m measurement is good to 22 cm will believe it over dead
reckoning that is genuinely good to 17 cm. The 2.6x penalty over the
inspection needs no explanation beyond this.

The fit's own residual is a usable, causal, truth-free stand-in: 0.18 m
where truth said 0.28, 1.94 m where truth said 1.64. It tracks the order
of magnitude and flags the bad run as bad.

## What changed in `slam_pose_bridge.py`

1. `_try_solve` fits Sim(2) instead of SO(2) -- scale alongside rotation,
   EMA-damped on the same cadence, and bounded to [0.75, 1.25] because
   the initial lock has no previous scale to damp against.
2. The adopted transform's per-axis RMS residual is computed and kept.
3. `_publish_var()` derives the published x/y variance from it, inflated
   1.5x (the residual runs below the true error) and **floored at the old
   0.05**. The floor means this can only loosen the old figure, never
   tighten it: SLAM's pull on the filter can fall but not rise, so no
   previously acceptable run can be made to diverge by this change.
4. Yaw variance is untouched at 0.05. The residual is a position
   residual and says nothing about heading; deriving one from the other
   would be invented.

The Sim(2) maths was verified against a synthesised transform
(theta 7.300 deg, k 0.9620, t (-1.400, 0.800), noise sd 0.050) and
recovered it to 0.001 deg, 0.0001 in k, 0.004 m and 0.0002 m.

## Expected effect, stated before the next run

On a slamned-class run the scale guard rejects 56% of windows (median
per-window k is 0.578 there), so the bridge will often decline to lock
rather than publish a confidently wrong pose. That is the intent, but it
means **SLAM will contribute less, not more.**

And this does not make SLAM load-bearing. The best alignment found by
any offline method puts the SLAM pose at 0.31-0.63 m against truth over
the clean run, while dead reckoning is at 0.17 m. A measurement that is
worse than the estimate cannot improve it; an honest covariance means
the filter will now correctly mostly ignore it. Expect the SLAM-aided
RMSE to fall toward the SLAM-free one and the ratio to approach 1.0
**from above** -- not to cross it.

Crossing 1.0 needs dead reckoning to get worse (higher yaw drift, per
`PREREG_heading_drift.md`) or the SLAM pose to get better (the loop
closure that matched the wrong face, `LOOP_CLOSURE_FACE_CONFUSION.md`).
This change does neither. It stops the bridge from actively damaging
the filter, which is a precondition for either, not a substitute.
