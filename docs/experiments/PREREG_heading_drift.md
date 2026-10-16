# Pre-registration: degraded heading (yaw_drift 0 -> 3 deg/hr)

Registered 2026-10-16, before the run.

## Why

On the only post-fix run, with SLAM tracking essentially perfectly
(0.31 m residual, scale 0.955, one fixed rotation) and before the loop
closure, fusing SLAM made the filter **4.3x worse** than not fusing it:

| over 65 m | DR only | with SLAM |
|---|---|---|
| RMSE | **0.10 m** | 0.43 m |

That is arithmetic, not a defect. SLAM's absolute accuracy after
alignment is 0.3-0.8 m; dead reckoning is at 0.10 m. A 0.5 m measurement
cannot improve a 0.1 m estimate.

Dead reckoning is that good because **heading is free**. Yaw error is
0.063 deg mean with no growth, in every run ever recorded here. The
IMU's `<noise>` tag set `angle` (zero-mean white, which the EKF averages
down) and omitted `yaw_drift`, which defaulted to 0. The one term that
destroys real dead reckoning was absent from the simulation.

## The change

`yaw_drift = 1.4544e-05` rad/s = **3 deg/hr** on the IMU in
`scenarios/bluerov2_imudrift.scn`, reached via
`scenarios/bluerov2_turbine_imudrift.scn`. Nothing else differs (the
only diff against `bluerov2_turbine.scn` is the include line).

**The value is deliberately conservative.** A BlueROV2 Navigator-class
MEMS IMU has 5-15 deg/hr in-run bias instability, and underwater beside
a steel jacket the magnetometer cannot aid heading, so yaw is
free-running gyro integration. 3 deg/hr is *better* than this vehicle
would achieve, which biases the test **against** SLAM. A SLAM win at
3 deg/hr implies a win on real hardware; a SLAM loss at 3 deg/hr is the
weaker conclusion, because real heading is worse than this.

Over the ~1130 s mission this accumulates ~0.94 deg.

**Limitation:** Stonefish applies a deterministic ramp, not a random
walk (`IMU.cpp:65`, `accumulatedYawDrift += yawDriftRate * dt`). Real
drift is stochastic and sign-varying. A ramp is right to first order and
is what the simulator offers, but it makes the error reproducible rather
than representative, and one run cannot average over drift sign.

## The comparison stays clean

Degrading heading also degrades the **controller**, which steers on
`yaw_ekf`, so the flown trajectory will differ from earlier runs and
cross-run comparisons against the 2026-10-14/15 bags are confounded.

The comparison that matters is **within-run** and is unaffected: the
SLAM-aided filter (`/bluerov2/odometry/filtered`) and the SLAM-free
reference (`/bluerov2/odometry/dr`) consume the same degraded IMU, fly
the same trajectory, and differ only in whether SLAM is fused.

## Predictions, stated before the run

**SLAM is load-bearing if**, over the whole run:

1. dead reckoning degrades: SLAM-free reference RMSE **> 0.5 m**
   (it is 0.10 m with drift off)
2. the SLAM-aided filter beats it: **RMSE ratio < 1.0**

**SLAM is not load-bearing if** the ratio stays **> 1.0**.

**Void if** criterion 1 fails -- no drift, no test. Void also if a
loop-closure jump of the kind at t=572 s on
`a50_slamned_seed0_20261016_160033` lands mid-run, which would make the
result about that event rather than about heading; report the split
either side if so.

ATE against ground truth is recorded but is **not** the criterion. The
SLAM-vs-DR ratio is.

## Run sequence

    cd ~/ros2_ws && colcon build --event-handlers console_direct+ \
        --packages-select stonefish_bluerov2

    ros2 launch stonefish_bluerov2 bluerov2_sim.py \
        scenario:=bluerov2_turbine_imudrift dvl_error:=a50 dvl_seed:=0 \
        dr_reference:=true

    bash run_stereo_slam.sh
    python3 src/stonefish_bluerov2/scripts/bluerov2_autonomous_controller.py
    bash record_run.sh slamdrift 0

Sanity check afterwards: the analyzer's `yaw err` must no longer read
0.063 deg. If it does, the drift never reached the filter and the run is
void.

---

# Outcome, recorded 2026-10-16

Bag: `bags/a50_slamdrift_seed0_20261016_190745` (1155 s, 131.2 m).
Tool: `eval/filter_ab.py`.

## The manipulation worked

The sanity check passes. Fitting the signed yaw error against time:

| | ramp | intercept | resid sd |
|---|---|---|---|
| IMU yaw | **+3.00 deg/hr** | +0.03 deg | 0.099 deg |
| SLAM-free filter yaw | **+3.01 deg/hr** | +0.03 deg | 0.083 deg |

Exactly the commanded rate, and the filter follows it with nothing to
pull it back. Mean yaw error went 0.063 -> 0.128 deg, end-of-run
+1.09 deg. The term that was missing is now present and reaches the
estimator.

## Criterion 1 FAILS -- the run is VOID

| over 118.6 m, after the first SLAM pose | RMSE |
|---|---|
| SLAM-free reference | **0.22 m** |
| SLAM-aided | 1.07 m |

Dead reckoning had to degrade past 0.5 m for the test to mean anything.
It reached 0.22 m. The run cannot answer the question it was run to
answer.

Not void on the second ground: no position-error step above 0.5 m in
the fused filter, so no loop closure of the t=572 s kind landed in it.

## Why 3 deg/hr was not enough, and why that is the finding

A heading bias displaces an estimate in proportion to its **distance
from where the heading was last known**, not to path length. This
mission is a closed orbit:

    path length                131.2 m
    max distance from start     37.9 m
    distance from start, final   3.8 m

0.96 deg over 131 m of *straight line* would be 2.20 m. Over this path
it is 0.96 deg x 37.9 m ~ 0.6 m at worst and 0.22 m RMSE, because the
lever arm never grows and the vehicle comes home. **Turbine inspection
geometry suppresses heading drift in position error by roughly 3.5x
relative to the same distance flown outbound.** That is a property of
the mission, not of the IMU, and it has to be stated wherever the DR
number is quoted.

## What the run does establish

Recovering the bridge's fitted rotation from the bag itself --
`aligned_i - aligned_j = R(theta)(raw_i - raw_j)`, so translation
cancels -- gives theta over 4363 displacement pairs. Its p5..p95 is
-1.2 .. +9.9 deg, against -1.2 .. +10.3 deg in the bridge's own console
log: an independent confirmation that the recovery is right.

    theta sd about its trend        3.83 deg
    theta secular ramp            +10.1 deg/hr
    vehicle heading drift          +3.0 deg/hr
    median distance from origin     25.9 m
    3.83 deg at 25.9 m              1.73 m

So the alignment rotation wanders 3.8 deg, which at the working radius
injects ~1.7 m -- the same order as the SLAM-aided RMSE of 1.07 m.
**The 5x penalty for fusing SLAM is the alignment wander, not SLAM's
own tracking.** The wander is not the vehicle's heading drift either:
it is 3.4x larger than it and oscillates where the drift is monotone.

Conditioning explains part of it. Binning by the trailing-50 s
horizontal spread of the fit window:

| spread | mean &#124;theta - trend&#124; |
|---|---|
| 0-5 m | 3.12 deg |
| 5-10 m | 2.56 deg |
| 10-15 m | 1.66 deg |

Monotone, r = -0.21. A vertical scan collapses the horizontal spread
and the rotation goes weakly determined -- but 1.66 deg remains at the
best conditioning, so conditioning is a contributor, not the cause.

## Split at the Atlas fragmentation (t = 1037 s, during RISE)

| window | SLAM-free | SLAM-aided | ratio |
|---|---|---|---|
| inspection, 220-1037 s | 0.17 m | 0.45 m | **2.63** |
| RISE tail, 1037-1155 s | 0.41 m | 2.78 m | 6.84 |
| whole | 0.22 m | 1.07 m | 4.99 |

The headline 5x is inflated by the tail, where the map collapsed
403918 -> 236 points and the bridge correctly stopped accepting poses
but the last bad alignment was already in the filter. **Over the
inspection proper the penalty is 2.6x.** Quote 2.6x, not 5x.

## Next

Raising the rate is the obvious move and is still defensible: 10-15
deg/hr is the middle of the Navigator-class band, not the pessimistic
end, so it needs no special pleading. Over 1155 s that is 3.2-4.8 deg,
and against the 37.9 m lever arm ~2.1-3.2 m peak, which clears 0.5 m
RMSE with margin.

But the ratio is 2.6 against SLAM **now**, and degrading heading moves
both filters: the SLAM-aided one inherits every bit of the drift too,
through the same IMU, and its alignment is fitted against the drifting
reference. Raising the rate raises the numerator as well as the
denominator. It is not obvious the ratio crosses 1.0 at any rate, and
that should be stated as the risk before the next run rather than
discovered after it. Fixing the 3.8 deg alignment wander is the change
with the clear mechanism behind it.
