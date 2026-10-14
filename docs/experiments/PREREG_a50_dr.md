# Pre-registered predictions: dead reckoning with a datasheet DVL

Written **before** the runs were recorded, so that agreement is a test of the
error model rather than a fit to it. Committed ahead of the data; check the git
timestamp on this file against the bag timestamps.

## Setup

Three dead-reckoning runs, no SLAM, no ground-truth anchor:

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine dvl_error:=a50 dvl_seed:=N
bash record_run.sh dr N
```

for `N` in 0, 1, 2. `dvl_error:=a50` applies `scale_sigma: 0.0101` (the A50's
quoted +/-1.01% long-term velocity accuracy) and `misalign_sigma_deg: 0.5`.
Baseline for comparison is `deadreckon_20261009_192352`: white noise only,
0.040 m final error, 0.03% of 132.8 m travelled.

## Predictions

Propagated open-loop through the *recorded* trajectory of the baseline run
(130.9 m of XY path), using the same seeded draws the node makes. The seed 0
draw was checked against the live node and matches to five decimals.

| seed | realised scale (x,y,z) | misalignment (deg) | final XY err | peak err | % of distance | peak bearing err @ 16.8 m |
|---|---|---|---|---|---|---|
| 0 | 1.00127, 0.99867, 1.00647 | +0.052, -0.268, +0.181 | 0.102 m | 0.143 m | 0.078% | 0.49 deg |
| 1 | 1.00349, 1.00830, 1.00334 | -0.652, +0.453, +0.223 | 0.312 m | 0.408 m | 0.238% | 1.39 deg |
| 2 | 1.00191, 0.99472, 0.99583 | -1.221, +0.900, +0.572 | 0.535 m | 0.690 m | 0.409% | 2.35 deg |

## What counts as agreement

These are **open-loop** propagations. The runs are closed-loop: a corrupted
estimate makes the controller fly a different path, which changes how the error
accumulates. So the prediction is "what this unit does on *that* path", not an
exact forecast. Treat as confirmation:

1. all three land in 0.05 - 1.0 m, i.e. clearly above the 0.040 m white-noise
   baseline and clearly below a metre;
2. the ordering holds, seed 0 < seed 1 < seed 2;
3. the per-axis scale factor recovered from `/bluerov2/dvl_twist` against
   ground-truth body velocity matches the realised scale in the table above.

Point 3 is the strongest check, because it is measured in the loop and does not
depend on the trajectory at all.

## What would falsify the reasoning

- **Errors far larger than predicted** (metres): the cancellation argument is
  wrong, or the controller's response to a biased estimate amplifies rather than
  averages the error. Either is a more interesting result than the prediction.
- **Errors indistinguishable from the 0.040 m baseline**: the error model is not
  reaching the filter. Check the `DVL systematic error ENABLED` warning in the
  launch log and the recovered scale factor before believing it.
- **Any run failing to complete the four-face inspection**: predicted bearing
  error peaks at 2.35 deg against a 12 deg gate, so all three should finish. A
  failure means something other than DVL error is driving it.

## Why this arm matters

It is the control for the SLAM arm. `anchorfree_run1_20261009_185821` reached
20.7 m position error and 62.7 deg bearing error with SLAM in the loop. If the
worst datasheet DVL unit stays under a metre, then the dominant failure mode in
GPS-denied inspection of a symmetric structure is not sensor drift but discrete
misassociation, and the paper is about that.

---

## Run log

Appended after the fact; the predictions above are unchanged from the commit
that registered them.

**2026-10-09, `a50_dr_seed0_20261009_210804` -- VOID, not an a50 run.** The
mission completed (four faces, home at 2.2 m, 1162 s, 131.9 m), but the scale
factor recovered from `/bluerov2/dvl_twist` came back at 0.99994 / 1.00006 /
0.99989, within 1.3 sigma of unity, where seed 0 should read 1.00127 / 0.99867 /
1.00647. This is falsification criterion 2: the error model never reached the
filter.

Cause was the launch file, not the run. `parameters` was built with a
`PythonExpression`, which returns a *string*, so the node was handed the literal
text `"['/.../dvl_a50.yaml']"` as a filename, found nothing, and kept its zero
defaults. Fixed by splitting into two nodes under `IfCondition` /
`UnlessCondition`, which cannot fail silently this way.

The bag is still useful: it is an independent **replicate of the white-noise
baseline**, and the two agree -- 0.01% vs 0.03% drift, ATE 0.03 m vs 0.02 m,
max bearing error 0.51 deg vs 0.47 deg over essentially the same distance.
That is the first replication of any result in this project.

This is also the case for keeping the gain check in the analyzer permanently.
Nothing else in the run looked wrong: it completed, the trajectory was clean,
and the numbers were plausible. Only a measurement that depends on the thing
being varied caught it.

**2026-10-09, `a50_dr_seed0_20261009_221008` -- VALID, seed 0.** Launch fix in
place; the error model reached the filter at 73 sigma. Mission completed, four
faces, home at 2.3 m, 1177 s, 132.5 m.

Realised unit, recovered from `/bluerov2/dvl_twist` against ground-truth body
velocity, against the registered draw:

| axis | registered | recovered | |
|---|---|---|---|
| x | 1.00127 | 1.00000 +/- 0.00055 | 2.3 sigma low |
| y | 0.99867 | 0.99816 +/- 0.00015 | agrees |
| z | 1.00647 | 1.00675 +/- 0.00009 | agrees |

The x axis sits a little low because the single-axis regression cannot separate
the gain from the misalignment's cross-axis coupling: during the orbit phases
surge is small and sway dominates, so part of v_y leaks into the v_x channel.
Its residual is correspondingly the largest of the three (mean|err| 0.0017
against 0.0012 m/s, correlation 0.999 against 1.000). The two axes that are not
confounded this way match to within a twentieth of a percent.

Drift against the prediction:

| | predicted | measured |
|---|---|---|
| final XY error | 0.102 m | 0.11 m |
| peak error | 0.143 m | 0.16 m |
| drift, % of distance | 0.078% | 0.08% |

All three inside 10%, from an open-loop propagation through a *different* run's
trajectory. Criterion 1 is met; criterion 3 is met on the two unconfounded axes;
criterion 2 needs seeds 1 and 2.

The mechanism is also visible rather than merely inferred. Panel (b) of the
figure is a smooth structured curve -- error builds to 0.16 m by 40 m, unwinds
to near zero by 100 m, then builds again -- which is the body-fixed residual
being rotated through the circumnavigation and cancelling. The white-noise bags
show a jagged random walk in the same panel. Bearing error carries a matching
hump to 0.74 deg while yaw stays flat at 0.28 deg, confirming that scale error
moves position, not heading.

Against the two white-noise replicates: ATE RMSE 0.10 m against 0.02 and 0.03 m,
drift 0.08% against 0.03% and 0.01%. Roughly 4x, and still far inside the
controller's 12 deg bearing gate, which is why the mission completed exactly as
predicted.

**2026-10-12, `a50_dr_seed1_20261012_173121` -- VALID, seed 1.** Mission
completed, four faces, home at 2.3 m, 1183 s, 130.4 m.

| | predicted | measured |
|---|---|---|
| final XY error | 0.312 m | 0.34 m |
| peak error | 0.408 m | 0.39 m |
| drift, % of distance | 0.238% | 0.26% |
| peak bearing error | 1.39 deg | 1.17 deg |

The unit check is now exact rather than approximate. The per-axis gains read
+0.784 / +1.300 / +0.116%, against a registered scale of +0.349 / +0.830 /
+0.334%, which looks like a 0.45% disagreement. It is not: a single-axis
regression cannot separate the gain from the misalignment's cross-axis
coupling, and seed 1's misalignment is an order of magnitude larger than seed
0's on the roll axis (-0.652 deg against +0.052 deg), so more of the other
axes leaks in.

Recovering the whole 3x3 operator by least squares separates them.
`eval/verify_dvl_model.py` does this, and every element of both seeds' matrices
matches prediction to within 0.0008 -- eight parts in ten thousand, across all
nine entries, gains and coupling alike. That is a complete end-to-end check of
the error model, and it confirms both bags carry the unit their seed specifies.

## Status against the registered criteria

| seed | predicted | measured | unit verified |
|---|---|---|---|
| 0 | 0.078% | 0.08% | yes, to 0.0008 |
| 1 | 0.238% | 0.26% | yes, to 0.0008 |
| 2 | 0.409% | pending | |

Criterion 1 (0.05 - 1.0 m) holds for both. Criterion 3 holds exactly, once
measured with the full matrix rather than per-axis. Criterion 2, the ordering,
holds so far -- 0.08% then 0.26%, a factor of 3.3 against a predicted 3.1 --
and seed 2 completes it.

Both missions completed well inside the 12 deg bearing gate, as registered.

**2026-10-12, `a50_dr_seed1_20261012_191733` -- VALID, seed 2 (filename is
wrong).** Mission completed, four faces, home at 2.1 m, 1125 s, 132.6 m.

`record_run.sh` was given `1` on a run launched with `dvl_seed:=2`, so the bag
carries a name that contradicts its contents. The two are independent: the name
comes from the recording script, the unit from the launch argument. The 3x3
check settles it without ambiguity -- the bag matches seed 2 to 0.00049 and
mismatches seed 0 and seed 1 by 0.022 and 0.014, a factor of thirty. This is
the second time a check has caught a bookkeeping error that nothing else would
have.

The bag has since been renamed to `a50_dr_seed2_20261012_191733`, re-verified
against seed 2 after the rename, and its figure regenerated under the correct
name. Note that two figures carry a `seed0` name: `...210804` is the VOID
white-noise run logged above, and `...221008` is the valid one.

| | predicted | measured |
|---|---|---|
| final XY error | 0.535 m | 0.51 m |
| peak error | 0.690 m | 0.64 m |
| drift, % of distance | 0.409% | 0.37% |
| peak bearing error | 2.35 deg | 2.33 deg |

## Arm 1 complete: all three registered criteria met

| seed | predicted drift | measured drift | final error | unit verified |
|---|---|---|---|---|
| 0 | 0.078% | 0.08% | 0.11 m (pred 0.102) | 0.00079 |
| 1 | 0.238% | 0.26% | 0.34 m (pred 0.312) | 0.00080 |
| 2 | 0.409% | 0.37% | 0.51 m (pred 0.535) | 0.00049 |

1. **Magnitude.** All three land in 0.05 - 1.0 m, every one within 10% of a
   prediction propagated open-loop through a different run's trajectory.
2. **Ordering.** 0.08% < 0.26% < 0.37%, against a predicted 0.078% < 0.238% <
   0.409%. A 4.6x span across three independent closed-loop runs, predicted as
   5.2x.
3. **Unit.** Every element of all three 3x3 operators matches its seed's draw to
   better than 0.0008.

All three completed the four-face inspection and returned home, with peak
bearing error 0.47, 1.17 and 2.33 deg against a 12 deg gate -- as registered.

For comparison, the white-noise DVL gives 0.03% and 0.01% over the same mission.
Datasheet scale factor and misalignment make the drift roughly an order of
magnitude worse and leave it comfortably sub-metre, because the inspection path
closes a loop and a body-fixed residual largely cancels over 360 deg of yaw.

---

# Arm 2 (SLAM in the loop), seed 0 -- 2026-10-12

Bag `a50_dr_seed1_20261012_200040` (2449 s, 175.5 m travelled, 510 MB).

**The name is wrong twice over and the bag was kept anyway.** The 3x3 operator
fit identifies the unit as **seed 0**, to 0.00019 -- seeds 1 and 2 miss by
0.0126 and 0.0223. The arm reads `dr`, but ORB-SLAM3 was up for the whole run,
so `record_run.sh` dropped the SLAM topics. `record_run.sh` now reads
`/dvl_bridge error_seed` and the `/bluerov2/map_points` publisher count and
refuses to record a name the live system contradicts.

| | arm 1, seed 0 | arm 2, seed 0 |
|---|---|---|
| final position error | **0.11 m** | **40.75 m** |
| drift, % of distance | 0.08 % | 23.22 % |
| ATE RMSE | -- | 37.91 m |
| mean bearing error | -- | 97.2 deg (max 167.3) |
| yaw error | -- | 0.06 deg mean, 0.42 max |
| depth error | -- | 0.003 m mean, 0.015 max |
| mission outcome | four faces, returned home | `MISSION COMPLETE` |

Same unit, same controller, same mission. SLAM is the only difference, and the
error is 370x larger.

## The error did not come from the DVL

Integrating this bag's own recorded `/bluerov2/dvl_twist` through ground-truth
heading from the recorded start pose:

| | final error | % of distance |
|---|---|---|
| DVL-only dead reckoning | **0.78 m** | 0.45 % |
| the filter's own estimate | **40.75 m** | 23.22 % |

Dead reckoning on the same velocities that the filter was fed ends 0.78 m out.
Yaw error stays under 0.5 deg and depth error under 15 mm all run, so neither
the IMU nor the depth pose can carry 40 m. The horizontal error entered through
the SLAM pose.

## It is self-reinforcing

| t (bag) | event |
|---|---|
| 251 s | first SLAM alignment solved, theta = +158 deg |
| 269 s | filter is 5 m from dead reckoning |
| 301 s | controller's range-to-turbine starts climbing |
| 311 s | filter is 20 m from dead reckoning |
| 359 s | SLAM confidence -> 0.00, map frozen at 158,831 points |
| 652 s, 821 s | bridge rejects SLAM poses as 39.4 m and 55.7 m "from EKF" |

The filter departs dead reckoning 18 s after the first alignment and is 20 m
out within 60 s. The controller, believing itself at the 17 m standoff, drives
the vehicle out past 40 m -- at which point the sonar returns 999 (no echo) and
SLAM loses the structure it was tracking, so no recovery is possible. Faces 1-3
were "scanned" from roughly 45 m away, with the sonar reading 42-47 m while the
estimate insisted on 16-17 m, and the state machine reported success.

The two late rejections are worth their own line: the bridge's 5 m innovation
gate fires at 652 s and 821 s, but by then **the EKF is the thing that is
wrong**, so the gate is rejecting poses for disagreeing with a runaway estimate.
A gate referenced to the filter cannot recover a filter that has already left.
That is a constraint on arm 3's design, not a result.

## What this bag cannot settle

The SLAM topics were not recorded, so the *mechanism* inside SLAM -- whether
this is the misassociation seen before, or the alignment fit itself -- is not
decidable from it. The analyzer used to print "SLAM NEVER reached the filter"
for a bag like this; it was reading an unrecorded topic as an absent signal,
and now says so instead. The attribution above rests on exclusion (DVL, IMU and
depth each ruled out by their own residuals), not on observing the SLAM pose.

Seeds 1 and 2 of arm 2 still to run, with `record_run.sh slam <seed>`.

## The excursion, from ground truth

| | |
|---|---|
| commanded standoff | 17.0 m |
| true range to turbine | min 14.6 m, **max 57.3 m** at t = 890 s |
| believed range at that moment | 16.6 m |
| time beyond 2x standoff (34 m) | 717 s, 29 % of the run (first at t = 439 s) |

| t (s) | true r | believed r | gap |
|---|---|---|---|
| 200 | 17.3 | 17.2 | +0.1 |
| 400 | 25.7 | 36.3 | -10.6 |
| 600 | 46.6 | 26.4 | +20.3 |
| 800 | 57.2 | 16.5 | +40.7 |
| 1200 | 25.3 | 17.0 | +8.3 |

At its furthest the vehicle was 3.4x its commanded standoff from the structure
while reporting that it was holding the setpoint to within half a metre.

## The error is a fixed offset, not drift

After t = 500 s the estimate's error is a constant world-frame translation:

    magnitude  40.75 m  (sd 0.04 m)
    direction   168.0 deg  (circular sd 0.1 deg)

Four centimetres of spread over 1900 s. This is not accumulating error -- the
map was registered to the world about 40 m out, and from then on the navigation
was internally consistent and precisely wrong, dead reckoning at 0.45 % about
the wrong origin. Every residual the system can compute for itself stayed small
the whole time: yaw 0.42 deg, depth 15 mm, DVL innovation nil.

End state: the vehicle finished RETURN_HOME reporting `d=2.0 m` while sitting at
(38.41, -9.17) -- 39.5 m from home, on the far side of the turbine. Its estimate
placed it at (-1.44, -0.67).

A constant rigid offset of this kind, at a bearing near 168 deg on a structure
with four near-identical faces, is what misassociation looks like from outside
SLAM. That is a hypothesis and not a result: the SLAM topics were not recorded
here. Seeds 1 and 2 must be recorded with `record_run.sh slam <seed>` so the
pose and map-point streams are present to test it.

---

# Arm 2, seed 1 -- 2026-10-12: reproduced, and the mechanism measured

Bag `a50_slam_seed1_20261012_230457` (1463 s, 175.8 m). Unit verified as seed 1
to 0.00027. Name correct -- the first bag the `record_run.sh` guard passed.
SLAM topics present, which is what makes the rest of this section possible.

## Reproduced, and the DVL seed does not matter

| | arm 2 seed 0 | arm 2 seed 1 |
|---|---|---|
| final position error | 40.75 m | **41.04 m** |
| offset magnitude after t=500 s | 40.75 m (sd 0.04) | **40.76 m (sd 0.20)** |
| offset direction | 168.0 deg (sd 0.1) | **165.5 deg (sd 0.2)** |
| max true range from turbine | 57.3 m at t=890 s | **57.0 m at t=861 s** |
| believed range at that moment | 16.6 m | **16.5 m** |
| time beyond 2x standoff | 717 s | **715 s** |
| final true position | (38.41, -9.17) | **(38.42, -11.09)** |
| final estimated position | (-1.44, -0.67) | **(-1.31, -0.82)** |

Two different DVL units, agreeing to 0.3 m and 2.5 deg. Whatever this is, it is
not driven by the sensor noise the arm-1 experiment characterised.

Because the SLAM topics are recorded, the run splits cleanly:

| segment | distance | final error | drift |
|---|---|---|---|
| dead reckoning only, 0-220 s | 12.5 m | 0.16 m | 1.25 % |
| with SLAM, 220-1463 s | 163.2 m | **41.04 m** | 25.01 % |

The error is 0.21 m at the instant the first SLAM pose reaches the filter.

## SLAM is not inaccurate. It is in the wrong frame.

Fitting one rigid 2-D transform from ground truth to the pose the bridge
actually publishes, over 652 poses:

    rotation    +137.33 deg
    residual    mean 0.07 m   rms 0.09 m   max 0.40 m

Seven centimetres. The tracker is excellent; it is reporting the vehicle in a
frame rotated 137 deg from the world. This is a registration error, not a
tracking error, and it is what produces a 41 m offset that holds to 20 cm for
twenty minutes.

The bridge applied `theta = +152.4 deg`. Published pose = truth rotated
+137.33 deg, so the alignment it should have found is about **+15 deg**.

(The raw map-frame stream `/bluerov2/robot_pose_slam` does *not* admit a single
rigid transform -- residual 8.9 m mean, 50.8 m max -- consistent with the
bridge's own `Map point crash detected: 132170 -> 285` near the end.)

## Why it cannot recover: the alignment has no independent reference

From the code and the topic graph, not inferred:

- `slam_pose_bridge` publishes `/bluerov2/robot_pose_slam_ekf`
- `ekf.yaml` takes that as `pose0`
- the bridge subscribes `reference_odom`, defaulting to `/bluerov2/odometry/filtered`, with no launch override -- the EKF's own output
- `_try_solve` fits theta over `self.buf`, whose entries are
  `(slam_x, slam_y, ekf_x, ekf_y)` (lines 407, 414)

So the rotation taking SLAM into the world is estimated by correlating the SLAM
track against the EKF track, and the EKF track is produced by applying that
rotation. Nothing in the loop observes the world. Once SLAM and the EKF agree
in a rotated frame they keep agreeing, every refit confirms the frame it came
from, and the system sits at a self-consistent wrong fixed point.

The telemetry matches that reading exactly: theta moved 154.7 -> 152.4 deg, a
total of 2.3 deg, across twenty minutes during which the fit's baseline grew
from 5.0 m to 20.4 m. A four-fold increase in geometric information moved the
answer by two degrees. No `Refit rejected` warning appears in either run -- the
refits were not fighting the lock, they were agreeing with it.

The initial lock is taken at `MIN_BASELINE = 5.0 m`, and in both seeds it fired
at spread exactly 5.0 m, immediately after a descent that is nearly all
vertical motion -- the weakest rotation geometry in the whole mission. That
choice sets which fixed point the loop falls into; the feedback then makes it
permanent.

## Consequences for arm 3

A gate that compares SLAM to the filter cannot see this, because the two agree
by construction. Neither can any internal residual: yaw error stayed at
0.40 deg, depth at 15 mm, DVL innovation nil, and the SLAM-vs-EKF innovation
was small until the EKF was already 40 m out (the 55.6 m and 56.0 m rejections
at t=573 s and 600 s are the gate firing against a runaway estimate, not
against a bad pose).

The only signal in the stack that contradicted the estimate throughout is the
sonar range to the structure: 42-47 m measured against 16-17 m believed. Any
gate that can catch this has to be referenced to something outside the
SLAM-EKF loop. The design is still an open question and belongs to the author,
not to this log.

Seed 2 outstanding.

---

# Correction and extension, 2026-10-13 (same bag, seed 1)

The section above says "SLAM is not inaccurate. It is in the wrong frame",
on a rigid fit to the published stream with a 0.07 m residual. Two further
measurements on the same bag show that claim was too strong, and the real
picture is different in a way that matters for the fix.

## 1. The 0.07 m residual is partly self-selected

The published stream is what survives the bridge's gate, and that gate rejects
any pose more than 5 m from the EKF. Once the EKF is itself rotated, the gate
passes exactly those poses that agree with the rotated frame, so a tight rigid
fit of the *published* stream against truth is partly manufactured by the
selection. The rotation value is still real -- the EKF genuinely is ~137 deg
off truth -- but "the tracker is excellent to 7 cm" is not established by it.

## 2. SLAM's map frame rotation is not constant

Per-window rigid fits of the raw `/bluerov2/robot_pose_slam` track against
truth, 150 s windows:

| window | n | rotation | resid mean | track span |
|---|---|---|---|---|
| 0-150 s | 1900 | +9.83 deg | 0.24 m | 4.2 m |
| 150-300 s | 1899 | -102.66 deg | 2.78 m | 22.1 m |
| 300-450 s | 401 | -14.47 deg | 0.07 m | 8.6 m |
| 750-900 s | 211 | -39.90 deg | 1.09 m | 2.9 m |

Inside the quiet windows the track is locally rigid to 0.07-0.24 m, so the
tracker is locally sound. Between them the frame rotates by more than 100 deg.
The large residual in 150-300 s is the frame turning *within* the window.

A consistency check that does hold: in 300-400 s the raw fit is -14.47 deg and
the published fit is +137.87 deg, differing by 152.34 deg -- exactly the
theta = +152.4 deg the bridge reported applying. The bridge is doing what it
says; in that window the correct theta was +14.47 deg.

**This is the deeper problem.** `slam_pose_bridge` is built around a static
map->world rotation: one lock at `MIN_BASELINE = 5.0 m`, refits capped at
`REFIT_MAX_DELTA = 15 deg`, damped by `REFIT_EMA_ALPHA = 0.2`, plus a
cumulative drift cap from `theta_at_lock`. Every one of those is correct for a
constant rotation and wrong for one that moves 100 deg. Together with the
feedback loop recorded above -- theta fitted against the EKF that consumes it
-- the design cannot track the thing it is estimating and cannot detect that
it has stopped.

## 3. The attitude route is closed, but points at a concrete bug

Estimating the frame rotation from `yaw_ekf - yaw_slam` needs no baseline and
uses a reference the loop cannot move (the filter's yaw is IMU-only;
`pose0_config` has yaw false, and it held 0.40 deg against truth all run). It
does not work: attitude and the position fit disagree by 8, 61, 142, 93 and
89 deg across the windows.

But in the two windows where the position fit is tight, the disagreement is
92.54 deg and 89.03 deg. Subtracting 90 deg leaves agreement of 2.54 deg and
0.97 deg. That is the signature of a fixed ~90 deg frame-convention offset
between the orientation and position channels of `/bluerov2/robot_pose_slam`
-- the two are not expressed in the same frame. Two windows is not proof, and
the publisher is the ORB-SLAM3 container, outside this repo, so the conversion
could not be inspected here.

If that offset is real and fixed, the attitude route reopens: it would give a
baseline-free, loop-free, high-rate estimate of a rotation that is now known to
move. Worth settling before any estimator is written.

## Retraction, later on 2026-10-13: two of the claims above were fitting artefacts

**Retracted: "SLAM's map frame rotation is not constant."** The per-window
rotations came from Procrustes fits whose conditioning was never checked. The
cross-covariance singular-value ratio in those windows is 0.000-0.012 wherever
the span is meaningful: an inspection track inside a 40-150 s window is nearly
a straight line, so the residual stays at 0.15 m whatever rotation is fitted.
Adjacent windows returning -151.77 deg and -73.28 deg at 0.14 m residual was
the tell, and it should have stopped the claim rather than supporting it. The
frame may or may not rotate; these data do not say.

**Retracted: the ~90 deg orientation/position offset.** At 40 s resolution the
position-attitude gap is 20, 3, 5, 46, 102, 178, 144, 94, 87 and 89 deg. The
two windows that both read ~90 deg were a coincidence of where I happened to
cut. The ORB-SLAM3 wrapper applies the camera->base conjugation correctly
(`orb_slam3_interface.cpp:354`), so the frame-convention guess was wrong too.

**What survives, measured two independent ways.** Comparing the *direction of
travel* over displacements of 2 m or more avoids the conditioning problem
entirely: the angle between the truth displacement and the SLAM displacement
is the frame rotation, to about (noise / displacement) radians.

| t | method | frame rotation |
|---|---|---|
| 300-400 s | direction of travel, 4 pairs | **-14.42 deg (circ sd 2.0)** |
| 300-450 s | Procrustes, 8.6 m span, 0.07 m resid | -14.47 deg |
| 280-360 s | Procrustes, 40 s windows | -15.39, -9.61 deg |

So in the one interval where the geometry supports a measurement, SLAM's map
frame sits at about -14.4 deg and the bridge should have applied theta =
+14.4 deg. It applied **+152.4 deg**. That is the error, and it is the only
frame-rotation number in this run that should be quoted.

Elsewhere the direction-of-travel estimate has a circular sd of 56 deg (12
pairs, t 200-300 s) -- too slow a vehicle and too few long displacements. The
scale ratio |slam step| / |truth step| is 0.935 with sd 0.328 over 20 pairs,
which is consistent with unit scale and says nothing sharper.

**Attitude remains closed, for a better reason.** At t 300-400 s the position
frame is at -14.4 deg (sd 2.0) while `yaw_ekf - yaw_slam` reads +77 to +79 deg
(sd 0.4-1.3). Both are tight; they simply disagree. SLAM's reported
orientation is not the orientation of the frame its positions live in, so it
cannot be used to estimate that frame's rotation -- whatever the cause.

**None of this touches the main result.** The 41 m failure, its reproduction
across two seeds, the constant 40.76 m world-frame offset, the departure from
dead reckoning 18 s after the first alignment, and the feedback loop
(`reference_odom` defaulting to the filter the bridge feeds) rest on direct
truth-vs-estimate differences, on telemetry, and on the code -- not on any
fit. The open question is why the solver locked at +152.4 deg when +14.4 deg
was right, and the `reference_odom` run is the experiment that answers it.

---

# The reference experiment, seed 1 -- 2026-10-14

Bag `a50_slamref_seed1_20261014_140856` (1894 s, 185.9 m), unit verified as
seed 1 to 0.00038, launched with `dr_reference:=true` so the bridge fits its
rotation against `/bluerov2/odometry/dr` -- a second filter fusing DVL, depth
and IMU and no SLAM. Everything else unchanged.

## The loop was the cause

| | loop closed (12 Oct) | loop open |
|---|---|---|
| final position error | **41.04 m** | **1.68 m** |
| mean / max error | 38.50 / 41.10 m | 2.81 / 5.08 m |
| drift | 25.01 % | 0.90 % |
| ATE RMSE | 39.35 m | 3.19 m |
| bearing error, mean / max | 73.1 / 163.3 deg | 4.40 / 15.34 deg |
| true range to turbine, max | 57.0 m | **26.2 m** |
| time beyond 2x standoff | 715 s | **0 s** |

Twenty-four times less final error, and the excursion is gone entirely: the
vehicle never left the working envelope. The registration was being estimated
against the filter it fed, and that is what produced the 41 m failure.

## What the open loop reveals

With an independent reference the alignment stops agreeing with itself, and
the disagreement is large:

- **13 separate re-alignments from scratch**, each triggered by `SLAM
  alignment stale after 40 consecutive rejections`.
- theta solutions spanning the circle: -159.2, +79.5, -168.4, -15.1, +55.0,
  -133.5, -10.3, +142.0, -79.9, +44.0, +164.5, -48.2, +80.7, -138.8, -43.8,
  +116.3, -156.9 deg.
- refits rejected for proposing changes of 31, 65, 131, 148, 154, 164 and
  **174 deg**.

In the closed-loop runs **no refit was ever rejected**. That contrast is the
result: the loop did not merely bias the estimate, it made the estimate
unfalsifiable. The same solver, given a reference it does not drive,
disagrees with itself by up to 174 deg.

## SLAM is still not contributing

Both filters ran in this bag, so they can be compared directly:

| | final error | % of 185.9 m | max |
|---|---|---|---|
| SLAM-free reference filter (DVL + depth + IMU) | **0.47 m** | 0.25 % | 0.53 m |
| primary filter (same, plus SLAM as pose0) | **1.68 m** | 0.90 % | 5.08 m |

Adding SLAM makes the estimate 3.6x worse at the end and 10x worse at its
worst. The gap between the primary filter and its own reference averages
2.82 m and peaks at 4.97 m -- pressed against the bridge's 5.0 m rejection
threshold. The error is bounded by the gate, not by SLAM being right: the
filter is pulled off by poses that pass the gate until it reaches the
threshold, then the gate catches the rest.

So the honest statement of where this stands:

1. The catastrophic failure is diagnosed and fixed. It was an observability
   defect, not a SLAM accuracy problem, and the fix is a configuration change.
2. The gate works now, because it finally has something independent to gate
   against. That is what bounds the error at ~5 m.
3. **The alignment itself still does not work.** It never found a stable
   map->world rotation in 1894 s. On this mission dead reckoning alone beats
   dead reckoning plus SLAM, and claiming SLAM-aided navigation on this
   evidence would not survive review.

The remaining problem is `_try_solve`: a rotation-only Procrustes fit over a
75 s buffer, gated on `spread` (a bounding-box diagonal) rather than on
whether the track in that buffer actually determines a rotation. Every one of
the 13 locks fired at spread = 5.0 m exactly, the minimum. Seeds 0 and 2 of
this arm, and a conditioning test at the lock, are the next steps.

## Registered before seeds 0 and 2 of the reference arm -- 2026-10-14

Written before either run exists. Seed 1 is the only `dr_reference` bag so
far, so these are predictions from one sample plus the arm-1 baselines, not
from a fitted model.

1. **The DR reference filter reproduces arm 1.** It is the same configuration
   (DVL + depth + IMU, no SLAM), so `/bluerov2/odometry/dr` should land near
   arm 1's final errors -- 0.11 m for seed 0 and 0.51 m for seed 2 -- within
   a factor of about three. Seed 1 gave 0.47 m here against 0.34 m in arm 1,
   so the trajectories differ enough to move it but not by an order of
   magnitude. **Falsified if either exceeds 2 m.**
2. **The primary filter stays bounded by the gate, not by SLAM.** Final error
   of order 1-5 m, maximum under 10 m, because the bridge rejects any SLAM
   pose more than 5.0 m from the reference. **Falsified by anything above
   10 m**, which would mean the gate is not what bounds it.
3. **No excursion.** Maximum true range under about 30 m and zero time beyond
   twice the 17 m standoff, against 57 m and ~715 s with the loop closed.
4. **Refits get rejected.** A non-zero count of `Refit rejected` and at least
   one `re-aligning from scratch`. Zero rejections in either run would mean
   the alignment agreed with an independent reference, which neither the
   seed-1 reference run nor the mechanism predicts.
5. **SLAM still does not help.** The primary filter ends further from truth
   than its own SLAM-free reference in both runs. **Falsified if SLAM beats
   dead reckoning on either seed** -- which would be the first evidence that
   the alignment sometimes works, and would change what the paper can claim.

Criterion 5 is the one worth watching. The others confirm the mechanism; that
one asks whether anything is salvageable in the current solver.

# Reference arm, seed 0 -- 2026-10-14

Bag `a50_slamref_seed0_20261014_153610`, 1287 s, 183.0 m travelled, 10 topics
including `/bluerov2/odometry/dr`. The recording guard passed against the live
system (`live seed='0' slam_pubs='1'`), and `verify_dvl_model.py` recovers the
seed 0 operator to a maximum elementwise difference of 0.00021, so the arm and
seed on the label are the arm and seed that flew.

## Scoring against the registered predictions

| # | prediction | result | |
|---|---|---|---|
| 1 | DR reference within ~3x of arm 1 (0.11 m); falsified above 2 m | **0.11 m** | held |
| 2 | primary filter 1-5 m final, max under 10 m; falsified above 10 m | 0.90 m final, **5.06 m** max | held, but see below |
| 3 | max true range under ~30 m, zero time beyond 34 m | **31.9 m**, 0 s | mostly held |
| 4 | non-zero `Refit rejected`, at least one re-align from scratch | **20** and **12** | held |
| 5 | SLAM loses to its own SLAM-free reference; falsified if SLAM wins | lost by 8x | held |

Two of these need the qualification rather than the tick:

- **Prediction 2 missed low on the final value.** 0.90 m is below the stated
  1-5 m band. The band was not the falsification criterion and the maximum
  (5.06 m) sits exactly where the gate predicts, but the point estimate was
  wrong and the final-error metric is the reason: the primary filter peaked at
  5.06 m and happened to be near truth when the mission ended. Its RMSE over
  the run is 3.40 m. Quoting 0.90 m without that would flatter it.
- **Prediction 3 is 31.9 m against "under about 30".** Close enough to be
  consistent with the mechanism and nowhere near the 57 m of the closed-loop
  runs, but it is over the number written down, and the excursion happened at
  t = 1146 s during RISE, where the filter believed 28.0 m.

## The reference is as good as the no-SLAM arm, and SLAM is 34x worse

| | final | RMSE | max |
|---|---|---|---|
| SLAM-free reference filter (DVL + depth + IMU) | **0.11 m** | **0.10 m** | 0.17 m |
| primary filter (same, plus SLAM as pose0) | 0.90 m | 3.40 m | 5.06 m |

0.11 m final against arm 1 seed 0's 0.11 m: the reference filter reproduces
the no-SLAM arm exactly, which is the control this whole design needed. Over
183 m it never exceeds 0.17 m -- 0.09 % of distance travelled.

The gap between the primary filter and its own reference averages **2.70 m and
peaks at 4.98 m**, against the bridge's 5.0 m rejection threshold. Seed 1 gave
2.82 m and 4.97 m. Two seeds now press against the gate to within 2 cm. The
error is set by where the gate is, not by SLAM being approximately right.

## The alignment still does not converge

13 alignments solved from scratch in 1287 s, at theta = +150, +55, -2, -159,
+130, +35, +134, -59, +42, -155, -108, -159 and -45 deg -- spread over the
full circle, as in seed 1. 20 refits rejected, the largest proposing a 35.7 deg
change. **Every one of the 13 locks fired at `spread = 5.0 m` exactly**, the
configured minimum, which is the same signature as seed 1 and confirms that
`_try_solve` is firing the instant the bounding box is large enough rather than
when the geometry determines a rotation.

Each lock then decays the same way: a run of accepted refits drifting a few
degrees per cycle, then rejections as the proposals jump by 15-35 deg, then 40
consecutive pose rejections, then a re-align from scratch. The cycle ran 12
times. There is no sign of convergence toward a fixed map->world rotation at
any point in the mission.

## Status

Criterion 5 is what mattered and it held on a second independent seed: with
the loop open, dead reckoning alone beats dead reckoning plus SLAM, by 8x on
final error and 34x on RMSE. Nothing here is salvageable by tuning the gate --
the gate is doing its job. The defect is in `_try_solve`'s decision of *when*
it is entitled to claim a rotation. Seed 2 remains, then the conditioning test
at the lock.

# Reference arm, seed 2 -- 2026-10-14: VOID for this arm

Bag `a50_slamref_seed2_20261014_162539`, 1157 s, 158.1 m. **The a50 error
model was not running.** `verify_dvl_model.py` recovers an operator that is
the identity to 0.02, and the analyzer measures the DVL gains as 0.99997 /
1.00009 / 1.00002 -- deviations of 0.009 %, 0.6 sigma, consistent with
estimation noise alone. This bag carries the white-noise-only DVL, not the
seed 2 unit, so it cannot be scored against a pre-registration written about
the a50 arm.

## How it got past the guard

`dvl_error` defaults to `none`. The launch file runs the plain bridge and the
a50 bridge under opposite conditions **with the same node name**, and
`dvl_bridge.py` declares `error_seed` in both cases, defaulting `scale_sigma`
and `misalign_sigma_deg` to zero. A launch that omits `dvl_error:=a50`
therefore answers `ros2 param get /dvl_bridge error_seed` with exactly the
seed that was asked for, while the error operator reduces to the mount
rotation. The guard checked the seed and nothing else, so it passed.

This is the second time this experiment has produced a white-noise bag under
an a50 name. The first (2026-10-09, seed 0) came from a `PythonExpression`
that silently yielded a non-existent filename; the conditions were introduced
to make that impossible, and they did. This one came from the argument simply
not being typed -- a different hole in the same wall.

Fixed in `record_run.sh`: the guard now also reads `scale_sigma` and refuses
to record when it is zero. `scale_sigma` is the parameter that actually
separates the two bridges; `error_seed` never could.

## What the bag does show

It is a valid run of a different condition, and worth keeping for one reason:

| | final | RMSE | max |
|---|---|---|---|
| SLAM-free reference (white-noise DVL) | **0.04 m** | **0.02 m** | 0.04 m |
| primary filter (same, plus SLAM) | 4.97 m | 3.53 m | 4.98 m |

Over 158 m the reference filter never exceeds 4 cm. Adding SLAM makes it
**170x worse in RMSE**, and the gap to its own reference peaks at 4.97 m
against the 5.0 m gate -- the same signature as seeds 0 and 1. The alignment
is not merely failing to help a drifting estimate; it degrades one that is
essentially exact. Thirteen locks again, all at `spread = 5.0 m`, theta
spanning +161, +117, -24, +72, -115, -35, +142, -131, +40, +113, -57, -19,
+157 deg, with 15 refits rejected.

Criterion 5 therefore holds in this run too, under a condition that makes it
harder to pass, not easier. Criteria 1-4 are not scored: the arm is wrong.

Seed 2 of the reference arm still needs to be flown, with `dvl_error:=a50`.

# Reference arm, seed 2 (reflown with dvl_error:=a50) -- 2026-10-14

Bag `a50_slamref_seed2_20261014_200359`, 1360 s, 157.2 m. This time the model
was on: `verify_dvl_model.py` recovers the seed 2 operator to a maximum
elementwise difference of 0.00046, and the analyzer measures gains of -0.069 %,
+0.412 %, -0.684 % (33 sigma). The arm is now complete with three valid seeds.

## Scoring

| # | prediction | result | |
|---|---|---|---|
| 1 | DR reference within ~3x of arm 1 (0.51 m); falsified above 2 m | **0.57 m** | held |
| 2 | primary 1-5 m final, max under 10 m; falsified above 10 m | 5.52 m final, 5.54 m max | held, band exceeded by 0.5 m |
| 3 | max true range under ~30 m, zero time beyond 34 m | **23.3 m**, 0 s | held |
| 4 | non-zero `Refit rejected`, at least one re-align from scratch | **17** and **13** | held |
| 5 | SLAM loses to its own SLAM-free reference; falsified if SLAM wins | lost by 9.7x | held |

Prediction 2 overshot its band on the high side here, as it undershot on seed 0.
Both are inside the stated falsification bound, and in both cases the maximum
sat within half a metre of the gate -- which is the quantity the prediction was
really about. The final-error band was the wrong thing to have written down.

## The whole arm

| seed | DR reference final | reference RMSE | primary final | primary RMSE | arm-1 no-SLAM final |
|---|---|---|---|---|---|
| 0 | 0.11 m | 0.10 m | 0.90 m | 3.40 m | 0.11 m |
| 1 | 0.47 m | 0.34 m | 1.68 m | 3.19 m | 0.34 m |
| 2 | **0.57 m** | **0.40 m** | **5.52 m** | **4.06 m** | 0.51 m |

The reference filter reproduces arm 1 on all three seeds (0.11/0.47/0.57
against 0.11/0.34/0.51), which is the control the design needed: the thing the
SLAM alignment is fitted against is itself sound, and is the same estimator
arm 1 reported.

Against it, SLAM costs a factor of **8 to 10 on final error and 9 to 34 on
RMSE, on every seed**. The filter-to-reference gap peaks at 4.98, 4.97 and
4.96 m against a 5.0 m gate -- three independent seeds within 4 cm of the
threshold. The error is set by where the gate is.

## The solver, three for three

| seed | locks | locks at spread = 5.0 m | refits rejected | largest rejected change |
|---|---|---|---|---|
| 0 | 13 | 13 | 20 | 35.7 deg |
| 1 | 13 | 13 | 7 | 174 deg |
| 2 | 13 | 12 (one at 5.1) | 17 | 44.7 deg |

Thirteen locks in every run, essentially all at the configured minimum
baseline, theta spread over the full circle each time, and no convergence
toward a fixed map->world rotation in any of them. `_try_solve` fires on
`spread` -- a bounding-box diagonal -- the instant the box is big enough,
without asking whether the track inside it determines a rotation at all. That
is the defect, and it is now characterised on three seeds rather than argued
from one.

## Side observation: RISE destroys the map

Not predicted, and worth recording before it is forgotten. ORB-SLAM3 lost its
map (`Map point crash detected`) during RISE in both runs whose RISE handoff
demanded a large yaw swing -- 495777 -> 317 points at a -125 deg swing here,
552411 -> 946 at -137 deg in the voided run. The seed 0 run, whose swing was
only -34 deg, kept its map until after MISSION COMPLETE. Three runs is not a
result, but the pattern is specific enough to test deliberately, and it bears
on the open TRANSIT->RISE yaw question.

## Status of the arm

All five criteria held on all three seeds. The mechanism is established:

1. The catastrophic 41 m failure was an observability defect -- the alignment
   was fitted against the filter it fed -- and opening that loop fixes it.
2. With the loop open the gate bounds the error at ~5 m, confirmed to within
   4 cm on three seeds.
3. **The alignment itself does not work.** On this mission, on every seed,
   dead reckoning alone beats dead reckoning plus SLAM. No amount of gate
   tuning changes that; the gate is doing its job.

The next step is not another seed. It is a conditioning test at the lock:
measure `sv[1]/sv[0]` of the cross-covariance at each of the 39 solves across
the three bags, and check whether the locks that held longest were the ones
with the best-conditioned geometry. If they were, the fix to `_try_solve` is
to gate on conditioning instead of `spread`.

# The conditioning test -- 2026-10-14: hypothesis refuted, defect relocated

Registered expectation (end of the seed 2 section): that the locks which
survived longest would be the better-conditioned ones, and that the fix to
`_try_solve` would be to gate on conditioning instead of `spread`.

**That is wrong, and the measurement says so clearly.**

`eval/lock_conditioning.py` reconstructs every lock from the bags alone. The
bridge copies `out.header = m.header` and overwrites only x,y, so each
published pose matches its raw pose by header stamp and the transform it
carried is recoverable exactly; locks are where publishing resumes after a
silence. The reconstruction is checked rather than assumed -- theta recomputed
from the reconstructed window against theta read off the transform agrees to a
**median of 2.5 deg, 40 of 42 locks within 15 deg**. The two that fail (the
first lock of seed 1, and one after a map crash) are dropped.

## No conditioning metric predicts survival

Spearman rank correlation against how long each of the 40 locks lasted:

| metric | rho_s |
|---|---|
| `spread` (what the gate uses today) | **+0.09** |
| axis ratio of the window | -0.31 |
| rho (a rotation explains the pair) | +0.38 |
| total SLAM-side scatter | -0.06 |
| sigma_theta (std error of theta) | -0.36 |
| residual RMS | -0.34 |

`spread` has no predictive power at all, which was expected. But nothing else
reaches 0.4 either. There is no conditioning gate to build.

## Because the windows were never ill-conditioned

| | median | extreme |
|---|---|---|
| sigma_theta | **0.39 deg** | 1.31 deg |
| rho | **0.984** | 0.777 |
| axis ratio | 0.087 | 0.468 |

Every window pins theta down to under half a degree, with a rotation
explaining the correspondence pair almost exactly. The earlier reading --
ill-conditioned fits producing arbitrary rotations -- was wrong. **The fits
are sharp. They sharply disagree with each other.** That is a different
defect, and a conditioning gate cannot touch it.

## There is no fixed rotation to find

Sliding 40 s windows, 10 s apart, over each whole mission, with the bridge
out of the loop entirely:

| seed | median step between adjacent windows | max | circular dispersion of theta |
|---|---|---|---|
| 0 | 19.0 deg | 170 deg | 0.88 |
| 1 | 21.4 deg | 175 deg | 0.97 |
| 2 | 18.4 deg | 178 deg | 0.74 |

Adjacent windows overlap by 75 % of their data and still disagree by ~20 deg,
while each is internally sharp to 0.4 deg. Dispersion near 1 means theta
covers the circle. The reference filter's own error cannot explain this: its
worst case across all three runs is 0.62 m, about 7 deg over a 5 m baseline,
and it varies far more slowly than the 10 s between windows.

## What the SLAM track actually is

`eval/slam_track_fidelity.py`:

| | seed 0 | seed 2 |
|---|---|---|
| continuity | 15275 poses, median step 2.0 cm, **no step over 2 m** | 15710 poses, 2.0 cm, none |
| local scale over 40 s | 0.93 [0.82, 1.14] | 0.95 [0.80, 1.03] |
| path length SLAM vs truth | 428 m vs 181 m (**2.4x**) | 435 m vs 157 m (**2.8x**) |
| best global similarity fit | scale 0.44, residual **16.8 m** | scale 0.24, residual **16.5 m** |
| true track span | 53 x 37 m | 41 x 40 m |

The track is continuous -- it is not being re-founded, so that explanation is
closed off. Locally it is good: over 40 s the scale is within ~7 % of unity
and a rotation fits to 0.4 deg. Globally it is wrong: the **best possible**
similarity transform still leaves a **16.5-16.8 m RMS residual on a ~40 m
trajectory**, and the integrated path is 2.4-2.8x too long.

That residual is the floor for any global alignment whatsoever. No trigger,
no gate, no conditioning test, no amount of re-fitting beats it, because it
is not an alignment problem. ORB-SLAM3 is accumulating drift faster than a
rigid map->world transform can absorb.

## What this changes

The three-seed result stands and its explanation is now complete: SLAM makes
the filter 8-10x worse not because the alignment is triggered badly but
because **there is nothing for the alignment to converge to.** Every repair
aimed at `_try_solve` -- including the one registered above -- is aimed at the
wrong layer.

Three directions follow, and they are genuinely different claims for the
paper, so the choice is not mine to make:

1. **Make the SLAM work.** The drift is the defect; loop closure or scene
   handling is the fix. Largest scope, and the only route to "GPS-denied
   navigation using visual SLAM" as a global-pose claim.
2. **Use SLAM locally, not globally.** The measurements say the local track
   is good -- 0.93 scale, 0.4 deg rotation over 40 s -- and only the global
   frame is wrong. Relative or incremental constraints are supported by this
   evidence in a way a global pose is not. Smaller scope, and defensible.
3. **Report what is measured.** DR for navigation, SLAM for mapping, with
   this drift characterisation as a result in its own right.

Nothing further should be run against the current bridge. The next experiment
depends on which of the three this becomes.
