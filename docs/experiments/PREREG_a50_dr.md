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
