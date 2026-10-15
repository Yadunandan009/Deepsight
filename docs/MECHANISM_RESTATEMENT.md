# Restating the mechanism (2026-10-15)

The Introduction and Section 2.5 of the draft say the failure is perceptual
aliasing: the four congruent faces of the jacket make place recognition
match the wrong face, and the signature is an alignment error at a multiple
of ninety degrees.

Three measurements say that is not what is happening, and a fourth says what
is.

## 1. Loop closure almost never executes

ORB-SLAM3's own console logs over the three reference runs report **one**
loop detection in total (1, 0, 0) across three complete orbits of a
structure that is revisited four times by construction. Over the same runs:
41-202 tracking losses and 2-14 new maps created.

Perceptual aliasing is a failure *of place recognition at loop closure*. A
mechanism cannot be responsible for a run's error budget when it fires once
in three runs.

## 2. The rotation drifts continuously; aliasing predicts discrete jumps

The two models make opposite predictions about how the SLAM->world rotation
moves between adjacent 40 s windows (`eval/stepdist.py`, 476 steps over four
open-loop runs):

| adjacent-window step | observed | aliasing predicts |
|---|---|---|
| < 15 deg (same basin) | 42.2 % | the majority |
| 75-105 deg (one face) | **3.6 %** | the rest |
| > 165 deg (two faces) | 0.2 % | a little |
| 15-75 deg (neither) | **48.7 %** | ~0, forbidden |

Half of all observed steps land in a band that face confusion cannot
produce, and the 90 deg band that it must produce is the emptiest. Median
step 18.3 deg. This is drift, not switching.

## 3. The error is not a rotation at all

A rotation about the structure's axis is an isometry. It cannot change the
length of a path and it cannot change scale. Measured:

- integrated SLAM path length **1.8-2.8x** the true path
- local scale over 40 s windows **0.91-0.95**, p10 as low as 0.70

Whatever dominates the error budget changes lengths, so it is not a
rotation ambiguity.

## 4. The actual mechanism: the stereo rig cannot see scale at the mission's range

`slam_params/bluerov2_stereo.yaml`: `fx = 417.03`, `Stereo.b = 0.145`,
`Stereo.ThDepth = 40.0`. ORB-SLAM3 computes

    mThDepth = mbf * ThDepth / fx = b * ThDepth = 0.145 * 40 = 5.80 m

and treats any point beyond `mThDepth` as **far**: bearing only, no
triangulated stereo depth, usable for a map point only after multi-view
triangulation with enough parallax.

The mission's commanded standoff is 17.0 m and the observed track range runs
11.6-23.6 m. Disparity at those ranges:

| standoff | disparity | close? |
|---|---|---|
| 5.0 m | 12.09 px | yes |
| **5.8 m** | 10.43 px | **the boundary** |
| 12.0 m | 5.04 px | no |
| 17.0 m | 3.56 px | no |
| 22.0 m | 2.75 px | no |

**Every feature on the structure is a far point for the entire mission.** The
system is stereo in configuration and monocular in behaviour: scale is not
observed, only accumulated. That single fact predicts all four of the
symptoms already measured -- scale sagging to 0.91, path length inflating
1.8-2.8x, no fixed map-to-world similarity transform (16.5-17.5 m residual
on a ~40 m trajectory), and frequent tracking loss from map points that
never acquire reliable depth.

Note what this is not. The baseline is *consumed correctly*: the earlier
source audit traced `Stereo.T_c1_c2` and confirmed `b_ = 0.145` reaches
`bf_` properly. There is no calibration bug. The rig is correctly configured
and geometrically unable to do the job at this working distance.

## What the restated paragraph has to say

Symmetry may still matter, but as a contributor to *tracking loss* -- a
low-texture, self-similar lattice is hard to match frame-to-frame -- not as
false loop closure. The honest chain is:

    working range >> stereo close-depth limit
      -> no triangulated depth on the structure
      -> effectively monocular: scale accumulates, never corrects
      -> drift outruns any rigid map-to-world transform
      -> the bridge re-locks repeatedly at arbitrary angles
      -> (symmetry plausibly worsens the frame-to-frame matching
         that drives the tracking losses, but this is untested)

The ninety-degree signature is a separate, weaker claim and must be reported
on its own evidence. See R3 below.

## R3's current status, stated honestly

Pre-registered test, three open-loop seeds, n = 40 locks:
4-fold R = 0.253, **p = 0.076**. Largest of the six folds tested, correct
direction, **not significant**.

Pooling the fourth run (fps 12, seed 0) gives n = 53, 4-fold R = 0.238,
**p = 0.049**. This crosses 0.05 and **must not be reported as the result**:
the registered test was the three-seed one, and adding a run post hoc until
a threshold is crossed is exactly the practice pre-registration exists to
prevent. The honest statement is p = 0.076 at n = 40, with a fourth run
since collected that does not change the picture.

The dose-response (L0 vs L4) is the test that would make R3 a result rather
than a hint, and it has not been run open-loop.
