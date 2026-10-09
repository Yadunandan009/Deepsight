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
