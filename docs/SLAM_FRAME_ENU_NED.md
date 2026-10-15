# The SLAM pose arrives in ENU; this workspace is NED (2026-10-16)

Every result that rested on "no fixed map-to-world transform exists" was
measuring an integration bug, not the SLAM.

## The finding

`/bluerov2/robot_pose_slam` is published in a z-up (ENU-style) frame. The
simulator, the EKF, the DVL and the controller all work in a z-down (NED)
frame -- `D_BASE = 22.0` is 22 m of depth, not altitude. Nothing in the
chain converts between them.

The two frames differ by `(x, y, z) -> (x, -y, -z)`. That matrix has
determinant **+1**: it is a 180 deg rotation about the x axis, a perfectly
ordinary frame change. It only looks pathological because
`slam_pose_bridge` fits a rotation **in the horizontal plane** and discards
z -- and in 2D, `y -> -y` is a reflection, which no rotation can represent.

So the bridge was asked to find a rotation that does not exist, every five
seconds, for the whole mission.

## Applying it recovers the trajectory

`eval/enu_ned_check.py`, full 3D, fitting only what the bridge already
fits -- one yaw and one scale:

| run | pre-transform | yaw | scale | 3D residual |
|---|---|---|---|---|
| slamref seed 0 | none (as published) | +2 deg | 0.316 | 18.72 m |
| | **ENU->NED** | **+4 deg** | **0.976** | **1.63 m** |
| slamref12 seed 0 | none | -6 deg | 0.067 | 18.57 m |
| | **ENU->NED** | **+5 deg** | **0.962** | **1.79 m** |
| slamb30 seed 0 | none | -27 deg | 0.060 | 18.14 m |
| | **ENU->NED** | **+14 deg** | **0.950** | **2.65 m** |

A fixed transform exists after all, the scale is near metric, and the
residual is 1.6-2.7 m on a ~40 m trajectory. **The SLAM was never the
problem.** It has been good the whole time and the consumer has been
reading it in the wrong frame.

## Which cause: frame convention, not a swapped stereo pair

Two mechanisms can mirror a track. They were separated on evidence, not
on argument.

**1. The vertical axis is inverted at unit gain.** `eval/axis_discriminator.py`
regresses the published SLAM z against the independently measured depth
(pressure sensor, 0.005 m error):

| run | gain | correlation |
|---|---|---|
| slamref seed 0 | **-0.985** | -0.990 |
| slamref12 seed 0 | **-0.957** | -0.957 |
| slamb30 seed 0 | **-0.976** | -0.916 |

A swapped stereo pair has no mechanism to invert up and down. It breaks
triangulation; it does not flip the vertical axis. A z-up/z-down
relabelling inverts it at exactly -1. Measured -0.96 to -0.99.

**2. Scale is metric and repeatable.** After correction, 0.950, 0.962,
0.976. With the images swapped, every true correspondence has negative
disparity and ORB-SLAM3 rejects it, leaving an effectively monocular
system whose scale is arbitrary and differs run to run. Three runs within
3% of unity says stereo triangulation is working correctly.

**3. The mirror line is fixed.** The horizontal mirror sits at +2.0, +2.5
and +7.0 deg from the world x axis across three runs -- including one
where the stereo baseline changed from 0.145 m to 0.30 m. A frame
convention does that. A sensor- or scene-dependent effect does not. The
few degrees of spread is the arbitrary yaw of SLAM's map initialisation,
which the bridge is designed to estimate.

**It is not an upstream bug.** `se3ORBToROS` in the wrapper
(`type_conversion.cpp`) was checked: `tfORBToROS` has determinant +1 and
the algebra is a correct conjugation. The wrapper does what it says --
emits ROS REP-103, x forward, y left, z up. This workspace is NED. The
defect is that the boundary between them has no conversion.

## What it invalidates

- **"No fixed map-to-world transform exists."** False. It exists; the
  bridge's horizontal-rotation model cannot express it.
- **`docs/MECHANISM_RESTATEMENT.md` section 4** (stereo scale starvation).
  Already refuted independently by the wide-baseline run; this supersedes
  it. Sections 1-3 stand: the failure is still not perceptual aliasing.
- **`eval/slam_track_fidelity.py`'s global residual and local scale**, and
  the "degenerate scale 0.06-0.44" fits. All of them fit a horizontal
  rotation, so all inherited the same blind spot. The continuity and
  path-length numbers are unaffected -- those are frame-invariant.
- **The three-seed "SLAM costs 8-10x" result.** The observation stands;
  the explanation was wrong. The filter was being fed poses through a
  model that could not fit them.

Path length is reflection-invariant, so the 1.8-2.8x path inflation is
**not** explained by this and remains open.

## The fix

Convert on ingest, in the consumers, not in the container. Four live
files subscribe to `/bluerov2/robot_pose_slam`:

    src/stonefish_bluerov2/scripts/slam_pose_bridge.py
    src/stonefish_bluerov2/scripts/adaptive_fusion_node.py
    src/stonefish_bluerov2/scripts/bluerov2_autonomous_controller.py
    src/stonefish_bluerov2/scripts/bluerov2_expo_demo.py        (demo)
    src/stonefish_bluerov2/scripts/bluerov2_controller2nd.py    (old)

Position takes `y -> -y`, `z -> -z`. Orientation takes the same 180 deg
rotation about x, which inverts yaw -- consistent with the measured net
yaw of +453 deg against a true -452 deg.

A single shared helper is better than five copies, and the conversion
should be asserted once at startup rather than trusted.

## Pre-registered prediction for the next run

With ENU->NED applied, on seed 0 at fps 12, b = 0.145:

1. the bridge's theta locks and stops wandering: circular dispersion
   **< 0.3** (now 0.88-0.96)
2. re-alignments from scratch fall from 13-14 per run to **<= 2**
3. `slam_track_fidelity` global residual **< 4 m** (now 16.5-17.5)
4. the filter stops riding the 5 m rejection gate: filter-vs-reference
   max **< 3 m** (now 4.96-4.99 against a 5.00 gate)

Refuted if theta still disperses above 0.5 after the conversion.

ATE is again **not** a criterion. Whether a correctly-framed SLAM pose
actually beats the 0.11-0.19 m dead-reckoning reference is a separate
question, and the honest answer may still be no.
