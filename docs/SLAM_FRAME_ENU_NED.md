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

---

# The eval tools were measuring through the bug too (2026-10-16)

`slam_track_fidelity.py`, `lock_conditioning.py` and `stepdist.py` read
`/bluerov2/robot_pose_slam` straight from the bag. The live nodes convert
on ingest; these did not. So every fidelity number this project has
quoted was computed on the unconverted track. Fixed via `eval/slamio.py`,
which imports the one definition from `stonefish_bluerov2.frames` so the
eval path and the live path cannot disagree.

Validation: on `a50_slamref12_seed0_20261015_180532` the repaired tool
returns scale 0.96, theta +5 deg, residual 0.8 m -- matching
`reflect_check.py`'s independent answer (0.961, +5 deg, 0.82 m). Path
length ratio did not move (1.90 both ways), which is the control: path
length is reflection-invariant and must not change.

## The corrected history

| run | residual, as reported | residual, corrected | local scale, was -> is | path ratio |
|---|---|---|---|---|
| slamref seed 0 | 16.8 m | **1.4 m** | 0.93 -> **0.96** | 2.37 |
| slamref seed 1 | 17.3 m | **1.6 m** | 0.91 -> **0.97** | 1.83 |
| slamref seed 2 | 16.5 m | **2.9 m** | 0.95 -> **0.97** | 2.77 |
| slamref12 seed 0 | 17.5 m | **0.8 m** | 0.91 -> **0.95** | 1.90 |
| slamb30 seed 0 | 17.0 m | **1.3 m** | 0.91 -> **0.96** | 1.82 |
| slamned seed 0 | 16.7 m | 10.5 m | - | 2.51 |

Rotation dispersion over `slamref12`, recomputed: **0.00**. Adjacent
40 s windows step by a median of **2.1 deg**, not 18.4. The rotation was
a single fixed value for the entire mission.

**Every run before the fix had good SLAM.** Residual 0.8-2.9 m at scale
0.95-0.97 on a ~40 m trajectory, one fixed transform throughout. The
wandering theta, the degenerate scales, the 16-17 m residuals and "no
fixed map-to-world transform exists" were all one measurement artifact.

Tables elsewhere in `docs/` that quote residual, local scale or theta
dispersion from before this date are through-the-bug values. The
comparisons between arms in them remain valid -- the artifact was
identical in every arm -- but the absolute numbers are not.

## What survives

**Path-length inflation, 1.8-2.8x.** Reflection-invariant, so untouched
by any of this, and now the only unexplained defect. With the global fit
at 0.8-2.9 m and scale 0.96, the track is right at mission scale and
wrong at short timescales: high-frequency jitter, not drift. That is a
different problem from the one this project has been chasing.

**The slamned run is genuinely the worst of the six**, at 10.5 m and
local scale 0.48, because of a real 25.24 m relocalisation jump in the
raw topic at t = 572 s -- upstream of every line the frame fix touched.
Before that jump it fits at 0.82 m and scale 0.960 with one alignment
held for 342 s, four times the previous best lock.
