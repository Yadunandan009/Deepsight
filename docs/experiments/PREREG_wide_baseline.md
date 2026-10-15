# Pre-registration: wide stereo baseline (b 0.145 -> 0.30 m)

Registered 2026-10-15, before the run.

## Hypothesis

The dominant cause of the SLAM failure on this target is that the stereo
rig cannot observe scale at the mission's working range.

ORB-SLAM3 gates triangulated stereo depth at

    mThDepth = mbf * ThDepth / fx = b * ThDepth        (fx cancels)

Beyond it a point is *far*: bearing only, no stereo depth, usable as a map
point only after multi-view triangulation with sufficient parallax. With
`Stereo.b = 0.145` and `Stereo.ThDepth = 40`, that gate sits at **5.80 m**.

Measured against `a50_slamref12_seed0_20261015_180532`
(11488 multibeam scans, 673955 returns):

- median range to the nearest visible surface **8.1 m**; median return 12.8 m
- **86.6 %** of surface returns are beyond the gate
- **71.8 %** of frames contain no close point at all

So the system is stereo in configuration and close to monocular in
behaviour: scale accumulates and is never corrected.

## Why the baseline and not the standoff

The obvious fix is to fly closer. It is not available. The jacket legs
splay to a p95 radius of **14.1 m** at 18-24 m depth, and `SCAN_DIST` is
already **17.0 m**, leaving 2.9 m of clearance against a 2.5 m
`SONAR_STOP`. 15 m leaves 0.9 m; 14 m is inside the envelope. The vehicle
is already as close as this structure permits.

Of the four levers on the gate, three are unavailable or dishonest:

| lever | effect | verdict |
|---|---|---|
| fly closer | would move the gate's reach | blocked by the structure |
| raise `ThDepth` | moves the gate, not the disparity | relabels 3.6 px as trustworthy |
| narrow the FOV | raises disparity; fx cancels in `mThDepth` | does not move the gate |
| **widen `b`** | moves the gate and the disparity | **the only one left** |

This is itself a result: a 0.145 m baseline is the wrong sensor for a
~28 m-wide structure that cannot be approached.

## The change

`b = 0.30 m`, chosen as the widest value mountable on a 0.457 m-wide
BlueROV2 frame. Cameras move from y = +/-0.0725 to y = +/-0.15.

They also move forward, **x 0.16 -> 0.22**. This was not in the original
plan and was forced by a failed launch on 2026-10-15: at y = +/-0.15 and
the stock x = 0.16, the vehicle's own hull enters the 75x60 deg frustum
(676 mesh vertices, nearest 3.3 cm) and both images are mostly the ROV.
The body's nose is at x = +0.222; x = 0.22 is clear even at an inflated
110x95 deg frustum. The 6 cm forward shift is a constant offset in the
SLAM frame and is absorbed by the alignment's translation term, so it is
not a second experimental variable in any way the metrics can see.

Nothing else changes: same trajectory, same `SCAN_DIST`, same mission
timing, same symmetry exposure, same DVL seed.

- `scenarios/bluerov2_b30.scn`, `scenarios/bluerov2_turbine_b30.scn`
- `slam_params/bluerov2_stereo_b30.yaml` (three numbers: `Stereo.b`,
  `T_c1_c2` translation, `RIGHT.P[0,3] = -fx*b`)
- `eval/check_stereo_baseline.py` enforces that the five places the
  baseline is written down agree, for both arms; that each config is
  readable by `cv::FileStorage` (ORB-SLAM3 aborts on a settings file whose
  first line is not `%YAML:1.0`, which cost one launch); and that no
  camera has the vehicle's own body in frame (which cost another)
- `record_run.sh` additionally refuses to record if the baseline Stonefish
  is simulating, read live from the right `CameraInfo` as `P[3] = -fx*b`,
  disagrees with the `Stereo.b` in the yaml bind-mounted into the running
  SLAM container

## Predictions, stated before the run

Mechanical, from the gate arithmetic on the existing bag:

| | b = 0.145 | b = 0.30 |
|---|---|---|
| close-depth gate | 5.80 m | 12.00 m |
| returns inside the gate | 13.4 % | **46.4 %** |
| frames with any close point | 28.2 % | **89.1 %** |
| disparity at 8.1 m | 7.5 px | **15.4 px** |

Those follow from geometry and are not the test. The test is whether the
navigation metrics move. Baselines are the three fps=30 seeds plus the
fps=12 seed-0 run.

**The hypothesis is SUPPORTED if, on seed 0 at fps 12:**

1. local scale over 40 s windows rises from 0.91-0.95 toward 1.00
   (criterion: median within 0.97-1.03)
2. SLAM path length / truth falls from 1.83-2.77 toward 1.0
   (criterion: < 1.3)
3. the best global similarity residual falls well below trajectory scale
   (criterion: < 5 m, against 16.5-17.5 m now, on a ~40 m trajectory)
4. circular dispersion of theta falls from 0.88-0.91 (criterion: < 0.5)

**The hypothesis is REFUTED if** (1) and (2) do not move outside the
existing four-run spread. In that case scale starvation is not the
dominant mechanism and the restated mechanism in
`docs/MECHANISM_RESTATEMENT.md` is wrong in turn.

Secondary, recorded but not criteria: tracking losses, Atlas map count,
loop-closure detections, whole-run ATE, ATE against the SLAM-free
reference. **ATE is deliberately not a criterion.** SLAM currently makes
the filter 8-10x worse, and a fix to scale observability could plausibly
improve the map while the 5 m rejection gate still dominates the filter's
error. Conflating the two is how the previous round's conclusions went
wrong.

## Run sequence

    python3 eval/check_stereo_baseline.py        # must pass first

    ros2 launch stonefish_bluerov2 bluerov2_sim.py \
        scenario:=bluerov2_turbine_b30 dvl_error:=a50 dvl_seed:=0 \
        dr_reference:=true

    bash run_stereo_slam.sh bluerov2_stereo_b30.yaml
    python3 src/stonefish_bluerov2/scripts/bluerov2_autonomous_controller.py
    bash record_run.sh slamb30 0

Scenario files are installed by `install(DIRECTORY scenarios ...)`, so the
two new `.scn` files need a `colcon build` before the launch will find
them.

## Known confound

`record_run.sh` verifies the DVL seed and `scale_sigma` from live
parameters, but has no equivalent check that the stereo baseline in the
running scenario matches the SLAM yaml. `check_stereo_baseline.py` checks
the files on disk; it cannot check what Stonefish actually loaded. Running
it immediately before the launch is the mitigation, not a guarantee.

---

# Result: REFUTED (2026-10-16)

`a50_slamb30_seed0_20261015_204105`, verified by `verify_dvl_model.py` as
carrying the seed 0 unit (max elementwise difference 0.00020).

| criterion | threshold | b = 0.30 | prior four-run spread |
|---|---|---|---|
| local scale, 40 s windows | 0.97-1.03 | **0.91** | 0.91-0.95 |
| SLAM path / truth | < 1.3 | **1.82** | 1.83-2.77 |
| best global similarity residual | < 5 m | **17.0 m** | 16.5-17.5 |
| circular dispersion of theta | < 0.5 | **0.96** | 0.88-0.91 |

The registered refutation condition was that criteria 1 and 2 fail to move
outside the existing spread. Both are inside it. **Refuted.**

Secondary: whole-run ATE 3.78 m (prior 3.19-3.46), SLAM-free reference
0.19 m, filter-vs-reference max 4.97 m against the 5.00 m gate, two Atlas
collapses both during RISE.

Doubling the baseline moved the close-depth gate from 5.80 m to 12.00 m
and changed nothing measurable. Scale starvation at the mission's working
range is **not** the dominant mechanism.

One caveat on how cleanly the intervention was delivered: peak map points
fell from 483769 (b = 0.145) to 291578 (b = 0.30), a 40% reduction. The
predicted rise in close points was never confirmed directly -- ORB-SLAM3
does not publish its close/far counts -- and a wider baseline also reduces
stereo overlap at short range, which plausibly costs more matches than the
deeper gate wins. So this run refutes the hypothesis as *implemented*; a
cleaner delivery might not change the conclusion, but it was not achieved
here.

## What it led to

Chasing why the metrics would not move produced the actual defect: the
SLAM pose is published in a z-up (ENU) frame while this workspace is NED,
and `slam_pose_bridge` fits a horizontal rotation that cannot represent
the difference. See `docs/SLAM_FRAME_ENU_NED.md`. With the conversion
applied, the same three runs fit a single yaw and scale to a 1.6-2.7 m
residual at scale 0.95-0.98.

That means **this arm's own criteria were measured through the bug**. The
refutation above still holds -- the bug is identical in both arms, so the
comparison between them is controlled -- but the absolute numbers in this
table are not the SLAM's true fidelity. The wide-baseline question should
be re-asked after the frame fix if it matters; the cheap arm is b = 0.145,
which is also the arm everything else was flown with.
