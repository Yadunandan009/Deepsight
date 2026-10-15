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
BlueROV2 frame. Cameras move from y = +/-0.0725 to y = +/-0.15. Nothing
else changes: same trajectory, same `SCAN_DIST`, same mission timing, same
symmetry exposure, same DVL seed. Single variable.

- `scenarios/bluerov2_b30.scn`, `scenarios/bluerov2_turbine_b30.scn`
- `slam_params/bluerov2_stereo_b30.yaml` (three numbers: `Stereo.b`,
  `T_c1_c2` translation, `RIGHT.P[0,3] = -fx*b`)
- `eval/check_stereo_baseline.py` enforces that the five places the
  baseline is written down agree, for both arms

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
