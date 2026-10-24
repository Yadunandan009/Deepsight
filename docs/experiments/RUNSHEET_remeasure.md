# Run sheet: the four re-measurement flights

*Drafted 2026-10-19. Run these after the map-scoring batch on the ten extra flights
has finished (it is CPU-heavy and reads the bag drive). You launch every process;
I read the logs you paste back.*

## Why these four

The paper's claim about what fusing SLAM does rests on **two** recordings made
after the axis fix. These four give it three seeds, and they are the first flights
of the **Sim(2) bridge** (`slam_pose_bridge.py`: fits scale, publishes an honest
covariance), which has been tested offline and never flown. So the flights also
test the bridge.

| flight | arm name | seed | `dr_reference` | what it measures |
|---|---|---|---|---|
| 1 | `slamref` | 0 | `true` | SLAM fused, alignment fitted against the SLAM-free filter |
| 2 | `slamref` | 1 | `true` | same, second DVL unit |
| 3 | `slamref` | 2 | `true` | same, third DVL unit |
| 4 | `slamcirc` | 0 | `false` | alignment fitted against the filter it feeds (the circular case), for the 41 m comparison |

Flight 4's number is **not** a SLAM result. It shows what the circular alignment
costs. Do it last.

## The sequence, for each flight

From `~/ros2_ws`, one terminal per step, in this order (the same as `CLAUDE.md`;
ArduSub is not involved). Start a fresh simulator for every flight; do not reuse a
running one. Each flight is about 20 to 30 minutes of mission (earlier bags ran 1150
to 1900 s). `dvl_error:=a50` must be in the launch: without it the bag looks right
and the DVL has no systematic error (this has happened).

**Heat.** On 2026-10-20 the laptop reached 97 C (limit 100 C) with the stack up and
the controller not yet started: ORB-SLAM3 alone uses about four cores. The launches
below turn rviz off (`rviz:=false`; display only, it does not touch the data). Do
**not** lower `rendering_quality`: it changes the images SLAM sees and would make
these flights different from the earlier ones. Close the browser, run on mains
power on a hard raised surface, let the machine cool below about 90 C between
flights, and do not leave the stack idling; bring it up only when you are ready to
start the controller within a few minutes. A flight run while throttled can run
slower than real time, so I will check the camera rate in each bag against
`Camera.fps` before counting it.

Steps 2 and 3 are the same for all four flights:

```bash
bash run_stereo_slam.sh
```

```bash
python3 src/stonefish_bluerov2/scripts/bluerov2_autonomous_controller.py
```

Start the controller only once `/bluerov2/map_points` is climbing.

### Flight 1 (seed 0)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine dvl_error:=a50 dvl_seed:=0 dr_reference:=true rviz:=false
```

```bash
bash record_run.sh slamref 0
```

### Flight 2 (seed 1)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine dvl_error:=a50 dvl_seed:=1 dr_reference:=true rviz:=false
```

```bash
bash record_run.sh slamref 1
```

### Flight 3 (seed 2)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine dvl_error:=a50 dvl_seed:=2 dr_reference:=true rviz:=false
```

```bash
bash record_run.sh slamref 2
```

### Flight 4 (circular alignment, seed 0)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine dvl_error:=a50 dvl_seed:=0 dr_reference:=false rviz:=false
```

```bash
bash record_run.sh slamcirc 0
```

`record_run.sh` refuses to start if the live DVL seed or scale does not match the
arm you named, or if SLAM is not publishing.

## What to paste back after each flight

- the **bag path** `record_run.sh` prints;
- any line from `record_run.sh` that starts `WARNING` or says it refused;
- the **last 40 lines of the bridge log** (the `slam_pose_bridge` node), and any line
  mentioning `reset`, `scale`, or `fit_resid`;
- from the Docker console, any `Loop detected` or `Merge` lines, or the log file name
  under `slam_atlas/` (it is saved there automatically).

Do not paste the whole controller log.

## What I will do with it

Per flight: confirm from the bag that the DVL model and seed are what the name says
(the 3x3 operator fit), classify any closure (`closure_events.py`), score the
reconstruction under all four navigation sources (`recon_batch.py`), and score
ORB-SLAM3's map (`slam_map_recon.py`). Then update Table [R] and Table [F] and
replace "two recordings" in the abstract, discussion and limitations.

## What would change the paper

- **All three `slamref` flights fuse worse than dead reckoning**, as the two
  post-fix recordings did: the claim stands on three seeds plus the two.
- **A flight where fusing SLAM helps or ties**: the claim becomes conditional,
  and the abstract, discussion and conclusion all change. Report it as it comes.
- **A wrong-face closure appears**: one more event for the closure tables.
- **The bridge misbehaves** (loses the alignment, publishes a covariance that
  stays at its floor): that is a finding about the bridge, not about SLAM; stop
  and send me the log before flying the next one.

## Not in these four

The gyro-drift sweep (5, 10, 15 degrees per hour) and the control structure
without the symmetry are separate, later runs. Neither is needed for the claim
these four test.

---

# Added 2026-10-21: three more DVL units (flights 5 to 7)

**Status.** Flights 1 to 3 are flown and scored (seeds 0, 1, 2; see the project
notes). Flight 4 (`slamcirc 0`) is still to fly. The target is a journal
(IEEE Journal of Oceanic Engineering), so the claim "fusing SLAM helps once dead
reckoning has drifted by more than about 0.2 m" needs more than three units.

**How the seeds were chosen, stated before flying.** A seed's DVL unit is a fixed
draw, reproducible offline (`eval/verify_dvl_model.py` reproduces it; the offline
numbers for seeds 0 to 2 matched the flown bags). Seeds 0 to 30 span total
misalignment 0.14 to 1.62 degrees and worst scale error 0.30 to 3.11 %. The three
flown units span 0.33 to 1.62 degrees but only 0.5 to 0.8 % scale error, so the
range is extended at both ends and along the scale axis:

| flight | seed | total misalignment | worst scale error | why this unit |
|---|---|---|---|---|
| 5 | 29 | 0.14 deg | 0.40 % | the cleanest unit in seeds 0 to 30: does dead reckoning win clearly? |
| 6 | 3 | 0.38 deg | 2.58 % | scale-dominated error, small misalignment |
| 7 | 26 | 1.05 deg | 3.05 % | large on both axes |

**Predictions, written before the flights** (so they can fail):
1. Seed 29: dead reckoning beats the fused filter in whole-run reconstruction RMS.
2. Seeds 3 and 26: the fused filter beats dead reckoning.
3. Any wrong-face closure is contained: the fused filter's RMS after the closure
   stays below 0.5 m.
Report each as held or failed, as in the earlier preregistrations. If seed 3 or 26
behaves unlike seeds 1 and 2, say so; it would mean scale error and misalignment
act differently on dead reckoning, and the "drift level" framing would need
splitting.

**Commands.** One terminal per step, fresh simulator each time, cool below about
90 C between flights (rviz off, rendering quality untouched).

Steps 2 and 3, same for every flight:

```bash
bash run_stereo_slam.sh
```

```bash
python3 src/stonefish_bluerov2/scripts/bluerov2_autonomous_controller.py
```

### Flight 5 (seed 29)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine dvl_error:=a50 dvl_seed:=29 dr_reference:=true rviz:=false
```

```bash
bash record_run.sh slamref 29
```

### Flight 6 (seed 3)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine dvl_error:=a50 dvl_seed:=3 dr_reference:=true rviz:=false
```

```bash
bash record_run.sh slamref 3
```

### Flight 7 (seed 26)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine dvl_error:=a50 dvl_seed:=26 dr_reference:=true rviz:=false
```

```bash
bash record_run.sh slamref 26
```

Paste back, per flight: the bag path, the bridge log, the SLAM Docker console and
any `WARNING` line from `record_run.sh`. Per-flight checks as before: DVL unit
match, message rates, closure classification, reconstruction scores.

## Not covered by flights 5 to 7

The gyro-drift sweep (5, 10, 15 degrees per hour) is a second axis of the same
regime map and would help the journal case, but it needs its own pre-registration
and scenario variants. It is a separate decision.

---

# Added 2026-10-21: the symmetry control (L4), time-boxed to today

Governed by `PREREG_symmetry_L0L4_20261021.md`. **This replaces the order of flights
5 to 7 above:** fly the three L4 flights first; flights 5 to 7 (L0, seeds 29, 3, 26)
come only if gate A passes. Flight 4 (`slamcirc 0`) is now optional.

Same routine as before: one terminal per step, fresh simulator each time, below about
90 C between flights, rviz off, rendering quality untouched. `scenario:=bluerov2_turbine_sym_l4`
is already installed (symlinked), so no build is needed.

Steps 2 and 3 are the same for every flight:

```bash
bash run_stereo_slam.sh
```

```bash
python3 src/stonefish_bluerov2/scripts/bluerov2_autonomous_controller.py
```

### L4 flight 1 (seed 0)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine_sym_l4 dvl_error:=a50 dvl_seed:=0 dr_reference:=true rviz:=false
```

```bash
bash record_run.sh symL4 0
```

### L4 flight 2 (seed 1)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine_sym_l4 dvl_error:=a50 dvl_seed:=1 dr_reference:=true rviz:=false
```

```bash
bash record_run.sh symL4 1
```

### L4 flight 3 (seed 2)

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine_sym_l4 dvl_error:=a50 dvl_seed:=2 dr_reference:=true rviz:=false
```

```bash
bash record_run.sh symL4 2
```

**Gate A** is read after these three (see the pre-registration). If it passes, the
remaining flights are L4 with seeds 29, 3, 26 (same commands with `dvl_seed:=` and
`record_run.sh symL4 <seed>`), then the L0 flights 5 to 7 above.

Paste back per flight: the bag path, the bridge log, the SLAM Docker console and any
`WARNING` line from `record_run.sh`. Checks per flight: DVL unit match, message rates,
orbit radius against the L0 flights (void rule), closure classification, scores.
