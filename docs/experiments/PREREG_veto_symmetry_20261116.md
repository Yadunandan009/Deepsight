# Pre-registration: heading veto in flight, and symmetry control stage 2. Written 2026-11-16, before any flight

## Questions

- **Q1 (manuscript gap G1).** With the heading check enforced, do wrong-face loop corrections stop
  happening in flight?
- **Q2 (gap G2).** Does breaking the jacket's four-fold symmetry (scene L5) make them rarer?
- **Secondary (gaps G3, G5).** In the same flights: is the fused estimate better or worse than
  dead reckoning, and how often do the bridge's gate and quarantine act?

## Disclosed history

- **Stage 1 of the symmetry control** (`PREREG_symmetry_L0L5_20261021.md`) ended at its Gate B on
  2026-10-22: stock 2/3, L5 0/3, p = 0.20. This document resumes it, after submission, to close G2.
- **What changed since stage 1:**
  - the bridge's jump quarantine (2026-10-25) and its first-alignment arming (2026-11-16);
  - the heading check, which stage 1 did not have (Mode 0).
- **Pooling.** Stage 1 is therefore not pooled into the primary tests here. A pooled analysis is
  reported as secondary.
- **Historical rate.** 10/17 missions had a wrong-face correction, 2026-09 to 2026-10.
- **Offline evidence for the check.** It separated the one in-flight wrong-face loop (residual
  90.5 deg) from the correct one (0.2 deg) in the log-only flight `a50_hdglog_seed0_20261107_184202`.

## Arms

All arms: `dvl_error:=a50`, `dr_reference:=true`, `rviz:=false`, the current controller and bridge,
stereo ORB-SLAM3 image `orb-slam3-humble:bluerov2-heading`.

| arm | scene | SLAM settings | Heading.Mode |
|---|---|---|---|
| `stkL0` | `bluerov2_turbine` | `bluerov2_stereo_log.yaml` | 1: log every decision, refuse none (stock behaviour) |
| `vetoL0` | `bluerov2_turbine` | `bluerov2_stereo.yaml` | 2: refuse a match whose turn disagrees with the IMU by > 45 deg |
| `stkL5` | `bluerov2_turbine_sym_l5` | `bluerov2_stereo_log.yaml` | 1 |

**Why Mode 1 for the stock arms.** It changes no decision, and it prints the heading residual of
every loop, merge and relocalisation. So every stock wrong-face correction is identified twice,
independently:
- by truth (`eval/closure_events.py`);
- by the check (`decision=would_veto`).

**Verified at record time.** `record_run.sh` refuses to record unless:
- the running simulator's `.scn` matches the arm;
- the SLAM console's `[HEADING] config mode=` line matches the arm.

## Design

- **Seeds.** 8 DVL seeds: 0, 1, 2, 29, 3, 26, 5, 7. Each seed is flown once in each arm (a block of three).
- **Run order.** Within block k (k = 0 to 7), the arm order is rotated so that no arm is always
  first or last in a session:

  | k mod 3 | order |
  |---|---|
  | 0 | stkL0, vetoL0, stkL5 |
  | 1 | vetoL0, stkL5, stkL0 |
  | 2 | stkL5, stkL0, vetoL0 |

- **Size.** 24 flights.
- **Sample-size reasoning** (one-sided Fisher exact, alpha 0.05, other arm 0/8):
  - stock needs at least 4/8 (p = 0.038);
  - at the historical rate 0.59 that happens with probability 0.81;
  - with 6 per arm it would be 0.52, which is why 8.

## Endpoints

**Primary, per mission (both questions).** Did the mission have at least one *accepted* wrong-face
correction before RETURN_HOME?
- "Accepted" covers a loop closure, a map merge or a relocalisation.
- Wrong-face is classified from truth by `eval/closure_events.py` with the mission's console log.
- No correction at all counts as "no".
- Unscorable events are scored both ways. A conclusion must hold both ways, or it is reported as unresolved.

**Tests.**
- Q1: stkL0 against vetoL0.
- Q2: stkL0 against stkL5.
- Both are one-sided Fisher exact at alpha 0.05, with the mission as the unit.
- They are separate claims about separate gaps, so each is tested at 0.05. Holm-adjusted p-values
  are reported alongside.
- **A non-significant result is reported as "not detected at this sample size", never as "no effect".**

**Q1, event level (descriptive, reported in full).**
- In vetoL0: every refused match, with its residual and its truth class, and every accepted match,
  with the same.
- The check works as intended if:
  - every refused match is wrong-face by truth;
  - no accepted match is.
- In stkL0: the same two counts for `would_veto`. These give the check's hit and false-alarm
  counts on a fresh sample.

**Secondary, per mission, all arms.**
- Horizontal error of the fused estimate (`/bluerov2/odometry/filtered`) and of the SLAM-free
  reference (`/bluerov2/odometry/dr`) against truth:
  - median over the mission;
  - at mission end;
  - and whether the fused estimate beats the reference (G3).
- Bridge events (G5): gate rejections, quarantines, locks.
- Collisions; end distance from home.

**Prediction on mechanism (from stage 1, unchanged).** Any wrong-face correction in L5 should turn
between the face pairs that are still most alike (0-1 at 0.751, 1-2 at 0.787). A turn between 1-3,
2-3 or 0-3 counts against symmetry as the cause.

## Voids and stop rules

**Void and re-flown, same seed and arm, at most 4 re-flights in total:**
- SLAM never initialises;
- the recorder or console log fails;
- the DVL seed or arm check fails;
- the mission does not reach RETURN_HOME for a reason outside navigation (crash of a node, the user
  stopping it);
- the orbit radius (median 150 to 900 s, truth) differs from the same seed's stkL0 flight by more than 1 m.

**A collision is not a void.** It is an outcome. It is reported per arm, and the mission is scored
on the events before it.

**No looking at outcomes by arm until the last block.** Each flight is checked only for the void
conditions above.

**If time runs out** (deadline 2026-11-25):
- the analysis uses all complete blocks (all three arms of a seed);
- the number reached is stated;
- incomplete blocks are reported but not tested.

## Commands (per flight)

```bash
# simulator: scene per arm
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine        dvl_error:=a50 dvl_seed:=<SEED> dr_reference:=true rviz:=false   # stkL0, vetoL0
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine_sym_l5 dvl_error:=a50 dvl_seed:=<SEED> dr_reference:=true rviz:=false   # stkL5
# SLAM: settings per arm
bash run_stereo_slam.sh bluerov2_stereo_log.yaml   # stkL0, stkL5
bash run_stereo_slam.sh bluerov2_stereo.yaml       # vetoL0
# record BEFORE the controller, then fly
bash record_run.sh <ARM> <SEED>
python3 src/stonefish_bluerov2/scripts/bluerov2_autonomous_controller.py
```

## Results log

(one line per flight: date, block, arm, seed, bag, void check)
- 2026-11-17, block 0, stkL0, seed 0, `a50_stkL0_seed0_20261117_131341` (1270 s), console `orb_slam3_console_20261117_131327.log` mode=1. Void check: PASS (DVL unit seed 0, max diff 0.00066; orbit r 17.13 m, seed-0 history 16.97-17.15; MISSION COMPLETE; recorder metadata written). Outcome not examined.
- 2026-11-17, block 0, vetoL0, seed 0, `a50_vetoL0_seed0_20261117_134842` (1366 s), console `orb_slam3_console_20261117_134833.log` mode=2. Void check: PASS (DVL unit seed 0, max diff 0.00026; orbit r 17.29 m vs stkL0 17.13, diff 0.16 < 1 m; MISSION COMPLETE; recorder metadata written). Outcome not examined.
- 2026-11-17, block 0, stkL5, seed 0, `a50_stkL5_seed0_20261117_174745` (1916 s recorded; mission complete at ~1127 s, idle at surface after), console `orb_slam3_console_20261117_174736.log` mode=1. Void check: PASS (DVL unit seed 0, max diff 0.00042; orbit r 17.07 m vs stkL0 17.13, diff 0.06 < 1 m; MISSION COMPLETE; recorder metadata written). Outcome not examined. Block 0 complete.
- 2026-11-19, block 1, vetoL0, seed 1, `a50_vetoL0_seed1_20261119_151429` (1125 s), console `orb_slam3_console_20261119_151415.log` mode=2. Void check: PASS (DVL unit seed 1, max diff 0.00021; orbit r 16.89 m vs stkL0 16.97, diff 0.08 < 1 m (checked after stkL0 seed 1 was flown); MISSION COMPLETE; recorder metadata written). Outcome not examined.
- 2026-11-19, block 1, stkL5, seed 1, `a50_stkL5_seed1_20261119_172622` (1212 s), console `orb_slam3_console_20261119_172558.log` mode=1. Void check: PASS (DVL unit seed 1, max diff 0.00031; orbit r 16.92 m vs stkL0 16.97, diff 0.05 < 1 m (checked after stkL0 seed 1 was flown); MISSION COMPLETE; recorder metadata written; scene checked by record_run.sh at record time). Outcome not examined.
- 2026-11-19, block 1, stkL0, seed 1, `a50_stkL0_seed1_20261119_212200` (1284 s), console `orb_slam3_console_20261119_212151.log` mode=1. Void check: PASS (DVL unit seed 1, max diff 0.00047; orbit r 16.97 m, the seed-1 reference; vetoL0 16.89 and stkL5 16.92 both within 1 m; MISSION COMPLETE; recorder metadata written; scene checked by record_run.sh at record time). Outcome not examined. Block 1 complete.
