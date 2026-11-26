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
- 2026-11-20, block 2, stkL5, seed 2, `a50_stkL5_seed2_20261120_010207` (1201 s), console `orb_slam3_console_20261120_010159.log` mode=1. Void check: PASS (DVL unit seed 2, max diff 0.00037; orbit r 17.16 m vs stkL0 17.27, diff 0.11 < 1 m (checked after stkL0 seed 2 was flown); MISSION COMPLETE; recorder metadata written; scene checked by record_run.sh at record time). Outcome not examined.
- 2026-11-20, block 2, stkL0, seed 2, `a50_stkL0_seed2_20261120_134734` (1199 s), console `orb_slam3_console_20261120_134720.log` mode=1. Void check: PASS (DVL unit seed 2, max diff 0.00017; orbit r 17.27 m, the seed-2 reference; stkL5 17.16 within 1 m; MISSION COMPLETE; recorder metadata written; scene checked by record_run.sh at record time). Outcome not examined.
- 2026-11-21, block 2, vetoL0, seed 2, `a50_vetoL0_seed2_20261121_005903` (1179 s), console `orb_slam3_console_20261121_005850.log` mode=2 thresh=45.0. Void check: PASS (DVL unit seed 2, max diff 0.00036; orbit r 17.28 m vs stkL0 17.27, diff 0.01 < 1 m; MISSION COMPLETE; recorder metadata written; scene checked by record_run.sh at record time). Outcome not examined. Block 2 complete.
- 2026-11-21, block 3, stkL0, seed 29, `a50_stkL0_seed29_20261121_013920` (1230 s), console `orb_slam3_console_20261121_013334.log` mode=1 thresh=45.0. Void check: PASS (DVL unit seed 29, max diff 0.00030; orbit r 17.06 m, sets the seed-29 reference; MISSION COMPLETE; recorder metadata written; scene checked by record_run.sh at record time). Outcome not examined.
- 2026-11-23, block 3, vetoL0, seed 29, `a50_vetoL0_seed29_20261123_132730` (1295 s), console `orb_slam3_console_20261123_132718.log` mode=2 thresh=45.0. Void check: PASS (DVL unit seed 29, max diff 0.00052; orbit r 17.08 m vs stkL0 17.06, diff 0.02 < 1 m; MISSION COMPLETE; recorder metadata written; scene checked by record_run.sh at record time). Outcome not examined.
- 2026-11-23, block 3, stkL5, seed 29, `a50_stkL5_seed29_20261123_135721` (1275 s), console `orb_slam3_console_20261123_135712.log` mode=1 thresh=45.0. Void check: PASS (DVL unit seed 29, max diff 0.00034; orbit r 16.91 m vs stkL0 17.06, diff 0.15 < 1 m; MISSION COMPLETE; recorder metadata written; scene checked by record_run.sh at record time). Outcome not examined.
- 2026-11-23, CORRECTION, block 3: `a50_vetoL0_seed29_20261123_132730` and `a50_stkL5_seed29_20261123_135721` are VOID (re-flights 1 and 2 of the 4 allowed). Both ran with RViz on: the launch commands supplied for them omitted `rviz:=false` and the launch default is true, so they do not match the registered arms. Their PASS entries above were written without checking this. Found from the launch logs and decided with all outcomes sealed. The bags are kept and excluded from the analysis; both are re-flown with the same seed and arm. The launch logs of the other 10 flights show RViz off. Block 3 is incomplete until the re-flights pass.
- 2026-11-23, block 3, vetoL0, seed 29, `a50_vetoL0_seed29_20261123_203758` (1867 s; recorder left running about 12 min after MISSION COMPLETE), console `orb_slam3_console_20261123_203741.log` mode=2 thresh=45.0. Void check: PASS (RViz off per launch log; DVL unit seed 29, max diff 0.00036; orbit r 17.10 m vs stkL0 17.06, diff 0.04 < 1 m; MISSION COMPLETE; recorder metadata written; scene checked by record_run.sh at record time). Outcome not examined. Re-flight 1 of 4.
- 2026-11-24, block 3, stkL5, seed 29, `a50_stkL5_seed29_20261124_125956` (1177 s), console `orb_slam3_console_20261124_125947.log` mode=1 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine_sym_l5 per parser log and record_run.sh; DVL unit seed 29, max diff 0.00026; orbit r 17.00 m vs stkL0 17.06, diff 0.06 < 1 m; MISSION COMPLETE; recorder metadata written). A first SLAM start (`orb_slam3_console_20261124_125939.log`, 82 lines) ended before recording; the scorer's nearest-preceding-log rule picks the flight's log. Outcome not examined. Re-flight 2 of 4. Block 3 complete.
- 2026-11-24, block 4, vetoL0, seed 3, `a50_vetoL0_seed3_20261124_133412` (1309 s; recorder left running about 3 min after MISSION COMPLETE), console `orb_slam3_console_20261124_133358.log` mode=2 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine per parser log; DVL unit seed 3, max diff 0.00102; MISSION COMPLETE; recorder metadata written; orbit r 17.65 m vs stkL0 seed 3 17.62 m, diff 0.03 m < 1 m; settled 2026-11-25). Outcome not examined.
- 2026-11-24, block 4, stkL5, seed 3, `a50_stkL5_seed3_20261124_164458` (1375 s; recorder left running about 4 min after MISSION COMPLETE), console `orb_slam3_console_20261124_164434.log` mode=1 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine_sym_l5 per parser log and record_run.sh; DVL unit seed 3, max diff 0.00024; MISSION COMPLETE; recorder metadata written; orbit r 17.53 m vs stkL0 seed 3 17.62 m, diff 0.09 m < 1 m; settled 2026-11-25). Outcome not examined.
- 2026-11-25, block 4, stkL0, seed 3, `a50_stkL0_seed3_20261125_001434` (1144 s; MISSION COMPLETE 5.6 s before the recorder stopped), console `orb_slam3_console_20261125_001418.log` mode=1 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine per parser log; DVL unit seed 3, max diff 0.00078; MISSION COMPLETE; recorder metadata written; orbit r 17.62 m, the block 4 reference). Outcome not examined. Block 4 complete.
- 2026-11-25, block 5, stkL5, seed 26, `a50_stkL5_seed26_20261125_133032` (1414 s; recorder left running about 4.5 min after MISSION COMPLETE), console `orb_slam3_console_20261125_133017.log` mode=1 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine_sym_l5 per parser log; DVL unit seed 26, max diff 0.00014; MISSION COMPLETE; recorder metadata written; orbit r 17.76 m vs stkL0 seed 26 17.80 m, diff 0.04 m < 1 m; settled 2026-11-25). Outcome not examined.
- 2026-11-25, block 5, stkL0, seed 26, `a50_stkL0_seed26_20261125_173339` (1130 s; MISSION COMPLETE 4.7 s before the recorder stopped), console `orb_slam3_console_20261125_173323.log` mode=1 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine per parser log; DVL unit seed 26, max diff 0.00057; MISSION COMPLETE; recorder metadata written; orbit r 17.80 m, the block 5 reference). Outcome not examined.
- 2026-11-25, block 5, vetoL0, seed 26, `a50_vetoL0_seed26_20261125_203844` (1169 s; MISSION COMPLETE 27.1 s before the recorder stopped), console `orb_slam3_console_20261125_203830.log` mode=2 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine per parser log; DVL unit seed 26, max diff 0.00082; MISSION COMPLETE; recorder metadata written; orbit r 17.81 m vs stkL0 seed 26 17.80 m, diff 0.01 m < 1 m). Outcome not examined. Block 5 complete.
- 2026-11-26, block 6, stkL0, seed 5, NOT RECORDED, VOID (re-flight 3 of the 4 allowed). SLAM was started with the default `bluerov2_stereo.yaml` (`orb_slam3_console_20261126_012106.log` mode=2) in place of the arm's `bluerov2_stereo_log.yaml`. `record_run.sh stkL0 5` refused to record on the mode mismatch, and the mission was flown anyway with no recorder. No bag exists. The arm check failed, which is a void condition, so this counts against the re-flight cap even though nothing was recorded. stkL0 seed 5 is re-flown with mode=1.
- 2026-11-26, block 6, stkL0, seed 5, `a50_stkL0_seed5_20261126_022436` (1254 s; MISSION COMPLETE 126.8 s before the recorder stopped), console `orb_slam3_console_20261126_022422.log` mode=1 thresh=45.0. Void check: PASS (MISSION COMPLETE per controller log, settled 2026-11-26; RViz off per launch log; scene bluerov2_turbine per parser log; DVL unit seed 5, max diff 0.00103; recorder metadata written; orbit r 17.35 m, the block 6 reference). Outcome not examined. Re-flight 3 of 4 (replaces the unrecorded attempt above).
- 2026-11-26, block 6, vetoL0, seed 5, `a50_vetoL0_seed5_20261126_160632` (1158 s; MISSION COMPLETE 26.2 s before the recorder stopped), console `orb_slam3_console_20261126_160622.log` mode=2 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine per parser log; DVL unit seed 5, max diff 0.00086; MISSION COMPLETE per controller log; recorder metadata written; orbit r 17.43 m vs stkL0 seed 5 17.35 m, diff 0.08 m < 1 m). Outcome not examined.
- 2026-11-26, block 6, stkL5, seed 5, `a50_stkL5_seed5_20261126_170231` (1134 s; MISSION COMPLETE 4.0 s before the recorder stopped), console `orb_slam3_console_20261126_170218.log` mode=1 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine_sym_l5 per parser log; DVL unit seed 5, max diff 0.00063; MISSION COMPLETE per controller log; recorder metadata written; orbit r 17.28 m vs stkL0 seed 5 17.35 m, diff 0.07 m < 1 m). Outcome not examined. Block 6 complete.
- 2026-11-27, block 7, vetoL0, seed 7, `a50_vetoL0_seed7_20261127_000907` (1199 s; MISSION COMPLETE 63.5 s before the recorder stopped), console `orb_slam3_console_20261127_000853.log` mode=2 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine per parser log; DVL unit seed 7, max diff 0.00037; MISSION COMPLETE per controller log; recorder metadata written; orbit r 16.97 m vs stkL0 seed 7 16.97 m, diff 0.00 m < 1 m; settled 2026-11-27). Outcome not examined.
- 2026-11-27, block 7, stkL5, seed 7, `a50_stkL5_seed7_20261127_004122` (1184 s; MISSION COMPLETE 48.1 s before the recorder stopped), console `orb_slam3_console_20261127_004108.log` mode=1 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine_sym_l5 per parser log; DVL unit seed 7, max diff 0.00042; MISSION COMPLETE per controller log; recorder metadata written; orbit r 16.90 m vs stkL0 seed 7 16.97 m, diff 0.07 m < 1 m; settled 2026-11-27). Outcome not examined.
- 2026-11-27, block 7, stkL0, seed 7, `a50_stkL0_seed7_20261127_010854` (1276 s; MISSION COMPLETE 141.6 s before the recorder stopped), console `orb_slam3_console_20261127_010840.log` mode=1 thresh=45.0. Void check: PASS (RViz off per launch log; scene bluerov2_turbine per parser log; DVL unit seed 7, max diff 0.00043; MISSION COMPLETE per controller log; recorder metadata written; orbit r 16.97 m, the block 7 reference). Outcome not examined. Block 7 complete. All 24 flights flown.
