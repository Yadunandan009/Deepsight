# Pre-registration v2: the adaptive consistency gate, offline choice and in-loop test

Written 2026-10-26, after `PREREG_adaptive_gate_offline.md` (v1) was run and failed. v1 is closed; nothing here changes its outcome.

## What is different, and why it is not a fresh test

v1 judged gates by the share of poses more than 1 m from the truth. That line sat too close to the 1 m gate width: a pose 1 m from dead reckoning can be up to about 1.7 m from the truth, so no gate near 1 m could pass. v2 changes two things:

1. **The harm measured** is a pose more than **2 m** from the truth (metres wrong, the failure that matters for standoff), plus the worst published error per bag.
2. **The data** are the bags flown with the SLAM settings in use now: `Camera.fps 12` and a 0.145 m stereo baseline, as printed in each run's ORB-SLAM3 console log.

Both choices were made **after** seeing v1's per-bag results, and the replay is deterministic, so the offline rerun below is not an independent test: it fixes the criteria and selects one gate. **The confirmatory test is the in-loop flight**, whose pass line is fixed here, before it is flown.

## Part A: offline selection

**Data.** The 17 bags holding `/bluerov2/robot_pose_slam` and `/bluerov2/odometry/dr` whose console log shows FPS 12 and baseline 0.145:

`a50_slamref12_seed0_20261015_180532`, `a50_slamned_seed0_20261016_160033`, `a50_slamdrift_seed0_20261016_190745`, `a50_slamref_seed0_20261020_164933`, `a50_slamref_seed1_20261020_172618`, `a50_slamref_seed2_20261020_200356`, `a50_symL4_seed0_20261021_010852`, `a50_symL4_seed1_20261021_172653`, `a50_risefix_seed0_20261021_205753`, `a50_symL5_seed0_20261021_215607`, `a50_ctlL0_seed1_20261021_234743`, `a50_symL5_seed1_20261022_124631`, `a50_ctlL0_seed2_20261022_175232`, `a50_symL5_seed2_20261022_190833`, `a50_oldbridge_seed1_20261022_200605`, `a50_imgL0_seed1_20261025_132041`, `a50_qL0_seed1_20261026_194426`.

Excluded: the four bags of 14 October (FPS 30) and `a50_slamb30_seed0_20261015_204105` (baseline 0.30). They are still replayed and reported, but do not decide anything.

**Method.** As v1: the real bridge, `jump_policy:=quarantine`, `slam_frame:=enu`, the same five gates (base 5 m; A 1.0 m; B 1.0 m with probation; C 1.5 m with probation; D 1.0 m + 0.003/m with probation).

**Pass line.** A gate passes if, over the 17 bags:

- no published pose is more than 2 m from the truth, in any bag; and
- it publishes at least 97 % of the poses within 1 m of the truth that base publishes.

**Choice.** Of the passing gates, the one keeping the most poses within 1 m goes to the flight; on a tie, the one with fewer settings. If none passes, nothing is flown.

## Part B: in-loop test (confirmatory)

**Flights.** Three stock missions, `scenario:=bluerov2_turbine dvl_error:=a50 dr_reference:=true jump_policy:=quarantine adaptive_gate:=true` with the chosen gate's parameters, `dvl_seed` 0, 1 and 2, one each, recorded as `gate<ID>L0`. The package is rebuilt first (`colcon build --packages-select stonefish_bluerov2`) so `consistency_gate.py` is installed. A flight that does not reach `MISSION COMPLETE` is reported and re-flown once.

**Pass line, per flight, against ground truth.** The gate passes the in-loop test only if all three flights meet all three:

1. the primary filter's horizontal error never exceeds 2.0 m;
2. its final error is within 0.5 m of the SLAM-free filter's final error;
3. the vehicle ends within 2.5 m of its start point.

**Read-out, whatever the result.** Per flight: the primary filter's maximum, median and final error; the same for dead reckoning; poses the bridge published, refused and withheld on probation; every loop closure with its class (`closure_events.py`) and whether the quarantine fired.

**What a pass would and would not mean.** That on one lap, with the quarantine on, a ~1 m gate keeps SLAM from pulling the vehicle metres off course. Not that SLAM improves navigation: on one lap dead reckoning is good to under 0.7 m, and a 1 m gate around it cannot make SLAM better than dead reckoning. That needs a longer mission, where dead reckoning drifts.

## Outcome A

Run 2026-10-26, `python3 eval/gate_offline.py --v2` (`eval/figures/gate_offline_v2.json`). The 17 registered bags, pooled:

| gate | poses > 2 m | poses > 1 m | poses ≤ 1 m kept | worst error |
|---|---|---|---|---|
| base (5 m) | 442 | 3,291 | 100 % | 5.00 m |
| **A (1.0 m)** | **0** | 355 | **98.7 %** | **1.25 m** |
| B (1.0 m, probation) | 0 | 350 | 95.0 % | 1.25 m |
| C (1.5 m, probation) | 0 | 2,193 | 96.8 % | 1.61 m |
| D (1.0 m + 0.003/m, probation) | 0 | 2,115 | 96.6 % | 1.61 m |

**A passes and goes to the in-loop test.** All four candidates removed every pose more than 2 m wrong; only A kept at least 97 % of the poses within 1 m. Probation (B, C, D) withheld good poses without removing bad ones that A missed (B: 350 poses over 1 m against A's 355).

The five excluded bags, for the record: base published 1,709 poses more than 2 m wrong there (worst 3.92 m); every candidate published none (A worst 1.44 m).

Gate A in the launch: `adaptive_gate:=true gate_floor:=1.0 gate_k_drift:=0.0 gate_probation_poses:=0 gate_probation_spread:=0.0`.

## Outcome B

**Flight 1, seed 0** (2026-11-02, bag `a50_gateAL0_seed0_20261102_204717`, mission complete at 1,142 s into the bag). Gate A was live (refusals logged at "> 1.00m"). One loop closure, benign (0.18 m at 1,038 s); no quarantine.

| | max | median | at MISSION COMPLETE |
|---|---|---|---|
| primary filter error | 0.95 m | 0.25 m | 0.93 m |
| SLAM-free filter error | 0.19 m | 0.09 m | 0.02 m |
| published SLAM pose error | 1.02 m | 0.31 m | 0.98 m |

Against the pass line, taking "final" and "ends" at MISSION COMPLETE, when the vehicle declared itself home:

1. primary filter never above 2.0 m: **pass** (0.95 m);
2. final error within 0.5 m of the SLAM-free filter's: **fail** (0.93 m against 0.02 m);
3. ends within 2.5 m of start: **fail** (2.85 m).

Read at the end of the recording instead (183 s later, at the surface), 2 passes (0.26 against 0.01 m) and 3 still fails (2.66 m). The registration did not say which instant "final" is; MISSION COMPLETE is the one that matters for the vehicle, and criterion 3 fails either way.

**Gate A fails the in-loop test.** The rule needs all three flights to pass all three criteria, so seeds 1 and 2 cannot change the verdict; flying them would only be descriptive.

What happened: the primary filter followed dead reckoning to within about 0.5 m for most of the mission (0.07 m at 200 s, 0.15 m at 800 s), then SLAM poses during RISE pulled it to 0.61 m by 1,050 s and 0.95 m by 1,100 s, each pose inside the 1 m gate. At the start of RETURN_HOME the poses moved past 1 m and were refused; after 40 refusals the bridge re-aligned from scratch (1,113 s), on a fit at the scale bound (theta +13.2 deg, scale 1.250, residual 0.45 m). The filter stayed 0.93-0.95 m off through the return, so the controller stopped 2 m from where it thought home was and 2.85 m from where home is. The gate did what a 1 m gate can do: the worst error was 0.95 m, against 4.4-5.0 m in the 5 m-gate flights of 21-22 October. It cannot make SLAM better than dead reckoning, and on this lap dead reckoning was 0.02 m off at the end.
