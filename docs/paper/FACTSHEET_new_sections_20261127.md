# Fact sheet for the revised manuscript (2026-11-27)

Every number for the new and corrected sections, where it comes from, and where it goes.
Doc is at rev 326 (2026-11-14); none of this is in it yet. Numbers only, no prose to paste.

Sources, all committed:
- R1 `docs/paper/results_tables.md` (Tables A-D), from `eval/results_tables.py`
- R2 `docs/paper/prereg_results.md` + `eval/figures/prereg_score.json`
- R3 `docs/paper/prereg_bridge_events.md` + `eval/figures/prereg_bridge_events.json`
- R4 `docs/paper/g3_replay_results.md` + `eval/figures/g3_replay.json`
- R5 `docs/paper/table7.md`
- P1 `docs/experiments/PREREG_veto_symmetry_20261116.md`
- P2 `docs/experiments/PREREG_g3_gyro_replay_20261127.md`
- N1 `docs/paper/NOVELTY_heading_check_20261117.md`

---

## 1. Methods additions

### 1a. Heading check (new subsection, near 3.5)

| fact | value | source |
|---|---|---|
| What it compares | the turn a loop, merge or relocalisation applies about the gravity axis, against the turn the vehicle's heading sensor measured between the same two keyframe times | `ORB_SLAM3/src/HeadingPrior.cc` |
| Threshold | 45 deg = half the structure's 90 deg symmetry angle | P1 arms table |
| Modes | 1 = log every decision, refuse none; 2 = refuse when the residual exceeds 45 deg | P1 |
| Where it acts | loops and merges: a refused match is reset before ORB-SLAM3 applies it. Relocalisation: the refused candidate is dropped and the next is tried | `LoopClosing.cc`, `Tracking.cc`; P1 analysis line |
| Why it needs an independent heading | it also covers merges and relocalisations, where stock stereo-inertial ORB-SLAM3 leaves yaw free; works without VI SLAM | N1 |
| Prior art to cite | ORB-SLAM3 stereo-inertial 20 deg loop yaw gate (code only, `LoopClosing.cc`); Kimera-RPGO / PCM odometry check (Mangelson 2018, already [35]); Doppelgangers (Cai et al., ICCV 2023) as the learned alternative; optionally visloc-rs PR #281 | N1 |
| Claim | the measurement and the in-flight test, NOT a new method | N1 |

### 1b. Bridge quarantine (in 3.3)

| fact | value | source |
|---|---|---|
| Trigger | the raw SLAM position jumps > 5 m between consecutive poses inside the same map (no map-point crash within 10 s; map already aligned) | `slam_pose_bridge.py` JUMP_THRESHOLD, CRASH_JUMP_WINDOW |
| Effect | the bridge stops publishing SLAM for the rest of the mission; the fused filter runs on DVL dead reckoning | same |
| Why not re-lock | replay of 39 bags: 15 had an in-map jump; all 9 re-locks after one were bad, median error 0.85 to 4.07 m | code comment, `eval/replay_bridge.py` |
| Armed only after the first alignment on a map | added 2026-11-16 | same |
| Bridge alignment | SLAM to dead reckoning, horizontal similarity (Sim(2)), refit every 5 s, each refit moves heading by 20 % of the change (EMA 0.2) | REFIT_INTERVAL, REFIT_EMA_ALPHA |

### 1c. Dead-reckoning filter depth (disclosure, in 3.3)

| fact | value | source |
|---|---|---|
| Fault | the SLAM-free reference filter's config named depth as its second pose input with no first; robot_localization stops at the first missing index, so depth was never fused | `ekf_dr.yaml` (unchanged since 2026-10-13) |
| Effect | only its height drifted (seed 2: -2.3 m by 1020 s). Horizontal position, every position RMS, and the bridge (horizontal alignment) are unaffected | memory note, recon_vs_position |
| Correction used | dead reckoning's height replaced by the depth sensor's (within 1 cm of truth) for every reconstruction | `eval/sonar_recon.py`, "DR DEPTH" |
| Not changed in the 24 new flights | the config was left as flown to avoid a mid-study change | P1 |

### 1d. Gyro replay (offline, new subsection)

| fact | value | source |
|---|---|---|
| Why | the simulated IMU yaw is truth + 0.1 deg white noise, -0.001 deg/h trend: a perfect compass. Both filters fuse it as absolute yaw. Next to steel a magnetometer can't be trusted, so heading is an integrated gyro | P2 "Why" |
| Model | yaw' = yaw + s(yaw - yaw0) + b t + angle random walk; roll, pitch unchanged; b, s drawn from the flight's DVL seed | P2 |
| Grades (bias sigma, scale sigma, ARW) | in-water MEMS (PRIMARY) 36 deg/h, 1 %, 0.24 deg/rt-h; tactical (ADIS16495) 0.8 deg/h, 0.1 % (assumed), 0.09; on-deck MEMS 360 deg/h, 1 %, 0.24 | `eval/match_gap.py` GRADES |
| Sources of grades | TDK ICM-20602 datasheet (tempco 0.01 dps/degC x 1 or 10 degC); ADIS16495-1 datasheet | same |
| Pipeline | real ekf_dr node, then the real bridge class against that DR, then the real fused ekf node; recorded bags played at 16x | P2 method |
| Fidelity, uncorrupted, alone | DR 1.8 mm median vs recording; fused 2.3 cm median; error vs truth fused 0.435 vs 0.424 m | P2 log |
| Fidelity under the 6-way load used | DR 4-9 mm, fused 0.4-1.5 cm median; error vs truth within 0.01 m of the flight in all six checked | P2 log |
| Limits | open loop (vehicle flies the recorded path); ORB-SLAM3 not re-run, so its heading-check decisions are the flown ones | P2 limits |

---

## 2. Results: the 24-flight pre-registered study (new section, after 4.5 or replacing 4.6)

### Design (P1)

| fact | value |
|---|---|
| Registered | 2026-11-16, before any flight |
| Arms | stkL0 (stock jacket, check logs only), vetoL0 (stock jacket, check refuses), stkL5 (every face different, check logs only) |
| Seeds | 8 DVL units: 0, 1, 2, 29, 3, 26, 5, 7; one flight per seed per arm; arm order rotated per block |
| Primary endpoint | mission had at least one accepted wrong-face correction (loop, merge or relocalisation) before RETURN_HOME |
| Tests | one-sided Fisher exact, mission as unit, alpha 0.05 each; Holm alongside; unscorable scored both ways |
| Power assumption | stock rate 0.59 gave 0.81 power for 4/8 vs 0/8 |
| Stage 1 | not pooled into the primary (the bridge and check changed); a pooled analysis is secondary |
| Voids | 3 of 4 allowed re-flights used: 2 flown with RViz on (launch default), 1 with the wrong SLAM config (recorder refused, nothing recorded). All decided with outcomes sealed |

### Table A, mission level (R1, R2)

| arm | wrong-face missions | p vs stkL0 | Holm |
|---|---|---|---|
| stkL0 | 2/8 (3/8 counting the unscorable) | reference | |
| vetoL0 | 0/8 | 0.23 (0.10) | 0.47 (0.20) |
| stkL5 | 0/8 | 0.23 (0.10) | 0.47 (0.20) |

- Both Q1 and Q2: **not detected at this sample size**. Never "no effect" (P1 wording).
- The unscorable one (stkL0 seed 29): pose-jump test fit residual 1.95 m; the keyframe test says same-place (positions 1.2 m apart, turn 1 deg). Strict reading is the better one.
- Why underpowered: stock came in at 2/8 = 25 %. Power of the design at that rate: 0.11; at the 9/20 = 45 % census rate: 0.52; at the registered 0.59: 0.81.
- Pooled with stage 1 (secondary, not primary): Q2 4/11 vs 0/11, p = 0.045; 5/11 vs 0/11, p = 0.018 (loose).
- No collisions in any of the 24 (shortest sonar return 1.42 m, stkL0 seed 29).

### Table B, decision level (R1, R2)

| truth class | decisions | residual | flagged | accepted |
|---|---|---|---|---|
| wrong-face | 5 | 86.0 to 90.1 deg | 5 | 0 |
| same-place | 27 | 0.0 to 1.8 deg | 0 | 27 |

- Hits 5/5, false alarms 0/27, over every loop and relocalisation in the 24 flights.
- Gap between the classes: 84 deg; threshold 45 deg sits in the middle.
- **vetoL0 had wrong-face candidates in 3 of 8 missions** (seeds 2, 3, 29, all before the return), all three refused. Stock had them in 2 of 8, both accepted (would_veto). So the stock jacket produced wrong-face candidates in 5 of 16 missions; with the check on, none got through.
- stkL5: no wrong-face candidate in any mission, so the registered mechanism prediction (any L5 wrong-face turn between faces 0-1 or 1-2) was not tested.
- stkL0's two wrong-face closures came at 804 s (seed 2) and 1025 s (seed 1); RETURN_HOME at 1092 and 1079 s. All five wrong-face candidates: chords 24.2-24.9 m, turns 88.6-90.6 deg.

### Table C, bridge quarantine in the loop (R1, R3)

| fact | value |
|---|---|
| Fired | in both stkL0 wrong-face missions (seeds 1, 2), 2 jump resets each; in none of the other 22 |
| Fused error in those two | median 0.23 and 0.38 m; dead reckoning 0.17 and 0.34 m on the same missions |
| Compare old bridge | before the quarantine, a re-lock after a wrong-face closure put the fused estimate up to 4.7 m off (abstract) |
| Odd one out | stkL0 seed 29: 5 locks, 118 rejected fits, fused median 3.82 m vs DR 0.07 m; no wrong-face closure. Map instability, not aliasing |
| Most gate activity | after RETURN_HOME (the surface leg) |

---

## 3. Results: does fusing SLAM help? (rewrite of 4.4 + new G3 part)

### 3a. Table 7 / Figure 15, corrected (R5)

| flight | DR (corrected) | fused | fused vs DR |
|---|---|---|---|
| stock seed 0 | 9.7 cm | 15.9 cm | +63 % |
| stock seed 1 | 15.9 cm | 17.3 cm | +9 % |
| stock seed 2 | 20.1 cm | 23.4 cm | +16 % |
| L0 seed 0 | 9.4 cm | 1.18 m | +1157 % |
| L0 seed 1 | 15.2 cm | 19.2 cm | +26 % |
| L0 seed 2 | 19.6 cm | 42.1 cm | +115 % |
| earlier bridge seed 1 | 15.2 cm | 45.0 cm | +196 % |

- Fusing worsens the sonar model in all 7. The old "19-50 % gain" was the depth fault (1c).
- The "two measures disagree" paragraph is resolved: position RMS and sonar model agree.
- Truth floor 7.3-7.4 cm; trust-SLAM after closure 2.91-19.1 m (unchanged).
- Figure 8 caption: DR 9 cm (was 10), 99 % within 25 cm (was 98 %). Figure 9: unchanged (99 %).

### 3b. The 24 flights, compass heading (R2 secondary; Table D row 1)

| fact | value |
|---|---|
| Fused beats DR on median error | 1 of 24 |
| at mission end | 0 of 24 |
| Median over flights | DR 0.31 m, fused 0.40 m |
| End error | fused 0.42-4.75 m, DR 0.01-0.18 m |

### 3c. Gyro replay (Table D, Figure "fig_g3_mechanism") (R1, R4, P2)

| heading | median bias | DR' | fused' | fused' better (median / end) | p (median / end) |
|---|---|---|---|---|---|
| compass (flown) | - | 0.31 m | 0.40 m | 1/24 / 0/24 | not tested |
| tactical gyro | 0.3 deg/h | 0.29 m | 0.39 m | 0/24 / 0/24 | 1.00 / 1.00 |
| in-water MEMS (**registered primary**) | 13.3 deg/h | 0.67 m | 0.71 m | 9/24 / 3/24 | 0.77 / 1.00 |
| on-deck MEMS | 132.7 deg/h | 8.11 m | 7.75 m | 15/24 / 12/24 | 0.08 / 0.25 |

- Registered criterion (p < 0.05 AND better in > half): **not met**. Report as "not shown".
- Worst primary unit (73.5 deg/h): DR' 6.5 m vs fused' 6.3 m, a 4 % difference.
- Mechanism (the figure): fused/DR ratio is 2-60x when DR is good and goes to ~1.0 when DR is bad; never meaningfully below 1. The bridge refits SLAM to DR every 5 s, so its output carries DR's drift. Architectural, not a simulator artefact.
- Outliers in the figure: all stkL0 seed 29 (fused 3.5-3.8 m), the map-instability flight.
- Future work (Discussion): a bridge that locks its alignment while DR is still good and stops refitting to it. Not tested.

---

## 4. Text that is now false (search the Doc)

| section | old | new |
|---|---|---|
| Abstract, Conclusion | "offers no remedy" / "no remedy tested in the loop" | the check was flown: 5/5, 0/27; mission level not detected |
| 3.7, 4.4, 5.8, 6.6, App. A | seeds 29, 3, 26 "not flown" | flown in the 2026-11-16 study (8 seeds, 3 arms) |
| 4.4 (x2), 5.1 | "19 to 50 %" gain | worse in all 7 (3a) |
| 4.4 | "10, 29 and 47 cm" | 10, 16 and 20 cm |
| 4.3 Fig. 8 caption | 98 %, 10 cm | 99 %, 9 cm |
| 6.6 | "the gyro-drift experiment ... was void" | the gyro replay (3c) |
| App. B | stage 2 "was not flown, as the registration required" | keep, and add: a separate registration (2026-11-16), written after stage 1 stopped, flew those seeds; stage 1 is not pooled into its primary |
| 4.6 | offline checks, "none of this was run in the loop" | the heading check now ran in the loop; 4.6 can shrink to the chord/size checks |

## 5. Gaps, final status

| gap | status | one-line evidence |
|---|---|---|
| G1 remedy in the loop | partly closed | check: 5/5 hits, 0/27 false alarms, 3 refused in vetoL0; mission level not detected (2/8 vs 0/8) |
| G2 symmetry as cause | open | 2/8 vs 0/8, p = 0.23; pooled secondary 0.045 |
| G3 SLAM helps | closed, negative | never, with any heading source; bridge inherits DR drift |
| G4 one 20 m scene | open | deep scene attempted, stereo SLAM failed; not in the paper |
| G5 bridge fixes in the loop | closed | quarantine 2/2 wrong-face missions, 0/22 others |
| G6 one SLAM system | open | stereo-inertial attempt ran away in replay; not in the paper |
