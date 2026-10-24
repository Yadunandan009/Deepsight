# Perceptual aliasing in stereo ORB-SLAM3 around a four-fold symmetric subsea structure: characterisation, cost, and a controlled test

*Working draft, 2026-10-24. Simulation study. Target: IEEE Journal of Oceanic Engineering (author's choice; format not applied yet).*
*All flights are in (last 2026-10-22). Figures are numbered in order of first citation. The status notes and the checklist at the end are not part of the manuscript.*

---

## Abstract

Inspecting an offshore jacket without a position fix needs navigation good enough to hold a standoff from the steel. We study this in simulation: a BlueROV2 Heavy orbits a four-fold symmetric jacket using a Doppler velocity log with datasheet-grounded systematic error, an inertial unit and a depth sensor, with stock stereo ORB-SLAM3 fused through a loosely coupled bridge. On one 130 m lap, dead reckoning alone holds position to 0.1 to 0.5 m. Stock ORB-SLAM3 adds a characteristic error: 9 of 20 complete stock missions contained a loop closure that re-posed the estimate by 90 degrees (once 180) about the structure's axis, onto a different face, mostly at the seabed end of the structure. A vehicle that followed SLAM through such a closure would rebuild the structure to 2.9 to 19 m RMS from its CAD model. Our bridge instead discards its alignment and goes silent; when it re-locked mid-mission, its first poses were 1.2 to 2.9 m wrong, its 5 m consistency gate admitted them, and the fused estimate ended up to 4.7 m wrong while dead reckoning stayed within 0.6 m. In a pre-registered paired test of symmetry as the cause, 2 of 3 stock missions and 0 of 3 missions on a variant with every face different had a wrong-face closure (exact p = 0.20, not significant; the registered stop rule ended the test). The study is simulated, in one scene, with one SLAM system and small samples, and it offers no remedy.

---

## 1. Introduction

Inspection of subsea infrastructure, including the jacket foundations of offshore wind turbines, has relied on remotely operated vehicles flown from a support ship [1], [2]. Operation and maintenance is up to 30 % of the cost of energy for offshore wind, and the EU project ATLANTIS reported that robotic inspection and maintenance could cut that cost by up to 50 % [1]; that is the project's own result, not an independent study. An autonomous vehicle that surveys the structure itself would reduce the need for the ship and the pilot, but only if it can navigate on its own. An inspection trajectory is not a transit. The vehicle holds a standoff of some metres from steel, face by face, so a position error that is harmless in transit becomes a collision, or a sensor pointed at the wrong face.

Below the surface the vehicle cannot be told where it is. Acoustic positioning exists but restores a surface vessel or a seabed array [3], [4]. Two onboard sources remain. A Doppler velocity log, integrated with attitude from an inertial unit and depth from a pressure sensor, gives dead reckoning whose horizontal error is unbounded and grows with distance, because its dominant terms (a scale-factor error and a misalignment between the instrument and the vehicle) are fixed properties of the installed unit and not white noise [46], [47]. These three are the usual proprioceptive sensors for underwater SLAM [43]. Visual SLAM can in principle bound that error: a vehicle that recognises a place it has already mapped closes a loop and redistributes the drift.

That argument has a weak point for one asset class. Jacket foundations are lattices of tubular steel with three, four or more legs. A four-legged jacket with levels of X-braces, such as the UpWind reference jacket for a 5 MW turbine in 50 m of water [45], is close to four-fold symmetric about the vertical axis in its primary steel, although boat landings, J-tubes, anodes, cables and ladders, which that reference model leaves out [45], break the symmetry by amounts that vary (we do not resolve here how much they matter). If the four faces are interchangeable, a place-recognition system can match a view of one face to an earlier view of another. Geometric verification does not help, because when two places are congruent the consistent transform exists. The result would be a loop closure that re-poses the whole estimate by a symmetry operation of the structure.

This paper asks four questions about that failure in a simulated inspection mission.

1. **Does it happen, and what does it look like?** We record every loop closure that stock stereo ORB-SLAM3 makes and classify each by the correction it imposes about the structure's axis.
2. **What does it cost?** We score the damage on position, on the inspection product (a sonar reconstruction of the structure against its CAD model), and on where the vehicle ends up.
3. **Is the symmetry the cause?** The geometric signature is circumstantial. We pre-registered a paired comparison between the stock structure and a variant in which every face differs, with the symmetry measured from the CAD, and report the result and a first control that failed by design.
4. **What does the fusion architecture do with it?** A loosely coupled bridge decides how far a wrong SLAM pose can pull the navigation filter. We report what ours did, including a ceiling set by its consistency gate that we did not design.

**Terms.** The failure studied is perceptual aliasing in loop closure: the recogniser matches the view of one face to the view of another. We call such a closure *wrong-face* (the classifier's label is *face-aliased*) and a closure that corrects drift at the true place *benign*.

**Contributions.** Narrow, and stated with their status.

- **C1. A characterisation** of wrong-face loop closures by stock ORB-SLAM3 on a four-fold symmetric jacket: a classification rule fixed before most of the data, each large closure checked against the ground-truth trajectory (the estimate after it lands on a place the vehicle really visited, not the one it was at), a census over all recorded missions, and a description of where in the mission the closures fall. *Done.*
- **C2. A controlled, pre-registered test of the cause.** The symmetry is manipulated through the structure's CAD and measured as a pair score; the design is paired by DVL unit and tested with an exact test. One control that left two faces identical is reported as a failed design. *Stage 1 finished: 2 of 3 stock against 0 of 3 variant missions (p = 0.20); stopped by the registered rule.*
- **C3. A cost accounting at three levels**: position, the reconstructed structure, and the end-of-mission state, with a finding about the fusion bridge: after a wrong-face closure it goes silent, and when it re-locks mid-mission its first poses are metres wrong, its 5 m consistency gate admits them, and the primary filter cannot recover. *Done on the recorded flights; the analysis is offline and was not registered.*
- **C4. Measurement practice** for this kind of study: a DVL error model with per-run verification of the realised unit, a SLAM-free reference filter that makes the alignment falsifiable, the diagnosis of two defects (a circular alignment and an unconverted frame convention) that made a discrete failure look like continuous drift, and the habit of checking labels against data. *Done.*

**What is not claimed.** We do not offer a remedy that has been run in the loop. We do not show that SLAM never helps. We do not show the results hold on real data, in turbid water, on a real structure, or for another SLAM system. We do not claim the wrong-face rate is a property of the structure alone: the per-mission rate is uncertain (9 of 20 complete stock missions, Section 6.3), the stock baseline in the control was not near-certain, and we have not shown why the wrong-face closures concentrate at the seabed end of the structure (Section 4.2b).

Section 2 positions the work. Section 3 describes the testbed, the bridge, the classification rule and the control. Section 4 reports results. Section 5 discusses them, Section 6 collects the limitations, Section 7 concludes. Appendices list the data, the deviations from the pre-registrations, and the commands that reproduce every number.

---

## 2. Related work

**Dead reckoning error.** The way horizontal error grows matters for how it should be simulated. White-noise velocity error gives a random walk that grows with the square root of time. Calibration studies of real installations name the DVL's scale factor and its misalignment to the inertial unit as the key factors that limit dead-reckoning accuracy [46], and the misalignment is calibrated in situ for that reason [47]; both are fixed for a given installation, so they integrate with distance and do not average out. Scale errors of 0.1 to 0.7 % were estimated on four INS/DVL systems in [46], and the long-term accuracy quoted for the Water Linked DVL A50 is 1.01 % (standard version) [5], which is the order of the scale term we model. A simulator that omits them overstates dead reckoning. We model both, drawn once per run from a seed (Section 3.2).

**Visual SLAM and its underwater difficulties.** ORB-SLAM3 [7] is a feature-based system with a multi-map (Atlas) representation: when tracking is lost it starts a new map and may merge maps when it recognises a place across them. Underwater imaging is hard for physical reasons (poor visibility, changing light, water-induced distortion) [17], [18], and multi-sensor systems add sonar and inertial data [21]. We use an unmodified stereo front end deliberately, because the claim concerns a property of the inspection target and should apply to any appearance- or geometry-based place recogniser.

**Place recognition, aliasing and false closures.** Appearance-based place recognition [11], [23] fails when distinct places contain the same visual words; Lowry et al. survey this as perceptual aliasing [8]. Geometric verification by RANSAC [42] or joint compatibility [9] is the standard defence and fails for congruent places. Rejecting false closures after they are proposed is an established problem: switchable constraints [33], max-mixtures [34], pairwise consistency [35]. ROVER [41] verifies each candidate closure against the front-end odometry trajectory in repetitive environments, close to a check against dead reckoning; its authors note that large odometry drift can mislead it, as we find for our plain checks (Section 4.6). So the plain consistency check is prior art here, not a contribution.

**Symmetry in SLAM.** Object SLAM handles symmetric objects by keeping several pose hypotheses [37], [38], by using the camera pose as a prior for symmetric keypoints [36], or by discarding ambiguous pose parameters [39]; localisation on symmetric maps keeps every equally probable pose alive [40]. The closest inspection-specific work we found is a sonar-based AUV localiser that meets the same problem: symmetric assets produce near-identical sonar observations from opposing views, and it adds a particle filter and a place-recognition module to resolve the ambiguity [44]. It addresses localisation with a prior map, not loop closure in a SLAM map. The idea that symmetry makes a measurement ambiguous and the ambiguity should be carried is not new. In a web search that was not a systematic review we did not find a measurement of how often and how badly camera loop closure fails to the wrong face on a jacket-type structure, nor a controlled manipulation of the structure's symmetry to test the cause. That is the narrow space this paper occupies, and "not found" is not "does not exist".

**Evaluation practice.** Trajectory metrics align an estimate to an independent reference and then measure the difference [27], [28]. A navigation solution anchored to the reference it is scored against, or an alignment fitted against the filter that uses it, violates that without announcing itself (Section 3.3). Pre-registration [30] is uncommon in the robotics literature we have read and cheap in simulation; we use it and report the predictions that failed.

---

## 3. Testbed, methods and protocol

*Every parameter below was read from the code or configuration in this repository; the status note at the end of this section names the files.*

### 3.1 Simulator and scene

The testbed is the Stonefish marine robotics simulator (v1.3) with its ROS 2 interface, running on ROS 2 Jazzy [32]. The vehicle is a BlueROV2 in the Heavy configuration: eight thrusters, four vectored in the horizontal plane and four vertical (the scenario file defines FrontRight, FrontLeft, BackRight, BackLeft and four Dive thrusters). The Heavy configuration was chosen because, unlike the six-thruster standard layout, it can be commanded in all six degrees of freedom; the inspection mission here uses surge, sway, heave and yaw only, so nothing in the result depends on that choice. The scene is 25 m of water (Jerlov type 0.22, overhead sun) over a flat seabed, with two jacket-type structures built from 3D meshes, which we call the CAD model: the structure under inspection at (20, 0) m, rising from the seabed through the surface, and a second at (-20, 0) m. The mesh is the turbine model distributed with the open-source `stonefish_bluerov2` package (github.com/bvibhav/stonefish_bluerov2, Apache-2.0; a wind turbine with a four-legged jacket, topside and crane, whose source file names it as a commercial asset, `WindTurbine_02`), not an engineering model of a real foundation. Its four legs and X-braces are of the type described in [45]; its crane, boat landing and cable tubes are not symmetric. The vehicle starts at the origin facing the first. Ground truth is the simulator's own pose, used to score and never to navigate (Figure 1; the scene is shown in Figure 2).

**Figure 2.** The simulated scene, from Stonefish's viewer (`eval/fig_scene.py`; screenshots in `docs/paper/stonefish_screens/`). (a) The jacket as modelled, with topside and crane, with the water rendering switched off in the viewer so that the whole structure is visible. (b), (c) Under water, with the viewer at about mid-depth: the structure fades into the water with distance (Jerlov 0.22). (d) The BlueROV2 Heavy at the surface, with the jacket's leg behind it. These are viewer screenshots, not images from the vehicle's stereo cameras, which were not recorded.

### 3.2 Sensors

- **Stereo cameras**, 640 by 480, 75 degree field of view, 0.145 m baseline, rendered at 30 Hz and processed by SLAM at 12 Hz.
- **Doppler velocity log.** Stonefish's own model is zero-mean white noise, which is not what limits a real unit. We add the two terms that do: a per-axis scale error and a small rotation between the true beam frame and the assumed mount frame, so the measured velocity is v_meas = S dR v with S = diag(1 + e_x, 1 + e_y, 1 + e_z). The scale error has a standard deviation of 1.01 % per axis, the long-term velocity accuracy quoted for the Water Linked DVL A50 [5], and the misalignment 0.5 degree per axis. Misalignment is real and is calibrated for in practice [47], but we found no source for its typical size, so 0.5 degree is our assumption of a mounting budget and only the dead-reckoning numbers depend on it. Both are drawn once per run from an integer seed and held, so a seed is one physical unit. The published covariance is not inflated to cover them, as a real unit's covariance describes only its random error. For every flight we recover the realised 3 by 3 operator from the recorded velocities and compare it with the seed's prediction; all flights reported here match to better than 0.001.
- **Inertial unit**: attitude noise 0.1 degree, rate noise 1 degree per second, no yaw drift by default.
- **Pressure sensor** for depth, and a **multibeam sonar**: a forward horizontal fan, 90 degrees, 128 beams, 10 Hz, range 0.25 to 50 m, 0.1 m range noise. It returns a planar slice at the vehicle's depth, so a three-dimensional picture exists only because the vehicle sweeps up and down.

### 3.3 Navigation architecture

**Primary filter.** A 15-state extended Kalman filter (robot_localization [24]) at 50 Hz fusing DVL velocity, IMU attitude and rate, and depth. SLAM enters through one input, its horizontal position.

**SLAM.** Stock ORB-SLAM3 [7] in stereo mode in a container with a ROS 2 wrapper, with 1500 features per frame, scale factor 1.2, 8 pyramid levels, a close-point threshold of 40 baselines (5.8 m), and the default Atlas (Figure 3).

**Figure 3.** ORB-SLAM3 during a flight, two screenshots of its own viewer taken live (which flight and mission time were not recorded; no bag holds camera images) (`eval/fig_orbslam.py`). (a) The left camera image as ORB-SLAM3 receives it, after the image-enhancement node, facing one face of the jacket: green squares are features matched to map points, most on the edges of the X-braces and legs. Status bar: one map, 197 keyframes, 1767 map points, 199 matches. (b) The map seen from the side: black, all map points; red, the local map being tracked against; blue and green, the keyframes and the camera path, here the vertical line of a face sweep. The map is a sparse set of points on the brace edges, which is what the place recogniser matches between faces.

**SLAM-free reference filter.** A second instance of the same filter without the SLAM input. The bridge fits its alignment against this filter, not against the primary filter that consumes the result. Fitting an alignment to the output of the filter that uses it is unfalsifiable: the alignment is fitted to a quantity it determines, so it agrees with itself whether or not it is right.

**SLAM-to-filter bridge.** The wrapper publishes east-north-up and the stack is north-east-down; the bridge converts on ingest. (The conversion negates y and z, which flips the handedness of the horizontal plane, so no rotation-only alignment can absorb it. It was added on 2026-10-16; earlier fused tracks carry a mirrored alignment.) Two bridge versions appear in this paper:

- **Version A** (through 2026-10-16, and the one flight of 2026-10-22 in Section 4.5.3b): a rotation-only horizontal fit, scale fixed at one, constant published position variance of 0.05 m^2.
- **Version B, "Sim(2)"** (from 2026-10-20, all flights in Sections 4.3 to 4.5 except the one in 4.5.3b): a similarity fit (rotation, translation and scale) in closed form over a trailing window of about 75 s, refitted every 5 s with exponential smoothing (0.2), a refit that would turn the frame by more than 15 degrees discarded, scale limited to [0.75, 1.25], and a published variance of max((1.5 times the fit residual)^2, 0.05 m^2). Three protections act on a wrong pose: a **jump reset** (a step of more than 5 m between consecutive raw SLAM poses discards the alignment and refits from scratch), a **consistency gate** (a SLAM pose more than 5 m from the SLAM-free reference filter is rejected, and 40 consecutive rejections reset the alignment), and a **map-collapse detector** (a drop of the map below half its size invalidates the alignment).

Version A's fused results are bridge-dependent and are not used to say what SLAM fusion does (Section 4.4).

### 3.4 Mission and controller

The controller steers on the primary filter. It holds a 17 m standoff using only the minimum sonar return as range, and computes the bearing to the structure from its known position and the filter's, with a one-time yaw calibration at the start. The mission descends, closes to the standoff, then visits four faces; at each it sweeps vertically between 22 m and 1 m depth and back, and between faces it orbits at about 9.5 m depth through an arc of 69 to 90 degrees. After the fourth face it returns to the first, rises (RISE), and returns home (RETURN_HOME). A mission is about 1100 to 1500 s.

**A controller change mid-series.** Until 2026-10-21 the RISE phase pointed the vehicle at its home heading, which swung it up to about 140 degrees off the structure into open water and blinded the cameras. From 2026-10-21 RISE holds the structure heading and the turn home is deferred to RETURN_HOME. Flights are labelled *old RISE* or *fixed RISE*; the re-measure and L4 flights (Sections 4.3, 4.5) used the old controller, and stage 1 of the control used the fixed one.

### 3.5 Locating and classifying a loop closure

A closure is located from the SLAM console log's "Loop detected" line, bracketed by timestamps. The largest step of the raw SLAM track in the surrounding interval is the correction. A similarity SLAM-to-world is fitted on the preceding 150 s of poses only, so the "after" point is placed by a transform that knew nothing about the jump. For each event we report the chord (the size of the correction), the change in radius about the structure's axis, and the turn in bearing about it. A correction under 3 m is **benign**. A larger one is **face-aliased** (wrong-face) if the radius changes by under 10 % and the turn is within 15 degrees of 90 or 180; otherwise **other-large**. An event whose pre-event fit is too short or too poor is **unscorable**; these are counted, not dropped. Two log lines that describe one event are merged. The rule was fixed before most of the data and has not changed (`docs/experiments/PREREG_closure_events.md`). Ground truth enters only to score.

Two checks against the ground-truth track were fixed in advance (`eval/closure_revisit.py`). P1: at a wrong-face closure the vehicle's true position has no earlier visit within 5 m in its own track older than 120 s, and at a benign closure it has. P2: the estimate after a wrong-face closure lands within 5 m of a place the vehicle really did visit earlier. P2 says the closure matched a real place; P1 says it was not the place the vehicle was at.

### 3.6 The inspection product

The vehicle's multibeam is projected into the world under each navigation source and every point is measured against the structure's CAD mesh. The set of returns is fixed (a return counts if it is within 0.5 m of the mesh under the true pose), so a difference between sources is pose error and nothing else. With true poses the reconstruction is 7.3 to 7.4 cm RMS in every recording scored, consistent with the sonar's 10 cm range noise; that is the floor no source can beat. A **trust-SLAM counterfactual** rebuilds the structure as a vehicle would have if it had followed the raw SLAM pose through a wrong-face closure (alignment fitted before the closure and held, with a calibrated rotation offset). It is constructed from the recording, not flown.

### 3.7 The symmetry control

**Manipulation** (Figure 4). Scenario variants add real appurtenances (risers, sacrificial anodes, a horizontal pipe run) to chosen faces. The faces' similarity is measured from the CAD alone: area-weighted points are sampled in the band the vehicle inspects, a sector per face is cut and de-rotated into a shared frame, and ICP registers every face against every other at a 1.0 m inlier threshold in both directions. Two faces are interchangeable only if each covers the other, so a **pair score** is the lower of the two fitness values; 1.0 means interchangeable.

**Figure 4.** The structure from the CAD (`eval/fig_cad.py`). Top: the jacket in the stock scene and in L5, shaded CAD triangles with the water line (blue) and the seabed (brown); L5 adds risers (blue, face 0), anode columns (orange, face 1) and horizontal pipe runs (green, face 2), and leaves face 3 bare. Bottom: L5 seen from outside each of the four faces, with depth below the surface. The tower and rotor above the topside, the second turbine and the seabed plane are not drawn. The jacket also carries a crane, boat landing and cable tubes that are not four-fold symmetric; the pair scores of Figure 5 are measured in the 1 to 22 m band the vehicle inspects.

**Arms** (Figure 5).

| arm | scenario | what is on the faces | worst pair score |
|---|---|---|---|
| L0 | stock | nothing; four bare faces | 1.000 |
| L4 | `sym_l4` | the same hardware on faces 0 and 1, faces 2 and 3 bare | 1.000 |
| L5 | `sym_l5` | face 0: three risers; face 1: two columns of twelve anodes; face 2: horizontal pipe runs at seven depths; face 3: bare | 0.787 |

L4 was a first attempt. Its mean pair score looked like a broken symmetry (0.716), but faces 2 and 3 were still identical to each other and faces 0 and 1 nearly so (0.947); the pair that remained confusable is exactly the pair the closures turned between. We found this only after flying it, and we report it as a failed design (Section 4.5). L5 was built to the criterion fixed before any L5 flight: no pair above 0.8.

**Design.** Paired by DVL unit: seeds 0, 1, 2 in both arms in stage 1, then 29, 3, 26 in stage 2 if the stop rule allowed it (it did not; stage 2 was not flown). Run order alternates arms. **Endpoint:** the mission had at least one face-aliased closure before RETURN_HOME. **Test:** one-sided exact Fisher test, L0 against L5, alpha 0.05, unit = mission; unscorable events are scored both ways and the conclusion must hold under both. A non-significant result is reported as "not detected at this margin and power", never as "no effect". **Void rules:** SLAM never initialises, recorder failure, collision, an incomplete mission, a missing console log, a DVL unit that does not match the seed, or an orbit radius differing from the paired flight by more than 1 m. **Stop rule (Gate B):** after stage 1, if L5 is aliased in 2 or 3 of 3 the experiment stops as a null; if 0 or 1 of 3 and stock is 3 of 3, stage 2 runs; if stock is below 3 of 3 the baseline is not what was assumed and we stop and reassess. With three per arm the best possible result (3 of 3 against 0 of 3) gives p = 0.05.

### 3.8 Protocol

Experiments are pre-registered in the repository before the runs (arms, seeds, predictions, criteria that would falsify them). The recording script refuses to start if the arm name contradicts the live system. Where a prediction failed or a run was void, the registration records it. Every deviation from a registration is listed in Appendix B.

---

*Status note, not for the manuscript.* Sources: `scenarios/bluerov2_turbine.scn`, `bluerov2_turbine_sym_l4.scn`, `bluerov2_turbine_sym_l5.scn` (scene, hardware); `scripts/dvl_bridge.py`, `dvl_a50.yaml`; `ekf.yaml`; `slam_params/bluerov2_stereo.yaml`; `scripts/slam_pose_bridge.py` (version B constants: JUMP_THRESHOLD 5.0, EKF_CONSISTENCY_MAX 5.0, MAX_CONSECUTIVE_REJECTS 40, PUBLISH_VAR_FLOOR 0.05, PUBLISH_VAR_SLACK 1.5, REFIT_INTERVAL 5, REFIT_EMA_ALPHA 0.2, REFIT_MAX_DELTA 15 deg, SCALE 0.75 to 1.25, MAP_CRASH_RATIO 0.5); `scripts/bluerov2_autonomous_controller.py`. **Unchecked:** the 0.5 degree misalignment budget has no citation; the claim that the 5 m gate compares against the SLAM-free filter holds for `dr_reference:=true` (the launch passes `/bluerov2/odometry/dr` as `reference_odom`) and was read from the launch file, not confirmed live.
## 4. Results

Table 1 lists the data. Eleven flights were made with bridge version B (the three stock re-measure flights, two L4 flights, and six stage-1 flights), and one more with the earlier bridge; their DVL units all match the seed's prediction (largest difference 0.0008). Earlier recordings (September to mid-October, bridge A or an anchored filter) supply the closure census and the reconstruction results of Sections 4.2 and 4.3.1.

**Table 1.** Data by period. "Stock" means the unmodified structure.

| period | pipeline | flights | used for |
|---|---|---|---|
| 2026-09-05 to 09-22 | primary filter anchored to ground truth until 10-08; two camera-rate settings across the early recordings | 7 stock recordings with a classified closure (6 complete) | closure classification and census; trust-SLAM counterfactual |
| 2026-10-09 to 10-12 | SLAM-free dead reckoning runs, three DVL units; one SLAM flight with a circular alignment | 4 (one more excluded); 1 SLAM | dead reckoning on a lap; census |
| 2026-10-14 to 10-16 | bridge A (rotation only; mirrored frame until 10-16) | 7 stock SLAM flights (3 without a closure), plus one wide-baseline variant | census; reconstruction, bridge A |
| 2026-10-20 | bridge B, old RISE | 3 stock ("re-measure") | closures; bridge behaviour |
| 2026-10-21 | bridge B, old RISE | 2 L4 | first control (flawed) |
| 2026-10-21 to 22 | bridge B, fixed RISE | 3 stock (L0), 3 L5, 1 stock with the earlier bridge | the paired control |

### 4.1 Dead reckoning on a lap

A lap is 130 to 133 m of path. Dead reckoning from the DVL, inertial unit and pressure sensor, with no SLAM, held horizontal position to a fraction of a metre (Table 2). The three units differ because each carries its own fixed scale and misalignment error. In the eleven flights of bridge B the SLAM-free filter's error against ground truth was 0.09 to 0.36 m RMS, never above 0.65 m, and ended between 0.02 and 0.64 m.

**Table 2.** Dead reckoning over one lap, no SLAM, by DVL unit (older runs).

| DVL unit | final error (m) | per cent of path | RMS (m) |
|---|---|---|---|
| white noise only | 0.04 | 0.01 | 0.03 |
| seed 0 | 0.11 | 0.08 | 0.10 |
| seed 1 | 0.34 | 0.26 | 0.20 |
| seed 2 | 0.51 | 0.37 | 0.36 |

The inertial heading error is 0.064 degrees on average with no growth. With a yaw drift of 3 degrees per hour added, dead reckoning is still 0.20 m RMS over the lap, because an orbit never takes the vehicle far from where heading was last known (at most 37.9 m from the start, 3.8 m at the end). **This is a result about single-lap missions**, and it is why a map-based correction has little to correct. Reconstructed from the vehicle's own sonar, the structure is 8 to 59 cm RMS from the CAD under dead reckoning, median 14 cm (interquartile range 10 to 30 cm) over 18 recordings, against a sensor floor of 7.3 cm (the two excluded recordings of Appendix A are not counted; `eval/recon_batch.py`).

### 4.2 Wrong-face loop closures

**What they look like.** Table 3 lists every closure classified in the first nine bridge-B flights; the two newest (L0 seed 2, L5 seed 2) each had one benign closure and appear in Table 3b. Seven are face-aliased: the correction turns the estimate about the structure's axis by 88.5 to 92.2 degrees (once 179.7), changes its radius by at most 3 %, and moves it 23.5 to 32.8 m. Each is within 2.2 degrees of a symmetry operation of the structure. Added to the twelve events of the earlier recordings (Figure 6; five face-aliased: 87.5 to 90.5 degrees, once 178.4; 24 to 33 m, with the vehicle's true position 18.7 to 20.5 m from its earlier track, against 0.0 to 1.0 m at the six benign closures), the geometric signature is the same in both sets. The revisit checks were repeated on the 12 new flights (Table 3b). P2 held for all seven face-aliased closures: the estimate after the closure was within 1.0 m of a place the vehicle really had been (0.0 to 1.0 m). P1, as registered, failed: it held for three of the seven (the two L4 closures and L0 seed 0, true position 17.6 to 22.5 m from any earlier visit) and failed for the other four (the three re-measure flights and L0 seed 1, 0.2 to 0.7 m). Those four are the closures at the return to the starting face, where the vehicle truly was at that place about 200 s earlier, at the start of the mission. There the recogniser had a correct candidate and chose another face, so the closure is wrong not because the vehicle had never been there but because the match is to a different place from the one it was at. The five benign closures had an earlier visit within 1.1 m, as predicted. P1 as worded is therefore only a valid confirmation for closures that are not at a true revisit, and we say so.

**Table 3.** Closures in the bridge-B flights. Chord is the size of the correction; turn is the change in bearing about the structure's axis.

| flight | t (s) | chord (m) | radius change | turn (deg) | class |
|---|---|---|---|---|---|
| stock seed 0 | 1034 | 23.5 | -3 % | +88.5 | face-aliased |
| stock seed 1 | 1018 | 24.4 | +1 % | +90.1 | face-aliased |
| stock seed 2 | 1024 | 24.6 | +3 % | +90.9 | face-aliased |
| L4 seed 0 | 331 | 24.2 | +1 % | +89.8 | face-aliased |
| L4 seed 1 | 336 | 23.8 | 0 % | +89.8 | face-aliased |
| L0 seed 0 | 758 | 32.8 | 0 % | +179.7 | face-aliased |
| L0 seed 0 | 802 | 11.1 | +21 % | +32.5 | other-large (new map, tracking lost; fit scale 0.32) |
| L0 seed 1 | 1025 | 25.1 | +2 % | +92.2 | face-aliased |
| L5 seed 0 | 1021 | 1.7 | -1 % | -5.7 | benign |
| L5 seed 1 | 1026 | 1.0 | -3 % | -3.0 | benign |

**Table 3b.** Ground-truth revisit checks on the closures of the 12 flights since 2026-10-20 (`eval/closure_revisit.py`). "Earlier visit" is the distance from the vehicle's true position to its own track older than 120 s; "estimate after" is the distance from the estimate after the closure to the nearest earlier true visit.

| flight | closure (s) | class | earlier visit to true position (m) | estimate after to a true earlier visit (m) |
|---|---|---|---|---|
| re-measure seed 0 | 1034 | face-aliased | 0.3 | 0.0 |
| re-measure seed 1 | 1018 | face-aliased | 0.2 | 0.6 |
| re-measure seed 2 | 1024 | face-aliased | 0.4 | 0.0 |
| L4 seed 0 | 331 | face-aliased | 22.3 | 0.0 |
| L4 seed 1 | 336 | face-aliased | 22.5 | 0.2 |
| L0 seed 0 | 758 | face-aliased | 17.6 | 0.1 |
| L0 seed 0 | 802 | other-large | 20.8 | 13.8 |
| L0 seed 1 | 1025 | face-aliased | 0.7 | 1.0 |
| L0 seed 2 | 1015 | benign | 0.9 | |
| L5 seeds 0, 1, 2 | 1019 to 1026 | benign | 0.5 | |
| earlier bridge, seed 1 | 1018 | benign | 1.1 | |


In five of these flights the console log holds two "Loop detected" lines 13 to 44 s apart, and the classifier attributes the single large raw step to both; one closure is counted. Whether the earlier line is a separate small closure was not resolved.

**How often.** Counted by mission (Table 4). The census counts every recorded mission on the stock structure with the standard cameras, SLAM running, its console log recorded, and at least 1000 s of mission, so that it reached the return to the starting face (about 1015 to 1035 s); the recordings left out are listed under the table. Four complete missions had no loop closure at all. By this rule 9 of 20 missions had a wrong-face closure (45 %; 95 % interval 23 to 68 %). Split at 2026-10-20, where the bridge, frame handling and camera-rate setting and, later, the RISE controller changed, it is 4 of 14 before and 5 of 6 after (one-sided exact p = 0.04; 0.06 if the one mission that stopped at 616 s, after a wrong-face closure at 525 s, is counted). The split was chosen after seeing the data, the periods differ in several ways at once, and the result crosses 0.05 with one recording, so we report it as a difference between the recorded periods, not as the effect of any one change. The wrong-face closures fall at two points in the mission, mid-mission (525 to 776 s, and 331 to 336 s in L4) and at the return (1018 to 1034 s); Section 4.2b describes where on the structure. The counts are a floor: closures in unrecorded missions can only raise them.

**Table 4.** Complete stock-structure missions with SLAM running and a console log, by period.

| period | missions | with a wrong-face closure | without any closure |
|---|---|---|---|
| 2026-09-05 to 09-22 (filter anchored to ground truth) | 6 | 3 | 0 |
| 2026-10-12 (circular alignment) | 1 | 0 | 1 |
| 2026-10-14 to 10-16 (bridge A) | 7 | 1 | 3 |
| 2026-10-20 (bridge B, old RISE) | 3 | 3 | 0 |
| 2026-10-21 to 22 (bridge B, fixed RISE) | 3 | 2 | 0 |
| total | 20 | 9 | 4 |
| 2026-10-22 (earlier bridge, one flight; not in the total) | 1 | 0 | 0 |

Not counted: `DeepSight_20260922_230945`, which stopped at 616 s after a wrong-face closure at 525 s (counting it gives 10 of 21); two September recordings with no console log; three recordings aborted within 310 s; `a50_dr_seed1_20261012_200040`, in which the SLAM pose was not recorded; and the wide-baseline camera variant (`slamb30`), a different camera with no console log. The L4 and L5 flights are on modified structures and are not in the census. Every recording and what it is used for is listed in Table A1.

**4.2b Where in the mission they fall.** Figure 7 (`eval/fig_where.py`) plots the true depth of the vehicle at each classified closure. Of 12 wrong-face closures, 9 fell at 20 m depth or more, at the seabed end of the 1 to 22 m scan, 10 while its vertical speed was under 0.25 m/s (at the bottom of a face sweep, where the vehicle turns, not while sweeping up or down the structure), and 11 within 4 degrees of a face centre. Of 11 benign closures, 1 fell at 20 m or more; 10 fell while descending at 0.26 to 0.44 m/s on the starting face at 14 to 19 m. Median depth was 20.8 m for wrong-face and 18.0 m for benign (Mann-Whitney one-sided p = 0.005); the 20 m split is 9 of 12 against 1 of 11 (Fisher p = 0.002), with a threshold chosen after looking, so these describe the data and do not test a hypothesis. The place a 90-degree closure pointed to had been visited 145 to 222 s earlier at the same depth, the bottom of the previous face. This puts the period difference of Section 4.2 in context: early wrong-face closures fell mid-mission at the bottom, those since 2026-10-20 mostly fall at the bottom on the return to the starting face, and the benign closure at the return is the one that happens higher up. No L5 flight had a closure at the bottom. We did not find why the bottom: the stock structure is symmetric at every depth, no camera images were recorded, and two candidates (light falling with depth in the water model, and a view dominated by the base and seabed) remain untested.

### 4.3 What a wrong-face closure costs

**4.3.1 The inspection product, if the vehicle trusts SLAM.** Following the raw SLAM pose through a wrong-face closure rebuilds the structure to 3.8 to 7.5 m RMS from the CAD in all five wrong-face missions of the earlier recordings (median 5.8 m), against 0.26 to 0.73 m for the same method before the closure. The picture is a second, rotated copy of the structure laid across the first (Figure 8). For the five bridge-B stock flights it gives 10.0, 6.2, 3.6, 19.1 and 2.9 m after the closure (Table 7), so across the ten missions the range is 2.9 to 19.1 m (median 6.0 m); the largest, 19.1 m, is the 180-degree closure. This is constructed from recordings, not flown. Figures 9 and 10 show, for the earlier recordings and bridge A, ORB-SLAM3's own map against the CAD before and after one such closure, and the bearing and position error that closure produced.

**4.3.2 What bridge B did.** Bridge B does not follow the raw pose, so the counterfactual is not what its vehicle did. What it did, in every flight with a wrong-face closure, was go silent. Figure 11 follows one such closure through a whole flight.

**Figure 11.** One wrong-face closure followed through a bridge-B flight (stage-1 stock flight L0 seed 0, the 180-degree closure; `eval/fig_story.py`). (a) Plan view: the true track (black), dead reckoning (dashed), the primary filter that fuses SLAM, and ORB-SLAM3's raw pose placed by a similarity fitted on the 150 s before the closure (residual 0.11 m), which jumps 33 m across the structure at 758 s. Red dots are the poses the bridge published after it re-locked, during the return home. (b) Horizontal error against the truth: the fused filter, dead reckoning, and each published SLAM pose. After the closure the bridge is silent (grey) for 219 s; its first poses after the re-lock are about 3 m wrong, the 5 m gate admits them, and the fused error rises above 4 m and ends 4.7 m wrong, while dead reckoning stays under 0.2 m.


A 24 m step between consecutive SLAM poses exceeds the 5 m jump threshold and discards the alignment. The bridge then publishes nothing until it can fit a new one, which needs 5 m of horizontal spread in the correspondence window, and rejects a fit whose scale leaves [0.75, 1.25]. (In L0 seed 1 the bridge log shows every fit after the closure rejected, for scales of 0.35 to 0.48, until the end; for the other three late-closure flights we have the silence but not the log lines.) Table 5 shows what followed.

**Table 5.** The bridge's output after each wrong-face closure (`eval/jump_attribution.py`, `eval/gate_plateau.py`). "Silent" is the time with no published SLAM pose. "First pose" is the median distance from ground truth of the first five poses published after a re-lock.

| flight | closure (s) | mission left (s) | bridge silent | first pose after re-lock (m) | primary filter, max / final (m) | dead reckoning, final (m) | ends from home (m) |
|---|---|---|---|---|---|---|---|
| stock seed 0 | 1034 | 444 | to the end | none | 0.55 / 0.49 | 0.12 | 1.74 |
| stock seed 1 | 1018 | 93 | to the end | none | 0.42 / 0.36 | 0.27 | 1.29 |
| stock seed 2 | 1024 | 24 | to the end | none | 1.12 / 0.33 | 0.64 | 1.42 |
| L0 seed 1 | 1025 | 100 | to the end | none | 0.84 / 0.84 | 0.04 | 1.95 |
| L4 seed 0 | 331 | 1056 | 176 s, then again 257 s | 2.5, then 1.9 | 4.68 / 4.15 | 0.14 | 2.81 |
| L4 seed 1 | 336 | 781 | 172 s, then five shorter | 1.2, then 0.04 to 3.2 | 5.02 / 1.25 | 0.55 | 3.14 |
| L0 seed 0 | 758 | 517 | 219 s, then 37 s and 160 s | 2.9, 2.6, 1.6 | 4.89 / 4.74 | 0.02 | 4.23 |

**In the four flights where the closure came within the last 24 to 444 s of the mission, the bridge never re-locked and the fused estimate stayed within 1.2 m.** That is not containment by design: SLAM was switched off for the remainder, and dead reckoning carried the vehicle home. **In the three flights where the closure came mid-mission, the bridge did re-lock, on windows dominated by vertical sweeps and short arcs, and the first poses it published were 1.2 to 2.9 m from the truth.** The consistency gate (5 m against the SLAM-free filter) accepted them, the primary filter, which was told each pose was good to about 0.22 m, followed them, and the fused estimate was wrong by 4 to 5 m to the end of the mission in two flights (L4 seed 0, L0 seed 0, the latter with one dip) and repeatedly rose to the gate in the third, ending 1.25 m out.

**The filter cannot recover.** Nothing in the primary filter other than SLAM observes absolute horizontal position: the DVL, inertial unit and depth sensor integrate. Once a bad pose has pulled the estimate, the offset stays until the end. This is the flat line in Figure 12 (b) and (c) (L4 seed 0 sits at 4.2 to 4.7 m from about 560 s to the end, 830 s).

**4.3.3 The ceiling.** Across all eleven bridge-B flights the published SLAM pose was never farther than 5.00 m from dead reckoning (the gate), and in the seven flights whose fused error became large it reached 4.69 to 5.00 m while the primary filter's error peaked at 4.4 to 5.0 m (Figure 13). In the four flights where it stayed small, the published pose was never more than 1.0 m from dead reckoning. The gate sets the ceiling: a SLAM pose that is wrong by 4.9 m passes. Thirteen re-locks in the first nine flights (the two newest, L0 seed 2 and L5 seed 2, are not in this count) gave first poses that were 0.04 to 4.85 m from the truth (median 1.9 m); nine of the thirteen were more than 1 m wrong.

**4.3.4 The same failure without a wrong-face closure.** Four flights had no wrong-face closure (L5 seeds 0, 1, 2 and L0 seed 2), and all four ended with the fused estimate 4.4 to 4.8 m wrong and the vehicle 6.3 to 6.7 m from home (Figure 14; L0 seed 1, which had a wrong-face closure but no re-lock, ended 1.95 m from home). Each had a benign closure at 1015 to 1026 s; in the first two L5 flights the fused estimate stepped by 1.0 to 1.7 m at that time. The map then collapsed during the return home (1104 to 1115 s in the four flights, with the fused error above 2 m by 1092 to 1106 s). For the first two L5 flights the bridge re-locked after a silence of 10 to 12 s and the first pose it published was 4.8 m from the truth, which the gate accepted; we did not repeat that measurement for the other two. So avoiding the wrong-face closure did not make the end of the mission safe, and the stage-1 comparison is a comparison of closures, not of navigation outcomes.

### 4.4 Does fusing SLAM help on a lap?

**Position.** In the window between the first published SLAM pose and the first wrong-face closure (or 1000 s where there was none), the primary filter's horizontal RMS was 1.04 to 1.69 times the SLAM-free filter's in nine of the eleven bridge-B flights and 0.88 and 0.97 times in the two L4 flights, whose windows are only 100 s long. With the DVL units of seeds 0, 1, 2 the pre-closure RMS was 0.12 m (dead reckoning) against 0.17 m (fused), 0.20 against 0.24 and 0.36 against 0.44, and in the stage-1 repeats 0.20 against 0.27 (seed 1) and 0.35 against 0.42 (seed 2). **On position, fusing SLAM through bridge B did not help on a lap and cost 4 to 69 %.** The one flight with the earlier bridge (seed 1) was worse, 2.28 times (0.45 m against 0.20 m). This is the same sign as the earlier finding with bridge A and a far smaller size (bridge A: 5 to 37 times); the earlier ratios were mostly the bridge's defects and are not used.

**The inspection product (Table 7, Figure 15).** The reconstruction RMS is a different measure from position RMS. With bridge B and the three DVL units of seeds 0, 1 and 2, whole-run dead reckoning gave 10, 29 and 47 cm against 16, 17 and 23 cm for the primary filter that fuses SLAM; the stage-1 repeats gave 31 cm against 19 cm (seed 1) and 52 cm against 42 cm (seed 2). So fusing SLAM improved the sonar model by 19 to 50 % in all four flights on the units with the larger dead-reckoning error (seeds 1 and 2) and worsened it by 55 % for the cleanest unit (seed 0). With the earlier bridge on seed 1 it worsened it, 45 cm against 31 cm, where bridge B had improved it twice; this is one flight. **The exception is a flight where the bridge re-locked after a mid-mission closure** (L0 seed 0): 1.18 m fused against 11 cm dead reckoning, which is the 1 to 3 m first-pose error of Section 4.3.2 appearing in the product.

The two measures disagree on seeds 1 and 2: position RMS says fusing costs 17 to 36 % there, the product says it helps by 19 to 50 %. We have not resolved this. They weight time differently (the product is scored on structure returns, which exist only while the vehicle is scanning at the standoff, whereas position RMS includes the descent, the transits and the return), and the product also depends on orientation. The honest statement is that fusing SLAM did not improve position on a lap and may have improved the sonar model where dead reckoning is worst, that three units cannot say which, and that the planned flights with the cleanest, the scale-dominated and the worst units (seeds 29, 3, 26) were not flown.

**Table 7.** Reconstruction RMS against the jacket CAD, bridge-B stock flights and the one earlier-bridge flight (`eval/recon_batch.py`, group `sim2`). Whole run unless stated. "Trust-SLAM" is the counterfactual, after the closure only.

| flight | DVL seed | ground-truth poses | dead reckoning | primary filter (fused) | fused, after closure | trust-SLAM, after closure |
|---|---|---|---|---|---|---|
| stock seed 0 | 0 | 7.4 cm | 10.3 cm | 15.9 cm | 24.9 cm | 9.98 m |
| stock seed 1 | 1 | 7.3 cm | 29.1 cm | 17.3 cm | 17.6 cm | 6.16 m |
| stock seed 2 | 2 | 7.3 cm | 46.6 cm | 23.4 cm | 33.7 cm | 3.64 m |
| L0 seed 0 (fixed RISE) | 0 | 7.3 cm | 10.5 cm | **1.18 m** | 1.95 m | 19.1 m |
| L0 seed 1 (fixed RISE) | 1 | 7.3 cm | 31.0 cm | 19.2 cm | 25.6 cm | 2.91 m |
| L0 seed 2 (fixed RISE; no wrong-face closure) | 2 | 7.3 cm | 51.7 cm | 42.1 cm | n/a | n/a |
| stock seed 1, earlier bridge (version A) | 1 | 7.3 cm | 30.5 cm | 45.0 cm | n/a | n/a |

---

### 4.5 The control: is the symmetry the cause?

**4.5.1 The manipulation.** Figure 5 shows the pair scores. L0's six pairs are all 1.000. L4's mean (0.716) looked like a broken symmetry, but its pair 2-3 stayed at 1.000 and 0-1 at 0.947. L5's worst pair is 0.787 (1-2), met the criterion fixed beforehand (none above 0.8), and its other pairs are 0.685 to 0.776.

**4.5.2 The first control failed by design.** Both L4 flights had a wrong-face closure, both at the moment of arriving on face 1 (331 and 336 s, +89.8 degrees). L4 left the 0-1 pair almost interchangeable and the closures turned between exactly that pair. We found the flaw only after flying; it is stated in the pre-registration's outcome section, and L4 neither supports nor refutes the hypothesis. The pair-score summary hid it: a mean over pairs hides the pair that still confuses.

**4.5.3 Stage 1 of the paired test.** Table 6 (generated) and Figure 16. The fixed RISE controller was used in both arms, the same DVL units were paired, and the paired orbit radii differ by 0.09, 0.07 and 0.14 m (the void rule is 1 m).

**Table 6.** Stage 1 of the paired test, by DVL unit (`docs/paper/live_numbers.md`).

| flight | any wrong-face closure | first closure (s) | turn (deg) | chord (m) | benign closures | orbit radius (m) |
|---|---|---|---|---|---|---|
| L0 seed 0 | **yes** | 758 | +179.7 | 32.8 | 0 | 17.15 |
| L5 seed 0 | no | | | | 1 (1.7 m, 1021 s) | 17.06 |
| L0 seed 1 | **yes** | 1025 | +92.2 | 25.1 | 0 | 16.95 |
| L5 seed 1 | no | | | | 1 (1.0 m, 1026 s) | 16.88 |
| L0 seed 2 | no | | | | 1 (0.4 m, 1015 s) | 17.27 |
| L5 seed 2 | no | | | | 1 (1.1 m, 1019 s) | 17.13 |

The result is 2 of 3 stock missions with a wrong-face closure and 0 of 3 L5 missions (one-sided exact Fisher p = 0.20). **This is not significant; with three per arm the best possible split gives p = 0.05.** It is reported as "not detected at this margin and power". The pre-registered stop rule (Gate B) says that if the stock arm is below 3 of 3 the baseline is not what was assumed and flying stops for reassessment; stock was 2 of 3, so stage 2 was not flown. Stock seeds 1 and 2 met the same mission event (the return to the starting face, 1025 s and 1015 s) with different outcomes, so at that event the stock outcome is not near-certain. All benign closures in stage 1, in both arms, fell at that event.

**4.5.3b One flight with the earlier bridge.** Because the closure rate might depend on the bridge, which sets the path the vehicle steers (Section 5.4), we flew one more stock mission on DVL unit 1, the unit on which the current bridge had a wrong-face closure, with the bridge replaced by the committed version from before the similarity fit (rotation-only; log lines confirm no scale term). The DVL unit matches (max difference 0.0002), the orbit radius is 16.90 m, and the one closure, at 1018 s at the return to the starting face, was benign. This is one flight. With the current bridge, 2 of the 3 stock missions had a wrong-face closure (L0 seeds 0 and 1) and one did not (seed 2), so a mission without one is expected about a third of the time even if the bridge made no difference; the contemporaneous comparison is 0 of 1 against 2 of 3 (exact p = 0.25). It is compatible with the bridge influencing the path, and through it the closure, but it does not show it. The earlier-bridge flight ended with the same fused-error ceiling (4.81 m) as the other flights in which the map collapsed at the start of the return, so the 5 m gate, not the bridge version, set that ceiling.

**4.5.4 What stage 1 cannot separate.** L5 changes the symmetry and also gives the cameras many strong features. A reduction in wrong-face closures in L5 is consistent with symmetry as the cause and equally with feature density as the cause; the design does not separate them.

**4.5.5 The L5 map.** Because the L5 hardware dominates what the cameras see, we measured how much of ORB-SLAM3's own map lies on the bare jacket, for one flight per arm (Figure 17; the CAD omits the hardware, so points on hardware against a leg escape the count and the L5 numbers are a lower bound on contamination). Placed by the true trajectory, which uses no CAD, the two maps look alike: 19.4 % (stock) and 17.7 % (L5) of points within 25 cm of the jacket. Placed as well as possible against the CAD, 66.5 % of the stock map and 28.2 % of the L5 map are within 25 cm (median distance 0.14 m against 0.54 m); the L5 fit also finds a different scale (0.93 against 1.01). So the best-fit placement of the L5 map is worse, which may mean the hardware pulls the fit or that the L5 map is really scaled differently; the data do not say that most of the L5 map is hardware. We therefore use L5 flights only for the closure question and exclude them from every reconstruction score.

### 4.6 Checking a closure (offline, exploratory)

*A limit found later.* These checks were scored on mid-mission wrong-face closures. The revisit check says a closure is suspect when dead reckoning shows the vehicle has not been near the place; it would not flag the four return-time wrong-face closures of the later flights (Table 3b), where the vehicle truly had been there. The size check (a 24 to 33 m correction against sub-metre dead-reckoning uncertainty) and the chord check would still apply.

Three checks were scored on the eleven cleanly classified closures of the earlier recordings: a **size** check (correction larger than a tolerance plus three times the dead-reckoning uncertainty), a **revisit** check (the dead-reckoned track says the vehicle has not been near this place), and a **chord** check (the correction's length equals the chord of a symmetry operation of the orbit, 2R sin(pi k / n), 24.0 or 34.0 m for R = 17 m and n = 4). On a lap every check separates the classes and they are interchangeable. With drift injected into the true track, growing linearly with distance (Figure 18), the size and revisit checks begin to miss wrong-face closures from about 7 m of drift and catch none by 18 m, while the chord check, which uses no dead reckoning, catches all of them until honest corrections grow as large as the jumps (false alarms from about 24 m of drift). The 7 to 18 m window is hours of mission away at this vehicle's rates. The chord check was specified after the five wrong-face chords had been seen, so it is in-sample, and none of this was run in the loop. The plain check against odometry is prior art [41]. We report this as a hypothesis with a stated model, not a result.

---
## 5. Discussion

### 5.1 What the study found

**First, on one lap dead reckoning is enough, and fusing SLAM did not improve position.** With a DVL carrying a datasheet-grounded systematic error, dead reckoning held position to 0.1 to 0.5 m and rebuilt the structure to a median 14 cm RMS against a 7 cm sensor floor. With the best bridge we built (version B), fusing SLAM raised the position RMS by 4 to 69 % in nine of eleven flights over the stretch where SLAM was fused. The sonar model of the structure tells a different story: fusing improved it by 19 to 50 % in four flights on the two DVL units with the larger dead-reckoning error and worsened it for the cleanest unit, and it was ruined when the bridge re-locked badly (Section 4.4). Three units cannot say which regime a real unit sits in.

**Second, stock ORB-SLAM3 closes loops to the wrong face, and the failure has a recognisable geometric shape.** The correction rotates the estimate by 90 degrees (once 180) about the structure's axis, keeps its radius, and moves it 24 to 33 m to a place that is not the one the vehicle is at, though it is a place the vehicle really visited earlier (Section 3.5, P2 held in all seven new cases). In the three mid-mission closures of the later flights, and in the earlier recordings, the vehicle's true position was 17.6 to 22.5 m from any earlier visit; at the return to the starting face it had been there about 200 s earlier, and the matcher chose another face. The rotation signature held in every wrong-face closure of the bridge-B flights.

**Third, the damage is done by the bridge as much as by the closure.** The wrong-face closure makes the bridge discard its alignment and go silent. When the mission ends before it re-locks, the vehicle is saved by SLAM being off. When it re-locks mid-mission, the first poses are 1 to 3 m wrong, the 5 m gate admits them, the filter follows them, and nothing else in the filter can pull it back. The gate's width, not the closure's size, sets how wrong the fused estimate gets. The same happens without a wrong-face closure when the map collapses and the bridge re-locks (Section 4.3.4).

**Fourth, whether the symmetry is the cause is the open question, and the evidence is suggestive and thin.** In stage 1, two of three stock missions and none of three missions on a structure with every face different had a wrong-face closure (p = 0.20). The direction is what a symmetry cause predicts. The size cannot show it, and the stock baseline itself was not near-certain, which the registered rule treats as a reason to stop.

### 5.2 The mechanism, and how much of it is shown

The signature is what a four-fold symmetric structure would produce: a match to an unrelated place would not preserve the radius and land on a symmetry rotation. Ground truth confirms the closures are wrong in the sense that matters: the estimate after each one lands within 1.0 m of a place the vehicle really visited, not the one it was at (P2, Section 3.5), and in the mid-mission cases the vehicle's true position was 17.6 to 22.5 m from any earlier visit. At the return to the starting face the vehicle had been there, and the matcher chose another face. That is strong circumstantial evidence for symmetry as the mechanism and is not a test of it.

The control is the test, and it has two limits. It is small, and L5 changes feature density as well as symmetry. Two things in the data keep us from over-reading it. First, the L5 closures fell at the very event where most stock wrong-face closures since 2026-10-20 fell (the return to the starting face), and were benign there, which fits a symmetry story, but second, stock seeds 1 and 2 met that event with different outcomes on the same structure, so the stock outcome there is far from certain and something other than the structure's symmetry also sets it (Section 5.4). The first control (L4) taught one lesson worth keeping: a mean over pairs of faces hid that one pair was still interchangeable, and the closures turned between exactly that pair. A symmetry index for a design should be reported as its worst pair.

### 5.3 What to change in a bridge like this one

All of the following are hypotheses; none was run in the loop.

- **Do not trust the first poses after a re-lock.** A re-lock fitted on a window of vertical sweeps and short arcs is poorly conditioned; 9 of 13 first poses were more than 1 m wrong. Hold the new alignment until the window has real horizontal spread and the fit's residual is small, or publish with a variance that reflects how short the window is.
- **Make the gate adaptive, or replace it with a chi-squared test against the SLAM-free filter's own uncertainty.** A fixed 5 m gate lets a 4.9 m error through and bounds the fused error at the gate's width. A gate of the order of the dead-reckoning error (tenths of a metre on a lap) would have rejected nearly all of the badly wrong poses published; on a long mission, where dead reckoning has drifted, a tight gate would also reject honest corrections, which is the trade-off an adaptive form has to manage.
- **Recognise a symmetry jump instead of resetting on it.** A correction whose length equals the chord of a symmetry operation of the orbit, and whose axis is the structure's, is a wrong-face closure by construction. Rejecting it, instead of discarding the alignment and refitting, would keep the old frame (Section 4.6 evaluates this offline and in-sample).
- **Let the filter recover.** Nothing in the primary filter observes absolute horizontal position other than SLAM, so a bad pull is permanent. A bounded-rate pull or a second, independent check would avoid a permanent offset.

### 5.4 Did the closure rate change, and is it the bridge?

We first read the data as saying the rate had changed with our pipeline. Counted by complete mission it is 4 of 14 before 2026-10-20 and 5 of 6 after (one-sided p = 0.04, exploratory; Section 4.2), with several changes between the periods, a difference in the recorded missions that we do not attribute to any one change. What remains unexplained is when in the mission the wrong-face closure falls: mid-mission (525 to 776 s) in most of the earlier recordings, at the return (1018 to 1034 s) in most since. Both are at the seabed end of the structure (Section 4.2b), and why that end is unexplained too. Bridge B changes the fused estimate the vehicle steers on and so the path, and a different path could change which place the recogniser matches. We flew the cheapest discriminating run once (stock, earlier rotation-only bridge, fixed RISE, DVL unit 1): the closure was benign (Section 4.5.3b). One flight cannot separate a bridge effect from chance. What this means for the control is that the stock rate per mission is between about one in four and two in three over all periods (9 of 20, Section 6.3) and 5 of 6 in the period of the control, which is why three flights per arm could not decide it.

### 5.5 What the stage-1 outcome does

Stage 1 ended with stock 2 of 3 and L5 0 of 3 (p = 0.20). Under the registered rule (stock below 3 of 3) this means stop and reassess, and stage 2 was not flown. The rows we planned for are kept here so a reader can see what each outcome would have meant.

| outcome of stage 1 | what it does |
|---|---|
| stock 3 of 3, L5 0 of 3 (p = 0.05) | the control supports the symmetry as the cause; stage 2 recommended |
| stock 3 of 3, L5 1 of 3 (p = 0.20) | consistent, not shown; stage 2 decides |
| **stock below 3 of 3 (this is what happened: 2 of 3)** | the baseline is not near 1, the control is underpowered; stop (the registered rule) |
| L5 aliased in 2 or 3 of 3 | no reduction detected at this margin; stop and report as such |

Whatever the outcome, Section 4.3 stands: the bridge-level findings do not depend on the control.

### 5.6 A practical reading, with its limits

For an operator of a vehicle like this one around a symmetric structure: do not let a camera loop closure move the estimate by tens of metres, and do not let a SLAM source that has just re-locked pull the filter by metres, without a check against the dead-reckoned track. On a single lap the dead-reckoned track is the better estimate. This is advice about a simulated vehicle with a particular bridge; it is stated for the setting it was measured in.

### 5.7 Implications for simulation-based evaluation

These are practice findings from this study, not a comparison of simulators; we ran one simulator (Stonefish) and did not run another.

- **Sensor error models.** A white-noise DVL error makes dead reckoning look far better than a real unit, whose scale-factor and misalignment errors are fixed per unit and integrate with distance. We drew both per run from a seed and checked the realised unit from the bag each time; without that the dead-reckoning baseline would have been overstated.
- **Frame conventions and circular references.** An unconverted ENU/NED convention and an alignment fitted against the filter it feeds each made a discrete failure look like continuous drift. A SLAM-free reference filter made the alignment falsifiable.
- **Where in the scene.** Closure failures concentrated at the seabed end of the structure (Section 4.2b), so a study that samples only mid-depth views, or reports a pooled rate without saying where, would miss them.
- **Denominators.** Counting the rate only over missions that had a closure overstated it (10 of 17 against 9 of 20 complete missions); missions with no closure belong in the denominator.
- **Labels.** Two recordings were mislabelled (a seed, and a SLAM-in-the-loop run named dead reckoning); every flight's identity was recovered from its bag.

### 5.8 Next steps, in order of value

1. Find why the wrong-face closures concentrate at the seabed end of the structure (Section 4.2b), starting with a run that records the camera images, and whether the higher recent rate (Section 4.2) follows any one pipeline change.
2. Test a conditioned re-lock and an adaptive gate in the loop.
3. Fly the three remaining DVL units (seeds 29, 3, 26) to settle whether fusing helps the model where dead reckoning is worst.
4. A second structure geometry (three or six legs) or a second SLAM front end, to take the result from one scene to two.
5. Real data.

---
## 6. Limitations

The first four decide how far the conclusions reach. The rest qualify particular numbers.

**6.1 Simulation, one scene, one SLAM system, one mission.** Every mission is simulated in Stonefish's water model (Jerlov coefficient 0.22 in the scenario file, flat surface, overhead sun at 90 degrees elevation), around one structure, with one stereo SLAM system and one mission profile. Real water adds turbidity, marine growth, caustics and lighting change, each of which would change which views are matched. The sonar noise is Gaussian. We claim the failure has a geometric signature that does not depend on the simulator; we do not claim its frequency transfers, and we have no sea-trial or tank data. The two exposures that matter most are, first, that real water usually carries more particles and less light than this scenario, which reduces the number of features and could raise or lower the rate of wrong matches, and second, that real structures are not clean: marine growth and hardware break symmetry, which is the effect the L5 control probes. A test on recorded real-world stereo footage of a symmetric structure is the needed next step, and a reader should treat every frequency in this paper as a simulated one.

**6.2 The control is small, and what it changes is two things at once.** Stage 1 has three missions per arm. With three per arm the best attainable result (3 of 3 against 0 of 3) gives p = 0.05, and any weaker split cannot reach significance. The L5 manipulation breaks the four-fold symmetry and also gives the cameras a great many strong, distinctive features that the bare jacket does not have; the design cannot separate the two, so a reduction in wrong-face closures in L5 is consistent with symmetry as the cause and not proof of it. The pair score that sets the manipulation is a geometric proxy measured at a 1.0 m inlier threshold on sampled CAD surface; it says nothing directly about what ORB features the cameras see. L5 is "every pair differs by a margin" (worst pair 0.787), not "every pair is unmistakable". A null at this margin would not rule out a stronger manipulation. The L5 map is also less representative of the jacket than the stock map (Section 4.5.5), which is why L5 flights are used for this one question and are excluded from every reconstruction score.

**6.3 The per-mission rate is uncertain.** 9 of 20 complete stock missions had a wrong-face closure (45 %, 95 % interval 23 to 68 %; the inclusion rule is in Section 4.2). The rate differs by period, 4 of 14 before 2026-10-20 and 5 of 6 after (one-sided p = 0.04 by a test chosen after seeing the data, 0.06 if one incomplete mission is counted), and the periods differ in many ways (a ground-truth-anchored filter in the September flights, a circular alignment on 2026-10-12, the frame handling, the camera-rate setting, the bridge, the RISE controller). We do not attribute the difference to any one of them. The control arms are contemporaneous with each other, so the comparison between them does not depend on this. When in the mission the closure falls (mid-mission or at the return) also changed and is unexplained, although both are at the seabed end of the structure (Section 4.2b).

**6.4 What the cost numbers are.** The trust-SLAM reconstruction is a counterfactual built from a recording, not a flown condition, and its accuracy before the closure sets a floor under what it can show (mean rotation residual of the pre-closure fit 1.0 to 4.0 degrees in nine of the ten missions and 11.7 degrees in the tenth; `eval/recon_batch.py`). The fused-filter results are results about bridge version B and its 5 m gate; a different bridge (a tighter or adaptive gate, a robust back-end, a chi-squared check against the SLAM-free filter) would behave differently, and we did not test one. The analysis of the gate (Section 4.3.3) is offline and was not registered.

**6.5 Samples are not random draws.** Closures are counted by mission and are not independent within a mission; the earlier recordings span two camera-rate settings and several versions of the controller and alignment code; seven of them were flown with the primary filter anchored to ground truth until 2026-10-08, which makes the trajectories better than a GPS-free vehicle's although SLAM itself uses only the cameras. A closure that undoes an earlier error is also a rotation of the same size, so only the first closure in a mission is classified against an uncontaminated frame.

**6.6 Dead reckoning on a lap.** A heading error displaces an estimate in proportion to its distance from where heading was last known, and an orbit never takes the vehicle far from where it began (0.96 degrees over 131 m of straight line would be 2.2 m; at most 0.6 m here). Dead reckoning's strong showing is a statement about single-lap missions with this DVL model, which has scale and misalignment error but no temperature effects, bottom-lock loss or beam failures, and whose 0.5 degree misalignment budget is our assumption (Section 3.2). We did not find where SLAM starts to help: the gyro-drift experiment built for that was void because dead reckoning stayed under the criterion, and the planned three further DVL units (seeds 29, 3, 26) have not been flown, so the dependence of fusion's value on DVL quality rests on three units.

**6.7 The inspection-product score** uses the simulator's own CAD, sonar noise that is Gaussian, clear water, and a return set defined under the true pose. The comparison between navigation sources is not affected by the last; the absolute numbers are.

**6.8 The offline closure checks** (Section 4.6) were evaluated on recorded closures with drift injected into the true track under a stated model. The chord check was specified after the five wrong-face chords had been seen and is in-sample. None was run in the loop.

**6.9 Data hygiene.** Some recordings are mislabelled: one named for a DVL seed had no systematic DVL error, and one named dead reckoning had SLAM in the loop. Both were found by checking data against label and are excluded from the position and reconstruction results (Appendix A). The label in a bag's name cannot be trusted; every flight reported here has its DVL unit recovered from the bag.

**6.10 Literature.** The search for prior work was by web search, not a systematic review. Every statement that something has not been done is conditional on it. Reference [1] is a project report whose cost figures are the project's own; [2] is a 2016 review that is about subsea pipelines and cables, not jackets; [5] is the manufacturer's product datasheet, not a peer-reviewed source; [44] is an arXiv preprint. The claims they support are stated as practice, not as a quantified cost.

---

## 7. Conclusion

We asked what camera SLAM does to the navigation of an inspection vehicle that already has a velocity log, an inertial unit and a depth sensor, around a four-fold symmetric jacket, and we scored the answer on the structure the vehicle delivers as well as on its position.

On one lap, in simulation, dead reckoning alone was accurate to a fraction of a metre and reconstructed the structure to a median 14 cm RMS against a 7 cm sensor floor. Stock ORB-SLAM3 added one recognisable hazard: a loop closure to the wrong face, which re-poses the estimate by 90 degrees (once 180) about the structure's axis, keeping its radius, onto a different face from the one the vehicle was at. Such closures occurred in 9 of 20 complete stock missions (45 %, 95 % interval 23 to 68 %; Table 4), more often in the recent period than earlier, over periods in which the pipeline changed, so the rate is uncertain. A vehicle that follows the raw SLAM pose through one rebuilds the structure to metres. Our bridge goes silent after such a closure. When the mission ended first (four flights) the fused estimate stayed within 1.2 m; when the bridge re-locked mid-mission (three flights) its first poses were 1.2 to 2.9 m wrong, the 5 m consistency gate admitted them, and the fused estimate ended up to 4.7 m wrong while dead reckoning stayed within 0.6 m.

Whether the symmetry causes the closures is the open question, and the pre-registered paired test of it is the centre of the paper. Two of three stock missions and none of three missions on a structure with every face made different had a wrong-face closure (p = 0.20). Three per arm can reach p = 0.05 only at the extreme split, the stock baseline was below the level the design assumed, and the registered stop rule ended the experiment. The result is consistent with a symmetry cause and does not show it. One flight with the earlier bridge did not have a wrong-face closure, which leaves open whether the closure depends on the bridge. The first control we flew was flawed by design, and we say so.

What this does not show is as important: one simulated scene, one SLAM system, a small sample with an uncertain baseline rate, and no remedy tested in the loop. The next steps are to find why the wrong-face closures fall at the seabed end of the structure, to test a tighter or adaptive consistency gate, and to repeat the study with a second structure and a second SLAM system.

---

## Figures

Files are in `eval/figures/` (PDF and PNG). Full captions, sources and regeneration commands are in `docs/paper/FIGURES.md`. Numbered in order of first citation in the text.

| Figure | File | Shows | First cited |
|---|---|---|---|
| 1 | `fig_system` | the testbed and navigation architecture | 3.1 |
| 2 | `fig_scene` | Stonefish viewer screenshots: jacket, under water, vehicle | 3.1 |
| 3 | `fig_orbslam` | ORB-SLAM3 live: current frame with tracked features; map and camera path | 3.3 |
| 4 | `fig_cad` | the jacket in 3D from the CAD, stock and L5; L5 from each face | 3.7 |
| 5 | `fig_symmetry` | CAD pair scores for L0, L4, L5; what is on each face | 3.7 |
| 6 | `closures_classes` | the twelve earlier classified closures | 4.2 |
| 7 | `fig_where` | true depth of the vehicle at each classified closure | 4.2b |
| 8 | `recon_3d`, `recon_explain` | the structure rebuilt under each source, one earlier recording (bridge A) | 4.3.1 |
| 9 | `slam_map_3d` | ORB-SLAM3's map against the CAD, before and after a wrong-face closure | 4.3.1 |
| 10 | `closures_cost` | the cost of one wrong-face closure (bridge A) | 4.3.1 |
| 11 | `fig_story` | one wrong-face closure followed through a bridge-B flight: plan view and error against time | 4.3.2 |
| 12 | `fig_nav` | filtered and dead-reckoned error through each mission | 4.3.2 |
| 13 | `fig_gate_plateau` | the bridge's published pose against dead reckoning and the 5 m gate | 4.3.3 |
| 14 | `fig_endstate` | final fused error against final distance from home | 4.3.4 |
| 15 | `fig_recon_sim2` | reconstruction RMS per flight and source | 4.4 |
| 16 | `fig_control` | closure timeline per stage-1 flight; size against turn | 4.5.3 |
| 17 | `fig_contam` | share of ORB-SLAM3's map near the jacket, stock against L5 | 4.5.5 |
| 18 | `gates_vs_drift` | three closure checks against injected drift | 4.6 |

Tables: 1 data by period; 2 dead reckoning by DVL unit; 3 closures in the bridge-B flights; 3b revisit checks; 4 census by period; 5 the bridge's output after each wrong-face closure; 6 stage 1 (generated, `live_numbers.md`); 7 reconstruction by flight (`recon_batch.py`).

## Appendix A. Data and exclusions

**Bridge-B flights** (`docs/experiments/flights_manifest.csv`; each scored by `eval/flights_summary.py`): `a50_slamref_seed{0,1,2}_20261020_*` (stock, old RISE), `a50_symL4_seed{0,1}_20261021_*` (L4, old RISE), `a50_risefix_seed0_20261021_205753` (L0 seed 0, fixed RISE), `a50_ctlL0_seed1_20261021_234743`, `a50_symL5_seed0_20261021_215607`, `a50_symL5_seed1_20261022_124631`, and the stage-1 seed-2 flights L0 and L5 (bag names in the manifest). Every flight's DVL operator was recovered from the bag and matches its seed's to at most 0.0008.

**Earlier recordings:** `DeepSight_2026090*`, `DeepSight_20260912_*`, `DeepSight_20260921_*`, `DeepSight_20260922_*` (September, filter anchored to ground truth until 10-08), `a50_dr_*` (dead reckoning, 10-09 to 10-12), `a50_slam*` (10-12 to 10-16, bridge A). Classification: `eval/closure_events.py` (`docs/experiments/PREREG_closure_events.md`).

**Excluded:** `a50_dr_seed1_20261012_200040` (named dead reckoning, had SLAM in the loop); `a50_slamref_seed2_20261014_162539` (named seed 2, measured DVL gains 1.0000: no systematic error). Both are left out of every position and reconstruction result; the second one's benign closure is still counted in Table 4 (Section 6.3 gives the rates without it).

**Table A1.** Every recording and what it is used for. Census: Table 4. Closure: the classified closure(s) (Tables 3 and 3b, Figure 6). Position: dead reckoning on a lap (Table 2, 4.1) or fused against SLAM-free (4.4). Reconstruction: the 18 dead-reckoning reconstructions of 4.1, Table 7 and the trust-SLAM counterfactual (4.3.1). Bridge: the bridge-B analysis of 4.3.2 to 4.3.4. "-" means not used. The September bags do not record the DVL; their "white noise" entry is from the code of that period (commit 0281ecc, 2026-09-22), whose DVL bridge has no scale or misalignment terms (added on 2026-10-09, commit 3e54ad6), and their filter was anchored to ground truth, so the DVL did not affect their position.

| recording | date | structure, pipeline | DVL unit | census | closure | position | reconstruction | bridge | stage 1 |
|---|---|---|---|---|---|---|---|---|---|
| `DeepSight_20260905_141654` | 09-05 | stock, anchored filter | white noise | yes | benign | - | - | - | - |
| `DeepSight_20260905_173822` | 09-05 | stock, anchored filter | white noise | yes | benign | - | - | - | - |
| `DeepSight_20260905_201108` | 09-05 | stock, anchored filter | white noise | yes | wrong-face (776 s), other-large | - | trust-SLAM | - | - |
| `DeepSight_20260912_132725` | 09-12 | stock, anchored filter | white noise | yes | benign | - | - | - | - |
| `DeepSight_20260921_232316` | 09-21 | stock, anchored filter | white noise | yes | wrong-face (642 s) | - | trust-SLAM | - | - |
| `DeepSight_20260922_000133` | 09-22 | stock, anchored filter | white noise | yes | wrong-face (561 s) | - | trust-SLAM | - | - |
| `DeepSight_20260922_230945` | 09-22 | stock, anchored filter, stopped at 616 s | white noise | no (incomplete) | wrong-face (525 s) | - | trust-SLAM | - | - |
| `DeepSight_20260904_161944`, `_20260905_003615` | 09-04, 09-05 | stock | white noise | no (no console log) | - | - | - | - | - |
| three recordings of 29 to 310 s | 09-05 to 09-22 | aborted | - | no | - | - | - | - | - |
| `a50_dr_seed0_20261009_210804` | 10-09 | no SLAM | white noise | - | - | Table 2 | DR | - | - |
| `a50_dr_seed0_20261009_221008` | 10-09 | no SLAM | 0 | - | - | Table 2 | DR | - | - |
| `a50_dr_seed1_20261012_173121` | 10-12 | no SLAM | 1 | - | - | Table 2 | DR | - | - |
| `a50_dr_seed2_20261012_191733` | 10-12 | no SLAM | 2 | - | - | Table 2 | DR | - | - |
| `a50_dr_seed1_20261012_200040` | 10-12 | SLAM in the loop, pose not recorded | 1 | no | - | excluded | excluded | - | - |
| `a50_slam_seed1_20261012_230457` | 10-12 | stock, circular alignment | 1 | yes | none | - | - | - | - |
| `a50_slamref_seed1_20261014_140856` | 10-14 | stock, bridge A | 1 | yes | none | - | DR | - | - |
| `a50_slamref_seed0_20261014_153610` | 10-14 | stock, bridge A | 0 | yes | benign | - | DR | - | - |
| `a50_slamref_seed2_20261014_162539` | 10-14 | stock, bridge A | none (mislabelled) | yes | benign | excluded | excluded | - | - |
| `a50_slamref_seed2_20261014_200359` | 10-14 | stock, bridge A | 2 | yes | none | - | DR | - | - |
| `a50_slamref12_seed0_20261015_180532` | 10-15 | stock, bridge A, SLAM at 12 Hz | 0 | yes | none | - | DR | - | - |
| `a50_slamb30_seed0_20261015_204105` | 10-15 | stock, wide-baseline cameras | 0 | no (other camera) | - | - | DR | - | - |
| `a50_slamned_seed0_20261016_160033` | 10-16 | stock, bridge A, frame fixed | 0 | yes | wrong-face (573 s) | - | DR, trust-SLAM; Figures 8 to 10 | - | - |
| `a50_slamdrift_seed0_20261016_190745` | 10-16 | stock, bridge A, 3 deg/h yaw drift | 0 | yes | benign | heading drift (4.1) | DR | - | - |
| `a50_slamref_seed{0,1,2}_20261020_*` | 10-20 | stock, bridge B, old RISE | 0, 1, 2 | yes | wrong-face (1018 to 1034 s) | 4.4 | DR, Table 7 | yes | - |
| `a50_symL4_seed{0,1}_20261021_*` | 10-21 | L4, bridge B, old RISE | 0, 1 | no (modified) | wrong-face (331, 336 s) | 4.4 | - | yes | - (first control) |
| `a50_risefix_seed0_20261021_205753` (L0 seed 0) | 10-21 | stock, bridge B, fixed RISE | 0 | yes | wrong-face (758 s), other-large | 4.4 | DR, Table 7 | yes; Figure 11 | yes |
| `a50_ctlL0_seed1_20261021_234743`, `a50_ctlL0_seed2_*` | 10-21, 10-22 | stock, bridge B, fixed RISE | 1, 2 | yes | wrong-face (1025 s); benign | 4.4 | DR, Table 7 | yes | yes |
| `a50_symL5_seed{0,1,2}_*` | 10-21, 10-22 | L5, bridge B, fixed RISE | 0, 1, 2 | no (modified) | benign | 4.4 | excluded (L5) | yes | yes |
| `a50_oldbridge_seed1_20261022_200605` | 10-22 | stock, bridge A, fixed RISE | 1 | separate | benign | 4.4 | DR, Table 7 | - | no (4.5.3b) |

**Not flown:** the circular-alignment flight (`slamcirc`), so the "41 m against 1.7 m" figure of the earlier draft is not re-measured and is not used; the three further DVL units (seeds 29, 3, 26); the gyro-drift sweep.

## Appendix B. Deviations from the pre-registrations

1. **L0 seed 0 of stage 1 was flown before the L0/L5 pre-registration** (bag `a50_risefix_seed0_20261021_205753`), as a test of the RISE fix. It is disclosed in the registration and counted as the stage-1 L0 seed-0 flight because it used the fixed controller, the stock scenario and DVL seed 0.
2. **The endpoint changed** from "the first closure is face-aliased" (L4 registration) to "at least one face-aliased closure before RETURN_HOME" (L5 registration), because closures come in pairs and the first is not special. The L4 flights are not scored under the new endpoint (they had one closure each, both aliased).
3. **The RISE controller changed between the re-measure/L4 flights and stage 1.** Old-RISE flights are not used as comparators for the paired test.
4. **The L4 control was stopped by its registered gate** after two flights that both aliased, and it was then found to have been designed badly (the 0-1 pair stayed at 0.947). A second control (L5) was designed afterwards, with its criterion fixed before any L5 flight, and registered before the first L5 flight.
5. **The time-box on the control was extended by the author** (the deadline of 18:00 on 2026-10-21 was dropped); the statistical stop rule (Gate B) is unchanged.
6. **Exploratory, after seeing the data:** the bridge-silence and first-pose analysis (`eval/gate_plateau.py`, `eval/jump_attribution.py`), the pre-closure RMS comparison (Section 4.4), and the map-contamination check (Section 4.5.5). They were not registered and are reported as exploratory.
7. **The three predictions of the 2026-10-21 run sheet** (seed 29: dead reckoning beats the fused filter; seeds 3 and 26: the fused filter beats it; any wrong-face closure is contained) concern flights not made, except the third, which the bridge-silence analysis shows was not a test of containment (Section 4.3.2).
8. **Gate B stopped stage 1.** Stock was 2 of 3, below the registered 3 of 3, so stage 2 (seeds 29, 3, 26 in both arms) was not flown, as the registration requires.
9. **One flight with the earlier bridge** (`a50_oldbridge_seed1_20261022_200605`, stock, DVL unit 1) was flown after Gate B, as a single discriminating run for the bridge. It is outside the registered design and is not counted in the Fisher test.
10. **Exploratory, after seeing the data:** the per-mission rate by period (Section 4.2), the depth and motion description of where closures fall (Section 4.2b; the 20 m threshold was chosen after looking), and the outcome of the revisit checks, where P1 as registered failed for four of seven wrong-face closures (Section 3.5). The classifier was not changed. The census denominator was corrected on 2026-10-24: earlier drafts counted only missions with a classified closure (10 of 17); the census now counts every complete stock mission with SLAM running and a console log (9 of 20).

## Appendix C. Reproducing every number

```bash
# per flight: the DVL unit, the closures, the errors, the endpoint, the stage-1 test
python3 eval/flights_summary.py                  # writes docs/paper/live_numbers.md
python3 eval/gate_plateau.py                     # bridge output against the 5 m gate and dead reckoning
python3 eval/closure_where.py                    # where in the mission each closure falls
python3 eval/closure_revisit.py FILE             # ground-truth revisit checks
python3 eval/jump_attribution.py                 # SLAM events, map collapses, silences
python3 eval/recon_batch.py                      # sonar reconstruction and the trust-SLAM counterfactual
python3 eval/pre_window_ratio.py                 # Section 4.4: fused against SLAM-free position RMS before the first closure
python3 eval/analyze_anchorfree.py BAG           # Table 2 and the heading-drift numbers of 4.1 (one dead-reckoning bag at a time)
python3 eval/gate_sweep.py                       # Section 4.6 and Figure 18: closure checks against injected drift
python3 eval/slam_map_recon.py BAG --closure-t T --no-json --out eval/figures/slam_map_contam_X.npz
python3 eval/closure_events.py BAG [LOG]         # one flight's closures
python3 eval/verify_dvl_model.py BAG SEED        # DVL unit against seed
python3 eval/fig_symmetry.py                     # CAD pair scores (cached; --refresh)
python3 eval/fig_scene.py; python3 eval/fig_orbslam.py; python3 eval/fig_cad.py   # Figures 2, 3, 4
python3 eval/fig_story.py                        # Figure 11 (reads the L0 seed 0 bag)
bash docs/paper/make_paper_assets.sh             # everything above plus every figure
```

## Status, not for the manuscript

All flights are in and every [LIVE] item is filled. If a flight is ever added: add a row to `docs/experiments/flights_manifest.csv`, run `bash docs/paper/make_paper_assets.sh`, and recheck the abstract, Tables 3 to 7, Sections 4.2 to 4.5, 5.1, 6.3 and 7, and `CLAIMS_AND_EVIDENCE.md`.

Rechecked against the bags on 2026-10-24: Table 2 and the heading-drift numbers of 4.1 and 6.6 (`eval/analyze_anchorfree.py`, and the SLAM-free filter of the 3 degree per hour bag: 0.20 m RMS, 37.9 m farthest, 3.8 m at the end), and every number in 4.5.5 including the fitted scales 0.93 and 1.01 (`eval/slam_map_recon.py`). All match. Licence of the jacket mesh (a commercial asset) to be checked before Figures 2, 3, 7 and 8 are published.

Unresolved before submission: most citation judgements are from knowledge of the papers, not re-read (`CITATION_CHECK.md`); the 0.5 degree DVL misalignment budget has no citation; section numbers and figure numbers are placeholders until the format is chosen.

---

## Reference list

The numbering follows `DeepSight_Introduction_LitSurvey_v4.docx`. Cited above: [1]–[5] (motivation and sensors; [1] ATLANTIS (EU Horizon 2020 grant 871571), Periodic Reporting for period 2 (2021-07-01 to 2023-12-31), CORDIS, doi:10.3030/871571; [2] C. Mai, S. Pedersen, L. Hansen, K. L. Jepsen, Z. Yang, "Subsea infrastructure inspection: a review study", 2016 IEEE USYS, pp. 71-76, doi:10.1109/USYS.2016.7893928, [5] Water Linked AS, DVL A50 product datasheet (waterlinked.com/datasheets/dvl-a50, PDF dated 2026-08-06, accessed 2026-10-24), long-term accuracy +-1.01 % standard / +-0.1 % performance, 1 MHz, 22.5 degree beams; new in this draft: [43] X. Wang, X. Fan, P. Shi, J. Ni, Z. Zhou, "An overview of key SLAM technologies for underwater scenes", Remote Sensing 15(10):2496, 2023, doi:10.3390/rs15102496; [44] Olivastri et al. 2024; [45] F. Vorpahl, W. Popko, D. Kaufer, "Description of a basic model of the 'UpWind reference jacket' for code comparison in the OC4 project under IEA Wind Annex 30", Fraunhofer IWES technical report, 19 July 2013; [46] K. Tang, J. Wang, W. Li, W. Wu, "A novel INS and Doppler sensors calibration method for long range underwater vehicle navigation", Sensors 13(11):14583-14600, 2013, doi:10.3390/s131114583; [47] G. Troni, L. L. Whitcomb, "Advances in in situ alignment calibration of Doppler and high/low-end attitude sensors for underwater vehicle navigation: theory and experimental evaluation", J. Field Robotics 32(5):655-674, 2015, doi:10.1002/rob.21551), [7] ORB-SLAM3, [8] Lowry et al., [9] Neira and Tardós, [11] Gálvez-López and Tardós, [17] Köser and Frese, [18] Heshmat et al., [21] SVIn2, [23] FAB-MAP, [24] robot_localization, [27] Sturm et al., [28] Zhang and Scaramuzza, [30] Nosek et al., [32] Stonefish, [33] Sünderhauf and Protzel, [34] Olson and Agarwal, [35] Mangelson et al., [36]–[40] symmetry in SLAM and localisation, [41] ROVER, [42] RANSAC. Full entries are in the v4 document; the sentence-level support of each is recorded in `CITATION_CHECK.md`.
