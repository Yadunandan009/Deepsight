# Task #15 — Why ORB-SLAM3's Map Fragments Mid-Mission: Root Cause and Geometric Verification

## Abstract

Task #15 ("investigate and repair post-final-face point-cloud distortion") has been open since at least 2026-09-05, tracked alongside a separate, unrelated EKF yaw-instability bug that dominated most debugging attention up to this point. That EKF bug (a missing `base_link`↔`imu_filter` static transform silently dropping all IMU corrections, letting the fused pitch state free-drift into the filter's own Euler-angle gimbal-lock singularity) was root-caused and fixed on 2026-09-21/22, and confirmed via bag analysis to fully eliminate the yaw-glitch pattern that had been chasing this project since early September. That fix also, for the first time, produced a mission log completely free of controller-side yaw disputes — which made it possible to see Task #15's actual problem cleanly, on its own, with no confound from the EKF bug.

With the EKF issue out of the way, a fresh full-mission run (`DeepSight_20260922_000133`) still shows ORB-SLAM3 losing tracking and abandoning its own map twice, each time producing a downstream cascade of `slam_pose_bridge` re-alignments with wildly uncorrelated rotation offsets. ORB-SLAM3's own console log shows both incidents immediately preceded by its loop-closure/map-merge machinery (`*Loop detected`, `*Merge detected`, `Local Mapping STOP`), not by anything the vehicle was doing — ground-truth angular velocity was checked directly from the bag at all six SLAM-instability events recorded that run, and five of six occurred during essentially calm periods (yaw rate under 2.5°/s), ruling out a motion-triggered explanation.

Two rounds of Iterative Closest Point (ICP) verification, following the same independent-geometry method established in Task A, were run against this specific failure. The first (comparing ORB-SLAM3's whole accumulated map against each of the turbine's four faces) was inconclusive on its own. The second, sharper test — isolating only the freshly-triangulated points from the affected dwell — showed those points correctly matching their own true face's geometry, undercutting a naive "SLAM's 3D triangulation is just wrong" explanation. A third, decisive test settled the question: the turbine's four faces' *real* sonar-derived geometry, ICP-registered against each other independent of anything SLAM did, match with near-perfect fitness (0.996–1.000) and RMSE within centimeters of the sonar noise floor. **The structure itself is close to radially symmetric across all four faces.** This is a property of the mission's own 3D scenario geometry, not a SLAM tuning defect — any vision-only place-recognition system pointed at this structure is at structural, not incidental, risk of false loop closures, independent of feature-matching thresholds.

---

## 1. Background

### 1.1 Task #15's status entering this investigation

Per the 2026-09-05 handoff doc, Task #15 was "in progress," with two prior mitigation attempts (an IMU-based yaw-glitch corroboration swap, and a yaw-command slew-rate limiter) both reverted after live telemetry showed each made things worse. At that point, the working theory was that Task #15 (SLAM losing tracking near mission end) and Task #16 (a persistent DESCEND-phase rotation glitch, tracked separately) were related, possibly sharing a cause, since both symptoms tended to co-occur in the same bad runs.

### 1.2 Why the EKF fix changes what Task #15 evidence actually means

Task #16's real root cause — found this session — turned out to be structural and unrelated to SLAM entirely: `robot_localization` requires a TF transform between `bluerov2/imu_filter` (the frame on every `/bluerov2/imu` message) and `bluerov2/base_link` to fuse IMU data at all. Nothing in this launch stack ever published it (`stonefish_ros2`'s own `PublishTF` function exists in C++ but is never called; no `robot_state_publisher`/URDF is used). Every IMU correction had therefore been silently failing at the transform-lookup stage for the entire project's history, leaving roll/pitch to run open-loop and occasionally random-walk into the EKF's own unguarded `1/cos(pitch)` singularity (confirmed by reading `ekf.cpp` directly) — producing the exact "yaw jumps by ~180°, roll/yaw discontinuously pop" signature this project had been fighting under Task #16. Adding the missing static transform (`bluerov2_sim.py`) fixed it: a full-mission bag taken afterward shows **zero** yaw jumps over 90° in either ground truth or the fused output, and filtered pitch tracking ground truth to within 0.2°, for the entire ~17-minute mission.

This matters for Task #15 specifically because it means: any SLAM-crash evidence gathered *before* this fix landed was gathered in the presence of a second, independent, real instability (the EKF's own periodic ~180° yaw hallucination). Historical observations that "SLAM crashes and yaw glitches happen together" were confounded — it was never established which one caused which, or whether they were two symptoms of a shared trigger. The run analyzed below is the first one where that confound doesn't exist: the controller log for the full mission shows zero yaw-glitch-guard disputes, from `MISSION START` through `RISE`. Whatever SLAM instability shows up in this run is Task #15's problem, standing on its own.

---

## 2. Evidence: the map is fragmenting on its own, not because of vehicle motion

### 2.1 Downstream symptoms (already visible in `slam_pose_bridge`/`adaptive_fusion_node`)

The full-mission run's console output shows a repeating pattern roughly ten times over the mission:

```
SLAM alignment solved: theta=+153.1deg ...
SLAM alignment refit: theta=+142.7deg ...          (drifts smoothly over ~45s)
SLAM pose rejected: 5.7m from EKF (> 5.0m) streak=27
SLAM alignment stale after 40 consecutive rejections — re-aligning from scratch
SLAM alignment solved: theta=+46.1deg ...           (completely uncorrelated with +142.7deg)
```

If the true SLAM-map-to-world rotation offset were physically fixed (as it should be, since the physical world doesn't rotate), re-alignment from scratch should keep landing near the *same* value each time it resets. Instead, the recovered `theta` after each reset is essentially uncorrelated with the value before it (+153°→+46°→…→-136°→+63°→+19°→-158°→-53°→+50°, across the mission), and two explicit `Map point crash detected` events appear in the same log (`203662→6554`, `395288→4600`) — catastrophic, >95% drops in the accumulated map's point count.

### 2.2 Ruling out vehicle motion as the trigger

The obvious first hypothesis — fast rotation causing motion blur / feature-tracking loss — was tested directly against ground truth. For each of the two map-point crashes and four explicit `SLAM jump detected` resets, ground-truth angular velocity and body speed were pulled from the bag (`/bluerov2/odometry`) in a 14-second window bracketing the event:

| Event | max \|yaw_rate\| | max body speed |
|---|---|---|
| Map crash 203662→6554 | 1.3°/s | 0.147 m/s |
| Map crash 395288→4600 | 41.1°/s (peaks ~8s *before* the crash, back near 0 by crash time) | 0.145 m/s |
| SLAM jump 26.20m | 1.5°/s | 0.154 m/s |
| SLAM jump 5.04m | 1.3°/s | 0.154 m/s |
| SLAM jump 7.83m | 2.2°/s | 0.228 m/s |
| SLAM jump 17.61m | 1.3°/s | 0.151 m/s |

Five of six events occur during periods with no meaningful vehicle motion at all. Only the last map crash coincides with a real rotation (the expected, geometrically-necessary RISE yaw swing), and even there the rotation had already settled several seconds before tracking actually failed. **This rules out motion-triggered tracking loss as the general explanation** — whatever's causing this is happening on ORB-SLAM3's own initiative, not in response to what the vehicle is doing.

### 2.3 ORB-SLAM3's own console log names the mechanism directly

`slam_atlas/orb_slam3_console_20260922_000119.log` shows, for the first map-point crash:

```
[t=1790016054.36] *Loop detected
[t=1790016054.36] Local Mapping STOP
[t=1790016058.36] Local Mapping RELEASE
[t=1790016059.36] Fail to track local map!
[t=1790016059.36] ORB-SLAM failed: Tracking LOST.
   ... (37 consecutive Fail-to-track/LOST pairs) ...
Stored map with ID: 0
Creation of new map with id: 1
New Map created with 770 points
```

and for the second:

```
[t=1790016511.36] *Merge detected
[t=1790016511.36] Local Mapping STOP, "Change to map with id: 0"
   ... tracking frequency drops to 0 frames/sec while "Waiting for merge to finish." repeats ...
[t=1790016513.36] Local Mapping RELEASE / Merge finished!
   ... tracking appears to recover normally for ~9s ...
[t=1790016522.36] Fail to track local map! / Tracking LOST.  (never recovers before process shutdown)
```

A third, unrelated `Local Mapping STOP`/`RELEASE` pair (line 1576–1577, one line apart, no `Loop`/`Merge detected` tag) causes no disruption at all — confirming that ordinary local-mapping synchronization pauses are routine and harmless, and it is specifically the ones triggered by loop-closure/merge detection that precede catastrophic tracking loss. The first incident's `Creation of new map with id: 1` line is the direct mechanistic explanation for §2.1's uncorrelated re-alignment thetas: `slam_pose_bridge` isn't failing to re-align a continuous map — it's correctly re-aligning a **genuinely new, disconnected coordinate frame** with no relationship to the old one.

---

## 3. Geometric verification: is this a false loop closure, and is it face-specific?

Following the same method established in Task A (§1.3 of that report): reanchor ORB-SLAM3 map points to world frame per-point, using the nearest ground-truth trajectory sample's own heading at that instant (immune to whatever SLAM's own internal frame is doing), then compare against independent sonar-derived reference geometry via bootstrapped ICP (50 resamples, 80% subsample with replacement; fitness = fraction of points within 1.0 m after alignment, RMSE among inliers).

### 3.1 First pass — whole accumulated map vs. all four faces

Using map-0's entire accumulated point cloud (subsampled 235k→15k points) from just before the first crash, reanchored to world frame, against each face's real (frustum-restricted sonar) reference:

| face | fitness | RMSE (m) |
|---|---|---|
| 0 | 0.180 ± 0.009 | 0.630 ± 0.011 |
| 1 | 0.140 ± 0.006 | 0.619 ± 0.015 |
| 2 (vehicle was actually here) | 0.153 ± 0.011 | 0.635 ± 0.013 |
| 3 | 0.153 ± 0.008 | 0.619 ± 0.020 |

Face 0 scores marginally highest — but fitness is weak everywhere (14–18%), and face 0 having accumulated the most points/longest dwell time (it was visited first) is a confound: a random subsample of the *whole* map is disproportionately face-0-flavored regardless of any true aliasing effect. This test alone is suggestive but not clean.

### 3.2 Sharper pass — only the newly-triangulated points from the affected dwell

Isolating specifically the points added during the face-2 approach/orbit, before the loop-closure corruption (map-at-orbit-start minus map-at-crash-onset, the same nearest-neighbor-distance method as Task A §2 Step 2), reanchored and subsampled to 7,500 points:

| face | fitness | RMSE (m) |
|---|---|---|
| 0 | 0.401 ± 0.011 | 0.646 ± 0.012 |
| 1 | 0.438 ± 0.107 | 0.485 ± 0.025 |
| 2 (correct answer — this is where the data was captured) | 0.560 ± 0.090 | 0.531 ± 0.036 |
| 3 | 0.183 ± 0.046 | 0.790 ± 0.036 |

Here, face 2 — the *correct* answer — wins, with face 1 a fairly close second (overlapping bootstrap ranges) and faces 0/3 clearly behind. Taken alone, this would suggest SLAM's own triangulation is basically sound (good) with some residual ambiguity specifically between neighboring faces 1 and 2 (a narrower, "adjacent-face-confusion" story). That reading turns out to be incomplete — see §3.3.

### 3.3 Decisive test — is the structure itself self-similar, independent of SLAM entirely?

To test whether §3.2's face1/face2 ambiguity reflects something special about those two faces specifically, or is a general property of the whole structure, each face's *real* sonar reference cloud (not SLAM's output at all) was converted into a shared face-local frame (de-rotated by that face's own bearing, so "facing the turbine" is a common local convention for every face) and ICP-registered against every other face's local-frame cloud, with a free rigid search:

| compare | fitness | RMSE (m) |
|---|---|---|
| face2 vs face0 | 1.000 ± 0.000 | 0.102 ± 0.002 |
| face2 vs face1 | 1.000 ± 0.000 | 0.080 ± 0.002 |
| face2 vs face3 | 0.996 ± 0.001 | 0.085 ± 0.004 |
| face2 vs face2 (self, ceiling) | 1.000 ± 0.000 | 0.000 ± 0.000 |

Every cross-face comparison registers at essentially the self-match ceiling — RMSE 8–10 cm above a 0 cm ceiling, almost certainly within the sonar reconstruction's own noise floor rather than a real structural difference. **All four faces of this turbine are, geometrically, close to indistinguishable from each other**, not just face 1 and face 2. This is a fact about the mission's own 3D scenario (the lattice/jacket structure's bracing pattern repeats with close to exact radial symmetry), established without reference to anything ORB-SLAM3 did.

This reframes §3.2's result: since *every* face is nearly geometrically identical, the fitness differences observed there (face 2 highest, face 1 close second, faces 0/3 behind) are best read as measurement noise riding on top of near-total structural symmetry, not evidence of a special face-1/face-2 relationship. The fact that the correct face (2) still edged out the others is a mildly reassuring sign that SLAM's own stereo triangulation isn't grossly broken — but it is not evidence against the false-loop-closure hypothesis, because on a structure this symmetric, *any* face's freshly-triangulated points would be expected to plausibly register against *any other* face's reference almost as well, essentially by construction.

---

## 4. Discussion

**This is not a SLAM tuning defect.** A false loop closure on a structure this close to perfectly self-similar across all viewing directions is close to expected behavior for a vision-only, appearance-based place-recognition system (ORB-SLAM3's DBoW2/3 bag-of-words matcher), not an edge case a feature-count or similarity-threshold adjustment would meaningfully reduce. The structure itself removes almost all of the visual/geometric signal a place-recognizer needs to correctly reject a false candidate.

**Where the actual failure happens.** `slam_pose_bridge.py` already gates its own downstream re-alignment against the EKF (rejecting a SLAM pose more than 5 m from the EKF's own estimate, per §2.1's log lines) — but that gate sits *after* ORB-SLAM3 has already accepted the loop closure/merge and corrupted its own internal map and pose graph. By the time `slam_pose_bridge` sees bad data, the damage (map abandonment, in the first incident; an unrecovered tracking loss, in the second) is already done inside ORB-SLAM3 itself. The existing downstream gate protects the EKF/controller from *consuming* corrupted SLAM output — confirmed working, since the mission completed normally despite both incidents — but it does nothing to protect ORB-SLAM3's own map from the corruption in the first place.

**Practical implication.** Fixing this would mean gating loop-closure/map-merge *acceptance* inside (or immediately downstream of) ORB-SLAM3 against an independent consistency check — e.g., rejecting a proposed loop closure/merge outright if the pose correction it implies disagrees with the EKF's own position estimate by more than some threshold, mirroring what `slam_pose_bridge` already does for ordinary pose updates, but applied *before* the map is altered rather than after. The alternative, blunter option is disabling loop closure/map merging for this specific structure profile entirely and relying on continuous frame-to-frame tracking without global correction — a real tradeoff (accepting slow drift instead of risking catastrophic, discrete map corruption), not a clear win either way, and a decision this write-up deliberately leaves open rather than recommending unilaterally.

---

## 5. Limitations

- **Two incidents, one mission run.** The console-log evidence (§2.3) and the motion-correlation table (§2.2) are drawn from a single mission. The mechanism (loop/merge detection → STOP/RELEASE → tracking loss) is clean and specific enough to trust on its own terms, but a claim about *frequency* (how often this happens per mission, or whether every loop-closure attempt is this destructive vs. only some) would need more runs.
- **The sonar reference mosaic predates this run** (built 2026-09-02, from an earlier mission) and is treated as fixed ground truth with no uncertainty model of its own, the same caveat Task A's write-up already flagged for its own use of this same reference.
- **§3.1 and §3.2 use different point-set sizes and different windows**, so their fitness numbers are not directly comparable to each other in absolute terms — only the *relative* pattern within each table (which face wins, and by how much) is meaningful, exactly as Task A's own write-up cautions for its absolute fitness values.
- **This does not establish whether the specific loop closure/merge detected on 2026-09-22 was geometrically *false*** in the sense of matching the wrong 3D location — given §3.3's finding, that question may not even be well-posed for this structure, since "correct" and "false" 3D matches are nearly indistinguishable by construction. What is established is that the *consequence* (map abandonment / unrecovered tracking loss) occurs immediately after loop/merge detection, independent of vehicle motion, on a structure geometrically capable of producing exactly this kind of confusion.

## 6. Conclusion

Task #15's SLAM map fragmentation is real, reproducible within a single mission (twice), and now cleanly separable from the unrelated EKF yaw-instability bug that used to co-occur with it. ORB-SLAM3's own console log directly names the mechanism: loop-closure/map-merge detection, followed immediately by tracking loss and (in one case) outright map abandonment — not vehicle motion, confirmed against ground-truth angular velocity at every recorded instability event this mission. Geometric ICP verification, extending Task A's method, shows this structure's four faces are close to radially symmetric with each other (fitness ≥0.996, RMSE within centimeters of the reference's own noise floor) — a property of the scenario's own 3D geometry that makes any vision-only place-recognition system structurally vulnerable here, not a symptom fixable by retuning ORB-SLAM3's feature-matching parameters. The existing downstream defense (`slam_pose_bridge`'s EKF-consistency rejection) is confirmed working as a *last line of defense*, but the fix that would actually prevent the map corruption itself would need to gate loop-closure/merge *acceptance*, not just its downstream consumption — left as an open decision (§4) rather than a recommendation made unilaterally here.
