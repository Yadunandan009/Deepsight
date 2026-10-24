# Adaptive consistency gate: brief and review checklist (after submission)

Not for the paper. Written 2026-10-23. Roles: see the status section at the end. The gate has been written and tested offline; it has not flown.

## 1. What problem it addresses

After a wrong-face closure the bridge goes silent. When it re-locks, its first poses can be 1 to 3 m wrong, and the consistency gate (`EKF_CONSISTENCY_MAX = 5.0` m against the SLAM-free reference filter) lets them in. The filter follows them and nothing pulls it back. In 7 of 11 bridge-B flights the published pose reached 4.69 to 5.00 m from dead reckoning, while dead reckoning itself stayed within 0.65 m.

Offline replay on the 11 flights (`eval/gate_replay.py`, not closed loop): of 5,893 published poses more than 1 m from the truth, a 1.5 m gate rejects 76 %, a 1.0 m gate 97 %; wrongly rejected good poses: none at 1.5 m, 0.12 % at 1.0 m; the 5 m gate rejects 0.02 %.

## 2. The trap: the reference filter's own covariance is not usable as is

Read from `/bluerov2/odometry/dr` in one stage-1 bag: the published position variance grows from 1.8 m^2 to 62 m^2 over the mission (sigma about 1.3 m to 7.9 m), while the real dead-reckoning error stayed under 0.65 m. A chi-squared test against that covariance would be about as wide as the current gate by the end of the mission. So "chi-squared against the reference filter's covariance" does not work until the covariance is calibrated against the truth (a consistency check: the normalised error squared, NEES, over many flights). Decide first whether to calibrate that covariance, or to build the gate on a separate error budget.

Also: the DVL's published covariance is deliberately not inflated for the scale and misalignment errors (Section 3.2 of the paper), so the covariance misses the term that actually dominates dead reckoning. An error budget that grows with distance travelled (a fraction of a percent to about 1.5 % of path, plus a floor) is the honest model for the gate width.

## 3. Design questions to settle before writing

1. **Statistic.** Mahalanobis distance of the SLAM pose from the reference, with what combined covariance: reference error budget plus the SLAM alignment uncertainty? At a re-lock the alignment uncertainty is large (short, vertical window); a poorly conditioned fit should widen the gate, not be trusted.
2. **Width over time.** Narrow when dead reckoning is fresh (start, and just after a trusted correction), wider as path length grows. What is the floor, and the growth rate per metre?
3. **What a rejection does.** Today 40 consecutive rejections reset the alignment and re-align from scratch, and the first re-lock is accepted if it is within 5 m. With a narrow gate that path must not accept the first re-lock blindly: require a probation window of consistent poses before publishing.
4. **Recovery.** If dead reckoning has drifted, honest SLAM will be rejected. A sequential test on the innovation sequence (several consecutive poses agreeing with each other and disagreeing with the reference by a steady offset) is one way to let a real correction back in. Without it a narrow gate is unsafe on long missions.
5. **The reference must stay SLAM-free** (`dr_reference:=true`). Never feed it the primary filter, and never the ground-truth odometry topic.
6. **Interaction with the jump reset and the map-collapse detector**, which fire first in some cases. List the order of checks and what each one does to the alignment.

## 4. Review checklist (what I will check in your code)

- The gate statistic uses the reference filter and a budget that grows with path length, and the units are consistent (m versus m^2).
- Time alignment of the SLAM pose and the reference sample (interpolation, not nearest).
- No path from rejected poses into the filter, and no path from the primary filter into the gate.
- Behaviour with no reference message (startup, dropout), and after a reset.
- Probation and recovery logic cannot deadlock (permanently silent) or oscillate.
- Constants are named, documented, and have units.
- A log line for every accept, reject and reset with the statistic and the threshold, so a bag can be scored afterwards.

## 5. Offline test plan (I can build the harness; it is analysis, not estimator code)

Replay the recorded raw SLAM poses, the reference odometry and the truth through the candidate gate (open loop), on the 11 stage-1 and re-measure flights:

- **Primary:** share of published poses more than 1 m from the truth that are rejected, and share of poses within 1 m that are wrongly rejected. Fix the pass line before running (suggestion: at least 95 % and at most 0.5 %).
- **Recovery:** time to the first accepted pose after each re-lock, and how many poses were bad when it did.
- **Drift injection:** inject linear dead-reckoning drift of 5, 20 and 50 m at the end of the mission (the model in `eval/gate_sweep.py`) and check that honest corrections are accepted. This is the test that stops a narrow fixed gate from passing.
- **Not enough on its own:** a closed-loop flight is still needed, because rejection changes the filter, the path and later SLAM output.

## 6. Longer roadmap after the paper is sent (your list, with what each buys)

| item | what it would add to this study | caveat |
|---|---|---|
| Particle filter (or a clustered one) instead of the EKF | the multi-hypothesis case fits the finding: keep both faces alive until evidence separates them (the symmetric-map localisation literature does this) | the scale and misalignment errors are systematic; a PF needs them in the state or the model, or it is fooled the same way |
| Fossen marine craft model in the prediction | a dynamics-based prediction when DVL is poor, and a better simulated vehicle | it does not address the wrong-face closure itself; it improves prediction between measurements |
| Multi-sensor fusion (sonar place recognition with the camera) | a second, independent view that does not share the camera's symmetry failure | a sonar sees geometry, and the structure is geometrically symmetric too (the CAD pair score is 1.0), so it may fail the same way |
| More structures and props (three and six legs, different appurtenances) | turns one scene into several; a dose-response over distinguishable faces is the strongest form of the control | many flights; fix the baseline rate first (stock was 10 of 17, wide interval) |
| Record camera images | tests the two candidate reasons for the failure at the seabed end (darker images, or a view of base and seabed) | one run, plus image storage |
| Real data | the standing gap | not something this plan closes |

Suggested order: images first (cheapest, answers an open "why"), then the gate offline, then the gate in the loop, then a second structure, then the filter change.

---

## Status, 2026-10-23: the gate is written (by Claude, to be reviewed by you)

Roles changed: you said the red-zone rule does not apply to this project, so I wrote it.

**Files**
- `src/stonefish_bluerov2/stonefish_bluerov2/consistency_gate.py`: pure Python, no ROS. `ConsistencyGate(floor, k_drift, cap, probation_poses, probation_spread)`.
- `src/stonefish_bluerov2/scripts/slam_pose_bridge.py`: new parameters `adaptive_gate` (default false), `gate_floor` (1.0), `gate_k_drift` (0.0), `gate_probation_poses` (25), `gate_probation_spread` (8.0). With `adaptive_gate:=false` the old fixed 5 m path runs unchanged; the new module is imported only when the gate is on, so a workspace that has not been rebuilt still runs every existing configuration.
- `launch/bluerov2_sim.py`: new argument `adaptive_gate` (default false). Use `adaptive_gate:=true dr_reference:=true`.
- `eval/test_consistency_gate.py` (6 unit tests), `eval/test_bridge_gate.py` (synthetic in-process test of the bridge, no simulator), `eval/gate_replay.py` (offline replay on the 11 flights).

**Before the first flight:** `colcon build --packages-select stonefish_bluerov2` (the new module has to be installed).

**Offline results** (`eval/gate_replay.py`, pass line fixed beforehand: at least 95 % of poses more than 1 m from the truth rejected, at most 0.5 % of the rest):

| gate | bad rejected | good wrongly rejected | pass |
|---|---|---|---|
| current: fixed 5 m | 0.0 % | 0.00 % | fail |
| fixed 1.5 m | 76.4 % | 0.00 % | fail |
| fixed 1.2 m | 90.9 % | 0.01 % | fail |
| **fixed 1.0 m** | **96.6 %** | **0.12 %** | **pass** |
| 1.0 m + 0.0015 per m of path | 91.2 % | 0.03 % | fail |
| 1.0 m + 0.003 per m of path | 84.8 % | 0.01 % | fail |
| 1.0 m, probation | 96.6 % | 0.12 % | pass (probation not credited, see below) |

So the defaults are floor 1.0 m and growth 0. The growth term costs protection on the data we have and is left in the module for longer or worse-DVL missions, to be set from evidence.

**Synthetic bridge test:** after a frame jump followed by a drifting re-lock, the old gate let poses through up to 4.78 m from the truth (the real ceiling was 4.7 to 5.0 m), the new gate held every published pose within 0.73 m.

**Self-review: what I am not sure of**
1. The replay cannot see the benefit of probation (it assumes the spread condition is met) and is not closed loop. Rejection changes the filter, the path and later SLAM output. Only a flight settles it.
2. A 1.0 m floor is tight against honest poses: the largest honest disagreement seen was 1.30 m (L4 seed 1), and 113 good poses in 94,626 would be refused.
3. Why a narrow fixed gate may be safer on long missions than I first feared: the bridge refits its alignment to the reference every 5 s over a 75 s window, so slow dead-reckoning drift is absorbed with a lag of tens of seconds. The flip side: a genuine loop closure after long drift looks like a jump above 5 m and already resets the alignment to the reference, so this bridge cannot use SLAM to correct long-term dead-reckoning drift anyway. That is a design limit, not something this gate adds.
4. The probation spread (8 m) and pose count (25) are my choices, untested on real re-locks.
5. `k_drift = 0` means the gate is "adaptive" only through probation. If you want width to track a real error estimate, the reference covariance has to be calibrated first (Section 2 above).
6. The bridge's start-up now waits for probation (25 poses and 8 m of spread) before the first publish.
