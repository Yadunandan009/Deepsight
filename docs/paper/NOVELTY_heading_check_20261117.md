# Is the heading check new? Search of 2026-11-17

Verdict: **the idea of checking a loop's turn against an inertial or odometric turn is prior art.**
What is ours: where it is applied, what it is tested against, and the measurement it rests on.
Claim it as a targeted, tested application, not as a new method.

## Prior art found

| Work | What it checks | Relation to our check |
|---|---|---|
| ORB-SLAM3, stereo-inertial mode (source code, `LoopClosing.cc` line 272; not described in the paper [7]) | Rejects a loop whose correction turns the map > 20 deg in yaw (or > 0.46 deg in roll/pitch) | Same idea, for loops only. Map merges: scale checked (0.9-1.1), yaw left free. Relocalisation: no check. |
| Kimera-RPGO odometry check + PCM (Mangelson 2018, cited [35]) | Each loop must agree with the odometry chain between its two poses, translation and rotation, then loops must agree pairwise | Includes a rotation test, but only against SLAM's own odometry, and only where an odometry chain links the two poses (not across separate maps). |
| visloc-rs PR #281 (open-source code, merged 2026-10-13) https://github.com/rsasaki0109/visloc-rs/pull/281 | Loop match rejected if its visual relative rotation disagrees with the IMU-pose relative rotation by > 30 deg (5 deg on LaMAria) | Closest in mechanism. Code, not a paper; no symmetric-structure evaluation. Notes a frame bug (IMU vs camera frame) that let bad loops through: our frame handling must be stated. |
| ROVER (cited [41]) https://arxiv.org/abs/2508.13488 | Loop accepted only if the trajectory after optimisation stays consistent with the trajectory before it (repetitive warehouses) | Trajectory prior, not heading; already cited as "close to a check against dead reckoning". |
| MASt3R-Fusion (Zhou et al. 2025) https://arxiv.org/abs/2509.20757 | Drops loop candidates whose VIO-predicted positions are too far apart, with a scale and a heading term in the uncertainty | A position gate to save computation, not framed as an aliasing defence; heading only enters the uncertainty. |
| Zhu et al., Drones 7(6):338, 2023, doi:10.3390/drones7060338 (full text read, Sec. 3.2) | Before the bag-of-words query, a keyframe is a loop candidate only if (a) the IMU-derived travel direction has reversed on some axis (stored as 1/0 per axis) and (b) it lies within r = 20 x stereo baseline of an earlier keyframe | A cheap pre-filter to cut CPU load as the database grows, inspired by PCM [their 22-24]. Never compares the match's turn with a measured turn; not aimed at aliasing; tested on EuRoC only. Would not reject a wrong-face match on an orbit, where travel direction keeps changing. |
| Segway DRIVE benchmark https://arxiv.org/abs/1907.03424 | Marks a localisation false if its relative motion disagrees with wheel odometry | Evaluation use, not a gate. |
| Doppelgangers (Cai et al., ICCV 2023) https://arxiv.org/abs/2309.02420 | Learned classifier: same surface vs a different, similar one (e.g. opposite faces of a symmetric building) | The learned alternative. It relies on small visual differences. Our faces have none, but real jackets may (marine growth, anodes). Worth one sentence in Discussion. |

No paper was found that measures loop-closure aliasing on a rotationally symmetric offshore structure
and tests a heading check against it. (A web search, not a systematic review.)

## What we can claim (each must match the evidence we end up with)

1. **The measurement.** How often stereo loop closure fails to the wrong face of a jacket. The
   controlled symmetry test (G2).
2. **The scope of the check:**
   - **Independent heading.** It uses the vehicle's heading sensor, which is independent of the
     SLAM map and runs continuously across map boundaries. So it also checks map merges and
     relocalisations. Stock stereo-inertial ORB-SLAM3 leaves yaw free on those, and an
     odometry-chain check cannot test them.
   - **Stereo-only.** It works without visual-inertial SLAM.
   - **Threshold from the structure.** The threshold is half the structure's symmetry angle, and
     the margin is checked against gyro drift for each gyro grade.
3. **The in-flight test.** The pre-registered stkL0 vs vetoL0 flights, with per-event hit and
   false-alarm counts from the Mode 1 logs.

## What not to claim

- That checking a loop against inertial rotation is new.
- That learned place recognition cannot help. Say instead: on identical faces no appearance cue
  exists; on real structures with small differences, a Doppelgangers-style classifier might.

## Sentences the manuscript needs (in the user's words)

- Related work: ORB-SLAM3's own inertial loop yaw gate; the Kimera-RPGO / PCM odometry check;
  Doppelgangers. Optionally visloc-rs as code.
- Method: why the check needs an independent heading (merges, relocalisation, stereo-only), and the
  frame convention (relative camera turn about the gravity axis, `DownInCamera`).
