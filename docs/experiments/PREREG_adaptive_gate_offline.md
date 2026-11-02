# Pre-registration: the adaptive consistency gate, offline

Written 2026-10-26, before any candidate was replayed. Offline step before an in-loop flight.

## Question

The bridge refuses a SLAM pose only if it is more than 5 m from the SLAM-free reference (dead reckoning). Since 2026-10-25 it also quarantines SLAM after an in-map jump. Which consistency gate, run inside the real bridge, removes the badly wrong poses that remain, without throwing away the good ones?

## Why this replay is exact

With `dr_reference:=true` the bridge's inputs are the raw SLAM pose, the map-point count and `/bluerov2/odometry/dr`. None of them depends on what the bridge publishes. So feeding the recorded inputs through `SlamPoseBridge` (as `eval/replay_bridge.py` does) gives exactly what it would have published under any gate: including the poses the 5 m gate refused, the real probation and the re-align after 40 refusals. This replaces the earlier `eval/gate_replay.py` estimate, which saw only already-published poses and approximated probation. What is NOT replayed is the primary filter's response to the poses; that needs the in-loop flight.

## Data (fixed now)

All 22 bags on the external drive holding both `/bluerov2/robot_pose_slam` and `/bluerov2/odometry/dr` (listed in the outcome). Every bag is replayed with `slam_frame:=enu` and `jump_policy:=quarantine`, i.e. as the bridge now flies, whatever bridge flew at the time. Ground truth (`/bluerov2/odometry`) is used only to score.

## Candidates (fixed now)

| id | gate | floor (m) | k_drift (/m) | probation |
|---|---|---|---|---|
| base | fixed, the current default | 5.0 | 0 | none |
| A | fixed | 1.0 | 0 | none |
| B | fixed, with probation | 1.0 | 0 | 25 poses and 8 m spread |
| C | fixed, with probation | 1.5 | 0 | 25 poses and 8 m spread |
| D | growing with path, with probation | 1.0 | 0.003 | 25 poses and 8 m spread |

Cap 5 m for all. Probation "none" is `gate_probation_poses:=0, gate_probation_spread:=0`.

## Metrics

A published pose is **BAD** if it is more than 1 m from the truth, **GOOD** otherwise. Pooled over the 22 bags: the number of BAD and GOOD poses published. Per bag: the worst published error.

## Decision rule

A candidate passes if:

- it publishes at most 5 % of the BAD poses `base` publishes, and
- at least 95 % of the GOOD poses `base` publishes.

Of the candidates that pass, the one with the fewest BAD poses goes to the in-loop flight; on a tie, the one with fewer settings (A before B, C, D). If none passes, nothing is flown and the outcome says why.

## Limits, stated now

- One lap. Dead reckoning is good to under 0.7 m here, so any gate of about 1 m around it bounds the published error near 1.7 m by construction. This tests how often SLAM disagrees with dead reckoning, not whether SLAM helps when dead reckoning has drifted. A long mission would need a growing width (D) and a different test.
- The 1 m BAD line is a choice, kept from the earlier replay.
- Exploratory in the sense that the candidates come from the earlier, imperfect replay; the rule above is fixed before this one.

## Outcome

Run 2026-10-26, `python3 eval/gate_offline.py` (results in `eval/figures/gate_offline.json`). All 22 bags, pooled:

| gate | BAD published | % of base | GOOD published | % of base | worst error |
|---|---|---|---|---|---|
| base (5 m) | 18,947 | 100 | 182,471 | 100 | 5.00 m |
| A (1.0 m) | 1,957 | 10.3 | 176,404 | 96.7 | 1.44 m |
| B (1.0 m, probation) | 1,251 | 6.6 | 163,016 | 89.3 | 1.25 m |
| C (1.5 m, probation) | 11,196 | 59.1 | 174,885 | 95.8 | 1.85 m |
| D (1.0 m + 0.003/m, probation) | 6,220 | 32.8 | 169,353 | 92.8 | 1.69 m |

**No candidate passes.** A keeps enough GOOD poses but publishes 10.3 % of base's BAD poses, against a 5 % limit; B publishes fewer BAD poses but loses 10.7 % of the GOOD ones; C and D fail both. By the rule, nothing goes to the in-loop flight on this result.

What the numbers show, exploratory (not part of the rule):

- 92 % of base's BAD poses (17,399 of 18,947) are in the six bags of 14-15 October, flown with earlier SLAM settings (Camera.fps 30, or the 0.30 m baseline). In the 16 bags from 16 October on, A removes 89 % of the BAD poses (1,548 to 170) and loses 0.3 % of the GOOD ones; probation (B) adds nothing there except a 2.4-point loss of GOOD poses.
- The 5 % line was probably out of reach for any gate near 1 m. A pose 1 m from dead reckoning can be up to 1 m plus dead reckoning's own error (up to about 0.7 m) from the truth, so a 1 m gate cannot remove poses between 1.0 and about 1.7 m from the truth. The worst error confirms it: every gate kept the worst pose under 1.9 m, against 5.0 m for the current gate. The BAD line and the gate width were set too close together. That is a flaw in the rule, found after the run; it does not change the outcome.
- A new test needs a new registration; this one is closed.
