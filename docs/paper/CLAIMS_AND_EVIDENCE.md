# Claims, evidence, novelty and an honest assessment

*Written 2026-10-22 alongside `MANUSCRIPT_DRAFT_20261022.md`. This is not manuscript text. It is the page to read before deciding what the paper says. Numbers marked **[LIVE]** change as flights come in; the current values are in `live_numbers.md`.*

## 1. The paper in one sentence

On a four-fold symmetric jacket in simulation, stock ORB-SLAM3 sometimes closes a loop to the wrong face, which re-poses the estimate by 90 or 180 degrees about the structure's axis; we measure how often and how badly, test with a pre-registered paired experiment whether the symmetry is the cause, and show what a loosely coupled fusion bridge does with the error.

## 2. What is new, even though it is narrow

**New (to the best of a web search that was not a systematic review):**

1. **A controlled manipulation of the structure's symmetry to test the cause of wrong-face closures**, with the symmetry measured from the CAD (pair score, both ICP directions) before any flight, a paired design by DVL unit, an exact test and a stop rule, plus one control that failed by design and is reported as such. Prior symmetry work in SLAM is on object pose ambiguity and symmetric maps; we did not find a test of this kind for loop closure on an inspection structure.
2. **A measurement of wrong-face closures on a jacket-type structure** with a classification rule fixed in advance and confirmed against ground truth by a test that does not use the rotation geometry (distance to the nearest earlier true position).
3. **The scoring of the inspection product**, a sonar reconstruction against the CAD under each navigation source, as the cost of a navigation fault. Position error says 1.8 m; the reconstruction says 94 % of points are unusable.
4. **A measurement of what a loosely coupled bridge does after a wrong-face closure**: it goes silent, re-locks on a poor window, and its own 5 m consistency gate admits first poses 1 to 3 m wrong and sets the ceiling on the fused error, which the filter cannot then undo. Measured on nine flights; the analysis was not registered.

**Not new, and the paper says so:** that perceptual aliasing exists; that false closures should be rejected (switchable constraints, max-mixtures, pairwise consistency, ROVER); that a check against odometry catches them (ROVER); that symmetry makes a measurement ambiguous and the ambiguity should be carried (object SLAM, clustered particle filters). The chord check (a closure whose length equals a symmetry chord) is in-sample and offline, so it is discussed, not claimed.

**Not a contribution:** a remedy. Nothing here was run in the loop.

## 3. Claim ledger

Strength: **solid** = measured, reproducible from the repository, would survive a hostile read within its stated conditions; **supported** = measured but small n or constructed; **hypothesis** = a reading of the data that has not been tested.

| # | Claim | Evidence | Strength | What would break it |
|---|---|---|---|---|
| K1 | On one lap dead reckoning holds 0.1 to 0.5 m and reconstructs the structure at median 14 cm RMS, range 8 to 59 cm (floor 7 cm) | 18 recordings (two excluded, Appendix A), recomputed 2026-10-24 from `recon_results.json`; 3 DVL units, per-run DVL operator check | solid, for this DVL model and a single lap | a DVL model with bottom-lock loss or temperature terms; a longer mission |
| K2 | Stock ORB-SLAM3 makes closures that rotate the estimate by 90 or 180 degrees about the axis with the radius kept, to a different face from the one the vehicle is at (a place it really visited earlier) | 12 classified events in the earlier recordings plus the new flights; revisit test on 12 new flights: P2 held 7 of 7 (estimate within 1.0 m of a real earlier visit), P1 as registered failed 4 of 7 because those closures are at the return to the starting face, where the vehicle really had been | **solid for the geometry and P2; the old wording "never been" was wrong for return-time closures** | a different SLAM system that never does it (that would be a finding, not a refutation) |
| K3 | 9 of 20 complete stock missions (45 %, 95 % interval 23 to 68 %) had a wrong-face closure; 4 of 14 before 2026-10-20 against 5 of 6 after (one-sided p = 0.04, exploratory, 0.06 with one incomplete mission) | census by period (Table 4) | **supported as a floor (denominator corrected 2026-10-24 to count complete missions without a closure); the period difference is exploratory and not attributed to any one change** | the periods differ in many ways; the earlier reading (6 of 6 benign at the return) was a conditional count and has been retracted |
| K4 | Trusting SLAM through one rebuilds the structure to 3.8 to 10 m | counterfactual from recordings (5 earlier missions, plus the bridge-B stock flights once scored) | supported (constructed, not flown) | flying it |
| K5 | After a wrong-face closure the bridge discards its alignment and goes silent; in the 4 flights where the closure was in the last 24 to 444 s it never re-locked and the fused estimate stayed within 1.2 m (SLAM was off, not handled) | 7 flights with a wrong-face closure; bridge output read from the bags (`gate_plateau.py`, `jump_attribution.py`) | **solid for these flights** (measured, not inferred) | a flight where the bridge re-locks quickly and stays accurate |
| K6 | When the closure is mid-mission (L4 x2, L0 s0) the bridge re-locks on a poor window, the first poses are 1.2 to 2.9 m wrong, the 5 m gate admits them, and the fused estimate ends up to 4.7 m wrong while dead reckoning stays within 0.6 m | 3 flights; 13 re-locks over 9 flights, 9 of 13 first poses more than 1 m wrong | supported (3 flights, exploratory, not registered) | a different bridge; more mid-mission closures that re-lock well |
| K7 | The bridge's 5 m gate sets the ceiling on the fused error | published pose never farther than 5.00 m from dead reckoning in 9 flights; 4.91 to 5.00 m in the five flights with a large fused error, at most 1.0 m in the four without | **supported (measured); the mechanism is read from the code** | an unregistered analysis, so a replication on new flights is the test |
| K8 | Fusing SLAM did not improve position on a lap, even with bridge B (fused RMS 1.04 to 1.69 times dead reckoning in 9 of 11 bridge-B flights, pre-closure window; 2.28 times with the earlier bridge on one flight); the product (sonar model) improved by 19 to 50 % in 4 flights on DVL units 1 and 2 and worsened for unit 0 | 11 flights for position; 7 flights for the product (Table 7) | supported for position; **for the product, 3 DVL units, single flights, and the two measures disagree** | seeds 29, 3, 26 (not flown) |
| K9 | **The symmetry causes the wrong-face closures** | stage 1 finished: stock 2 of 3, L5 0 of 3, p = 0.20; one stock flight with the earlier bridge benign | **suggestive only; not shown. The registered stop rule ended the experiment** | stage 2 (not flown); the feature-density confound; the stock baseline was not near 1 |
| K10 | The L5 map is mostly not on the jacket | map-to-CAD scoring, one flight per arm | **not supported as stated**: only the best-fit placement differs | placement by the true trajectory shows no difference |
| K11 | A closure check against dead reckoning works on a lap; a chord check works longer | offline, injected drift, in-sample | weak; exploratory | any closed-loop test |
| K12 | Wrong-face closures concentrate at the seabed end of the structure (9 of 12 at 20 m or more, against 1 of 11 benign) | closure_where.py on 23 flights | **supported as a description; threshold chosen post hoc; events not independent** | a recorded-image or brighter-water run; flights where stock goes through the bottom without a closure |

**On K10, a correction made today** (and a second one on K5). I told you the L5 map was visibly worse and the numbers "held up". They hold up only under the best-case placement that uses the CAD (66 % of stock map points within 25 cm of the jacket, 28 % for L5). With the map placed by the true trajectory, which uses no CAD, the two are the same (19.5 % against 17.7 %). The best-case fit may be pulled by hardware points, or the L5 map may really be scaled differently (0.93 against 1.01). So the data support "the best-fit placement is worse for L5", not "most of the L5 map is hardware". The manuscript says that.

**On K5, the second correction.** I had told you, and the earlier notes said, that bridge B "contained" the wrong-face closures. Reading the bridge's published poses shows it did so by going silent after the closure: in all four late-closure flights its last published pose is at the closure time. That is luck of timing, not handling, and the three mid-mission closures show what happens when it does re-lock (K6).

## 4. What a reviewer will ask, and what we can answer

| Question | Answer now | Gap |
|---|---|---|
| What is new? | Section 2 above | the novelty rests on the control, which is small so far |
| Is it a simulator artefact? | The signature (90 degrees, radius kept) is geometric, but the frequency is not shown to transfer | no real data, one SLAM system |
| Is symmetry the cause? | The paired test; **[LIVE]** | n = 3 per arm in stage 1; feature-density confound |
| Why does the rate differ between your earlier and recent recordings? | **We do not know** | **the biggest hole a careful reviewer will find**; section 5 |
| Why not fix it? | A gate or check was evaluated offline only | no closed-loop test |
| Why is dead reckoning so good? | It is a lap, with this DVL model | no longer mission |
| Why ORB-SLAM3? | An unmodified stock system on purpose: the claim is about the target, not the implementation | one system |
| Is L5 realistic? | It adds real appurtenance types (risers, anodes, pipe runs) but exaggerates them | not real jacket geometry |

## 5. Honest assessment

**Where it stands.** This is a defensible characterisation paper with one real experiment in it, and the experiment is not finished. As a standalone result today it is thin: simulation only, one scene, one SLAM system, a control with at most three missions per arm, and a closure rate that changed with our own configuration for a reason we have not found. I would not call it ready for a journal as it stands, and I would not call it hopeless.

**What is genuinely good.** The methodology is better than most simulation papers: the DVL unit is verified per run, labels are checked against data (and two recordings were excluded because they were wrong), the alignment is fitted against a SLAM-free filter, the classification rule was fixed before the data, failed predictions and a failed control are reported, and the product-level metric makes the cost concrete.

**What moves the needle, in order of value for effort:**

1. **Finish the paired test** (stage 1: two flights left; stage 2: six more if Gate B allows). This is the centre of the paper. A 3-of-3 against 0-of-3 stage 1 gives p = 0.05; stage 2 would take it to about 0.001 to 0.03 if the effect is real.
2. **Find why the rate changed between configurations.** A reviewer who sees six benign closures at the return to face 0 in the earlier recordings and four wrong-face closures there in the recent ones will ask. Candidate causes: the bridge version (it changes the fused estimate the controller steers on, so the vehicle's path), the RISE controller, the camera-rate setting. A cheap discriminating run: fly stock with the old bridge and the fixed RISE. It would tell us whether the rate is a property of the structure or of the path.
3. **Test a tighter or adaptive consistency gate in the loop** (a chi-squared check against the SLAM-free filter, or a gate that shrinks with time since the last good closure). If the gate analysis confirms the plateau, this is the natural remedy and it is a one-parameter change. It is controller/estimator code, so in your red zone: you would write it; I would review.
4. **Generality.** A second structure geometry (three or six legs) or a second SLAM front end would turn "one scene" into "two". A dose-response over the number of distinguishable faces is the strongest form of the control but costs many flights.
5. **Real data**, which is the standing gap and not something this plan can close.

**Venue.** Which journal this suits is your call and your supervisor's, not mine. My honest read is that the paper is more credible as "a characterisation and a controlled test" than as a method paper, and that claiming more than that would cost credibility.

## 6. What each stage-1 outcome does to the paper

Final state (2026-10-22): stage 1 complete, stock 2 of 3 and L5 0 of 3 (p = 0.20), Gate B said stop; one earlier-bridge stock flight on DVL unit 1 was benign (0 of 1 against 2 of 3, p = 0.25).

| L0 s2 | L5 s2 | Stage 1 | p (one-sided) | Gate B | What the paper becomes |
|---|---|---|---|---|---|
| aliased | not | 3/3 vs 0/3 | 0.05 | stage 2 | "controlled test supports symmetry as the cause"; stage 2 recommended |
| aliased | aliased | 3/3 vs 1/3 | 0.20 | stage 2 | "consistent with, not shown"; stage 2 decides |
| not | not | 2/3 vs 0/3 | 0.20 | **stop and reassess** | stock baseline is not near 1; a characterisation with a rate estimate; control underpowered |
| not | aliased | 2/3 vs 1/3 | 0.5 | **stop and reassess** | no evidence either way; report as such |

If stage 1 ends 3/3 against 0/3 or 3/3 against 1/3, the pre-registered secondary prediction also applies: any aliased closures in L5 should be confined to the most-alike pairs (0-1 at 0.751, 1-2 at 0.787); one between 1-3, 2-3 or 0-3 would count against the mechanism.

## 7. Things to not write

- "Symmetry causes the closures", until K9 says so.
- "Fusing SLAM hurts" or "helps" without naming the bridge version and the DVL unit.
- Any pooled closure rate across the earlier and recent recordings.
- "Most of the L5 map is hardware".
- The 0.38 m and 1.8 m fused reconstruction figures as what SLAM does: they are bridge version A.
- A claim that the closure rate changed between pipeline periods (the earlier "6 of 6 benign vs 4 of 4 wrong-face at the return" was conditional on not having failed earlier).
- The 41 m against 1.7 m circular-alignment figure: measured before the frame fix and not re-measured (the planned `slamcirc` flight was never flown).
