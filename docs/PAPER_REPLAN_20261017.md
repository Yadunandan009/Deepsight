# Paper replan, 2026-10-17

Supersedes the sequencing in `PAPER_EXECUTION_PLAN.md` where they differ.
**Section 7 (added later the same day) supersedes sections 1, 3 and 4 where they
differ: the pitch is broadened and the sonar use is changed.**
Reason for the replan: the gate was found to add nothing on the current data,
the sonar evidence was found to be circular in simulation, and the one
question every reviewer asks first -- what is new -- needed a straight answer.

## 1. What this paper is about

**One sentence.** On four-fold symmetric offshore structures, camera loop
closure fails in a predictable, geometrically identifiable way; we quantify
how often and how badly, show that dead reckoning alone suffices on a single
lap, and test a symmetry-based check for the failure in the regime where a
size-based check stops working.

### What we bring, and its status

| | contribution | status |
|---|---|---|
| B1 | **The hazard, characterised.** Twelve closures in eleven missions classified by the rotation they impose about the structure's axis; five are wrong-face (90 deg, once 180, radius kept, 24-33 m), each confirmed against ground truth; at least 5 of 28 missions; one closure took the error from 0.3 m to tens of metres and fragmented the map. | **Done** |
| B2 | **A measurement methodology.** A SLAM-free reference filter that makes the alignment falsifiable; diagnosis of a circular alignment and an unconverted ENU/NED frame that together made a discrete failure look like continuous drift. | Done; the 41 m vs 1.7 m number needs re-measuring |
| B3 | **A negative result that matters.** On a single lap dead reckoning holds 0.2 m RMS and fusing SLAM makes it worse. | One clean run; needs R1 |
| B4 | **A symmetry-based check** that flags a closure whose correction is a symmetry operation of the known structure, regardless of its size. | **Not done.** See section 4 |

### What we do not bring (say it in the paper)
- A control on a non-symmetric structure. The 90 degree signature is strong
  evidence that symmetry is the mechanism; the dose-response that would be
  the control is pre-registered and not run.
- Real data. Everything is simulated, one scene, one SLAM system.
- A closed-loop gate. Any check will be evaluated offline on recorded events.

### What is not new (say this too)
Checked by web search on 2026-10-17 (`docs/paper/CITATION_CHECK.md`).
Perceptual aliasing and place recognition failing in symmetric scenes are
known. **Rejecting false loop closures is an established problem**: switchable
constraints, max-mixtures, dynamic covariance scaling, pairwise consistency,
and chi-squared checks against odometry. ROVER (2025) verifies closures
against the robot's own trajectory in repetitive environments, which is close
to our dead-reckoning check. So the plain consistency check is prior art, not
a contribution.

What the search did **not** find: a method that uses a known rotational
symmetry group of the target to identify a wrong-face closure, or any
quantification of wrong-face closures on a jacket-type structure with camera
SLAM. That is the narrowed claim. It rests on a web search, not a systematic
review, so "not found" is not "does not exist"; the proper search is still to
do, and ROVER must be read and distinguished.

## 2. What a reviewer will ask, and what we can answer

| question | answer today | gap |
|---|---|---|
| What is new? | B1 and B2, section 1 | B4 unproven |
| Why use SLAM if dead reckoning is 0.2 m? | Only when dead reckoning drifts | **No evidence yet.** R2 |
| Is it a simulator artefact? | Partly yes; the signature is geometric and transfers | Limitation; no real data |
| Is symmetry the cause? | The 90 degree radius-preserving signature | No control run |
| What should I do about it? | B4 | Not built |
| Is a plain consistency check enough? | **On our data, yes**: any size threshold from 2 to 24 m separates all 11 scored closures, a 13x gap | B4 only earns its place if this stops being true at realistic dead-reckoning error |

The last row is the honest centre of the paper. A size-based check works
while dead reckoning is good, because legitimate corrections are then small.
When dead reckoning has drifted, legitimate corrections are large too, and
size can no longer separate them from a wrong-face jump. A wrong-face jump is
still a rotation about the structure's axis by 90 or 180 degrees with the
radius kept; a drift correction generally is not. **That is the hypothesis B4
rests on. It is a hypothesis, not a result.**

## 3. Figures: 6 main, 1 optional

| # | figure | status |
|---|---|---|
| 1 | System diagram: simulator, cameras, SLAM, bridge, filter, SLAM-free reference | drawing, not made |
| 2 | Where the wrong-face closures land: plan view and size-vs-turn | **done** |
| 3 | What one wrong closure costs: bearing and position error over time | **done** |
| 4 | Dead reckoning vs SLAM-aided, three seeds, corrected pipeline | data after R1 |
| 5 | **Where SLAM starts to help:** error vs gyro drift rate | data after R2 |
| 6 | **Check performance vs dead-reckoning error:** plain size check against symmetry check | offline, needs no flights |
| 7 (optional) | Frame-bug diagnosis: track before and after the ENU/NED fix | data exists |

**Cut:** the sonar overlay and the sonar flights. In simulation, sonar of a
symmetric model coming out symmetric is close to circular. One sentence and a
small table. The range-against-stereo-gate figure moves to a supplement.

## 4. Runs

### Must do
- **R1. Re-measure on the corrected pipeline.** Three seeds, each in two
  setups (alignment against the SLAM-free reference; alignment against the
  filter that uses it). Six flights, about 2-3 h. It tests the bridge
  changes, which have never flown; re-checks that fusing SLAM hurts; and
  re-measures 41 m vs 1.7 m.
- **R2. Gyro-drift sweep.** Scenario variants at 5, 10 and 15 deg/hr
  (`yaw_drift` 2.424e-05, 4.848e-05, 7.272e-05 rad/s), one flight each, about
  1 h. The comparison is within each run (SLAM-aided against SLAM-free), as
  in `PREREG_heading_drift.md`. At 10-15 deg/hr the earlier estimate is
  2-3 m peak error, which clears the 0.5 m criterion that the 3 deg/hr run
  missed. Pre-register before flying. This is the only experiment that can
  answer "when is SLAM needed", and without that answer B4 has no purpose.

### No flights needed
- **A1. Offline check evaluation.** Using the recorded closures and
  dead-reckoning as the reference (not ground truth), sweep an injected
  dead-reckoning error and measure, for a plain size check and for the
  symmetry check, how many of the five wrong-face closures each catches and
  how often each wrongly flags a legitimate correction. The specificity of the
  symmetry check on legitimate corrections is the thing to measure, not
  assume: a random correction could mimic a symmetry operation by chance.

### Conditional
- **R3. One long mission at high drift** (three circuits, about 50 min),
  only if R2 shows dead reckoning reaching the multi-metre regime. Needs the
  circuit counter in `_advance`; red-zone code, drafted for review, not
  authored.

### Skipped, stated as limitations
- Dose-response (60-80 flights). Pre-registered, not run.
- Sonar flights.

## 5. Decision point

After R1, R2 and A1:

- **The symmetry check beats a plain check at realistic dead-reckoning
  error** -> the paper is "the hazard, and a check that works where the
  obvious one does not". Restore B4 as a contribution, offline.
- **A plain check suffices throughout** -> the paper is a hazard paper whose
  remedy is a plain consistency gate. Smaller, honest, still publishable as a
  characterisation. Retitle; the intro and contributions change.
- **R2 shows SLAM never helps at any realistic drift** -> state it plainly.
  The discussion becomes "when not to use loop closure on a symmetric
  structure", which is itself a finding.

## 6. Writing order

1. **Abstract and contribution list now.** They fix the pitch in section 1;
   everything else is written against them.
2. Methods: simulator, sensor error models, navigation architecture,
   reference filter, closure classifier. No dependency on new runs.
3. Results, in the order the runs return: closure events (written),
   dead reckoning and fusion (after R1), drift sweep (after R2), check
   evaluation (after A1).
4. Section 2.5 rewrite, discussion, limitations.
5. Verify every citation.

## 7. Revision after the novelty check

**Why.** The web check found prior art for the plain check, and found that
symmetry ambiguity is already handled in object SLAM and in localisation on
symmetric maps. A paper whose contribution is the hazard plus a symmetry test
is narrow. The author has said so repeatedly, and the check agrees.

**Sonar, corrected.** Using sonar to *prove* the faces are identical was
circular in simulation and is dropped. Sonar becomes an independent result:
**reconstruct the structure from the vehicle's own multibeam and score it
against the CAD mesh.** The code to build a sonar cloud exists
(`plot_mission.py`), as do the meshes (`data/turbines/`). Scoring is
point-to-surface distance to the mesh.

### Broadened pitch
*A falsifiable evaluation of vision-aided navigation for inspecting a
symmetric offshore structure: what SLAM buys, what it costs, and when.*

| | contribution | status |
|---|---|---|
| C1 | **Testbed and methodology as a resource**: datasheet-grounded DVL error model, SLAM-free reference filter, closure classifier, pre-registered protocol | built; needs packaging |
| C2 | **Inspection-product metric (new).** Reconstruct the structure from the AUV's own multibeam using each navigation source and score against the CAD mesh: ground-truth poses (the sensor floor), dead reckoning, SLAM-fused, and after a wrong-face closure | **not built**; offline, no flights |
| C3 | **Regime map.** Where SLAM starts to help as gyro drift grows, scored on pose error and on C2 | needs R2 |
| C4 | **The hazard**: wrong-face closures, frequency, cost | done |
| C5 | **A symmetry-based test.** Now clearly an extension of known symmetric-ambiguity handling | contingent on A1 |

### Do the flights widen it?
- **R1 (re-measure, 6 flights): no.** It consolidates numbers the paper
  already quotes. Needed for credibility, not for novelty. It could be cut to
  4 flights: three seeds on the independent-reference setup, plus one
  circular-alignment run for the 41 m comparison.
- **R2 (gyro-drift sweep): yes, modestly.** Each flight gives a point on the
  regime curve **and** a navigation source for C2. Rates 5, 10, 15 deg/hr
  alongside the existing 0 and 3 give five points.
- **Neither creates novelty alone.** The widening comes from C2, C3 and C1
  together.
- **No sonar flights** for the baseline reconstruction: 34 of 36 recordings
  already contain multibeam. Decide after checking coverage on them.

### Revised figures (7 main)
1 system diagram; 2 closures (done); 3 cost of a closure (done);
4 dead reckoning vs SLAM-fused (R1); **5 reconstruction against CAD under
each navigation source (A2, replaces the sonar overlay)**; **6 regime map:
pose error and reconstruction error against gyro drift (R2 + A2)**;
7 check performance against dead-reckoning error (A1, optional).

### New offline work
**A2.** Extend the sonar cloud builder to take a chosen pose source, score
against the mesh, run on the existing recordings. **Needs the external drive.**

### What will still be said
Simulation only. An evaluation paper, not a method paper. If A2 shows dead
reckoning already gives a clean model of the structure on a single lap, then
"SLAM is not needed to reconstruct a jacket on a lap, and it is a hazard
if used" is the finding. Real data remains the standing gap, and which
venue this suits is a question for the author, not something this plan can
settle.

## 8. Progress, same day

Done from section 7: the sonar reconstruction scorer (`eval/sonar_recon.py`) and
its run over 17 recordings; the closure-check comparison; the methods,
dead-reckoning, inspection-product, checks and limitations sections; the abstract.

**What the reconstruction showed.** Sensor floor 7.3 cm. Dead reckoning median
11 cm over ten recordings. Fused filter 0.38 and 1.8 m in the two post-fix
recordings. A vehicle trusting SLAM through a wrong-face closure, 3.8 to 7.5 m in
all five missions.

**What the checks showed.** On a lap every check works. Plain checks fail from
about 7 m of drift; the chord check holds; none of this is reached by a lap or
by the flights planned.

**Two data problems found by checking labels against data.** One recording named
for a DVL seed had no systematic DVL error; one named dead reckoning had SLAM in
the loop. Both excluded.

**What is still open, in order of consequence:**
1. The post-fix fused-filter evidence is two recordings. The re-measurement
   flights are the fix, and they also test the unflown bridge.
2. The gyro-drift sweep. Its value changed: drift reaching the 7 to 18 m window is
   hours away, so it tests where SLAM starts to *help* (about 1 m), not the checks.
3. The symmetry-based check is in-sample and offline.
4. Discussion, conclusion, Section 2.5, the system diagram, citation support.

