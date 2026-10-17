# 7. Limitations

*Draft 2026-10-17. Written to be read by a reviewer looking for what the work
does not show. The first three limits decide how far the conclusions reach;
the rest qualify particular numbers. Section numbers are placeholders.*

## 7.1 What the results do not reach

**Simulation only, one scene, one SLAM system.** Every mission is simulated,
in clear water, under an overhead sun, around one structure, with one stereo
SLAM system and one mission profile. Real water adds turbidity, marine growth,
caustics and lighting change; each would degrade tracking and change which
views are matched. The sonar noise is a Gaussian 0.1 m. We claim that the
failure has a geometric signature that does not depend on the simulator. We do
not claim its frequency transfers.

**We do not show that symmetry is the cause.** The wrong-face closures turn the
estimate by 90 degrees, once by 180, about the structure's axis and preserve
its radius, which a match to an unrelated place would not do. That is strong
circumstantial evidence. The control is missing: we did not run the same
missions on a structure without the symmetry. A matched comparison of the stock
structure against a more heavily outfitted variant is pre-registered and was
not run; at the rate observed here it needs 30 to 40 missions per arm to resolve
a difference.

**The structure is an idealisation.** Real jacket foundations have three, four
or more legs, and boat landings, cable tubes, ladders and anodes break whatever
symmetry the primary steel has, by amounts that vary. How much that reduces the
hazard is open. The simulated structure is the idealised case.

## 7.2 What qualifies the closure result

**The sample is not a random draw of missions.** The twelve classified closures
come from eleven recordings made over six weeks under several configurations
(two camera frame rates, different controller and alignment code). Eighteen
closures were logged across 28 full missions; six have no recording and could
not be classified, so at least 5 of 28 missions (18 %, 95 % interval 6 to 37 %)
is a floor, not a rate. The seven September recordings were flown with a
controller steering on a filter anchored to ground truth, so the vehicle's
trajectories were better than a GPS-free vehicle's. SLAM uses only the
cameras, so its closures do not depend on this, but the trajectories do.

**Events in one mission are not independent**, and a closure that undoes an
earlier error is also a rotation of the same size and would be scored
wrong-face. Only the first closure in a mission is classified against an
uncontaminated frame; later ones are reported but not relied on.

**The cost is measured in five missions, and causation is not shown.** After a
wrong-face closure the SLAM position was wrong by a median of 13 to 33 m for two
minutes, and the map re-initialised at least once afterwards. But two missions
re-initialised within seconds, one after 471 s, and re-initialisation also
occurs in missions whose only closure was benign.

**The shared landing bearing is two cases, not five.** Four of the five
wrong-face closures start from the same face and so end together.

## 7.3 What qualifies the navigation result

**Dead reckoning is accurate on a lap partly because of the lap.** A heading
error displaces an estimate in proportion to its distance from where heading was
last known, and an orbit never takes the vehicle far from where it began: 0.96
degrees over 131 m of straight line would be 2.2 m, against at most 0.6 m here.
The result is a statement about single-lap inspection missions.

**We did not show that SLAM never helps.** The experiment designed to find the
point where it starts to help, a gyro drift of 3 degrees per hour, was void:
dead reckoning degraded to 0.22 m, short of the 0.5 m the registration required.
Missions long enough, or gyros poor enough, for dead reckoning to drift by
metres were not flown.

**Some recordings are mislabelled.** One recording named for a DVL seed was
flown with no systematic DVL error (its measured gains are 1.0000 on all axes),
and another named as dead reckoning had SLAM in the loop. Both were found by
checking the data against the label and are excluded from the statistics they
would have distorted; the labels in bag names cannot be trusted.

**The fused-filter results are the least solid.** Recordings made before
2026-10-16 fed the filter an unconverted SLAM pose. Two recordings postdate the
fix, and one of them is the only one in which dead reckoning and the fused filter
can be compared on an inspection mission. A bridge that fits SLAM's scale and
publishes an honest covariance has been written and has never flown, so nothing
here says what it would do.

## 7.4 What qualifies the inspection-product result

The structure is scored against the simulator's own CAD mesh; a real structure
is not its drawing. The multibeam returns a planar slice, so the reconstruction
depends on how the vehicle sweeps. The set of returns is defined using the true
pose, so the comparison between navigation sources is clean but the absolute
numbers are conditional on that selection. The "trust SLAM through the closure"
condition is a counterfactual built from the raw SLAM pose, an alignment fitted
before the closure and a calibrated rotation offset; it was not flown, and its
accuracy before the closure sets a floor under what it can show: its rotation
residual is 1.7 to 4.0 degrees in four missions and 11.7 in the fifth, whose
pre-closure error of 0.73 m comes from that calibration, not from SLAM.

## 7.5 What qualifies the proposed checks

The three closure checks are evaluated offline, on recorded closures, with drift
injected into the true track under a stated model (linear growth with distance,
random direction). The injected drift is an assumption, not a measurement. The
chord check was specified after the five wrong-face chords had been seen, so it
is in-sample here. It needs the orbit radius and the symmetry order, and it
raises false alarms once honest corrections become as large as the jumps. None
was run in the loop.

## 7.6 Positioning

Rejecting false loop closures is an established problem with robust back-ends
and consistency checks, and symmetry ambiguity is handled in object SLAM and in
localisation on symmetric maps. The contribution is the characterisation on this
asset class and the inspection-product evaluation, not the idea of a check. The
literature search was by web search, not a systematic review, and every claim
that something has not been done is conditional on it.

---

### Status note (not for the manuscript)

Numbers here come from `section_closure_events.md`, `CITATION_CHECK.md` and
`docs/experiments/PREREG_*.md`. **Update when the planned flights return:** 7.3
(fused-filter solidity, the drift sweep) and 7.2 (if more closures are logged).
**Section 7.4 now quotes the measured rotation residuals** (from
`eval/figures/recon_results.json`).
