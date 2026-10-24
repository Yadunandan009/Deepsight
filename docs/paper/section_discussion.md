# [8]. Discussion

*Draft 2026-10-19. Section numbers are placeholders. Every number is quoted from
the results sections (`section_closure_events.md`, `section_checks.md`,
`section_dr_results.md`); nothing here is new evidence. Written against
`section_limitations.md`: where a claim below is hedged, the reason is there.*

## [8].1 What the study found

Three things, in the order a vehicle meets them.

**First, on a single lap dead reckoning is enough.** A vehicle with a Doppler
velocity log, an inertial unit and a depth sensor held its position to 0.1 to
0.5 m over a lap of about 130 m, and the structure it reconstructed from its own
sonar was 11 cm RMS from the CAD, against a sensor floor of 7 cm. Nothing else
was needed to produce an inspection-grade model of the structure.

**Second, adding camera SLAM did not help, and in one way it hurt.** Fusing the
SLAM pose raised the position error in all seven valid recordings. Five of those
recordings carried a frame defect that has since been fixed, so they show what
the defect cost, not what SLAM does. The two recordings made after the fix give
a reconstruction error of 0.38 m (a mission whose only closure was benign) and
1.8 m (a mission with a wrong-face closure), against 0.12 and 0.10 m for dead
reckoning in the same flights. Two recordings is little, and we say so.

**Third, the way SLAM hurt has a recognisable shape.** Of twelve classified loop
closures, five re-posed the estimate by 90 degrees (once 180) about the
structure's axis, keeping the radius, to a place the vehicle had never been. A
vehicle that trusted SLAM through such a closure would have reconstructed the
structure to 3.8 to 7.5 m. ORB-SLAM3's own map shows the same thing from
another side: it follows the structure before the closure but is noisy (17 to
21 % of its points within 25 cm of the CAD, against 99 % for the sonar placed by
dead reckoning), and after the closure it is badly damaged in three of the five
missions measured, with duplicate copies of the structure where none exists. In ten flights with no wrong-face closure the map does not change across the
same split, which is the control that ties the damage to the closure.

## [8].2 Why dead reckoning does so well, and what that does not mean

A heading error displaces an estimate in proportion to its distance from where
heading was last known, and an orbit never takes the vehicle far from where it
began. The result is therefore a statement about single-lap inspection missions,
and the limitations section says as much: on a longer mission, or with a
poorer gyro, dead reckoning drifts by metres and the argument changes. We did
not find the point where SLAM starts to help. The experiment designed to find it
was void, because dead reckoning degraded to 0.22 m, short of the 0.5 m the
registration required.

What the result does say is narrower and still useful: **the bar SLAM has to
clear on a lap is high, and in this simulator it did not clear it.** A study
that evaluates SLAM on a lap without a dead-reckoning baseline will find SLAM
looks good, because its error is small in absolute terms, and will miss that the
baseline was smaller.

## [8].3 The mechanism, and how much of it we have shown

The wrong-face closures preserve the radius and turn the estimate by a multiple
of 90 degrees about the structure's axis. A match to an unrelated place would
not do that. The signature is what a four-fold symmetric structure would
produce, and it is the reason we call the closures wrong-face rather than
merely wrong.

It is circumstantial. We did not run the same missions on a structure without
the symmetry, so we cannot say the symmetry caused the closures, only that their
geometry is what it would cause. The control that would settle it is
pre-registered and was not flown. Two further points weaken the claim:
the sample is a floor (at least 5 of 28 missions), not a rate, and four of the
five closures start from the same face, so the shared landing bearing is two
cases, not five.

## [8].4 What a vehicle can do about it

A closure that is wrong-face is large and goes to a place the vehicle has not
been, and each of the three checks we tried uses a different part of that.
On a single lap a plain check against dead reckoning catches every wrong-face
closure and raises no false alarm, and the three checks are interchangeable.
Checking a closure against odometry in repetitive environments has been done
before, and the plain check is prior art here, not a contribution.

The checks differ once dead reckoning has drifted. In an offline test with drift
injected into the true track, the plain checks fail between about 7 and 18 m of
drift, because honest corrections have grown large enough to look like jumps,
while a check on the closure's length alone, which uses no dead reckoning,
keeps catching every wrong-face closure until honest corrections become as large
as the jumps, at about 24 m. This is a window that only a symmetry-based test
reaches, and it is a long-mission regime: by order-of-magnitude arithmetic
reaching 7 m takes one to five hours of operation, which nothing flown here
approaches. The chord check was specified after the five wrong-face chords had
been seen, so its performance here is in-sample, and none of the checks was run
in the loop. We offer the window as a hypothesis with a stated model, not as a
result.

For an operator the practical reading is short. On a single lap of a symmetric
structure, with good velocity and heading sensing, do not let a camera loop
closure move the estimate by tens of metres without a check against the
dead-reckoned track; on a longer mission, add a check that does not depend on
that track.

## [8].5 The inspection product as the measure

We scored the vehicle on the structure it delivers, not only on where it thought
it was, and that changed what the numbers mean. A 1.8 m position error is a
number; a reconstruction in which 94 % of the points lie more than 25 cm from
the surface is a model nobody could use. The sonar is the right instrument for
this, and the sonar reconstruction stays within a few centimetres of the CAD
under dead reckoning. SLAM's own map is not a substitute: it holds hundreds of thousands of
points but most are not on the structure, and in three of five missions it is
damaged by the same closure that damages the pose.

This is a statement about this simulated scene. The sonar noise is Gaussian
and the water is clear; a real multibeam in turbid water and a real structure
with marine growth would change both the sonar's accuracy and what the cameras
can match. The comparison between navigation sources, which uses the same
returns in every case, is not affected by the first; the absolute numbers are.

## [8].6 What to do next

In the order the evidence needs them:

1. **Replicate the corrected-pipeline result.** The post-fix evidence for what
   fusing SLAM does is two recordings. Three seeds in two setups would turn it
   into a measurement, and would test the Sim(2) bridge, which has never flown.
2. **Find where SLAM starts to help.** A gyro-drift sweep at 5, 10 and 15 degrees
   per hour, compared within each run, is the one experiment that can say when
   loop closure is worth its hazard.
3. **Run the control.** The same missions around a structure without the
   symmetry, or with it deliberately broken, would show whether the closures
   follow the symmetry. This is the most expensive item and the one a sceptical
   reader will ask for first.
4. **Run a check in the loop.** The chord and plain checks were evaluated on
   recorded closures. Whether they hold when the vehicle acts on them is
   untested.
5. **Real data.** None of this transfers until it has been seen on a real
   structure, with real water.

---

### Status note (not for the manuscript)

**Numbers that will change when the re-measurement flights return:** [8].1 and
[8].4 (fused-filter results rest on two recordings), and [8].2 if the gyro-drift
sweep is flown. **The SLAM-map claims** in [8].1 and [8].5 use five wrong-face flights and ten
control flights (`eval/figures/slam_map_results.json`); the ten controls were
scored and left "three of five" unchanged. **Unverified:** "94 % of the points lie more than 25 cm from the
surface" is the SLAM-fused filter after the closure in the 2026-10-16 mission
(6 % within 25 cm, `section_dr_results.md` N.2), not a general figure; check it
reads that way in the manuscript.
