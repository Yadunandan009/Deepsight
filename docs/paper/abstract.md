# Abstract (draft 2026-10-17)

Inspecting an offshore jacket structure without a position fix demands
navigation good enough to build a trustworthy model of the structure. We
evaluate vision-aided navigation for this task in simulation: a BlueROV2-class
vehicle orbits a jacket-type structure, navigating by Doppler velocity log,
inertial unit and depth, with stereo ORB-SLAM3 as an additional position source,
and we score both its position and a sonar reconstruction of the structure
against the structure's CAD model. On a single lap of about 130 m, dead
reckoning alone holds position to 0.1 to 0.5 m and reconstructs the structure to
a median of 11 cm RMS against a sensor floor of 7 cm. Fusing SLAM raised the
position error in all seven valid recordings and, in the two made with the
corrected pipeline, the reconstruction error to 0.38 and 1.8 m. One cause is a
loop-closure hazard specific to symmetric structures: five of twelve classified
closures re-posed the estimate by 90 degrees (once 180) about the structure's
axis, to a place the vehicle had never been, and at least 5 of 28 missions were
affected. A vehicle trusting SLAM through such a closure reconstructs the
structure to 3.8 to 7.5 m. A plain check against dead reckoning catches every
such closure on a lap but fails once dead reckoning has drifted 7 to 18 m; a
check on the closure's length alone, which uses no dead reckoning, does not, in
an offline test with injected drift. The study is simulated, in one scene with
one SLAM system, and does not show that symmetry is the cause.

---

### Status note (not for the manuscript)

About 235 words; trim to the journal's limit. Every number traces to a results
section: 11 cm / 7 cm / 0.1-0.5 m and the fusion figures to `section_dr_results.md`
([N].1 to [N].3); 5 of 12, 90 / 180 degrees and 5 of 28 to `section_closure_events.md`;
3.8 to 7.5 m to [N].3; 7 to 18 m to `section_checks.md`.

**Three claims rest on less than they seem, and the abstract must not be
upgraded:** (1) "in the two made with the corrected pipeline" is two recordings,
single seeds; (2) the 3.8 to 7.5 m figure is a counterfactual constructed from
recordings, not a flown condition; (3) the 7 to 18 m window is injected drift under
a stated model, and the chord check is in-sample. Replace (1) once the planned
re-measurement flights return.
