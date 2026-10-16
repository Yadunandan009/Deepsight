# [N]. Loop closure on a four-fold symmetric structure

*Draft 2026-10-17. Results section. Section number and figure numbers are
placeholders. Evidence: `eval/closure_events.py` (sha256 `383ad7de...`), run
over every recorded mission that has the SLAM track, ground truth and the
ORB-SLAM3 console log. See the status note at the end for what is and is not
settled.*

## [N].1 Measuring a closure

A loop closure is the event at which the SLAM back end decides that the
current view matches an earlier one and re-poses its map to make them agree.
The question for a symmetric structure is whether that decision is ever made
against the wrong face. A wrong-face match has a precise geometric form:
because the four faces are interchangeable, the corrected estimate is
the correct one rotated about the structure's vertical axis by a multiple of
90 degrees, with its distance from the axis unchanged.

We locate each closure from the SLAM system's own console log, take the
largest step of the raw SLAM track in the surrounding interval as the
correction, and place the estimate before and after it in the world frame
using an alignment fitted only on the preceding 150 s of poses, so that the
"after" point is placed by a transform that knew nothing about the jump.
For each event we report the size of the correction (the chord), the change
in radius about the structure's axis, and the turn in bearing about it.
A correction under 3 m is classed benign. A larger one is classed
face-aliased if the radius changes by under 10 % and the turn is within 15
degrees of 90 or 180. Anything else large is reported as such. The
thresholds were fixed before nine of the eleven runs below were classified
and were not changed afterwards.

## [N].2 What closures do

Eleven recorded missions contain at least one logged closure; one contains
two. Of the twelve events (Table [T]), six are benign, five are face-aliased
and one is large but not a symmetry operation.

**Table [T].** Every classified closure. Chord is the size of the correction
in metres; turn is the change in bearing about the structure's axis.

| mission | t (s) | chord (m) | radius change | turn (deg) | class |
|---|---|---|---|---|---|
| 2026-09-05 14:16 | 1028 | 0.80 | -1.4 % | +2.5 | benign |
| 2026-09-05 17:38 | 1014 | 0.87 | +2.8 % | -2.4 | benign |
| 2026-09-05 20:11 | 776 | 33.29 | +2.9 % | +178.4 | face-aliased |
| 2026-09-05 20:11 | 1007 | 27.13 | -14.0 % | -162.1 | other-large |
| 2026-09-12 13:27 | 1027 | 1.82 | +2.7 % | -6.0 | benign |
| 2026-09-21 23:23 | 642 | 24.59 | +0.6 % | +89.0 | face-aliased |
| 2026-09-22 00:01 | 561 | 24.25 | +0.8 % | +90.5 | face-aliased |
| 2026-09-22 23:09 | 525 | 24.37 | -2.5 % | +87.5 | face-aliased |
| 2026-10-14 15:36 | 1056 | 1.15 | -4.2 % | -2.4 | benign |
| 2026-10-14 16:25 | 1034 | 0.73 | -0.0 % | -2.7 | benign |
| 2026-10-16 16:00 | 573 | 23.91 | -0.9 % | +88.2 | face-aliased |
| 2026-10-16 19:07 | 1015 | 0.90 | -3.3 % | +2.6 | benign |

The two classes separate completely and differ in kind. The six benign
closures correct the estimate by 0.7 to 1.8 m and turn it by at most 6
degrees: ordinary drift correction. The five face-aliased closures move it
by 24 to 33 m.

## [N].3 The large closures are rotations by a symmetry operation

Four of the five face-aliased closures turn the estimate by 87.5 to 90.5
degrees and the fifth by 178.4, each within 2.5 degrees of a symmetry
operation of the structure, with the radius preserved to within 3 %. A
rotation about the axis is the only thing that preserves radius and turns the
bearing; a wrong match to an unrelated place would do neither. Of the six
large closures, five fall in the 15-degree windows around 90 and 180 degrees,
which together cover a quarter of the circle. If large corrections turned the
estimate by an arbitrary angle, five or more of six landing there would occur
with probability 0.005. This is a descriptive check against a deliberately
simple null, not a substitute for the geometric argument above.

The ground-truth trajectory confirms that these are false matches, by a
test that does not use the rotation geometry at all. For each closure we
measured the distance from the vehicle's true position to the nearest point
of its own track older than 120 s. At the five face-aliased closures that
distance is 18.7 to 20.5 m: the vehicle had never been near that place. At
the six benign closures it is 0.0 to 1.0 m: the vehicle was back where it
had been. The two groups are separated by a factor of eighteen, against a
5 m criterion fixed beforehand. The wrong match is nonetheless to a real
place: after each face-aliased closure the estimate lies within 0.3 m of a
point the vehicle truly visited earlier in the same mission. The timing is
consistent. The face-aliased closures fall between 525 and 776 s, while the
vehicle is scanning faces it has not yet visited; the benign ones fall
between 1014 and 1056 s, when it returns to its starting face, the one
place in a single circuit where a correct revisit exists.

The five also share a destination. Whatever bearing they start from (about
0 degrees in four cases, -89 in one), all five place the estimate at a
bearing of +85 to +89 degrees about the axis. If the confusion were between
arbitrary faces the destination would vary. This suggests a single earlier
face is winning the match, and it matters for any gate designed to reject
such closures. We report it as an observation from five events and have not
tested it prospectively.

## [N].4 What one costs

The best-characterised case is the mission of 2026-10-16 16:00. Before the
closure at 573 s, a similarity fitted to the 7253 preceding SLAM poses leaves
a residual of 0.31 m. After it, the residual over the whole run is 10.5 m,
and the map fragments twice and does not recover. A navigation filter that
trusts SLAM would, from that moment, hold a position wrong by about 24 m
and aim at the wrong face. We have inspected
this consequence in one mission only.

## [N].5 How often

The console logs record 18 closures across 28 full-length missions; 12 are
classified here, as the other six have no recorded bag. At least 5 of 28
missions (18 %, 95 % interval 6 to 37 %) therefore contained a face-aliased
closure. This is a floor, not an estimate: the six unclassified closures can
only raise it. It also mixes configurations, since the missions span two
frame-rate settings and several versions of the controller and alignment code,
so it should be read as "not rare", not as a rate for any one system.

## [N].6 What this does not show

- **Dose-response.** We have not shown that the aliased fraction falls as the
  structure is made less symmetric. A matched comparison of the stock
  structure against the most-outfitted variant was designed and
  pre-registered, but it needs on the order of 30 to 40 missions per arm to
  resolve a difference at the rates seen here and has not been flown.
- **A remedy.** Nothing here shows that rejecting these closures improves
  navigation; that requires the gate, which is evaluated separately.
- **Generality.** All missions are simulated, in one scene, with one
  structure, one SLAM system and one mission profile.
- **Later closures.** Only the first closure in a mission is classified
  against an uncontaminated frame. A closure that corrected an earlier error
  would also be a 90-degree rotation and would be scored as aliased, so
  events after the first are reported in Table [T] but not relied upon.

---

### Status note (not for the manuscript)

**Settled:** the revisit test (`eval/closure_revisit.py`: nearest earlier
visit 18.7-20.5 m at the five face-aliased closures, 0.0-1.0 m at the six
benign, estimate-after within 0.3 m of an earlier true visit; both criteria
fixed before computing); the twelve classifications and the numbers in Table [T], which
come straight from `eval/closure_events.py` on recorded bags; the count of 18
logged closures across 28 missions; the 0.005 and the 6-37 % interval, which
are arithmetic on those counts.

**Not yet verified:**
1. *The shared destination bearing* is five events, found after looking.
2. *The cost* is one mission. The other four aliased missions have not been
   examined for fragmentation or residual.
3. *Turbine axis position* (20, 0) is assumed for all missions. Radii of
   15 to 18 m for every event suggest it holds for the older scenarios too,
   but it was not checked per scenario.
4. The section describes the 5 and 6 as "events" not independent trials: the
   two events in one mission are not independent, and the missions were not
   sampled from a defined population.

**Figures:** made in the journal style (`pubstyle.use_journal`: serif, boxed axes,
inward ticks, no grid), `eval/fig_closures.py` -> `eval/figures/closures_classes`
(double column) and `eval/figures/closures_cost` (single column), .pdf for LaTeX
and .png to preview. Check fonts and column width against the target journal's
author guide before submission; Liberation Serif is a stand-in for Times. Re-run
with `python3 eval/fig_closures.py` (`--refresh` re-reads the bags). The
numbers quoted in section [N].4 (7253 poses, 0.31 m, 10.5 m) are printed by
the extraction and match. The map re-initialisation times in Figure B were
checked against the console log (537 s and 564 s after the closure).

---

### Figure captions (draft)

**Figure [A].** The twelve classified loop closures. **(a)** Plan view, north
up, with the structure at the centre and the 17 m scan orbit dashed. Each
wrong-face closure (orange) is drawn as the arc along the orbit through which
the closure moved the estimate, from the open square to the arrowhead; the four
+88 degree closures overlap. The dotted grey arc is the one large closure that
is not a symmetry operation. Blue points are the six small drift corrections,
all near the start. **(b)** Size of each correction against the turn it makes
about the structure's axis; shaded bands are the 15 degree windows around 90
and 180 degrees and the dashed line is the 3 m threshold for a benign
correction. All five wrong-face closures end on the east side of the structure
(bearing +85 to +89 degrees); this is an observation from five events, found
after looking, and has not been tested prospectively.

**Figure [B].** The cost of one wrong-face closure (mission of 2026-10-16
16:00), for the SLAM pose with its alignment fitted on the 7253 poses before the
closure and held fixed (solid) and for dead reckoning without SLAM (dashed).
**(a)** Bearing error about the structure's axis, kept in the range -180 to 180
degrees with the line broken where it wraps. **(b)** Horizontal position error
as a 10 s trailing RMS, logarithmic axis. Before the closure the SLAM curves
are the fit's own residual, not a prediction (residual 0.31 m, scale 0.955);
the step at 573 s is out of sample. After it the SLAM bearing error leaves zero
by 88 degrees and then wanders through the whole circle. Triangles mark the
two later map re-initialisations (1110 s and 1137 s). Dead reckoning starts
near zero because the filter is initialised from ground truth.

**Read this about Figure B before using it.** It shows that trusting SLAM
through the closure is catastrophic. It does not show that SLAM is the worse
navigator in general: before the closure the SLAM curve (0.1 to 0.7 m) sits
above dead reckoning (about 0.1 m), consistent with the result that dead
reckoning is the better estimate on this mission.
