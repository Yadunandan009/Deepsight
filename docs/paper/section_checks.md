# [N]. Checking a closure

*Draft 2026-10-17. Results section. Evidence: `eval/gate_sweep.py`,
`eval/fig_gates.py`; numbers in `eval/figures/gate_results.json`. Section and
figure numbers are placeholders.*

A wrong-face closure is large and, by construction, to a place the vehicle has
not been. Three checks exploit different parts of that. We score each on the 11
closures that are cleanly classified (five wrong-face, six benign; the twelfth
follows an earlier error in its mission). A check *flags* a closure as suspect;
a wrong-face closure that is flagged is a true positive and a benign closure that
is flagged is a false alarm.

- **Size.** Flag the closure if its correction exceeds a tolerance plus three
  times the dead-reckoning uncertainty at that time: too large to be drift.
- **Revisit.** Flag the closure if the dead-reckoned track says the vehicle has
  not been near this place before, that is, the nearest earlier dead-reckoned
  position (at least 120 s old) is farther than the same tolerance.
- **Chord.** Flag the closure if the length of its correction equals the chord
  of a symmetry operation of the orbit, 2R sin(pi k / n). For this orbit
  (R = 17 m, n = 4) that is 24.0 m for a quarter turn and 34.0 m for a half
  turn. This uses neither dead reckoning nor ground truth: R comes from the
  mission and n from the structure's design.

## [N].1 On the recorded closures

The simplest case is the one the vehicle actually flew. Where the SLAM-free
filter was recorded (four of the eleven closures), the revisit distance is
20.3 m for the wrong-face closure and 0.5, 1.0 and 1.1 m for the three benign
ones, with dead-reckoning error of 0.06 to 0.27 m at those points. Taking the
true track instead gives the same picture for all eleven: 18.7 to 20.4 m for
wrong-face and 0.0 to 1.0 m for benign, a margin of 18. The size of the
correction separates the classes by 13 (23.9 to 33.3 m against 0.7 to 1.8 m).
**On a single lap any threshold between these ranges works, and the three checks
are interchangeable.**

## [N].2 As dead reckoning drifts

The checks differ in what happens when dead reckoning is not good. We inject
drift into the true track, growing linearly with distance travelled to X metres
at the end of the mission in a random fixed direction (20 draws per mission),
and give the size and revisit checks a tolerance of 3 m plus three times the
uncertainty that drift implies. A benign closure then corrects the drift
accumulated between its two visits; a wrong-face closure's size is a property of
SLAM's own frame and does not change.

Figure [G]. Up to 5 m of drift all three checks catch every wrong-face closure
and raise no false alarm. From 7 m the revisit check begins to miss them (79 % caught,
then 57 % at 10 m and 11 % at 14 m) and the size check follows (80 % at 10 m,
40 % at 14 m); both catch none by 18 m, because the uncertainty they are
allowed has grown to exceed the separation they rely on. The chord check
catches every wrong-face closure at every drift tested, because the jump's
length does not depend on dead reckoning. It pays for that only at the far end:
from about 24 m of drift, honest corrections have grown to the length of the
jumps and it raises false alarms (50 % at 24 m, 33 % at 30 m, 100 % at 40 m).
**There is a window, roughly 7 to 18 m of drift, in which only the symmetry-based
check separates the classes.**

## [N].3 Where that window sits

A single lap is nowhere near it: dead reckoning ended 0.1 to 0.5 m out over three
DVL units and 0.2 m RMS with a gyro drifting at 3 degrees per hour. Reaching
7 m takes, by order-of-magnitude arithmetic on the lap's own rates, 1 to 5 hours
of operation with a gyro drifting at 15 to 3 degrees per hour (taking a 30 m
lever arm), or 2 to 9 km of path on DVL scale and misalignment alone, which at
this vehicle's lap speed is several hours to a day. The window is a
long-mission regime. It is not reached by anything flown here, and it is not
reached by lengthening the missions we can fly in a session.

## [N].4 What this does and does not show

- It shows that **on the missions flown, a plain check is enough**, and that
  the symmetry-based check earns its place only when dead reckoning has drifted
  by metres.
- It does **not** show the chord check works in the loop. The drift is injected
  under a stated model, and nothing here was run closed-loop.
- The chord check was specified after the five wrong-face chords had been seen,
  so it is **in-sample**. Its tolerance (10 %) was set from the spread of SLAM
  scale and orbit radius before it was scored, but a blind test needs closures
  not yet recorded.
- The false-alarm band suggests combining it with a plain check. We did not
  test that.
- Rejecting false loop closures is an established problem, and checking a
  closure against odometry in repetitive environments has been done recently
  [ROVER]. The contribution here is the symmetry-operation form of the test and
  where it matters, not the idea of a check.

---

### Figure caption (draft)

**Figure [G].** Three closure checks against dead-reckoning drift, scored on
five wrong-face and six benign closures. Drift is injected into the true track,
growing linearly with distance to the value shown on the horizontal axis at the
end of the mission; this is an assumption, not a flown condition. **(a)** Per
cent of wrong-face closures flagged. **(b)** Per cent of benign closures flagged.
The dark band marks the dead-reckoning error of a single lap (0.1 to 0.5 m). The
light bands mark where the plain checks fail (7 to 18 m) and where honest
corrections become as large as the jumps (from about 22 m). The chord check uses
no dead reckoning and was specified after the wrong-face chords had been seen.

### Status note (not for the manuscript)

**Settled:** the numbers in 1 and 2 (`gate_results.json`, 20 random drift
directions, fixed seed); the arithmetic in 3, which is order-of-magnitude.
**Not yet verified:** (a) ROVER (arXiv 2508.13488) must be read and the
difference between its trajectory prior and the revisit check stated; the
placeholder cites it by name only. (b) The 10 % chord tolerance and R = 17 m are
design choices, not fits. (c) The injected-drift model is linear in distance;
heading drift is not, and a model with heading drift would move the window.
(d) Four closures have real dead reckoning; seven have only the true track.
