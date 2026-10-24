# Pre-registration: does breaking the symmetry remove the wrong-face closure? (time-boxed, 2026-10-21)

Written before any L4 flight. Supersedes the sizing in section 5 of
`PREREG_closure_events.md` (that sizing assumed a first-closure rate of 0.18;
all three flights of 2026-10-20 had a wrong-face first closure, so the rate in
this configuration is far higher and a small design can resolve the question).
Hypotheses, classifier and void rules of that document stand.

## Question

The only evidence that symmetry causes the wrong-face closures is circumstantial
(90-degree turns about the axis, radius preserved). Does a structure with the
symmetry broken show fewer of them? This is the control a reviewer will ask for.

## Arms

- **L0**: stock structure (`bluerov2_turbine`; identical to `bluerov2_turbine_sym_l0`
  apart from a comment). Symmetry index about 1.0 from CAD.
- **L4**: `bluerov2_turbine_sym_l4`: hardware on faces 0 and 1 (bearings 180 and
  90 degrees): 12 anodes, 16 riser segments and a pipeline run; faces 2 and 3
  bare. CAD symmetry index 0.717 in `PREREG_closure_events.md`; faces measured
  distinguishable today (cross-face spread 0.584 to 1.000). The earlier text says
  "20 anodes"; the scenario file holds 12 anodes plus the risers and pipeline.

Everything else is unchanged: controller, SLAM configuration, bridge, `rviz:=false`,
`dvl_error:=a50`, `dr_reference:=true`.

## Design

Paired by DVL unit: the same six seeds in both arms, **0, 1, 2, 29, 3, 26**.

- **L0 for seeds 0, 1, 2 already exists** (flights of 2026-10-20, arm `slamref`).
  Disclosed departures from the earlier protocol: these were flown before the L4
  flights, not interleaved, and the runtime code has not changed between them.
- **L0 for seeds 29, 3, 26** are the flights 5 to 7 of `RUNSHEET_remeasure.md`,
  flown only if gate A passes.
- **L4**: six flights, seeds in the order 0, 1, 2 then 29, 3, 26. Arm name `symL4`.

Maximum new flights: **9** (6 L4 and 3 L0).

## Endpoint and test

**Primary:** a mission's *first* closure, classified by `eval/closure_events.py`,
is face-aliased (yes/no). A mission with no closure counts as not aliased and is
reported separately.
**Test:** one-sided Fisher exact, L0 against L4, alpha 0.05, unit = mission.
Unscorable first closures are scored both ways; the conclusion must hold under
both or it is reported as unresolved.

Exact p, n = 6 per arm, L0 = 6/6 aliased: L4 = 0/6 gives 0.001; 1/6 gives 0.008;
2/6 gives 0.030; 3/6 gives 0.091. Power at n = 6: about 0.9 if the true rates
are 0.9 against 0.1, about 0.55 if 0.9 against 0.3. **A non-significant result is
reported as "not detected at this power", never as "no effect".**

**Secondary, descriptive:** destination bearing of aliased closures (S2); time of
the first closure; whether closures occur at all in L4; fused-filter error after
the closure in each arm; the closure's phase of the mission (all six earlier
closures came at the start of the rise).

**Known confound, stated now:** the controller path is the same in both arms and
every closure of 2026-10-20 came at the start of the rise. If L4 does not change
that, the closure is driven by the manoeuvre and not by the structure's
appearance, and that is itself the result.

## Void rules

A flight is void if SLAM never initialises, the recorder fails, the vehicle
collides, the mission does not complete, the console log is missing, or the
vehicle's orbit radius departs from the L0 flights' by more than 1 m (the L4
hardware could disturb the sonar-based controller). Voids are counted, reported,
and replaced by a repeat of the same seed in the same arm.

## Stop rules (the time-box)

Set now, so that "not working out" is not decided afterwards:

1. **Phase 0 fails** (L4 will not launch, or the first L4 flight is void twice):
   stop; write the paper with what exists.
2. **Gate A, after L4 seeds 0, 1, 2:**
   - L4 aliased in **2 or 3 of 3**: no reduction detected. Stop chasing. Report as
     "not reduced by hardware on two faces at n = 3; the closure appears tied to
     the manoeuvre", and write the paper with what exists. Flights 5 to 7 are then
     optional, only if time remains.
   - L4 aliased in **0 or 1 of 3**: continue to the full 6 against 6.
3. **No new flight starts after 18:00 on 2026-10-21** (set by the user at the start
   of the day). Whatever is flown by then is analysed as pre-registered,
   with n stated.
4. **No optional stopping inside a stage:** each stage is completed or abandoned,
   not peeked at.
5. After the last decision, the paper is written with the result as it stands.

---

## Outcome (written 2026-10-21 17:55, before any further flight)

**Gate A reached.** L4 seeds 0 and 1 flown, both valid (DVL unit verified, orbit radius
within 0.3 m of L0, mission complete). Both first closures **face-aliased**:

| seed | closure time | chord | turn about axis | radius change | fused error 700 s / 900 s | DR error 700 s / 900 s |
|---|---|---|---|---|---|---|
| 0 | 331 s | 24.2 m | +89.8 deg | +1.1 % | 4.6 / 4.3 m | 0.09 / 0.03 m |
| 1 | 336 s | 23.8 m | +89.8 deg | +0.1 % | 4.8 / 4.4 m | 0.23 / 0.21 m |

L4 aliased 2 of 2. Stop rule 2 is satisfied without seed 2 (at least 2 of 3 whatever
seed 2 does). Per the pre-registration: stop chasing; flights 5 to 7 and L4 seeds
29, 3, 26 are not flown.

**The control did not test what it was meant to test.** The CAD pairwise table for L4
(`eval/structure_symmetry.py --scn ...sym_l4.scn`) gives face 0 vs face 1 fitness 0.947
(RMSE 0.27 m) and face 2 vs face 3 fitness 1.000. Hardware was put on faces 0 and 1 in
the same pattern, so those two are nearly interchangeable with each other; the faces
that differ are the 0/1 pair against the 2/3 pair. Both observed closures are a +90 degree
turn, from the face being scanned (0) to the next one (1), exactly the pair L4 left
alike. The mean index (0.716) hid this; the pre-registration did not look at the pairs.
Consequence: the result cannot be read as "hardware on two faces does not remove the
confusion". It is "a design that left faces 0 and 1 alike did not remove a 0/1
confusion". The symmetry hypothesis is neither supported nor refuted by this control.

**Other differences from L0.** Closures came at about 330 s (arrival at face 1), not at
the start of the rise (1018 to 1034 s), so the L0 timing was not a property of the
manoeuvre. Fused error after the closure was 4 to 5 m for the rest of the mission, not
0.2 to 0.5 m: the bridge's reset worked, but its later re-alignments accepted fits to
the wrong frame. The L0 "contained" result may reflect only that the mission ended
about a minute after the closure.

### Addendum 2026-10-21 21:30: RISE-fix test flight (not part of the pre-registered design)

Flight `a50_risefix_seed0_20261021_205753` (stock scenario, DVL seed 0, controller with
the RISE change: RISE holds the turbine heading, the turn home happens in RETURN_HOME).
Flown after Gate A, disclosed here, not used in the Fisher test.

- RISE fix worked: the log shows "deferred to RETURN_HOME; RISE holds turbine_psi"; true heading stayed
  within about 10 deg of the turbine bearing through RISE; the map did not collapse in RISE or at the surface.
- First closure came at 758 s (FACE-ALIASED, +179.7 deg, chord 32.8 m), mid-mission during the transit to
  face 3, not at the start of the rise as in the three earlier L0 flights (1018 to 1034 s). A second large
  jump followed at 802 s (OTHER-LARGE, pre-fit too poor to score well) and the map collapsed at about 14 s
  after it, in SCAN of face 3, long before RISE.
- Fused filter error 4.6 m at 1000 s, 1.4 m at 1200 s; dead reckoning 0.03 to 0.15 m throughout. The vehicle
  finished 4.2 m from home (dead reckoning 4.2 m, filtered believed about 2 m) because it navigated on
  the fused estimate.
- Consequence: the closure time varies run to run on the same scenario and seed (1034 s vs 758 s), so the earlier L0
  "closures at the start of the rise" were not a fixed property, and the L4 mid-mission timing is not evidence
  of an arm effect. The "containment" seen in flights 1 to 3 holds only for closures near the end of a mission.
