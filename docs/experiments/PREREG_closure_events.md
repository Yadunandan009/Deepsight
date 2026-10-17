# Pre-registration: loop-closure events and the symmetry dose-response (R3, rebuilt)

**DRAFT 2026-10-17.** Registered by the commit that adds this file; the
amendment log at the end is append-only. Two fields marked **[USER]** must be
filled before the first confirmatory flight.

## Why this replaces the old R3

The old R3 looked for 90-degree periodicity in the bridge's re-lock angles.
That sample is void: those angles measured a failing least-squares fit
caused by the unconverted ENU/NED frame (`docs/SLAM_FRAME_ENU_NED.md`).

The event that actually carries aliasing is the ORB-SLAM3 **loop closure**.
It is discrete, it is logged, and it is geometrically classifiable: on a
four-fold symmetric structure a wrong-face closure is a rotation of the
estimate about the structure's axis by 90 or 180 degrees, with the radius
preserved. A correct closure moves the estimate by a drift correction.

## 1. What is already known -- exploratory, NOT confirmatory

`eval/closure_events.py` was run over every bag on disk that has the SLAM
track, ground truth and a matching ORB-SLAM3 console log with at least one
`*Loop detected`. Order of looking, for the record: the rule's geometry
(the chord for 90 deg at r = 17 m is 24 m, so a 3 m floor separates drift
corrections from rotations with a wide margin) was set after the two
2026-10-16 events; the other nine bags were classified afterwards **with
no change to any threshold**.

| bag | t (s) | chord (m) | radius | turn (deg) | class |
|---|---|---|---|---|---|
| DeepSight_20260905_141654 | 1028 | 0.80 | -1.4 % | +2.5 | benign |
| DeepSight_20260905_173822 | 1014 | 0.87 | +2.8 % | -2.4 | benign |
| DeepSight_20260905_201108 | 776 | 33.29 | +2.9 % | +178.4 | **face-aliased (180)** |
| DeepSight_20260905_201108 | 1007 | 27.13 | -14.0 % | -162.1 | other-large (fit resid 1.31 m) |
| DeepSight_20260912_132725 | 1027 | 1.82 | +2.7 % | -6.0 | benign |
| DeepSight_20260921_232316 | 642 | 24.59 | +0.6 % | +89.0 | **face-aliased (90)** |
| DeepSight_20260922_000133 | 561 | 24.25 | +0.8 % | +90.5 | **face-aliased (90)** |
| DeepSight_20260922_230945 | 525 | 24.37 | -2.5 % | +87.5 | **face-aliased (90)** |
| a50_slamref_seed0_20261014_153610 | 1056 | 1.15 | -4.2 % | -2.4 | benign |
| a50_slamref_seed2_20261014_162539 | 1034 | 0.73 | -0.0 % | -2.7 | benign |
| a50_slamned_seed0_20261016_160033 | 573 | 23.91 | -0.9 % | +88.2 | **face-aliased (90)** |
| a50_slamdrift_seed0_20261016_190745 | 1015 | 0.90 | -3.3 % | +2.6 | benign |

12 events in 11 bags: **5 face-aliased, 6 benign, 1 other-large**.

**These counts are floors, not rates.** The logs contain 18 closures across
28 full-length missions; 12 have a bag and were classified, 6 have none.
"At least 5 of 28 missions (18 %, 95 % CI 6-37 %) lost a mission to a
face-aliased closure" is the defensible reading, and the true figure can
only be higher. The runs span several configurations (fps 30/12, pre- and
post-frame-fix controllers), so this is a mixture, not one rate.

Exploratory observations, stated so they can be tested rather than
remembered:

- **Every benign closure** is at t = 1014-1056 s, the end-of-circuit return
  to face 0, with chord 0.7-1.8 m and |turn| <= 6 deg.
- **Every aliased closure** is mid-mission, t = 525-776 s, with the turn
  within 2.5 deg of 90 or 180 and the radius within 3 %.
- **All five land on the same bearing**, +85 to +89 deg about the axis,
  whatever face they started from (four from ~0 deg, one from -89 deg).
  Random face confusion would scatter the destination; this looks like a
  specific earlier face winning the match.
- **After an aliased closure the map does not recover** (the Atlas
  fragments; the second event in `..._20260905_201108` is classified
  other-large and its pre-event fit is poor at 1.31 m).

## 2. The measurement -- frozen

`eval/closure_events.py`, sha256 `383ad7deedcdce2859f942c2456b73af107cb7b13634722f60aca92103c20063`.

Each `Loop detected` in the console log is bracketed by the nearest
timestamped lines and the following `Local Mapping RELEASE`. The largest
consecutive step of the raw SLAM track within [t_lo - 3 s, t_hi + 30 s] is
the correction. A similarity SLAM->world is fitted on the preceding 150 s of
poses only (clipped at any earlier event), so the "after" point is placed by
a transform that knew nothing about the jump. Ground truth scores; nothing
here feeds navigation.

| constant | value | meaning |
|---|---|---|
| `BENIGN_CHORD` | 3.0 m | below this a closure is a drift correction |
| `RADIUS_TOL` | 0.10 | fraction of r a rotation may change it |
| `TURN_TOL` | 15 deg | about 90 or 180 |
| `PRE_FIT_S` / `MIN_FIT_POSES` / `MIN_FIT_SPREAD` / `MAX_FIT_RESID` | 150 s / 100 / 5 m / 1.5 m | below these the event is **unscorable** |

Classes: `benign`, `face-aliased`, `other-large`, `unscorable`.
Unscorable events are counted and reported, never dropped.

**Known limit, and why it shapes the design.** Only the *first* closure in a
mission is classified against an uncontaminated frame. A later closure that
*undoes* an earlier error is a rotation of the same size and would be scored
face-aliased. The tool has no way to tell the two apart. Therefore **the unit
of analysis is the mission, and the confirmatory event is its first
closure.** Later events are reported descriptively.

## 3. Hypotheses

Stock `bluerov2_turbine_sym_l0` (symmetry index 1.000) against
`bluerov2_turbine_sym_l4` (index 0.717: 20 sacrificial anodes on faces 0-1,
faces 2-3 left bare), single-circuit missions.

- **H1 (existence).** In L0, the probability that a mission's first closure
  is face-aliased exceeds **0.05**. One-sided exact binomial, alpha = 0.05.
  0.05 is the operating threshold: one mission in twenty ruined is not an
  acceptable inspection failure rate whatever its cause.
- **H2 (dose-response).** That probability is lower in L4 than in L0.
  One-sided Fisher exact, alpha = 0.05, unit = mission.

**Not supported if** H1's one-sided 95 % lower bound is <= 0.05, or H2's
p > 0.05, or the L4 proportion is >= the L0 proportion. A non-significant
H2 is reported as "not detected at power X", never as "no effect".

Unscorable first closures are handled by reporting the primary endpoint
twice, once scoring them all as aliased and once as none; the conclusion
must hold under both or it is reported as unresolved.

## 4. Secondary, descriptive only

- **S1.** Among missions with at least one closure, the aliased fraction per
  arm with a Clopper-Pearson interval. This separates *fewer closures* from
  *fewer wrong closures*, which H2 alone cannot.
- **S2.** Destination bearing of aliased first closures. Prediction from
  section 1: at least 80 % land within 15 deg of one bearing.
- **S3.** All closures including later ones, tabulated, with the
  contamination caveat of section 2.

## 5. Design

Identical in both arms except the scenario:

    ros2 launch stonefish_bluerov2 bluerov2_sim.py \
        scenario:=bluerov2_turbine_sym_l{0|4} dvl_error:=a50 \
        dvl_seed:=<k> dr_reference:=true
    bash run_stereo_slam.sh
    python3 src/stonefish_bluerov2/scripts/bluerov2_autonomous_controller.py
    bash record_run.sh sym{L0|L4} <k>

- Controller **unchanged**: single circuit. SLAM config
  `slam_params/bluerov2_stereo.yaml` as committed. ORB-SLAM3 stock.
- Seeds `k = 0 .. n-1`, **the same list in both arms**.
- **Interleaved** L0, L4, L0, L4, ... (starting arm by coin flip) so slow
  changes in the machine or the build cannot masquerade as an arm effect.
- A mission is **void** if SLAM never initialises, the recorder fails, the
  vehicle collides, or the console log is missing (closures would be
  unobservable). Voids are counted and replaced by the next seed in the
  same arm; the count is reported.
- No interim looks, no optional stopping: **n is fixed before flight 1.**

**Sample size [USER].** Exact power of the one-sided Fisher test, n missions
per arm:

| n / arm | L0 .18 vs L4 .00 | L0 .18 vs L4 .05 | L0 .30 vs L4 .05 | L0 .30 vs L4 .10 |
|---|---|---|---|---|
| 12 | 0.16 | 0.09 | 0.32 | 0.20 |
| 20 | 0.28 | 0.16 | 0.54 | 0.34 |
| 30 | 0.65 | 0.28 | 0.75 | 0.49 |
| 40 | 0.87 | 0.44 | 0.89 | 0.66 |

At the historical floor of 0.18, **n = 20 per arm (40 flights, ~13 h) has
28 % power for H2.** Reaching 87 % takes 40 per arm (80 flights). H1 needs
at least 4 aliased of 20 L0 missions to clear its bound.

Fill in before flight 1:  **n per arm = [USER]**.  Recommended: decide
first whether H2 is to be confirmatory at all. If the budget cannot reach
~30 per arm, register H2 as *descriptive* (effect size with interval) and
keep H1 as the only confirmatory test; that is a smaller claim stated
honestly in advance, which is better than a larger one that cannot be
resolved.

## 6. Deferred, deliberately

**Multi-circuit missions are not in this design.** They were proposed to
raise the event count, and section 1 argues against it: the first closure is
the only cleanly classifiable one, aliased closures occur mid-mission before
a second circuit begins, and an aliased closure fragments the map so later
circuits in that run yield nothing scorable. More circuits would add events
the analysis cannot use. Revisit only if the single-circuit L0 arm shows
first-closure rates too low to resolve H1.

**R4 (the gate)** is a separate pre-registration. It needs the gate to
exist and will use the same classifier as its ground truth for "this
closure was aliased".

The earlier L0/L4 table in `docs/PAPER_EXECUTION_PLAN.md` (Rayleigh
R = 0.61 vs 0.25) was computed from re-lock angles, shares the void
premise above, and cannot be re-scored: no bag with SLAM and truth matches
those logs.

## Amendment log

*(append-only; each entry dated, with the reason, before any data it affects)*

**2026-10-17, before any confirmatory data.** Section 1's bullet "All five land
on the same bearing ... whatever face they started from" overstated the
evidence. Four of the five closures start from the same face and turn the same
way, so they necessarily end together; only two starting faces are
independent (about 0 and -89 degrees), both ending at about +88 degrees. S2
(at least 80 % of aliased first closures land within 15 degrees of one
bearing) would therefore be met by construction whenever the start faces
repeat, which they do with an unchanged controller. **S2 is amended:** it is
informative only if the aliased first closures come from at least three
distinct starting faces, and it is to be reported as the number of distinct
(start face, end face) pairs rather than as a proportion landing on one
bearing.
