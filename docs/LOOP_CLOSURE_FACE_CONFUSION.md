# A loop closure matched the wrong face (2026-10-16)

One event, measured three ways, in `a50_slamned_seed0_20261016_160033`.

## What happened

`slam_atlas/orb_slam3_console_20261016_160002.log` contains exactly one
`*Loop detected`, timestamped between wall 1792147192 and 1792147212.
`slam_pose_bridge` logged `SLAM jump detected 25.24m` at 1792147211.7 --
inside that window. The Atlas then fragmented twice (new maps at wall 1792147748
and 1792147775, 537 s and 564 s after the closure) and never recovered.
(An earlier version gave ~1792147695 and ~1792147770; that was approximate
and 53 s wrong for the first. Corrected 2026-10-17 from the console log.)

Before the loop closure the SLAM track was excellent: a similarity fitted
on the 7253 pre-jump poses gives yaw +7 deg, scale 0.955, **residual
0.31 m**. After it, the whole-run residual is 10.5 m.

## It was a rotation about the turbine axis, by 90 degrees

A face confusion on a four-fold symmetric structure is a specific
geometric object: a rotation of the estimate about the structure's axis
by a multiple of 90 deg. A rotation has two independent signatures, and
magnitude alone is not one of them -- a chord of the right length can
come from anywhere.

Mapping the SLAM pose either side of the jump into the world using the
**pre-jump** alignment only (`eval/jump_direction.py`), so the "after"
point is placed by a transform that knew nothing about the jump:

| | position | radius from axis | bearing about axis |
|---|---|---|---|
| ground truth | (+36.88, -0.00) | 16.88 m | -0.0 deg |
| SLAM before | (+37.26, -0.14) | 17.26 m | -0.5 deg |
| SLAM after | (+20.45, +17.13) | **17.14 m** | **+88.5 deg** |

- **radius preserved**: -0.12 m, **-0.7 %**
- **bearing turned +89.0 deg**, off a right angle by **1.0 deg**
- magnitude: the jump is 24.10 m in world units (25.24 SLAM units x the
  0.955 scale); the chord for 90 deg at r = 17.26 is 24.41 m, a **1.3 %**
  difference

Three quantities, all consistent with one 90 deg rotation about the
turbine axis, and inconsistent with 45, 135 or 180 deg.

## What this does and does not revive

**It revives perceptual aliasing, in a form the earlier refutation did
not test.** `docs/MECHANISM_RESTATEMENT.md` argued aliasing could not own
the error budget because loop closure fires once in three runs. The
inference was wrong. Rarity does not bound severity: loop closure seldom
fires on this structure, and the one firing we have caught cost 10 m of
accuracy and two Atlas fragmentations.

The restated claim is narrower and stronger than the Introduction's:
aliasing here is not a continuous drift mechanism, it is a **rare,
catastrophic, geometrically identifiable** one. Sections 1-3 of the
mechanism restatement still stand -- they concern continuous drift, and
they correctly refute aliasing as its cause.

**It does not revive R3 as measured.** R3 looked for 90 deg periodicity
in the bridge's re-lock angles and returned 4-fold R = 0.253, p = 0.076.
Those angles were recovered by `lock_conditioning.py`, which read the
unconverted SLAM pose -- so R3 was computed through the frame bug and
must be re-run now that the tool is fixed. Do not cite the p = 0.076
until it has been.

## Caveats

**n = 1.** One loop closure, in one run. Everything above describes a
single event in exhaustive detail; it is not a rate, and it does not
establish how often this happens or under what conditions.

The structure has four-fold symmetry, so a +90 deg rotation is the
hypothesis with the most prior support -- but the test cannot distinguish
"matched the adjacent face" from any other mechanism that happens to
rotate the estimate by a right angle about that particular axis. No such
alternative is obvious.

The before/after poses are single samples either side of a discontinuity.
They are not averaged, because averaging across the jump is what hid it.

---

# R3 re-run, and a correction (2026-10-16)

**The claim above that "R3 was computed through the frame bug and must be
re-run" is wrong.** Re-run on exactly the three registered bags, the
statistic is identical: n = 40 after the reconstruction filter, 4-fold
R = 0.253, **p = 0.076**. Not a digit moved.

It could not have moved. The original tool read the raw pose unconverted
and paired it with the published pose, which the bridge had also computed
from the unconverted raw. The pairing was self-consistent, so the
recovered angles faithfully record what the bridge chose. The frame bug
corrupted the bridge's *output*; it did not corrupt the measurement of
that output.

(`lock_conditioning.py` needed repairing anyway for a different reason:
its first fix converted the raw side for the bridge pairing too, which is
only right for post-fix bags. It now picks the convention from the bag's
date -- a fact, not an inference. Choosing by best-fit residual was tried
and selected wrong, 0/13 locks reconstructing within 5 deg against 11/13
for the correct convention.)

## R3's sample is void anyway, for a better reason

Corrected sliding-window rotation, raw SLAM against the SLAM-free
reference, both in NED:

| run | circular dispersion | median step, adjacent 40 s windows |
|---|---|---|
| slamref seed 0 | **0.01** | 1.3 deg |
| slamref seed 1 | **0.07** | 2.9 deg |
| slamref seed 2 | **0.01** | 1.5 deg |
| slamref12 seed 0 | **0.00** | 2.1 deg |

**One fixed SLAM-to-world rotation, for the whole of every mission.** The
map frame never jumped. So there were no re-lock events for R3 to
analyse: the 13-14 "locks" per run were the bridge repeatedly re-solving
a rotation-only model against reflected data, which cannot fit. The
angles it settled on are numerical artifacts of that failure, not
observations of the estimate moving to a wrong face.

So p = 0.076 should not be reported in either direction. It is a test of
the wrong population, and the fact that it is "nearly significant" is an
accident of what a failing least-squares fit happens to return.

## What is left

The only genuine face confusion in six runs is the single loop closure
documented above: 89.0 deg about the turbine axis, radius preserved to
0.7 %, magnitude within 1.3 % of the predicted chord.

That is n = 1 -- but it is a real event with three independent
signatures, where R3 had 40 measurements of nothing. For the paper, one
clean mechanism beats forty artifacts, and it should be written up as a
case study of a predicted failure observed, not as a rate.
