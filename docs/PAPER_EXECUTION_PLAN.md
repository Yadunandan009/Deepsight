# DeepSight — Journal Submission Execution Plan

**Created:** 2026-10-01 · **Status:** living document, update as workstreams close
**Target:** mid-tier journal — *JMSE*, *Sensors*, *Applied Ocean Research*, or *Ocean Engineering*
**Supersedes:** the task backlog in `SESSION_HANDOFF_2026-09-05.md` §Part 3 (tasks #8-14 are re-scoped here)

---

## 0. Verified state — what actually exists right now

Checked against disk on 2026-10-01, not assumed from prior notes.

| Asset | State | Evidence |
|---|---|---|
| v8 controller, full 4-face mission | Working, reproducible | 7/7 trials `complete`, 1087-1091s each |
| Fixed-matrix baseline batch (#8) | **Done**, metrics extracted, figure built | `eval/results.jsonl` (7 rows), `eval/figures/task8_fixed_matrix_metrics.png` |
| Buggy-matrix ablation (#9) | **Not run** — zero `buggy_*` rows exist | `results.jsonl` has only `buggy_alloc=false` rows |
| Ablation switch mechanism | Working (`BLUEROV2_BUGGY_ALLOC=1` env flag) | `bluerov2_autonomous_controller.py`, verified live |
| Trial harness | Working, hardened | `eval/run_trial.py` (collision guard + stdout capture, both added 2026-09-23/24) |
| Task A — ICP false-loop-closure verification | **Re-run 2026-10-07 under corrected calibration; effect revised from 0.13 to ~0.02.** Write-up not yet updated | `docs/task_a/`, `eval/task_a_reextract.py`, `eval/task_a_out/` |
| Task #15 — SLAM map fragmentation | **Root-caused + written up**; fix not implemented | `docs/TASK_15_SLAM_MAP_FRAGMENTATION_WRITEUP.md` |
| Task #16 — EKF yaw instability | **Fixed + bag-verified** | missing `base_link`↔`imu_filter` TF; `bluerov2_sim.py` |
| Literature base | Exists, ~40 refs, unverified | `~/Desktop/RESEARCH_DIRECTIONS_HANDOFF.md` §351-505 — **not in repo** |
| Structure symmetry index | **Done**, validated against sonar to ~1 cm | `eval/structure_symmetry.py` |
| Symmetry variant ladder | **Done**, 5 levels, index 1.000 → 0.717 | `eval/make_variants.py`, `eval/symmetry_ladder.csv` |
| Symmetry failure axis | **Chosen and measured 2026-10-07**: 90°-periodicity of alignment re-locks. L0 vs L4 matched config, permutation p = 0.0087 | `eval/symmetry_doseresponse.py` |
| Stereo calibration | **Fixed 2026-10-04** (`fx` 457.1 → 417.03); confirmed in map geometry against sonar to 0.08-0.15 m on a 0.74 m predicted effect | `slam_params/bluerov2_stereo.yaml`, c34769e |
| Anchor-free closed loop | **Config complete, never run end to end** — `pose0`/`pose1`/`imu0`/`twist0` wired, `odom0` removed | `src/stonefish_bluerov2/ekf_gpsfree.yaml` |
| GPS-free heading probe | **Passed** 2026-10-02: yaw 0.08° mean, no growth | `eval/gpsfree_probe.py` |
| Depth conversion | **Fixed and verified**, 0.738 m → 0.013 m error | `depth_bridge.py`, `eval/verify_depth.py` |
| v7 baseline / bearing metrics | **Dropped** with the v7-vs-v8 comparison (§1.2); files remain on disk, unused | `eval/bearing_stability_metrics.py` |
| MPC / Fossen / IMM / Mahalanobis prototypes | Exist on Desktop, stale, never executed | forked pre-TF-fix; see §6 |

**The contribution changed on 2026-10-03** (§1, §1.2). The v7-vs-v8 comparison and the
ablation battery are dropped; the symmetry dose-response is now the main result, with
GPS-denied operation as its setting. WS-B below is that dose-response, not the old
Phase 1 work.

---

## 1. The contribution — what this paper actually argues

**Reframed 2026-10-07.** The previous framing made ORB-SLAM3 the subject: *place
recognition fails more on symmetric structures*. That is a benchmark of third-party
software. The failure mode — perceptual aliasing — is textbook, the obvious reviewer
reply is *"use different place recognition, or switch loop closure off"*, and it does not
survive the question of why this is an underwater-inspection paper rather than a SLAM
paper. The objection was raised directly and it is correct.

What this project has that a SLAM paper does not is **the CAD model of the asset**. An
inspection vehicle always knows what it is inspecting; generic SLAM throws that prior
away. The arc now:

> **Offshore jackets are built with 4-fold symmetry for load reasons**, which makes their
> faces geometrically interchangeable — a property of the asset class, computable from the
> drawing before the vehicle enters the water. **Any appearance- or geometry-based place
> recognition must therefore alias between faces**, and it does so with a specific
> geometric signature: the SLAM→world alignment re-locks at multiples of 90°. **The method
> is to exploit the known symmetry rather than be defeated by it** — reject a loop closure
> whose correction corresponds to a symmetry operation of the structure, arbitrated by a
> modality that is blind to appearance (DVL dead reckoning plus depth). **The symmetry
> index predicts how hard that arbitration has to work.**

ORB-SLAM3 is a *component* here, not the object of study. It is the off-the-shelf stereo
front end anyone would use; the claim is not that it fails but that its failure is
**predictable from the asset's geometry** and **correctable with a sensor that does not
care what the structure looks like**.

Each piece is now answerable to a reviewer asking "why this":

- **Why a jacket structure.** Symmetric by engineering design, not by coincidence, so the
  aliasing is systematic across the whole asset class rather than a quirk of one scene.
- **Why GPS-denied.** There is no position fix underwater, and inspection requires
  station-keeping at a fixed standoff, so a 90° place-recognition error is a collision
  risk, not a mapping artefact.
- **Why this controller.** Bearing-hold on each face is what manufactures the
  near-identical viewpoints — the mission profile itself maximises the aliasing risk.
- **Why ORB-SLAM3.** It is the standard open stereo SLAM with an appearance-based loop
  closer. Nothing in the claim depends on it beyond those two properties, which is the
  point: the result should transfer to any system with them.

| # | Result | Status |
|---|---|---|
| R1 | Jacket faces are geometrically interchangeable (ICP fitness 1.000, RMSE ~0.11 m), measured independently from sonar and from CAD | **Done** — algorithm-free and calibration-independent, so it is the foundation the rest rests on |
| R2 | A face's map points show no preference for their own geometry over the adjacent face's | **Done, revised down 2026-10-07** — effect is ~2 points of inlier fraction, not the 0.13 originally reported; see §1.3 |
| R3 | The aliasing carries a 90°-periodic signature whose strength scales with the a-priori symmetry index | **First real evidence 2026-10-07** — L0 vs L4 at matched configuration, permutation p = 0.0087; see WS-B |
| R4 | A symmetry-aware cross-modal gate suppresses the aliased correction | **Promoted: this is now the method half, not optional.** Without it the paper reverts to characterising ORB-SLAM3 |

### 1.1 What counts as a result here, and what does not

Most of what this project spends its time on is **debugging, not research**, and the two
must not be confused in the manuscript.

- **Engineering.** Making your own system work. Generalises to nobody. The missing
  `base_link`↔`imu_filter` TF; the pressure-to-depth conversion off by 226x; an
  `exec VAR=val` shell bug; a sign-reversal metric counting quantisation noise. All real,
  all necessary before any measurement could be trusted, **none of them paper material**
  beyond a line in a reproducibility appendix.
- **Research.** A finding that holds beyond this codebase, that someone else can act on,
  and that was not predictable in advance. R1 and R3 qualify: four-fold symmetry is a
  property of the *inspection target class*, and the ICP procedure is reusable by anyone
  asking whether their SLAM falsely closed a loop.

**Task 9/10 does not go in the paper at all.** Inverting the surge column makes the
vehicle travel backwards; that the mission then fails is derivable in seconds, and ten
trials with p=0.000051 add rigour to something nobody doubted. An earlier draft of this
plan said to "report it as protocol validation" — that was still too generous. *We
verified our code works by breaking it* is a regression test, and any section that
invites the question "what is big about this?" costs more than it earns.

Its value was real but private, and is already banked: it proved the
run → extract → compare → report loop end to end, and surfaced three harness bugs
(silent bag-directory collisions, discarded controller stdout, the `exec VAR=val` shell
bug) that would otherwise have corrupted the symmetry study invisibly. The harness,
metrics extractor and statistics stay. The comparison does not appear in the manuscript.

### 1.2 Dropped, with reasons

- **v7-vs-v8 bearing comparison (old WS-B).** The baseline was a reconstruction of code
  that no longer exists, so the headline would have been "our method beats a baseline we
  rebuilt ourselves". Worse, `current_bearing()` is `atan2` on two known quantities — with
  ground truth supplying position, "noiseless arithmetic beats a noisy sonar centroid" is
  close to tautological, and it does not transfer to the real case where position is
  exactly what you do not know.
- **Ablation battery on our own fixes (old WS-C).** "Our fix fixes our bug" is hygiene,
  not a contribution. The three non-tautological ablations (yaw guard, crab throttle,
  anti-windup) would have been informative, but cost ~20 h of simulation for a secondary
  result that does not survive the "why does this matter" question.

### 1.3 What the Task A re-run changed (2026-10-07)

Task A was re-run under the corrected focal length. Two things came out of it, and the
second matters more than the first.

**The calibration error is confirmed in the map geometry itself.** `fx` was too large by
457.1/417.03 = 1.0961, which inflates every triangulated range by 9.6% and so pulls
surface points toward the structure's axis by a predictable amount. Measured against the
sonar reference — which never depended on `fx` — the pre-fix map sits 8.62 m and 8.71 m
from the axis where the prediction is 8.54 m and 8.55 m: a parameter-free prediction
confirmed to 0.08-0.15 m on a 0.74 m effect. This is a far better validation of the fix
than the bridge-rejection counts, and it belongs in the methods section.

**R2's original effect size was an artefact of an invalid comparison.** The write-up's
headline was a 0.13 gap between the control (0.294) and the two face-2 rows (0.165,
0.158). But fitness is an inlier *fraction of the source cloud*, so those three rows have
different denominators — the comparison the figure invited is not one the statistic
supports. Re-run as a proper paired test (same source cloud, swap only the reference):

| run | face 1 prefers own | face 2 prefers own |
|---|---|---|
| original, fx = 457.1 | **+0.019** [+0.015, +0.023] | −0.008 [−0.014, −0.002] |
| corrected fx, FAST 3/1 | +0.001 [−0.008, +0.008] | −0.007 [−0.016, +0.001] |
| corrected fx, FAST 20/7 | **+0.025** [+0.012, +0.037] | −0.004 [−0.012, +0.004] |

Face 2 fits the adjacent face better in all three runs; face 1 prefers its own in two of
three. The asymmetry is real and replicates in direction, but it is a ~2-point effect.
Report it at that size.

**The methodological lesson, which now applies to every comparison in this project:**
comparing two independently-resampled estimates by whether their error bars overlap tests
the wrong thing, and inflated a 2-point effect into an apparent 13-point one. Where the
two quantities share a source, resample once and score both — the statistic is the
distribution of the *difference*. The same error in its other guise — inferring that two
groups differ because one is significant and the other is not — is why WS-B's
dose-response is tested by permutation on the difference rather than by comparing two
p-values.

## 2. Workstreams

### WS-0 — GPS-free probe (run this before anything else)
**Status:** built 2026-10-02, **not yet run against a live sim**. **Effort:** one mission.

**Why this gates everything.** §1 frames the paper around results that are defensible *given* the ground-truth anchor. But the anchor is also the reason a reviewer can ask "why ORB-SLAM3 at all" and get no good answer — SLAM currently drives nothing. Removing it is the difference between station-keeping and navigation. The infrastructure for anchor-free operation already exists (`depth_bridge`, `dvl_bridge`, `slam_pose_bridge` all run in `bluerov2_sim.py`; SLAM/DVL/IMU are already active in `ekf.yaml`), so the rework may be closer to a config change than a rebuild — **except for heading**, which with `odom0` gone rests on IMU alone, with no magnetometer or gyrocompass to bound drift. That single unknown decides whether this is a one-week job or a research problem.

This probe answers it in one mission.

**Design.** A second `robot_localization` instance runs `src/stonefish_bluerov2/ekf_gpsfree.yaml` (no `odom0`; depth and IMU-yaw re-enabled; `publish_tf: false` so it can't fight the primary node's transforms) as a **passive observer**. The vehicle still flies on the anchored estimate — the mission is unaffected and completes normally — while `eval/gpsfree_probe.py` compares both estimates against the ground truth Stonefish publishes regardless.

**Run it** — three terminals, mission first:
```bash
# 1. normal mission (your usual flow)

# 2. the probe EKF, alongside the live one
ros2 run robot_localization ekf_node --ros-args \
  -r __node:=ekf_filter_node_gpsfree \
  --params-file ~/ros2_ws/src/stonefish_bluerov2/ekf_gpsfree.yaml \
  -r odometry/filtered:=/bluerov2/odometry/filtered_gpsfree

# 3. the monitor — prints a verdict on Ctrl-C
python3 eval/gpsfree_probe.py
```

**Reading the verdict:**
- *heading HELD* — anchor-free rework is the config-scale job it looks like; budget ~1 week, and the first real step is re-pointing `slam_pose_bridge` at the GPS-free estimate (see confound below).
- *BORDERLINE* — bounded but trending up; re-run over a full mission before committing.
- *did NOT hold* — IMU-only heading is insufficient; heading needs a real solution before anchor-free navigation is viable. Do not budget one week.

**Stated confound:** `slam_pose_bridge.py` aligns against `/bluerov2/odometry/filtered` — the *primary, anchored* EKF. So `pose0` is indirectly ground-truth-informed, and the probe's **XY numbers are a lower bound, not a clean GPS-free result**. Yaw comes from `imu0` and is untouched by that path, so the **yaw verdict is valid** — which is the question being asked. The real experiment must re-point that bridge.

**If heading holds**, §1's framing changes substantially: the paper becomes "why GPS-free inspection is hard on lattice structures, with the structural cause quantified," Task #15 becomes the documented cause of navigation failure rather than a SLAM curiosity, and "why ORB-SLAM3" answers itself. Revisit §1 before writing anything.

---

### WS-A — Allocation-matrix ablation (#9 → #10) — **DONE, and CUT from the paper**
**Outcome:** 7/7 fixed complete vs 0/10 buggy, Fisher exact p=0.000051; four of five
secondary metrics separate completely (Cliff's δ = 1.00) over a matched 180 s window.
`eval/compare_alloc.py`, `eval/figures/task10_alloc_comparison.png`.

**This does not go in the manuscript** — see §1.1. The result is tautological, and
presenting it invites exactly the question a reviewer would ask. It earned its keep by
proving the pipeline end to end and by surfacing three harness bugs (bag-directory
collisions silently logging success, controller stdout being discarded, and an
`exec VAR=val` shell bug that killed every buggy trial in 0.3 s) that would have corrupted
the symmetry study invisibly. That is a private benefit, not a published one.

**Two methodological points worth carrying forward:**
- Continuous metrics use a **matched window**, because the arms have very different
  durations and therefore different phase mixes. Comparing full runs would have measured
  the phase mix, not the matrix.
- `orbit_tracking_err` is **undefined** for the buggy arm (0/10 trials ever entered the
  scan band). Reported as such rather than dropped — "never reached the condition the
  metric is defined on" is the stronger statement.

1. Run the buggy-matrix batch (already has a short timeout — the failure fully expresses within ~90s):
   ```bash
   cd ~/ros2_ws && python3 eval/run_batch.py --n 10 --prefix buggy --buggy-alloc --timeout 180 --log eval/results.jsonl
   ```
2. Extract metrics for all `buggy_*` bags via `eval/trial_metrics.py`.
3. Produce the fixed-vs-buggy comparison figure + table (same style as `eval/figures/task8_*`).
4. **Expected result** (observed live on 2026-09-24, not yet replicated): with the buggy matrix the vehicle drifts monotonically away — `trk_r` passed 193 m against a 17 m setpoint, sonar out of range the whole time, mission never completes. So the contrast is *categorical* (completes vs. never completes), not a shift in a continuous metric.
5. **Reporting consequence of that:** a mean±std comparison is the wrong instrument for a categorical outcome difference. Report mission-outcome rate (7/7 vs 0/10) as the primary result, with the continuous metrics as secondary characterisation of *how* it fails.

**Exit criteria:** ≥10 buggy trials logged; comparison figure + table committed; one paragraph written stating the result in the form the paper will use.

---

### WS-B — Symmetry dose-response (R3) — **the stress axis for the method**
**Status:** instrument and ladder done 2026-10-03; **failure axis chosen and first
evidence obtained 2026-10-07** from logs already on disk. **Effort remaining:** ~6 h
simulation to fill the ladder, but *after* WS-C exists (see §4).

**No longer the whole contribution.** Under the §1 reframe this workstream is not the
paper's claim — it is the axis on which the WS-C gate is shown to work and to degrade
gracefully. On its own it characterises ORB-SLAM3; as the stress axis for a method it
characterises the method.

**The instrument.** `eval/structure_symmetry.py` computes face-to-face ICP similarity
from the scenario's CAD. On the stock turbine all six face pairs register at fitness
1.000, RMSE 0.101-0.113 m — against TASK_15's independent sonar-derived 0.996-1.000 and
0.08-0.10 m. Two unrelated measurement paths agreeing to ~1 cm is what licenses using the
cheap one: seconds per variant instead of a mission plus a sonar reconstruction. That is
the only reason a dose-response is affordable at all, and it makes the index *a priori* —
a designer or inspection planner has the CAD before anything is built.

**The ladder** (`eval/make_variants.py`, `eval/symmetry_ladder.csv`). Faces 0 and 1 carry
hardware; faces 2 and 3 stay bare. Two of four is deliberate — the unmarked pair remains
mutually confusable, so failure should fall without reaching zero, which is the
informative case rather than simply deleting the phenomenon.

| level | hardware | index @1.0 m |
|---|---|---|
| L0 | bare | 1.000 |
| L1 | anodes ×8 | 0.959 |
| L2 | + riser | 0.829 |
| L3 | + pipeline | 0.778 |
| L4 | anodes ×20 | 0.717 |

**Three manipulations were discarded before this one**, each caught by looking at the
render rather than at the index — worth recording because the failure mode recurs:
scaling a few objects up gave 7 m barrels and a 22.5 m pipe through the sea surface;
scattering realistic objects put them at r = 13 m, visibly floating, with the jacket
surface at r = 8.0-9.9 m; and drums bolted to a turbine represent nothing that exists
offshore regardless of placement. The current hardware — risers, pipeline, sacrificial
anodes — is what actually differs between faces of a real jacket.

**The failure axis, chosen and measured.** The response variable is **where** the
SLAM→world alignment re-locks, not how often. That distinction is the reframe in
miniature: a rate is a performance number about one implementation, whereas 90°
periodicity is a prediction from the asset's geometry that any appearance-based place
recognition must obey. Measured by `eval/symmetry_doseresponse.py` from the launch logs:

| level | index | n jumps | mean &#124;residual&#124; vs *n*·90° | Rayleigh *R* | *p* |
|---|---|---|---|---|---|
| L0 | 1.000 | 12 | 12.0° | 0.610 | 0.0088 |
| L4 | 0.717 | 11 | 25.6° | 0.254 | 0.50 |

Uniformly random re-locks average 22.5°. Both runs share an identical configuration
(`fx` = 457.1, FAST 3/1), verified by reading each run's ORB-SLAM3 startup block rather
than inferred from timestamps. The difference is tested directly, by permutation on the
pooled jumps (200 000 resamples, direction pre-specified): **Δmean|residual| = +13.5°,
p = 0.0087**; ΔR = +0.356, p = 0.047.

**The rate is not the finding.** L0 re-aligns 13 times and L4 twelve — outfitting the
structure did not make the alignment more stable. It changed *where* the alignment
landed: onto a symmetry-equivalent face, or arbitrarily. A paper that reported only the
rate would have found nothing here.

**L0 clustering is robust to configuration**, which matters because the focal-length fix
is otherwise a confound: L0 clusters at p = 0.0088 (`fx` 457.1, FAST 3/1), p = 0.0213
(`fx` 417.03, FAST 20/7), and at 8.1° mean residual in the third run (n = 7, below the
Rayleigh floor). Three configurations, same signature.

**What is still missing:** L1-L3 entirely, and replication at each level. Two levels give
a direction, not a curve. The config guard in `symmetry_doseresponse.py` refuses to pool
across calibrations by default — the 2026-10-04 fix alone moved re-align counts 13→8,
the same size as the effect being measured, and the L4 run predates it.

**Probe before committing.** Two missions, L0 and L4, ~36 minutes. Clear separation
justifies the full 5 x N batch; no separation means learning that for 36 minutes rather
than 7 hours.

**THE STANDING RISK.** The index is **geometric**; ORB-SLAM3's place recognition is
**appearance**-based. TASK_15 assumed geometric symmetry implies visual symmetry —
plausible, and it was true when the faces were bare, but with textured hardware added
that link is now an assumption rather than a measurement. If the dose-response comes out
flat, suspect this first. The fix is an appearance-side index (bag-of-words similarity
between rendered face views) alongside the geometric one, and **which of the two better
predicts failure would itself be a result** worth more than the curve.

---

### WS-C — Symmetry-aware cross-modal gate (R4) — **the method, and now the contribution**
**Status:** not started. Promoted from optional 2026-10-07: under the §1 reframe this is
what makes the paper a method rather than a characterisation of someone else's SLAM.
**Effort:** moderate. Do this BEFORE filling the ladder, not after — see §4.

**The gate has two tiers, and only the second is research.**

*Tier 1 — reject the full 90° snap.* A 90° re-lock at 9-17 m radius displaces the pose
by 13-24 m. DVL velocity noise is 0.0015 m/s, so dead reckoning holds to roughly
0.5-2 m over a mission; a 13 m jump is trivially rejected. This should simply be shipped
— it is good engineering, and by §1.1's own rule it is not paper material on its own.

*Tier 2 — the sub-threshold case, which is the actual contribution.* The damaging
failures are not the clean snaps but the slow slides already fought in `c4343c0`:
10-14° per 5 s refit, each step inside any single-step outlier gate, compounding past
100°. Those sit under a fixed distance threshold. A CAD-derived symmetry prior can
reject them because it knows *which* rotations are the structure's symmetry operations,
and a drift threshold does not. That is the claim worth making: **prior-informed
place-recognition gating using the inspected asset's own symmetry group.**

**The sensor stack is already wired** — `ekf_gpsfree.yaml` fuses `pose0` (SLAM→bridge,
closed loop, no ground truth), `pose1` (depth, verified to 0.013 m), `imu0` (yaw, 0.08°
mean without the anchor) and `twist0` (`/bluerov2/dvl_twist`, 5 Hz, via `dvl_bridge.py`),
with `odom0` — the ground-truth anchor — removed. It has never been run end to end.
That is one session, not weeks.

Reject a proposed visual loop closure when independent sonar geometry disagrees, and
measure map fragmentation before and after. Turns the paper from "here is a problem" into
"here is a problem and a mitigation", which is the standard arc.

**Expect it may not work, and report that honestly.** The sonar-derived faces are
*themselves* near-identical (fitness ≥0.996 — that is R1), so a geometric gate may have
no more to discriminate on than the visual matcher does. If so, the finding is sharper
than a working gate would have been: *neither appearance nor geometry disambiguates faces
on a radially symmetric jacket; disambiguation requires odometric continuity or an
external reference.* That is a real claim about the problem class.

### WS-D — Consolidate the two existing geometric results
**Status:** both written up; need conversion from internal findings docs into paper sections. **Effort:** ~1 day.

Task A and Task #15 are currently written as internal investigation reports — correct hedging, full method detail, honest limitations. For the paper they need: shared notation with the rest of the manuscript, figures regenerated at publication DPI with consistent styling, and the two ICP methodologies described once rather than twice (they share the per-point re-anchoring and sonar-frustum reference machinery).

Note that #15's write-up deliberately leaves the loop-closure-gating decision open. **Keep it open in the paper too** — it belongs in future work, not results. Implementing and evaluating that gate is a separate contribution and would delay submission.

---

### WS-E — Literature and positioning
**Status:** base exists but is **outside the repo and unverified**. **Effort:** ~1-2 days.

`~/Desktop/RESEARCH_DIRECTIONS_HANDOFF.md` §351-505 already holds ~40 references across five themes, including the closest domain matches (ATLANTIS/WindFloat jacket inspection; Chun et al. 2025 mooring-line tracking at FOWT Hibiki — the nearest peer-reviewed analogue to fixed-standoff structure-relative navigation) and the Stonefish justification chain (Cieślak 2019; Aldhaheri et al. 2025 five-simulator comparison).

1. Move it into the repo (`docs/literature/`) so it's version-controlled alongside the work.
2. **Verify every DOI resolves and every claimed finding is accurately characterised.** These were compiled in an earlier session and have not been checked against the actual papers. Mischaracterising related work is a correctness problem, not a formatting one.
3. Write the positioning paragraph: jacket/lattice-specific structure-relative inspection is thin relative to smooth-hull and pipeline-following work — that gap is the paper's slot.

---

### WS-F — Limitations / threats to validity
**Status:** inputs known, section unwritten. **Effort:** ~half day, but write it early.

Write this section *before* the results are final. It constrains what the results sections are allowed to claim, and drafting it early prevents overclaiming that then has to be unwound.

Known, non-negotiable disclosures:
- **`/bluerov2/odometry` (the EKF's primary anchor) is Stonefish ground truth, not a drifting INS.** This bounds the entire paper: it validates control-loop disturbance rejection and bearing stability, *not* real-world dead-reckoning robustness. State it plainly and early.
- **Simulation only.** No hardware trials.
- **`thrust_coeff=10` (~10 N/thruster) vs real T200 ~88 N** — a known sim/reality gap, unaddressed.
- **v7 baseline is a reconstruction** (WS-B), not recovered code.
- **Task #15's structural-symmetry result is from a single mission's incidents** (n=2 events); the mechanism is clean but frequency claims are not supported.
- **The sonar reference mosaic predates the analysed runs** (built 2026-09-02) and is treated as exact, with no uncertainty model of its own.

---

### WS-G — Reproducibility and repo hygiene
**Status:** partially done; several loose ends. **Effort:** ~half day.

- Commit the outstanding work: `eval/run_batch.py`, `eval/run_trial.py`, the controller's ablation flag, `eval/figures/`, `eval/results.jsonl`.
- **`slam_atlas/deepsight_map.osa` is a recurring problem** — it regenerates to 165 MB and already blocked one push (GitHub's 100 MB limit). Decide now: Git LFS, or `.gitignore` it as a build artifact. It will block a push again otherwise.
- Pin the environment: ROS 2 Jazzy, Stonefish/stonefish_ros2 v1.3, ORB-SLAM3 container tag, exact scenario file. Journals increasingly require this.
- Write the code-availability statement and make sure every reported number traces to a committed script.

---

## 3. Statistics — specify this now, not at writing time

The protocol writes "(p<...)" without naming a test. Fill that in deliberately, because the metrics being compared are mostly **maxima** (`max_abs_sway_mps`, `max_abs_yawrate_*`) and outcome rates, not well-behaved means:

- **Continuous metrics:** Mann-Whitney U (non-parametric; appropriate for N=10-15 and for max-statistics, which are not normally distributed), reported with **median + IQR** alongside mean±std.
- **Effect size, always** — Cliff's delta or rank-biserial correlation. With N≈12, a p-value alone is nearly uninformative; reviewers in this space will ask.
- **Bootstrap CIs** — already this project's established idiom (Task A used 50 resamples); keep it consistent across the manuscript.
- **Categorical outcomes** (complete/estop/timeout): Fisher's exact test, reported as rates with Wilson CIs. This is the right instrument for WS-A's 7/7-vs-0/10 result.
- **Multiple comparisons:** 4 ablations × 4 metrics = 16 tests. Apply Holm-Bonferroni and say so.

---

## 4. Sequencing and critical path

**Reordered 2026-10-07.** The previous ordering said WS-B first, "it is the contribution;
the rework is the setting." Both halves of that are now wrong: WS-C is the contribution,
and doing WS-B first would waste the batch.

```
WS-0 GPS-free probe  [heading PASSED 2026-10-02]
        |
        v
anchor-free closed loop  (ekf_gpsfree end to end, never yet run)   ~1 session
        |
        v
WS-C  tier 1: reject the 90 deg snap via DVL + depth                ~1 session
        |     tier 2: CAD symmetry-group prior  <-- the contribution
        v
WS-B  fill the ladder, L0-L4 x 3, frozen config                     ~6 h sim
        |                                                            |
        +--> WS-F --> write-up <------------------------------------+
WS-D, WS-E, R2 write-up  (no simulation, fully parallel) -----------+
```

**Why WS-B must come last.** Its response variable — where the alignment re-locks — is
measured *through* the bridge and the EKF. The anchor-free rework replaces what feeds
both, and WS-C changes how re-locks are accepted at all. Collecting 15 missions before
those exist voids all 15. This is not hypothetical: it is exactly what happened to the L4
run at 1/15th the scale, when the focal-length fix landed between L4 and L0 and left the
two endpoints on different calibrations.

**Freeze discipline.** Before the first ladder mission, freeze
`slam_params/bluerov2_stereo.yaml`, the gate parameters, and `eval/make_variants.py`, and
do not touch any of them until all runs are recorded. Every config edit mid-collection
costs the runs already banked. `eval/symmetry_doseresponse.py` enforces this after the
fact by refusing to pool across configurations, but that only detects the loss; it cannot
undo it.

**Critical path is now the anchor-free closed loop**, because WS-C depends on it and WS-B
depends on WS-C. It is also the single largest unknown: the bridge's thresholds (5 m
reject, 40-rejection stale, 15° refit cap) were all tuned against a ground-truth-anchored
reference, and the 2026-10-04 log shows the alignment never settling even *with* the
anchor present — eight re-locks, stepping ~90° each time. Remove the anchor and that
instability feeds back into the filter the bridge compares itself against. Watch the first
run live rather than batching it.

**What is already bankable and needs no machine time:** R1, the R2 revision (numbers in
hand), the two-level dose-response result, WS-D, WS-E. These should absorb every period
the simulator is busy.

---

## 5. Risk register

| Risk | Severity | Mitigation |
|---|---|---|
| **Thermal** — ~26h total sustained sim; machine already runs 85-91°C with fans maxed at ~4.1-4.3k RPM under light load, 9°C below throttle | **High** — could corrupt long unattended batches | Check vents/dust before long runs; stage batches overnight rather than back-to-back; `platform_profile` to `cool` if trials start timing out anomalously |
| **Disk** — ~68 trials × ~90 MB ≈ 6 GB of bags; this machine has already hit a disk-space wall once | Medium | Trim `BAG_TOPICS` to only what `trial_metrics.py` reads, or auto-delete bags after extraction; resolve the 165 MB `.osa` question (WS-G) |
| **v7 reconstruction called a straw man** | Medium | Demoted from hook to one of four results (§1); disclosed in methods; R1/R2 carry the paper independently |
| **Ablations come back null** | Medium | A null result is publishable *if* framed honestly ("the fix is defensive against a failure mode that doesn't arise under nominal conditions"). Do not discard runs or retune to chase significance |
| **Literature mischaracterised** | Medium | WS-E verification pass is non-optional — compiled but never checked against the actual papers |
| **Scope creep back into MPC/IMM** | Medium | §6 — explicitly fenced |
| **Reviewer rejects on sim-only** | Known, structural | WS-F honesty + framing as control-loop/bearing validation, not drift robustness. This is the main reason to target mid-tier rather than top-tier |

---

## 6. Explicitly out of scope for this submission

MPC, the Fossen dynamic model, IMM fusion, and the Mahalanobis gating config are **deferred**, for two independent reasons that already exist in evidence:

1. The one numeric check actually run against the fetched Fossen coefficients showed a **3.6× speed overshoot** vs BlueRobotics' documented max speed at real thrust levels — an unresolved parameter-validity problem.
2. **None of that code has ever been executed against the sim.** The three prototypes on Desktop were additionally forked *before* the TF/gimbal-lock fix, so they encode the pre-fix (wrong) understanding of the yaw instability and would need rebasing before they even run correctly.

These belong in **future work as stated directions**, never as claimed results. A natural second paper is "dynamic-model MPC vs. the PID cascade for lattice-structure inspection" — sequenced after the Fossen coefficients are re-identified against this specific sim and the MPC is actually run.

Task #15's loop-closure/merge gate is likewise future work (§WS-D).

---

## 7. Immediate next actions

**Done 2026-10-07** (items 1-2 of the previous list, from logs already on disk rather
than new missions): the failure axis is chosen, and L0 vs L4 at matched configuration
gives the first dose-response evidence (permutation p = 0.0087). The two probe missions
that were supposed to gate the batch turned out to have already been flown on 2026-10-03.

1. **Run `ekf_gpsfree` end to end** — the anchor-free closed loop, never once executed.
   Needs the simulator and ORB-SLAM3, so it is a manual run; watch it live. This is now
   the critical path, because WS-C depends on it and WS-B depends on WS-C.
2. **WS-C tier 1** — reject a re-lock whose correction is a near-90° rotation, arbitrated
   by DVL dead reckoning plus depth. Expected to be easy; ship it as engineering.
3. **WS-C tier 2 — the contribution.** Gate on the structure's CAD-derived symmetry group
   so the slow sub-threshold slides (10-14° per 5 s, `c4343c0`) are rejected too, which a
   fixed distance threshold cannot do.
4. **Then WS-B**, frozen config, L0-L4 × 3. Not before — see §4.
5. **Parallel, no machine needed:** update the Task A write-up to §1.3's numbers and fix
   its stale point counts (it reports 45,884/37,651 new points; the committed code gives
   51,104/43,638 before outlier rejection and 41,106/35,293 after, so the quoted figures
   match neither); regenerate its figures with `eval/fig_task_a.py`, since no figure
   script was ever committed for the originals; move the literature base into
   `docs/literature/` and verify DOIs (WS-E); draft WS-F early.
6. **Housekeeping:** delete the stale `history-fix-20260922` branch and `refs/original/`
   backup refs. The L0 bags live on the `more space` drive, currently unmounted — the
   extracted `.npz` files are in `plots/`, so the analyses are reproducible without it,
   but the raw bags are not.
