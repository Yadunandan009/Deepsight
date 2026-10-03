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
| Task A — ICP false-loop-closure verification | **Done**, written up with figures | `docs/task_a/TASK_A_ICP_LOOP_CLOSURE_WRITEUP.md` |
| Task #15 — SLAM map fragmentation | **Root-caused + written up**; fix not implemented | `docs/TASK_15_SLAM_MAP_FRAGMENTATION_WRITEUP.md` |
| Task #16 — EKF yaw instability | **Fixed + bag-verified** | missing `base_link`↔`imu_filter` TF; `bluerov2_sim.py` |
| Literature base | Exists, ~40 refs, unverified | `~/Desktop/RESEARCH_DIRECTIONS_HANDOFF.md` §351-505 — **not in repo** |
| Structure symmetry index | **Done**, validated against sonar to ~1 cm | `eval/structure_symmetry.py` |
| Symmetry variant ladder | **Done**, 5 levels, index 1.000 → 0.717 | `eval/make_variants.py`, `eval/symmetry_ladder.csv` |
| Symmetry failure axis | **Not measured** — probe pending | — |
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

**Decided 2026-10-03.** The arc is:

> **GPS-denied operation is the setting** that makes SLAM load-bearing. **Structural
> self-similarity is the finding**: jacket faces are geometrically interchangeable, and
> place-recognition failure scales with how interchangeable they are. **A cross-modal
> gate is the method half**, if time allows — and a negative result there is still a
> result.

Without the GPS-denied setting a reviewer asks why ORB-SLAM3 is in the paper at all,
because with the ground-truth anchor in place SLAM drives nothing. Without the
dose-response the symmetry work is a single observation on a single structure, which is
a problem statement, not a finding.

| # | Result | Status |
|---|---|---|
| R1 | Jacket faces are geometrically interchangeable (ICP fitness 1.000, RMSE ~0.11 m), measured independently from sonar and from CAD | **Done** |
| R2 | A suspected false loop closure, confirmed against independent geometry | **Done** (Task A) |
| R3 | Place-recognition failure rate scales with a computable a-priori symmetry index | **In progress** — ladder built, failure axis not yet measured |
| R4 | Gating loop closure on an independent modality does / does not mitigate it | Optional method half |

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

**Task 9/10 is the degenerate case.** Inverting the surge column makes the vehicle travel
backwards; that the mission then fails is derivable in seconds. Ten trials and
p=0.000051 added rigour to something nobody doubted. Its value was instrumental — it
proved the run → extract → compare → report pipeline and surfaced three harness bugs that
would have corrupted later work silently. **Report it as protocol validation, not as a
finding.**

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

### WS-A — Close the allocation-matrix ablation (#9 → #10) — **DONE 2026-10-03**
**Outcome:** 7/7 fixed complete vs 0/10 buggy, Fisher exact p=0.000051; four of five
secondary metrics separate completely (Cliff's δ = 1.00) over a matched 180 s window.
`eval/compare_alloc.py`, `eval/figures/task10_alloc_comparison.png`.

**Read §1.1 before using this anywhere.** The result is tautological as science — inverting
the surge column makes the vehicle go backwards, and the rest follows. Its value was
proving the pipeline end to end, which it did, including by surfacing three harness bugs
(bag-directory collisions silently logging success, controller stdout being discarded, and
an `exec VAR=val` shell bug that killed every buggy trial in 0.3 s) that would have
corrupted WS-C invisibly. Present it as protocol validation, not as a finding.

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

### WS-B — Symmetry dose-response (R3) — **the main contribution**
**Status:** measurement instrument and variant ladder done 2026-10-03; failure axis not
yet measured. **Effort:** ~7 h simulation plus analysis, once the probe justifies it.

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

**The failure axis is not yet chosen.** Candidates, all from existing tooling: re-aligns
per mission (`eval/slam_align_harvest.py`), map-fragmentation events (detected in
`slam_pose_bridge`), tracking-loss episodes (ORB-SLAM3 console). Re-aligns are the most
direct, but a stock mission yields only 2-4, so if L4 drops to 0-1 the levels may not
separate without many missions each.

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

### WS-C — Cross-modal loop-closure gate (R4) — optional method half
**Status:** not started; recommended by TASK_15 but deliberately left open there.
**Effort:** moderate; only attempt once WS-B lands.

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

```
WS-0 GPS-free probe  [heading PASSED 2026-10-02]
        |
        +--> anchor-free rework (the setting)  ~1 week  --+
        |                                                 |
WS-B symmetry dose-response  [ladder done]                |
        |                                                 +--> WS-F --> write-up
        +--> L0/L4 probe (36 min) --> 5 x N batch (~7 h)  |
                      |                                   |
                      +--> WS-C gate, if time ------------+
WS-D, WS-E  (no simulation, fully parallel) --------------+
```

**Critical path is WS-B's failure axis.** Everything else is either done, optional, or
parallelisable. The L0/L4 probe gates the 7-hour batch and should always run first.

**WS-D and WS-E need no machine time** and should absorb any period where the simulator
is busy — which, given the batch sizes here, is most of them.

**The anchor-free rework and WS-B are independent** and can proceed in either order. Doing
the rework first makes the dose-response more meaningful (SLAM actually drives the
vehicle, so place-recognition failure has mission-level consequences rather than just
corrupting telemetry). Doing WS-B first de-risks the contribution. If time is tight,
WS-B first — it is the contribution; the rework is the setting.

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

1. **L0 / L4 probe missions** (~36 min) — gates the 7-hour batch. Needs ORB-SLAM3 running,
   so it is a manual run rather than `run_trial.py`, which does not launch the container.
2. **Choose the failure axis** from the probe: re-aligns per mission, map-fragmentation
   events, or tracking-loss episodes.
3. **If the probe separates**, run 5 levels x N missions and fit failure against index.
   **If it does not**, test the appearance-side index before concluding the effect is
   absent (see WS-B's standing risk).
4. **Parallel, any time, no machine needed:** move the literature base into
   `docs/literature/` and verify the DOIs (WS-E); draft the limitations section (WS-F)
   *before* the results are final, so it constrains what they are allowed to claim.
5. **Still open from earlier:** the anchor-free rework's first full run has not happened.
   `slam_pose_bridge` is parameterised and `ekf_gpsfree.yaml` points at it, but the
   bridge's thresholds (5 m reject, 40-rejection stale, 15 deg refit cap) were tuned
   against a ground-truth-anchored reference and may not survive the feedback loop. Watch
   that run live rather than batching it.
