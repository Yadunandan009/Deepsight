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
| v7 baseline controller | Delivered 2026-10-01, **forked from a stale base — needs rework** | `~/Desktop/bluerov2_autonomous_controller_v7baseline.py`; see WS-B1 |
| `eval/bearing_stability_metrics.py` | Delivered, **metric bug found and fixed**, now in repo | `eval/bearing_stability_metrics.py`; see WS-B2 |
| MPC / Fossen / IMM / Mahalanobis prototypes | Exist on Desktop, stale, never executed | forked pre-TF-fix; see §6 |

**Both Phase 1 files arrived on 2026-10-01**, after this plan's first draft. Neither is ready to run as delivered — one has a metric-validity bug (fixed, §WS-B2), the other was forked from a pre-2026-09-22 base and would confound the comparison (§WS-B1). WS-B is unblocked but is still the critical path.

---

## 1. The contribution — what this paper actually argues

The protocol proposes v7-vs-v8 bearing stability as "the paper's hook." **I'd recommend against making that the primary claim**, for one reason: the v7 baseline is a reconstruction of code that no longer exists, so the headline result would be "our method beats a baseline we rebuilt ourselves." That is a straw-man vulnerability a reviewer will go straight for, and it's load-bearing if it's the hook.

The project already holds two *independently verified* results that don't have that weakness, both derived from sonar geometry that owes nothing to the controller being evaluated:

- **Structural self-similarity (Task #15)** — the turbine's four faces register against each other at fitness 0.996-1.000, RMSE 0.08-0.10 m, barely above the self-match floor. This is a measured property of the inspection target, established without reference to ORB-SLAM3's output at all. It explains *why* appearance-based place recognition is structurally unreliable on jacket geometry — not as an opinion, as a number.
- **False loop closure, geometrically verified (Task A)** — ICP against an independent sonar reference shows face 2's map points have no preference for their own true geometry over face 1's.

**Recommended framing:** *lattice/jacket geometry breaks the assumptions that both appearance-based SLAM and sonar-centroid bearing extraction rely on; here is the measured evidence for why, and a geometry-anchored control stack whose design decisions are each individually quantified.*

That gives four results legs instead of one, and the two strongest legs don't depend on a reconstructed baseline:

| # | Result | Source | Strength |
|---|---|---|---|
| R1 | Four faces are geometrically near-indistinguishable → vision-only place recognition is structurally at risk here | Task #15, done | **Strongest** — independent reference geometry, nothing to straw-man |
| R2 | A suspected false loop closure, confirmed geometrically | Task A, done | Strong — same independence property |
| R3 | Each v8 design decision measurably contributes (ablation battery) | WS-C, to run | Strong — self-contained, no external baseline needed |
| R4 | Pure-geometry bearing is more stable than sonar-centroid bearing on a lattice | WS-B, to build+run | Useful, but disclose the reconstruction |

R4 moves from "the hook" to "one result among four," which makes the reconstruction caveat survivable rather than fatal.

---

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

### WS-A — Close the allocation-matrix ablation (#9 → #10)
**Status:** half done. #8 complete; #9 never ran. **Effort:** ~1 evening unattended + 1h analysis.
**Why it's first:** it is the smallest complete loop through the entire experimental pipeline (run → extract → compare → report). Proving that loop end-to-end de-risks WS-C, which is the same machinery at 4× the scale.

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

### WS-B — Repair Phase 1 infrastructure, then run it
**Status:** files delivered, both need work before use. **Effort:** ~1 day rework + ~6h unattended runs.

#### B1 — The v7 baseline is forked from a stale base (must fix before any run)

The delivered file is well-constructed in design: it is explicit that it is a reconstruction, documents the inverse-range weighting choice, isolates bearing extraction as the single experimental variable, and keeps v8's low-pass filtering style. **But it was forked from a pre-2026-09-22 controller** and is missing every fix made since:

| Fix | occurrences in v7baseline | in current controller |
|---|---|---|
| `max_yaw` authority caps | **0** | **13** |
| `YAW_SEED_SETTLE` | 0 | 5 |
| `RISE_YAW_MAX` | 0 | 3 |
| `RISE_RETREAT_SPEED` | 0 | 4 |

The `max_yaw` row is disqualifying on its own: the current controller caps yaw authority at 0.15 in 13 places across DESCEND / CLOSE_IN / SCAN / TRANSIT — **precisely the four phases `bearing_stability_metrics.py` measures**. Running as-is would compare *bearing method + yaw caps + seed-settle + RISE tuning* bundled together, and a reviewer could reasonably attribute any difference to the yaw caps rather than to bearing extraction, which is the actual claim.

**Recommended fix — make bearing method an ablation switch instead of a separate fork:**

```
DEEPSIGHT_BEARING=geometry   (default, current behaviour)
DEEPSIGHT_BEARING=centroid   (ports _sonar_centroid_bearing from the v7baseline file)
```

read at class-definition time, exactly like the existing `BLUEROV2_BUGGY_ALLOC` flag. This is strictly better than maintaining a second file:
- **Zero confound by construction** — one code path, one set of fixes, only the bearing source differs
- Matches the generalised ablation-switch mechanism WS-C needs anyway
- Committable and reproducible; the configuration of every trial is recoverable from its log
- Two forks cannot silently drift apart over the months this paper takes

The alternative — re-forking the v7 baseline from the *current* controller — also works and keeps the arms visibly separate, at the cost of a file that must be manually kept in sync. **The env-switch is the better option**; the reconstruction disclosure (below) is unaffected either way.

Regardless of mechanism, the reconstruction must be disclosed verbatim in the paper's methods: this is a reconstruction from v8's own documented description of v7, with a stated inverse-range weighting choice, not recovered original code.

#### B2 — Metric bug in `bearing_stability_metrics.py` (found and fixed 2026-10-01)

`sign_reversals_per_min` is the protocol's "single most legible number for a reviewer." As delivered it was measuring something else, and the error ran **against** the method the paper argues for.

Measured on a real v8 log (2147 bearing-phase samples, 1073 s):

| counting method | reversals | per min |
|---|---|---|
| as delivered (`np.sign` comparison) | 60 | **3.36** |
| exact zeros carried forward | 13 | 0.73 |
| + 1.0° deadband (true side-swaps) | 1 | **0.06** |

Two compounding causes, both of which inflate the count *more* the better the controller holds bearing:
1. `np.sign(0.0)` is `0`, distinct from both `+1` and `-1`, so every entry into and exit from an exact zero scored as two reversals. That log had 78 exact zeros; 47 of the 60 counted "reversals" came from them.
2. `world_bearing_deg` is logged at 0.1° resolution, and a well-tuned controller parks it within a few tenths of zero (`bearing_abs_mean_deg` = 0.45, `std` = 0.42) — so quantisation jitter alone crosses the axis constantly.

Left unfixed, v8 would have been reported as believing the turbine swapped sides every ~18 s while actually holding bearing to under half a degree. v7's swings are large and mostly real, so the bug would have compressed the measured gap between the arms and handed a reviewer an easy line of attack.

**Fixed** (`eval/bearing_stability_metrics.py`, now in repo): last nonzero sign is carried forward, and a stated `deadband_deg` parameter (default 1.0°) requires the estimate to actually commit to a side. Both the deadbanded and non-deadbanded counts are reported so the parameter's effect is visible rather than buried. The deadband is a **stated analysis parameter — do not sweep it until the comparison looks good.**

#### B3 — Run it
N≥10 per arm, through `run_batch.py` extended with a controller/bearing-mode argument so both arms get byte-identical orchestration. Metrics via the corrected script, aggregated per §3.

**Exit criteria:** bearing mode switchable within one controller; ≥10 runs per arm; metrics aggregated with §3 statistics; reconstruction disclosure drafted.

---

### WS-C — Ablation battery on v8 itself
**Status:** not started; depends on WS-A proving the loop. **Effort:** ~20h unattended sim + ~1 day analysis.

**Methodological upgrade over the protocol:** the protocol says to disable each fix "in a throwaway local edit — never commit." Don't. We already built the better pattern during #9 — an environment-variable ablation switch (`BLUEROV2_BUGGY_ALLOC`). Generalise it:

```
DEEPSIGHT_ABLATE=yaw_guard,crab_throttle,...
```

read once at class-definition time, each flag selecting the degraded code path. This is reproducible, reviewable, committable, and means the exact configuration of every reported trial is recoverable from its log — which is precisely what a journal's code-availability statement needs. Throwaway uncommitted edits are unreproducible by construction.

| Ablation | Degraded path | Primary metric expected to worsen |
|---|---|---|
| `yaw_guard` | `_filter_yaw()` sets `self.yaw_ekf = raw_yaw` unconditionally | `max_abs_yawrate_overall_degs`, outcome rate |
| `crab_throttle` | `crab_factor = 1.0` always in `_do_close_in()` | `max_abs_sway_mps`, `orbit_tracking_err_rms_m` |
| `antiwindup` | `vel_ctrl()` integrator reverts to plain clamp | tracking error during TRANSIT |
| `alloc_matrix` | already implemented (`BLUEROV2_BUGGY_ALLOC`) | outcome rate — **done in WS-A** |

Baseline arm is the existing #8 data, **extended from 7 to ≥12 trials** so every comparison has matched N.

**Exit criteria:** ≥12 trials per arm across 4 arms + baseline; per-ablation table with the statistics from §3; each row convertible into the sentence form *"disabling X increases Y from A±B to C±D across N trials (effect size, CI)."*

---

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
WS-A (1 evening)  ──┬──> WS-C (20h sim, ~1 week wall-clock)  ──┐
                    │                                          ├──> WS-F ──> write-up
WS-B (1-2d build) ──┴──> WS-B runs (6h sim)  ─────────────────┤
WS-D (1 day)  ────────────────────────────────────────────────┤
WS-E (1-2 days, fully parallel) ──────────────────────────────┘
```

**Critical path is WS-B's implementation** — it's the only workstream requiring substantial new code, and nothing in Phase 1 can start until it exists. Start it as soon as WS-A's batch is running unattended.

**WS-D and WS-E are fully parallel** — they need no sim time at all and can absorb any evening where the machine is busy running trials.

Suggested order:
1. Kick off WS-A's batch tonight (unattended)
2. Begin WS-B implementation while it runs
3. WS-E verification in parallel (no machine contention)
4. WS-C once WS-A proves the loop end-to-end
5. WS-D + WS-F during WS-C's long unattended runs

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

1. **Tonight, unattended:** `python3 eval/run_batch.py --n 10 --prefix buggy --buggy-alloc --timeout 180 --log eval/results.jsonl`
2. **Decide:** `DEEPSIGHT_BEARING` env-switch vs. re-forking the v7 baseline (§WS-B1) — this gates all of Phase 1
3. **Decide:** Git LFS vs `.gitignore` for `slam_atlas/deepsight_map.osa` (blocks the next push either way)
4. **Parallel, any time:** move the literature base into `docs/literature/` and start DOI verification

### Already done in this session
- `eval/bearing_stability_metrics.py` moved into the repo with the side-swap counting bug fixed and verified (§WS-B2)
- Stale-fork confound in the v7 baseline identified before any runs were wasted on it (§WS-B1)
