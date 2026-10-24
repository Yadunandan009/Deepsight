# Paper draft: what is written, as of 2026-10-22

**Start here:** `MANUSCRIPT_DRAFT_20261022.md` is the single consolidated draft (abstract to conclusion,
appendices, figure captions, update checklist). `CLAIMS_AND_EVIDENCE.md` is the framing, the claim ledger
and an honest assessment. `FIGURES.md` indexes every figure and how to regenerate it.
`live_numbers.md` is generated from the flights by `eval/flights_summary.py`. After each flight:
add a row to `../experiments/flights_manifest.csv`, then `bash docs/paper/make_paper_assets.sh`.

**The older section files below are superseded where they disagree with the manuscript draft.** They still hold
the full evidence text for the earlier recordings and are cited from it. In particular `abstract.md`,
`section_discussion.md`, `section_conclusion.md` and `section_limitations.md` were written around "fusing SLAM
hurts", which was a property of bridge version A and does not hold with version B (see the manuscript, 4.4).


| file | what it is | state |
|---|---|---|
| `DeepSight_Introduction_LitSurvey_v3.docx` | Introduction, background and related work, references, status note | Introduction rewritten; Section 2.5 rewritten 2026-10-19 around prior art (refs [36]-[42] added); 3 reference placeholders; citation support checked in pass 2 (see CITATION_CHECK.md), not by reading every paper |
| `section_methods.md` | testbed, sensors, navigation architecture, mission, protocol | written; every fact traced to code or config |
| `section_closure_events.md` | wrong-face loop closures: classification, evidence, cost, rate | written; two figures made |
| `section_checks.md` | three closure checks against dead-reckoning drift | written; one figure made; rests on injected drift |
| `section_dr_results.md` | dead reckoning on a lap; fusing SLAM; the reconstructed structure | written, all three parts |
| `section_limitations.md` | what the work does not show | written |
| `abstract.md` | ~235 words | written; three claims flagged as resting on less than they seem |
| `section_discussion.md`, `section_conclusion.md` | what was found, mechanism, checks, inspection product, next steps; conclusion | written 2026-10-19; numbers that depend on the re-measure flights are listed in the status note |

Supporting records: `CITATION_CHECK.md` (what was verified and what was not),
`../PAPER_REPLAN_20261017.md` (the plan and the reasons for it),
`../experiments/PREREG_*.md` (pre-registrations).

Figures, all in `eval/figures/` as .pdf and .png: `closures_classes`,
`closures_cost`, `gates_vs_drift`, `recon_3d`, `recon_explain`, `recon_by_source`, `slam_map_3d` (ORB-SLAM3 map vs CAD; `eval/slam_map_recon.py`, results `eval/figures/slam_map_results.json`). The 3D figure needs
`/usr/bin/python3` (the venv's matplotlib cannot load mplot3d).
Missing: system diagram. Scripts: `fig_closures.py`, `fig_gates.py`,
`fig_recon.py`; analysis: `closure_events.py`, `closure_revisit.py`,
`gate_sweep.py`, `sonar_recon.py`, `recon_batch.py`.

**Still to write:** system diagram.
**Still to check:** the three reference placeholders ([1], [2], [5]); the sentence citing [1], [2] has no verified source; most citation judgements are from knowledge of the papers, not re-read; ROVER's method section was read, not the whole paper.
**Not yet flown:** the planned re-measurement flights (post-fix fused-filter
results rest on two recordings) and the gyro-drift sweep.
