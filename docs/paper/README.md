# Paper draft: what is written, as of 2026-10-17

| file | what it is | state |
|---|---|---|
| `DeepSight_Introduction_LitSurvey_v2.docx` | Introduction, background and related work, references, status note | Introduction rewritten; Section 2.5 still tells the old aliasing story; 3 reference placeholders; citations checked for existence only |
| `section_methods.md` | testbed, sensors, navigation architecture, mission, protocol | written; every fact traced to code or config |
| `section_closure_events.md` | wrong-face loop closures: classification, evidence, cost, rate | written; two figures made |
| `section_checks.md` | three closure checks against dead-reckoning drift | written; one figure made; rests on injected drift |
| `section_dr_results.md` | dead reckoning on a lap; fusing SLAM; the reconstructed structure | written, all three parts |
| `section_limitations.md` | what the work does not show | written |
| `abstract.md` | ~235 words | written; three claims flagged as resting on less than they seem |
| discussion, conclusion | | not written |

Supporting records: `CITATION_CHECK.md` (what was verified and what was not),
`../PAPER_REPLAN_20261017.md` (the plan and the reasons for it),
`../experiments/PREREG_*.md` (pre-registrations).

Figures, all in `eval/figures/` as .pdf and .png: `closures_classes`,
`closures_cost`, `gates_vs_drift`, `recon_planview`, `recon_by_source`.
Missing: system diagram. Scripts: `fig_closures.py`, `fig_gates.py`,
`fig_recon.py`; analysis: `closure_events.py`, `closure_revisit.py`,
`gate_sweep.py`, `sonar_recon.py`, `recon_batch.py`.

**Still to write:** discussion, conclusion, Section 2.5 rewrite, system diagram.
**Still to check:** every citation's support for its sentence (existence checked
only); ROVER must be read; the three reference placeholders.
**Not yet flown:** the planned re-measurement flights (post-fix fused-filter
results rest on two recordings) and the gyro-drift sweep.
