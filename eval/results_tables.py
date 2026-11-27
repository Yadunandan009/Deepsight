#!/usr/bin/env python3
"""Manuscript results tables for the gap-closing experiments, generated.

    G1/G2  veto and symmetry flights: mission-level Fisher tests and the
           heading residual as a wrong-face classifier (every decision)
    G5     bridge quarantine in the loop, per arm
    G3     gyro replay: dead reckoning vs fused, per heading grade, with the
           flown compass-heading flights as the reference row

Reads eval/figures/prereg_score.json (prereg_score.py), prereg_bridge_events.json
(bridge_events.py) and g3_replay.json (g3_replay.py score). Nothing is recomputed
from bags; every number traces to those three files.

Writes docs/paper/results_tables.md.   Run:  python3 eval/results_tables.py
"""
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
OUT = os.path.join(HERE, "..", "docs", "paper", "results_tables.md")
ARMS = (("stkL0", "stock jacket", "off (logged only)"),
        ("vetoL0", "stock jacket", "veto"),
        ("stkL5", "symmetry broken (L5)", "off (logged only)"))
FLAGGED = ("veto", "would_veto")


def load(name):
    return json.load(open(os.path.join(FIG, name)))


def by_arm(rows, arm):
    return [r for r in rows if r["arm"] == arm]


def g1_g2(P):
    F, T = P["flights"], P["tests"]
    p = {(t["test"], mode): t for mode in ("strict", "loose") for t in T[mode]}
    L = ["## Table A. Wrong-face loop closures per mission (G1, G2)", "",
         "| arm | scene | heading gate | missions | accepted closures before return | "
         "missions with a wrong-face closure | unscorable | one-sided Fisher p vs stkL0 | Holm-adjusted p |",
         "|---|---|---|---|---|---|---|---|---|"]
    for arm, scene, gate in ARMS:
        R = by_arm(F, arm)
        n_acc = sum(1 for r in R for d in r["decisions"]
                    if d["before_cutoff"] and d["decision"] not in ("veto",))
        strict, loose = sum(r["hit_strict"] for r in R), sum(r["hit_loose"] for r in R)
        unsc = loose - strict
        test = {"vetoL0": "Q1", "stkL5": "Q2"}.get(arm)
        if test:
            ps, pl = p[(test, "strict")], p[(test, "loose")]
            pcol = f"{ps['p']:.2f} ({pl['p']:.2f})"
            hcol = f"{ps['p_holm']:.2f} ({pl['p_holm']:.2f})"
        else:
            pcol = hcol = "reference"
        L.append(f"| {arm} | {scene} | {gate} | {len(R)} | {n_acc} | {strict}/{len(R)} | {unsc} | {pcol} | {hcol} |")
    why = [(r["seed"], e["why"]) for r in F for e in r["events"] if e["cls"] == "unscorable"]
    L += ["", "p in brackets: the unscorable closure counted as wrong-face. Unscorable: "
          + "; ".join(f"seed {s}, {w}" for s, w in why)
          + " (pose-jump test). The keyframe test of Table B classes it same-place.", ""]

    D = [d for r in F for d in r["decisions"]]
    L += ["## Table B. Heading residual as a wrong-face test, every heading decision (G1)", "",
          "| truth class | candidates | abs residual, min to max (deg) | flagged (veto or would-veto) | accepted |",
          "|---|---|---|---|---|"]
    for cls in ("wrong-face", "same-place"):
        C = [d for d in D if d["match_cls"] == cls]
        a = np.abs([d["resid"] for d in C])
        nf = sum(d["decision"] in FLAGGED for d in C)
        L.append(f"| {cls} | {len(C)} | {a.min():.1f} to {a.max():.1f} | {nf} | {len(C) - nf} |")
    other = [d for d in D if d["match_cls"] not in ("wrong-face", "same-place")]
    L += ["", f"Loops and relocalisations in all 24 missions, before and after the return leg; "
          f"{len(other)} decision(s) of other truth class. Veto threshold as registered."
          if other else "Loops and relocalisations in all 24 missions, before and after the return leg. "
          "Veto threshold as registered.", ""]
    return L


def g5(P, B):
    F = {r["bag"]: r for r in P["flights"]}
    L = ["## Table C. Bridge quarantine in the loop (G5)", "",
         "| arm | missions | with a wrong-face closure | quarantine fired | jump resets | "
         "quarantine fired without a wrong-face closure | fused median error, wrong-face missions (m) | "
         "dead reckoning, same missions (m) |",
         "|---|---|---|---|---|---|---|---|"]
    for arm, _, _ in ARMS:
        R = by_arm(B, arm)
        wf = [r for r in R if F[r["bag"]]["hit_strict"]]
        q = [r for r in R if r["before_cutoff"]["quarantine"]]
        fa = [r for r in q if not F[r["bag"]]["hit_strict"]]
        jr = sum(r["before_cutoff"]["jump_reset"] for r in R)
        fm = ", ".join(f"{F[r['bag']]['filt_median']:.2f}" for r in wf) or "n/a"
        dm = ", ".join(f"{F[r['bag']]['dr_median']:.2f}" for r in wf) or "n/a"
        L.append(f"| {arm} | {len(R)} | {len(wf)} | {len(q)} | {jr} | {len(fa)} | {fm} | {dm} |")
    L += ["", "Counts before the return leg. Error: horizontal, median over the mission, against truth.", ""]
    return L


def g3(P, G):
    F = P["flights"]
    L = ["## Table D. Does fusing SLAM beat dead reckoning? Per heading source (G3)", "",
         "| heading source | flights | gyro bias, abs, median (max) (deg/h) | dead reckoning, median error (m) | "
         "fused, median error (m) | fused better, median error | fused better, at mission end | "
         "one-sided Wilcoxon p (median / end) |",
         "|---|---|---|---|---|---|---|---|"]
    dm, fm = [r["dr_median"] for r in F], [r["filt_median"] for r in F]
    nb = sum(f < d for f, d in zip(fm, dm))
    ne = sum(r["filt_end"] < r["dr_end"] for r in F)
    L.append(f"| simulated compass (as flown) | {len(F)} | n/a | {np.median(dm):.2f} | {np.median(fm):.2f} | "
             f"{nb}/{len(F)} | {ne}/{len(F)} | not tested |")
    for g, label in (("tactical MEMS (ADIS16495-class)", "tactical MEMS gyro"),
                     ("MEMS, bias cal. in water (dT 1 C)", "MEMS gyro, calibrated in water (registered primary)"),
                     ("MEMS, bias cal. on deck (dT 10 C)", "MEMS gyro, calibrated on deck")):
        t = G["tests"][g]
        b = np.abs([r["gyro"]["bias_dph"] for r in G["flights"] if r["grade"] == g])
        L.append(f"| {label} | {t['n']} | {np.median(b):.1f} ({b.max():.1f}) | {t['median_dr']:.2f} | "
                 f"{t['median_fused']:.2f} | {t['fused_better_median']}/{t['n']} | {t['fused_better_end']}/{t['n']} | "
                 f"{t['p_median']:.2f} / {t['p_end']:.2f} |")
    L += ["", "Errors: horizontal against truth from mission start to MISSION COMPLETE; per flight the "
          "median, then the median over flights. Gyro rows are offline replays of the 24 flown missions "
          "through the same filters and bridge with the compass replaced by an integrated gyro "
          "(one gyro unit per DVL seed). Wilcoxon tests fused better than dead reckoning; only the "
          "primary grade was registered with a criterion.", ""]
    return L


def main():
    P, B, G = load("prereg_score.json"), load("prereg_bridge_events.json"), load("g3_replay.json")
    if G.get("missing"):
        raise SystemExit(f"g3_replay.json lists {len(G['missing'])} missing replays")
    L = ["# Results tables, gap-closing experiments", "",
         "Generated by `eval/results_tables.py`; do not edit by hand.", ""]
    L += g1_g2(P) + g5(P, B) + g3(P, G)
    txt = "\n".join(L)
    open(OUT, "w").write(txt)
    print(txt)


if __name__ == "__main__":
    main()
