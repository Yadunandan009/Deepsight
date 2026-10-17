#!/usr/bin/env python3
"""Figure: how three loop-closure checks hold up as dead reckoning drifts.

Reads eval/figures/gate_results.json (written by gate_sweep.py). The drift is
INJECTED into the true track under the model stated in gate_sweep.py; it is a
sensitivity analysis, not a flown condition, and the caption must say so.

(a) wrong-face closures caught, per cent of 5.
(b) benign closures wrongly flagged, per cent of 6.
Log x-axis: the interesting range spans two decades. The vertical line marks
where a single lap sits (dead-reckoning error of 0.1 to 0.5 m measured over
three DVL units).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from pubstyle import use_journal, save, COL2, SERIES, MARKERS

use_journal()
R = json.load(open(os.path.join(HERE, "figures", "gate_results.json")))
sw = [r for r in R["sweep"] if r["X"] >= 0.3]
x = [r["X"] for r in sw]
STY = {"size": ("size of the correction", SERIES[0], "o", (0, (4, 2))),
       "revisit": ("revisit (dead-reckoned track)", SERIES[2], "^", (0, (1, 1.2))),
       "chord": ("chord length (no dead reckoning)", SERIES[1], "s", "-")}

fig, axs = plt.subplots(1, 2, figsize=(COL2, 2.5), sharex=True,
                        gridspec_kw=dict(wspace=0.22, left=0.075, right=0.995,
                                         bottom=0.2, top=0.96))
for ax, key, yl in ((axs[0], "tpr", "wrong-face closures caught (%)"),
                    (axs[1], "fpr", "benign closures wrongly flagged (%)")):
    for g, (lab, col, mk, ls) in STY.items():
        ax.plot(x, [100 * r[g][key] for r in sw], color=col, marker=mk, ms=3.6,
                ls=ls, lw=1.0, mec="white", mew=0.3, label=lab,
                zorder=3 if g == "chord" else 2)
    ax.axvspan(0.1, 0.5, color="#cfcfcf", lw=0, zorder=0)
    ax.set_xscale("log")
    ax.set_xlim(0.25, 50)
    ax.set_ylim(-6, 106)
    ax.set_xticks([0.3, 1, 3, 10, 30])
    ax.set_xticklabels(["0.3", "1", "3", "10", "30"])
    ax.set_xlabel("dead-reckoning drift at the end of the mission (m)")
    ax.set_ylabel(yl)
axs[0].text(0.31, 50, "one lap\n(0.1–0.5 m)", fontsize=6.5, ha="left", va="center",
            rotation=90)
axs[0].axvspan(7, 18, color="#e4e4e4", lw=0, zorder=0)
axs[0].text(30, 50, "7–18 m:\nplain checks\nfail, chord\ncheck holds", fontsize=6.5,
            ha="center", va="center")
axs[1].axvspan(22, 50, color="#e4e4e4", lw=0, zorder=0)
axs[1].text(5.5, 55, "from about 22 m,\nhonest corrections\nare as large as\nthe jumps", fontsize=6.5,
            ha="center", va="center")
axs[0].legend(loc="lower left", bbox_to_anchor=(0.115, 0.04), frameon=False,
              handlelength=2.0, labelspacing=0.4, fontsize=6.5)
for ax, s in ((axs[0], "(a)"), (axs[1], "(b)")):
    ax.text(0.03 if ax is axs[1] else 0.97, 0.95 if ax is axs[1] else 0.86, s,
            transform=ax.transAxes, ha="left" if ax is axs[1] else "right",
            va="top", fontsize=8.5, fontweight="bold")
for f in save(fig, os.path.join(HERE, "figures", "gates_vs_drift")):
    print("wrote", os.path.relpath(f, os.path.join(HERE, "..")))
