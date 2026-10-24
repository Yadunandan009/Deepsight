#!/usr/bin/env python3
"""Figure: where each mission ended. Reads eval/figures/flights_summary.json.

Final error of the primary filter against truth (x) and the true distance of the
vehicle from where it started (y); the vehicle was sent home by the primary
filter, so the second is the cost of the first. Filled = the flight had a
wrong-face closure, open = it did not. Dashed lines: the 5 m bridge gate and the
2 m radius inside which the controller declares the mission complete.

Re-run after each flight: python3 eval/fig_endstate.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from pubstyle import use_journal, save, COL1, COL2, SERIES

use_journal()
R = json.load(open(os.path.join(HERE, "figures", "flights_summary.json")))
NAME = {"slamref": "stock re-measure", "symL4": "L4", "ctlL0": "L0 stock", "symL5": "L5",
        "oldbridge": "stock, earlier bridge"}
ORDER = ["slamref", "symL4", "ctlL0", "symL5", "oldbridge"]
rows = sorted([r for r in R if r["arm"] in NAME], key=lambda r: (ORDER.index(r["arm"]), r["seed"]))
lab = ["%s, seed %d" % (NAME[r["arm"]], r["seed"]) for r in rows]
c = [SERIES[1] if r["n_aliased"] > 0 else "#9aa7b4" for r in rows]
y = list(range(len(rows)))[::-1]
fig, (a1, a2) = plt.subplots(1, 2, figsize=(COL2 * 0.78, 3.0), sharey=True)
a1.barh(y, [r["filt_final"] for r in rows], color=c, height=0.7)
a1.axvline(5.0, color="#555", lw=0.7, ls=(0, (3, 2)))
a1.text(4.95, len(rows) - 0.35, "5 m gate", ha="right", va="center", fontsize=6, color="#555")
a1.set_xlabel("final error of the fused filter (m)")
a1.set_yticks(y); a1.set_yticklabels(lab, fontsize=6.5)
a1.set_xlim(0, 6)
a2.barh(y, [r["final_dist_from_start"] for r in rows], color=c, height=0.7)
a2.axvline(2.0, color="#555", lw=0.7, ls=(0, (3, 2)))
a2.text(2.1, len(rows) - 0.35, "2 m: mission complete", va="center", fontsize=6, color="#555")
a2.set_xlabel("true final distance from home (m)")
a2.set_xlim(0, 8)
for a in (a1, a2):
    a.grid(True, axis="x"); a.grid(False, axis="y"); a.tick_params(axis="y", length=0)
from matplotlib.patches import Patch
fig.legend(handles=[Patch(color=SERIES[1], label="had a wrong-face closure"),
                    Patch(color="#9aa7b4", label="no wrong-face closure")],
           loc="lower center", ncol=2, fontsize=6.5, frameon=False, bbox_to_anchor=(0.6, -0.01))
for a in (a1, a2):
    a.set_ylim(-0.6, len(rows) + 0.1)
a1.text(-0.02, 1.02, "(a)", transform=a1.transAxes, fontsize=8)
a2.text(-0.02, 1.02, "(b)", transform=a2.transAxes, fontsize=8)
fig.tight_layout(rect=(0, 0.06, 1, 1))
save(fig, os.path.join(HERE, "figures", "fig_endstate"))
print("wrote eval/figures/fig_endstate.{pdf,png}")
