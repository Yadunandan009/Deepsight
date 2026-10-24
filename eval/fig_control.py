#!/usr/bin/env python3
"""Figure: the symmetry control. Reads eval/figures/flights_summary.json.

(a) One row per planned stage-1 flight (stock L0 above, every-face-different L5
    below, seeds 0 to 2). A marker is a loop closure at the mission time it
    happened: filled triangle = wrong-face (face-aliased), open circle = benign,
    cross = unscorable, square = large but not a symmetry operation. The bar ends
    where the recording ends. Rows not yet flown are drawn empty and say so.
(b) Every classified closure in the manifest: size of the correction against the
    turn it made about the structure's axis, by arm. Shaded: within 15 degrees of
    90 and of 180. Dashed: 3 m, the benign threshold. Grey dots are the twelve
    closures of the earlier recordings (closures_data.json).

Re-run after each flight: python3 eval/fig_control.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from pubstyle import use_journal, save, COL2, SERIES

use_journal()
R = json.load(open(os.path.join(HERE, "figures", "flights_summary.json")))
H = json.load(open(os.path.join(HERE, "figures", "closures_data.json")))["events"]
by = {(r["arm"], r["seed"]): r for r in R}

ROWS = [("ctlL0", s) for s in (0, 1, 2)] + [("symL5", s) for s in (0, 1, 2)]
LAB = {"ctlL0": "L0 stock", "symL5": "L5 every face different"}
COL = {"ctlL0": SERIES[1], "symL5": SERIES[0], "symL4": SERIES[2], "slamref": SERIES[3]}
MK = {"face-aliased": ("^", True), "benign": ("o", False), "unscorable": ("x", False),
      "other-large": ("s", True)}

fig = plt.figure(figsize=(COL2, 3.0))
gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1.0], wspace=0.33, left=0.17, right=0.995,
                      bottom=0.17, top=0.97)
ax = fig.add_subplot(gs[0])
ax.grid(False)
for i, (arm, sd) in enumerate(ROWS):
    y = len(ROWS) - 1 - i - (0.5 if arm == "symL5" else 0)
    r = by.get((arm, sd))
    c = COL[arm]
    if r is None:
        ax.plot([0, 1250], [y, y], color="#bbb", lw=0.8, ls=(0, (2, 2)))
        ax.text(640, y + 0.18, "not yet flown", fontsize=6, color="#888", ha="center")
    else:
        ax.plot([0, r["duration_s"]], [y, y], color="#aaa", lw=1.0, solid_capstyle="butt", zorder=1)
        for cl in r["closures"]:
            m, filled = MK[cl["cls"]]
            ax.plot(cl["t"], y, marker=m, ms=7 if cl["cls"] == "face-aliased" else 5.5,
                    mfc=c if filled else "white", mec=c, mew=1.0, ls="none", zorder=3)
            if cl["cls"] == "face-aliased":
                ax.text(cl["t"], y + 0.22, "%+.0f°" % cl["turn"], fontsize=6, ha="center")
    ax.text(-25, y, "seed %d" % sd, ha="right", va="center", fontsize=6.5)
ax.set_yticks([])
ax.set_ylim(-1.0, 6.3)
ax.set_xlim(-10, 1400)
ax.set_xlabel("mission time (s)")
for arm, y in (("ctlL0", 5.65), ("symL5", 2.65)):
    ax.text(-390, y - 1.1, LAB[arm], rotation=90, va="center", ha="center", fontsize=7,
            color=COL[arm], fontweight="bold")
ax.axhline(2.95, color="#ddd", lw=0.6)
ax.spines["left"].set_visible(False)
ax.text(0.01, 0.99, "(a)", transform=ax.transAxes, va="top", fontsize=8)
# legend
from matplotlib.lines import Line2D
leg = [Line2D([], [], marker="^", ls="none", mfc="#555", mec="#555", ms=6, label="wrong-face"),
       Line2D([], [], marker="o", ls="none", mfc="white", mec="#555", ms=5, label="benign"),
       Line2D([], [], marker="s", ls="none", mfc="#555", mec="#555", ms=5, label="large, not a symmetry op"),
       Line2D([], [], marker="x", ls="none", mec="#555", ms=5, label="unscorable")]
ax.legend(handles=leg, loc="lower right", fontsize=5.8, ncol=2, columnspacing=0.8, handletextpad=0.2,
          bbox_to_anchor=(1.0, 0.02))

bx = fig.add_subplot(gs[1])
for a0, a1 in ((75, 105), (165, 195)):
    bx.axhspan(a0, a1, color="#e6e6e6", lw=0, zorder=0)
    bx.axhspan(-a1, -a0, color="#e6e6e6", lw=0, zorder=0)
bx.axvline(3.0, color="#888", lw=0.7, ls=(0, (3, 2)))
bx.plot([e["chord"] for e in H], [e["turn"] for e in H], "o", ms=3.2, mfc="#c8c8c8", mec="#888",
        mew=0.4, ls="none", label="earlier recordings", zorder=2)
SHOW = (("slamref", "re-measure, stock"), ("symL4", "L4 (flawed control)"),
        ("ctlL0", "L0 stock"), ("symL5", "L5"))
for arm, lab in SHOW:
    xs, ys = [], []
    for r in R:
        if r["arm"] != arm:
            continue
        for cl in r["closures"]:
            if "chord" in cl:
                xs.append(cl["chord"])
                ys.append(cl["turn"])
    bx.plot(xs, ys, marker={"slamref": "D", "symL4": "s", "ctlL0": "^", "symL5": "o"}[arm],
            ms=4.5, ls="none", mfc=COL[arm] if arm != "symL5" else "white", mec=COL[arm],
            mew=1.0, label=lab, zorder=3)
bx.set_xscale("log")
bx.set_xlim(0.4, 60)
bx.set_ylim(-195, 195)
bx.set_yticks([-180, -90, 0, 90, 180])
bx.set_xlabel("size of the correction (m)")
bx.set_ylabel("turn about the structure axis (deg)")
bx.legend(loc="lower left", bbox_to_anchor=(0.0, 0.0), fontsize=5.8, handletextpad=0.2,
          borderaxespad=0.1, labelspacing=0.25)
bx.text(0.02, 0.99, "(b)", transform=bx.transAxes, va="top", fontsize=8)
save(fig, os.path.join(HERE, "figures", "fig_control"))
print("wrote eval/figures/fig_control.{pdf,png}")
