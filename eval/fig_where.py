#!/usr/bin/env python3
"""Figure: where in the mission and at what depth each classified closure falls.

Reads eval/figures/closure_where.json (made by eval/closure_where.py). Depth is the
vehicle's TRUE depth at the closure; time is seconds from the first odometry sample.
Filled = wrong-face closure, open = benign. The dashed line marks 20 m, a threshold
chosen after looking at the data (see docs/paper/WHERE_IN_THE_MISSION.md).

Re-run: python3 eval/fig_where.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from pubstyle import use_journal, save, COL1, SERIES

use_journal()
import numpy as np
D = json.load(open(os.path.join(HERE, "figures", "closure_where.json")))
rng = np.random.default_rng(3)
cols = {"face-aliased": SERIES[1], "benign": SERIES[0]}
xs = {"face-aliased": 0, "benign": 1}
fig, ax = plt.subplots(figsize=(COL1, 2.9))
ax.axhspan(1, 22, color="#f3f3f0", zorder=0)
ax.axhline(20.0, color="#777", lw=0.7, ls=(0, (3, 2)), zorder=1)
ax.text(1.42, 19.7, "20 m", fontsize=6, color="#666", ha="right", va="bottom")
for cls in ("face-aliased", "benign"):
    z, lab = [], []
    for name, v in D.items():
        for e in v["events"]:
            if e["cls"] == cls:
                z.append(e["z"]); lab.append({"symL5": "L5", "symL4": "L4"}.get(v["arm"], ""))
    z = np.array(z); x = xs[cls] + rng.uniform(-0.13, 0.13, len(z))
    ax.scatter(x, z, s=22, color=cols[cls], edgecolors="white", linewidths=0.5, zorder=3)
    ax.plot([xs[cls] - 0.25, xs[cls] + 0.25], [np.median(z)] * 2, color="k", lw=1.2, zorder=4)
    ax.text(xs[cls] - 0.27, np.median(z), "median\n%.1f m" % np.median(z), fontsize=5.8, va="center", ha="right")
    print(cls, len(z), np.median(z).round(1))
ax.set_xticks([0, 1]); ax.set_xticklabels(["wrong-face (n = 12)", "benign (n = 11)"])
ax.set_xlim(-0.6, 1.45)
ax.set_ylim(23, 0)
ax.set_ylabel("true depth at the closure (m)")
ax.grid(False)
save(fig, os.path.join(HERE, "figures", "fig_where"))
print("wrote eval/figures/fig_where.{pdf,png}")
