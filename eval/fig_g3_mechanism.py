#!/usr/bin/env python3
"""Figure: fusing SLAM does not beat dead reckoning, whatever the heading source (G3).

(a) per flight, fused median horizontal error against dead-reckoning median error,
    log-log, one colour per heading source, identity line. Points on the line:
    the fused estimate carries the dead-reckoning error, because the bridge refits
    SLAM to dead reckoning every 5 s. Below the line: fusing helped.
(b) the ratio fused / dead reckoning against dead-reckoning error. Fusing only
    drifts below 1 once dead reckoning is already metres off, and not reliably.

Compass points are the flown missions (prereg_score.json); gyro points are the
offline replays of the same missions (g3_replay.json).

Run: python3 eval/g3_replay.py score && python3 eval/fig_g3_mechanism.py
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from pubstyle import use_journal, save, COL2, SERIES, MARKERS, INK_SECONDARY

use_journal()
P = json.load(open(os.path.join(HERE, "figures", "prereg_score.json")))["flights"]
G = json.load(open(os.path.join(HERE, "figures", "g3_replay.json")))["flights"]
SRC = [("simulated compass (flown)", [(r["dr_median"], r["filt_median"]) for r in P])]
for g, lab in (("tactical MEMS (ADIS16495-class)", "tactical MEMS gyro"),
               ("MEMS, bias cal. in water (dT 1 C)", "MEMS gyro, calibrated in water"),
               ("MEMS, bias cal. on deck (dT 10 C)", "MEMS gyro, calibrated on deck")):
    SRC.append((lab, [(r["dr_median"], r["fused_median"]) for r in G if r["grade"] == g]))

fig, (a, b) = plt.subplots(1, 2, figsize=(COL2, 2.6))
lim = (0.03, 100)
a.plot(lim, lim, color=INK_SECONDARY, lw=0.7, ls="--", zorder=0)
a.text(80, 0.045, "below the line: fusing helped", ha="right", fontsize=6, color=INK_SECONDARY)
b.axhline(1, color=INK_SECONDARY, lw=0.7, ls="--", zorder=0)
for i, (lab, pts) in enumerate(SRC):
    d, f = np.array(pts).T
    kw = dict(color=SERIES[i], marker=MARKERS[i], s=12, alpha=0.8, lw=0, label=lab)
    a.scatter(d, f, **kw)
    b.scatter(d, f / d, **kw)
for ax in (a, b):
    ax.set_xscale("log")
    ax.set_xlim(lim)
    ax.set_xlabel("dead reckoning, median error (m)")
a.set_yscale("log")
a.set_ylim(lim)
a.set_ylabel("fused, median error (m)")
b.set_yscale("log")
b.set_ylim(0.5, 100)
b.set_ylabel("fused / dead reckoning")
a.set_title("(a)", loc="left", fontsize=8)
b.set_title("(b)", loc="left", fontsize=8)
a.legend(loc="lower center", bbox_to_anchor=(1.1, 1.06), ncol=4, fontsize=6, frameon=False,
         columnspacing=0.8, handletextpad=0.2)
fig.subplots_adjust(wspace=0.32)
save(fig, os.path.join(HERE, "figures", "fig_g3_mechanism"))
print("wrote eval/figures/fig_g3_mechanism.{pdf,png}")
