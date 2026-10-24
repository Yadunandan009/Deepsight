#!/usr/bin/env python3
"""Figure: navigation error through each mission, Sim(2) bridge.

Reads eval/figures/flights_summary.json and flights_series.npz.
Solid = the primary filter (the one that fuses SLAM and that the vehicle steers
on); dashed = the SLAM-free reference filter, same flight, same sensors. Both
are horizontal error against ground truth. Triangles on the axis mark wrong-face
closures, circles benign ones. The dotted line is the bridge's 5 m gate.
Colour is the DVL seed. Panels: (a) the three stock re-measure flights (old RISE
controller), (b) the flawed L4 control, (c) stock L0 and (d) L5 of stage 1
(RISE fix applied).

Re-run after each flight: python3 eval/fig_nav.py
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from pubstyle import use_journal, save, COL2, SERIES

use_journal()
R = json.load(open(os.path.join(HERE, "figures", "flights_summary.json")))
S = np.load(os.path.join(HERE, "figures", "flights_series.npz"))
PANELS = [("(a) stock, 3 DVL units (old RISE)", lambda r: r["arm"] == "slamref"),
          ("(b) L4: hardware on faces 0 and 1 (old RISE)", lambda r: r["arm"] == "symL4"),
          ("(c) L0 stock, stage 1", lambda r: r["arm"] == "ctlL0"),
          ("(d) L5 every face different, stage 1", lambda r: r["arm"] == "symL5")]
CSEED = {0: SERIES[0], 1: SERIES[1], 2: SERIES[2], 3: SERIES[3]}

fig, axs = plt.subplots(2, 2, figsize=(COL2, 4.4), sharex=True, sharey=True,
                        gridspec_kw=dict(hspace=0.16, wspace=0.07, left=0.085, right=0.995,
                                         bottom=0.1, top=0.985))
for ax, (title, pick) in zip(axs.flat, PANELS):
    ax.axhline(5.0, color="#999", lw=0.7, ls=(0, (1, 1.5)))
    for r in filter(pick, R):
        k = r["id"]
        g = S[f"{k}_grid"]
        c = CSEED[r["seed"]]
        ax.plot(g, np.maximum(S[f"{k}_dr"], 0.01), color=c, lw=0.8, ls=(0, (3, 1.5)), alpha=0.9)
        ax.plot(g, np.maximum(S[f"{k}_filt"], 0.01), color=c, lw=1.2,
                label="seed %d" % r["seed"])
        for cl in r["closures"]:
            if cl["cls"] == "face-aliased":
                ax.plot(cl["t"], 0.0125, marker="^", ms=6, color=c, mec="black", mew=0.5,
                        ls="none", clip_on=False, zorder=5)
            elif cl["cls"] == "benign":
                ax.plot(cl["t"], 0.0125, marker="o", ms=4.5, mfc="white", mec=c, mew=1.0,
                        ls="none", clip_on=False, zorder=5)
    ax.set_yscale("log")
    ax.set_ylim(0.01, 12)
    ax.set_xlim(0, 1400)
    ax.set_xticks([0, 200, 400, 600, 800, 1000, 1200, 1400])
    ax.text(0.015, 0.97, title, transform=ax.transAxes, va="top", fontsize=6.8)
    ax.legend(loc="upper left", fontsize=6, ncol=3, columnspacing=0.8, handlelength=1.2,
              bbox_to_anchor=(0.0, 0.89))
for ax in axs[1]:
    ax.set_xlabel("mission time (s)")
for ax in axs[:, 0]:
    ax.set_ylabel("horizontal error (m)")
axs[0, 1].text(1390, 5.6, "5 m bridge gate", ha="right", fontsize=6, color="#777")
axs[0, 0].text(0.015, 0.76, "solid: primary filter; dashed: SLAM-free reference",
               transform=axs[0, 0].transAxes, fontsize=5.8, color="#555", va="top")
save(fig, os.path.join(HERE, "figures", "fig_nav"))
print("wrote eval/figures/fig_nav.{pdf,png}")
