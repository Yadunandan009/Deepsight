#!/usr/bin/env python3
"""Figure: the primary filter's error against the bridge's 5 m gate.

Reads eval/figures/gate_plateau_series.npz (gate_plateau.py). For each flight
that ends with a large fused error: the primary filter's horizontal error against
ground truth (solid), the dead-reckoned filter's error (dashed), and, for the
poses the bridge actually published, their distance from the dead-reckoned filter
(dots), which is the quantity the gate tests. The dotted horizontal line is the
gate. If the gate sets the ceiling, the dots stay under it while the solid line
rises to just below it and stays.

Re-run after each flight: python3 eval/gate_plateau.py && python3 eval/fig_gate_plateau.py
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
S = np.load(os.path.join(HERE, "figures", "gate_plateau_series.npz"))
R = json.load(open(os.path.join(HERE, "figures", "gate_plateau.json")))
SUM = {r["id"]: r for r in json.load(open(os.path.join(HERE, "figures", "flights_summary.json")))}
SHOW = [k for k in ("L4_s0", "L4_s1", "L0_s0", "L5_s0", "L5_s1", "L0_s1") if k in R]
n = len(SHOW)
fig, axs = plt.subplots(2, (n + 1) // 2, figsize=(COL2, 3.9), sharex=True, sharey=True,
                        gridspec_kw=dict(hspace=0.12, wspace=0.06, left=0.075, right=0.995,
                                         bottom=0.2, top=0.985))
axs = np.atleast_2d(axs).flatten()
for ax, k in zip(axs, SHOW):
    ax.axhline(5.0, color="#999", lw=0.7, ls=(0, (1, 1.5)))
    ax.plot(S[k + "_tb"], S[k + "_bdr"], ".", ms=1.0, color=SERIES[3], alpha=0.5, rasterized=True,
            label="published SLAM pose, distance from dead reckoning (the gate's test)")
    ax.plot(S[k + "_g"], np.maximum(S[k + "_ed"], 0.01), color="#444", lw=0.8, ls=(0, (3, 1.5)),
            label="dead-reckoned filter error")
    ax.plot(S[k + "_g"], np.maximum(S[k + "_ef"], 0.01), color=SERIES[1], lw=1.3,
            label="primary filter error")
    for cl in SUM[k]["closures"]:
        if cl["cls"] == "face-aliased":
            ax.plot(cl["t"], 0.0125, marker="^", ms=6, color=SERIES[1], mec="black", mew=0.5,
                    ls="none", clip_on=False, zorder=5)
    ax.set_yscale("log")
    ax.set_ylim(0.01, 40)
    ax.set_xlim(0, 1400)
    ax.set_xticks([0, 400, 800, 1200])
    ax.text(0.02, 0.97, k.replace("_s", " seed "), transform=ax.transAxes, va="top", fontsize=7)
for ax in axs[n:]:
    ax.axis("off")
for ax in axs[-(len(axs) // 2):]:
    if ax.axison:
        ax.set_xlabel("mission time (s)")
for ax in axs[::(len(axs) // 2)]:
    ax.set_ylabel("horizontal error / distance (m)")
h, l = axs[0].get_legend_handles_labels()
fig.legend(h, l, loc="lower center", ncol=3, fontsize=6, bbox_to_anchor=(0.5, 0.0), frameon=False,
           markerscale=6, columnspacing=1.2, handlelength=1.8)
save(fig, os.path.join(HERE, "figures", "fig_gate_plateau"))
print("wrote eval/figures/fig_gate_plateau.{pdf,png}")
