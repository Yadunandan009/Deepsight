#!/usr/bin/env python3
"""Figure: how much of ORB-SLAM3's map lies on the jacket, stock against L5.

Distance of every sampled map point to the bare-jacket CAD, last map before the
first closure (eval/slam_map_recon.py, --out slam_map_contam_*.npz). The CAD does
not contain the L5 hardware, so a point on a riser, anode or pipe counts as off the
jacket; points on hardware lying against a leg are close to the jacket and escape
the count, so the L5 numbers are a lower bound on contamination.

(a) map placed with a similarity fitted to the true trajectory before the closure
    (uses no CAD). (b) map placed as well as it can be against the CAD (a best
    case that uses the CAD). One flight per arm, different DVL seeds.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from pubstyle import use_journal, save, COL2, SERIES

use_journal()
A = {"stock (L0, seed 1)": np.load(os.path.join(HERE, "figures", "slam_map_contam_L0s1.npz")),
     "L5 (seed 0)": np.load(os.path.join(HERE, "figures", "slam_map_contam_L5s0.npz"))}
STY = {"stock (L0, seed 1)": (SERIES[1], "-"), "L5 (seed 0)": (SERIES[0], "--")}
fig, axs = plt.subplots(1, 2, figsize=(COL2, 2.5), sharey=True,
                        gridspec_kw=dict(wspace=0.08, left=0.075, right=0.995, bottom=0.17, top=0.97))
x = np.linspace(0, 3, 400)
for ax, key, ttl in ((axs[0], "before_dtraj_sample", "(a) placed by the true trajectory"),
                     (axs[1], "before_dbest_sample", "(b) placed best against the CAD")):
    for lab, z in A.items():
        d = np.sort(z[key])
        c, ls = STY[lab]
        ax.plot(x, np.searchsorted(d, x) / len(d) * 100, color=c, ls=ls, lw=1.3, label=lab)
    ax.axvline(0.25, color="#bbb", lw=0.6)
    ax.axvline(0.5, color="#bbb", lw=0.6)
    ax.set_xlim(0, 3)
    ax.set_xlabel("distance of a map point to the jacket CAD (m)")
    ax.text(0.02, 0.97, ttl, transform=ax.transAxes, va="top", fontsize=7)
    for lab, z in A.items():
        d = z[key]
        c, _ = STY[lab]
    ax.legend(loc="lower right", fontsize=6.5)
axs[0].set_ylabel("map points within that distance (%)")
save(fig, os.path.join(HERE, "figures", "fig_contam"))
print("wrote eval/figures/fig_contam.{pdf,png}")
