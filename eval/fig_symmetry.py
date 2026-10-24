#!/usr/bin/env python3
"""Figure: how different the faces are in each arm, from the CAD.

Runs structure_symmetry.py on the three scenarios (cached in
eval/figures/symmetry_arms.json; --refresh to recompute) and plots, per pair of
faces, the LOWER of the two ICP fitness values (forward and reverse) at a 1.0 m
inlier threshold. Two faces are interchangeable only if each covers the other, so
the pair score is the minimum. 1.0 = interchangeable. The pre-set criterion for the
L5 arm was: no pair above 0.8.

One 4 x 4 grid per arm: the pair score of every pair of faces, bold above the L5 criterion (0.8, the line on the colour bar).
"""
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from pubstyle import use_journal, save, COL2, SERIES

SC = os.path.join(os.path.dirname(HERE), "src", "stonefish_bluerov2", "scenarios")
ARMS = (("L0", "bluerov2_turbine.scn"), ("L4", "bluerov2_turbine_sym_l4.scn"),
        ("L5", "bluerov2_turbine_sym_l5.scn"))
CACHE = os.path.join(HERE, "figures", "symmetry_arms.json")


def run(scn):
    out = subprocess.run([sys.executable, os.path.join(HERE, "structure_symmetry.py"),
                          "--scn", os.path.join(SC, scn), "--seed", "0"],
                         capture_output=True, text=True).stdout
    pairs = {}
    for m in re.finditer(r"f(\d) vs f(\d)\s+([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)", out):
        pairs["%s-%s" % (m.group(1), m.group(2))] = dict(fwd=float(m.group(3)),
                                                         rev=float(m.group(5)),
                                                         pair_min=float(m.group(6)))
    worst = re.search(r"WORST PAIR.*?=\s*([0-9.]+)", out)
    mean = re.search(r"mean of pair minima\s*=\s*([0-9.]+)", out)
    return dict(pairs=pairs, worst=float(worst.group(1)) if worst else None,
                mean_min=float(mean.group(1)) if mean else None)


if "--refresh" in sys.argv or not os.path.exists(CACHE):
    res = {a: run(s) for a, s in ARMS}
    json.dump(res, open(CACHE, "w"), indent=1)
res = json.load(open(CACHE))

use_journal()
import numpy as np
fig, axes = plt.subplots(1, 3, figsize=(COL2, 2.55))
cmap = plt.get_cmap("Blues")
for ax, (arm, _) in zip(axes, ARMS):
    p = res[arm]["pairs"]
    M = np.full((4, 4), np.nan)
    for k, v in p.items():
        a, b = map(int, k.split("-"))
        M[a, b] = M[b, a] = v["pair_min"]
    im = ax.imshow(M, cmap=cmap, vmin=0.5, vmax=1.0)
    for a in range(4):
        for b in range(4):
            if a == b:
                ax.text(b, a, "-", ha="center", va="center", fontsize=7, color="#999")
                continue
            v = M[a, b]
            ax.text(b, a, "%.2f" % v, ha="center", va="center", fontsize=6.8,
                    color="white" if v > 0.8 else "#222", fontweight="bold" if v > 0.8 else "normal")
    ax.set_xticks(range(4)); ax.set_yticks(range(4))
    ax.set_xticklabels(["face %d" % k for k in range(4)], fontsize=6.5)
    ax.set_yticklabels(["face %d" % k for k in range(4)], fontsize=6.5)
    ax.tick_params(length=0)
    ax.grid(False)
    w = res[arm]["worst"]
    ax.set_title("%s: worst pair %.3f" % (arm, w), fontsize=7.5)
cb = fig.colorbar(im, ax=axes, fraction=0.025, pad=0.02)
cb.set_label("pair score (1 = interchangeable)", fontsize=6.5)
cb.ax.axhline(0.8, color="k", lw=0.8)
cb.ax.tick_params(labelsize=6)
save(fig, os.path.join(HERE, "figures", "fig_symmetry"))
print("wrote eval/figures/fig_symmetry.{pdf,png}")
print(json.dumps({a: (res[a]['worst'], res[a]['mean_min']) for a, _ in ARMS}))
