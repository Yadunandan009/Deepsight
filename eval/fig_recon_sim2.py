#!/usr/bin/env python3
"""Figure: the inspection product, flight by flight, Sim(2) bridge.

Reads eval/figures/recon_results.json (group "sim2", from recon_batch.py).
Reconstruction RMS against the jacket CAD, whole run, for each flight: ground-truth
poses (the sensor floor), the SLAM-free dead-reckoned filter, the primary filter
that fuses SLAM, and, for flights with a wrong-face closure, the AFTER-closure RMS
of the trust-SLAM counterfactual. Log axis. Only stock-scenario flights are shown:
L4 and L5 carry hardware the CAD does not contain, so their reconstructions are not
scored against the bare jacket.

Re-run: python3 eval/recon_batch.py && python3 eval/fig_recon_sim2.py
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
R = json.load(open(os.path.join(HERE, "figures", "recon_results.json")))
M = {r["id"]: r for r in json.load(open(os.path.join(HERE, "figures", "flights_summary.json")))}
ids = [k for k in ("slamref_s0", "slamref_s1", "slamref_s2", "L0_s0", "L0_s1", "L0_s2") if k in M]
rows = []
for k in ids:
    key = "sim2/" + os.path.basename(M[k]["bag"])
    if key in R and "sources" in R[key]:
        rows.append((k, R[key]))
fig, ax = plt.subplots(figsize=(COL2, 2.7))
w = 0.19
SRC = (("truth", "ground-truth poses (floor)", "#bbbbbb", None),
       ("dr", "dead reckoning (SLAM-free)", SERIES[2], None),
       ("filtered", "primary filter (SLAM fused)", SERIES[1], None),
       ("slam_trust", "trust-SLAM counterfactual, after the closure", SERIES[3], "after_closure"))
for j, (src, lab, col, part) in enumerate(SRC):
    xs, ys = [], []
    for i, (k, r) in enumerate(rows):
        s = r["sources"].get(src)
        if not s:
            continue
        v = (s.get(part) or s["all"]) if part else s["all"]
        xs.append(i + (j - 1.5) * w)
        ys.append(v["rms"])
    ax.bar(xs, ys, width=w * 0.92, color=col, label=lab, edgecolor="white", lw=0.4)
    for x, y in zip(xs, ys):
        ax.text(x, y * 1.08, "%.1f m" % y if y >= 1 else "%.0f cm" % (y * 100), ha="center", fontsize=4.8,
                rotation=90, va="bottom")
ax.set_yscale("log")
ax.set_ylim(0.04, 40)
ax.set_xticks(range(len(rows)))
ax.set_xticklabels([k.replace("slamref_s", "stock\nseed ").replace("L0_s", "L0 stage 1\nseed ")
                    for k, _ in rows], fontsize=6.5)
ax.set_ylabel("reconstruction RMS to the CAD (m)")
ax.legend(loc="upper left", fontsize=6, ncol=2, columnspacing=0.8)
save(fig, os.path.join(HERE, "figures", "fig_recon_sim2"))
print("wrote eval/figures/fig_recon_sim2.{pdf,png}", [k for k, _ in rows])
