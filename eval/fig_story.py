#!/usr/bin/env python3
"""Figure: one wrong-face closure followed through, in one bridge-B flight.

Stage-1 stock flight L0 seed 0 (bag a50_risefix_seed0_20261021_205753), the 180-degree
closure at 758 s. (a) Plan view: the true track, dead reckoning, the primary (fused)
filter, ORB-SLAM3's raw pose placed by a similarity fitted on the 150 s before the
closure (as the classifier does) so the jump is visible, and the poses the bridge
published after it re-locked. (b) Horizontal error against time: fused filter, dead
reckoning, and each published SLAM pose's distance from the truth, with the 5 m gate,
the closure and the bridge's silences.

Reads the bag (truth, raw SLAM) and the caches of eval/gate_plateau.py and
eval/jump_attribution.py. Re-run: python3 eval/fig_story.py
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
import closure_events as ce
import gate_plateau as gp
from pubstyle import use_journal, save, COL2, SERIES

ROOT = os.path.dirname(HERE)
BAG = os.path.join(ROOT, "bags", "a50_risefix_seed0_20261021_205753")
FID = "L0_s0"
TC = 758.3                      # s, first wrong-face closure (Table 3)

use_journal()
D = gp.read(BAG)
tr = ce.track(BAG, "/bluerov2/odometry", False)
sl = ce.track(BAG, "/bluerov2/robot_pose_slam", True)
t0 = tr[0, 0]
ser = np.load(os.path.join(HERE, "figures", "gate_plateau_series.npz"))
ja = json.load(open(os.path.join(HERE, "figures", "jump_attribution.json")))[FID]

# raw SLAM placed by the pre-closure similarity
ts = sl[:, 0] - t0
pre = (ts >= TC - 150.0) & (ts < TC)
E = np.column_stack([np.interp(sl[pre, 0], tr[:, 0], tr[:, 1]), np.interp(sl[pre, 0], tr[:, 0], tr[:, 2])])
to_w, th, k, resid = ce.fit_similarity(sl[pre, 1:3], E)
seg = (ts >= TC - 60.0) & (ts <= TC + 40.0)
raw = np.array([to_w(p) for p in sl[seg, 1:3]])
rawt = ts[seg]

def ne(a):                      # NED (x north, y east) -> plot (east, north)
    return a[:, 2], a[:, 1]

fig = plt.figure(figsize=(COL2, 4.0))
gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.35], wspace=0.28)
ax = fig.add_subplot(gs[0])
cx, cy = ce.TURBINE
th_ = np.linspace(0, 2 * np.pi, 200)
ax.plot(cy + 17 * np.sin(th_), cx + 17 * np.cos(th_), c="#c8c8c4", lw=0.6, ls=":")
ax.plot(cy, cx, "+", c="#555", ms=6)
ax.text(cy + 0.8, cx + 0.8, "structure axis", fontsize=6, color="#555")
T = D["truth"]; ax.plot(T[:, 2], T[:, 1], c="k", lw=0.9, label="truth")
DR = D["dr"]; ax.plot(DR[:, 2], DR[:, 1], c=SERIES[0], lw=0.8, ls="--", label="dead reckoning")
F = D["filt"]; ax.plot(F[:, 2], F[:, 1], c=SERIES[1], lw=0.9, label="fused filter")
step = np.hypot(*np.diff(raw, axis=0).T)
kj = int(np.argmax(step))                 # the closure: the largest step in the window
col = SERIES[3] if len(SERIES) > 3 else "m"
ax.plot(raw[:kj + 1, 1], raw[:kj + 1, 0], c=col, lw=1.4, label="raw SLAM, placed before the closure")
ax.plot(raw[kj + 1:, 1], raw[kj + 1:, 0], c=col, lw=1.4)
ax.annotate("", xy=(raw[kj + 1, 1], raw[kj + 1, 0]), xytext=(raw[kj, 1], raw[kj, 0]),
            arrowprops=dict(arrowstyle="->", lw=0.8, color=col, ls=(0, (3, 2)), shrinkA=0, shrinkB=0))
ax.text(raw[kj, 1] + 1.0, raw[kj, 0] + 1.2, "wrong-face closure, 758 s:\nSLAM jumps 33 m, +180\u00b0", fontsize=6)
B = D["bridge"]; tb = B[:, 0] - t0
rel = tb > TC
ax.scatter(B[rel, 2], B[rel, 1], s=1.2, c="#b2182b", label="published after re-lock", zorder=3)
ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
ax.set_aspect("equal")
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2, fontsize=5.5, frameon=False)
ax.text(0.02, 0.98, "(a)", transform=ax.transAxes, va="top", fontweight="bold")

ax2 = fig.add_subplot(gs[1])
g, ef, ed = ser[FID + "_g"], ser[FID + "_ef"], ser[FID + "_ed"]
tbs, btr = ser[FID + "_tb"], ser[FID + "_btr"]
for gap in ja["bridge_gaps"]:
    ax2.axvspan(gap["gap_start"], gap["gap_start"] + gap["gap_len"], color="#eeeeea", lw=0)
ax2.scatter(tbs, btr, s=0.8, c="#b2182b", label="published SLAM pose")
ax2.plot(g, ef, c=SERIES[1], lw=1.0, label="fused filter")
ax2.plot(g, ed, c=SERIES[0], lw=1.0, ls="--", label="dead reckoning")
ax2.axhline(5.0, c="#555", lw=0.6, ls=":")
ax2.text(g[-1], 5.1, "5 m gate", ha="right", va="bottom", fontsize=6, color="#555")
ax2.axvline(TC, c="k", lw=0.6)
ax2.text(TC - 8, 5.6, "wrong-face\nclosure", ha="right", va="bottom", fontsize=6)
ax2.text(TC + 110, 0.35, "bridge silent", ha="center", fontsize=6, color="#777")
ax2.set_xlim(0, g[-1]); ax2.set_ylim(0, 6.5)
ax2.set_xlabel("time into the mission (s)"); ax2.set_ylabel("horizontal error against truth (m)")
ax2.legend(loc="upper left", fontsize=5.5, frameon=False)
ax2.text(0.98, 0.98, "(b)", transform=ax2.transAxes, va="top", ha="right", fontweight="bold")
save(fig, os.path.join(HERE, "figures", "fig_story"))
print(f"pre-closure fit: scale {k:.3f}, residual {resid:.2f} m; wrote eval/figures/fig_story.{{pdf,png}}")
