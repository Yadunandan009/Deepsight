#!/usr/bin/env python3
"""Section 4.4: horizontal position RMS of the primary (fused) filter against the
SLAM-free filter, over the window from the first published SLAM pose to the first
wrong-face closure (1000 s when the flight had none).

Reads the cached series of eval/gate_plateau.py (gate_plateau_series.npz and
gate_plateau.json) and eval/figures/flights_summary.json; no bag is opened.

Run: python3 eval/pre_window_ratio.py
"""
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
F = os.path.join(HERE, "figures")
gp = json.load(open(os.path.join(F, "gate_plateau.json")))
ser = np.load(os.path.join(F, "gate_plateau_series.npz"))
fs = {r["id"]: r for r in json.load(open(os.path.join(F, "flights_summary.json")))}

print(f"{'flight':11s} {'window (s)':>16s} {'DR rms':>7s} {'fused rms':>9s} {'ratio':>6s}")
ratios = {}
for k in gp:
    t0 = gp[k]["first_bridge_s"]
    t1 = fs[k].get("first_aliased_t") or 1000.0
    t1 = min(t1, 1000.0)
    g, ef, ed = ser[k + "_g"], ser[k + "_ef"], ser[k + "_ed"]
    m = (g >= t0) & (g <= t1)
    d, f = np.sqrt(np.mean(ed[m] ** 2)), np.sqrt(np.mean(ef[m] ** 2))
    ratios[k] = f / d
    print(f"{k:11s} {t0:7.0f}-{t1:6.0f} {d:7.2f} {f:9.2f} {f / d:6.2f}")
bB = [k for k in ratios if k != "OB_s1"]
v = np.array([ratios[k] for k in bB])
print(f"bridge B ({len(bB)} flights): ratio min {v.min():.2f} max {v.max():.2f}; >1 in {(v > 1).sum()}")
