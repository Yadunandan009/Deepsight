#!/usr/bin/env python3
"""
Publication figure: fixed vs buggy thruster allocation (Task 10).

FORM CHOICE. The headline is categorical -- 7/7 missions complete against
0/10 -- so the left panel is a proportion with its confidence interval, not a
bar of means. The right panels carry the continuous metrics that characterise
HOW it fails, over a matched 180 s window, because the arms have very different
run lengths and comparing full runs would compare phase mixes instead of the
matrix.

Every point is shown rather than a bar of the mean: with n = 7 and 10 the
individual trials fit, and a reader can see that the separation is complete
rather than taking a p-value's word for it.

Run:  python3 eval/fig_task10.py
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib.pyplot as plt
from pubstyle import use, save, despine, COL2, SERIES, MARKERS
from compare_alloc import window_metrics, fisher_exact_2x2, wilson

RESULTS = os.path.expanduser('~/ros2_ws/eval/results.jsonl')
WINDOW = 180.0

use()
rows = [json.loads(l) for l in open(RESULTS) if l.strip()]
fixed = [r for r in rows if not r['buggy_alloc']]
buggy = [r for r in rows if r['buggy_alloc']]

fc, bc = sum(r['success'] for r in fixed), sum(r['success'] for r in buggy)
p = fisher_exact_2x2(fc, len(fixed) - fc, bc, len(buggy) - bc)

# Cache the per-bag extraction: re-reading 17 mcap files takes minutes and
# figure layout always needs several iterations. Delete the cache to force
# a re-extract after new trials.
CACHE = os.path.expanduser('~/ros2_ws/eval/figures/.task10_window_cache.json')
if os.path.exists(CACHE):
    m = json.load(open(CACHE))
else:
    m = {'fixed': [], 'buggy': []}
    for label, rs in (('fixed', fixed), ('buggy', buggy)):
        for r in rs:
            w = window_metrics(r['bag_path'], WINDOW)
            if w:
                m[label].append(w)
    os.makedirs(os.path.dirname(CACHE), exist_ok=True)
    json.dump(m, open(CACHE, 'w'))

fig, axes = plt.subplots(1, 3, figsize=(COL2, COL2 * 0.30))

# -- (a) completion rate with Wilson intervals -------------------------------
ax = axes[0]
for i, (lab, k, n) in enumerate([('fixed', fc, len(fixed)), ('buggy', bc, len(buggy))]):
    lo, hi = wilson(k, n)
    ax.errorbar(i, k / n, yerr=[[k / n - lo], [hi - k / n]], fmt=MARKERS[i],
                color=SERIES[i], capsize=2.5, elinewidth=0.8, markeredgewidth=0)
    ax.annotate(f'{k}/{n}', (i, k / n), textcoords='offset points',
                xytext=(7, -1), fontsize=7, color=SERIES[i])
ax.set_xlim(-0.5, 1.5); ax.set_ylim(-0.08, 1.12)
ax.set_xticks([0, 1]); ax.set_xticklabels(['fixed', 'buggy'])
ax.set_ylabel('mission completion rate')
ax.set_title('(a) mission completion', loc='left')
ax.annotate(f'Fisher exact\n$p$ = {p:.1e}', (0.5, 0.52), xycoords='axes fraction',
            ha='center', fontsize=6.5, color='#52514e')
despine(ax)

# -- (b) range to turbine, the mechanism ------------------------------------
ax = axes[1]
for i, lab in enumerate(['fixed', 'buggy']):
    v = [w['final_range_m'] for w in m[lab]]
    ax.scatter(np.full(len(v), i) + np.random.default_rng(i).normal(0, 0.028, len(v)),
               v, s=16, color=SERIES[i], marker=MARKERS[i],
               edgecolors='white', linewidths=0.4, zorder=3)
ax.axhline(17.0, color='#52514e', lw=0.7, ls='--', zorder=1)
# Label on the right, clear of the fixed cluster that sits on this line.
# An earlier version placed it left with a white background box, which
# silently masked every fixed-arm point.
ax.annotate('17 m setpoint', (1.45, 17.6), ha='right', va='bottom',
            fontsize=6.5, color='#52514e')
ax.set_ylim(13.5, 47)
ax.set_xlim(-0.5, 1.5); ax.set_xticks([0, 1]); ax.set_xticklabels(['fixed', 'buggy'])
ax.set_ylabel('range at window end (m)')
ax.set_title('(b) station keeping', loc='left')
despine(ax)

# -- (c) time spent in the scan band ----------------------------------------
ax = axes[2]
for i, lab in enumerate(['fixed', 'buggy']):
    v = [w['frac_time_in_scan_band'] for w in m[lab]]
    ax.scatter(np.full(len(v), i) + np.random.default_rng(i + 9).normal(0, 0.028, len(v)),
               v, s=16, color=SERIES[i], marker=MARKERS[i],
               edgecolors='white', linewidths=0.4, zorder=3)
ax.set_xlim(-0.5, 1.5); ax.set_ylim(-0.05, 0.85)
ax.set_xticks([0, 1]); ax.set_xticklabels(['fixed', 'buggy'])
ax.set_ylabel('fraction of window in scan band')
ax.set_title('(c) useful survey time', loc='left')
despine(ax)

fig.tight_layout(w_pad=1.6)
print('wrote:', ', '.join(save(fig, os.path.expanduser('~/ros2_ws/eval/figures/task10_alloc'))))
