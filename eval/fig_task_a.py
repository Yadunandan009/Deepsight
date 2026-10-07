#!/usr/bin/env python3
"""
Publication figure for Task A, re-run under the corrected stereo calibration.

FORM CHOICE. The original write-up's figure was a bar chart of three
independent means with +-1 SD error bars, and reading it required comparing
bars whose denominators were different source clouds -- so the eye was
invited to make a comparison the statistic does not support. The question
is actually paired: for ONE source cloud, does its own face's reference
geometry register better than the other face's? So:

  (a) shows the pairing itself -- each faint line is one bootstrap resample,
      drawn from its fitness against its own reference to its fitness
      against the other face's. The direction of the lines IS the result;
      no error bar is needed to read it.

  (b) is the statistic that follows from (a): the difference own - other,
      with a percentile 95% CI, as a forest plot against a zero line. Rows
      are the three runs, so whether the finding replicates is visible
      rather than asserted.

Run:  python3 eval/fig_task_a.py <boot_orig.npz> <boot_fixonly.npz> <boot_fxfast.npz>
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib.pyplot as plt
from pubstyle import use, save, despine, COL2, SERIES, MARKERS, INK_SECONDARY

LABELS = ['original\n(fx = 457.1)', 'corrected fx\n(FAST 3/1)',
          'corrected fx\n(FAST 20/7)']

if len(sys.argv) != 4:
    sys.exit(__doc__.strip().splitlines()[-1])
runs = [np.load(p) for p in sys.argv[1:4]]
PRIMARY = 2          # the configuration the paper reports

use()
fig, axes = plt.subplots(1, 2, figsize=(COL2, COL2 * 0.36))

# -- (a) paired resamples, primary run ---------------------------------------
ax = axes[0]
D = runs[PRIMARY]
for k, (face, own, other) in enumerate([
        ('face 1', D['f1_own'], D['f1_other']),
        ('face 2', D['f2_own'], D['f2_other'])]):
    x0, x1 = k * 1.0, k * 1.0 + 0.42
    for a, b in zip(own, other):
        ax.plot([x0, x1], [a, b], color=SERIES[k], lw=0.35, alpha=0.30,
                solid_capstyle='butt', zorder=2)
    ax.plot([x0, x1], [own.mean(), other.mean()], color=SERIES[k], lw=1.6,
            marker=MARKERS[k], markersize=4, markeredgecolor='white',
            markeredgewidth=0.5, zorder=4, label=face)
ax.set_xticks([0, 0.42, 1.0, 1.42])
ax.set_xticklabels(['own', 'other', 'own', 'other'])
ax.set_xlim(-0.22, 1.64)
ax.set_ylabel('ICP fitness (inlier fraction)')
ax.set_title('(a) paired registration, corrected calibration', loc='left')
ax.legend(loc='upper right', ncol=2)
# Name which reference "own"/"other" are, once, rather than in a caption the
# reader may not have beside the figure.
ax.annotate('reference geometry', (0.5, -0.185), xycoords='axes fraction',
            ha='center', fontsize=6.5, color=INK_SECONDARY)
despine(ax)

# -- (b) forest plot of the paired difference -------------------------------
ax = axes[1]
ypos, yticklab, ytickcol = [], [], []
row = 0
for k, key in enumerate([('f1_own', 'f1_other'), ('f2_own', 'f2_other')]):
    for j, D in enumerate(runs):
        d = D[key[0]] - D[key[1]]
        lo, hi = np.percentile(d, [2.5, 97.5])
        ax.plot([lo, hi], [row, row], color=SERIES[k], lw=1.1,
                solid_capstyle='butt', zorder=3)
        ax.plot([d.mean()], [row], marker=MARKERS[k], markersize=4.2,
                color=SERIES[k], markeredgecolor='white', markeredgewidth=0.5,
                zorder=4)
        ypos.append(row); yticklab.append(LABELS[j]); ytickcol.append(SERIES[k])
        row += 1
    row += 0.8
ax.axvline(0, color=INK_SECONDARY, lw=0.7, ls='--', zorder=1)
ax.set_yticks(ypos)
ax.set_yticklabels(yticklab, fontsize=6.2)
for lab, c in zip(ax.get_yticklabels(), ytickcol):
    lab.set_color(c)
ax.invert_yaxis()
ax.set_xlabel('fitness against own face $-$ against other face')
ax.set_title('(b) does a face prefer its own geometry?', loc='left')
ax.grid(axis='y', visible=False)
ax.grid(axis='x', visible=True)
# Direction cue: a reader should not have to work out which side is which.
ax.annotate('prefers own $\\rightarrow$', (0.985, 0.03), xycoords='axes fraction',
            ha='right', fontsize=6.2, color=INK_SECONDARY)
despine(ax)

fig.tight_layout(w_pad=2.0)
out = os.path.expanduser('~/ros2_ws/docs/task_a/task_a_figures/task_a_paired_registration')
print('wrote:', ', '.join(save(fig, out)))
