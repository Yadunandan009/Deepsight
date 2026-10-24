#!/usr/bin/env python3
"""Figure: ORB-SLAM3 during a flight, two screenshots of its own viewer.

Inputs: docs/paper/stonefish_screens/orbslam3_current_frame.png and
orbslam3_map_viewer.png, taken by hand during a live run; no bag holds camera
images, so these are the only views of what the cameras saw.

Re-run: python3 eval/fig_orbslam.py
"""
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from pubstyle import use_journal, save, COL2

SRC = os.path.join(os.path.dirname(HERE), "docs", "paper", "stonefish_screens")
PANELS = [("orbslam3_current_frame.png", "(a) current frame, tracked features"),
          ("orbslam3_map_viewer.png", "(b) map and keyframe trajectory")]

use_journal()
ims = []
for fn, _ in PANELS:
    im = Image.open(os.path.join(SRC, fn)).convert("RGB")
    if fn.startswith("orbslam3_current"):
        im = im.crop((5, 32, im.width - 5, 537))         # drop the title bar, border and the terminal strip below
    ims.append(np.asarray(im))
ratios = [a.shape[1] / a.shape[0] for a in ims]
fig, axes = plt.subplots(1, 2, figsize=(COL2, COL2 / sum(ratios) + 0.35),
                         gridspec_kw=dict(width_ratios=ratios))
for ax, a, (_, ttl) in zip(axes, ims, PANELS):
    ax.imshow(a)
    ax.set_title(ttl, fontsize=7)
    ax.axis("off")
fig.tight_layout(pad=0.4)
save(fig, os.path.join(HERE, "figures", "fig_orbslam"))
print("wrote eval/figures/fig_orbslam.{pdf,png}")
