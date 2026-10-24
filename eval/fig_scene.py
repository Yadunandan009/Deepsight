#!/usr/bin/env python3
"""Figure 1b: the simulated scene, four screenshots from Stonefish's own viewer.

Inputs are the PNGs in docs/paper/stonefish_screens/ (taken by hand in the running
simulator). They are viewer screenshots, not images from the vehicle's stereo cameras,
and the viewer's position in (b) and (c) is approximate (about mid-depth). Each is downsampled to 640 px
wide so the PDF stays small.

Re-run: python3 eval/fig_scene.py
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
PANELS = [("jacket_above_front.png", "(a) jacket, water not rendered"),
          ("jacket_underwater_far.png", "(b) mid-depth, far view"),
          ("jacket_underwater_near.png", "(c) mid-depth, nearer view"),
          ("rov_at_surface.png", "(d) BlueROV2 Heavy at the surface")]

use_journal()
fig, axes = plt.subplots(1, 4, figsize=(COL2, 2.35))
for ax, (fn, ttl) in zip(axes, PANELS):
    im = Image.open(os.path.join(SRC, fn)).convert("RGB")
    w = 640
    im = im.resize((w, int(im.height * w / im.width)), Image.LANCZOS)
    ax.imshow(np.asarray(im))
    ax.set_title(ttl, fontsize=7)
    ax.axis("off")
fig.tight_layout(pad=0.4)
save(fig, os.path.join(HERE, "figures", "fig_scene"))
print("wrote eval/figures/fig_scene.{pdf,png}")
