#!/usr/bin/env python3
"""Figure 1: the testbed and the navigation architecture.

Drawn from the code and configuration, not from memory:
  scene, sensors      scenarios/bluerov2_turbine.scn
  DVL error model     scripts/dvl_bridge.py, dvl_a50.yaml
  primary filter      ekf.yaml (DVL velocity, IMU, depth; SLAM position only)
  SLAM                slam_params/bluerov2_stereo.yaml, run_stereo_slam.sh
  bridge              scripts/slam_pose_bridge.py (Sim(2) fit, 5 m gate, reset on a jump)
  reference filter    the same filter without the SLAM input (dr_reference:=true)
  controller          scripts/bluerov2_autonomous_controller.py

Ground truth feeds the scorer only (dashed). Nothing from the scorer returns to
the vehicle.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from pubstyle import use_journal, save, COL2, SERIES

use_journal()
fig, ax = plt.subplots(figsize=(COL2, 3.3))
ax.set_xlim(0, 100)
ax.set_ylim(0, 50)
ax.axis("off")
ax.grid(False)

BLUE, ORANGE, GREEN, GREY = SERIES[0], SERIES[1], SERIES[2], "#8a8a85"


def box(x, y, w, h, text, ec="#444", fc="white", fs=6.4, ls="-", lw=0.8):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.25,rounding_size=0.8",
                                ec=ec, fc=fc, lw=lw, ls=ls, zorder=2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs,
            zorder=3, linespacing=1.25)


def path(pts, color="#333", ls="-", lw=0.9, head=True):
    xs, ys = zip(*pts)
    ax.plot(xs[:-1] if head else xs, ys[:-1] if head else ys, color=color, ls=ls, lw=lw,
            solid_capstyle="butt", zorder=1)
    if head:
        ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle="-|>", mutation_scale=7,
                                     color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0, zorder=1))
        ax.plot(xs[-2:], ys[-2:], color=color, ls=ls, lw=lw, zorder=1)


# simulator
ax.add_patch(FancyBboxPatch((0.5, 1.5), 26, 47, boxstyle="round,pad=0.3,rounding_size=1.2",
                            ec=GREY, fc="#f6f6f3", lw=0.8, zorder=0))
ax.text(13.5, 46, "Stonefish v1.3 (simulated)", fontsize=7, ha="center", fontweight="bold")
box(2.5, 34, 22, 8, "stereo cameras\n640×480, 0.145 m baseline")
box(2.5, 17.5, 22, 12, "DVL A50 model\n(scale S, misalignment dR,\none unit per seed),\nIMU, pressure, sonar", fs=6.2)
box(2.5, 4, 22, 8, "vehicle and structure\nground-truth pose", fc="#eef3fb")

# SLAM branch
box(31, 34, 19, 8, "ORB-SLAM3 stereo\nstock, Atlas multi-map\n12 Hz, in Docker", ec=BLUE, fs=6.2)
box(56, 34, 20, 8, "SLAM-pose bridge\nENU→NED, Sim(2) fit,\n5 m gate, reset on jump", ec=BLUE, fs=6.2)
path([(24.8, 38), (30.7, 38)], color=BLUE)
path([(50.3, 38), (55.7, 38)], color=BLUE)

# filters
box(56, 22, 20, 9, "primary filter (EKF, 50 Hz)\nDVL + IMU + depth\n+ SLAM position", ec=ORANGE)
box(56, 9, 20, 9, "SLAM-free reference filter\nDVL + IMU + depth\n(no SLAM input)", ec=GREEN)
path([(66, 33.7), (66, 31.4)], color=BLUE)
# sensor bus
path([(24.8, 23.5), (52.5, 23.5)], head=False)
path([(52.5, 13.5), (52.5, 26.5)], head=False)
path([(52.5, 26.5), (55.7, 26.5)])
path([(52.5, 13.5), (55.7, 13.5)])
# the alignment is fitted against the reference filter
path([(76.3, 13.5), (79.3, 13.5), (79.3, 38), (76.3, 38)], color=GREEN)
ax.text(66, 5.2, "the bridge's alignment is fitted against\nthis filter, not the one it feeds", fontsize=5.8,
        ha="center", va="center", color=GREEN, linespacing=1.2)

# controller and scorer
box(82, 21.5, 17, 10, "mission controller\nsteers on the primary\nfilter; sonar range;\n17 m standoff", fs=6.1)
path([(76.3, 26.5), (81.7, 26.5)], color=ORANGE)
path([(90.5, 21.2), (90.5, 1.2), (13.5, 1.2), (13.5, 3.7)])
ax.text(52, 2.0, "thrust commands", fontsize=5.8, ha="center", va="bottom")
box(31, 5, 20, 8.5, "offline scorer (truth is\nused here only): closures,\nerror, CAD reconstruction", ec="#888", ls=(0, (3, 2)), fs=6.2)
path([(24.8, 8), (30.7, 8)], color="#888", ls=(0, (3, 2)))

save(fig, os.path.join(HERE, "figures", "fig_system"))
print("wrote eval/figures/fig_system.{pdf,png}")
