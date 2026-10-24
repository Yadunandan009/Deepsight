#!/usr/bin/env python3
"""Figure: the structure from the CAD, stock against L5.

Top: the jacket in 3D (shaded CAD triangles, east-north-up so nothing is mirrored).
(a) stock, seen from the south-east; (b) L5 from the south-east, so faces 0 (south) and
1 (east) face the reader; (c) L5 from the north-west, faces 2 (north) and 3 (west).
Bottom: L5 seen square-on from outside each face; only the half of the jacket nearer
the viewer and that face's own hardware are drawn, so each panel shows one face.
Hardware on the faces turned away from the reader is drawn faded. Hardware: risers on face 0 (blue), anode columns on face 1 (orange), horizontal pipe
runs on face 2 (green); face 3 is bare. Depth below the surface; water line (blue) and
seabed (brown). The tower above the topside, the second structure and the seabed plane
are not drawn. Not a camera image or a sonar return.

Re-run: python3 eval/fig_cad.py
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import structure_symmetry as ss
from pubstyle import use_journal, save, COL2, SERIES

use_journal()
GREY = np.array([0.62, 0.62, 0.60])
HWCOL = {0: SERIES[0], 1: SERIES[1], 2: SERIES[2]}
HWNAME = {0: "risers", 1: "anode columns", 2: "pipe runs"}
FACES = [(0, 180.0, "south"), (1, 90.0, "east"), (2, 0.0, "north"), (3, 270.0, "west")]
AX_E, AX_N = ss.TURBINE_Y, ss.TURBINE_X        # structure axis in east, north
LIGHT = np.array([0.35, -0.45, 0.8]); LIGHT /= np.linalg.norm(LIGHT)


def hexrgb(c):
    c = c.lstrip("#")
    return np.array([int(c[i:i + 2], 16) / 255.0 for i in (0, 2, 4)])


def scene(scn):
    """[(triangles in ENU relative to the axis, rgb, face index or None)]"""
    out = []
    for e in ss.parse_scenario(os.path.join(ss.WS, "scenarios", scn + ".scn")):
        n = e["name"]
        if n.startswith("FaceHardware") or n.startswith("FaceRun"):
            face = int(n[len("FaceHardware")]) if n.startswith("FaceHardware") else int(n[len("FaceRun")])
        elif n == "Turbine2_base":
            face = None
        else:
            continue
        V, F = ss.load_obj(e["mesh"])
        V = V * e["scale"]
        V = V @ ss.rpy_matrix(*e["origin_rpy"]).T + e["origin_xyz"]
        V = V @ ss.rpy_matrix(*e["world_rpy"]).T + e["world_xyz"]
        enu = np.column_stack([V[:, 1] - AX_E, V[:, 0] - AX_N, -V[:, 2]])   # NED -> ENU
        col = GREY if face is None else hexrgb(HWCOL[face])
        out.append((enu[F], col, face))
    return out


def shade(T, col):
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    nl = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = nrm / np.where(nl == 0, 1, nl)
    k = 0.45 + 0.55 * np.abs(nrm @ LIGHT)
    return np.clip(col[None, :] * k[:, None], 0, 1)


def basis(bearing, elev):
    b, el = math.radians(bearing), math.radians(elev)
    d = np.array([math.sin(b) * math.cos(el), math.cos(b) * math.cos(el), math.sin(el)])  # toward camera
    f = -d
    r = np.cross(f, [0.0, 0.0, 1.0]); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return d, r, u


def draw(ax, parts, bearing, elev, keep=None, tint=None):
    """keep(T, face, d) -> boolean mask per triangle; tint(face) -> True to fade it."""
    d, r, u = basis(bearing, elev)
    polys, cols, depth = [], [], []
    for T, col, face in parts:
        if keep is not None:
            m = keep(T, face, d)
            if not np.any(m):
                continue
            T = T[m]
        if tint is not None and tint(face):
            col = col * 0.35 + 0.65                  # behind the lattice: faded
        c = shade(T, col)
        sx, sy, dz = T @ r, T @ u, (T @ d).mean(axis=1)
        for i in range(len(T)):
            polys.append(np.column_stack([sx[i], sy[i]])); cols.append(c[i]); depth.append(dz[i])
    o = np.argsort(depth)
    ax.add_collection(PolyCollection([polys[i] for i in o], facecolors=[cols[i] for i in o],
                                     edgecolors="none", linewidths=0))
    return d, r, u


def view3d(ax, parts, bearing, title, labels):
    d, r, u = draw(ax, parts, bearing, 18, tint=lambda face: face is not None and face not in labels)
    for z, c in ((0.0, "#3b7ea1"), (-25.0, "#8a7a5a")):
        sq = np.array([[-14, -14, z], [14, -14, z], [14, 14, z], [-14, 14, z], [-14, -14, z]])
        ax.plot(sq @ r, sq @ u, c=c, lw=0.6)
    for fi, fb, nm in FACES:
        if fi in labels:
            p = np.array([15.5 * math.sin(math.radians(fb)), 15.5 * math.cos(math.radians(fb)), -25.0])
            ax.text(p @ r, p @ u - 2.5, f"face {fi}", ha="center", va="top", fontsize=6.5)
    ax.set_xlim(-22, 22); ax.set_ylim(-33, 31); ax.set_aspect("equal"); ax.set_axis_off()
    ax.set_title(title, fontsize=7.5)


stock, l5 = scene("bluerov2_turbine"), scene("bluerov2_turbine_sym_l5")
fig = plt.figure(figsize=(COL2, 6.6))
gs = fig.add_gridspec(2, 12, height_ratios=[1.25, 1], hspace=0.16, wspace=0.6)
view3d(fig.add_subplot(gs[0, 0:4]), stock, 135, "(a) stock, from the south-east", {0, 1})
view3d(fig.add_subplot(gs[0, 4:8]), l5, 135, "(b) L5, faces 0 and 1 in front", {0, 1})
view3d(fig.add_subplot(gs[0, 8:12]), l5, 315, "(c) L5, faces 2 and 3 in front", {2, 3})

for k, (fi, fb, nm) in enumerate(FACES):
    ax = fig.add_subplot(gs[1, 3 * k:3 * k + 3])
    def near(T, face, d, fi=fi):
        if face is None:
            return (T @ d).mean(axis=1) > 0.0          # jacket triangles in front of the axis
        return np.full(len(T), face == fi)
    draw(ax, l5, fb, 0.0, keep=near)
    ax.axhline(0, c="#3b7ea1", lw=0.6); ax.axhline(-25, c="#8a7a5a", lw=0.6)
    ax.set_xlim(-13, 13); ax.set_ylim(-27, 4); ax.set_aspect("equal")
    ax.set_yticks([0, -5, -10, -15, -20, -25]); ax.set_yticklabels(["0", "5", "10", "15", "20", "25"])
    ax.set_title(f"face {fi} ({nm}): " + (HWNAME[fi] if fi in HWNAME else "bare"), fontsize=7)
    ax.set_xlabel("across the face (m)")
    if k == 0:
        ax.set_ylabel("depth below surface (m)")
    else:
        ax.set_yticklabels([])
fig.text(0.5, 0.465, "(d) L5 seen square-on from outside each face (near half of the jacket and that face's hardware only)",
         ha="center", fontsize=7.5)
save(fig, os.path.join(HERE, "figures", "fig_cad"))
print("wrote eval/figures/fig_cad.{pdf,png}")
