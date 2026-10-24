#!/usr/bin/env python3
"""Manuscript figures for the inspection-product result.

  Figure `recon_3d`        (double column)  four 3-D reconstructions
      The structure rebuilt from the vehicle's own sonar in the 2026-10-16 16:00
      mission, as a 3-D point cloud under four navigation sources: the true
      poses, dead reckoning, the SLAM-fused filter, and a vehicle that trusted
      SLAM through the wrong-face closure. The same sonar returns in every panel;
      only the pose that places them changes. Colour is the distance of each
      point to the CAD mesh.

  Figure `recon_explain`   (double column)  what the four pictures mean
      (a) cumulative distribution of that distance for each source: the fraction
          of the reconstruction that lies within a given distance of the CAD.
      (b) the same error through the mission, in 10 s windows, with the
          wrong-face closure marked: the four agree until it, then separate.

  Figure `recon_by_source` (single column)  the same measure across recordings
      One dot per recording, one marker shape throughout; the bar is the median.
      Groups are named on the axis, not coded by symbol.

Run:  /usr/bin/python3 eval/fig_recon.py          render from the cache
      python3 eval/fig_recon.py --refresh         rebuild the cache first
                                                  (needs mcap_ros2, so the venv)
THE 3-D PANELS NEED /usr/bin/python3. The venv's matplotlib (3.10) cannot load
mplot3d, because the system's older mpl_toolkits shadows it ("Unknown
projection '3d'"). Build the cache with the venv, render with the system one.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np

RES = os.path.join(HERE, "figures", "recon_results.json")
CACHE = os.path.join(HERE, "figures", "recon_3d.npz")
BAG = os.path.join(HERE, "..", "bags", "a50_slamned_seed0_20261016_160033")
CLOSURE_T = 573.0
SOURCES = [("truth", "true poses"), ("dr", "dead reckoning"),
           ("filtered", "SLAM-fused filter"),
           ("slam_trust", "trusting SLAM through\nthe wrong-face closure")]
N_PLOT, WIN = 22000, 10.0
XLIM, YLIM, ZLIM = (-23, 14), (2, 36), (0, 25)       # east, north, depth (m)


def build_cache():
    import sonar_recon as sr
    res = sr.score(BAG, CLOSURE_T)
    rng = np.random.default_rng(0)
    out = {}
    xs = np.logspace(np.log10(0.01), np.log10(30), 240)
    for name, _ in SOURCES:
        pw, d, trel = res["pts"][name]
        # keep the points that fall inside the plotted volume, then thin
        inside = ((pw[:, 1] > XLIM[0]) & (pw[:, 1] < XLIM[1]) &
                  (pw[:, 0] > YLIM[0]) & (pw[:, 0] < YLIM[1]) &
                  (pw[:, 2] > -0.5) & (pw[:, 2] < ZLIM[1] + 0.5))
        idx = np.flatnonzero(inside)
        k = rng.choice(idx, size=min(N_PLOT, len(idx)), replace=False)
        out[f"{name}_xyz"], out[f"{name}_d"] = pw[k], d[k]
        out[f"{name}_ecdf_x"] = xs
        out[f"{name}_ecdf_y"] = np.array([np.mean(d <= x) for x in xs])
        edges = np.arange(0, trel.max() + WIN, WIN)
        ctr, rms = [], []
        for a, b in zip(edges[:-1], edges[1:]):
            m = (trel >= a) & (trel < b)
            if m.sum() > 50:
                ctr.append(0.5 * (a + b))
                rms.append(float(np.sqrt(np.mean(d[m] ** 2))))
        out[f"{name}_wt"], out[f"{name}_wrms"] = np.array(ctr), np.array(rms)
        out[f"{name}_rms"] = np.array(res["sources"][name]["all"]["rms"])
        out[f"{name}_in25"] = np.array([np.mean(d[trel >= CLOSURE_T] < 0.25)])
    _, P = sr.mesh_tree()
    inside = ((P[:, 1] > XLIM[0]) & (P[:, 1] < XLIM[1]) & (P[:, 0] > YLIM[0]) &
              (P[:, 0] < YLIM[1]) & (P[:, 2] > -0.5) & (P[:, 2] < ZLIM[1] + 0.5))
    Q = P[inside]
    out["mesh_xyz"] = Q[rng.choice(len(Q), size=min(30000, len(Q)), replace=False)]
    np.savez(CACHE, **out)


def _style():
    from pubstyle import use_journal, SERIES
    use_journal()
    return {"truth": ("#222222", "-"), "dr": (SERIES[0], "--"),
            "filtered": (SERIES[1], "-"), "slam_trust": (SERIES[1], (0, (4, 1.5)))}


def plot_3d():
    import matplotlib.pyplot as plt
    from pubstyle import save, COL2
    _style()
    Z = np.load(CACHE)
    fig = plt.figure(figsize=(COL2, 5.6))
    sc = None
    width, height = 0.47, 0.385
    for i, ((name, title), tag) in enumerate(zip(SOURCES, "abcd")):
        row, col = divmod(i, 2)
        x0, y0 = 0.02 + col * 0.495, 0.535 - row * 0.465
        ax = fig.add_axes([x0, y0, width, height], projection="3d")
        ax.patch.set_visible(False)
        m = Z["mesh_xyz"]
        ax.scatter(m[:, 1], m[:, 0], m[:, 2], s=0.3, c="#c8c8c8", alpha=0.35,
                   linewidths=0, rasterized=True, depthshade=False)
        p, d = Z[f"{name}_xyz"], Z[f"{name}_d"]
        o = np.argsort(d)
        sc = ax.scatter(p[o, 1], p[o, 0], p[o, 2], c=np.clip(d[o], 0, 1.0), s=1.3,
                        cmap="viridis", vmin=0, vmax=1.0, linewidths=0,
                        rasterized=True, depthshade=False)
        ax.set_xlim(*XLIM)
        ax.set_ylim(*YLIM)
        ax.set_zlim(*ZLIM)
        ax.invert_zaxis()
        ax.set_box_aspect((XLIM[1] - XLIM[0], YLIM[1] - YLIM[0],
                           1.15 * (ZLIM[1] - ZLIM[0])), zoom=1.12)
        ax.view_init(elev=17, azim=-58)
        ax.set_xticks([-20, -10, 0, 10])
        ax.set_yticks([10, 20, 30])
        ax.set_zticks([0, 10, 20])
        ax.tick_params(labelsize=6.2, pad=-3)
        ax.set_xlabel("east (m)", fontsize=7, labelpad=-5)
        ax.set_ylabel("north (m)", fontsize=7, labelpad=-5)
        ax.set_zlabel("depth (m)", fontsize=7, labelpad=-5)
        for a in (ax.xaxis, ax.yaxis, ax.zaxis):
            a.pane.set_facecolor((1, 1, 1, 0))
            a.pane.set_edgecolor("#bbbbbb")
            a._axinfo["grid"]["linewidth"] = 0.25
            a._axinfo["grid"]["color"] = "#dddddd"
        rms = float(Z[f"{name}_rms"])
        fig.text(x0 + 0.005, y0 + height + 0.012, f"({tag})", fontsize=9,
                 fontweight="bold", va="center")
        fig.text(x0 + 0.045, y0 + height + 0.012, title.replace("\n", " "),
                 fontsize=8, va="center")
        fig.text(x0 + width - 0.01, y0 + height + 0.012,
                 (f"RMS {100 * rms:.0f} cm" if rms < 0.5 else f"RMS {rms:.1f} m"),
                 ha="right", va="center", fontsize=8, color="#333333")
    cax = fig.add_axes([0.30, 0.035, 0.40, 0.014])
    cb = fig.colorbar(sc, cax=cax, orientation="horizontal")
    cb.set_label("distance of a reconstructed point to the CAD (m), clipped at 1 m",
                 fontsize=7, labelpad=2)
    cb.ax.tick_params(labelsize=6.5)
    return save(fig, os.path.join(HERE, "figures", "recon_3d"))


def plot_explain():
    import matplotlib.pyplot as plt
    from pubstyle import save, COL2
    sty = _style()
    Z = np.load(CACHE)
    fig, (a, b) = plt.subplots(1, 2, figsize=(COL2, 2.7),
                               gridspec_kw=dict(wspace=0.26, left=0.08,
                                                right=0.985, bottom=0.17, top=0.96))
    labs = {"truth": "true poses", "dr": "dead reckoning",
            "filtered": "SLAM-fused filter",
            "slam_trust": "trusting SLAM through\nthe wrong-face closure"}
    for name, _ in SOURCES:
        c, ls = sty[name]
        a.plot(Z[f"{name}_ecdf_x"], 100 * Z[f"{name}_ecdf_y"], color=c, ls=ls,
               lw=1.2, label=labs[name])
        b.plot(Z[f"{name}_wt"], Z[f"{name}_wrms"], color=c, ls=ls, lw=1.2)
    a.set_xscale("log")
    a.set_xlim(0.02, 30)
    a.set_ylim(0, 102)
    a.axvline(0.25, color="black", lw=0.5, ls=(0, (2, 2)))
    a.set_xlabel("distance of a reconstructed point to the CAD (m)")
    a.set_ylabel("share of the reconstruction\nwithin that distance (%)")
    a.text(0.27, 4, "25 cm", fontsize=6.5, ha="left", va="bottom")
    a.legend(loc="lower right", frameon=False, fontsize=6.8, handlelength=2.4,
             labelspacing=0.55, borderaxespad=0.5)
    b.axvline(CLOSURE_T, color="black", lw=0.6)
    b.text(CLOSURE_T - 12, 17, "wrong-face\nclosure", fontsize=6.8, ha="right",
           va="center")
    b.set_yscale("log")
    b.set_ylim(0.04, 40)
    b.set_xlim(0, 1130)
    b.set_yticks([0.1, 1, 10])
    b.set_yticklabels(["0.1", "1", "10"])
    b.set_xlabel("time into the mission (s)")
    b.set_ylabel("reconstruction error\n(RMS in 10 s windows, m)")
    for ax, s_ in ((a, "(a)"), (b, "(b)")):
        ax.text(0.02, 0.96, s_, transform=ax.transAxes, fontsize=8.5,
                fontweight="bold", va="top")
    rows = [("true poses", "truth"), ("dead reckoning", "dr"),
            ("SLAM-fused", "filtered"), ("trusting SLAM", "slam_trust")]
    b.text(0.05, 0.80, "within 25 cm, after the closure",
           transform=b.transAxes, fontsize=6.4, ha="left", va="top", weight="bold")
    for k, (lab, n) in enumerate(rows):
        b.text(0.05, 0.735 - 0.063 * k,
               f"{lab}: {100 * float(Z[n + '_in25']):.0f} %",
               transform=b.transAxes, fontsize=6.4, ha="left", va="top")
    return save(fig, os.path.join(HERE, "figures", "recon_explain"))


def plot_by_source():
    import matplotlib.pyplot as plt
    from pubstyle import save, COL1, SERIES
    _style()
    R = json.load(open(RES))
    ok = {k: v for k, v in R.items() if "failed" not in v}
    # Recordings that must not count as "dead reckoning under the a50 model":
    #   200040 SLAM was in the loop although the arm reads `dr`
    #   210804 white noise only, before the DVL error model existed
    #   162539 named seed 2, but its measured DVL gains are 1.0000 (no error)
    NOMODEL, CIRC, BASE = ("a50_slamref_seed2_20261014_162539",
                           "a50_dr_seed1_20261012_200040",
                           "a50_dr_seed0_20261009_210804")
    POST = ("a50_slamned_seed0_20261016_160033", "a50_slamdrift_seed0_20261016_190745")
    cats = {"truth": [], "dr": [], "pre": [], "post": [], "loop": [], "trust": []}
    for key, v in ok.items():
        g, bag = key.split("/", 1)
        s = v["sources"]
        rms = lambda n: s[n]["all"]["rms"] if n in s else None
        cats["truth"].append(rms("truth"))
        if g == "dr_only" and bag == CIRC:
            cats["loop"].append(rms("filtered"))
        if g == "dr_only" and bag not in (CIRC, BASE):
            cats["dr"].append(rms("filtered"))
        if g == "paired" and bag != NOMODEL:
            cats["dr"].append(rms("dr"))
            cats["post" if bag in POST else "pre"].append(rms("filtered"))
        if g == "wrong_face":
            cats["trust"].append(rms("slam_trust"))
            if bag == POST[0]:
                cats["post"].append(rms("filtered"))
                cats["dr"].append(rms("dr"))
    order = [("truth", "true poses\n(sensor floor)", "#6e6e6e"),
             ("dr", "dead\nreckoning", SERIES[0]),
             ("pre", "SLAM-fused,\nbefore axis\nfix", "#e8956f"),
             ("loop", "SLAM aligned\nto its own\nfilter", "#e8956f"),
             ("post", "SLAM-fused,\nafter axis\nfix", SERIES[1]),
             ("trust", "SLAM trusted\nthrough a\nwrong closure", "#b5481c")]
    fig, ax = plt.subplots(figsize=(COL1 * 1.35, 2.8))
    rng = np.random.default_rng(1)
    ncount = {}
    for i, (k, lab, col) in enumerate(order):
        y = np.array([v for v in cats[k] if v is not None])
        if not len(y):
            continue
        x = i + (rng.random(len(y)) - 0.5) * 0.36
        ax.plot(x, y, ls="", marker="o", ms=4.6, color=col, mec="white", mew=0.5,
                zorder=3)
        ax.plot([i - 0.30, i + 0.30], [np.median(y)] * 2, color="black", lw=1.1,
                zorder=4)
        ncount[k] = len(y)
    ax.set_yscale("log")
    ax.set_ylim(0.05, 14)
    ax.set_xlim(-0.55, len(order) - 0.45)
    ax.set_xticks(range(len(order)))
    ax.set_xticklabels([f"{o[1]}\nn = {ncount.get(o[0], 0)}" for o in order],
                       fontsize=6.0, linespacing=1.05)
    ax.tick_params(axis="x", which="both", top=False, bottom=False)
    ax.set_yticks([0.1, 1, 10])
    ax.set_yticklabels(["0.1", "1", "10"])
    ax.set_ylabel("reconstruction RMS error\nagainst the CAD (m)")
    return save(fig, os.path.join(HERE, "figures", "recon_by_source"))


def plot_slam_map():
    """ORB-SLAM3's own map beside the sonar reconstruction, same colour scale."""
    import matplotlib.pyplot as plt
    from pubstyle import save, COL2
    _style()
    Z, M = np.load(CACHE), np.load(os.path.join(HERE, "figures", "slam_map.npz"))
    panels = [("(a)", "sonar, dead reckoning", Z["dr_xyz"], Z["dr_d"],
               "within 25 cm: %.0f %%" % (100 * np.mean(Z["dr_d"] < 0.25))),
              ("(b)", "ORB-SLAM3 map, before the closure", M["before_xyz"],
               M["before_d"], "within 25 cm: %.0f %%" % (100 * M["before_stats"][2])),
              ("(c)", "ORB-SLAM3 map, after the closure", M["after_xyz"],
               M["after_d"], "within 25 cm: %.0f %%" % (100 * M["after_stats"][2]))]
    fig = plt.figure(figsize=(COL2, 3.5))
    sc, w = None, 0.325
    for i, (tag, title, p, d, note) in enumerate(panels):
        x0 = 0.005 + i * 0.333
        ax = fig.add_axes([x0, 0.17, w, 0.70], projection="3d")
        ax.patch.set_visible(False)
        m = Z["mesh_xyz"]
        ax.scatter(m[:, 1], m[:, 0], m[:, 2], s=0.3, c="#c8c8c8", alpha=0.35,
                   linewidths=0, rasterized=True, depthshade=False)
        o = np.argsort(d)
        sc = ax.scatter(p[o, 1], p[o, 0], p[o, 2], c=np.clip(d[o], 0, 1.0), s=1.3,
                        cmap="viridis", vmin=0, vmax=1.0, linewidths=0,
                        rasterized=True, depthshade=False)
        ax.set_xlim(*XLIM); ax.set_ylim(*YLIM); ax.set_zlim(*ZLIM)
        ax.invert_zaxis()
        ax.set_box_aspect((XLIM[1] - XLIM[0], YLIM[1] - YLIM[0],
                           1.15 * (ZLIM[1] - ZLIM[0])), zoom=1.12)
        ax.view_init(elev=17, azim=-58)
        ax.set_xticks([-20, -10, 0, 10]); ax.set_yticks([10, 20, 30])
        ax.set_zticks([0, 10, 20])
        ax.tick_params(labelsize=5.8, pad=-3)
        ax.set_xlabel("east (m)", fontsize=6.5, labelpad=-5)
        ax.set_ylabel("north (m)", fontsize=6.5, labelpad=-5)
        ax.set_zlabel("depth (m)", fontsize=6.5, labelpad=-5)
        for a in (ax.xaxis, ax.yaxis, ax.zaxis):
            a.pane.set_facecolor((1, 1, 1, 0)); a.pane.set_edgecolor("#bbbbbb")
            a._axinfo["grid"]["linewidth"] = 0.25
            a._axinfo["grid"]["color"] = "#dddddd"
        fig.text(x0 + 0.01, 0.93, tag, fontsize=8.5, fontweight="bold", va="center")
        fig.text(x0 + 0.055, 0.93, title, fontsize=7.4, va="center")
        fig.text(x0 + 0.01, 0.885, note, fontsize=6.8, va="center", color="#333333")
    cax = fig.add_axes([0.30, 0.075, 0.40, 0.016])
    cb = fig.colorbar(sc, cax=cax, orientation="horizontal")
    cb.set_label("distance of a point to the CAD (m), clipped at 1 m", fontsize=7,
                 labelpad=2)
    cb.ax.tick_params(labelsize=6.5)
    return save(fig, os.path.join(HERE, "figures", "slam_map_3d"))


if __name__ == "__main__":
    if "--refresh" in sys.argv or not os.path.exists(CACHE):
        build_cache()
    for f in plot_3d() + plot_explain() + plot_by_source() + plot_slam_map():
        print("wrote", os.path.relpath(f, os.path.join(HERE, "..")))
