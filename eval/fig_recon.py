#!/usr/bin/env python3
"""Manuscript figures for the inspection-product result.

  Figure `recon_planview`   (double column)
      The structure as reconstructed from the vehicle's own sonar in the
      2026-10-16 16:00 mission, in plan view (north up), under four navigation
      sources: the true poses, dead reckoning, the SLAM-fused filter, and a
      vehicle that trusted SLAM through the wrong-face closure. Colour is each
      point's distance to the CAD mesh. The same sonar returns in every panel;
      only the pose used to place them changes.

  Figure `recon_by_source`  (single column)
      Reconstruction RMS error against the CAD mesh, one dot per mission,
      grouped by navigation source. Log axis: the range is two orders of
      magnitude.

Run:  python3 eval/fig_recon.py [--refresh-planview]
Needs eval/figures/recon_results.json (recon_batch.py).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np

RES = os.path.join(HERE, "figures", "recon_results.json")
PV = os.path.join(HERE, "figures", "recon_planview.npz")
BAG = os.path.join(HERE, "..", "bags", "a50_slamned_seed0_20261016_160033")
CLOSURE_T = 573.0
N_PLOT = 30000
SOURCES = [("truth", "true poses"), ("dr", "dead reckoning"),
           ("filtered", "SLAM-fused filter"),
           ("slam_trust", "trusting SLAM through\nthe wrong-face closure")]


def build_planview():
    import sonar_recon as sr
    res = sr.score(BAG, CLOSURE_T)
    rng = np.random.default_rng(0)
    out = {}
    for name, _ in SOURCES:
        pw, d, trel = res["pts"][name]
        k = rng.choice(len(d), size=min(N_PLOT, len(d)), replace=False)
        out[f"{name}_xy"] = pw[k][:, :2]
        out[f"{name}_d"] = d[k]
        out[f"{name}_rms"] = np.array(res["sources"][name]["all"]["rms"])
    _, P = sr.mesh_tree()
    k = rng.choice(len(P), size=40000, replace=False)
    out["mesh_xy"] = P[k][:, :2]
    np.savez(PV, **out)


def plot_planview():
    import matplotlib.pyplot as plt
    from pubstyle import use_journal, save, COL2
    use_journal()
    Z = np.load(PV)
    fig, axs = plt.subplots(1, 4, figsize=(COL2, 2.95), sharex=True, sharey=True,
                            gridspec_kw=dict(wspace=0.06, left=0.06, right=0.9,
                                             bottom=0.16, top=0.92))
    sc = None
    for ax, (name, title), tag in zip(axs, SOURCES, "abcd"):
        m = Z["mesh_xy"]
        ax.plot(m[:, 1], m[:, 0], ls="", marker=".", ms=0.5, color="#d9d9d9",
                zorder=0, rasterized=True)
        xy, d = Z[f"{name}_xy"], Z[f"{name}_d"]
        o = np.argsort(d)                       # worst on top
        sc = ax.scatter(xy[o, 1], xy[o, 0], c=np.clip(d[o], 0, 1.0), s=0.5,
                        cmap="viridis", vmin=0, vmax=1.0, linewidths=0,
                        rasterized=True, zorder=2)
        ax.set_aspect("equal")
        ax.set_xlim(-27, 27)
        ax.set_ylim(-34, 46)
        ax.set_title(title, fontsize=7.5, pad=3, linespacing=1.0)
        ax.text(0.04, 0.03, f"RMS {100 * float(Z[f'{name}_rms']):.0f} cm"
                if float(Z[f"{name}_rms"]) < 0.5 else
                f"RMS {float(Z[f'{name}_rms']):.1f} m",
                transform=ax.transAxes, fontsize=7, va="bottom")
        ax.text(0.04, 0.97, f"({tag})", transform=ax.transAxes, fontsize=8.5,
                fontweight="bold", va="top")
        ax.set_xticks([-20, 0, 20])
        if ax is not axs[0]:
            ax.tick_params(labelleft=False)
    axs[0].set_ylabel("north (m)")
    fig.text(0.475, 0.035, "east (m)", ha="center", fontsize=8)
    cax = fig.add_axes([0.915, 0.2, 0.012, 0.64])
    cb = fig.colorbar(sc, cax=cax)
    cb.set_label("distance to CAD (m), clipped at 1 m", fontsize=7)
    cb.ax.tick_params(labelsize=6.5)
    return save(fig, os.path.join(HERE, "figures", "recon_planview"))


def plot_by_source():
    import matplotlib.pyplot as plt
    from pubstyle import use_journal, save, COL1, SERIES
    use_journal()
    R = json.load(open(RES))
    ok = {k: v for k, v in R.items() if "failed" not in v}
    # Recordings that must not count as "dead reckoning under the a50 model":
    #   200040  SLAM was in the loop although the arm reads `dr`
    #   210804  white noise only, before the DVL error model existed
    #   162539  named seed 2 but its measured DVL gains are 1.0000 on all axes
    #           (no systematic error), so its dead reckoning is near perfect
    NOMODEL = "a50_slamref_seed2_20261014_162539"
    CIRC = "a50_dr_seed1_20261012_200040"          # SLAM in the loop, mislabelled
    BASE = "a50_dr_seed0_20261009_210804"          # white noise only, no DVL model
    POST = ("a50_slamned_seed0_20261016_160033", "a50_slamdrift_seed0_20261016_190745")
    cats = {"truth": [], "dr": [], "fused_pre": [], "fused_post": [], "trust": []}
    for key, v in ok.items():
        g, bag = key.split("/", 1)
        s = v["sources"]
        rms = lambda n: s[n]["all"]["rms"] if n in s else None
        cats["truth"].append(rms("truth"))
        if g == "dr_only" and bag not in (CIRC, BASE):
            cats["dr"].append(rms("filtered"))
        if g == "paired" and bag != NOMODEL:
            cats["dr"].append(rms("dr"))
            cats["fused_post" if bag in POST else "fused_pre"].append(rms("filtered"))
        if g == "wrong_face":
            cats["trust"].append(rms("slam_trust"))
            if bag in POST[:1]:
                cats["fused_post"].append(rms("filtered"))
                cats["dr"].append(rms("dr"))
    circ = ok.get("dr_only/" + CIRC)
    n = {k: len([v for v in cats[k] if v is not None]) for k in cats}
    labels = [("truth", f"true poses\n(floor)\nn={n['truth']}"),
              ("dr", f"dead\nreckoning\nn={n['dr']}"),
              ("fused_pre", f"fused,\nbefore fix\nn={n['fused_pre']}"),
              ("fused_post", f"fused,\nafter fix\nn={n['fused_post']}"),
              ("trust", f"SLAM trusted\nthrough closure\nn={n['trust']}")]
    fig, ax = plt.subplots(figsize=(COL1, 2.85))
    rng = np.random.default_rng(1)
    cols = {"truth": "#555555", "dr": SERIES[0], "fused_pre": SERIES[1],
            "fused_post": SERIES[1], "trust": SERIES[1]}
    mks = {"truth": "o", "dr": "o", "fused_pre": "^", "fused_post": "s", "trust": "D"}
    for i, (k, _) in enumerate(labels):
        y = np.array([v for v in cats[k] if v is not None])
        x = i + (rng.random(len(y)) - 0.5) * 0.28
        hollow = k == "fused_pre"
        ax.plot(x, y, ls="", marker=mks[k], ms=4.4, color=cols[k],
                mec=cols[k] if hollow else "white", mew=0.9 if hollow else 0.4,
                mfc="none" if hollow else cols[k], zorder=3)
        if len(y):
            ax.plot([i - 0.28, i + 0.28], [np.median(y)] * 2, color="black",
                    lw=0.9, zorder=4)
    if circ and "filtered" in circ["sources"]:
        ax.plot([2.3], [circ["sources"]["filtered"]["all"]["rms"]], ls="",
                marker="x", ms=5, color="black", mew=1.0, zorder=4,
                label="alignment against its\nown filter (SLAM in the loop)")
        ax.legend(loc="upper left", frameon=False, fontsize=6.3, handletextpad=0.3,
                  borderaxespad=0.3)
    ax.set_yscale("log")
    ax.set_ylim(0.05, 40)
    ax.set_xlim(-0.6, len(labels) - 0.4)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels([l for _, l in labels], fontsize=6.0, linespacing=1.05)
    ax.set_yticks([0.1, 1, 10])
    ax.set_yticklabels(["0.1", "1", "10"])
    ax.set_ylabel("reconstruction RMS error\nagainst CAD (m)")
    ax.tick_params(axis="x", which="both", top=False, bottom=False)
    for f in save(fig, os.path.join(HERE, "figures", "recon_by_source")):
        pass
    return [os.path.join(HERE, "figures", "recon_by_source.pdf")]


if __name__ == "__main__":
    if "--refresh-planview" in sys.argv or not os.path.exists(PV):
        build_planview()
    for f in plot_planview() + plot_by_source():
        print("wrote", os.path.relpath(f, os.path.join(HERE, "..")))
