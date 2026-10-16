#!/usr/bin/env python3
"""
Manuscript figures for the loop-closure section (docs/paper/section_closure_events.md).

  Figure 1  `closures_classes`   (double column)
    (a) PLAN VIEW. The structure at the centre, the 17 m scan orbit as a
        ring, north up. Each wrong-face closure is drawn as what it is: an arc
        along the orbit, from where the estimate was to where the closure put
        it. Benign closures are points on the orbit near the start, where the
        vehicle returns to a place it had been. The geometry is the argument:
        every arc is a rotation about the structure, and all end on the east
        side.
    (b) Size of the correction against the turn it makes about the axis. The
        classification itself: two clusters, separated completely.

  Figure 2  `closures_cost`      (single column, two stacked panels, shared time)
    (a) Bearing error about the structure's axis, SLAM against dead
        reckoning. SLAM sits on zero until the closure and then on +88 deg.
    (b) Horizontal position error, log axis, 10 s trailing RMS.
    The SLAM-to-world alignment is fitted on the poses BEFORE the closure and
    held fixed, so before the closure the SLAM curves are the fit's own
    residual, and the step is out of sample.

FORM CHOICE. A twelve-point scatter on its own has little to say; the plan
view carries the same twelve events in the geometry a reader can check by eye.
Fig. 2 is two time series on one clock because the point is a step at one
instant seen in two quantities.

Ground truth scores; nothing here feeds navigation.

Run:  python3 eval/fig_closures.py                (plot from the cache)
      python3 eval/fig_closures.py --refresh      (re-read every bag)
      python3 eval/fig_closures.py --refresh-cost (re-read only the Fig. 2 bag)
"""
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
CACHE = os.path.join(HERE, "figures", "closures_data.json")

BAGS = [
    "DeepSight_20260905_141654", "DeepSight_20260905_173822",
    "DeepSight_20260905_201108", "DeepSight_20260912_132725",
    "DeepSight_20260921_232316", "DeepSight_20260922_000133",
    "DeepSight_20260922_230945", "a50_slamref_seed0_20261014_153610",
    "a50_slamref_seed2_20261014_162539", "a50_slamned_seed0_20261016_160033",
    "a50_slamdrift_seed0_20261016_190745",
]
COST_BAG = "a50_slamned_seed0_20261016_160033"
MEDIA = "/media/yadunandan/more space/bags"
TX, TY = 20.0, 0.0           # structure axis, world frame (x north, y east)


def bag_path(name):
    p = os.path.join(MEDIA, name)
    return p if os.path.isdir(p) else os.path.join(HERE, "..", "bags", name)


# ------------------------------------------------------------------ extract
def extract_events():
    import closure_events as ce          # needs mcap_ros2
    events = []
    for b in BAGS:
        p = bag_path(b)
        rows, _ = ce.classify(p, ce.pick_log(p))
        for r in rows:
            if r["cls"] == "unscorable":
                continue
            events.append(dict(bag=b, **{k: r[k] for k in (
                "t", "cls", "chord", "turn", "dr", "r_before",
                "bearing_before", "bearing_after")}))
    return events


def extract_cost(events):
    import numpy as np
    import closure_events as ce
    p = bag_path(COST_BAG)
    log = ce.pick_log(p)
    sl = ce.track(p, "/bluerov2/robot_pose_slam", True)
    tr = ce.track(p, "/bluerov2/odometry", False)
    dr = ce.track(p, "/bluerov2/odometry/dr", False)
    t0 = tr[0, 0]
    ev = next(e for e in events if e["bag"] == COST_BAG and e["cls"] == "face-aliased")
    tj = t0 + ev["t"]

    def truth_at(t):
        return np.column_stack([np.interp(t, tr[:, 0], tr[:, 1]),
                                np.interp(t, tr[:, 0], tr[:, 2])])

    pre = sl[:, 0] < tj
    to_w, th, k, resid = ce.fit_similarity(sl[pre, 1:3], truth_at(sl[pre, 0]))
    P = np.array([to_w(s) for s in sl[:, 1:3]])
    Ts, Td = truth_at(sl[:, 0]), truth_at(dr[:, 0])
    e_slam = np.hypot(*(P - Ts).T)
    e_dr = np.hypot(*(dr[:, 1:3] - Td).T)
    _, _, _, resid_all = ce.fit_similarity(sl[:, 1:3], truth_at(sl[:, 0]))

    def bearing(xy):
        return np.degrees(np.arctan2(xy[:, 1] - TY, xy[:, 0] - TX))
    wrap = lambda a: (a + 180.0) % 360.0 - 180.0
    b_slam = wrap(bearing(P) - bearing(Ts))
    b_dr = wrap(bearing(dr[:, 1:3]) - bearing(Td))

    ts_re, last, maps = re.compile(r"\[(\d{10}\.\d+)\]"), None, []
    for ln in open(log, errors="replace"):
        m = ts_re.search(ln)
        if m:
            last = float(m.group(1))
        if "New Map created" in ln and last is not None:
            maps.append(last - t0)

    def thin(t, *cols, dt=0.5):
        keep, nxt = [], -1e18
        for i, x in enumerate(t):
            if x >= nxt:
                keep.append(i)
                nxt = x + dt
        return [[float(c[i]) for i in keep] for c in (t,) + cols]

    ts, es, bs = thin(sl[:, 0] - t0, e_slam, b_slam)
    td, ed, bd = thin(dr[:, 0] - t0, e_dr, b_dr)
    return dict(t_slam=ts, e_slam=es, b_slam=bs, t_dr=td, e_dr=ed, b_dr=bd,
                t_closure=float(ev["t"]), new_maps=maps[1:],
                n_prefit=int(pre.sum()), prefit_resid=float(resid),
                prefit_scale=float(k), wholerun_resid=float(resid_all))


# --------------------------------------------------------------------- plot
def plot(D):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.ticker import FixedLocator
    from pubstyle import use_journal, save, COL1, COL2, SERIES, MARKERS

    use_journal()
    BLUE, ORANGE, GREY = SERIES[0], SERIES[1], "#555555"
    ev = D["events"]
    pol = lambda r, b: (r * math.sin(math.radians(b)),      # east
                        r * math.cos(math.radians(b)))      # north

    # ================================================== Figure 1
    fig = plt.figure(figsize=(COL2, 2.9))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.12], wspace=0.24,
                          left=0.065, right=0.995, bottom=0.14, top=0.97)
    a = fig.add_subplot(gs[0])
    b = fig.add_subplot(gs[1])

    # (a) plan view, north up
    R = 17.0
    th = np.linspace(0, 2 * math.pi, 361)
    a.plot(R * np.sin(th), R * np.cos(th), color=GREY, lw=0.5,
           ls=(0, (3, 3)), zorder=1)
    a.add_patch(plt.Rectangle((-2.6, -2.6), 5.2, 5.2, fc="black", ec="black",
                              zorder=2))
    a.text(0, -5.2, "structure", ha="center", va="top", fontsize=6.5)
    a.plot([0], [-20.0], marker="^", ms=4.2, color="black", ls="", zorder=3)
    a.text(1.8, -20.2, "start", fontsize=6.5, va="center")
    a.annotate("", xy=(21.5, 20.5), xytext=(21.5, 14.5),
               arrowprops=dict(arrowstyle="-|>", lw=0.6, color="black",
                               mutation_scale=7))
    a.text(21.5, 21.3, "N", ha="center", va="bottom", fontsize=7)

    def arc(e, scale, color, ls, lw, alpha, z):
        bb, tn = e["bearing_before"], e["turn"]
        ang = bb + np.linspace(0.0, tn, 90)
        rr = e["r_before"] * scale
        xy = np.array([pol(rr, x) for x in ang])
        a.plot(xy[:, 0], xy[:, 1], color=color, ls=ls, lw=lw, alpha=alpha,
               zorder=z, solid_capstyle="butt")
        a.annotate("", xy=xy[-1], xytext=xy[-4],
                   arrowprops=dict(arrowstyle="-|>", lw=0, color=color,
                                   mutation_scale=7, alpha=alpha), zorder=z)

    for e in ev:
        if e["cls"] == "face-aliased":
            half = abs(abs(e["turn"]) - 180.0) < 30.0
            arc(e, 1.13 if half else 1.0, ORANGE, "-", 1.5, 0.9, 4)
            x0, y0 = pol(e["r_before"] * (1.13 if half else 1.0),
                         e["bearing_before"])
            a.plot([x0], [y0], marker="s", ms=3.4, mfc="white", mec=ORANGE,
                   mew=0.9, ls="", zorder=5)
        elif e["cls"] == "other-large":
            arc(e, 0.86, GREY, (0, (1, 1.4)), 1.0, 1.0, 3)
    for e in ev:
        if e["cls"] == "benign":
            x, y = pol(e["r_before"], e["bearing_before"])
            a.plot([x], [y], marker="o", ms=3.4, color=BLUE, mec="white",
                   mew=0.4, ls="", zorder=6)
    a.text(*pol(22.6, 45.0), "+88°", fontsize=7, color=ORANGE, ha="center",
           va="center")
    a.text(*pol(22.6, -45.0), "+178°", fontsize=7, color=ORANGE, ha="center",
           va="center")
    a.set_xlim(-24, 24)
    a.set_ylim(-24, 24)
    a.set_aspect("equal")
    a.set_xticks([-20, -10, 0, 10, 20])
    a.set_yticks([-20, -10, 0, 10, 20])
    a.set_xlabel("east of structure axis (m)")
    a.set_ylabel("north of structure axis (m)")
    a.text(0.03, 0.965, "(a)", transform=a.transAxes, va="top", fontsize=8.5,
           fontweight="bold")

    # (b) correction size against turn
    for lo, hi in ((75, 105), (-105, -75), (165, 195), (-195, -165)):
        b.axhspan(lo, hi, color="#e4e4e4", lw=0, zorder=0)
    b.axvline(3.0, color="black", lw=0.5, ls=(0, (4, 3)), zorder=1)
    STY = {"benign": ("o", BLUE, BLUE, "benign"),
           "face-aliased": ("s", ORANGE, ORANGE, "wrong-face"),
           "other-large": ("D", GREY, "none", "other large")}
    for cls, (mk, col, mfc, lab) in STY.items():
        pts = [e for e in ev if e["cls"] == cls]
        b.plot([e["chord"] for e in pts], [e["turn"] for e in pts], ls="",
               marker=mk, color=col, mfc=mfc,
               mec=col if mfc == "none" else "white",
               mew=0.9 if mfc == "none" else 0.4, ms=4.4, zorder=3, label=lab)
    b.set_xscale("log")
    b.set_xlim(0.5, 60)
    b.set_ylim(-195, 195)
    b.set_yticks([-180, -90, 0, 90, 180])
    b.xaxis.set_major_locator(FixedLocator([1, 10]))
    b.set_xticklabels(["1", "10"])
    b.set_xlabel("size of the correction (m)")
    b.set_ylabel("turn about the structure's axis (°)")
    b.text(2.8, -192, "3 m", fontsize=6.5, ha="right", va="bottom")
    b.text(0.58, 125, "symmetry windows ±15°", fontsize=6.5, va="center")
    b.text(27.5, 66, "four\nclosures", fontsize=6.5, ha="center", va="top")
    b.legend(loc="center left", bbox_to_anchor=(0.42, 0.30), frameon=False,
             handletextpad=0.3, labelspacing=0.45, borderaxespad=0)
    b.text(0.03, 0.965, "(b)", transform=b.transAxes, va="top", fontsize=8.5,
           fontweight="bold")
    out1 = save(fig, os.path.join(HERE, "figures", "closures_classes"))

    # ================================================== Figure 2
    C = D["cost"]

    def rolling_rms(x, n=20):
        sq = np.cumsum(np.concatenate([[0.0], np.square(x)]))
        o = np.empty(len(x))
        for i in range(len(x)):
            lo = max(0, i + 1 - n)
            o[i] = math.sqrt((sq[i + 1] - sq[lo]) / (i + 1 - lo))
        return o

    fig, (p1, p2) = plt.subplots(2, 1, figsize=(COL1, 4.1), sharex=True,
                                 gridspec_kw=dict(hspace=0.07,
                                                  height_ratios=[1, 1.15]))
    tc = C["t_closure"]
    xmax = max(C["t_slam"][-1], C["t_dr"][-1]) + 30
    def break_at_wrap(t, ang):
        """An angle is only meaningful modulo 360, so keep it in (-180, 180]
        and BREAK the line where it wraps. Joining the two sides draws a
        vertical stroke that is not in the data; unwrapping instead
        manufactures a trend (the error reads as 400 deg) that is not in it
        either."""
        t, ang = np.asarray(t, float), np.asarray(ang, float)
        k = np.flatnonzero(np.abs(np.diff(ang)) > 180.0) + 1
        return np.insert(t, k, np.nan), np.insert(ang, k, np.nan)

    p1.plot(C["t_dr"], C["b_dr"], color=BLUE, ls="--", lw=0.9,
            label="dead reckoning", zorder=2)
    p1.plot(*break_at_wrap(C["t_slam"], C["b_slam"]), color=ORANGE, ls="-",
            lw=0.9, label="SLAM, alignment held", zorder=3)
    p1.axvline(tc, color="black", lw=0.5, zorder=1)
    p1.axhline(0, color="black", lw=0.3, zorder=0)
    p1.set_ylabel("bearing error about\nstructure axis (°)")
    p1.set_ylim(-190, 190)
    p1.set_yticks([-180, -90, 0, 90, 180])
    p1.legend(loc="upper left", frameon=False, handlelength=1.6,
              borderaxespad=0.3, bbox_to_anchor=(0.0, 1.0))
    p1.text(tc - 12, -112, "loop closure\n(+88°)", fontsize=6.5, ha="right",
            va="center")
    p1.text(0.97, 0.06, "(a)", transform=p1.transAxes, ha="right", va="bottom",
            fontsize=8.5, fontweight="bold")

    p2.plot(C["t_dr"], rolling_rms(np.array(C["e_dr"])), color=BLUE, ls="--",
            lw=0.9, zorder=2)
    p2.plot(C["t_slam"], rolling_rms(np.array(C["e_slam"])), color=ORANGE,
            ls="-", lw=0.9, zorder=3)
    p2.axvline(tc, color="black", lw=0.5, zorder=1)
    for i, t in enumerate(C["new_maps"]):
        p2.plot([t], [0.0075], marker="v", ms=3.6, color="black", mec="white",
                mew=0.3, ls="", zorder=4,
                label="map re-initialised" if i == 0 else None)
    p2.set_yscale("log")
    p2.set_ylim(0.005, 150)
    p2.set_yticks([0.01, 0.1, 1, 10])
    p2.set_yticklabels(["0.01", "0.1", "1", "10"])
    p2.set_ylabel("horizontal position error\n(10 s RMS, m)")
    p2.set_xlabel("time into the mission (s)")
    p2.set_xlim(0, xmax)
    p2.legend(loc="upper left", frameon=False, handlelength=1.0,
              borderaxespad=0.3, bbox_to_anchor=(0.0, 0.97))
    p2.text(0.97, 0.95, "(b)", transform=p2.transAxes, ha="right",
            va="top", fontsize=8.5, fontweight="bold")
    out2 = save(fig, os.path.join(HERE, "figures", "closures_cost"))
    return out1 + out2


if __name__ == "__main__":
    D = None
    if "--refresh" in sys.argv or not os.path.exists(CACHE):
        ev = extract_events()
        D = dict(events=ev, cost=extract_cost(ev))
    elif "--refresh-cost" in sys.argv:
        D = json.load(open(CACHE))
        D["cost"] = extract_cost(D["events"])
    if D is not None:
        json.dump(D, open(CACHE, "w"))
        c = D["cost"]
        print(f"cached {len(D['events'])} events; cost mission: "
              f"{c['n_prefit']} pre-closure poses, fit residual "
              f"{c['prefit_resid']:.2f} m, scale {c['prefit_scale']:.3f}, "
              f"whole-run residual {c['wholerun_resid']:.1f} m, "
              f"map re-inits at {[round(x) for x in c['new_maps']]} s")
    else:
        D = json.load(open(CACHE))
    for f in plot(D):
        print("wrote", os.path.relpath(f))
