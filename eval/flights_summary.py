#!/usr/bin/env python3
"""Score every flight in the manifest and write the numbers the paper quotes.

    python3 eval/flights_summary.py            # score new flights, reuse cached ones
    python3 eval/flights_summary.py --refresh  # re-read every bag

Input   docs/experiments/flights_manifest.csv   (one row per flight; add a row
                                                  after each flight, nothing else)
Output  eval/figures/flights_summary.json       per-flight numbers
        eval/figures/flights_series.npz         1 Hz error series for the figures
        docs/paper/live_numbers.md              the stage-1 table and test, as text

Per flight:
  * the DVL unit recovered from the bag and its difference from the seed's unit
    (verify_dvl_model.py), so a mislabelled bag is caught here;
  * every loop closure, classified by closure_events.py with the matching SLAM
    console log; duplicate `Loop detected` lines for one event are merged;
  * horizontal error against ground truth of the dead-reckoning (SLAM-free) and
    the primary filter, on a 1 Hz grid;
  * orbit radius about the structure axis (150 to 900 s) and where the vehicle
    ended relative to where it started.

Ground truth is used to score only. Nothing here feeds navigation.

Stage-1 endpoint (docs/experiments/PREREG_symmetry_L0L5_20261021.md): the mission
had at least one face-aliased closure. Unscorable events are reported separately;
the conclusion must hold when they are scored both ways.
"""
import csv
import json
import math
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import closure_events as C   # noqa: E402
import sonar_recon as sr     # noqa: E402

MANIFEST = os.path.join(ROOT, "docs", "experiments", "flights_manifest.csv")
OUT_JSON = os.path.join(HERE, "figures", "flights_summary.json")
OUT_NPZ = os.path.join(HERE, "figures", "flights_series.npz")
OUT_MD = os.path.join(ROOT, "docs", "paper", "live_numbers.md")
TURBINE = np.array([20.0, 0.0])
DEDUP_S = 10.0


def dvl_check(bag, seed):
    """Max element difference between the recovered DVL operator and the seed's."""
    p = subprocess.run([sys.executable, os.path.join(HERE, "verify_dvl_model.py"),
                        bag, str(seed)], capture_output=True, text=True)
    m = re.search(r"max elementwise difference\s+([0-9.]+)", p.stdout)
    return float(m.group(1)) if m else None


def merge_duplicates(rows):
    """Two `Loop detected` lines for one event give two rows at the same time;
    keep the scorable one."""
    rows = sorted(rows, key=lambda r: r["t"])
    out = []
    for r in rows:
        if out and abs(r["t"] - out[-1]["t"]) < DEDUP_S:
            if out[-1]["cls"] == "unscorable" and r["cls"] != "unscorable":
                out[-1] = r
            continue
        out.append(r)
    return out


def score(fl):
    bag = os.path.join(ROOT, fl["bag"])
    tr, _, _ = sr.read_bag(bag)
    tt, dr, fl_ = tr["truth"], tr["dr"], tr["filtered"]
    t0 = tt.t[0]
    lo = max(tt.t[0], dr.t[0], fl_.t[0]) - t0
    T = min(tt.t[-1], dr.t[-1], fl_.t[-1]) - t0
    grid = np.arange(math.ceil(lo), math.floor(T), 1.0)
    P = tt.at(t0 + grid)[0]
    e_dr = np.linalg.norm(dr.at(t0 + grid)[0][:, :2] - P[:, :2], axis=1)
    e_fl = np.linalg.norm(fl_.at(t0 + grid)[0][:, :2] - P[:, :2], axis=1)
    sel = (grid >= 150) & (grid <= 900)
    r = np.hypot(P[sel, 0] - TURBINE[0], P[sel, 1] - TURBINE[1])

    log = C.pick_log(bag)
    rows, n_lines = C.classify(bag, log) if log else ([], 0)
    rows = merge_duplicates(rows)
    ali = [x for x in rows if x["cls"] == "face-aliased"]
    first = min((x["t"] for x in rows if x["cls"] != "benign"), default=None)

    def at(t):
        return float(np.interp(t, grid, e_dr)), float(np.interp(t, grid, e_fl))

    out = dict(
        id=fl["id"], group=fl["group"], arm=fl["arm"], seed=int(fl["seed"]),
        scenario=fl["scenario"], rise=fl["rise"], bag=fl["bag"],
        console_log=os.path.basename(log) if log else None,
        duration_s=float(T),
        dvl_max_diff=dvl_check(bag, fl["seed"]),
        orbit_r_median=float(np.median(r)), orbit_r_p5=float(np.percentile(r, 5)),
        orbit_r_p95=float(np.percentile(r, 95)),
        final_xy=[float(P[-1, 0]), float(P[-1, 1])],
        final_dist_from_start=float(np.hypot(P[-1, 0] - tt.pos[0, 0],
                                             P[-1, 1] - tt.pos[0, 1])),
        series_start_s=float(grid[0]),
        dr_final=float(e_dr[-1]), filt_final=float(e_fl[-1]),
        dr_max=float(e_dr.max()), filt_max=float(e_fl.max()),
        dr_rms=float(np.sqrt((e_dr ** 2).mean())),
        filt_rms=float(np.sqrt((e_fl ** 2).mean())),
        n_loop_lines=n_lines,
        closures=[{k: (float(v) if isinstance(v, (np.floating, float)) else v)
                   for k, v in x.items()} for x in rows],
        n_aliased=len(ali),
        n_other_large=sum(x["cls"] == "other-large" for x in rows),
        n_unscorable=sum(x["cls"] == "unscorable" for x in rows),
        n_benign=sum(x["cls"] == "benign" for x in rows),
        first_aliased_t=min((x["t"] for x in ali), default=None),
        first_nonbenign_t=first,
    )
    if ali:
        t = ali[0]["t"]
        out["filt_err_before_aliased"] = at(t - 5)[1]
        out["filt_err_30s_after_aliased"] = at(t + 30)[1]
        out["dr_err_30s_after_aliased"] = at(t + 30)[0]
        out["filt_err_end_after_aliased"] = float(e_fl[-1])
        out["mission_left_after_aliased_s"] = float(T - t)
    return out, grid, e_dr, e_fl


def fisher_one_sided(a_hit, a_n, b_hit, b_n):
    """P(a arm has >= a_hit aliased | margins), arm a vs arm b, exact."""
    K, N = a_hit + b_hit, a_n + b_n
    den = math.comb(N, a_n)
    return sum(math.comb(K, k) * math.comb(N - K, a_n - k)
               for k in range(a_hit, min(K, a_n) + 1)) / den


def stage1_table(res):
    lines = []
    s1 = [r for r in res if r["group"] == "stage1"]
    L0 = sorted([r for r in s1 if r["arm"] == "ctlL0"], key=lambda r: r["seed"])
    L5 = sorted([r for r in s1 if r["arm"] == "symL5"], key=lambda r: r["seed"])
    lines.append("| flight | arm | seed | any wrong-face closure | first (s) | turn (deg) | "
                 "chord (m) | benign | unscorable | orbit r (m) | filt err end (m) | "
                 "ends from home (m) |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in sorted(s1, key=lambda r: (r["seed"], r["arm"])):
        ali = [c for c in r["closures"] if c["cls"] == "face-aliased"]
        a0 = ali[0] if ali else None
        lines.append("| %s | %s | %d | %s | %s | %s | %s | %d | %d | %.2f | %.2f | %.2f |" % (
            r["id"], r["arm"], r["seed"], "**yes**" if ali else "no",
            "%.0f" % a0["t"] if a0 else "-", "%+.1f" % a0["turn"] if a0 else "-",
            "%.1f" % a0["chord"] if a0 else "-", r["n_benign"], r["n_unscorable"],
            r["orbit_r_median"], r["filt_final"], r["final_dist_from_start"]))
    h0 = sum(1 for r in L0 if r["n_aliased"] > 0)
    h5 = sum(1 for r in L5 if r["n_aliased"] > 0)
    lines.append("")
    lines.append("L0 (stock): %d of %d missions with a wrong-face closure. L5: %d of %d."
                 % (h0, len(L0), h5, len(L5)))
    if L0 and L5:
        p = fisher_one_sided(h0, len(L0), h5, len(L5))
        lines.append("One-sided exact Fisher test, L0 against L5, unit = mission: p = %.3f "
                     "(n = %d vs %d). A non-significant result is reported as not "
                     "detected at this margin and power, never as no effect." %
                     (p, len(L0), len(L5)))
    # paired radius check (void rule: more than 1 m between paired flights)
    for sd in sorted({r["seed"] for r in s1}):
        a = [r for r in L0 if r["seed"] == sd]
        b = [r for r in L5 if r["seed"] == sd]
        if a and b:
            d = abs(a[0]["orbit_r_median"] - b[0]["orbit_r_median"])
            lines.append("Seed %d paired orbit radius: L0 %.2f m, L5 %.2f m, difference %.2f m (%s)." %
                         (sd, a[0]["orbit_r_median"], b[0]["orbit_r_median"], d,
                          "ok" if d <= 1.0 else "VOID by the 1 m rule"))
    pend = [sd for sd in (0, 1, 2) if not any(r["seed"] == sd and r["arm"] == "ctlL0" for r in s1)]
    pend5 = [sd for sd in (0, 1, 2) if not any(r["seed"] == sd and r["arm"] == "symL5" for r in s1)]
    lines.append("Still to fly in stage 1: L0 seeds %s; L5 seeds %s." % (pend or "none", pend5 or "none"))
    return "\n".join(lines)


def main():
    refresh = "--refresh" in sys.argv
    flights = list(csv.DictReader(open(MANIFEST)))
    cache = {}
    if os.path.exists(OUT_JSON) and not refresh:
        cache = {r["id"]: r for r in json.load(open(OUT_JSON))}
    series = dict(np.load(OUT_NPZ, allow_pickle=True)) if (os.path.exists(OUT_NPZ) and not refresh) else {}
    res = []
    for fl in flights:
        bag = os.path.join(ROOT, fl["bag"])
        if not os.path.isdir(bag):
            print("skip (no bag):", fl["id"])
            continue
        if fl["id"] in cache and cache[fl["id"]]["bag"] == fl["bag"] and f"{fl['id']}_grid" in series:
            res.append(cache[fl["id"]])
            continue
        print("scoring", fl["id"], flush=True)
        out, g, edr, efl = score(fl)
        res.append(out)
        series[f"{fl['id']}_grid"], series[f"{fl['id']}_dr"], series[f"{fl['id']}_filt"] = g, edr, efl
        json.dump(res, open(OUT_JSON, "w"), indent=1)
        np.savez(OUT_NPZ, **series)
    json.dump(res, open(OUT_JSON, "w"), indent=1)
    np.savez(OUT_NPZ, **series)

    md = ["# Live numbers (generated by eval/flights_summary.py; do not edit by hand)", "",
          "## Stage 1: stock (L0) against every-face-different (L5)", "", stage1_table(res), "",
          "## Every scored flight", "",
          "| id | group | arm | seed | DVL diff | duration (s) | orbit r (m) | closures (class @ s) | "
          "DR rms (m) | filt rms (m) | filt end (m) | ends from home (m) |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in res:
        cl = ", ".join("%s@%.0f" % (c["cls"], c["t"]) for c in r["closures"]) or "none"
        md.append("| %s | %s | %s | %d | %s | %.0f | %.2f | %s | %.2f | %.2f | %.2f | %.2f |" % (
            r["id"], r["group"], r["arm"], r["seed"],
            "%.4f" % r["dvl_max_diff"] if r["dvl_max_diff"] is not None else "?",
            r["duration_s"], r["orbit_r_median"], cl, r["dr_rms"], r["filt_rms"],
            r["filt_final"], r["final_dist_from_start"]))
    open(OUT_MD, "w").write("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
