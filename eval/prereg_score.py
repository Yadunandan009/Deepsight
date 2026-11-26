#!/usr/bin/env python3
"""Score the heading-veto / symmetry pre-registration
(docs/experiments/PREREG_veto_symmetry_20261116.md).

    python3 eval/prereg_score.py              # score the 24 flights, reuse cache
    python3 eval/prereg_score.py --refresh    # re-read every bag
    python3 eval/prereg_score.py --one BAG    # one bag, no arms, no tests (dry run)

Flights come from the PREREG results log: every line that ends its void
check with PASS, minus every bag a CORRECTION line declares VOID.

Primary endpoint, per mission: at least one ACCEPTED wrong-face correction
before RETURN_HOME (and before a collision, if any).
  accepted loop      a `*Loop detected` line. A vetoed loop is reset before
                     that line is printed (LoopClosing.cc), so every such
                     line is an accepted loop.
  accepted merge     a `*Merge detected` line, same reason.
  accepted reloc     a [HEADING] kind=reloc line whose decision is not
                     `veto` (Tracking.cc: a vetoed candidate is dropped and
                     the next one is tried; any other decision returns true).
  wrong-face         closure_events.classify on the pose jump, unchanged
                     rule (face-aliased). Unscorable events are scored both
                     ways; other-large and benign are not wrong-face.

Implementation choices the PREREG did not fix, made before any of the 24
outcomes was read, and logged in the PREREG file under that date:
  * merges are located like loops (marker `*Merge detected`, release
    `Merge finished!`); relocs by their [HEADING] t_a, as [t_a, t_a].
    All accepted events of a mission go through ONE classify call in time
    order, so the pre-fit clipping at an earlier event applies across kinds.
    Rows within DEDUP_S of each other are one event (flights_summary).
  * RETURN_HOME start, from truth: the first time after the deepest point
    from which the vehicle never again goes deeper than RH_DEPTH. The
    controller switches RISE -> RETURN_HOME at 3.0 m.
  * collision: the first multibeam scan whose shortest valid return is
    below COLLISION_RANGE. The mission's shortest return is reported, so
    the margin to the threshold is visible.
  * mission end = MISSION COMPLETE, from truth: the first time after
    RETURN_HOME starts from which the horizontal speed stays below
    HOLD_SPEED for HOLD_S (COMPLETE holds position while it surfaces).
    For the flights whose PREREG line gives the exact time ("MISSION
    COMPLETE x s before the recorder stopped") the difference is reported.
  * truth class of a heading DECISION (Q1 event level; a refused match has
    no pose jump to classify): the true positions at the two keyframe times
    t_a and t_m, with closure_events' constants -- chord < BENIGN_CHORD is
    same-place; radius kept and turn ~90/180 is wrong-face; else other.

Ground truth is used to score only.
"""
import json
import math
import os
import re
import sys

try:
    import mcap_ros2  # noqa: F401
except ImportError:
    _alt = os.path.expanduser("~/venv-ardupilot/bin/python3")
    if not os.environ.get("_BAGREADER_REEXEC") and os.path.exists(_alt) \
            and os.path.abspath(_alt) != os.path.abspath(sys.executable):
        os.environ["_BAGREADER_REEXEC"] = "1"
        os.execv(_alt, [_alt] + sys.argv)
    sys.exit("mcap_ros2 not found in any available interpreter")

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import closure_events as ce        # noqa: E402
import heading_log as hl           # noqa: E402
import sonar_recon as sr           # noqa: E402
from flights_summary import fisher_one_sided, merge_duplicates  # noqa: E402

PREREG = os.path.join(ROOT, "docs", "experiments", "PREREG_veto_symmetry_20261116.md")
OUT_JSON = os.path.join(HERE, "figures", "prereg_score.json")
OUT_MD = os.path.join(ROOT, "docs", "paper", "prereg_results.md")

RH_DEPTH = 3.05          # m    RISE hands over to RETURN_HOME at 3.0 m
COLLISION_RANGE = 0.5    # m    shortest multibeam return that counts as contact
NEAR_CUTOFF_S = 10.0     # s    events this close to the cutoff are flagged
HOLD_SPEED, HOLD_S = 0.03, 5.0   # m/s, s   COMPLETE: holding still
ARMS = ("stkL0", "vetoL0", "stkL5")
TESTS = (("Q1", "stkL0", "vetoL0"), ("Q2", "stkL0", "stkL5"))

LINE = re.compile(r"block (\d+), (\w+), seed (\d+), `(a50_\w+)` \(([^)]*)\).*?"
                  r"console `(orb_slam3_console_\w+\.log)`.*?Void check: PASS")
EXACT = re.compile(r"MISSION COMPLETE ([\d.]+) s before the recorder stopped")


# ------------------------------------------------------------------ flights
def flights():
    txt = open(PREREG).read()
    void = set()
    for ln in txt.splitlines():
        if "CORRECTION" in ln and "VOID" in ln:
            void.update(re.findall(r"`(a50_\w+)`", ln))
    out = []
    for m in LINE.finditer(txt):
        blk, arm, seed, bag, span, log = m.groups()
        if bag in void:
            continue
        x = EXACT.search(span)
        out.append(dict(block=int(blk), arm=arm, seed=int(seed), bag=bag, log=log,
                        complete_before_stop=float(x.group(1)) if x else None))
    return out


def bag_span(bag):
    meta = open(os.path.join(bag, "metadata.yaml")).read()
    t_start = int(re.search(r"nanoseconds_since_epoch:\s*(\d+)", meta).group(1)) / 1e9
    dur = int(re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", meta).group(1)) / 1e9
    return t_start, t_start + dur


# ------------------------------------------------------------------ geometry
def polar(xy):
    d = np.asarray(xy) - np.array(ce.TURBINE)
    return math.hypot(*d), math.degrees(math.atan2(d[1], d[0]))


def match_class(pa, pm):
    """Truth class of a match between keyframes at true positions pa, pm."""
    chord = float(np.hypot(*(np.asarray(pa) - np.asarray(pm))))
    ra, ba = polar(pa)
    rm, bm = polar(pm)
    turn = ce.wrap_deg(ba - bm)
    dr = (ra - rm) / rm
    if chord < ce.BENIGN_CHORD:
        cls = "same-place"
    else:
        m = min((90.0, 180.0), key=lambda v: abs(abs(turn) - v))
        cls = ("wrong-face" if abs(dr) <= ce.RADIUS_TOL and abs(abs(turn) - m) <= ce.TURN_TOL
               else "other")
    return dict(match_cls=cls, match_chord=chord, match_turn=turn, match_dr=dr)


def return_home_start(truth):
    d = np.abs(truth.pos[:, 2])
    deepest = int(np.argmax(d))
    suffix_max = np.maximum.accumulate(d[::-1])[::-1]
    ok = np.flatnonzero((np.arange(len(d)) > deepest) & (suffix_max < RH_DEPTH))
    return float(truth.t[ok[0]]) if len(ok) else None


def complete_start(truth, t_rh):
    if t_rh is None:
        return None
    g = np.arange(t_rh, truth.t[-1], 1.0)
    if len(g) < 2:
        return None
    P = truth.at(g)[0][:, :2]
    still = np.hypot(*np.diff(P, axis=0).T) < HOLD_SPEED
    n = int(HOLD_S)
    for i in range(len(still) - n + 1):
        if still[i:i + n].all():
            return float(g[i])
    return None


def sonar_minimum(scans):
    t_first, best = None, (math.inf, None)
    for t, rg, _, _, rmin, rmax in scans:
        ok = np.isfinite(rg) & (rg > rmin) & (rg < rmax)
        if not ok.any():
            continue
        lo = float(rg[ok].min())
        if lo < best[0]:
            best = (lo, t)
        if t_first is None and lo < COLLISION_RANGE:
            t_first = t
    return t_first, best[0], best[1]


# ------------------------------------------------------------------ one bag
def score_bag(bag, log, t_complete_exact=None):
    tracks, _, scans = sr.read_bag(bag)
    truth, dr, filt = tracks["truth"], tracks["dr"], tracks["filtered"]
    t0 = truth.t[0]
    t_rh = return_home_start(truth)
    t_complete = complete_start(truth, t_rh)
    t_coll, son_min, son_min_t = sonar_minimum(scans)
    cutoff = min(x for x in (t_rh, t_coll, truth.t[-1]) if x is not None)

    dec = hl.decisions(log)
    loops = ce.log_events(log)
    merges = ce.log_events(log, "*Merge detected", "Merge finished!")
    relocs = [(d["t_a"], d["t_a"]) for d in dec if d["kind"] == "reloc" and d["decision"] != "veto"]
    n_relocalized = sum("Relocalized!!" in ln for ln in open(log, errors="replace"))
    ev = sorted([(lo, hi, "loop") for lo, hi in loops] + [(lo, hi, "merge") for lo, hi in merges] +
                [(lo, hi, "reloc") for lo, hi in relocs], key=lambda e: e[0] if e[0] is not None else 0.0)
    rows = []
    if ev:
        cls_rows, _ = ce.classify(bag, log, [(lo, hi) for lo, hi, _ in ev])
        for (lo, hi, kind), r in zip(ev, cls_rows):
            r = dict(r, kind=kind)
            tj = t0 + r["t"]
            if kind == "reloc":
                hit = [d for d in dec if d["kind"] == "reloc" and d["decision"] != "veto"
                       and abs(d["t_a"] - lo) < 1e-6]
            else:
                hit = [d for d in dec if d["kind"] == kind and hl.PAIR[0] <= d["t_a"] - tj <= hl.PAIR[1]]
            r["decision"] = [(h["decision"], h["resid"]) for h in hit]
            r["before_cutoff"] = bool(tj < cutoff)
            r["near_cutoff"] = bool(abs(tj - cutoff) < NEAR_CUTOFF_S)
            rows.append({k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in r.items()})
    rows = merge_duplicates(rows)

    pre = [r for r in rows if r["before_cutoff"]]
    hit_strict = any(r["cls"] == "face-aliased" for r in pre)
    hit_loose = hit_strict or any(r["cls"] == "unscorable" for r in pre)

    decisions = []
    for d in dec:
        pa, pm = truth.at(np.array([d["t_a"]]))[0][0, :2], truth.at(np.array([d["t_m"]]))[0][0, :2]
        inside = bool(truth.t[0] <= d["t_m"] <= d["t_a"] <= truth.t[-1])
        row = dict(kind=d["kind"], t=d["t_a"] - t0, age=d["t_a"] - d["t_m"], resid=d["resid"],
                   decision=d["decision"], before_cutoff=bool(d["t_a"] < cutoff), in_bag=inside)
        if inside:
            row.update(match_class(pa, pm))
        decisions.append({k: (float(v) if isinstance(v, (np.floating, float)) else v)
                          for k, v in row.items()})

    t_end = t_complete if t_complete is not None else truth.t[-1]
    grid = np.arange(math.ceil(max(truth.t[0], dr.t[0], filt.t[0])), math.floor(t_end), 1.0)
    P = truth.at(grid)[0][:, :2]
    e_dr = np.linalg.norm(dr.at(grid)[0][:, :2] - P, axis=1)
    e_fl = np.linalg.norm(filt.at(grid)[0][:, :2] - P, axis=1)
    return dict(
        t_return_home=None if t_rh is None else t_rh - t0,
        t_collision=None if t_coll is None else t_coll - t0,
        sonar_min=son_min, sonar_min_t=None if son_min_t is None else son_min_t - t0,
        cutoff=cutoff - t0, t_complete=t_end - t0,
        t_complete_exact=None if t_complete_exact is None else t_complete_exact - t0,
        n_loop_lines=len(loops), n_merge_lines=len(merges), n_reloc_accepted=len(relocs),
        n_relocalized_lines=n_relocalized,
        events=rows, decisions=decisions,
        hit_strict=hit_strict, hit_loose=hit_loose,
        filt_median=float(np.median(e_fl)), dr_median=float(np.median(e_dr)),
        filt_end=float(e_fl[-1]), dr_end=float(e_dr[-1]),
        end_from_home=float(np.hypot(*(P[-1] - truth.pos[0, :2]))),
    )


# ------------------------------------------------------------------ tests
def holm(ps):
    order = sorted(range(len(ps)), key=lambda i: ps[i])
    adj, run = [None] * len(ps), 0.0
    for rank, i in enumerate(order):
        run = max(run, min(1.0, (len(ps) - rank) * ps[i]))
        adj[i] = run
    return adj


def tests(res):
    out = {}
    for way in ("strict", "loose"):
        key = "hit_" + way
        ps, rows = [], []
        for name, a, b in TESTS:
            A = [r for r in res if r["arm"] == a]
            B = [r for r in res if r["arm"] == b]
            ha, hb = sum(r[key] for r in A), sum(r[key] for r in B)
            p = fisher_one_sided(ha, len(A), hb, len(B))
            ps.append(p)
            rows.append(dict(test=name, a=a, b=b, a_hit=ha, a_n=len(A), b_hit=hb, b_n=len(B), p=p))
        for r, h in zip(rows, holm(ps)):
            r["p_holm"] = h
        out[way] = rows
    return out


def markdown(res, T):
    L = ["# Pre-registered results: heading veto and symmetry stage 2",
         "", "Generated by `eval/prereg_score.py`; do not edit by hand.", "",
         "## Primary endpoint per mission", "",
         "| arm | seed | bag | accepted events before cutoff (kind:class@s) | wrong-face | "
         "unscorable | cutoff (s) | shortest sonar (m) |", "|---|---|---|---|---|---|---|---|"]
    for r in sorted(res, key=lambda r: (ARMS.index(r["arm"]), r["seed"])):
        pre = [e for e in r["events"] if e["before_cutoff"]]
        ev = ", ".join(f"{e['kind']}:{e['cls']}@{e['t']:.0f}" for e in pre) or "none"
        L.append(f"| {r['arm']} | {r['seed']} | `{r['bag']}` | {ev} | "
                 f"{'**yes**' if r['hit_strict'] else 'no'} | "
                 f"{sum(e['cls'] == 'unscorable' for e in pre)} | {r['cutoff']:.0f} | {r['sonar_min']:.2f} |")
    L += ["", "## Tests (one-sided Fisher exact, mission as unit, alpha 0.05 each; Holm alongside)", ""]
    for way, label in (("strict", "unscorable scored as NOT wrong-face"),
                       ("loose", "unscorable scored as wrong-face")):
        L.append(f"**{label}**")
        L.append("")
        L.append("| test | arms | hits | p | p (Holm) |")
        L.append("|---|---|---|---|---|")
        for t in T[way]:
            L.append(f"| {t['test']} | {t['a']} vs {t['b']} | {t['a_hit']}/{t['a_n']} vs "
                     f"{t['b_hit']}/{t['b_n']} | {t['p']:.3f} | {t['p_holm']:.3f} |")
        L.append("")
    for name, _, _ in TESTS:
        s = next(t for t in T["strict"] if t["test"] == name)
        lo = next(t for t in T["loose"] if t["test"] == name)
        if s["p"] < 0.05 and lo["p"] < 0.05:
            verdict = "significant both ways"
        elif s["p"] >= 0.05 and lo["p"] >= 0.05:
            verdict = "not detected at this sample size (both ways)"
        else:
            verdict = "UNRESOLVED: the two scorings disagree"
        L.append(f"- {name}: {verdict}.")
    L += ["", "## Q1 event level: every heading decision, truth class from the two keyframe times", "",
          "| arm | seed | kind | t (s) | age (s) | resid (deg) | decision | truth class | chord (m) | "
          "turn (deg) | before cutoff |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(res, key=lambda r: (ARMS.index(r["arm"]), r["seed"])):
        for d in r["decisions"]:
            L.append(f"| {r['arm']} | {r['seed']} | {d['kind']} | {d['t']:.0f} | {d['age']:.0f} | "
                     f"{d['resid']:+.1f} | {d['decision']} | {d.get('match_cls', 'outside bag')} | "
                     f"{d.get('match_chord', float('nan')):.1f} | {d.get('match_turn', float('nan')):+.1f} | "
                     f"{'yes' if d['before_cutoff'] else 'no'} |")
    L += ["", "## Secondary per mission (horizontal error against truth, to MISSION COMPLETE)", "",
          "| arm | seed | fused median (m) | DR median (m) | fused end (m) | DR end (m) | fused beats DR "
          "(median / end) | collision | ends from home (m) |", "|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(res, key=lambda r: (ARMS.index(r["arm"]), r["seed"])):
        L.append(f"| {r['arm']} | {r['seed']} | {r['filt_median']:.2f} | {r['dr_median']:.2f} | "
                 f"{r['filt_end']:.2f} | {r['dr_end']:.2f} | "
                 f"{'yes' if r['filt_median'] < r['dr_median'] else 'no'} / "
                 f"{'yes' if r['filt_end'] < r['dr_end'] else 'no'} | "
                 f"{'at %.0f s' % r['t_collision'] if r['t_collision'] is not None else 'none'} | "
                 f"{r['end_from_home']:.2f} |")
    ex = [(r, r["t_complete"] - r["t_complete_exact"]) for r in res if r.get("t_complete_exact") is not None]
    if ex:
        L += ["", "Mission end from truth against the controller's MISSION COMPLETE (flights where the "
              "PREREG line gives it): " + ", ".join(f"{r['arm']} s{r['seed']} {d:+.1f} s" for r, d in ex) + "."]
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ main
def main():
    if "--one" in sys.argv:
        bag = os.path.abspath(sys.argv[sys.argv.index("--one") + 1])
        log = ce.pick_log(bag)
        print(json.dumps(dict(bag=os.path.basename(bag), log=os.path.basename(log) if log else None,
                              **score_bag(bag, log)), indent=1))
        return
    refresh = "--refresh" in sys.argv
    fl = flights()
    counts = {a: sum(f["arm"] == a for f in fl) for a in ARMS}
    print("flights:", len(fl), counts)
    cache = {}
    if os.path.exists(OUT_JSON) and not refresh:
        cache = {r["bag"]: r for r in json.load(open(OUT_JSON))["flights"]}
    res = []
    for f in fl:
        if f["bag"] in cache:
            res.append(cache[f["bag"]])
            continue
        bag = os.path.join(ROOT, "bags", f["bag"])
        log = os.path.join(ROOT, "slam_atlas", f["log"])
        _, t_stop = bag_span(bag)
        exact = None if f["complete_before_stop"] is None else t_stop - f["complete_before_stop"]
        print("scoring", f["bag"], flush=True)
        res.append(dict(f, **score_bag(bag, log, t_complete_exact=exact)))
        json.dump(dict(flights=res), open(OUT_JSON, "w"), indent=1)
    T = tests(res)
    json.dump(dict(flights=res, tests=T), open(OUT_JSON, "w"), indent=1)
    md = markdown(res, T)
    open(OUT_MD, "w").write(md)
    print(md)


if __name__ == "__main__":
    main()
