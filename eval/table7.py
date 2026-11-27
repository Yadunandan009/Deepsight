#!/usr/bin/env python3
"""Table 7 of the manuscript, generated: reconstruction RMS against the jacket
CAD for the bridge-B stock flights and the one earlier-bridge flight.

Reads eval/figures/recon_results.json (group "sim2", from recon_batch.py) and
eval/figures/flights_summary.json. Dead reckoning takes z from the depth
sensor (sonar_recon.py, DR DEPTH): before 2026-11-27 it did not, and that
error alone produced the old table's 19-50 % "gain" from fusing SLAM.

Writes docs/paper/table7.md.   Run:  python3 eval/table7.py
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
R = json.load(open(os.path.join(HERE, "figures", "recon_results.json")))
M = {r["id"]: r for r in json.load(open(os.path.join(HERE, "figures", "flights_summary.json")))}
OUT = os.path.join(HERE, "..", "docs", "paper", "table7.md")
ROWS = [("slamref_s0", "stock seed 0"), ("slamref_s1", "stock seed 1"), ("slamref_s2", "stock seed 2"),
        ("L0_s0", "L0 seed 0 (fixed RISE)"), ("L0_s1", "L0 seed 1 (fixed RISE)"),
        ("L0_s2", "L0 seed 2 (fixed RISE; no wrong-face closure)"),
        ("OB_s1", "stock seed 1, earlier bridge (version A)")]


def fmt(v):
    if v is None:
        return "n/a"
    return f"{v:.2f} m" if v >= 1.0 else f"{100 * v:.1f} cm"


def rms(r, src, part="all"):
    s = r["sources"].get(src, {}).get(part)
    return s["rms"] if s else None


def main():
    L = ["| flight | DVL seed | ground-truth poses | dead reckoning | primary filter (fused) | "
         "fused vs dead reckoning | fused, after closure | trust-SLAM, after closure |",
         "|---|---|---|---|---|---|---|---|"]
    for fid, label in ROWS:
        r = R.get("sim2/" + os.path.basename(M[fid]["bag"]))
        if not r or "sources" not in r:
            L.append(f"| {label} | {M[fid]['seed']} | not scored |  |  |  |  |  |")
            continue
        d, f = rms(r, "dr"), rms(r, "filtered")
        rel = f"{100 * (f - d) / d:+.0f} %" if d and f else "n/a"
        L.append(f"| {label} | {M[fid]['seed']} | {fmt(rms(r, 'truth'))} | {fmt(d)} | {fmt(f)} | {rel} | "
                 f"{fmt(rms(r, 'filtered', 'after_closure'))} | {fmt(rms(r, 'slam_trust', 'after_closure'))} |")
    txt = "\n".join(L) + "\n"
    open(OUT, "w").write(txt)
    print(txt)


if __name__ == "__main__":
    main()
