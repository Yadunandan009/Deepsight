#!/usr/bin/env python3
"""The adaptive consistency gate, replayed inside the real bridge.

The test registered in docs/experiments/PREREG_adaptive_gate_offline.md. Every bag
holding the raw SLAM pose and the SLAM-free reference (/bluerov2/odometry/dr) is fed
through SlamPoseBridge (eval/replay_bridge.py) once per candidate gate, with
jump_policy:=quarantine and slam_frame:=enu. With that reference the bridge's inputs
do not depend on its output, so what it publishes here is what it would have
published live. Each published pose is scored against ground truth: BAD if more
than 1 m from it, GOOD otherwise.

Run:  source ROS, then  python3 eval/gate_offline.py        (v1; eval/figures/gate_offline.json)
                        python3 eval/gate_offline.py --v2   (docs/experiments/PREREG_adaptive_gate_v2.md;
                                                             eval/figures/gate_offline_v2.json)
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "src", "stonefish_bluerov2"))  # consistency_gate
sys.path.insert(0, HERE)
import replay_bridge as rb  # noqa: E402

T_BAD = 1.0
T_FAR = 2.0                       # v2: metres wrong
# v2: the bags flown with the SLAM settings in use now (Camera.fps 12, baseline 0.145 m,
# per each run's ORB-SLAM3 console log). Fixed in PREREG_adaptive_gate_v2.md.
CURRENT = {
    "a50_slamref12_seed0_20261015_180532", "a50_slamned_seed0_20261016_160033",
    "a50_slamdrift_seed0_20261016_190745", "a50_slamref_seed0_20261020_164933",
    "a50_slamref_seed1_20261020_172618", "a50_slamref_seed2_20261020_200356",
    "a50_symL4_seed0_20261021_010852", "a50_symL4_seed1_20261021_172653",
    "a50_risefix_seed0_20261021_205753", "a50_symL5_seed0_20261021_215607",
    "a50_ctlL0_seed1_20261021_234743", "a50_symL5_seed1_20261022_124631",
    "a50_ctlL0_seed2_20261022_175232", "a50_symL5_seed2_20261022_190833",
    "a50_oldbridge_seed1_20261022_200605", "a50_imgL0_seed1_20261025_132041",
    "a50_qL0_seed1_20261026_194426",
}
PROB = {"gate_probation_poses": 25, "gate_probation_spread": 8.0}
NOPROB = {"gate_probation_poses": 0, "gate_probation_spread": 0.0}
CANDIDATES = {
    "base": None,
    "A": dict(adaptive_gate="true", gate_floor=1.0, gate_k_drift=0.0, **NOPROB),
    "B": dict(adaptive_gate="true", gate_floor=1.0, gate_k_drift=0.0, **PROB),
    "C": dict(adaptive_gate="true", gate_floor=1.5, gate_k_drift=0.0, **PROB),
    "D": dict(adaptive_gate="true", gate_floor=1.0, gate_k_drift=0.003, **PROB),
}


def bags():
    out = []
    for b in sorted(glob.glob(os.path.join(rb.BAG_ROOT, "*"))):
        m = os.path.join(b, "metadata.yaml")
        if os.path.exists(m):
            txt = open(m).read()
            if "name: /bluerov2/odometry/dr\n" in txt and "name: /bluerov2/robot_pose_slam\n" in txt:
                out.append(b)
    return out


def main():
    v2 = "--v2" in sys.argv
    res = {k: dict(bad=0, good=0, far=0, worst={}, per_bag={}) for k in CANDIDATES}
    seen = set()
    for bag in bags():
        name = os.path.basename(bag)
        seen.add(name)
        ref, ev, g, _ = rb.load(bag)
        assert ref == "/bluerov2/odometry/dr"
        line = [name + ("" if not v2 or name in CURRENT else " [excluded]")]
        for k, params in CANDIDATES.items():
            pub, _ = rb.replay(ev, "enu", "quarantine", params)
            e = rb.err(pub, g)
            nb, nf = int((e > T_BAD).sum()), int((e > T_FAR).sum())
            res[k]["per_bag"][name] = dict(published=len(e), bad=nb, far=nf,
                                           worst=float(e.max()) if len(e) else None)
            if not v2 or name in CURRENT:
                res[k]["bad"] += nb
                res[k]["good"] += int(len(e) - nb)
                res[k]["far"] += nf
                res[k]["worst"][name] = float(e.max()) if len(e) else None
            line.append(f"{k}: {len(e)} pub, {nb} bad, {nf} >2m, worst {e.max() if len(e) else float('nan'):.2f}")
        print(" | ".join(line), flush=True)
    if v2:
        missing = CURRENT - seen
        assert not missing, f"registered bags not found: {missing}"
    b = res["base"]
    print()
    for k, r in res.items():
        fb = r["bad"] / b["bad"] if b["bad"] else float("nan")
        fg = r["good"] / b["good"] if b["good"] else float("nan")
        if v2:
            passes = k != "base" and r["far"] == 0 and fg >= 0.97
        else:
            passes = k != "base" and fb <= 0.05 and fg >= 0.95
        r.update(bad_frac_of_base=fb, good_frac_of_base=fg, passes=passes)
        w = [v for v in r["worst"].values() if v is not None]
        print(f"{k:5s} >2m {r['far']:6d}  BAD {r['bad']:6d} ({100*fb:5.1f}% of base)  GOOD {r['good']:7d} "
              f"({100*fg:5.1f}% of base)  worst {max(w):.2f} m  {'PASS' if passes else ''}")
    out = "gate_offline_v2.json" if v2 else "gate_offline.json"
    json.dump(res, open(os.path.join(HERE, "figures", out), "w"), indent=1)


if __name__ == "__main__":
    main()
