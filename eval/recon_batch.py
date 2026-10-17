#!/usr/bin/env python3
"""Run sonar_recon over every recording that feeds the inspection-product
comparison and cache the numbers in eval/figures/recon_results.json.

Three groups, each answering a different question:
  dr_only    SLAM-free runs, three DVL error seeds: how good is the model that
             dead reckoning alone builds?
  paired     one run, both filters: what does fusing SLAM do to the model?
  wrong_face missions with a wrong-face closure: what would a vehicle that
             trusted SLAM through it have built?

Run:  python3 eval/recon_batch.py            (skips what is already cached)
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sonar_recon as sr                                  # noqa: E402

OUT = os.path.join(HERE, "figures", "recon_results.json")
BAGS = os.path.join(HERE, "..", "bags")
GROUPS = {
    "dr_only": ["a50_dr_seed0_20261009_210804", "a50_dr_seed0_20261009_221008",
                "a50_dr_seed1_20261012_173121", "a50_dr_seed1_20261012_200040",
                "a50_dr_seed2_20261012_191733"],
    "paired": ["a50_slamref12_seed0_20261015_180532",
               "a50_slamref_seed0_20261014_153610",
               "a50_slamref_seed1_20261014_140856",
               "a50_slamref_seed2_20261014_162539",
               "a50_slamref_seed2_20261014_200359",
               "a50_slamb30_seed0_20261015_204105",
               "a50_slamdrift_seed0_20261016_190745"],
    # closure time in seconds into the run, from eval/closure_events.py
    "wrong_face": {"a50_slamned_seed0_20261016_160033": 573.0,
                   "DeepSight_20260905_201108": 776.0,
                   "DeepSight_20260921_232316": 642.0,
                   "DeepSight_20260922_000133": 561.0,
                   "DeepSight_20260922_230945": 525.0},
}


def main():
    cache = json.load(open(OUT)) if os.path.exists(OUT) else {}
    jobs = [(g, b, None) for g in ("dr_only", "paired") for b in GROUPS[g]]
    jobs += [("wrong_face", b, t) for b, t in GROUPS["wrong_face"].items()]
    for g, b, tc in jobs:
        key = f"{g}/{b}"
        if key in cache:
            continue
        t = time.time()
        print(f"[{g}] {b} ...", flush=True)
        try:
            r = sr.score(os.path.join(BAGS, b), tc)
        except Exception as e:                      # keep going; report it
            print(f"   FAILED: {type(e).__name__}: {e}", flush=True)
            cache[key] = {"failed": f"{type(e).__name__}: {e}"}
        else:
            cache[key] = {k: v for k, v in r.items() if k != "pts"}
            cache[key]["group"], cache[key]["closure_t"] = g, tc
            s = r["sources"]
            print("   rms: " + "  ".join(f"{k} {v['all']['rms']:.3f}"
                                         for k, v in s.items()) +
                  f"   ({time.time() - t:.0f}s)", flush=True)
        json.dump(cache, open(OUT, "w"), indent=1)


if __name__ == "__main__":
    main()
