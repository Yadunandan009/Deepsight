#!/usr/bin/env python3
"""Run N trials back-to-back, appending each result to a JSONL log."""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from run_trial import run_trial

p = argparse.ArgumentParser()
p.add_argument("--n", type=int, default=8)
p.add_argument("--prefix", default="trial")
p.add_argument("--bag-root", default=os.path.expanduser("~/ros2_ws/eval/bags"))
p.add_argument("--timeout", type=int, default=1800)
p.add_argument("--scenario", default="bluerov2_turbine")
p.add_argument("--log", default=os.path.expanduser("~/ros2_ws/eval/results.jsonl"))
args = p.parse_args()

os.makedirs(args.bag_root, exist_ok=True)

for i in range(args.n):
    name = f"{args.prefix}_{i:02d}"
    print(f"=== starting {name} ({i+1}/{args.n}) ===", flush=True)
    result = run_trial(name, args.bag_root, args.timeout, args.scenario)
    with open(args.log, "a") as f:
        f.write(json.dumps(result) + "\n")
    print(f"=== {name} done: {result['outcome']} ({result['duration_s']:.0f}s) ===", flush=True)

print("BATCH DONE", flush=True)
