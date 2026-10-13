#!/usr/bin/env python3
"""Fail if ekf.yaml and ekf_dr.yaml have drifted apart.

ekf.yaml's header refuses to keep a second near-identical config on the
grounds that two of them drift. That is the right worry and the wrong
conclusion: the dead-reckoning filter has to exist, because the SLAM
alignment cannot be estimated against the filter it feeds. What the worry
actually demands is that divergence be loud rather than forbidden.

The two filters are a controlled comparison only while every shared
parameter matches. This checks that, with an explicit allowlist of the
differences that are deliberate -- which doubles as the documentation of
what those differences are.

Exit status 0 if they agree, 1 if they do not.

Usage:  check_ekf_configs.py [--base ekf.yaml] [--dr ekf_dr.yaml]
"""
import argparse
import os
import sys

import yaml

# Parameters that are MEANT to differ, with why. Anything not named here
# must be identical in both files.
ALLOWED = {
    "publish_tf":
        "two filters publishing world_ned->base_link would fight; the "
        "primary owns the transform tree",
    "print_diagnostics":
        "halves the console noise of running two filters",
}
# Prefixes that exist only in the primary filter.
ONLY_IN_BASE = ("pose0",)

BASE_NODE = "ekf_filter_node"
DR_NODE = "ekf_dr_node"


def params(path, node):
    with open(path) as fh:
        doc = yaml.safe_load(fh)
    if node not in doc:
        sys.exit(f"{path}: no '{node}' block")
    return doc[node]["ros__parameters"]


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.join(here, "..", "src", "stonefish_bluerov2")
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=os.path.join(pkg, "ekf.yaml"))
    ap.add_argument("--dr", default=os.path.join(pkg, "ekf_dr.yaml"))
    args = ap.parse_args()

    base = params(args.base, BASE_NODE)
    dr = params(args.dr, DR_NODE)

    problems = []

    for k in sorted(set(base) | set(dr)):
        if k.startswith(ONLY_IN_BASE):
            if k in dr:
                problems.append(
                    f"{k}: present in ekf_dr.yaml. The dead-reckoning filter "
                    f"must not fuse SLAM -- that is the entire reason it "
                    f"exists.")
            continue
        if k in ALLOWED:
            continue
        if k not in base:
            problems.append(f"{k}: only in ekf_dr.yaml ({dr[k]!r})")
        elif k not in dr:
            problems.append(f"{k}: only in ekf.yaml ({base[k]!r})")
        elif base[k] != dr[k]:
            problems.append(f"{k}: ekf.yaml {base[k]!r} != ekf_dr.yaml {dr[k]!r}")

    if dr.get("publish_tf", True):
        problems.append(
            "publish_tf: must be false in ekf_dr.yaml, or it will fight the "
            "primary filter for world_ned->bluerov2/base_link")

    if problems:
        print(f"ekf.yaml and ekf_dr.yaml have DRIFTED ({len(problems)}):\n")
        for p in problems:
            print(f"  - {p}")
        print("\nFix them, or add the parameter to ALLOWED in this file with "
              "a reason. Comparisons between runs using the two filters are "
              "not controlled until this passes.")
        return 1

    print(f"ekf.yaml ({len(base)} params) and ekf_dr.yaml ({len(dr)} params) "
          f"agree.")
    print("  intended differences:")
    for k, why in ALLOWED.items():
        print(f"    {k}: {base.get(k)!r} -> {dr.get(k)!r}  ({why})")
    for p in ONLY_IN_BASE:
        n = sum(1 for k in base if k.startswith(p))
        print(f"    {p}*: {n} parameters in ekf.yaml, absent from ekf_dr.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
