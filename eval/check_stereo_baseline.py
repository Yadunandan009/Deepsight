#!/usr/bin/env python3
"""Fail if a scenario's stereo camera spacing disagrees with its SLAM yaml.

The baseline is written down in four places that must agree:

  scenario .scn   camera_left / camera_right  y offsets      -> b
  scenario .scn   camera_right  baseline="..."               -> -fx*b
  slam yaml       Stereo.b                                   -> b
  slam yaml       Stereo.T_c1_c2 translation norm            -> b
  slam yaml       RIGHT.P[0,3]                               -> -fx*b

A mismatch does not crash anything. It produces a run whose recorded
geometry is not the geometry that was simulated, which is how the seed-2
reference arm was voided. Check before launching, not after.

Usage:  check_stereo_baseline.py                 # all known arms
        check_stereo_baseline.py <scn> <yaml>    # one explicit pair
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.join(HERE, "..")
SCN = os.path.join(WS, "src", "stonefish_bluerov2", "scenarios")
SLAM = os.path.join(WS, "slam_params")

ARMS = [
    ("reference", os.path.join(SCN, "bluerov2.scn"),
     os.path.join(SLAM, "bluerov2_stereo.yaml")),
    ("wide b=0.30", os.path.join(SCN, "bluerov2_b30.scn"),
     os.path.join(SLAM, "bluerov2_stereo_b30.yaml")),
]
TOL = 1e-3


def from_scn(path):
    txt = open(path).read()
    out = {}
    for side in ("left", "right"):
        m = re.search(r'<sensor name="camera_%s".*?</sensor>' % side,
                      txt, re.S)
        if not m:
            raise SystemExit(f"{path}: no camera_{side}")
        blk = m.group(0)
        o = re.search(r'<origin xyz="([-\d.eE+ ]+)"', blk)
        out[side] = float(o.group(1).split()[1])
        b = re.search(r'baseline="([-\d.eE+]+)"', blk)
        if b:
            out["decl"] = float(b.group(1))
    out["b"] = abs(out["right"] - out["left"])
    return out


def from_yaml(path):
    txt = open(path).read()
    txt = "\n".join(l for l in txt.splitlines()
                    if not l.lstrip().startswith("#"))
    g = lambda p: re.search(p, txt, re.S)
    fx = float(g(r"Camera1\.fx:\s*([-\d.eE+]+)").group(1))
    b = float(g(r"Stereo\.b:\s*([-\d.eE+]+)").group(1))
    t = g(r"Stereo\.T_c1_c2:.*?data:\s*\[(.*?)\]")
    d = [float(v) for v in t.group(1).replace("\n", " ").split(",")]
    tnorm = (d[3] ** 2 + d[7] ** 2 + d[11] ** 2) ** 0.5
    rp = g(r"RIGHT\.P:.*?data:\s*\[(.*?)\]")
    p03 = float(rp.group(1).replace("\n", " ").split(",")[3])
    return fx, b, tnorm, p03


def check(name, scn, yml):
    if not (os.path.exists(scn) and os.path.exists(yml)):
        print(f"  {name}: MISSING ({scn if not os.path.exists(scn) else yml})")
        return False
    s = from_scn(scn)
    fx, b, tnorm, p03 = from_yaml(yml)
    rows = [
        (".scn camera spacing", s["b"], b),
        ("yaml Stereo.b", b, b),
        ("yaml T_c1_c2 norm", tnorm, b),
        ("yaml RIGHT.P[0,3]", -p03 / fx, b),
    ]
    if "decl" in s:
        rows.insert(1, ('.scn baseline="..."', -s["decl"] / fx, b))
    bad = [r for r in rows if abs(r[1] - r[2]) > TOL]
    print(f"  {name}:  b = {b:.4f} m   mThDepth = b*ThDepth = "
          f"{b * 40.0:.2f} m")
    for label, got, want in rows:
        flag = "  <-- MISMATCH" if abs(got - want) > TOL else ""
        print(f"      {label:22s} implies b = {got:.4f}{flag}")
    return not bad


def main():
    if len(sys.argv) == 3:
        arms = [("explicit pair", sys.argv[1], sys.argv[2])]
    else:
        arms = ARMS
    print("stereo baseline consistency\n")
    ok = all([check(*a) for a in arms])
    print()
    if ok:
        print("  all arms self-consistent.")
        return 0
    print("  MISMATCH. Do not launch: the recorded geometry would not be\n"
          "  the simulated geometry.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
