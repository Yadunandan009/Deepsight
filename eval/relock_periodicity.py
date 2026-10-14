#!/usr/bin/env python3
"""Does the 90-degree re-lock signature (R3) survive on OPEN-LOOP data?

R3 was measured on 2026-10-07, before the observability defect was found. Those
runs fitted the SLAM->world rotation against the filter that SLAM was driving,
so the re-lock angles they produced came from an apparatus later shown to be
unfalsifiable. The three dr_reference runs are the first open-loop locks, and
they can be tested for the same signature at no extra cost.

n-fold periodicity in theta is a Rayleigh concentration in n*theta: if theta
clusters near multiples of 360/n degrees, n*theta clusters at one angle.
"""
import math, sys
import numpy as np

rows = [l.split() for l in open(sys.argv[1]) if l.strip()]
th = np.radians(np.array([float(r[-2]) for r in rows]))
chk = np.array([float(r[-1]) for r in rows])
th = th[chk < 15.0]                     # same reconstruction filter
n = len(th)
print(f"  {n} open-loop locks, three seeds, baseline (L0) structure\n")
print(f"  theta at lock: {', '.join(f'{math.degrees(a):+.0f}' for a in sorted(th))}\n")
print("  fold   R     p(Rayleigh)   meaning")
for k in (1, 2, 3, 4, 5, 6):
    a = k * th
    R = abs(complex(np.cos(a).mean(), np.sin(a).mean()))
    z = n * R * R
    # Rayleigh p with the standard small-sample correction
    p = math.exp(-z) * (1 + (2 * z - z * z) / (4 * n)
                        - (24 * z - 132 * z**2 + 76 * z**3 - 9 * z**4) / (288 * n * n))
    p = min(max(p, 0.0), 1.0)
    note = {4: "   <-- the 90-degree signature R3 claims"}.get(k, "")
    print(f"   {k}-fold  {R:.3f}   p = {p:.3f}{note}")
print("\n  R is the resultant length of k*theta: 0 is uniform, 1 is perfectly")
print("  clustered. A 90-degree signature needs 4-fold to stand out from the")
print("  others, not merely to be non-zero.")
