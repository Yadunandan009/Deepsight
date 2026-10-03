#!/usr/bin/env python3
"""
Measure how self-similar an inspection structure's faces are, from CAD alone.

WHY THIS EXISTS. TASK_15 established, from sonar-derived geometry, that the
turbine's four faces register against each other at ICP fitness 0.996-1.000
-- they are close to indistinguishable, which is why appearance-based place
recognition falsely closes loops on them. That was a single observation on a
single structure, which is a problem statement, not a result.

Turning it into a result means showing that failure rate *scales* with
symmetry: build scenario variants that progressively break the four-fold
symmetry, and measure place-recognition failure against a symmetry index.
For that to be practical the index has to be computable WITHOUT running a
mission per variant -- otherwise every point on the dose-response curve
costs ~18 minutes of simulation plus a sonar reconstruction.

This computes it straight from the mesh. A structure designer or inspection
planner has the CAD; if the index predicts failure, it is an a-priori design
guideline rather than a post-hoc explanation.

METHOD, and why it is not simply "compare vertices":
  1. Parse the scenario's static meshes and place them in world coordinates.
  2. Area-weighted sample points over triangles. Vertex positions alone are
     a density measure, not a shape measure -- this structure's four legs
     carry 120/96/160/96 vertices purely from how it was tessellated, which
     would read as asymmetry where there is none.
  3. Keep only the band the vehicle actually inspects (depth and radius),
     since geometry it never observes cannot confuse its place recognition.
  4. Cut an angular sector per face, de-rotate each into a shared face-local
     frame, and ICP every face against every other.
  5. Report mean cross-face fitness as the symmetry index: 1.0 means the
     faces are interchangeable, lower means a place recogniser has something
     to work with.

Validation: run on the stock scenario, the index should land near the
0.996-1.000 that TASK_15 measured independently from sonar. Agreement
between a CAD-derived and a sonar-derived estimate is what licenses using
the cheap one for the study.

Usage:
  python3 eval/structure_symmetry.py                       # stock scenario
  python3 eval/structure_symmetry.py --scn path/to/variant.scn
  python3 eval/structure_symmetry.py --samples 40000 --seed 1
"""
import argparse
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

WS = os.path.expanduser('~/ros2_ws/src/stonefish_bluerov2')
DEFAULT_SCN = os.path.join(WS, 'scenarios/bluerov2_turbine.scn')
DATA = os.path.join(WS, 'data')

# Mission geometry, taken from bluerov2_autonomous_controller.py so the
# analysed band matches what the vehicle actually sees.
TURBINE_X, TURBINE_Y = 20.0, 0.0
FACE_BEARINGS = [180.0, 90.0, 0.0, 270.0]
DEPTH_MIN, DEPTH_MAX = 1.0, 22.0        # D_TOP .. D_BASE
SECTOR_HALF_WIDTH_DEG = 35.0            # matches the TASK_15 face extraction
ICP_INLIER_M = 1.0

# Radial cut-off, and it decides WHICH QUESTION is being asked:
#
#   STRUCTURE symmetry (default, 15 m): the jacket spans r = 8.0-12.8 m, so
#   this keeps the structure and nothing else. Answers "are the faces of this
#   structure interchangeable?" and is what TASK_15 measured from sonar
#   (its reference was frustum-restricted to the turbine surface).
#
#   SCENE symmetry (e.g. 30 m): also admits background. In the stock
#   scenario that pulls the Rustpipe (r=19 m, bearing 180 deg) into face 0's
#   sector and nothing into the others, so f0 reads as distinguishable
#   (fitness 0.17) while f1/f2/f3 stay interchangeable (1.000). That is a
#   real effect, not an artifact -- a vehicle inspecting face 0 does see the
#   pipe -- and it is precisely the lever the dose-response study uses:
#   placing distinct objects near faces is the cheapest way to break
#   symmetry without remodelling the structure.
#
# Report which radius was used alongside any index; the two are not
# comparable to each other.
MAX_RADIUS_M = 15.0


def load_obj(path):
    """Return (vertices, triangles). Only 'v' and 'f' are needed."""
    V, F = [], []
    with open(path, errors='replace') as fh:
        for line in fh:
            if line.startswith('v '):
                V.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('f '):
                idx = [int(tok.split('/')[0]) for tok in line.split()[1:]]
                idx = [i - 1 if i > 0 else len(V) + i for i in idx]
                for k in range(1, len(idx) - 1):      # fan-triangulate
                    F.append([idx[0], idx[k], idx[k + 1]])
    return np.asarray(V, dtype=np.float64), np.asarray(F, dtype=np.int64)


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _triple(text, default=(0.0, 0.0, 0.0)):
    if not text:
        return np.array(default)
    parts = [p for p in re.split(r'[\s,]+', text.strip()) if p]
    try:
        return np.array([float(p) for p in parts[:3]])
    except ValueError:
        return np.array(default)


def parse_scenario(scn_path):
    """Collect (mesh_path, scale, origin_rpy, world_xyz, world_rpy) for every
    static model, following one level of <include> with arg substitution.
    That covers how these scenarios are actually written; anything more
    exotic would need the real Stonefish parser."""
    entries = []

    def handle(root, args):
        def sub(s):
            if not s:
                return s
            return re.sub(r'\$\(arg\s+(\w+)\)', lambda m: str(args.get(m.group(1), m.group(0))), s)

        for st in list(root.findall('static')) + list(root.findall('dynamic')):
            mesh = st.find('./physical/mesh')
            if mesh is None:
                continue
            fn = sub(mesh.get('filename', ''))
            try:
                scale = float(sub(mesh.get('scale', '1.0')))
            except ValueError:
                scale = 1.0
            org = st.find('./physical/origin')
            wt = st.find('world_transform')
            entries.append({
                'mesh': os.path.join(DATA, fn),
                'scale': scale,
                'origin_xyz': _triple(sub(org.get('xyz')) if org is not None else None),
                'origin_rpy': _triple(sub(org.get('rpy')) if org is not None else None),
                'world_xyz': _triple(sub(wt.get('xyz')) if wt is not None else None),
                'world_rpy': _triple(sub(wt.get('rpy')) if wt is not None else None),
                'name': st.get('name', '?'),
            })

        for inc in root.findall('include'):
            f = sub(inc.get('file', ''))
            f = re.sub(r'\$\(find stonefish_bluerov2\)', WS, f)
            if not os.path.exists(f):
                continue
            sub_args = {a.get('name'): sub(a.get('value')) for a in inc.findall('arg')}
            handle(ET.parse(f).getroot(), sub_args)

    handle(ET.parse(scn_path).getroot(), {})
    return entries


def sample_surface(V, F, n, rng):
    """Area-weighted point sampling over triangles -- density-independent."""
    if len(F) == 0 or len(V) == 0:
        return np.zeros((0, 3))
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    tot = areas.sum()
    if tot <= 0:
        return np.zeros((0, 3))
    pick = rng.choice(len(F), size=n, p=areas / tot)
    u = rng.random(n)[:, None]
    v = rng.random(n)[:, None]
    over = (u + v) > 1
    u[over] = 1 - u[over]
    v[over] = 1 - v[over]
    return a[pick] + u * (b[pick] - a[pick]) + v * (c[pick] - a[pick])


def build_cloud(entries, n_samples, rng):
    """Sample all meshes with a GLOBALLY area-weighted budget.

    Giving each mesh an equal share instead would reintroduce exactly the
    density-vs-geometry error this module exists to avoid, one level up:
    the six small anodes would collectively receive six times the budget of
    the entire jacket, and the resulting cloud would describe the anode
    placement rather than the structure. Observed directly before this was
    fixed -- face sectors came out 2589/75/74/78 points.
    """
    placed = []
    for e in entries:
        if not os.path.exists(e['mesh']):
            continue
        V, F = load_obj(e['mesh'])
        if len(V) == 0 or len(F) == 0:
            continue
        V = V * e['scale']
        V = V @ rpy_matrix(*e['origin_rpy']).T + e['origin_xyz']
        V = V @ rpy_matrix(*e['world_rpy']).T + e['world_xyz']

        # Discard triangles outside the inspected band BEFORE weighting.
        # Weighting over whole meshes and filtering afterwards fails badly
        # here: the seabed is ~1e6 m^2 against the jacket's ~5.6e3, so it
        # would absorb 98% of the budget and then be thrown away entirely,
        # leaving ~70 usable points. Only observable geometry can influence
        # place recognition, so only observable geometry is sampled.
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        cen = (a + b + c) / 3.0
        rad = np.hypot(cen[:, 0] - TURBINE_X, cen[:, 1] - TURBINE_Y)
        keep = (cen[:, 2] > DEPTH_MIN) & (cen[:, 2] < DEPTH_MAX) & (rad < MAX_RADIUS_M)
        if not keep.any():
            continue
        F = F[keep]
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        area = float(0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum())
        if area <= 0:
            continue
        placed.append((V, F, area, e['name']))

    total = sum(p[2] for p in placed)
    if total <= 0:
        return np.zeros((0, 3)), []

    clouds, breakdown = [], []
    for V, F, area, name in placed:
        n = int(round(n_samples * area / total))
        if n < 1:
            breakdown.append((name, area, 0))
            continue
        pts = sample_surface(V, F, n, rng)
        breakdown.append((name, area, len(pts)))
        if len(pts):
            clouds.append(pts)
    return (np.vstack(clouds) if clouds else np.zeros((0, 3))), breakdown


def icp(src, dst, max_iters=60, tol=1e-6):
    if len(src) < 10 or len(dst) < 10:
        return 0.0, float('inf')
    P = src.copy()
    tree = cKDTree(dst)
    prev = None
    for _ in range(max_iters):
        d, idx = tree.query(P, k=1)
        Q = dst[idx]
        pc, qc = P.mean(0), Q.mean(0)
        H = (P - pc).T @ (Q - qc)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        P = (P - pc) @ R.T + qc
        rmse = float(np.sqrt(np.mean(np.sum((P - Q) ** 2, axis=1))))
        if prev is not None and abs(prev - rmse) < tol:
            break
        prev = rmse
    d, _ = tree.query(P, k=1)
    fit = float(np.mean(d < ICP_INLIER_M))
    inl = float(np.sqrt(np.mean(d[d < ICP_INLIER_M] ** 2))) if fit > 0 else float('inf')
    return fit, inl


def face_local(pts, face_idx):
    fb = math.radians(FACE_BEARINGS[face_idx])
    rel = pts[:, :2] - np.array([TURBINE_X, TURBINE_Y])
    c, s = math.cos(-fb), math.sin(-fb)
    return np.column_stack([c * rel[:, 0] - s * rel[:, 1],
                            s * rel[:, 0] + c * rel[:, 1],
                            pts[:, 2]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scn', default=DEFAULT_SCN)
    ap.add_argument('--samples', type=int, default=30000)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--inlier', type=float, default=ICP_INLIER_M,
                    help='ICP inlier threshold in m; 1.0 matches TASK_A/15, smaller resolves finer features')
    ap.add_argument('--max-radius', type=float, default=MAX_RADIUS_M,
                    help='radial cut-off; structure-only ~15 m, scene-with-background ~30 m')
    args = ap.parse_args()
    globals()['MAX_RADIUS_M'] = args.max_radius
    globals()['ICP_INLIER_M'] = args.inlier
    rng = np.random.default_rng(args.seed)

    entries = parse_scenario(args.scn)
    print('=' * 70)
    print(f'STRUCTURE SYMMETRY INDEX — {os.path.basename(args.scn)}')
    print('=' * 70)
    print(f'  static meshes found: {len(entries)}')
    for e in entries:
        tag = '' if os.path.exists(e['mesh']) else '   [MISSING]'
        print(f'    {e["name"]:<16s} {os.path.basename(e["mesh"]):<24s}{tag}')

    cloud, breakdown = build_cloud(entries, args.samples, rng)
    if len(cloud) == 0:
        print('\n  No geometry sampled — check mesh paths.')
        return
    print('\n  area-weighted sampling budget (in-band geometry only):')
    for name, area, n in sorted(breakdown, key=lambda b: -b[1])[:8]:
        print(f'    {name:<16s} area {area:10.1f} m^2 -> {n:6d} pts')

    r = np.hypot(cloud[:, 0] - TURBINE_X, cloud[:, 1] - TURBINE_Y)
    band = cloud[(cloud[:, 2] > DEPTH_MIN) & (cloud[:, 2] < DEPTH_MAX) & (r < MAX_RADIUS_M)]
    print(f'\n  sampled {len(cloud)} pts; {len(band)} inside the inspected band '
          f'(depth {DEPTH_MIN}-{DEPTH_MAX} m, r < {args.max_radius} m)')
    if len(band) < 200:
        print('  Too little geometry in the inspected band to compare.')
        return

    bearing = np.degrees(np.arctan2(band[:, 1] - TURBINE_Y, band[:, 0] - TURBINE_X)) % 360
    faces = {}
    for i, fb in enumerate(FACE_BEARINGS):
        d = np.abs((bearing - fb + 180) % 360 - 180)
        faces[i] = face_local(band[d < SECTOR_HALF_WIDTH_DEG], i)
    print('  points per face sector: ' + ', '.join(f'f{i}={len(faces[i])}' for i in range(4)))

    print(f'\n  cross-face ICP (fitness @ {ICP_INLIER_M:.1f} m inlier threshold)')
    print(f"    {'pair':>10s} {'fitness':>9s} {'RMSE (m)':>10s}")
    fits = []
    for i in range(4):
        for j in range(i + 1, 4):
            if len(faces[i]) < 50 or len(faces[j]) < 50:
                continue
            f, e = icp(faces[i], faces[j])
            fits.append(f)
            print(f'    f{i} vs f{j} {f:9.3f} {e:10.3f}')

    if fits:
        idx = float(np.mean(fits))
        print(f'\n  SYMMETRY INDEX (mean cross-face fitness) = {idx:.3f}')
        print(f'    spread: {min(fits):.3f} to {max(fits):.3f}')
        print('\n  Reference: TASK_15 measured 0.996-1.000 on this structure from')
        print('  sonar-derived geometry, entirely independently. A CAD-derived')
        print('  index in that range means the cheap a-priori measure agrees with')
        print('  the expensive observed one, and can stand in for it across variants.')
        if idx > 0.95:
            print('\n  => Faces are effectively interchangeable. A place recogniser')
            print('     has almost nothing to distinguish them by.')
        elif idx > 0.8:
            print('\n  => Substantially self-similar, with some distinguishing geometry.')
        else:
            print('\n  => Faces are geometrically distinguishable.')
    print('=' * 70)


if __name__ == '__main__':
    main()
