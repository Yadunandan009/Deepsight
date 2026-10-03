#!/usr/bin/env python3
"""
Generate the scenario variant ladder for the symmetry dose-response study.

THE INDEPENDENT VARIABLE. TASK_15 showed, and eval/structure_symmetry.py
confirms from CAD, that the stock turbine's four faces are interchangeable
(cross-face ICP fitness 1.000, RMSE ~0.11 m). To turn that observation into
a result we need failure rate as a function of symmetry, which needs
structures whose symmetry we control.

HOW SYMMETRY IS BROKEN. Not by remodelling the jacket -- that would change
many things at once and make any effect unattributable. Instead each face
gets a DIFFERENT distinguishing object, drawn from meshes the scenario
already loads (oil drum, gas tank, gas canister, rust pipe), at a
controllable scale. Scale is the single knob:

  scale 0  -> no markers, stock structure, index 1.000
  scale s  -> markers of increasing prominence, index falls

Note that marking all four faces with the SAME object would leave the
structure symmetric. It is the per-face DIFFERENCE that breaks it, which is
why each face gets its own object type.

WHY SCALE AND NOT COUNT. Varying how many faces are marked (0,1,2,3,4)
gives only five coarse steps and saturates: once three faces are marked
every face pair already differs, so the index stops moving while the
underlying distinguishability keeps changing. Scale is continuous and
monotone, and reads physically as "how prominent are the distinguishing
features" -- which is the actionable form for a design guideline.

The default scales were chosen empirically, by generating and scoring,
because the stock meshes are small (oil drum 0.69 x 1.16 m) against a face
sector of a few hundred square metres and barely move the index at scale 1.

Markers sit at radius MARKER_RADIUS_M on each face bearing, just outside
the jacket surface (which spans r = 8.0-12.8 m) so they are visible rather
than embedded, and inside structure_symmetry.py's 15 m analysis radius so
they register in the index.

Usage:
  python3 eval/make_variants.py --scales 0 2 4 6 8
  python3 eval/make_variants.py --scales 4 --faces 2      # mark 2 of 4
"""
import argparse
import os
import re

WS = os.path.expanduser('~/ros2_ws/src/stonefish_bluerov2')
SCN_DIR = os.path.join(WS, 'scenarios')
INSTALL_SCN = os.path.expanduser(
    '~/ros2_ws/install/stonefish_bluerov2/share/stonefish_bluerov2/scenarios')
STOCK = os.path.join(SCN_DIR, 'bluerov2_turbine.scn')

TURBINE_X, TURBINE_Y, TURBINE_Z = 20.0, 0.0, 25.0
FACE_BEARINGS = [180.0, 90.0, 0.0, 270.0]

# Just outside the jacket surface (r = 8.0-12.8 m), inside the 15 m analysis
# radius structure_symmetry.py uses for structure-only measurement.
MARKER_RADIUS_M = 13.0

# FIXED, PLAUSIBLE SIZE. The first version of this ladder used scale as the
# knob and produced physically absurd structures: at scale 6 the oil drum
# was 4.2 x 6.9 x 4.2 m (a seven-metre barrel, three of them stacked up each
# leg), the gas canister 15.2 m long, and the rust pipe 22.5 m tall --
# breaching the sea surface by 5.3 m. Visually confirmed in the simulator
# before it was caught by measurement. A reviewer would discard the whole
# dose-response on the grounds that the symmetry breaking does not
# correspond to anything that exists offshore, and they would be right.
#
# At scale 2 the same meshes are ordinary subsea hardware: a 1.4 x 2.3 m
# drum, a 2.0 x 2.5 m tank, a 1.1 x 5.1 m caisson, a 7.5 m vertical pipe
# section. Symmetry is now broken by HOW MANY distinguishing features a
# face carries, not by inflating a few to implausible size -- which is also
# the more useful question, since equipment count is something an operator
# actually varies.
MARKER_SCALE = 2.0

# Vertical band the markers occupy. Keeps everything well clear of the
# surface (z=0) and of the seabed (z=25); assert_submerged() enforces it.
MARKER_DEPTH_MIN, MARKER_DEPTH_MAX = 7.0, 20.0   # 7 m keeps the 7.5 m pipe clear
SURFACE_CLEARANCE_M = 2.0

# One distinct mesh per face -- the per-face difference is what breaks the
# symmetry. All four are already declared as <look>s in the stock scenario.
FACE_MARKERS = [
    ('models/oil_drum.obj', 'oil_drum'),
    ('models/gas_tank.obj', 'gas_tank'),
    ('models/gas_canister.obj', 'gas_canister'),
    ('models/rust_pipe.obj', 'rust_pipe'),
]


def mesh_z_extent(mesh_rel, scale):
    """Vertical extent of a scaled mesh, for the submersion check."""
    path = os.path.join(os.path.expanduser('~/ros2_ws/src/stonefish_bluerov2/data'), mesh_rel)
    zs = [float(l.split()[3]) for l in open(path) if l.startswith('v ')]
    return min(zs) * scale, max(zs) * scale


def assert_submerged(mesh_rel, scale, depth):
    """Refuse to place anything that would break the surface.

    The first ladder put a 22.5 m pipe at 6 m depth and pushed it 5.3 m into
    the air. Nothing catches that except an explicit check, because the
    scenario loads and the simulator renders it quite happily.
    """
    zmin, _ = mesh_z_extent(mesh_rel, scale)
    top = depth + zmin
    if top < SURFACE_CLEARANCE_M:
        raise SystemExit(
            f'{mesh_rel} at scale {scale}, depth {depth:.1f} m would reach z={top:.1f} m, '
            f'inside the {SURFACE_CLEARANCE_M} m surface clearance. Reduce scale or depth.')


def marker_block(face_idx, scale, depth, bearing_deg, n):
    import math
    mesh, look = FACE_MARKERS[face_idx % len(FACE_MARKERS)]
    assert_submerged(mesh, scale, depth)
    b = math.radians(bearing_deg)
    x = TURBINE_X + MARKER_RADIUS_M * math.cos(b)
    y = TURBINE_Y + MARKER_RADIUS_M * math.sin(b)
    return f'''
	<static name="FaceMarker{face_idx}_{n}" type="model">
		<physical>
			<mesh filename="{mesh}" scale="{scale}"/>
			<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
		</physical>
		<material name="Aluminium"/>
		<look name="{look}"/>
		<world_transform rpy="0.0 0.0 {b:.4f}" xyz="{x:.3f} {y:.3f} {depth:.3f}"/>
	</static>
'''


def marker_layout(count):
    """Spread `count` markers over depth and across the face, rather than
    stacking them in one column up the leg -- which is what produced the
    'three huge barrels stacked along the length' look. Returns a list of
    (depth, bearing_offset_deg)."""
    import math
    if count <= 0:
        return []
    out = []
    # Golden-angle offset so successive markers do not line up vertically.
    for i in range(count):
        frac = (i + 0.5) / count
        depth = MARKER_DEPTH_MIN + frac * (MARKER_DEPTH_MAX - MARKER_DEPTH_MIN)
        offset = ((i * 137.5) % 50.0) - 25.0      # within the +-35 deg sector
        out.append((depth, offset))
    return out


def build_variant(count, n_faces, out_path):
    src = open(STOCK).read()
    if count <= 0 or n_faces <= 0:
        body = ''
    else:
        body = ''.join(
            marker_block(f, MARKER_SCALE, depth, FACE_BEARINGS[f] + off, i)
            for f in range(min(n_faces, 4))
            for i, (depth, off) in enumerate(marker_layout(count))
        )
    # No "--" anywhere in this comment: a double hyphen is illegal inside an
    # XML comment, and Stonefish's parser tolerates it while strict parsers
    # (including eval/structure_symmetry.py's) reject the whole file. The
    # stock bluerov2.scn carried this bug from 2026-09-17 until 2026-10-03.
    comment_body = (f' Symmetry variant: {n_faces} face(s), {count} marker(s) each at scale {MARKER_SCALE}.\n'
                    f'\t     Generated by eval/make_variants.py; do not hand edit.\n'
                    f'\t     Score it with eval/structure_symmetry.py (pass this file). ')
    if '--' in comment_body:
        raise SystemExit('refusing to emit an XML comment containing a double hyphen')
    header = f'\n\t<!--{comment_body}-->\n'
    # Insert before the vehicle include so the markers are part of the scene.
    anchor = '\t<include file="$(find stonefish_bluerov2)/scenarios/bluerov2.scn">'
    if anchor not in src:
        raise SystemExit('could not find the vehicle include in the stock scenario')
    out = src.replace(anchor, header + body + '\n' + anchor, 1)
    with open(out_path, 'w') as fh:
        fh.write(out)
    link = os.path.join(INSTALL_SCN, os.path.basename(out_path))
    if os.path.isdir(INSTALL_SCN) and not os.path.exists(link):
        try:
            os.symlink(out_path, link)
        except OSError:
            pass
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--counts', type=int, nargs='+', default=[0, 3, 6, 12, 20],
                    help='markers per face; size is fixed at MARKER_SCALE')
    ap.add_argument('--faces', type=int, default=4,
                    help='how many of the 4 faces carry a marker (default all)')
    args = ap.parse_args()

    print('Generating symmetry variants')
    made = []
    for c in args.counts:
        tag = 'stock' if c <= 0 else f'n{c}'
        name = f'bluerov2_turbine_sym_{tag}.scn'
        path = os.path.join(SCN_DIR, name)
        build_variant(c, args.faces, path)
        made.append((c, path))
        linked = os.path.exists(os.path.join(INSTALL_SCN, name))
        print(f'  {c:>3} markers/face  {name:<34s} {"linked" if linked else "NOT LINKED"}')

    print('\nScore each with:')
    for c, p in made:
        print(f'  python3 eval/structure_symmetry.py --scn {p}')


if __name__ == '__main__':
    main()
