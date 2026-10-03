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

Markers sit at radius ATTACH_RADIUS_M on each face bearing, just outside
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

# Which faces carry the distinguishing hardware. Two of four, leaving the
# other two identical -- so confusion between the unmarked pair remains
# possible and failure rate should fall without reaching zero. That is the
# informative case; marking all four would just remove the phenomenon.
MARKED_FACES = [0, 1]

# WHAT GETS ADDED, and why only these two things. An earlier version
# scattered oil drums, gas tanks and canisters around each face. Rendered,
# they were visibly floating in open water, unattached, intersecting the
# bracing -- because they sat at r = 13 m while the jacket surface at the
# face bearings is at r = 8.0-9.9 m. Beyond the placement bug, drums bolted
# to a turbine do not represent anything that exists offshore, so the
# manipulation would not have survived review even mounted correctly.
#
# Real jackets differ between faces in two ways that are easy to model
# here: J-tubes and risers run up particular faces rather than all of them,
# and sacrificial anodes vary in size and placement. Both attach to the
# structure. The pipe mesh is literally a pipe; the anode mesh is already
# used six times in the stock scenario.
PIPE_MESH, PIPE_LOOK = 'models/rust_pipe.obj', 'rust_pipe'
ANODE_MESH, ANODE_LOOK = 'anodes/anode.obj', 'anode_bar'

# Flush against the structure. Surface sits at r = 8.0-9.9 m depending on
# depth, so this stands the hardware 0.3-2.2 m proud of the bracing: close
# enough to read as mounted, clear enough not to intersect it.
ATTACH_RADIUS_M = 10.2

# Riser run: from just above the seabed up through the sea surface to
# topside, which is what a riser actually does.
RISER_BOTTOM_M, RISER_TOP_M = 23.0, -4.0
HORIZONTAL_RUN_DEPTHS_M = [9.0, 16.0]


def mesh_z_extent(mesh_rel, scale):
    """Vertical extent of a scaled mesh, for the submersion check."""
    path = os.path.join(os.path.expanduser('~/ros2_ws/src/stonefish_bluerov2/data'), mesh_rel)
    zs = [float(l.split()[3]) for l in open(path) if l.startswith('v ')]
    return min(zs) * scale, max(zs) * scale


def assert_submerged(mesh_rel, scale, depth, allow_breach=False):
    """Refuse to place anything that would break the surface.

    The first ladder put a 22.5 m pipe at 6 m depth and pushed it 5.3 m into
    the air. Nothing catches that except an explicit check, because the
    scenario loads and the simulator renders it quite happily.
    """
    if allow_breach:
        return      # risers are meant to reach topside
    zmin, _ = mesh_z_extent(mesh_rel, scale)
    top = depth + zmin
    if top < SURFACE_CLEARANCE_M:
        raise SystemExit(
            f'{mesh_rel} at scale {scale}, depth {depth:.1f} m would reach z={top:.1f} m, '
            f'inside the {SURFACE_CLEARANCE_M} m surface clearance. Reduce scale or depth.')


def hardware_blocks(face_idx, anode_scale, n_anodes):
    """A vertical pipe run plus a column of sacrificial anodes on one face.

    Both are mounted at ATTACH_RADIUS_M so they sit against the bracing
    rather than hanging in open water, which is what the first version did.
    """
    import math
    b = math.radians(FACE_BEARINGS[face_idx])
    x = TURBINE_X + ATTACH_RADIUS_M * math.cos(b)
    y = TURBINE_Y + ATTACH_RADIUS_M * math.sin(b)
    out = []

    # J-tube / riser. Built from contiguous segments rather than one
    # oversized stub: uniform mesh scaling would fatten the diameter along
    # with the length, and at scale 2 the single 7.5 m section rendered as a
    # stray pipe floating in a bay. At scale 1 the mesh is 0.37 m across --
    # an ordinary J-tube -- and butting segments end to end gives a run of
    # any length at the right diameter.
    #
    # The run deliberately breaks the sea surface: a riser carries product
    # from the seabed to topside, so stopping it underwater would be the
    # unrealistic choice. This is the one case where the submersion check is
    # waived, by exception rather than by loosening the rule.
    seg_h = mesh_z_extent(PIPE_MESH, 1.0)[1] - mesh_z_extent(PIPE_MESH, 1.0)[0]
    depth_cursor = RISER_BOTTOM_M
    seg = 0
    while depth_cursor > RISER_TOP_M:
        assert_submerged(PIPE_MESH, 1.0, depth_cursor, allow_breach=True)
        out.append(f"""
\t<static name="FaceHardware{face_idx}_riser{seg}" type="model">
\t\t<physical>
\t\t\t<mesh filename="{PIPE_MESH}" scale="1.0"/>
\t\t\t<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t</physical>
\t\t<material name="Aluminium"/>
\t\t<look name="{PIPE_LOOK}"/>
\t\t<world_transform rpy="0.0 0.0 {b:.4f}" xyz="{x:.3f} {y:.3f} {depth_cursor:.3f}"/>
\t</static>
""")
        depth_cursor -= seg_h
        seg += 1

    # Sacrificial anodes, same orientation as the six already in the stock
    # scenario (pitched 90 deg so they lie along the member).
    for i in range(n_anodes):
        depth = MARKER_DEPTH_MIN + (i + 0.5) / max(1, n_anodes) * (MARKER_DEPTH_MAX - MARKER_DEPTH_MIN)
        assert_submerged(ANODE_MESH, anode_scale, depth)
        out.append(f"""
	<static name="FaceHardware{face_idx}_anode{i}" type="model">
		<physical>
			<mesh filename="{ANODE_MESH}" scale="{anode_scale}"/>
			<origin rpy="0.0 1.571 0.0" xyz="0.0 0.0 0.0"/>
		</physical>
		<material name="Aluminium"/>
		<look name="{ANODE_LOOK}"/>
		<world_transform rpy="0.0 0.0 {b:.4f}" xyz="{x:.3f} {y:.3f} {depth:.3f}"/>
	</static>
""")
    return ''.join(out)


def connecting_runs(anode_scale):
    """Continuous horizontal pipeline joining the risers on the two marked
    faces, routed around the structure rather than stubbing out into open
    water.

    The first version hung two short tangential segments off each riser.
    Rendered, they read as pipes pointing at nothing -- the run has to go
    somewhere. Faces 0 and 1 are 90 deg apart, so the pipeline follows that
    arc at ATTACH_RADIUS_M, passing the leg between them, with segments
    overlapping slightly so the run is visually unbroken.
    """
    import math
    if anode_scale <= 0:
        return ''
    b0 = math.radians(FACE_BEARINGS[MARKED_FACES[0]])
    b1 = math.radians(FACE_BEARINGS[MARKED_FACES[1]])
    sweep = (b1 - b0 + math.pi) % (2 * math.pi) - math.pi      # short way round
    seg_len = mesh_z_extent(PIPE_MESH, 1.0)[1] - mesh_z_extent(PIPE_MESH, 1.0)[0]
    arc_len = abs(sweep) * ATTACH_RADIUS_M
    n = max(1, int(math.ceil(arc_len / (seg_len * 0.85))))      # 15% overlap
    out = []
    for j, hd in enumerate(HORIZONTAL_RUN_DEPTHS_M):
        for i in range(n):
            th = b0 + sweep * (i + 0.5) / n
            hx = TURBINE_X + ATTACH_RADIUS_M * math.cos(th)
            hy = TURBINE_Y + ATTACH_RADIUS_M * math.sin(th)
            out.append(f"""
\t<static name="Pipeline_run{j}_seg{i}" type="model">
\t\t<physical>
\t\t\t<mesh filename="{PIPE_MESH}" scale="1.0"/>
\t\t\t<origin rpy="0.0 1.571 0.0" xyz="0.0 0.0 0.0"/>
\t\t</physical>
\t\t<material name="Aluminium"/>
\t\t<look name="{PIPE_LOOK}"/>
\t\t<world_transform rpy="0.0 0.0 {th + math.pi/2:.4f}" xyz="{hx:.3f} {hy:.3f} {hd:.3f}"/>
\t</static>
""")
    return ''.join(out)


def build_variant(anode_scale, n_anodes, out_path):
    src = open(STOCK).read()
    if anode_scale <= 0 or n_anodes <= 0:
        body = ''
    else:
        body = ''.join(hardware_blocks(f, anode_scale, n_anodes) for f in MARKED_FACES)
        body += connecting_runs(anode_scale)

    comment_body = (f' Symmetry variant: faces {MARKED_FACES} carry a riser plus '
                    f'{n_anodes} anodes at scale {anode_scale}.\n'
                    f'\t     Generated by eval/make_variants.py; do not hand edit.\n'
                    f'\t     Score it with eval/structure_symmetry.py (pass this file). ')
    if '--' in comment_body:
        raise SystemExit('refusing to emit an XML comment containing a double hyphen')
    header = f'\n\t<!--{comment_body}-->\n'

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
    ap.add_argument('--anode-scales', type=float, nargs='+', default=[0, 8, 14, 20],
                    help='anode amplification on the marked faces; 0 = stock')
    ap.add_argument('--n-anodes', type=int, default=6,
                    help='anodes per marked face')
    ap.add_argument('--faces', type=int, default=4,
                    help='how many of the 4 faces carry a marker (default all)')
    args = ap.parse_args()

    print('Generating symmetry variants')
    made = []
    for sc in args.anode_scales:
        tag = 'stock' if sc <= 0 else f'a{int(sc)}'
        name = f'bluerov2_turbine_sym_{tag}.scn'
        path = os.path.join(SCN_DIR, name)
        build_variant(sc, args.n_anodes, path)
        made.append((sc, path))
        linked = os.path.exists(os.path.join(INSTALL_SCN, name))
        print(f'  anode scale {sc:>5}  {name:<34s} {"linked" if linked else "NOT LINKED"}')

    print('\nScore each with:')
    for c, p in made:
        print(f'  python3 eval/structure_symmetry.py --scn {p}')


if __name__ == '__main__':
    main()
