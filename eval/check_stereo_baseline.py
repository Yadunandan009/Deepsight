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
        xyz = [float(v) for v in o.group(1).split()]
        out[side] = xyz[1]
        out[side + "_xyz"] = xyz
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


def opencv_parseable(path):
    """ORB-SLAM3 reads settings with cv::FileStorage, which is NOT a YAML
    parser. It requires the %YAML:1.0 directive on line 1 and rejects the
    whole file otherwise -- including when the only thing above it is a
    comment. That failure surfaces as

        OpenCV(4.4.0) persistence.cpp:717: error: (-49:Unknown error code
        -49) Input file is invalid in function 'open'

    and kills the container at startup. The regex checks in this file are
    happy with such a file, so ask OpenCV itself.
    """
    try:
        import cv2
    except ImportError:
        return None                      # cannot check here; not a failure
    # OpenCV 4.4 (the container's) returns isOpened()==False; 4.13 (the
    # host's) raises instead. Treat both as "ORB-SLAM3 will not read this".
    try:
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    except Exception:
        return False
    try:
        return bool(fs.isOpened())
    finally:
        fs.release()



def self_occlusion(scn, cams):
    """Does the vehicle's own body sit inside a camera's frustum?

    Widening the baseline moves the cameras outboard into the hull
    envelope. At y=+/-0.15 and the stock x=0.16 mount, 676 hull vertices
    fall inside the 75x60 deg frustum at 3.3 cm, and ORB-SLAM3 tracks
    features on the vehicle instead of the structure -- observed live
    2026-10-15, which cost a launch.

    Vertex sampling, so it under-reports a large face seen edge-on; the
    inflated frustum is the margin against that. Returns a list of
    complaints.
    """
    import glob
    data = os.path.join(os.path.dirname(scn), "..", "data", "bluerov2")
    objs = sorted(glob.glob(os.path.join(data, "bluerov2*.obj")))
    objs = [o for o in objs if "_phy" not in o]
    if not objs:
        return [], []
    try:
        import numpy as np
    except ImportError:
        return [], []
    v = []
    for o in objs:
        with open(o) as fh:
            v += [[float(x) for x in l.split()[1:4]]
                  for l in fh if l.startswith("v ")]
    body = np.array(v)
    # Only the REAL frustum blocks a launch. The reference arm trips an
    # inflated one (825 verts at 7.4 cm in 110x95) and has always worked,
    # so treat the margin as advisory or it cries wolf on the known-good
    # configuration.
    errs, warns = [], []
    for side in ("left", "right"):
        cam = np.array(cams[side + "_xyz"])
        pos = " ".join(f"{c:.3f}" for c in cam)
        d = body - cam
        d = d[d[:, 0] > 0.002]
        if not len(d):
            continue
        ah = np.degrees(np.arctan2(np.abs(d[:, 1]), d[:, 0]))
        av = np.degrees(np.arctan2(np.abs(d[:, 2]), d[:, 0]))
        for (hf, vf, bucket, label) in (
                (75.0, 60.0, errs, "IN FRAME"),
                (110.0, 95.0, warns, "inside a 110x95 margin frustum")):
            m = (ah < hf / 2) & (av < vf / 2)
            if m.sum():
                near = np.linalg.norm(d[m], axis=1).min()
                bucket.append(f"camera_{side} at ({pos}): {int(m.sum())} body "
                              f"vertices {label}, nearest {near:.3f} m")
                break
    return errs, warns


def check(name, scn, yml):
    if not (os.path.exists(scn) and os.path.exists(yml)):
        print(f"  {name}: MISSING ({scn if not os.path.exists(scn) else yml})")
        return False
    parse = opencv_parseable(yml)
    if parse is False:
        print(f"  {name}: {os.path.basename(yml)} is NOT readable by "
              f"cv::FileStorage.\n"
              f"      ORB-SLAM3 will abort on startup. Most likely the "
              f"%YAML:1.0 directive\n"
              f"      is not the first line of the file.")
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
    occ, occ_warn = self_occlusion(scn, s)
    pmsg = "" if parse else "  (cv::FileStorage check skipped: no cv2)"
    print(f"  {name}:  b = {b:.4f} m   mThDepth = b*ThDepth = "
          f"{b * 40.0:.2f} m{pmsg}")
    for label, got, want in rows:
        flag = "  <-- MISMATCH" if abs(got - want) > TOL else ""
        print(f"      {label:22s} implies b = {got:.4f}{flag}")
    for c in occ:
        print(f"      SELF-OCCLUSION: {c}")
    for c in occ_warn:
        print(f"      note: {c} (advisory -- the reference arm does this too)")
    return not bad and not occ



def resolve_scn(path):
    """A turbine scenario includes the robot scenario that holds the
    cameras. Follow one level of <include> to find it."""
    txt = open(path).read()
    if 'name="camera_left"' in txt:
        return path
    for m in re.finditer(r'<include file="([^"]+)"', txt):
        if m.group(1).lstrip().startswith("<!--"):
            continue
        cand = os.path.join(os.path.dirname(path),
                            os.path.basename(m.group(1)))
        if os.path.exists(cand) and 'name="camera_left"' in open(cand).read():
            return cand
    return None


def live_check():
    """Compare what the SIMULATOR loaded against what SLAM is solving.

    Both sides are read from the running system:

      simulator -- the scenario path on the stonefish process command
                   line. This is the installed copy, so it also catches a
                   source tree that was edited without a colcon build.
      SLAM      -- Stereo.b in the yaml bind-mounted into the container.

    An earlier version of this read the baseline from the right camera's
    CameraInfo P[3]. Stonefish publishes P[3] = 0 and never encodes the
    baseline there, so that check refused every SLAM arm. Do not
    reintroduce it.
    """
    import subprocess
    print("live check (running system, not the source tree)\n")
    sim_b = sim_scn = None
    try:
        ps = subprocess.run(["ps", "-eo", "args"], capture_output=True,
                            text=True, timeout=10).stdout
    except Exception:
        ps = ""
    for line in ps.splitlines():
        if "stonefish_simulator" in line and "grep" not in line:
            for tok in line.split():
                if tok.endswith(".scn") and os.path.exists(tok):
                    sim_scn = tok
            break
    if sim_scn:
        robot = resolve_scn(sim_scn)
        if robot:
            sim_b = from_scn(robot)["b"]
            print(f"  simulator : {os.path.basename(sim_scn)}")
            print(f"              -> {os.path.basename(robot)}   "
                  f"b = {sim_b:.4f} m")
        else:
            print(f"  simulator : {sim_scn} (no camera_left found)")
    else:
        print("  simulator : not running, or no .scn on its command line")

    slam_b = slam_cfg = None
    try:
        mounts = subprocess.run(
            ["docker", "inspect", "-f",
             r'{{range .Mounts}}{{.Source}}{{"\n"}}{{end}}',
             "orb_slam3_running"],
            capture_output=True, text=True, timeout=10).stdout
    except Exception:
        mounts = ""
    for m in mounts.split():
        if os.path.isfile(m):
            for l in open(m):
                if l.startswith("Stereo.b:"):
                    slam_b, slam_cfg = float(l.split()[1]), m
                    break
        if slam_b is not None:
            break
    if slam_b is not None:
        print(f"  ORB-SLAM3 : {os.path.basename(slam_cfg)}   "
              f"b = {slam_b:.4f} m")
    else:
        print("  ORB-SLAM3 : container not running, or no Stereo.b in its "
              "mounts")

    print()
    if sim_b is None or slam_b is None:
        print("  INCOMPLETE: one side could not be read. Nothing is "
              "asserted.")
        return 0
    if abs(sim_b - slam_b) > TOL:
        print(f"  MISMATCH: the simulator is running b = {sim_b:.4f} m while "
              f"ORB-SLAM3\n  solves for b = {slam_b:.4f} m. Different arms.")
        return 1
    print(f"  AGREE: both sides are b = {sim_b:.4f} m "
          f"(close-depth gate {sim_b * 40:.2f} m).")
    return 0


def main():
    if "--live" in sys.argv:
        return live_check()
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
    print("  PROBLEM. Do not launch: the geometry recorded would not be the\n"
          "  geometry simulated, or a camera is looking at the vehicle.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
