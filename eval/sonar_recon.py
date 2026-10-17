#!/usr/bin/env python3
"""Inspection-product error: how wrong is the reconstructed structure?

Pose error is a means. What an inspection vehicle delivers is a model of the
structure, so this scores THAT: reconstruct the structure from the vehicle's
own multibeam under each navigation source, and measure the distance of every
reconstructed point to the CAD mesh.

DESIGN, and why it is built this way.

  * The multibeam is a 2-D horizontal fan (a sensor_msgs/LaserScan: 90 deg,
    128 beams, 0.1 m range noise, 10 Hz, mounted 0.2 m forward and 0.2 m
    below the vehicle origin). A 3-D cloud only exists because the vehicle
    sweeps up and down the faces, so depth and heading errors matter as much
    as horizontal ones.
  * THE RETURN SET IS FIXED. A return counts as a structure return if, under
    the TRUE pose, it lies within 0.5 m of the mesh. The same returns are then
    re-projected through each navigation source. Any difference between sources
    is therefore pose error and nothing else; it cannot come from one source
    seeing different returns.
  * Ground truth enters in two places and nowhere else: defining that return
    set, and the "truth" source that measures the sensor floor. No navigation
    source sees it.
  * The mesh is the simulator's own CAD (data/turbines/*_base.obj, placed by
    the scenario file at (20, 0, 25) and (-20, 0, 20)). Distance is to a dense
    surface sampling (about 3 cm spacing in the underwater band), so the
    distance is biased high by at most about 1.5 cm, well under the sensor's
    10 cm noise; that bias is part of the 'truth' floor, not of any source.

NAVIGATION SOURCES.
    truth       /bluerov2/odometry                    the sensor floor
    dr          /bluerov2/odometry/dr                 SLAM-free reference filter
    filtered    /bluerov2/odometry/filtered           the primary filter (SLAM
                                                      fused when SLAM is up)
    slam_trust  raw SLAM pose through an alignment    a COUNTERFACTUAL: what a
                fitted on the poses BEFORE the         vehicle that trusted SLAM
                closure and then held fixed            through the closure would
                                                       have built
    slam_trust takes x, y and the rotation from SLAM and z from truth (the
    depth sensor is good to about 1 cm). The rotation is SLAM's own,
    calibrated by a constant offset fitted before the closure only; that fit's
    residual is reported so the counterfactual can be judged.

Usage:  sonar_recon.py BAG [--closure-t SECONDS_INTO_RUN] [--json OUT]
"""
import os
import sys

try:
    import mcap_ros2  # noqa: F401
except ImportError:
    _alt = os.path.expanduser("~/venv-ardupilot/bin/python3")
    if not os.environ.get("_BAGREADER_REEXEC") and os.path.exists(_alt) \
            and os.path.abspath(_alt) != os.path.abspath(sys.executable):
        os.environ["_BAGREADER_REEXEC"] = "1"
        os.execv(_alt, [_alt] + sys.argv)
    sys.exit("mcap_ros2 not found in any available interpreter")

import argparse
import glob
import json

import numpy as np
from mcap_ros2.reader import read_ros2_messages
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot, Slerp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from slamio import slam_to_ned_xyz                       # noqa: E402

MESH_DIR = os.path.join(HERE, "..", "src", "stonefish_bluerov2", "data",
                        "turbines")
# (file, world position from the scenario's <include ... position=...>)
STRUCTURES = (("turbine2_base.obj", (20.0, 0.0, 25.0)),
              ("turbine1_base.obj", (-20.0, 0.0, 20.0)))
SENSOR_ORIGIN = np.array([0.2, 0.0, 0.2])   # multibeam in base_link
RETURN_RADIUS = 0.5                          # m, defines a structure return
MAX_SLAM_GAP = 0.5                           # s, beyond this SLAM has no pose

T_TRUTH, T_DR, T_FILT = ("/bluerov2/odometry", "/bluerov2/odometry/dr",
                         "/bluerov2/odometry/filtered")
T_SLAM, T_SONAR = "/bluerov2/robot_pose_slam", "/bluerov2/multibeam_raw"


# ------------------------------------------------------------------- mesh
def _load_obj(path):
    V, F = [], []
    for ln in open(path, errors="replace"):
        if ln.startswith("v "):
            V.append(list(map(float, ln.split()[1:4])))
        elif ln.startswith("f "):
            idx = [int(t.split("/")[0]) for t in ln.split()[1:]]
            idx = [i - 1 if i > 0 else len(V) + i for i in idx]
            for k in range(1, len(idx) - 1):
                F.append((idx[0], idx[k], idx[k + 1]))
    return np.array(V), np.array(F)


_MESH = {}


def mesh_tree(samples_per_structure=6_000_000):
    """KD-tree of a dense surface sampling of the underwater band."""
    if "tree" in _MESH:
        return _MESH["tree"], _MESH["pts"]
    rng = np.random.default_rng(0)
    pts = []
    for name, off in STRUCTURES:
        V, F = _load_obj(os.path.join(MESH_DIR, name))
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        tri = rng.choice(len(F), size=samples_per_structure, p=area / area.sum())
        u, v = rng.random(len(tri)), rng.random(len(tri))
        m = u + v > 1
        u[m], v[m] = 1 - u[m], 1 - v[m]
        S = a[tri] + u[:, None] * (b[tri] - a[tri]) + v[:, None] * (c[tri] - a[tri])
        S = S + np.array(off)
        pts.append(S[(S[:, 2] > -2.0) & (S[:, 2] < 28.0)])
    P = np.vstack(pts)
    _MESH["pts"], _MESH["tree"] = P, cKDTree(P)
    return _MESH["tree"], P


# ------------------------------------------------------------------ tracks
class Track:
    """A pose source: position (N,3) and orientation, queried at any time."""

    def __init__(self, t, pos, rot):
        t = np.asarray(t, float)
        t, k = np.unique(t, return_index=True)
        self.t, self.pos = t, np.asarray(pos)[k]
        self.rot = rot[k]
        self._slerp = Slerp(self.t, self.rot)

    def covers(self, tq):
        return (tq >= self.t[0]) & (tq <= self.t[-1])

    def at(self, tq):
        p = np.column_stack([np.interp(tq, self.t, self.pos[:, i])
                             for i in range(3)])
        return p, self._slerp(tq)

    def gap(self, tq):
        """Distance in time to the nearest sample."""
        i = np.clip(np.searchsorted(self.t, tq), 1, len(self.t) - 1)
        return np.minimum(np.abs(tq - self.t[i - 1]), np.abs(self.t[i] - tq))


def read_bag(bag):
    want = (T_TRUTH, T_DR, T_FILT, T_SLAM, T_SONAR)
    od = {T_TRUTH: [], T_DR: [], T_FILT: []}
    slam, scans = [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=list(want)):
            t = (m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                 else m.log_time / 1e9)
            top, r = m.channel.topic, m.ros_msg
            if top in od:
                q = r.pose.pose
                od[top].append((t, q.position.x, q.position.y, q.position.z,
                                q.orientation.x, q.orientation.y,
                                q.orientation.z, q.orientation.w))
            elif top == T_SLAM:
                p, q = r.pose.position, r.pose.orientation
                x, y, z = slam_to_ned_xyz(p.x, p.y, p.z)
                # ENU -> NED for a quaternion: (w, x, y, z) -> (w, x, -y, -z)
                slam.append((t, x, y, z, q.x, -q.y, -q.z, q.w))
            elif top == T_SONAR:
                scans.append((t, np.array(r.ranges, float), r.angle_min,
                              r.angle_increment, r.range_min, r.range_max))
    tracks = {}
    for k, name in ((T_TRUTH, "truth"), (T_DR, "dr"), (T_FILT, "filtered")):
        if od[k]:
            a = np.array(od[k])
            tracks[name] = Track(a[:, 0], a[:, 1:4], Rot.from_quat(a[:, 4:8]))
    s = np.array(slam) if slam else None
    return tracks, s, scans


def sonar_points(scans, tmin, tmax):
    """Per-scan sensor points in base_link, with times."""
    out, tt = [], []
    for t, rg, a0, da, rmin, rmax in scans:
        if t < tmin or t > tmax:
            continue
        ok = (rg > max(rmin, 0.25)) & (rg < min(rmax, 49.0))
        if not ok.any():
            continue
        th = a0 + np.arange(len(rg))[ok] * da
        r = rg[ok]
        out.append(np.column_stack([r * np.cos(th), r * np.sin(th),
                                    np.zeros(len(r))]) + SENSOR_ORIGIN)
        tt.append(np.full(len(r), t))
    return np.vstack(out), np.concatenate(tt)


def project(track, pb, tt):
    """World points from base-link points under a pose source."""
    pw = np.empty_like(pb)
    ut, inv = np.unique(tt, return_inverse=True)
    pos, rot = track.at(ut)
    for i in range(len(ut)):                    # one rotation per scan
        m = inv == i
        pw[m] = rot[i].apply(pb[m]) + pos[i]
    return pw


# ------------------------------------------------------- SLAM counterfactual
def slam_trust_track(slam, truth, t_closure):
    """SLAM through an alignment fitted before the closure and held fixed."""
    import closure_events as ce
    t, xy = slam[:, 0], slam[:, 1:3]
    pre = t < t_closure
    tp, _ = truth.at(t[pre])
    to_w, th, k, resid = ce.fit_similarity(xy[pre], tp[:, :2])
    P = np.array([to_w(s) for s in xy])
    P_z = np.interp(t, truth.t, truth.pos[:, 2])
    pos = np.column_stack([P, P_z])
    Rs = Rot.from_quat(slam[:, 4:8])
    Rz = Rot.from_euler("z", th)
    Rpre = Rz * Rs
    Rt = truth.at(t[pre])[1]
    Rc = (Rpre[pre].inv() * Rt).mean()          # constant offset, pre-closure
    err = (Rpre[pre] * Rc).inv() * Rt
    ang = np.degrees(err.magnitude())
    info = dict(fit_resid_m=float(resid), fit_scale=float(k),
                fit_theta_deg=float(np.degrees(th)),
                rot_resid_deg_mean=float(ang.mean()),
                rot_resid_deg_p95=float(np.percentile(ang, 95)),
                n_pre=int(pre.sum()))
    return Track(t, pos, Rpre * Rc), info


# ----------------------------------------------------------------- scoring
def summarise(d):
    d = np.asarray(d)
    return dict(n=int(len(d)), rms=float(np.sqrt(np.mean(d ** 2))),
                median=float(np.median(d)), p90=float(np.percentile(d, 90)),
                within_025=float(np.mean(d < 0.25)),
                within_100=float(np.mean(d < 1.0)))


def score(bag, t_closure_rel=None):
    tracks, slam, scans = read_bag(bag)
    tr = tracks["truth"]
    t0 = tr.t[0]
    srcs = {k: v for k, v in tracks.items()}
    info = {}
    if slam is not None and t_closure_rel is not None:
        srcs["slam_trust"], info = slam_trust_track(slam, tr, t0 + t_closure_rel)
    tree, _ = mesh_tree()
    tmin = max(v.t[0] for v in srcs.values())
    tmax = min(v.t[-1] for v in srcs.values())
    pb, tt = sonar_points(scans, tmin, tmax)
    # the SLAM counterfactual has no pose where SLAM was not publishing
    keep = np.ones(len(tt), bool)
    if "slam_trust" in srcs:
        keep &= srcs["slam_trust"].gap(tt) <= MAX_SLAM_GAP
    pb, tt = pb[keep], tt[keep]
    # the fixed return set, defined under the TRUE pose
    pw_truth = project(srcs["truth"], pb, tt)
    d_truth, _ = tree.query(pw_truth, workers=-1)
    inl = d_truth < RETURN_RADIUS
    pb, tt = pb[inl], tt[inl]
    res = {"returns": int(inl.sum()), "mission_s": float(tt.max() - tt.min()),
           "slam_trust_calibration": info, "sources": {}, "pts": {}}
    for name, track in srcs.items():
        pw = project(track, pb, tt)
        d, _ = tree.query(pw, workers=-1)
        r = {"all": summarise(d)}
        if t_closure_rel is not None:
            tc = t0 + t_closure_rel
            r["before_closure"] = summarise(d[tt < tc]) if (tt < tc).any() else None
            r["after_closure"] = summarise(d[tt >= tc]) if (tt >= tc).any() else None
        res["sources"][name] = r
        res["pts"][name] = (pw, d, tt - t0)
    return res


def print_table(res):
    print(f"  structure returns: {res['returns']}   mission {res['mission_s']:.0f} s")
    c = res["slam_trust_calibration"]
    if c:
        print(f"  slam_trust calibration (before the closure): position fit "
              f"{c['fit_resid_m']:.2f} m, scale {c['fit_scale']:.3f}, rotation "
              f"residual mean {c['rot_resid_deg_mean']:.1f} deg / p95 "
              f"{c['rot_resid_deg_p95']:.1f} deg  (n={c['n_pre']})")
    keys = [("all", "whole run")]
    if any("after_closure" in v for v in res["sources"].values()):
        keys += [("before_closure", "before closure"),
                 ("after_closure", "after closure")]
    for k, lab in keys:
        print(f"\n  {lab}")
        print(f"  {'source':12s} {'rms (m)':>8s} {'median':>8s} {'p90':>8s} "
              f"{'<25 cm':>8s} {'<1 m':>7s}")
        for name, v in res["sources"].items():
            s = v.get(k)
            if s:
                print(f"  {name:12s} {s['rms']:8.3f} {s['median']:8.3f} "
                      f"{s['p90']:8.3f} {100 * s['within_025']:7.1f}% "
                      f"{100 * s['within_100']:6.1f}%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--closure-t", type=float, default=None,
                    help="seconds into the run of the wrong-face closure; "
                         "enables the slam_trust counterfactual")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    res = score(a.bag, a.closure_t)
    print(f"SONAR RECONSTRUCTION vs CAD: {os.path.basename(os.path.normpath(a.bag))}")
    print_table(res)
    if a.json:
        slim = {k: v for k, v in res.items() if k != "pts"}
        json.dump(slim, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
