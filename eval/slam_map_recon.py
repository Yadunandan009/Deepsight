#!/usr/bin/env python3
"""Score ORB-SLAM3's own map points against the CAD.

`/bluerov2/map_points` is the whole map ORB-SLAM3 holds, in SLAM's frame
(ENU, like the pose topic). To compare it with the CAD it has to be placed in
the world. The placement is a 7-DOF similarity fitted between the SLAM
trajectory and the TRUE trajectory before the wrong-face closure, then held:
the same rule as `slam_trust` in sonar_recon.py. The truth is used only for that
placement, so the result says how good the map is, not what a vehicle could know.

Two snapshots: the last one before the closure, and the last of the mission.

    python3 slam_map_recon.py BAG --closure-t 573
"""
import argparse, glob, os, sys
import numpy as np
from mcap_ros2.reader import read_ros2_messages

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sonar_recon as sr


def umeyama(src, dst):
    """Similarity (s, R, t) with dst ~ s R src + t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    A, B = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(B.T @ A / len(src))
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (A ** 2).sum() * len(src)
    return s, R, mu_d - s * R @ mu_s


def snapshots(bag, t0, want_rel, closure):
    """Map-point clouds (NED, SLAM frame): the last one before the closure and
    the LARGEST one after it (the last message of a run can be a freshly reset,
    near-empty map)."""
    got, big = {}, (0, None, None)
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/map_points"]):
            r = m.ros_msg
            rel = m.log_time.timestamp() - t0
            dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                           "offsets": [0, 4, 8], "itemsize": r.point_step})
            a = np.frombuffer(bytes(r.data), dtype=dt)
            xyz = lambda: np.column_stack([a["x"], -a["y"], -a["z"]]).astype(float)
            if rel <= closure - 5.0 and ("before" not in got or rel > got["before"][0]):
                got["before"] = (rel, xyz())
            if rel > closure and len(a) > big[0]:
                big = (len(a), rel, xyz())
    got["after"] = (big[1], big[2])
    return got


def icp(pts, tree, s, R, t, iters=15, trim=2.0):
    """Best similarity placement of the map on the CAD (trimmed ICP), started
    from the trajectory-based placement. Separates map shape from placement."""
    pts = pts[np.random.default_rng(1).choice(len(pts), size=min(len(pts), 40000),
                                              replace=False)]
    for _ in range(iters):
        w = s * (R @ pts.T).T + t
        d, i = tree.query(w)
        k = d < trim
        s2, R2, t2 = umeyama(w[k], tree.data[i[k]])
        s, R, t = s2 * s, R2 @ R, s2 * R2 @ t + t2
    return s, R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--closure-t", type=float, required=True)
    ap.add_argument("--out", default=os.path.join(HERE, "figures", "slam_map.npz"))
    ap.add_argument("--no-json", action="store_true",
                    help="do not add this bag to slam_map_results.json")
    a = ap.parse_args()
    tracks, slam, _ = sr.read_bag(a.bag)
    tr = tracks["truth"]
    t0 = tr.t[0]
    pre = slam[:, 0] < t0 + a.closure_t
    tp, _ = tr.at(slam[pre, 0])
    s, R, t = umeyama(slam[pre, 1:4], tp)
    fit = np.linalg.norm((s * (R @ slam[pre, 1:4].T).T + t) - tp, axis=1)
    print(f"alignment: scale {s:.4f}  trajectory residual rms {np.sqrt((fit**2).mean()):.2f} m  n={pre.sum()}")
    snaps = snapshots(a.bag, t0, None, a.closure_t)
    tree, P = sr.mesh_tree()
    out, rng = {}, np.random.default_rng(0)
    XLIM, YLIM, ZLIM = (-23, 14), (2, 36), (0, 25)
    for k, (rel, pts) in snaps.items():
        w = (s * (R @ pts.T).T + t)
        d, _ = tree.query(w)
        s_i, R_i, t_i = icp(pts, tree, s, R, t)
        d_best, _ = tree.query(s_i * (R_i @ pts.T).T + t_i)
        print(f"  best placement (ICP, uses the CAD): scale {s_i:.3f}  median {np.median(d_best):.2f} m  "
              f"within 0.25 m {np.mean(d_best<0.25)*100:.1f}%  0.5 m {np.mean(d_best<0.5)*100:.1f}%")
        out[f"{k}_best"] = np.array([np.median(d_best), np.mean(d_best < .25), np.mean(d_best < .5)])
        out[f"{k}_dbest_sample"] = rng.choice(d_best, size=min(40000, len(d_best)), replace=False)
        out[f"{k}_dtraj_sample"] = rng.choice(d, size=min(40000, len(d)), replace=False)
        box = ((w[:, 1] > XLIM[0]) & (w[:, 1] < XLIM[1]) & (w[:, 0] > YLIM[0]) &
               (w[:, 0] < YLIM[1]) & (w[:, 2] > -0.5) & (w[:, 2] < ZLIM[1] + 0.5))
        print(f"{k:7s} t={rel:6.0f}s  points {len(w):7d}  in-view {box.mean()*100:5.1f}%  "
              f"within 0.25 m {np.mean(d<0.25)*100:5.1f}%  0.5 m {np.mean(d<0.5)*100:5.1f}%  "
              f"1 m {np.mean(d<1)*100:5.1f}%  median {np.median(d):.2f} m  (in-view: "
              f"median {np.median(d[box]):.2f}, within 0.5 m {np.mean(d[box]<0.5)*100:.1f}%)")
        idx = np.flatnonzero(box)
        sel = rng.choice(idx, size=min(22000, len(idx)), replace=False)
        out[f"{k}_xyz"], out[f"{k}_d"] = w[sel], d[sel]
        out[f"{k}_stats"] = np.array([len(w), box.mean(), np.mean(d < .25),
                                      np.mean(d < .5), np.mean(d < 1), np.median(d), rel])
    np.savez(a.out, **out)
    if a.no_json:
        return
    import json
    rf = os.path.join(HERE, "figures", "slam_map_results.json")
    R_ = json.load(open(rf)) if os.path.exists(rf) else {}
    R_[os.path.basename(a.bag.rstrip("/"))] = dict(
        closure_t=a.closure_t, scale=float(s), traj_resid_rms_m=float(np.sqrt((fit**2).mean())),
        **{k: dict(zip(("n_points", "in_view", "within_025", "within_050", "within_100",
                        "median_m", "t_s"), map(float, out[f"{k}_stats"])),
                   best=dict(zip(("median_m", "within_025", "within_050"),
                                 map(float, out[f"{k}_best"]))))
           for k in ("before", "after")})
    json.dump(R_, open(rf, "w"), indent=1)


if __name__ == "__main__":
    main()
