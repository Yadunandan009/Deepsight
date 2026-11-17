#!/usr/bin/env python3
"""How good was the SLAM, against ground truth, and what would the bridge have
made of it? One flight, or one offline replay of a flight (replay_slam.sh).

  python3 eval/slam_quality.py --bag BAG --frame vi
  python3 eval/slam_quality.py --bag BAG --slam REPLAY_BAG --frame vi

--bag is the flight: ground truth, the SLAM-free reference filter
(/bluerov2/odometry/dr) and the DVL come from it. --slam, if given, is a
replay_slam.sh output whose SLAM pose and map points replace the flight's.
Both are put on the camera clock: the SLAM pose carries its image's stamp
(the logged time trails it by ~0.1 s of processing), so a replay lines up
with the flight's truth exactly. Map points carry the wall clock of whoever
published them; a replay's are shifted by the replay's own offset.

Reports, all against truth:
  frame     a Sim(3) fit over the longest map: tilt from level and scale.
            A gravity-aligned (stereo-inertial) map should be ~level; a large
            tilt means the frame mapping or the gravity estimate is wrong.
  rigidity  per map segment, the residual of ONE similarity over the whole
            segment. The map's own distortion; no bridge can remove it.
  local     the same over 30 s windows: what the map is worth short-term.
  heading   SLAM yaw minus IMU yaw, per 10 s window and in total: SLAM's
            heading drift, by motion type.
  bridge    the real slam_pose_bridge replayed over it against the flight's
            reference filter (eval/replay_bridge.replay), error per 3 min,
            beside the reference filter's own error.
Writes the numbers to eval/figures/slam_quality_<label>.json.

The slow part is reading the bags; each is cached once under
~/.cache/ros2ws_eval/.
"""
import argparse
import glob
import json
import math
import os
import pickle
import sys

os.environ.setdefault("ROS_DOMAIN_ID", "77")   # the bridge replay is a real node; keep it off a live system

import numpy as np
from mcap_ros2.reader import read_ros2_messages

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "src", "stonefish_bluerov2"))
from stonefish_bluerov2.frames import enu_to_ned_pose_inplace, vi_to_ned_pose_inplace  # noqa: E402
import replay_bridge as rb  # noqa: E402

CACHE = os.path.expanduser("~/.cache/ros2ws_eval")
CONVERT = {"enu": enu_to_ned_pose_inplace, "vi": vi_to_ned_pose_inplace, "ned": lambda p: p}


def yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def stamp(h):
    return h.sec + h.nanosec * 1e-9


def read(bag, topics):
    """{topic: rows}; pose rows (t_log, t_stamp, x, y, z, qx, qy, qz, qw)."""
    os.makedirs(CACHE, exist_ok=True)
    key = os.path.join(CACHE, os.path.basename(bag.rstrip("/")) + ".pkl")
    d = pickle.load(open(key, "rb")) if os.path.exists(key) else {}
    need = [t for t in topics if t not in d]
    if need:
        new = {t: [] for t in need}
        for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
            for m in read_ros2_messages(f, topics=need):
                t, k, r = m.log_time.timestamp(), m.channel.topic, m.ros_msg
                if k == "/bluerov2/map_points":
                    new[k].append((t, stamp(r.header.stamp), r.width * r.height))
                elif k == "/bluerov2/dvl_twist":
                    new[k].append((t, r.twist.covariance[0]))
                else:
                    pose = r.pose if k == "/bluerov2/robot_pose_slam" else r.pose.pose
                    p, q = pose.position, pose.orientation
                    new[k].append((t, stamp(r.header.stamp), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        d.update({k: np.array(v) for k, v in new.items()})
        pickle.dump(d, open(key, "wb"))
    return d


def umeyama(X, Y):
    mx, my = X.mean(0), Y.mean(0)
    Xc, Yc = X - mx, Y - my
    U, S, Vt = np.linalg.svd(Yc.T @ Xc / len(X))
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    k = np.trace(np.diag(S) @ D) / (Xc ** 2).sum(1).mean()
    r = np.linalg.norm((k * (R @ Xc.T)).T - Yc, axis=1)
    return k, R, r


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--bag", required=True)
    ap.add_argument("--slam", help="replay_slam.sh output to use instead of the flight's SLAM")
    ap.add_argument("--frame", required=True, choices=sorted(CONVERT))
    ap.add_argument("--label")
    a = ap.parse_args()
    label = a.label or os.path.basename((a.slam or a.bag).rstrip("/"))

    fl = read(a.bag, ["/bluerov2/odometry", "/bluerov2/odometry/dr", "/bluerov2/dvl_twist",
                      "/bluerov2/robot_pose_slam", "/bluerov2/map_points"])
    sl = read(a.slam, ["/bluerov2/robot_pose_slam", "/bluerov2/map_points"]) if a.slam else fl
    g, dr = fl["/bluerov2/odometry"], fl["/bluerov2/odometry/dr"]
    s, mp = sl["/bluerov2/robot_pose_slam"], sl["/bluerov2/map_points"].copy()
    # put everything on the camera / sim clock: truth and DR stamps are sim time
    g = g[:, [1, 2, 3, 4, 5, 6, 7, 8]]          # t_stamp, x, y, z, q
    dr = dr[:, [1, 2, 3, 4, 5, 6, 7, 8]]
    shift = np.median(s[:, 0] - s[:, 1])          # log time - image stamp of SLAM poses
    lat = np.median(fl["/bluerov2/robot_pose_slam"][:, 0] - fl["/bluerov2/robot_pose_slam"][:, 1])
    mp[:, 0] -= shift - lat                       # replay wall clock -> flight log clock (no-op on a flight)
    mpt = mp[:, 0] - lat                          # -> camera clock
    T = s[:, 1]
    t0 = g[0, 0]
    out = dict(label=label, bag=os.path.basename(a.bag.rstrip("/")), slam=a.slam and os.path.basename(a.slam.rstrip("/")),
               frame=a.frame, poses=int(len(T)), span_s=float(T[-1] - T[0]))

    from geometry_msgs.msg import Pose
    P, Y = [], []
    conv = CONVERT[a.frame]
    for r in s:
        p = Pose()
        p.position.x, p.position.y, p.position.z = r[2:5]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = r[5:9]
        conv(p)
        o = p.orientation
        P.append((p.position.x, p.position.y, p.position.z))
        Y.append(yaw(o.x, o.y, o.z, o.w))
    P = np.array(P)
    G = np.stack([np.interp(T, g[:, 0], g[:, i]) for i in (1, 2, 3)], 1)
    crash = [mpt[i] for i in range(1, len(mp)) if mp[i - 1, 2] >= 2000 and mp[i, 2] < 0.5 * mp[i - 1, 2]]
    jumps = [T[i] for i in range(1, len(T)) if np.linalg.norm(P[i] - P[i - 1]) > 3]
    out.update(crashes_s=[round(c - t0, 1) for c in crash], jumps_s=[round(j - t0, 1) for j in jumps])
    print(f"{label}: {len(T)} poses over {T[-1]-T[0]:.0f} s; map crashes at {out['crashes_s']} s, jumps > 3 m at {out['jumps_s']} s")

    edges = sorted(set([T[0] - 1] + crash + jumps + [T[-1] + 1]))
    segs = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (T > lo) & (T < hi)
        if m.sum() < 200:
            continue
        k, R, r = umeyama(P[m], G[m])
        segs.append(dict(t0=round(lo - t0), t1=round(hi - t0), n=int(m.sum()),
                         path_m=float(np.linalg.norm(np.diff(G[m], axis=0), axis=1).sum()),
                         scale=float(k), tilt_deg=float(math.degrees(math.acos(np.clip(R[2, 2], -1, 1)))),
                         rms_m=float(np.sqrt((r ** 2).mean())), max_m=float(r.max())))
    out["segments"] = segs
    print("rigidity, one similarity per map segment:")
    for sg in segs:
        print(f"  {sg['t0']:6d}-{sg['t1']:6d} s  path {sg['path_m']:5.0f} m  scale {sg['scale']:.3f}  "
              f"tilt {sg['tilt_deg']:4.1f} deg  rms {sg['rms_m']:5.2f} m  max {sg['max_m']:5.2f} m")

    loc = []
    for lo in np.arange(T[0], T[-1] - 30, 15):
        m = (T >= lo) & (T < lo + 30)
        if m.sum() < 150 or any(lo < e < lo + 30 for e in crash + jumps):
            continue
        k, _, r = umeyama(P[m], G[m])
        loc.append((k, np.sqrt((r ** 2).mean())))
    loc = np.array(loc)
    out["local30"] = dict(n=int(len(loc)), scale_med=float(np.median(loc[:, 0])), rms_med=float(np.median(loc[:, 1])),
                          rms_p90=float(np.percentile(loc[:, 1], 90)))
    print(f"local, 30 s windows: n={len(loc)}  scale med {out['local30']['scale_med']:.3f}  "
          f"rms med {out['local30']['rms_med']:.2f} m, p90 {out['local30']['rms_p90']:.2f} m")

    off = np.degrees(np.interp(T, dr[:, 0], np.unwrap([yaw(*r[4:8]) for r in dr])) - np.unwrap(Y))
    W = []
    for lo in np.arange(T[0], T[-1] - 10, 10):
        if any(lo - 5 < e < lo + 15 for e in crash + jumps):
            continue
        m = (T >= lo) & (T < lo + 10)
        if m.sum() < 50 or np.abs(np.diff(off[m])).max() > 5:
            continue
        gm = (g[:, 0] >= lo) & (g[:, 0] < lo + 10)
        Q = g[gm]
        W.append((off[m][-1] - off[m][0], abs(Q[-1, 3] - Q[0, 3]) / 10, math.hypot(Q[-1, 1] - Q[0, 1], Q[-1, 2] - Q[0, 2]) / 10))
    W = np.array(W)
    out["heading"] = dict(abs_mean_deg_per_10s=float(np.abs(W[:, 0]).mean()), net_deg=float(W[:, 0].sum()),
                          vertical=float(np.abs(W[W[:, 1] > 0.25, 0]).mean()),
                          horizontal=float(np.abs(W[(W[:, 2] > 0.15) & (W[:, 1] < 0.1), 0]).mean()))
    print(f"heading vs IMU: |drift| {out['heading']['abs_mean_deg_per_10s']:.2f} deg/10 s "
          f"(vertical {out['heading']['vertical']:.2f}, horizontal {out['heading']['horizontal']:.2f}), "
          f"net {out['heading']['net_deg']:+.1f} deg")

    # the bridge, replayed on the camera clock against the flight's reference filter
    ev = [(r[1], 0, tuple(r[2:9])) for r in s]
    ev += [(t, 1, n) for t, n in zip(mpt, mp[:, 2])]
    ev += [(r[0], 2, tuple(r[1:4])) for r in dr]
    ev.sort(key=lambda e: (e[0], e[1]))
    bo, _ = rb.replay(ev, a.frame, "quarantine")
    eb, edr = rb.err(bo, g[:, :3]), rb.err(dr[:, :3], g[:, :3])
    rows = []
    for lo in range(0, int(g[-1, 0] - t0), 180):
        mb = (bo[:, 0] - t0 >= lo) & (bo[:, 0] - t0 < lo + 180) if len(bo) else np.array([], bool)
        md = (dr[:, 0] - t0 >= lo) & (dr[:, 0] - t0 < lo + 180)
        rows.append(dict(t=lo, bridge_n=int(mb.sum()), bridge_med=float(np.median(eb[mb])) if mb.any() else None,
                         dr_med=float(np.median(edr[md])) if md.any() else None))
    out["bridge"] = dict(published=int(len(bo)), med=float(np.median(eb)) if len(eb) else None,
                         p90=float(np.percentile(eb, 90)) if len(eb) else None, per3min=rows)
    print(f"bridge: {len(bo)} poses published" + (f", error med {out['bridge']['med']:.2f} m, p90 {out['bridge']['p90']:.2f} m" if len(eb) else ""))
    for r in rows:
        b = f"{r['bridge_med']:5.2f}" if r["bridge_med"] is not None else "  -  "
        d = f"{r['dr_med']:5.2f}" if r["dr_med"] is not None else "  -  "
        print(f"  t={r['t']:5d} s  bridge {b} m (n={r['bridge_n']:4d})   reference filter {d} m")
    path = os.path.join(HERE, "figures", f"slam_quality_{label}.json")
    json.dump(out, open(path, "w"), indent=1)
    print("wrote", path)


if __name__ == "__main__":
    main()
