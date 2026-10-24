#!/usr/bin/env python3
"""Does the bridge's 5 m consistency gate set the ceiling on the fused error?

The bridge (slam_pose_bridge.py, EKF_CONSISTENCY_MAX = 5.0) rejects a SLAM pose
that is more than 5 m from its reference, which with dr_reference:=true is the
SLAM-free dead-reckoned filter. A SLAM pose that is wrong by 4.9 m and stays
there therefore passes every frame, and the primary filter is told to believe it
(variance floor 0.05 m^2). If that is what happens, the primary filter's error
should rise to just under the gate and stay there, while the dead-reckoned filter
stays within tenths of a metre, and the bridge's OUTPUT should sit inside the
gate relative to dead reckoning but far from the truth.

For every flight in the manifest this reads
    /bluerov2/robot_pose_slam_ekf  (what the bridge publishes; the primary filter's SLAM input)
    /bluerov2/odometry/dr          (the gate's reference)
    /bluerov2/odometry/filtered    (the primary filter)
    /bluerov2/odometry             (ground truth, scoring only)
and writes eval/figures/gate_plateau.json with, per flight, the largest and the
final distance of the bridge output from dead reckoning (the gate's own quantity),
from truth, and the primary filter's error. Figure: fig_gate_plateau.py.
"""
import csv
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from mcap_ros2.reader import read_ros2_messages  # noqa: E402

TOP = {"bridge": "/bluerov2/robot_pose_slam_ekf", "dr": "/bluerov2/odometry/dr",
       "filt": "/bluerov2/odometry/filtered", "truth": "/bluerov2/odometry"}
OUT = os.path.join(HERE, "figures", "gate_plateau.json")
OUTN = os.path.join(HERE, "figures", "gate_plateau_series.npz")


def read(bag):
    d = {k: [] for k in TOP}
    inv = {v: k for k, v in TOP.items()}
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=list(TOP.values())):
            t = m.log_time.timestamp() if hasattr(m.log_time, "timestamp") else m.log_time / 1e9
            r = m.ros_msg
            p = r.pose.pose.position
            d[inv[m.channel.topic]].append((t, p.x, p.y))
    return {k: np.array(v) for k, v in d.items()}


def main():
    flights = list(csv.DictReader(open(os.path.join(ROOT, "docs", "experiments", "flights_manifest.csv"))))
    res, ser = {}, {}
    if os.path.exists(OUT) and "--refresh" not in sys.argv:
        res = json.load(open(OUT))
        ser = dict(np.load(OUTN))
    for fl in flights:
        bag = os.path.join(ROOT, fl["bag"])
        if fl["id"] in res or not os.path.isdir(bag):
            continue
        print("reading", fl["id"], flush=True)
        D = read(bag)
        t0 = D["truth"][0, 0]
        g = np.arange(0.0, (D["truth"][-1, 0] - t0), 1.0)

        def at(a):
            return np.column_stack([np.interp(g + t0, a[:, 0], a[:, 1]),
                                    np.interp(g + t0, a[:, 0], a[:, 2])])
        T, DR, F = at(D["truth"]), at(D["dr"]), at(D["filt"])
        B = D["bridge"]
        # the bridge publishes only while it is aligned and a pose passes the gate
        tb = B[:, 0] - t0
        b_dr = np.hypot(*(B[:, 1:3] - np.column_stack([np.interp(B[:, 0], D["dr"][:, 0], D["dr"][:, 1]),
                                                     np.interp(B[:, 0], D["dr"][:, 0], D["dr"][:, 2])])).T)
        b_tr = np.hypot(*(B[:, 1:3] - np.column_stack([np.interp(B[:, 0], D["truth"][:, 0], D["truth"][:, 1]),
                                                     np.interp(B[:, 0], D["truth"][:, 0], D["truth"][:, 2])])).T)
        e_f = np.hypot(*(F - T).T)
        e_d = np.hypot(*(DR - T).T)
        late = tb > 0.0
        res[fl["id"]] = dict(
            n_bridge=int(len(B)), first_bridge_s=float(tb[0]) if len(tb) else None,
            bridge_to_dr_max=float(b_dr.max()), bridge_to_dr_p99=float(np.percentile(b_dr, 99)),
            bridge_to_truth_max=float(b_tr.max()), bridge_to_truth_p99=float(np.percentile(b_tr, 99)),
            frac_bridge_beyond_gate=float(np.mean(b_dr > 5.0)),
            filt_err_max=float(e_f.max()), filt_err_final=float(e_f[-1]),
            dr_err_max=float(e_d.max()), dr_err_final=float(e_d[-1]))
        ser[fl["id"] + "_g"], ser[fl["id"] + "_ef"], ser[fl["id"] + "_ed"] = g, e_f, e_d
        ser[fl["id"] + "_tb"], ser[fl["id"] + "_bdr"], ser[fl["id"] + "_btr"] = tb, b_dr, b_tr
        json.dump(res, open(OUT, "w"), indent=1)
        np.savez(OUTN, **ser)
    for k, v in res.items():
        print("%-11s bridge-DR max %.2f (p99 %.2f)  bridge-truth max %.2f  filt err max %.2f final %.2f  DR final %.2f"
              % (k, v["bridge_to_dr_max"], v["bridge_to_dr_p99"], v["bridge_to_truth_max"],
                 v["filt_err_max"], v["filt_err_final"], v["dr_err_final"]))


if __name__ == "__main__":
    main()
