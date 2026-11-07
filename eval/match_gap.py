#!/usr/bin/env python3
"""How long before a match was the matched keyframe made, and would a real
gyro's heading still tell a wrong-face match from a benign one after that
long?

The heading veto compares the turn a match implies with the turn the IMU
measured between the two keyframes the match joins. Its error is therefore
the heading sensor's error accumulated between those two times, not across
the 2 s heading_check.py looks at. ORB-SLAM3 does not log which keyframe it
matched, so this estimates it from the bags:

  the matched keyframe Km was made when the vehicle truly stood where the
  estimate lands after the jump, facing the way it faces after the jump.

For each closure closure_events classifies, the post-jump point (its
r_before, dr and bearing_after) and the post-jump heading (true heading at
the jump plus heading_check's step) are compared with ground truth earlier in
the run, at least EXCL_S before the jump (keyframes that recent are covisible
and excluded from place recognition). The earliest is not wanted; the closest
in position among those within POS_TOL and HEAD_TOL is taken as Km's time.
The in-loop patch (step 3) logs Km's real timestamp and replaces this.

The heading error of a strapdown gyro between the two times is modelled as
zero-mean Gaussian with variance

    (sigma_b * dt)^2 + (sigma_s * dpsi)^2 + arw^2 * dt

  sigma_b  residual gyro bias after the start-of-run bias calibration
  sigma_s  residual scale-factor error, applied to the net turn dpsi
           between the two times (the orbits make this large)
  arw      angle random walk

with per-grade numbers in GRADES (sources and assumptions there). From it,
per closure: the chance a benign match is vetoed (|err| > THRESH) and the
chance a wrong-face match passes (|step + err| < THRESH).

Usage:  python3 eval/match_gap.py
        (reads eval/figures/heading_check.json for the bags and steps;
         writes eval/figures/match_gap.json)
"""
import glob
import json
import math
import os
import sys

import numpy as np
from mcap_ros2.reader import read_ros2_messages

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import closure_events as ce  # noqa: E402
import heading_check as hc   # noqa: E402

EXCL_S = 30.0      # s    keyframes newer than this are covisible with the current one
POS_TOL = 3.0      # m    Km's true position within this of the post-jump point
HEAD_TOL = 30.0    # deg  and its true heading within this of the post-jump heading
THRESH = 45.0      # deg  the veto threshold: half the four-fold symmetry angle

DPS = 3600.0       # deg/h per deg/s
# sigma_b deg/h, sigma_s fraction, arw deg/sqrt(h)
GRADES = {
    # TDK ICM-20602 datasheet DS-000176 rev 1.0: ZRO tempco +-0.01 dps/degC,
    # scale factor +-1 % initial, noise 0.004 dps/rtHz (= 0.24 deg/rth).
    # ArduSub calibrates gyro bias at boot only; the residual is the tempco
    # times the temperature change since then. Scale factor is not
    # calibrated by ArduSub: the datasheet +-1 % is taken as 1 sigma.
    "MEMS, bias cal. on deck (dT 10 C)": (0.01 * 10 * DPS, 0.01, 0.24),
    "MEMS, bias cal. in water (dT 1 C)": (0.01 * 1 * DPS, 0.01, 0.24),
    # ADIS16495-1 datasheet: in-run bias stability 0.8 deg/h, ARW 0.09
    # deg/rth. The Allan floor is optimistic as a residual. Scale factor
    # 0.1 % is ASSUMED (factory calibrated; not checked against the sheet).
    "tactical MEMS (ADIS16495-class)": (0.8, 0.001, 0.09),
    # KVH 1775 brochure (2015): bias instability <=0.05 deg/h, ARW
    # <=0.012 deg/rth, bias offset +-0.5 deg/h max (used here). Scale factor
    # 0.01 % is ASSUMED.
    "FOG (KVH 1775-class)": (0.5, 0.0001, 0.012),
}


def yaw(o):
    return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))


def truth(bag):
    r = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/odometry"]):
            p = m.ros_msg.pose.pose
            r.append((m.log_time.timestamp(), p.position.x, p.position.y, yaw(p.orientation)))
    r = np.array(r)
    r[:, 3] = np.degrees(np.unwrap(r[:, 3]))
    return r


def sigma(g, dt, dpsi):
    b, s, arw = g
    return math.sqrt((b * dt / 3600.0) ** 2 + (s * dpsi) ** 2 + arw ** 2 * dt / 3600.0)


def phi(x):
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


def p_out(mu, sd, lo, hi):
    """P(lo < N(mu, sd) < hi)."""
    return phi((hi - mu) / sd) - phi((lo - mu) / sd)


def match_time(tr, tj, a, head):
    i = int(np.searchsorted(tr[:, 0], tj))
    ok = tr[:, 0] <= tj - EXCL_S
    dist = np.hypot(tr[:, 1] - a[0], tr[:, 2] - a[1])
    dh = np.abs((tr[:, 3] - head + 180.0) % 360.0 - 180.0)
    c = np.flatnonzero(ok & (dist <= POS_TOL) & (dh <= HEAD_TOL))
    if not len(c):
        k = int(np.flatnonzero(ok)[np.argmin(dist[ok])]) if ok.any() else None
        return None, (None if k is None else dict(dist=round(float(dist[k]), 2),
                                                  dhead=round(float(dh[k]), 1)))
    k = int(c[np.argmin(dist[c])])
    return k, dict(dist=round(float(dist[k]), 2), dhead=round(float(dh[k]), 1),
                   dt=round(float(tj - tr[k, 0]), 1),
                   dpsi=round(float(tr[min(i, len(tr) - 1), 3] - tr[k, 3]), 1))


def main():
    hcj = json.load(open(os.path.join(HERE, "figures", "heading_check.json")))
    out = []
    for b in hcj:
        rows = [r for r in b["loop_classes"]
                if r.get("cls") in ("benign", "face-aliased", "other-large")]
        if not rows:
            continue
        bag = os.path.join(hc.BAG_ROOT, b["bag"])
        crow, _ = ce.classify(bag, ce.pick_log(bag))
        tr = truth(bag)
        t0 = ce.track(bag, "/bluerov2/odometry", False)[0, 0]
        for r in rows:
            c = next(x for x in crow if abs(x["t"] - r["t"]) < 0.1)
            if r["heading_step"] is None:
                continue
            tj = t0 + c["t"]
            ra = c["r_before"] * (1.0 + c["dr"])
            a = (ce.TURBINE[0] + ra * math.cos(math.radians(c["bearing_after"])),
                 ce.TURBINE[1] + ra * math.sin(math.radians(c["bearing_after"])))
            i = int(np.searchsorted(tr[:, 0], tj))
            head = tr[min(i, len(tr) - 1), 3] + r["heading_step"]
            k, m = match_time(tr, tj, a, head)
            row = dict(bag=b["bag"], t=r["t"], cls=r["cls"], heading_step=r["heading_step"], km=m)
            if k is not None:
                step = r["heading_step"] if r["cls"] != "benign" else 0.0
                for name, g in GRADES.items():
                    sd = sigma(g, m["dt"], m["dpsi"])
                    if r["cls"] == "benign":
                        p = 1.0 - p_out(0.0, sd, -THRESH, THRESH)       # vetoed in error
                    else:
                        p = p_out(step, sd, -THRESH, THRESH)            # passes in error
                    row.setdefault("grades", {})[name] = dict(sd=round(sd, 2), p_wrong=float(f"{p:.2e}"))
            out.append(row)
            print(json.dumps(row))
    json.dump(out, open(os.path.join(HERE, "figures", "match_gap.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
