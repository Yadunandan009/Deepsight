#!/usr/bin/env python3
"""Why do wrong-face closures happen at the seabed end? Test it on camera frames.

Needs a bag recorded with IMAGES=1 (record_run.sh), which holds the left camera
as rendered (/bluerov2/left/image_color_rec) and as ORB-SLAM3 receives it
(/bluerov2/left/image_enhanced_rec), throttled to about 1 Hz, plus ground truth.

Per frame, at the vehicle's TRUE depth and bearing about the structure:
  brightness  mean grey level of the rendered image (0-255)
  contrast    standard deviation of the grey level
  orb         ORB keypoints in the enhanced image, with ORB-SLAM3's settings
              (1500 features, scale 1.2, 8 levels)

Then the test registered in docs/experiments/PREREG_images_depth.md: frames
taken square-on to a face (within 10 deg of its centre) are grouped into 2 m
depth bins, and pairs are matched as a loop-closure candidate would be (ORB
descriptors, ratio test, fundamental-matrix RANSAC). For each bin:
  cross   median RANSAC inliers between frames of DIFFERENT faces (aliasing)
  same    median inliers between frames of the SAME face taken at least 60 s
          apart (a true revisit)
  ratio   cross / same
A ratio near 1 means another face looks as much like this place as the place
itself does, which is what a wrong-face closure needs.

Run:  python3 eval/image_depth.py BAG            (writes eval/figures/image_depth.json
                                                  and eval/figures/fig_image_depth.{pdf,png})
"""
import glob
import itertools
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mcap_ros2.reader import read_ros2_messages  # noqa: E402
import closure_events as ce  # noqa: E402

FACES = np.array([180.0, 90.0, 0.0, -90.0])     # face centres, bearing about the axis (deg)
FACE_TOL = 10.0                                  # deg from a face centre to count as square-on
BIN = 2.0                                        # m, depth bin
REVISIT_GAP = 60.0                               # s, minimum time between same-face frames
MAX_PAIRS = 60                                   # per bin and kind, sampled for speed
ORB = cv2.ORB_create(nfeatures=1500, scaleFactor=1.2, nlevels=8)
BF = cv2.BFMatcher(cv2.NORM_HAMMING)
rng = np.random.default_rng(0)


def to_grey(msg):
    a = np.frombuffer(bytes(msg.data), np.uint8)
    h, w = msg.height, msg.width
    if msg.encoding in ("mono8", "8UC1"):
        return a.reshape(h, msg.step)[:, :w]
    img = a.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
    code = cv2.COLOR_RGB2GRAY if msg.encoding.lower().startswith("rgb") else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(img, code)


def read(bag):
    truth, frames = [], {"color": [], "enh": []}
    topics = {"/bluerov2/odometry": None, "/bluerov2/left/image_color_rec": "color",
              "/bluerov2/left/image_enhanced_rec": "enh"}
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=list(topics)):
            t = m.log_time.timestamp() if hasattr(m.log_time, "timestamp") else m.log_time / 1e9
            k = topics[m.channel.topic]
            if k is None:
                p = m.ros_msg.pose.pose.position
                truth.append((t, p.x, p.y, abs(p.z)))
            else:
                frames[k].append((t, to_grey(m.ros_msg)))
    return np.array(truth), frames


def state(tr, t):
    x, y, z = (np.interp(t, tr[:, 0], tr[:, i]) for i in (1, 2, 3))
    b = math.degrees(math.atan2(y - ce.TURBINE[1], x - ce.TURBINE[0]))
    off = [abs(ce.wrap_deg(b - c)) for c in FACES]
    k = int(np.argmin(off))
    return dict(z=float(z), bearing=b, face=k, face_off=off[k])


def inliers(d1, k1, d2, k2):
    if d1 is None or d2 is None or len(k1) < 8 or len(k2) < 8:
        return 0
    good = [m for m, n in (p for p in BF.knnMatch(d1, d2, k=2) if len(p) == 2) if m.distance < 0.8 * n.distance]
    if len(good) < 8:
        return 0
    p1 = np.float32([k1[m.queryIdx].pt for m in good])
    p2 = np.float32([k2[m.trainIdx].pt for m in good])
    _, mask = cv2.findFundamentalMat(p1, p2, cv2.FM_RANSAC, 1.0, 0.99)
    return int(mask.sum()) if mask is not None else 0


def main():
    bag = sys.argv[1]
    tr, fr = read(bag)
    if not fr["enh"]:
        sys.exit("no /bluerov2/left/image_enhanced_rec in this bag: record with IMAGES=1")
    color = dict(fr["color"])
    rows = []
    for t, g in fr["enh"]:
        s = state(tr, t)
        c = color.get(t)
        if c is None and color:                      # nearest rendered frame
            tc = min(color, key=lambda u: abs(u - t)); c = color[tc] if abs(tc - t) < 0.6 else None
        kp, de = ORB.detectAndCompute(g, None)
        s.update(t=t - tr[0, 0], orb=len(kp), brightness=float(c.mean()) if c is not None else None,
                 contrast=float(c.std()) if c is not None else None)
        rows.append((s, kp, de))
    on = [r for r in rows if r[0]["face_off"] <= FACE_TOL and 1.0 <= r[0]["z"] <= 22.0]
    bins = {}
    for r in on:
        bins.setdefault(int(r[0]["z"] // BIN), []).append(r)
    out = []
    for b in sorted(bins):
        rs = bins[b]
        cross = [(a, c) for a, c in itertools.combinations(rs, 2) if a[0]["face"] != c[0]["face"]]
        same = [(a, c) for a, c in itertools.combinations(rs, 2)
                if a[0]["face"] == c[0]["face"] and abs(a[0]["t"] - c[0]["t"]) >= REVISIT_GAP]
        res = {}
        for name, pairs in (("cross", cross), ("same", same)):
            if len(pairs) > MAX_PAIRS:
                pairs = [pairs[i] for i in rng.choice(len(pairs), MAX_PAIRS, replace=False)]
            v = [inliers(a[2], a[1], c[2], c[1]) for a, c in pairs]
            res[name] = float(np.median(v)) if v else None
            res["n_" + name] = len(v)
        res["ratio"] = (res["cross"] / res["same"]) if res["cross"] is not None and res["same"] else None
        out.append(dict(depth_lo=b * BIN, depth_hi=(b + 1) * BIN, frames=len(rs), **res))
        print("%4.0f-%-4.0f m  frames %3d  cross %6s (n %3d)  same %6s (n %3d)  ratio %s" % (
            b * BIN, (b + 1) * BIN, len(rs), res["cross"], res["n_cross"], res["same"], res["n_same"],
            "%.2f" % res["ratio"] if res["ratio"] is not None else "-"))
    per_frame = [r[0] for r in rows]
    json.dump(dict(bag=os.path.basename(bag.rstrip("/")), frames=per_frame, bins=out),
              open(os.path.join(HERE, "figures", "image_depth.json"), "w"), indent=1)

    import matplotlib.pyplot as plt
    from pubstyle import use_journal, save, COL2, SERIES
    use_journal()
    fig, ax = plt.subplots(1, 3, figsize=(COL2, 2.6), sharey=True)
    z = np.array([r["z"] for r in per_frame])
    for a, key, lab in ((ax[0], "brightness", "mean grey level (rendered)"), (ax[1], "orb", "ORB keypoints (enhanced)")):
        v = np.array([r[key] if r[key] is not None else np.nan for r in per_frame], float)
        a.scatter(v, z, s=3, color=SERIES[0], alpha=0.5, linewidths=0)
        a.set_xlabel(lab)
    zb = [(o["depth_lo"] + o["depth_hi"]) / 2 for o in out]
    ax[2].plot([o["cross"] for o in out], zb, "-o", ms=3, color=SERIES[1], label="different faces")
    ax[2].plot([o["same"] for o in out], zb, "-o", ms=3, color=SERIES[0], label="same face, revisit")
    ax[2].set_xlabel("median RANSAC inliers per pair")
    ax[2].legend(fontsize=6, frameon=False)
    ax[0].set_ylabel("true depth (m)")
    ax[0].set_ylim(23, 0)
    for a in ax:
        a.axhline(20, color="#999", lw=0.6, ls=(0, (3, 2)))
    save(fig, os.path.join(HERE, "figures", "fig_image_depth"))
    print("wrote eval/figures/image_depth.json and fig_image_depth.{pdf,png}")


if __name__ == "__main__":
    main()
