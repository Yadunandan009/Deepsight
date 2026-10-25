#!/usr/bin/env python3
"""Compare underwater image enhancers as SLAM front ends, offline.

The test registered in docs/experiments/PREREG_enhancement.md. Needs a bag
recorded with IMAGES=1 (record_run.sh). Each method is applied to the
rendered left-camera frames; frames square-on to a face at 6-22 m are scored
with ORB-SLAM3's ORB settings on the SAME sampled pairs for every method:

  orb    keypoints per frame
  same   RANSAC inliers between frames of one face >= 60 s apart (revisits)
  cross  RANSAC inliers between frames of different faces (aliasing)
  ratio  cross / same
  ms     time per 640x480 frame, this machine, one process

Methods: raw, current (the live node's own enhance()), fusion (Ancuti et al.
2018, colour balance and multi-scale fusion), udcp (Drews et al. 2013,
underwater dark channel prior). Candidate parameters are the papers', not
tuned here.

Run:  python3 eval/enhance_test.py BAG    (writes eval/figures/enhance_test.json
                                            and fig_enhance_test.{pdf,png})
"""
import glob
import itertools
import json
import os
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import image_depth as idp  # noqa: E402  (state(), inliers(), FACE_TOL, REVISIT_GAP)
from mcap_ros2.reader import read_ros2_messages  # noqa: E402

ORB = cv2.ORB_create(nfeatures=1500, scaleFactor=1.2, nlevels=8, fastThreshold=20)
idp.ORB = ORB
Z_LO, Z_HI = 6.0, 22.0
MAX_PAIRS = 300                                 # per kind, pooled over 6-22 m
rng = np.random.default_rng(0)


# ---------------------------------------------------------------- methods
def _current():
    """The live node's enhance(), lifted out of its source and called on a
    stand-in for the node, so the code under test is the code that flies
    (without starting ROS)."""
    path = os.path.join(os.path.dirname(HERE), "src", "stonefish_bluerov2", "scripts",
                        "image_enhancement_node.py")
    body = open(path).read().split("    def enhance(self, img):")[1].split("\ndef main")[0]
    ns = {"cv2": cv2, "np": np}
    exec("def enhance(self, img):" + body, ns)
    me = SimpleNamespace(denoise=True, sharpen_amount=0.6, fine_sharpen_amount=0.4,
                         clahe=cv2.createCLAHE(clipLimit=4.5, tileGridSize=(6, 6)),
                         _gamma_lut=np.array([((i / 255.0) ** 0.7) * 255
                                              for i in range(256)]).astype(np.uint8))
    return lambda img: ns["enhance"](me, img)


def fusion(img):
    """Ancuti, Ancuti, De Vleeschouwer & Bekaert, IEEE TIP 27(1) 2018."""
    I = img.astype(np.float32) / 255.0
    b, g, r = cv2.split(I)
    r = r + 1.0 * (g.mean() - r.mean()) * (1.0 - r) * g          # red compensation
    I = cv2.merge((b, g, np.clip(r, 0, 1)))
    I = I * (I.reshape(-1, 3).mean() / (I.reshape(-1, 3).mean(0) + 1e-6))  # grey world
    I = np.clip(I, 0, 1)
    in1 = np.power(I, 2.0)                                          # gamma-corrected
    G = cv2.GaussianBlur(I, (0, 0), 3)
    d = I - G
    d = (d - d.min()) / (d.max() - d.min() + 1e-6)                  # normalised unsharp
    in2 = (I + d) / 2.0
    ws = []
    for x in (in1, in2):
        L = cv2.cvtColor(x, cv2.COLOR_BGR2GRAY)
        wl = np.abs(cv2.Laplacian(L, cv2.CV_32F))
        lab = cv2.cvtColor(x, cv2.COLOR_BGR2LAB)
        wsal = np.linalg.norm(lab.reshape(-1, 3).mean(0) - cv2.GaussianBlur(lab, (5, 5), 0), axis=2)
        wsat = np.sqrt(((x - L[..., None]) ** 2).mean(2))
        ws.append(wl + wsal + wsat)
    W = np.stack(ws) + 0.1
    W = W / W.sum(0)
    lev = 5

    def gpyr(a):
        p = [a]
        for _ in range(lev - 1):
            p.append(cv2.pyrDown(p[-1]))
        return p

    def lpyr(a):
        gp = gpyr(a)
        return [gp[i] - cv2.pyrUp(gp[i + 1], dstsize=gp[i].shape[1::-1]) for i in range(lev - 1)] + [gp[-1]]

    out = None
    for x, w in zip((in1, in2), W):
        lp, gw = lpyr(x), gpyr(w)
        f = [l * gg[..., None] for l, gg in zip(lp, gw)]
        out = f if out is None else [o + ff for o, ff in zip(out, f)]
    R = out[-1]
    for i in range(lev - 2, -1, -1):
        R = cv2.pyrUp(R, dstsize=out[i].shape[1::-1]) + out[i]
    return (np.clip(R, 0, 1) * 255).astype(np.uint8)


def _guided(I, p, r=30, eps=1e-3):
    m = lambda x: cv2.boxFilter(x, -1, (r, r))  # noqa: E731
    mI, mp = m(I), m(p)
    a = (m(I * p) - mI * mp) / (m(I * I) - mI * mI + eps)
    b = mp - a * mI
    return m(a) * I + m(b)


def udcp(img, patch=15, omega=0.95, t0=0.1):
    """Drews, Nascimento, Moraes, Botelho & Campos, ICCV workshops 2013."""
    I = img.astype(np.float32) / 255.0
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (patch, patch))
    dark = cv2.erode(np.minimum(I[..., 0], I[..., 1]), k)          # B, G only
    n = max(1, dark.size // 1000)
    idx = np.argsort(dark.ravel())[-n:]
    A = I.reshape(-1, 3)[idx].mean(0)
    t = 1.0 - omega * cv2.erode(np.minimum(I[..., 0] / A[0], I[..., 1] / A[1]), k)
    t = _guided(cv2.cvtColor(I, cv2.COLOR_BGR2GRAY), t.astype(np.float32))
    J = (I - A) / np.maximum(t, t0)[..., None] + A
    return (np.clip(J, 0, 1) * 255).astype(np.uint8)


# ---------------------------------------------------------------- data
def bgr(msg):
    a = np.frombuffer(bytes(msg.data), np.uint8)
    h, w = msg.height, msg.width
    img = a.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding.lower().startswith("rgb") else img.copy()


def read(bag):
    truth, raw, rec = [], [], []
    tops = ["/bluerov2/odometry", "/bluerov2/left/image_color_rec", "/bluerov2/left/image_enhanced_rec"]
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=tops):
            t = m.log_time.timestamp()
            if m.channel.topic == tops[0]:
                p = m.ros_msg.pose.pose.position
                truth.append((t, p.x, p.y, abs(p.z)))
            elif m.channel.topic == tops[1]:
                raw.append((t, bgr(m.ros_msg)))
            else:
                rec.append((t, idp.to_grey(m.ros_msg)))
    return np.array(truth), raw, rec


def main():
    bag = sys.argv[1]
    tr, raw, rec = read(bag)
    if not raw:
        sys.exit("no /bluerov2/left/image_color_rec in this bag: record with IMAGES=1")
    cur = _current()

    # Check the stand-in reproduces the live node: recorded enhanced frames
    # against current() on the nearest rendered frame.
    rt = np.array([t for t, _ in raw])
    diffs = []
    for t, g in rec[::25]:
        i = int(np.argmin(np.abs(rt - t)))
        if abs(rt[i] - t) < 0.05:
            diffs.append(float(np.abs(cv2.cvtColor(cur(raw[i][1]), cv2.COLOR_BGR2GRAY).astype(int) - g).mean()))
    print("current() vs recorded enhanced frames: mean |grey diff| %.2f over %d frames" %
          (np.median(diffs) if diffs else float("nan"), len(diffs)))

    frames = []
    for t, img in raw:
        s = idp.state(tr, t)
        s["t"] = t - tr[0, 0]
        if s["face_off"] <= idp.FACE_TOL and Z_LO <= s["z"] <= Z_HI:
            frames.append((s, img))
    print(f"{len(frames)} square-on frames at {Z_LO:.0f}-{Z_HI:.0f} m")
    idx = range(len(frames))
    same = [(a, b) for a, b in itertools.combinations(idx, 2)
            if frames[a][0]["face"] == frames[b][0]["face"]
            and abs(frames[a][0]["t"] - frames[b][0]["t"]) >= idp.REVISIT_GAP]
    cross = [(a, b) for a, b in itertools.combinations(idx, 2)
             if frames[a][0]["face"] != frames[b][0]["face"]]
    same = [same[i] for i in rng.choice(len(same), min(MAX_PAIRS, len(same)), replace=False)]
    cross = [cross[i] for i in rng.choice(len(cross), min(MAX_PAIRS, len(cross)), replace=False)]

    methods = {"raw": lambda x: x, "current": cur, "fusion": fusion, "udcp": udcp}
    res, per_depth = {}, {}
    for name, fn in methods.items():
        t0 = time.perf_counter()
        out = [fn(img) for _, img in frames]
        ms = 1000 * (time.perf_counter() - t0) / len(frames)
        feats = [ORB.detectAndCompute(cv2.cvtColor(o, cv2.COLOR_BGR2GRAY), None) for o in out]
        orb = [len(k) for k, _ in feats]
        s_in = [idp.inliers(feats[a][1], feats[a][0], feats[b][1], feats[b][0]) for a, b in same]
        c_in = [idp.inliers(feats[a][1], feats[a][0], feats[b][1], feats[b][0]) for a, b in cross]
        med_s, med_c = float(np.median(s_in)), float(np.median(c_in))
        res[name] = dict(ms=ms, orb=float(np.median(orb)), same=med_s, cross=med_c,
                         ratio=med_c / med_s if med_s else None, n_same=len(s_in), n_cross=len(c_in))
        z = np.array([f[0]["z"] for f in frames])
        per_depth[name] = [(lo, float(np.median(np.array(orb)[(z >= lo) & (z < lo + 2)])))
                           for lo in range(6, 22, 2) if ((z >= lo) & (z < lo + 2)).any()]
        print("%-8s %6.1f ms  ORB %5.0f  same %5.1f  cross %5.1f  ratio %s" % (
            name, ms, res[name]["orb"], med_s, med_c,
            "%.2f" % res[name]["ratio"] if res[name]["ratio"] else "-"))
        if name != "raw":
            cv2.imwrite(os.path.join(HERE, "figures", f"enhance_example_{name}.png"),
                        out[len(out) // 2])
    cv2.imwrite(os.path.join(HERE, "figures", "enhance_example_raw.png"), frames[len(frames) // 2][1])

    c = res["current"]
    for name in ("fusion", "udcp"):
        r = res[name]
        rule = dict(same=r["same"] >= 1.2 * c["same"], orb=r["orb"] >= c["orb"],
                    ratio=r["ratio"] is not None and c["ratio"] is not None and r["ratio"] <= c["ratio"] + 0.05,
                    time=r["ms"] <= 40.0)
        r["rule"] = rule
        r["beats_current"] = all(rule.values())
        print(f"{name}: {rule} -> {'BEATS current' if r['beats_current'] else 'does not beat current'}")
    json.dump(dict(bag=os.path.basename(bag.rstrip('/')), frames=len(frames), results=res,
                   orb_by_depth=per_depth), open(os.path.join(HERE, "figures", "enhance_test.json"), "w"),
              indent=1)


if __name__ == "__main__":
    main()
