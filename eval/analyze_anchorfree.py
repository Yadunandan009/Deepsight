#!/usr/bin/env python3
"""
Anchor-free run analysis: how fast does the estimate drift, and why?

Produces the numbers Phase 1.4 of the paper plan calls the reference line --
dead-reckoning drift as a percentage of distance travelled, ATE, and the
bearing error that actually decides whether the mission works -- plus the
diagnostics that say which input caused it.

WHAT IT SEPARATES, and why that is the point. Bearing is
wrap(atan2(TY - py, TX - px) - yaw) over the FILTER's own position and
heading, so it has exactly two inputs and they fail for different reasons
and need different fixes. Position error alone cannot tell them apart: an
estimate can be metres off along the line to the structure and still aim
correctly at it, or be close in position and badly mis-aimed. This script
therefore reports bearing, yaw and position side by side, and splits the
run at the first SLAM alignment so the pure dead-reckoning segment is
measured on its own rather than averaged together with the corrected one.

It also checks the DVL against ground truth per axis. A sign error there is
invisible to any magnitude check -- speed is identical either way -- and it
was the cause of the 2026-10-09 divergence, so the check is permanent.

Usage: python3 eval/analyze_anchorfree.py <bag_dir> [--fig]
"""
# --- interpreter shim -------------------------------------------------
# mcap_ros2 is installed only in venv-ardupilot, while env.sh deliberately
# STRIPS that venv from PATH so ros2 launch's child nodes get the system
# python and a working cv_bridge. So the interpreter that can run ROS nodes
# is exactly the one that cannot read bags, and vice versa. Rather than
# require the caller to remember which shell they are in, re-exec under the
# interpreter that has the reader. The guard variable stops this recursing
# if that interpreter also lacks it.
import os
import sys

try:
    import mcap_ros2  # noqa: F401
except ImportError:
    _alt = os.path.expanduser("~/venv-ardupilot/bin/python3")
    # Compared as literal paths, not realpath: a venv's bin/python3 is a
    # symlink to the system interpreter, so realpath collapses the two and
    # the re-exec never fires -- even though invoking it by the venv path
    # is precisely what puts the venv's site-packages on sys.path.
    if not os.environ.get("_BAGREADER_REEXEC") and os.path.exists(_alt) \
            and os.path.abspath(_alt) != os.path.abspath(sys.executable):
        os.environ["_BAGREADER_REEXEC"] = "1"
        os.execv(_alt, [_alt] + sys.argv)
    sys.exit("mcap_ros2 not found in any available interpreter; "
             "pip install mcap-ros2-support")
# ----------------------------------------------------------------------

import argparse
import glob
import math
import os
import sys

import numpy as np
from mcap_ros2.reader import read_ros2_messages

TURBINE_X, TURBINE_Y = 20.0, 0.0

T_TRUTH = "/bluerov2/odometry"
T_EST = "/bluerov2/odometry/filtered"
T_DVL = "/bluerov2/dvl_twist"
T_SLAM = "/bluerov2/robot_pose_slam_ekf"


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _iter_recovered(path, topics):
    """Stream a bag that has no footer or summary index.

    `ros2 bag record` writes the footer, the summary and metadata.yaml only
    on clean shutdown, so a recorder that is SIGKILLed -- or a machine that
    loses power mid-run -- leaves a file whose data chunks are entirely
    intact but which the normal indexed reader refuses outright. The records
    themselves are still a valid sequence, so they can be walked from the
    header and decoded with the schemas as they appear. Recovered 108,716 of
    a 110 MB bag on 2026-10-09 that would otherwise have cost a re-flight.
    """
    from mcap.stream_reader import StreamReader
    from mcap_ros2.decoder import DecoderFactory
    from mcap.records import Channel, Message, Schema

    factory = DecoderFactory()
    schemas, channels, decoders = {}, {}, {}
    with open(path, "rb") as fh:
        try:
            for rec in StreamReader(fh, skip_magic=False).records:
                if isinstance(rec, Schema):
                    schemas[rec.id] = rec
                elif isinstance(rec, Channel):
                    channels[rec.id] = rec
                elif isinstance(rec, Message):
                    ch = channels.get(rec.channel_id)
                    if ch is None or ch.topic not in topics:
                        continue
                    if ch.id not in decoders:
                        decoders[ch.id] = factory.decoder_for(
                            ch.message_encoding, schemas.get(ch.schema_id))
                    dec = decoders[ch.id]
                    if dec is None:
                        continue
                    yield ch.topic, rec.log_time / 1e9, dec(rec.data)
        except Exception:
            # Truncation is the expected ending here, not an error.
            return


def load(bag):
    files = sorted(glob.glob(os.path.join(bag, "*.mcap")))
    if not files:
        sys.exit(f"no .mcap under {bag}")
    out = {k: [] for k in (T_TRUTH, T_EST, T_DVL, T_SLAM)}
    for f in files:
        try:
            stream = [(m.channel.topic,
                       m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                       else m.log_time / 1e9,
                       m.ros_msg)
                      for m in read_ros2_messages(f, topics=list(out))]
        except Exception as e:
            print(f"  indexed read failed ({type(e).__name__}); "
                  f"recovering by streaming -- bag was not closed cleanly",
                  file=sys.stderr)
            stream = _iter_recovered(f, set(out))
        for topic, t, r in stream:
            if topic in (T_TRUTH, T_EST):
                p = r.pose.pose.position
                v = r.twist.twist.linear
                out[topic].append((t, p.x, p.y, p.z, yaw_of(r.pose.pose.orientation),
                                   v.x, v.y, v.z))
            elif topic == T_DVL:
                v = r.twist.twist.linear
                out[topic].append((t, v.x, v.y, v.z))
            elif topic == T_SLAM:
                out[topic].append((t,))
    return {k: np.array(v) for k, v in out.items() if v}


def interp_to(src_t, src_cols, dst_t, angle_cols=()):
    """Resample columns onto dst_t.

    Columns named in angle_cols are angles in radians and are interpolated
    through sin/cos.  Interpolating them linearly is wrong: when the vehicle
    sits on the +/-pi seam (it parks at yaw = 180 deg on one of the turbine
    faces) consecutive samples straddle +180 and -180, and the linear blend
    lands near 0, manufacturing a ~180 deg error out of nothing.
    """
    out = []
    for i, c in enumerate(src_cols.T):
        if i in angle_cols:
            s = np.interp(dst_t, src_t, np.sin(c))
            k = np.interp(dst_t, src_t, np.cos(c))
            out.append(np.arctan2(s, k))
        else:
            out.append(np.interp(dst_t, src_t, c))
    return np.column_stack(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--fig", action="store_true", help="also write a publication figure")
    args = ap.parse_args()

    D = load(args.bag)
    for need in (T_TRUTH, T_EST):
        if need not in D:
            sys.exit(f"{need} missing from bag")
    truth, est = D[T_TRUTH], D[T_EST]
    t0 = truth[0, 0]

    # Resample the estimate onto ground-truth timestamps.
    tt = truth[:, 0]
    ee = interp_to(est[:, 0], est[:, 1:], tt, angle_cols=(3,))  # col 3 is yaw
    tx, ty, tz, tyaw = truth[:, 1], truth[:, 2], truth[:, 3], truth[:, 4]
    ex, ey, ez, eyaw = ee[:, 0], ee[:, 1], ee[:, 2], ee[:, 3]

    pos_err = np.hypot(ex - tx, ey - ty)
    yaw_err = np.abs(np.degrees([wrap(a - b) for a, b in zip(eyaw, tyaw)]))
    z_err = np.abs(ez - tz)
    tb = np.arctan2(TURBINE_Y - ty, TURBINE_X - tx) - tyaw
    eb = np.arctan2(TURBINE_Y - ey, TURBINE_X - ex) - eyaw
    brg_err = np.abs(np.degrees([wrap(a - b) for a, b in zip(eb, tb)]))

    # Distance travelled along the TRUE path -- the denominator for drift %.
    step = np.hypot(np.diff(tx), np.diff(ty))
    dist = np.concatenate([[0.0], np.cumsum(step)])

    rel = tt - t0
    print("=" * 72)
    print(f"ANCHOR-FREE RUN: {os.path.basename(args.bag)}")
    print("=" * 72)
    print(f"  duration {rel[-1]:.0f}s   distance travelled {dist[-1]:.1f} m")

    # ---- did SLAM ever contribute? --------------------------------------
    if T_SLAM in D and len(D[T_SLAM]):
        t_first = D[T_SLAM][0, 0] - t0
        print(f"  first SLAM pose to the filter at t = {t_first:.0f}s "
              f"({len(D[T_SLAM])} messages)")
    else:
        t_first = None
        print("  SLAM NEVER reached the filter -- this is pure dead reckoning")

    # ---- DVL sign check, per axis ---------------------------------------
    if T_DVL in D and len(D[T_DVL]) > 10:
        dvl = D[T_DVL]
        tv = interp_to(tt, truth[:, 5:8], dvl[:, 0])
        moving = np.hypot(tv[:, 0], tv[:, 1]) > 0.05
        print("\n  DVL vs ground-truth body velocity "
              f"({moving.sum()} samples while moving):")
        ok = True
        gains = []
        for i, ax in enumerate("xyz"):
            a, b = tv[moving, i], dvl[moving, i + 1]
            if len(a) < 5 or np.std(a) < 1e-6:
                print(f"    v{ax}: too little motion to judge")
                continue
            corr = float(np.corrcoef(a, b)[0, 1])
            # Least-squares gain through the origin recovers the scale factor
            # the bridge actually applied.  This is the one check on the error
            # model that does not depend on the trajectory: whatever the
            # vehicle flew, the gain is a property of the simulated unit.
            # Reported as a deviation because the interesting quantity is a
            # fraction of a percent.
            gain = float((a * b).sum() / (a * a).sum())
            # The gain is estimated from noisy data, so it is never exactly 1
            # even with no systematic error: on the white-noise baseline vx
            # came back at +0.043%.  Carry its standard error so "deviates
            # from 1" is a statistical claim rather than a magic threshold.
            resid = b - gain * a
            se = float(np.sqrt((resid ** 2).sum() / max(len(a) - 1, 1)
                               / (a * a).sum()))
            print(f"    v{ax}: correlation {corr:+.3f}   "
                  f"mean|err| {np.abs(a - b).mean():.4f} m/s   "
                  f"gain {gain:.5f} +/- {se:.5f} "
                  f"({100 * (gain - 1):+.3f}% +/- {100 * se:.3f}%)"
                  f"{'   <-- SIGN FLIPPED' if corr < -0.5 else ''}")
            if corr < -0.5:
                ok = False
            gains.append((gain, se))
        print("    DVL frame is correct" if ok else
              "    DVL FRAME IS WRONG -- the run is invalid for navigation claims")
        if gains:
            # Significant at 5 sigma on any axis, and large enough to matter.
            # scale_sigma for the A50 is 1.01%, so a real unit sits tens of
            # standard errors out; estimation noise sits at two or three.
            z = max(abs(g - 1) / se for g, se in gains)
            dev = max(abs(g - 1) for g, _ in gains)
            if z > 5 and dev > 1e-3:
                print(f"    systematic gain error IS present "
                      f"(up to {100 * dev:.3f}%, {z:.0f} sigma). Cross-check "
                      f"the realised unit against the launch log; drift should "
                      f"grow with DISTANCE.")
            else:
                print(f"    no systematic gain error (largest deviation "
                      f"{100 * dev:.3f}%, {z:.1f} sigma -- consistent with "
                      f"estimation noise). This is the WHITE-NOISE-ONLY DVL, "
                      f"so drift grows as sqrt(time) and is optimistic.")

    # ---- segmented drift -------------------------------------------------
    def seg(name, mask):
        if mask.sum() < 10:
            return
        d = dist[mask]
        travelled = d[-1] - d[0]
        pe = pos_err[mask]
        growth = pe[-1] - pe[0]
        print(f"\n  {name}  ({rel[mask][0]:.0f}-{rel[mask][-1]:.0f}s, "
              f"{travelled:.1f} m travelled)")
        print(f"    position err : {pe[0]:7.2f} -> {pe[-1]:7.2f} m   "
              f"mean {pe.mean():6.2f}  max {pe.max():6.2f}")
        if travelled > 1.0:
            print(f"    drift        : {100 * growth / travelled:6.2f} % of distance travelled")
        print(f"    ATE RMSE     : {np.sqrt((pe ** 2).mean()):7.2f} m")
        print(f"    bearing err  : mean {brg_err[mask].mean():6.2f}  max {brg_err[mask].max():7.2f} deg")
        print(f"    yaw err      : mean {yaw_err[mask].mean():6.3f}  max {yaw_err[mask].max():6.3f} deg")
        print(f"    depth err    : mean {z_err[mask].mean():6.3f}  max {z_err[mask].max():6.3f} m")

    if t_first is not None:
        seg("DEAD RECKONING ONLY (before first SLAM pose)", rel < t_first)
        seg("WITH SLAM", rel >= t_first)
    seg("WHOLE RUN", np.ones_like(rel, dtype=bool))

    if args.fig:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import matplotlib.pyplot as plt
        from pubstyle import use, save, despine, COL2, SERIES
        use()
        fig, axes = plt.subplots(1, 3, figsize=(COL2, COL2 * 0.30))
        axes[0].plot(tx, ty, color=SERIES[0], lw=1.0, label='ground truth')
        axes[0].plot(ex, ey, color=SERIES[1], lw=1.0, ls='--', label='estimate')
        axes[0].plot([TURBINE_X], [TURBINE_Y], marker='x', color='#52514e', ms=5)
        axes[0].set_aspect('equal'); axes[0].legend(loc='best')
        axes[0].set_xlabel('x (m)'); axes[0].set_ylabel('y (m)')
        axes[0].set_title('(a) trajectory', loc='left')
        axes[1].plot(dist, pos_err, color=SERIES[0], lw=1.0)
        axes[1].set_xlabel('distance travelled (m)')
        axes[1].set_ylabel('position error (m)')
        axes[1].set_title('(b) drift', loc='left')
        # The raw series are ~100 Hz: drawn directly they overplot into a solid
        # block that shows nothing.  Summarise each 5 s window by its MAXIMUM,
        # which is conservative -- it keeps every peak, so the envelope still
        # supports a "stayed below X" claim -- and is legible at ~300 points.
        def envelope(t, e, win=5.0):
            n = max(1, int(win / np.median(np.diff(t))))
            k = (len(e) // n) * n
            if k == 0:
                return t, e
            return (t[:k].reshape(-1, n).mean(1), e[:k].reshape(-1, n).max(1))

        bt, be = envelope(rel, brg_err)
        yt, ye = envelope(rel, yaw_err)
        axes[2].plot(bt, be, color=SERIES[1], lw=1.0, label='bearing')
        axes[2].plot(yt, ye, color=SERIES[2], lw=1.0, ls='-.', label='yaw')
        axes[2].set_xlabel('time (s)'); axes[2].set_ylabel('5 s peak error (deg)')
        axes[2].legend(loc='best')
        axes[2].set_title('(c) bearing vs heading', loc='left')
        for a in axes:
            despine(a)
        if t_first is not None:
            axes[2].axvline(t_first, color='#52514e', lw=0.7, ls=':')
        fig.tight_layout(w_pad=1.6)
        stem = os.path.expanduser('~/ros2_ws/eval/figures/') + os.path.basename(args.bag)
        print('\nwrote:', ', '.join(save(fig, stem)))
    print("=" * 72)


if __name__ == "__main__":
    main()
