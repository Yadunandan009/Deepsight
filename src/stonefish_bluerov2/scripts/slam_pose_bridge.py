#!/usr/bin/env python3
"""
Converts SLAM PoseStamped → PoseWithCovarianceStamped in world_ned frame.

Aligns SLAM's map frame to EKF world frame with a full rigid transform
(rotation + translation), solved in closed form from a buffer of
(slam_xy, ekf_xy) correspondences collected while the map is dense enough
to trust.

A translation-only offset (old approach: offset = ekf - slam at a single
lock instant) only matches the two frames AT the lock point — ORB-SLAM3's
map frame is anchored to its first keyframe's camera pose, not to
world_ned, so it carries an arbitrary yaw rotation relative to world_ned.
Confirmed via telemetry: after translation-only lock, SLAM/EKF
disagreement grew linearly with distance from the lock point (5m at
lock+50s up to 37m mid-orbit), oscillating with each orbit — exactly the
signature of an uncorrected rotation, not translation drift.

Rotation is only observable from real horizontal motion (points must have
spatial spread — a rotation can't be estimated from a cluster of
near-identical points), so the correspondence buffer keeps collecting
past the old fixed "20 frames" threshold until MIN_BASELINE metres of
spread is seen. During pure vertical descent this can take a while (little
horizontal motion) — the alignment simply waits until CLOSE_IN/orbit
motion provides a real baseline, rather than locking early into a wrong
rotation.

Once locked, the transform used to be frozen until a full reset (either an
explicit fragmentation signal, or 40 consecutive EKF-disagreement
rejections). Telemetry showed this was the actual cause of near-random
re-lock thetas even in missions with no detected fragmentation: ORB-SLAM3's
own loop closure / bundle adjustment continuously and *correctly*
re-optimizes its map's coordinate frame over time, which is normal SLAM
behavior but means a frozen transform is guaranteed to go stale eventually
regardless of fragmentation. When it did go stale and hit the reject
threshold, the recovery path threw away the (possibly still-fine) rotation
and re-derived a brand new one from the smallest, worst-conditioned sample
possible — every "SLAM alignment solved" line in that telemetry showed
spread=2.0m exactly, the old MIN_BASELINE floor.

The fix: keep collecting correspondences into a sliding window even after
locking, and periodically re-solve from that window (see REFIT_INTERVAL)
so the transform tracks slow, legitimate drift smoothly instead of going
stale and violently re-guessing. A refit that would change theta by more
than REFIT_MAX_DELTA is treated as an outlier and discarded rather than
adopted — genuine fragmentation is still handled by the explicit map-crash
and raw-jump detectors below, which do a full, unconstrained reset.

MEASURED 2026-10-16, four bags, against ground truth (scratch replay over
`/bluerov2/robot_pose_slam`; see docs/experiments/PREREG_heading_drift.md):

  * The refit wander is NOT noise and must not be damped away. Sweeping
    the window width 75s -> infinity drives theta's scatter down (sd 5.0
    -> 0.8 deg) and the aligned pose's error UP (0.63 -> 1.31 m), on
    every bag tried. The map frame really does move; a transform that
    refuses to follow it is simply wrong for longer. The sliding window
    above is earning its keep and was left alone.
  * Metric scale is NOT 1.0. Similarity Procrustes recovers k = 0.89 to
    0.99 across bags (0.967 on the clean one). The stereo baseline and
    focal length were both re-derived from the .scn and agree with the
    SLAM yaml to four figures, so this is ORB-SLAM3's own scale bias on
    a far-dominated scene (mThDepth = 5.8 m, 13% of returns inside it),
    not a calibration error. Fitting k instead of assuming it is worth
    5-16% on three of four bags, and is modelled below.
  * The published covariance was the dominant defect. See _publish_var().
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from collections import deque
import math
import time

# The SLAM wrapper publishes REP-103 (z up); this workspace is NED (z down).
# Nothing converted between them until 2026-10-16, and the bridge's
# horizontal-rotation model cannot represent the difference -- see
# stonefish_bluerov2/frames.py and docs/SLAM_FRAME_ENU_NED.md.
try:
    from stonefish_bluerov2.frames import enu_to_ned_pose_inplace, FRAME_NOTE
except ImportError:                      # running straight from scripts/
    import os as _os
    import sys as _sys
    _sys.path.insert(0, _os.path.join(
        _os.path.dirname(_os.path.abspath(__file__)), '..'))
    from stonefish_bluerov2.frames import enu_to_ned_pose_inplace, FRAME_NOTE


def wrap(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class SlamPoseBridge(Node):
    def __init__(self):
        super().__init__('slam_pose_bridge')
        # EKF output — world frame anchor
        self.ekf_x = None
        self.ekf_y = None
        self.ekf_z = None

        # Similarity transform: world = scale * R(theta) @ slam + t.
        # cos_t/sin_t stay a PURE rotation so atan2(sin_t, cos_t) keeps
        # recovering theta for the refit gates; scale is carried separately
        # and applied at use.
        self.aligned = False
        self.cos_t = 1.0
        self.sin_t = 0.0
        self.scale = 1.0
        self.tx = 0.0
        self.ty = 0.0

        # Per-axis RMS residual of the correspondences under the adopted
        # transform. This is the bridge's own, causal, ground-truth-free
        # estimate of how wrong the pose it is about to publish will be --
        # see _publish_var().
        self.fit_resid = None

        # Correspondence buffer for solving the transform
        self.MIN_CORRESPONDENCES = 20   # floor sample count (~1.5s at 13Hz)
        self.MIN_BASELINE = 5.0         # m — spread required before a
                                         # rotation is actually observable.
                                         # Raised from 2.0: the old floor
                                         # was also the value every actual
                                         # solve landed on (confirmed via
                                         # telemetry -- "spread=2.0m" on
                                         # every single re-lock line), i.e.
                                         # every solve fired at the worst,
                                         # most noise-sensitive baseline
                                         # possible. A bigger floor trades
                                         # a slightly slower initial lock
                                         # for a much better-conditioned fit.
        self.BUFFER_MAXLEN = 1000       # ~75s at 13Hz -- also now the
                                         # sliding window width used by
                                         # periodic refits (see below)
        self.buf = deque(maxlen=self.BUFFER_MAXLEN)

        # Periodic refit — see module docstring. Re-solves from the current
        # sliding window on a timer, even while already aligned, so slow
        # (legitimate) SLAM coordinate-frame drift gets tracked instead of
        # letting the transform go stale and then violently re-guessing.
        self.REFIT_INTERVAL = 5.0  # s -- cadence of refit attempts
        self.REFIT_MAX_DELTA = math.radians(15.0)  # rad -- a refit that
                                         # would change theta by more than
                                         # this is treated as an outlier
                                         # window and discarded rather than
                                         # adopted; real drift between
                                         # refits should be a small
                                         # fraction of this
        self.REFIT_EMA_ALPHA = 0.2  # each accepted refit moves theta only
                                         # this fraction of the way toward
                                         # the freshly-solved candidate,
                                         # instead of snapping straight to
                                         # it. Needed because the rotation-
                                         # only Procrustes fit is poorly
                                         # conditioned when the correspondence
                                         # window is dominated by the ROV's
                                         # own circular orbiting motion during
                                         # SCAN (two arcs of near-identical
                                         # shape don't pin down a rotation
                                         # well) -- confirmed via telemetry
                                         # showing theta sliding 10-14deg
                                         # almost every single 5s refit for
                                         # 30s+ stretches, each step just
                                         # under REFIT_MAX_DELTA so never
                                         # individually rejected, but
                                         # compounding past 100deg of total
                                         # drift while still "aligned". EMA
                                         # damping doesn't fix the underlying
                                         # ill-conditioning but slows any
                                         # one bad window from being fully
                                         # adopted, and the cumulative-drift
                                         # cap below catches the case where
                                         # the damped drift is still real.
        self.REFIT_MAX_CUMULATIVE_DRIFT = math.radians(30.0)  # rad -- total
                                         # drift from the theta at the last
                                         # lock/re-lock, across any number of
                                         # small accepted refits. Exceeding
                                         # this forces a full re-align rather
                                         # than letting compounding small
                                         # steps silently reach an arbitrary
                                         # final orientation.
        # Plausible range for the realised stereo scale. Observed 0.89 to
        # 0.99 over four bags; a fit outside this says the window is bad,
        # not that the camera changed. Matters most at the INITIAL lock,
        # which has no previous scale to damp against and would otherwise
        # adopt whatever a degenerate window produced -- a failure mode the
        # old rotation-only code could not have, since it never fitted a
        # scale at all.
        self.SCALE_MIN = 0.75
        self.SCALE_MAX = 1.25

        self.theta_at_lock = 0.0
        self.create_timer(self.REFIT_INTERVAL, self._periodic_refit)

        # Raw-SLAM-frame jump detector (relocalization artifacts — the
        # map frame itself may have changed origin/orientation, so any
        # existing alignment is invalidated, not just this one pose)
        self.last_slam_x = None
        self.last_slam_y = None
        self.JUMP_THRESHOLD = 5.0  # m between consecutive raw SLAM frames

        # Map density gate — minimum map points before SLAM is trusted at
        # all, for both collecting correspondences and publishing corrections
        self.map_pts = 0
        self.MIN_MAP_PTS = 500

        # Map-stability gate — detects ORB-SLAM3 Atlas fragmentation from
        # the host side, without needing slam_msgs/SlamInfo (not installed
        # in this workspace's ROS2 Jazzy env, only inside the SLAM docker
        # container's Humble one — confirmed via ModuleNotFoundError).
        # A fragmentation/re-init event reliably shows up as a hard crash
        # in map_pts (confirmed repeatedly in telemetry: 78395->857,
        # 83400->2238, etc.) because the new Atlas map segment starts
        # from ~0 keyframes with an arbitrary new origin/orientation
        # unrelated to the old one. Without this gate, _try_solve() has
        # no way to know a correspondence buffer straddles two unrelated
        # frames, or that a freshly-solved rotation belongs to a
        # short-lived segment rather than a stable long-lived map —
        # confirmed root cause of alignment re-locking to essentially
        # random thetas (-139° to +149°) every ~60-90s across an
        # otherwise-clean mission.
        self.map_pts_prev = 0
        self.MAP_CRASH_RATIO = 0.5   # drop below this fraction of the
                                       # previous sample -> treat as a
                                       # fragmentation/re-init event
        self.MAP_CRASH_FLOOR = 2000  # only apply the ratio check once
                                       # map_pts was already above this,
                                       # so early ramp-up from 0 doesn't
                                       # misfire as a "crash"
        self.MAP_STABLE_DWELL = 5.0  # s -- map_pts must grow without a
                                       # crash for this long before a
                                       # fresh correspondence buffer is
                                       # allowed to start collecting
        self.map_stable_since = None

        # Published covariance. The bridge used to declare a hardcoded
        # 0.05 m^2 (sigma = 0.22 m) on every pose it ever published,
        # regardless of how well the alignment actually fitted. Measured
        # against ground truth on 2026-10-16, the aligned pose's true
        # per-axis sigma was 0.28 m on a clean run and 1.64 m on one with
        # a loop closure in it -- the latter is 54x the declared VARIANCE.
        # An EKF handed a 1.6 m measurement labelled "good to 22 cm" will
        # believe it over a dead-reckoned position that is genuinely good
        # to 17 cm, which is the arithmetic behind SLAM making the filter
        # worse rather than better.
        #
        # The window residual is a usable stand-in and costs nothing: it
        # read 0.18 m where truth said 0.28 m, and 1.94 m where truth said
        # 1.64 m, so it tracks the order of magnitude that matters and
        # flags the bad run as bad without ever seeing truth.
        #
        # The floor means this can only ever LOOSEN the old figure, never
        # tighten it: SLAM's pull on the filter can be reduced by this
        # change but not increased, so it cannot make any previously
        # acceptable run diverge.
        self.PUBLISH_VAR_FLOOR = 0.05   # m^2 -- the old hardcoded value
        self.PUBLISH_VAR_SLACK = 1.5    # residual underestimates the true
                                        # error (0.18 vs 0.28, 1.94 vs
                                        # 1.64); inflate sigma by this so
                                        # the filter is told the larger of
                                        # the two, not the smaller

        # Consistency gate — max disagreement (m) between a transformed
        # SLAM pose and the EKF's own dead-reckoned (DVL+INS+pressure)
        # position before that frame is rejected outright.
        self.EKF_CONSISTENCY_MAX = 5.0

        # If disagreement stays above EKF_CONSISTENCY_MAX for this many
        # consecutive frames (~3s at 13Hz), the transform itself has gone
        # stale — re-align from scratch rather than rejecting forever.
        # Without this, one bad patch permanently zeroes out SLAM's
        # contribution for the rest of the mission (confirmed: 13+ minutes
        # of nothing but rejections after the first orbit, in a run that
        # lived long enough to matter for ATE). Now mostly a backstop:
        # ordinary slow drift should be absorbed by _periodic_refit()
        # before it ever accumulates this many rejections — this path is
        # for real fragmentation the map-crash/raw-jump detectors missed.
        self.MAX_CONSECUTIVE_REJECTS = 40
        self.reject_streak = 0

        # Both topics are parameters so a SECOND instance can serve the
        # GPS-free estimator chain alongside the normal one, rather than
        # this file having to be edited back and forth between experiments.
        # Defaults reproduce the original hardcoded behaviour exactly.
        #
        # WHY THIS IS PARAMETERISED (2026-10-02): the alignment is fitted
        # against reference_odom, and the default reference is the primary
        # EKF -- which is anchored on /bluerov2/odometry, i.e. Stonefish
        # ground truth. Every pose this bridge publishes is therefore
        # indirectly ground-truth-informed, which is the confound that made
        # the GPS-free probe's XY numbers a lower bound rather than a real
        # result (see eval/gpsfree_probe.py and docs/PAPER_EXECUTION_PLAN.md
        # WS-0). Pointing reference_odom at the anchor-free estimate removes
        # it.
        #
        # NOTE THE FEEDBACK LOOP that creates: the GPS-free EKF consumes
        # this bridge's output as pose0, so aligning against that same EKF
        # closes the loop SLAM -> bridge -> EKF -> bridge. That is the
        # correct architecture for genuine SLAM-based navigation (filter and
        # alignment co-evolve, with no external truth to lean on), but it can
        # diverge in ways the open-loop version cannot. Expect to need the
        # rejection/re-align thresholds retuned, and watch the first runs
        # rather than batching them.
        # 'enu'  convert the incoming SLAM pose to NED (correct, default)
        # 'ned'  take it as-is -- reproduces every run before 2026-10-16,
        #        kept so the pre-fix behaviour can be re-measured rather
        #        than remembered.
        self.declare_parameter('slam_frame', 'enu')
        self.declare_parameter('reference_odom', '/bluerov2/odometry/filtered')
        self.declare_parameter('output_topic', '/bluerov2/robot_pose_slam_ekf')
        reference_odom = self.get_parameter('reference_odom').value
        output_topic = self.get_parameter('output_topic').value
        slam_frame = str(
            self.get_parameter('slam_frame').value).strip().lower()
        if slam_frame not in ('enu', 'ned'):
            raise ValueError(
                f"slam_frame must be 'enu' or 'ned', got {slam_frame!r}")
        self.slam_is_enu = (slam_frame == 'enu')

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            output_topic, 10)
        self.create_subscription(
            PoseStamped,
            '/bluerov2/robot_pose_slam',
            self.slam_cb, 10)
        self.create_subscription(
            Odometry,
            reference_odom,
            self.ekf_cb, 10)
        self.create_subscription(
            PointCloud2,
            '/bluerov2/map_points',
            self.map_cb, 10)
        self.get_logger().info(
            f'SLAM pose bridge started (reference_odom={reference_odom}, '
            f'output={output_topic}, slam_frame={slam_frame})')
        self.get_logger().info(
            f'  {FRAME_NOTE}' if self.slam_is_enu else
            '  slam_frame=ned: NO conversion applied. This reproduces '
            'the pre-2026-10-16 behaviour, in which no fixed '
            'map-to-world rotation exists. Intended only for '
            'reproducing old runs.')

    def ekf_cb(self, m):
        self.ekf_x = m.pose.pose.position.x
        self.ekf_y = m.pose.pose.position.y
        self.ekf_z = m.pose.pose.position.z

    def map_cb(self, m):
        pts = m.width * m.height
        if (self.map_pts_prev >= self.MAP_CRASH_FLOOR
                and pts < self.map_pts_prev * self.MAP_CRASH_RATIO):
            self.get_logger().warn(
                f'Map point crash detected: {self.map_pts_prev} -> {pts} '
                f'-- likely SLAM re-init/fragmentation, invalidating alignment')
            self._reset_alignment()
            self.map_stable_since = None
        elif self.map_stable_since is None:
            self.map_stable_since = time.time()
        self.map_pts_prev = pts
        self.map_pts = pts

    def _map_stable(self):
        return (self.map_stable_since is not None
                and time.time() - self.map_stable_since >= self.MAP_STABLE_DWELL)

    def _reset_alignment(self):
        self.aligned = False
        self.buf.clear()
        self.reject_streak = 0
        # Scale and residual belong to the transform being discarded. The
        # residual especially: leaving a stale one behind would let the
        # next pose published after a re-lock carry the confidence of the
        # fit that just failed.
        self.scale = 1.0
        self.fit_resid = None

    def _periodic_refit(self):
        """Timer-driven re-solve from the current sliding window, even
        while already aligned -- see module docstring. No-op while not yet
        aligned (the initial lock is driven directly from slam_cb() as
        soon as the buffer qualifies, not by this timer)."""
        if self.aligned and len(self.buf) >= self.MIN_CORRESPONDENCES:
            self._try_solve()

    def _try_solve(self):
        """Closed-form 2D similarity Procrustes fit: theta = angle of
        sum(conj(slam') * ekf') over centered correspondence pairs, and
        scale = |sum(conj(slam') * ekf')| / sum|slam'|^2. Umeyama
        restricted to Sim(2), without needing numpy/SVD for a 2D-only
        problem.

        Scale used to be pinned at 1.0 on the grounds that stereo SLAM's
        metric scale is correct from the calibrated baseline. The baseline
        is correct -- eval/check_stereo_baseline.py re-derives it from the
        .scn and it agrees with the SLAM yaml to four figures -- but the
        realised scale is not: k comes out 0.89-0.99 across every bag
        measured. A rotation-only fit cannot represent that, so the error
        went into the residual and, through it, into the published pose.

        Called both for the initial lock (self.aligned False) and for
        periodic refits of an existing lock (self.aligned True) — in the
        refit case, a candidate theta that disagrees too much with the
        currently-trusted one is discarded as an outlier window rather
        than adopted, so a single noisy sample can't undo a good lock."""
        pts = list(self.buf)
        n = len(pts)

        sxs = [p[0] for p in pts]
        sys_ = [p[1] for p in pts]
        spread = math.hypot(max(sxs) - min(sxs), max(sys_) - min(sys_))
        if spread < self.MIN_BASELINE:
            return  # not enough horizontal motion yet to see rotation

        s_cx = sum(sxs) / n
        s_cy = sum(sys_) / n
        e_cx = sum(p[2] for p in pts) / n
        e_cy = sum(p[3] for p in pts) / n

        num = 0.0
        den = 0.0
        for sx, sy, ex, ey in pts:
            sxp, syp = sx - s_cx, sy - s_cy
            exp_, eyp = ex - e_cx, ey - e_cy
            num += sxp * eyp - syp * exp_
            den += sxp * exp_ + syp * eyp
        theta = math.atan2(num, den)
        # sum|slam'|^2; guard is belt-and-braces, the spread gate above
        # already rejects a degenerate cluster.
        sq = sum((sx - s_cx) ** 2 + (sy - s_cy) ** 2 for sx, sy, _, _ in pts)
        scale = math.hypot(num, den) / sq if sq > 1e-9 else 1.0

        if not (self.SCALE_MIN <= scale <= self.SCALE_MAX):
            self.get_logger().warn(
                f'Fit rejected: scale {scale:.3f} outside '
                f'[{self.SCALE_MIN}, {self.SCALE_MAX}] -- degenerate or '
                f'cross-fragment window (n={n} spread={spread:.1f}m)',
                throttle_duration_sec=5.0)
            return

        was_aligned = self.aligned
        if was_aligned:
            cur_theta = math.atan2(self.sin_t, self.cos_t)
            delta = abs(wrap(theta - cur_theta))
            if delta > self.REFIT_MAX_DELTA:
                self.get_logger().warn(
                    f'Refit rejected: would change theta by '
                    f'{math.degrees(delta):.1f}deg '
                    f'(> {math.degrees(self.REFIT_MAX_DELTA):.0f}deg) -- '
                    f'treating as an outlier window, keeping current '
                    f'alignment (n={n} spread={spread:.1f}m)',
                    throttle_duration_sec=5.0)
                return
            # Damp the accepted refit instead of snapping straight to it --
            # see REFIT_EMA_ALPHA docstring above.
            theta = cur_theta + self.REFIT_EMA_ALPHA * wrap(theta - cur_theta)
            # Damp scale on the same cadence and for the same reason.
            scale = self.scale + self.REFIT_EMA_ALPHA * (scale - self.scale)

            cumulative = abs(wrap(theta - self.theta_at_lock))
            if cumulative > self.REFIT_MAX_CUMULATIVE_DRIFT:
                self.get_logger().warn(
                    f'Refit drift of {math.degrees(cumulative):.1f}deg since '
                    f'last lock exceeds '
                    f'{math.degrees(self.REFIT_MAX_CUMULATIVE_DRIFT):.0f}deg '
                    f'cap -- forcing full re-align instead of compounding '
                    f'further (n={n} spread={spread:.1f}m)')
                self._reset_alignment()
                return

        self.cos_t = math.cos(theta)
        self.sin_t = math.sin(theta)
        self.scale = scale
        kc, ks = scale * self.cos_t, scale * self.sin_t
        self.tx = e_cx - (kc * s_cx - ks * s_cy)
        self.ty = e_cy - (ks * s_cx + kc * s_cy)

        # Per-axis RMS residual of THIS window under the transform just
        # adopted. Computed here because the centred pairs are already to
        # hand, and because it must describe the transform actually in
        # force -- after the EMA damping, not the raw candidate.
        ss = 0.0
        for sx, sy, ex, ey in pts:
            sxp, syp = sx - s_cx, sy - s_cy
            ss += ((kc * sxp - ks * syp) - (ex - e_cx)) ** 2
            ss += ((ks * sxp + kc * syp) - (ey - e_cy)) ** 2
        self.fit_resid = math.sqrt(ss / (2 * n))

        self.aligned = True
        self.reject_streak = 0
        if not was_aligned:
            self.theta_at_lock = theta
        kind = 'refit' if was_aligned else 'solved'
        self.get_logger().info(
            f'SLAM alignment {kind}: theta={math.degrees(theta):+.1f}deg '
            f'scale={scale:.3f} tx={self.tx:.2f} ty={self.ty:.2f} '
            f'resid={self.fit_resid:.2f}m sigma={math.sqrt(self._publish_var()):.2f}m '
            f'n={n} spread={spread:.1f}m map_pts={self.map_pts}')

    def _publish_var(self):
        """Per-axis variance to publish with the pose, from the current
        fit's own residual. See PUBLISH_VAR_FLOOR above for why this is
        floored at the value it replaces."""
        if self.fit_resid is None:
            return self.PUBLISH_VAR_FLOOR
        return max((self.PUBLISH_VAR_SLACK * self.fit_resid) ** 2,
                   self.PUBLISH_VAR_FLOOR)

    def slam_cb(self, m):
        if self.slam_is_enu:
            enu_to_ned_pose_inplace(m.pose)
        sx = m.pose.position.x
        sy = m.pose.position.y

        # Detect raw SLAM relocalization jumps — the map frame itself may
        # have shifted origin/orientation, so any existing alignment (or
        # in-progress buffer) is no longer valid.
        if self.last_slam_x is not None:
            jump = math.hypot(sx - self.last_slam_x, sy - self.last_slam_y)
            if jump > self.JUMP_THRESHOLD:
                self.get_logger().warn(
                    f'SLAM jump detected {jump:.2f}m — resetting alignment')
                self._reset_alignment()
        self.last_slam_x = sx
        self.last_slam_y = sy

        if self.ekf_x is None:
            return

        # GATE 1 — density gate: correspondences/corrections from a map
        # too sparse to trust are skipped entirely, whether we're still
        # collecting or already aligned.
        if self.map_pts < self.MIN_MAP_PTS:
            self.get_logger().warn(
                f'SLAM pose skipped: map_pts={self.map_pts} '
                f'< {self.MIN_MAP_PTS}', throttle_duration_sec=2.0)
            return

        if not self.aligned:
            # GATE 0 — stability gate: don't collect correspondences (or
            # solve from them) until the current map has been crash-free
            # for MAP_STABLE_DWELL seconds. Prevents fitting a rotation
            # against a buffer that straddles a fragmentation event, or
            # trusting a short-lived segment's arbitrary frame before it's
            # had a chance to prove out. Only gates the INITIAL lock —
            # once aligned, collection continues unconditionally below to
            # feed the periodic-refit sliding window (a real fragmentation
            # after that point goes through _reset_alignment() instead,
            # which re-enters this branch and re-applies the gate).
            if not self._map_stable():
                return
            self.buf.append((sx, sy, self.ekf_x, self.ekf_y))
            if len(self.buf) >= self.MIN_CORRESPONDENCES:
                self._try_solve()
            return

        # Keep the sliding window fresh for _periodic_refit() even after
        # locking — see module docstring.
        self.buf.append((sx, sy, self.ekf_x, self.ekf_y))

        kc, ks = self.scale * self.cos_t, self.scale * self.sin_t
        wx = kc * sx - ks * sy + self.tx
        wy = ks * sx + kc * sy + self.ty

        # GATE 2 — consistency gate: reject this frame outright if it
        # disagrees with the EKF's own position by more than
        # EKF_CONSISTENCY_MAX. Without this, a single bad frame gets
        # fused straight into pose0 and can yank the EKF (x,y) far enough
        # to flip the controller's bearing calculation.
        dist = math.hypot(wx - self.ekf_x, wy - self.ekf_y)
        if dist > self.EKF_CONSISTENCY_MAX:
            self.reject_streak += 1
            self.get_logger().warn(
                f'SLAM pose rejected: {dist:.1f}m from EKF '
                f'(> {self.EKF_CONSISTENCY_MAX}m) streak={self.reject_streak}',
                throttle_duration_sec=2.0)
            if self.reject_streak >= self.MAX_CONSECUTIVE_REJECTS:
                self.get_logger().warn(
                    f'SLAM alignment stale after {self.reject_streak} '
                    f'consecutive rejections — re-aligning from scratch')
                self._reset_alignment()
            return
        self.reject_streak = 0

        # Apply transform and publish
        out = PoseWithCovarianceStamped()
        out.header = m.header
        out.header.frame_id = 'world_ned'
        out.pose.pose = m.pose
        out.pose.pose.position.x = wx
        out.pose.pose.position.y = wy
        # SLAM covariance, from the alignment's own residual rather than
        # a constant — see PUBLISH_VAR_FLOOR.
        v = self._publish_var()
        c = out.pose.covariance
        c[0]  = v      # x variance
        c[7]  = v      # y variance
        c[14] = 9999.0 # z — not trusted
        c[21] = 9999.0 # roll
        c[28] = 9999.0 # pitch
        c[35] = 0.05   # yaw — unchanged: the residual above is a POSITION
                       # residual and says nothing about heading, so
                       # deriving a yaw variance from it would be made up.
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SlamPoseBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
