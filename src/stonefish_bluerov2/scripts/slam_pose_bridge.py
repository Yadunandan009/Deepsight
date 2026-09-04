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
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from collections import deque
import math
import time


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

        # Rigid transform: world = R(theta) @ slam + t
        self.aligned = False
        self.cos_t = 1.0
        self.sin_t = 0.0
        self.tx = 0.0
        self.ty = 0.0

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

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/bluerov2/robot_pose_slam_ekf', 10)
        self.create_subscription(
            PoseStamped,
            '/bluerov2/robot_pose_slam',
            self.slam_cb, 10)
        self.create_subscription(
            Odometry,
            '/bluerov2/odometry/filtered',
            self.ekf_cb, 10)
        self.create_subscription(
            PointCloud2,
            '/bluerov2/map_points',
            self.map_cb, 10)
        self.get_logger().info('SLAM pose bridge started')

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

    def _periodic_refit(self):
        """Timer-driven re-solve from the current sliding window, even
        while already aligned -- see module docstring. No-op while not yet
        aligned (the initial lock is driven directly from slam_cb() as
        soon as the buffer qualifies, not by this timer)."""
        if self.aligned and len(self.buf) >= self.MIN_CORRESPONDENCES:
            self._try_solve()

    def _try_solve(self):
        """Closed-form 2D rotation-only Procrustes fit (no scale — stereo
        SLAM's metric scale is already correct from the calibrated
        baseline): theta = angle of sum(conj(slam') * ekf') over centered
        correspondence pairs. Equivalent to Kabsch/Umeyama restricted to
        SO(2), without needing numpy/SVD for a 2D-only problem.

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
        self.tx = e_cx - (self.cos_t * s_cx - self.sin_t * s_cy)
        self.ty = e_cy - (self.sin_t * s_cx + self.cos_t * s_cy)
        self.aligned = True
        self.reject_streak = 0
        if not was_aligned:
            self.theta_at_lock = theta
        kind = 'refit' if was_aligned else 'solved'
        self.get_logger().info(
            f'SLAM alignment {kind}: theta={math.degrees(theta):+.1f}deg '
            f'tx={self.tx:.2f} ty={self.ty:.2f} '
            f'n={n} spread={spread:.1f}m map_pts={self.map_pts}')

    def slam_cb(self, m):
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

        wx = self.cos_t * sx - self.sin_t * sy + self.tx
        wy = self.sin_t * sx + self.cos_t * sy + self.ty

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
        # SLAM covariance — tune based on tracking quality
        c = out.pose.covariance
        c[0]  = 0.05   # x variance
        c[7]  = 0.05   # y variance
        c[14] = 9999.0 # z — not trusted
        c[21] = 9999.0 # roll
        c[28] = 9999.0 # pitch
        c[35] = 0.05   # yaw variance
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SlamPoseBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
