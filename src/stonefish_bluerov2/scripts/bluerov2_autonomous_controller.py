#!/usr/bin/env python3
"""
v8 — Lattice-Robust Turbine-Relative Inspection Controller
=============================================================

WHY v8 EXISTS — the failure mode v7 could not escape:

  v7 used sonar to estimate BOTH range and bearing to the turbine, via
  a weighted-centroid extraction across all valid beam returns. This
  works well for a smooth monopile (one continuous surface — "bearing
  to the turbine" is a single well-defined number). It does NOT work
  for a lattice/jacket tower (multiple legs and diagonal braces): the
  nearest, or most acoustically prominent, member changes from scan to
  scan as the vehicle moves and as sonar beams pass through gaps in the
  structure. "Bearing to the turbine" is not one stable number for a
  lattice — no amount of Kalman filtering fixes that, because the
  underlying quantity being measured keeps changing identity, not just
  jittering around a fixed value.

  Across several iterations we tried: cluster-based bearing extraction,
  wide-aperture weighted-centroid extraction, adaptive noise widening,
  gate-and-reinit recovery, and wall-clock-deadline recovery. All of
  these treat the SYMPTOM (unstable bearing estimate) rather than the
  CAUSE (sonar bearing is not a meaningful control signal for a lattice
  structure at all).

THE v8 FIX — stop asking sonar for bearing:

  Sonar → RANGE ONLY. The minimum valid return ("distance to the
  nearest part of the structure") IS a stable, physically meaningful
  quantity for a lattice, and it's exactly what standoff/collision
  control needs anyway.

  Bearing/pointing → PURE GEOMETRY. The turbine's world position is
  exactly known (TX, TY). The vehicle's world position comes from the
  EKF (x, y), which has proven smooth and reliable throughout this
  entire project. atan2() of those two gives a bearing with zero
  sensor noise, by construction.

  The only real unknown is a FIXED offset in EKF yaw (established in
  earlier debugging: real, but constant over a mission, not something
  that needs continuous online correction). Rather than estimating
  that offset continuously from noisy lattice sonar — which is what
  kept re-injecting oscillation — it is calibrated ONCE, at mission
  start, from the one fact known with certainty: the ROV spawns
  already facing the turbine. That single calibration is then held
  fixed for the whole mission.

  Sensor role summary:
    Depth          → pressure z from EKF odom (direct sensor)
    Body velocity  → DVL body-frame vx, vy (inner velocity loop)
    Turbine range  → SONAR minimum return (lattice-safe scalar),
                     Kalman-smoothed, fused with world-model backup
    Yaw pointing   → PURE GEOMETRY (EKF x,y + turbine position),
                     corrected by a ONE-TIME calibrated yaw bias
    World yaw      → EKF yaw + fixed bias (no live sonar correction)
    SLAM           → corrects EKF inside robot_localization + telemetry;
                     never drives a control loop directly

Mission:
  IDLE → DESCEND(22m) → CLOSE_IN → SCAN ↔ TRANSIT ×4 faces
       → RISE(2m) → RETURN_HOME → COMPLETE
"""
import math, time, csv, os
from enum import Enum, auto
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from stonefish_ros2.msg import DVL


def wrap(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ═══════════════════════════════════════════════════════════════════
# Online estimators
# ═══════════════════════════════════════════════════════════════════
class EWStat:
    """Exponentially-weighted mean & variance — sensor reliability score."""
    def __init__(self, alpha=0.05, init_var=1.0):
        self.alpha = alpha
        self.mean = 0.0
        self.var = init_var

    def update(self, x):
        d = x - self.mean
        self.mean += self.alpha * d
        self.var = (1 - self.alpha) * (self.var + self.alpha * d * d)
        return self.var


class YawBiasEstimator:
    """
    ONE-TIME calibration, not continuous online estimation.

    Rationale: sonar bearing to a lattice/jacket structure is not a
    stable frame-to-frame quantity (see module docstring), so it
    cannot be used as a continuous correction signal without
    re-injecting the exact oscillation this project spent many rounds
    chasing. Instead we take the ROV's known spawn condition — placed
    facing the turbine — as ground truth ONCE, solve the fixed EKF-yaw
    offset from that single fact, and hold it fixed for the rest of
    the mission. EKF yaw itself (IMU+INS, both absolute, mutually
    consistent once imu0_relative is configured correctly) is smooth
    over a single mission's timescale; it does not need continuous
    recalibration, only this single fixed-offset correction.
    """
    def __init__(self):
        self.bias = 0.0
        self.calibrated = False

    def calibrate(self, world_brg, psi_ekf):
        """Call once, at mission start, before any motion. Assumes the
        vehicle is physically facing the turbine at that instant."""
        if self.calibrated:
            return
        self.bias = wrap(world_brg - psi_ekf)
        self.calibrated = True

    def correct(self, psi_ekf):
        return wrap(psi_ekf + self.bias)


class RadiusEstimator:
    """Learns center-to-sonar-surface offset of the turbine online."""
    def __init__(self, init=8.5, alpha=0.03):
        self.r = init
        self.alpha = alpha

    def update(self, world_center_range, sonar_surface_range):
        est = world_center_range - sonar_surface_range
        if 2.0 < est < 15.0:                     # plausibility gate
            self.r += self.alpha * (est - self.r)

    def center_range(self, sonar_surface_range):
        return sonar_surface_range + self.r


class RangeTracker:
    """
    RANGE-ONLY constant-velocity Kalman filter: x = [range, range_rate].

    Bearing is deliberately NOT tracked here — see the module
    docstring for why sonar bearing was dropped entirely for a lattice
    structure. Range, by contrast, remains meaningful: the MINIMUM
    valid sonar return ("distance to the nearest part of the
    structure") is well-defined regardless of how many legs and braces
    are present, and is exactly the quantity standoff/collision
    control needs.

    Two sources, adaptively weighted by recent innovation variance:
      A) sonar minimum return   → direct, primary
      B) world-model range      → EKF-position-derived, backup only
    """
    PROC_R = 0.15

    def __init__(self):
        self.x = None          # [range, range_rate]
        self.P = None
        self.t = None
        self.ew_sonar = EWStat(0.08, 1.0)
        self.last_sonar_t = 0.0
        # Wall-clock deadline recovery (not a call-count) — if we go
        # this long without ANY accepted sonar update, force-accept
        # the next reading outright rather than staying locked out.
        self.FORCE_ACCEPT_AFTER = 1.5

    def _init(self, r, now):
        self.x = np.array([r, 0.0])
        self.P = np.diag([4.0, 0.25])
        self.t = now

    def predict(self, now):
        if self.x is None:
            return
        dt = min(max(now - self.t, 1e-3), 0.5)
        self.t = now
        F = np.array([[1.0, dt], [0.0, 1.0]])
        q11 = 0.25 * dt**4 * self.PROC_R**2
        q12 = 0.5 * dt**3 * self.PROC_R**2
        q22 = dt**2 * self.PROC_R**2
        Q = np.array([[q11, q12], [q12, q22]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def _update(self, r_meas, var_r):
        H = np.array([1.0, 0.0])
        pred = H @ self.x
        y = r_meas - pred
        S = H @ self.P @ H.T + var_r
        K = (self.P @ H) / S
        self.x = self.x + K * y
        self.P = self.P - np.outer(K, H @ self.P)
        return y

    def update_sonar(self, r_center, now):
        if self.x is None:
            self._init(r_center, now)
            self.last_sonar_t = now
            return
        self.predict(now)
        pred_r = self.x[0]
        gate_bad = abs(r_center - pred_r) > 8.0
        if gate_bad:
            since_accept = now - self.last_sonar_t if self.last_sonar_t else 999.0
            if since_accept < self.FORCE_ACCEPT_AFTER:
                return
            # Too long without any accepted correction — trust the
            # sensor now rather than staying locked out indefinitely.
            self._init(r_center, now)
            self.last_sonar_t = now
            return
        var_r = max(0.05, min(9.0, self.ew_sonar.var))
        y = self._update(r_center, var_r)
        self.ew_sonar.update(float(abs(y)))
        self.last_sonar_t = now

    def update_world(self, r_center, now):
        """Backup source only — generous fixed noise so it can never
        drag the track, only gently anchor it when sonar is stale."""
        if self.x is None:
            return
        self.predict(now)
        self._update(r_center, 4.0)

    def healthy(self, now, max_age=2.5):
        # Previously sonar-only, which stalled CLOSE_IN indefinitely
        # whenever sonar returned invalid (999) at the approach pose --
        # confirmed in telemetry: son=999.0 continuously while trk_r
        # stayed frozen and the vehicle just held station for 165s+.
        # update_world() keeps self.x alive via the EKF-position-derived
        # backup every odom tick regardless of sonar, so range() is
        # still a sane estimate even when sonar is dead. Track is only
        # truly unusable if it was never initialized at all.
        return self.x is not None

    def range(self):
        return None if self.x is None else float(self.x[0])


# ═══════════════════════════════════════════════════════════════════
class S(Enum):
    IDLE        = auto()
    DESCEND     = auto()
    CLOSE_IN    = auto()
    SCAN        = auto()
    TRANSIT     = auto()
    RISE        = auto()
    RETURN_HOME = auto()
    COMPLETE    = auto()
    ESTOP       = auto()


# ═══════════════════════════════════════════════════════════════════
class InspectV8(Node):

    # ── Mission geometry ────────────────────────────────────────
    TX, TY        = 20.0, 0.0
    FACE_BEARINGS = [180.0, 90.0, 0.0, 270.0]     # S, E, N, W
    SCAN_DIST     = 17.0        # center standoff during scan/orbit

    # ── Depths ──────────────────────────────────────────────────
    D_BASE, D_TOP     = 22.0, 1.0
    D_TRANSIT, D_RISE = 10.0, 2.0
    D_MAX             = 23.5

    # ── Velocity limits ─────────────────────────────────────────
    V_SURGE_MAX = 0.50
    V_SCAN_MAX  = 0.30
    V_ORBIT     = 0.35

    # CLOSE_IN crab-correction priority: surge, yaw, and sway share the
    # same 4 horizontal thrusters (T_PINV columns 0/1/5), so a strong
    # surge command combined with heading correction saturates the
    # shared allocation -- _full()'s saturate-and-rescale then shrinks
    # ALL of tau proportionally, diluting whatever sway correction
    # vel_ctrl() computed at exactly the moment it's needed most.
    # Confirmed via telemetry: vy_b grew to +/-0.2 m/s and was never
    # driven back toward zero across a full ~40s CLOSE_IN approach
    # while ux sat near 1.0 throughout -- real, sustained lateral
    # drift (not just visual wobble) that displaced SCAN face 0
    # off-center from the intended world-clock position. Throttling
    # surge back as measured sway grows frees up thruster headroom for
    # the sway correction to actually take effect.
    CRAB_VY_LIMIT   = 0.30  # m/s -- |vy_b| at which surge is cut to the floor
    CRAB_CAP_FLOOR  = 0.30  # minimum surge authority fraction even under
                             # sustained crab, so the approach still progresses
    CRAB_YAW_RATE_LIMIT = math.radians(2.5)  # rad/s -- |yaw_rate| at which
                             # surge is cut to the floor, mirroring
                             # CRAB_VY_LIMIT but keyed on the EARLIEST
                             # symptom of the surge-coupling disturbance
                             # rather than its downstream effect. Confirmed
                             # via telemetry across two separate missions:
                             # a real (gyro-verified, not EKF-glitch) yaw
                             # rotation reliably starts ~10-25s into
                             # CLOSE_IN -- reaching up to ~14.6deg/s before
                             # it's even confirmed -- and grows into a
                             # 33-57deg heading swing before bearing_hold's
                             # position-error term catches up, by which
                             # point the swing (and the sway it imparts)
                             # has already happened. yaw_rate is live every
                             # tick regardless of the yaw-glitch-guard's
                             # hold state, so throttling surge on it
                             # directly removes the disturbance's presumed
                             # energy source (high surge authority) the
                             # moment a rotation starts, instead of
                             # reacting after the fact.
    CLOSE_IN_SURGE_RAMP_T = 8.0  # s -- raised from 3.0. Telemetry across
                             # two independent runs (one with a yaw
                             # force-accept mid-CLOSE_IN, one with a
                             # completely clean yaw trace) showed the same
                             # vy_b deviation starting at t=6-8s into
                             # CLOSE_IN, 2-3s BEFORE any sonar/SLAM trouble
                             # -- same sign, same rough timing, right as
                             # the 3s ramp neared full authority. That
                             # rules out sonar/SLAM as the trigger and
                             # points back to the same surge-coupling
                             # torque already documented below (previously
                             # severe enough to swing bearing 168° before
                             # the ramp existed at all) -- the 3s ramp
                             # softened it but didn't remove it. Spreading
                             # the ramp over more time lowers the peak
                             # disturbance rate and gives the crab throttle
                             # more headroom to react before vy_b builds
                             # toward its ~0.25 m/s ceiling.
    SONAR_DROPOUT_CAUTION_T = 1.0  # s -- if the multibeam has reported NO
                             # valid return for this long during CLOSE_IN,
                             # slow to a cautious creep instead of closing
                             # in on world-model range alone at full
                             # authority. Confirmed via telemetry: in a run
                             # with two real (gyro-verified) yaw kicks
                             # during CLOSE_IN, BOTH bracketed the SAME
                             # ~13s sonar dropout (son=999 the entire
                             # time) and SLAM map crash (75702->101
                             # keyframes) -- one right as the dropout
                             # started, one right as it cleared. trk_r
                             # (world-model range, all that's left once
                             # sonar drops out) stayed ~20m throughout,
                             # i.e. this isn't a known-close reading the
                             # existing SONAR_STOP backoff would catch --
                             # it looks like real proximity to some part
                             # of the actual lattice (legs/braces reach
                             # out from the center the range-to-center
                             # abstraction tracks) that only the sonar
                             # would have seen, had it not gone blind.
    SONAR_DROPOUT_CREEP = 0.05  # m/s -- NOT zero. A hard v_sp=0 here
                             # deadlocked in testing: stopping removed the
                             # very motion that let sonar naturally
                             # reacquire in every prior (unfixed) run,
                             # where a dropout always self-cleared within
                             # ~13s while the vehicle kept moving -- so
                             # holding still meant sonar never recovered
                             # and CLOSE_IN just sat frozen at v_sp=0
                             # until the 90s timeout. A slow creep keeps
                             # some closing motion (and therefore some
                             # chance of reacquiring sonar) while still
                             # being far more cautious than a full-speed
                             # blind approach.

    # ── Gains ───────────────────────────────────────────────────
    KV         = 1.2
    KP_RANGE   = 0.30
    KP_BEAR    = 0.60
    KD_BEAR    = 0.55       # increased from 0.35 -- DESCEND showed
                             # underdamped ringing (world_b overshooting
                             # through multiple 180deg swings before
                             # settling, not a single disturbance decaying).
                             # Fix attempt 1/3; shared across bearing_hold()
                             # and yaw_hold_world() so watch CLOSE_IN/SCAN/
                             # RISE for any new twitchiness from stronger
                             # derivative gain amplifying yaw_rate noise.
    YAW_CMD_LP = 0.25

    # ── Sonar ───────────────────────────────────────────────────
    SONAR_STOP = 2.5

    # ── Timeouts / tolerances ───────────────────────────────────
    SCAN_TIMEOUT, TRANSIT_TIMEOUT, RETURN_TIMEOUT = 400.0, 500.0, 300.0
    CLOSE_IN_TIMEOUT = 90.0  # s -- safety valve for the new |vy_b|<0.08
                              # exit condition; CLOSE_IN previously had
                              # no timeout at all. If sustained crab
                              # coupling ever keeps sway from settling,
                              # force the SCAN handoff anyway rather
                              # than hang indefinitely.
    DEPTH_TOL_SCAN, DEPTH_TOL_PHASE = 0.8, 1.0
    POS_TOL_HOME = 2.0
    SCAN_FULL_CYCLES = 1

    # ── Thruster T-matrix pseudoinverse (unchanged, proven) ─────
    T_PINV = np.array([
        # Surge column (index 0) sign-corrected -- was producing
        # backward motion under a positive command. Confirmed from
        # telemetry: CLOSE_IN commanded ux=1.00 forward continuously,
        # yet trk_r grew monotonically for the entire run (never once
        # decreased) while vx_b stayed negative throughout, with
        # heading locked correctly at 0 deg the whole time.
        [-0.353557, +0.353557, +0.000000, 0.0, 0.0, +1.700680],
        [-0.353557, -0.353557, +0.000000, 0.0, 0.0, -1.700680],
        [+0.353557, +0.353557, +0.000000, 0.0, 0.0, -1.700680],
        [+0.353557, -0.353557, +0.000000, 0.0, 0.0, +1.700680],
        [+0.000000, +0.000000, +0.250000, 0.0, 0.0, +0.000000],
        [+0.000000, +0.000000, +0.250000, 0.0, 0.0, +0.000000],
        [+0.000000, +0.000000, +0.250000, 0.0, 0.0, +0.000000],
        [+0.000000, +0.000000, +0.250000, 0.0, 0.0, +0.000000],
    ])
    WRENCH_SCALE = 2.0

    # ── Yaw glitch guard ────────────────────────────────────────
    # /bluerov2/odometry/filtered's orientation quaternion has been
    # observed to make a discrete, instantaneous ~180deg jump (single
    # ~20ms tick at the EKF's 50Hz publish rate) with ZERO matching
    # motion in Stonefish's own ground-truth /bluerov2/odometry --
    # confirmed by direct side-by-side logging (ground truth stayed
    # flat at 0deg for the full window; EKF flipped to -180deg in one
    # tick, then tracked GT-180 in lockstep for ~48s -- i.e. the EKF
    # was internally self-consistent but offset by a fixed, wrong
    # 180deg for an extended period, only self-correcting via a
    # second identical instantaneous jump much later). Root cause is
    # upstream (robot_localization / IMU quaternion representation,
    # not this controller -- the atan2(2(qw*qz+qx*qy), qw^2+qx^2-qy^2-qz^2)
    # formula used below is already invariant to quaternion double-cover,
    # i.e. q vs -q, so it cannot itself be introducing this). A real
    # ROV physically cannot rotate 180deg in 20ms (~9000deg/s), so any
    # single-tick jump that large is trivially distinguishable from
    # real motion -- hold the last good value instead of ever handing
    # it to the controller.
    YAW_GLITCH_MAX_STEP = math.radians(15.0)  # ~750deg/s at 50Hz -- far
                                                # above the observed real
                                                # max (~39deg/s) and far
                                                # below the ~9000deg/s
                                                # implied by the glitch
    YAW_GLITCH_HOLD_MAX = 60.0  # s -- last-resort safety valve if gyro
                                 # never corroborates the jump (see
                                 # YAW_GLITCH_GYRO_TOL below) -- so a
                                 # miscalibrated threshold or a genuine
                                 # sustained fault can't wedge the
                                 # controller on a stale yaw forever.
                                 # Raised from an initial 10s: confirmed
                                 # via telemetry that the underlying
                                 # glitch can stay locked on the wrong
                                 # value for a full continuous 10s (not
                                 # just a rare one-off), so 10s let two
                                 # separate force-accepts through in one
                                 # DESCEND+CLOSE_IN run, each causing a
                                 # real corrective spin that settled on
                                 # a wrong heading. No real controller-
                                 # commanded turn needs a fast response
                                 # to an instantaneous >15deg jump --
                                 # deliberate turns are always driven
                                 # gradually through bearing_hold() --
                                 # so raising this has no real-motion
                                 # downside, only less exposure to the
                                 # glitch.
    YAW_GLITCH_GYRO_TOL = math.radians(8.0)  # deg -- a disputed jump is
                                 # accepted immediately (no waiting for
                                 # YAW_GLITCH_HOLD_MAX) once the gyro's own
                                 # integrated rotation over the disputed
                                 # window matches the jump within this
                                 # tolerance. Confirmed via CLOSE_IN
                                 # telemetry: a real 22deg force-accepted
                                 # jump triggered a genuine ~26deg
                                 # corrective spin (gyro would have shown
                                 # it), vs. earlier 163-180deg jumps that
                                 # reversed a moment later with no
                                 # corresponding motion at all -- gyro
                                 # would NOT have shown those.
                                 #
                                 # The corroboration signal is now the raw
                                 # /bluerov2/imu angular_velocity.z
                                 # (imu_yaw_rate), NOT the EKF's own
                                 # twist.angular.z. Previously it was the
                                 # latter, on the reasoning that rate
                                 # fusion and orientation fusion are
                                 # different pathways inside the EKF and
                                 # so wouldn't share a bug. That held for
                                 # the original glitch (an internal
                                 # quaternion-representation bug), but a
                                 # second, distinct failure mode was found
                                 # 2026-09-04: ORB-SLAM3 losing tracking
                                 # for ~12s and relocalizing produces a
                                 # discontinuous SLAM pose input to the
                                 # EKF, which can corrupt BOTH the EKF's
                                 # fused orientation AND its fused twist
                                 # at once (same bag: a 1.83rad/s twist
                                 # spike with an unexplained sign-reversal
                                 # overshoot appeared exactly inside a
                                 # 12s SLAM dropout, while the commanded
                                 # PWM over that window was a smooth,
                                 # modest, symmetric ramp -- too gentle to
                                 # physically produce that rate). In that
                                 # failure mode the EKF twist is not
                                 # independent of the thing being
                                 # disputed, so it can't be trusted to
                                 # corroborate or refute a jump in the
                                 # EKF's own yaw. Raw IMU angular velocity
                                 # is a physically separate sensor path
                                 # that SLAM fusion cannot corrupt.
    YAW_GLITCH_GYRO_MIN_HOLD = 0.3  # s -- ignore gyro corroboration until
                                 # this much integration time has passed,
                                 # so a near-zero integral on the very
                                 # first disputed tick doesn't spuriously
                                 # read as "confirmed" by accident.

    # ════════════════════════════════════════════════════════════
    def __init__(self):
        super().__init__('inspect_v8')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pwm = self.create_publisher(
            Float64MultiArray, '/bluerov2/setpoint/pwm', 10)

        self.create_subscription(Odometry, '/bluerov2/odometry/filtered',
                                 self.odom_cb, qos)
        self.create_subscription(PoseStamped, '/bluerov2/robot_pose_slam',
                                 self.slam_cb, qos)
        self.create_subscription(PointCloud2, '/bluerov2/map_points',
                                 self.map_cb, qos)
        self.create_subscription(LaserScan, '/bluerov2/multibeam_raw',
                                 self.sonar_cb, qos)
        self.create_subscription(DVL, '/bluerov2/dvl_sim',
                                 self.dvl_cb, qos)
        self.create_subscription(Odometry, '/deepsight/fused_pose',
                                 self.fused_cb, qos)
        self.create_subscription(Float32, '/deepsight/nav_confidence',
                                 self.conf_cb, qos)
        self.create_subscription(Imu, '/bluerov2/imu',
                                 self.imu_cb, qos)

        # ── Raw sensor state ────────────────────────────────────
        self.px = self.py = self.pz = self.yaw_ekf = None
        self.yaw_rate = 0.0
        self.imu_yaw_rate = 0.0
        self._yaw_glitch_since = None
        self._yaw_glitch_gyro_integral = 0.0
        self._yaw_glitch_last_t = None
        self.vx_b = self.vy_b = 0.0
        self.dvl_valid = False
        self.map_pts = 0
        self.slam_t = None
        self.nav_conf = 0.0
        self.fused_x = self.fused_y = None
        self.sonar_range = 999.0
        self.sonar_valid = False
        self.sonar_invalid_since = None

        # ── Estimators ──────────────────────────────────────────
        self.tracker  = RangeTracker()
        self.yaw_bias = YawBiasEstimator()
        self.radius   = RadiusEstimator()

        # ── Mission state ───────────────────────────────────────
        self.state = S.IDLE
        self.started = False
        self.phase_t = None
        self.face_idx = 0
        self.home_x = self.home_y = 0.0

        self.scan_going_up = True
        self.scan_cycles_done = 0

        # Set when the final (post-face-3) transit back to face 0 is
        # running, so the orbit machinery is reused but on arrival it
        # goes to RISE instead of SCAN.
        self.returning_to_face0 = False

        self.transit_phase = 'rise'
        self.orbit_sign = -1.0
        self.orbit_sign_checked = False
        self.orbit_target_arc = 0.0
        self.orbit_beta_prev = None
        self.orbit_swept = 0.0
        self.orbit_check_t = None

        self._yaw_cmd = 0.0

        # ── Logging ─────────────────────────────────────────────
        os.makedirs(os.path.expanduser('~/ros2_ws/logs'), exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.lf = open(os.path.expanduser(f'~/ros2_ws/logs/v8_{ts}.csv'),
                       'w', newline='')
        self.lw = csv.writer(self.lf)
        self.lw.writerow(['time', 'state', 'face', 'x', 'y', 'z',
                          'yaw_ekf_deg', 'yaw_bias_deg',
                          'trk_range', 'world_bearing_deg',
                          'sonar_m', 'radius_est',
                          'vx_b', 'vy_b', 'map_pts', 'nav_conf'])

        self.create_timer(0.05, self.ctrl)     # 20 Hz control
        self.create_timer(1.0,  self.status)
        self.create_timer(0.5,  self.log_row)
        self.create_timer(3.0,  self.trystart)

        self.get_logger().info(
            f"v8 lattice-robust controller | T=({self.TX},{self.TY}) "
            f"R={self.SCAN_DIST} | scan {self.D_TOP}↔{self.D_BASE}m | "
            f"sonar=RANGE-ONLY, bearing=WORLD-MODEL")

    # ════════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════════
    def imu_cb(self, m):
        self.imu_yaw_rate = m.angular_velocity.z

    def odom_cb(self, m):
        self.px = m.pose.pose.position.x
        self.py = m.pose.pose.position.y
        self.pz = m.pose.pose.position.z
        q = m.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z
        raw_yaw = math.atan2(siny, cosy)
        self.yaw_rate = m.twist.twist.angular.z
        self._filter_yaw(raw_yaw)
        if self.state not in (S.IDLE, S.COMPLETE, S.ESTOP):
            self._feed_world_model()

    def _filter_yaw(self, raw_yaw):
        """Reject implausible yaw jumps unless the gyro's own integrated
        rotation corroborates them -- see YAW_GLITCH_MAX_STEP/GYRO_TOL
        docstrings above for the confirmed glitch this guards against."""
        now = time.time()
        if self.yaw_ekf is None:
            self.yaw_ekf = raw_yaw
            self._yaw_glitch_since = None
            return

        signed_step = wrap(raw_yaw - self.yaw_ekf)
        step = abs(signed_step)
        if step <= self.YAW_GLITCH_MAX_STEP:
            self.yaw_ekf = raw_yaw
            self._yaw_glitch_since = None
            return

        if self._yaw_glitch_since is None:
            self._yaw_glitch_since = now
            self._yaw_glitch_gyro_integral = 0.0
            self._yaw_glitch_last_t = now
        else:
            dt = now - self._yaw_glitch_last_t
            self._yaw_glitch_gyro_integral += self.imu_yaw_rate * dt
            self._yaw_glitch_last_t = now

        held_for = now - self._yaw_glitch_since
        gyro_rot = wrap(self._yaw_glitch_gyro_integral)
        gyro_confirms = (held_for >= self.YAW_GLITCH_GYRO_MIN_HOLD
                          and abs(wrap(signed_step - gyro_rot)) <= self.YAW_GLITCH_GYRO_TOL)

        if gyro_confirms:
            self.get_logger().warn(
                f"yaw_ekf jump of {math.degrees(step):.0f}deg CONFIRMED by gyro "
                f"({math.degrees(gyro_rot):.0f}deg integrated over {held_for:.1f}s) "
                f"-- accepting (raw={math.degrees(raw_yaw):.0f})")
            self.yaw_ekf = raw_yaw
            self._yaw_glitch_since = None
        elif held_for >= self.YAW_GLITCH_HOLD_MAX:
            self.get_logger().warn(
                f"yaw_ekf jump of {math.degrees(step):.0f}deg persisted {held_for:.1f}s "
                f"with NO gyro support (gyro only integrated {math.degrees(gyro_rot):.0f}deg) "
                f"-- force-accepting anyway as safety valve (raw={math.degrees(raw_yaw):.0f}, "
                f"held={math.degrees(self.yaw_ekf):.0f})")
            self.yaw_ekf = raw_yaw
            self._yaw_glitch_since = None
        else:
            self.get_logger().warn(
                f"yaw_ekf glitch rejected: {math.degrees(step):.0f}deg jump, gyro supports "
                f"only {math.degrees(gyro_rot):.0f}deg so far (raw={math.degrees(raw_yaw):.0f}, "
                f"held={math.degrees(self.yaw_ekf):.0f})",
                throttle_duration_sec=1.0)

    def slam_cb(self, m):
        self.slam_t = time.time()

    def map_cb(self, m):
        self.map_pts = m.width * m.height

    def dvl_cb(self, m):
        self.vx_b = m.velocity.x
        self.vy_b = m.velocity.y
        self.dvl_valid = True

    def fused_cb(self, m):
        self.fused_x = m.pose.pose.position.x
        self.fused_y = m.pose.pose.position.y

    def conf_cb(self, m):
        self.nav_conf = float(m.data)

    def sonar_cb(self, m):
        """Sonar → RANGE ONLY. See module docstring: on a lattice/jacket
        structure, bearing is not extracted from sonar at all — only
        the minimum valid return ('distance to nearest part of the
        structure'), which remains stable and physically meaningful
        regardless of how many legs/braces are present."""
        ranges = list(m.ranges)
        valid = [r for r in ranges if 0.1 < r < m.range_max]
        if len(valid) < 3:
            self.sonar_valid = False
            self.sonar_range = 999.0
            if self.sonar_invalid_since is None:
                self.sonar_invalid_since = time.time()
            return
        self.sonar_invalid_since = None

        rmin = min(valid)
        self.sonar_range = rmin
        self.sonar_valid = True

        now = time.time()
        r_center = self.radius.center_range(rmin)
        self.tracker.update_sonar(r_center, now)

        # Radius learning — only while genuinely close to nose-on
        # (per the world-model bearing), since that's when "nearest
        # sonar return" best approximates "front face of structure".
        if self.px is not None and self.yaw_ekf is not None:
            world_r = math.hypot(self.TX - self.px, self.TY - self.py)
            b = self.current_bearing()
            if b is not None and abs(b) < math.radians(15):
                self.radius.update(world_r, rmin)

    def _feed_world_model(self):
        """World-pose-derived range as a backup tracker input (range
        only — bearing no longer comes from any estimator, it's
        computed live wherever it's needed, see current_bearing())."""
        if self.px is None or self.yaw_ekf is None:
            return
        r_center = math.hypot(self.TX - self.px, self.TY - self.py)
        self.tracker.update_world(r_center, time.time())

    # ════════════════════════════════════════════════════════════
    # SHARED CONTROL PRIMITIVES
    # ════════════════════════════════════════════════════════════
    def current_bearing(self):
        """Body-frame bearing to the turbine, computed PURELY from
        geometry: known turbine position + EKF (x,y) + one-time
        calibrated yaw bias. No sonar involved — zero sensor noise on
        this signal by construction. This is the entire fix for the
        lattice-tower bearing instability."""
        if self.px is None or self.yaw_ekf is None:
            return None
        psi = self.yaw_bias.correct(self.yaw_ekf)
        world_brg = math.atan2(self.TY - self.py, self.TX - self.px)
        return wrap(world_brg - psi)

    KI_VEL   = 0.35   # integral gain — closes the steady-state gap a
                      # pure-P loop cannot: confirmed empirically that
                      # ux settled at ~0.50 (not saturated) while vx_b
                      # plateaued at ~0.08 against a 0.50 setpoint —
                      # exactly KV*(0.50-0.08)=0.50, a stable P-only
                      # equilibrium with no mechanism to push harder.
    IMAX_VEL = 3.0    # integral clamp (anti-windup)

    def vel_ctrl(self, vx_sp, vy_sp):
        """Inner loop: body velocity closed on DVL, PI with CONDITIONAL
        INTEGRATION anti-windup (upgraded from a plain clamp). Immune to
        position estimate jumps by construction; the integral term closes
        the persistent steady-state gap a P-only loop leaves against
        sustained drag. Anti-windup freezes the integrator whenever the
        output is saturated AND the error is pushing further into that
        same saturation direction -- otherwise a sustained large-error
        command (e.g. full-authority sway during orbit) can consume all
        thruster headroom via windup and starve the other axis's
        correction, which is exactly the 'uy pinned at 1.00' pattern
        observed during TRANSIT."""
        if not self.dvl_valid:
            return 0.4 * vx_sp, 0.4 * vy_sp

        now = time.time()
        if not hasattr(self, '_vel_i_t'):
            self._ix, self._iy, self._vel_i_t = 0.0, 0.0, now
        dt = max(min(now - self._vel_i_t, 0.5), 1e-3)
        self._vel_i_t = now

        ex = vx_sp - self.vx_b
        ey = vy_sp - self.vy_b

        ux = self.KV * ex + self.KI_VEL * self._ix
        uy = self.KV * ey + self.KI_VEL * self._iy

        ux_sat = max(-1.0, min(1.0, ux))
        uy_sat = max(-1.0, min(1.0, uy))

        if not (abs(ux_sat) >= 0.99 and ux * ex > 0):
            self._ix = max(-self.IMAX_VEL, min(self.IMAX_VEL, self._ix + ex * dt))
        if not (abs(uy_sat) >= 0.99 and uy * ey > 0):
            self._iy = max(-self.IMAX_VEL, min(self.IMAX_VEL, self._iy + ey * dt))

        return ux_sat, uy_sat

    def _reset_vel_integrators(self):
        self._ix = self._iy = 0.0
        self._vel_i_t = time.time()

    def bearing_hold(self, target=0.0):
        """Yaw closed on the WORLD-MODEL bearing (pure geometry +
        one-time-calibrated yaw), not sonar. Smooth by construction —
        this directly replaces the sonar-bearing-driven version that
        could not be made stable against a lattice structure."""
        b = self.current_bearing()
        if b is None:
            raw = -0.20 * self.yaw_rate
        else:
            err = wrap(b - target)
            raw = self.KP_BEAR * err - self.KD_BEAR * self.yaw_rate
            raw = max(-0.30, min(0.30, raw))
        self._yaw_cmd += self.YAW_CMD_LP * (raw - self._yaw_cmd)
        return self._yaw_cmd

    def yaw_hold_world(self, psi_target):
        """World-frame yaw hold with bias-corrected yaw (RISE/RETURN)."""
        if self.yaw_ekf is None:
            return 0.0
        psi = self.yaw_bias.correct(self.yaw_ekf)
        err = wrap(psi_target - psi)
        raw = max(-0.30, min(0.30, 0.35 * err - self.KD_BEAR * self.yaw_rate))
        self._yaw_cmd += self.YAW_CMD_LP * (raw - self._yaw_cmd)
        return self._yaw_cmd

    def depth_ctrl(self, target):
        """Depth PD on pressure z — the one estimate that never lied."""
        now = time.time()
        if not hasattr(self, '_pz_prev'):
            self._pz_prev, self._pz_t = self.pz, now
        dt = max(now - self._pz_t, 1e-3)
        dz = (self.pz - self._pz_prev) / dt
        self._pz_prev, self._pz_t = self.pz, now
        u = 0.55 * (target - self.pz) - 0.30 * dz
        return max(-2.0, min(2.0, u))

    def world_vel_to_body(self, wx, wy):
        psi = self.yaw_bias.correct(self.yaw_ekf)
        c, s = math.cos(psi), math.sin(psi)
        return wx * c + wy * s, -wx * s + wy * c

    # ════════════════════════════════════════════════════════════
    # MISSION SEQUENCING
    # ════════════════════════════════════════════════════════════
    def trystart(self):
        if self.started or self.px is None or self.pz is None \
                or self.yaw_ekf is None:
            return
        if abs(self.yaw_rate) > 0.05:
            return
        self.started = True
        self.home_x, self.home_y = self.px, self.py

        # ONE-TIME yaw calibration: assume the vehicle is physically
        # facing the turbine right now (known spawn condition). This
        # solves the fixed EKF-yaw offset once and for all — no
        # further correction is applied for the rest of the mission.
        world_brg = math.atan2(self.TY - self.py, self.TX - self.px)
        self.yaw_bias.calibrate(world_brg, self.yaw_ekf)

        self.get_logger().info(
            f"MISSION START | pos=({self.px:.1f},{self.py:.1f},{self.pz:.2f}) "
            f"yaw_bias={math.degrees(self.yaw_bias.bias):.1f}° "
            f"(one-time calibration, spawn-facing-turbine assumption)")
        self._go(S.DESCEND)

    def _go(self, s):
        self.state = s
        self.phase_t = time.time()
        self._yaw_cmd = 0.0
        self._reset_vel_integrators()
        if s == S.SCAN:
            self.scan_going_up = True
            self.scan_cycles_done = 0
        if s == S.TRANSIT:
            self.transit_phase = 'rise'
            self.orbit_beta_prev = None
            self.orbit_swept = 0.0
            self.orbit_sign_checked = False
            self.orbit_check_t = None
            self._descend_enter_t = None
            if hasattr(self, '_orbit_stall_ref'):
                del self._orbit_stall_ref
        self.get_logger().info(f"=== {s.name} ===")

    def _elapsed(self):
        return time.time() - self.phase_t if self.phase_t else 0.0

    def world_bearing(self):
        """Bearing of ROBOT as seen FROM the turbine — pure position,
        no yaw at all. Drives orbit arc progress in TRANSIT."""
        return math.atan2(self.py - self.TY, self.px - self.TX)

    # ════════════════════════════════════════════════════════════
    # MAIN LOOP (20 Hz)
    # ════════════════════════════════════════════════════════════
    def ctrl(self):
        if self.state in (S.IDLE, S.COMPLETE, S.ESTOP):
            self._pub_zero()
            return
        if self.px is None or self.yaw_ekf is None or self.pz is None:
            return
        if self.pz > self.D_MAX:
            self.get_logger().error(f"ESTOP pz={self.pz:.1f}m")
            self.state = S.ESTOP
            return

        now = time.time()
        self.tracker.predict(now)

        if   self.state == S.DESCEND:     self._do_descend(now)
        elif self.state == S.CLOSE_IN:    self._do_close_in(now)
        elif self.state == S.SCAN:        self._do_scan(now)
        elif self.state == S.TRANSIT:     self._do_transit(now)
        elif self.state == S.RISE:        self._do_rise(now)
        elif self.state == S.RETURN_HOME: self._do_return(now)

    # ────────────────────────────────────────────────────────────
    def _do_descend(self, now):
        dep = self.depth_ctrl(self.D_BASE)
        derr = abs(self.pz - self.D_BASE)

        # Confirmed via isolated thruster test (pure heave at full
        # DESCEND-start magnitude -> zero rotation) that the yaw swing
        # is NOT the allocation matrix or a physics transient -- it's
        # bearing_hold()'s own proportional term, which gets
        # disproportionate thruster authority (yaw column ~1.70 vs
        # heave column ~0.25) relative to how small the triggering
        # bearing error actually is, right when heave demand is at its
        # saturated peak. Withhold proportional yaw correction while
        # heave is still saturated/near-saturated (derr > 2.0); use
        # pure rate damping only, which can only oppose existing
        # rotation, not initiate it. Once heave eases, hand off to full
        # bearing_hold() exactly as before -- that half was already
        # confirmed working (rock-steady through CLOSE_IN/SCAN).
        if derr > 2.0:
            if self._elapsed() < 5.0:
                self._full(0, 0, dep, 0.0)
                return
            # Damping alone only opposes ACTIVE rotation -- once
            # yaw_rate settles it goes to zero too, leaving no force
            # to correct a wrong-but-stationary heading. Confirmed in
            # telemetry: zero-proportional gating caused world_b to
            # park at ~180 deg for 40+ seconds, only correcting in one
            # big swing right at the end. Add back a SMALL, tightly
            # capped proportional pull (1/4 of KP_BEAR, capped low) so
            # the vehicle continuously walks back toward the turbine
            # instead of parking -- weak enough not to re-trigger the
            # original large-swing disturbance, unlike full
            # bearing_hold() at this stage.
            damp = max(-0.15, min(0.15, -0.35 * self.yaw_rate))
            b = self.current_bearing()
            point = 0.0
            if b is not None:
                point = max(-0.08, min(0.08, 0.25 * self.KP_BEAR * b))
            self.get_logger().info(f"  [desc-dbg] t={self._elapsed():.0f} b={math.degrees(b) if b is not None else 999:.1f} yaw_rate={math.degrees(self.yaw_rate):.1f} point={point:.3f} damp={damp:.3f}")
            self._full(0, 0, dep, damp + point)
            return

        yaw_c = self.bearing_hold()
        self._full(0, 0, dep, yaw_c)

        b = self.current_bearing()
        if b is not None and abs(b) < math.radians(12) \
                and abs(self.yaw_rate) < math.radians(4):
            self.get_logger().info(
                f"Turbine locked (b={math.degrees(b):.0f}°, "
                f"yaw_bias={math.degrees(self.yaw_bias.bias):.0f}°) "
                f"→ CLOSE_IN")
            self._go(S.CLOSE_IN)

    # ────────────────────────────────────────────────────────────
    def _do_close_in(self, now):
        dep   = self.depth_ctrl(self.D_BASE)
        yaw_c = self.bearing_hold()
        b     = self.current_bearing()

        if not self.tracker.healthy(now):
            self._full(0, 0, dep, yaw_c)
            return

        # Ramp surge authority over CLOSE_IN_SURGE_RAMP_T seconds. Jumping
        # straight to V_SURGE_MAX induced enough disturbance torque to
        # throw bearing_hold from ~0° to 168° before it could recover
        # (observed directly in telemetry) — that swing is also what
        # produced the diagonal, crab-walking approach instead of a
        # straight-in one, since sway kept fighting a heading error
        # that hadn't caught up yet.
        ramp_t = min(1.0, self._elapsed() / self.CLOSE_IN_SURGE_RAMP_T)
        v_cap  = self.V_SURGE_MAX * ramp_t

        # Crab-correction priority (see CRAB_VY_LIMIT/CRAB_YAW_RATE_LIMIT
        # docstrings above): linearly throttle surge authority down as
        # EITHER measured sway OR measured yaw rate grows, floored so the
        # approach never fully stalls under sustained coupling. The yaw
        # rate term is the preventive half of this -- it reacts to a
        # developing rotation directly, before it grows into a real
        # heading swing that then shows up as sway (which is what the vy_b
        # term alone can only catch after the fact).
        crab_vy  = 1.0 - abs(self.vy_b) / self.CRAB_VY_LIMIT
        crab_yaw = 1.0 - abs(self.yaw_rate) / self.CRAB_YAW_RATE_LIMIT
        crab_factor = max(self.CRAB_CAP_FLOOR, min(crab_vy, crab_yaw))
        v_cap *= crab_factor

        r = self.tracker.range()
        v_sp = max(-self.V_SURGE_MAX,
                   min(v_cap,
                       self.KP_RANGE * (r - self.SCAN_DIST)))
        if self.sonar_valid and self.sonar_range < self.SONAR_STOP:
            v_sp = -0.3
        elif (not self.sonar_valid and self.sonar_invalid_since is not None
                and now - self.sonar_invalid_since > self.SONAR_DROPOUT_CAUTION_T):
            v_sp = min(v_sp, self.SONAR_DROPOUT_CREEP)  # sonar can't
            # confirm what's actually in front of the vehicle right now
            # -- creep rather than either stop dead (deadlocks, see
            # SONAR_DROPOUT_CREEP) or keep closing in blind at full speed
        if b is not None and abs(b) > math.radians(15):
            v_sp = 0.0
            self._reset_vel_integrators()  # kill accumulated windup —
            # otherwise stored integral error keeps pushing surge even
            # after the setpoint is zeroed, driving the vehicle forward
            # exactly when it's most misaligned (observed: ux=0.73
            # while v_sp=0.00 at b=-173° in telemetry)

        ux, uy = self.vel_ctrl(v_sp, 0.0)
        self._full(ux, uy, dep, yaw_c)

        self.get_logger().info(
            f"  CLOSE_IN r={r:.1f}m "
            f"b={(math.degrees(b) if b is not None else 999):.0f}° "
            f"v_sp={v_sp:.2f} vx_b={self.vx_b:.3f} vy_b={self.vy_b:.3f} "
            f"yaw_rate={math.degrees(self.yaw_rate):.1f}°/s "
            f"ux={ux:.2f} uy={uy:.2f} crab={crab_factor:.2f}",
            throttle_duration_sec=2.0)

        # Conjunctive exit predicate -- added a velocity-settled check
        # (|vx_b| < 0.08) so CLOSE_IN can't hand off to SCAN while still
        # carrying meaningful surge momentum, which previously let SCAN
        # begin misaligned and trace a curved path instead of a clean
        # vertical sweep (same failure mode already fixed for TRANSIT's
        # descend->SCAN handoff). Added the matching |vy_b| check for
        # the same reason: telemetry showed sway sitting at 0.13-0.21
        # m/s right through the old handoff, which is exactly what let
        # SCAN face 0 begin displaced off to one side instead of
        # centered on the intended approach line.
        if (abs(r - self.SCAN_DIST) < 0.7
                and abs(self.pz - self.D_BASE) < self.DEPTH_TOL_PHASE
                and b is not None and abs(b) < math.radians(10)
                and abs(self.vx_b) < 0.08
                and abs(self.vy_b) < 0.08):
            self.get_logger().info(
                f"At r={r:.1f}m → SCAN face {self.face_idx}")
            self._go(S.SCAN)
            return

        if self._elapsed() > self.CLOSE_IN_TIMEOUT:
            self.get_logger().warn(
                f"  CLOSE_IN timeout → SCAN anyway "
                f"(vx_b={self.vx_b:.3f} vy_b={self.vy_b:.3f})")
            self._go(S.SCAN)

    # ────────────────────────────────────────────────────────────
    def _do_scan(self, now):
        """Turbine-relative station-keep during the depth sweep:
             yaw   : world-model bearing → 0
             surge : tracker range → R      (velocity loop)
             sway  : v_y → 0                (DVL — kills lateral drift)"""
        target_d = self.D_TOP if self.scan_going_up else self.D_BASE
        dep   = self.depth_ctrl(target_d)
        yaw_c = self.bearing_hold()

        r = self.tracker.range()
        if r is not None and self.tracker.healthy(now):
            vx_sp = max(-self.V_SCAN_MAX,
                        min(self.V_SCAN_MAX,
                            self.KP_RANGE * (r - self.SCAN_DIST)))
        else:
            vx_sp = 0.0
        if self.sonar_valid and self.sonar_range < self.SONAR_STOP:
            vx_sp = -0.3
        elif (not self.sonar_valid and self.sonar_invalid_since is not None
                and now - self.sonar_invalid_since > self.SONAR_DROPOUT_CAUTION_T):
            # Same reasoning as CLOSE_IN's SONAR_DROPOUT_CAUTION_T --
            # confirmed via telemetry that a real yaw kick during SCAN
            # landed the tick right after a sonar dropout here too, not
            # just during CLOSE_IN's approach.
            vx_sp = min(vx_sp, self.SONAR_DROPOUT_CREEP)

        ux, uy = self.vel_ctrl(vx_sp, 0.0)
        self._full(ux, uy, dep, yaw_c)

        self.get_logger().info(
            f"  SCAN f{self.face_idx} z={self.pz:.1f}→{target_d} "
            f"r={(r if r else -1):.1f} "
            f"cyc={self.scan_cycles_done}/{self.SCAN_FULL_CYCLES}",
            throttle_duration_sec=2.0)

        if self._elapsed() > self.SCAN_TIMEOUT:
            self.get_logger().warn("  SCAN timeout → next face")
            self._advance()
            return

        if abs(self.pz - target_d) < self.DEPTH_TOL_SCAN:
            if self.scan_going_up:
                self.scan_going_up = False
                self.get_logger().info(f"Face {self.face_idx}: top → down")
            else:
                self.scan_cycles_done += 1
                if self.scan_cycles_done >= self.SCAN_FULL_CYCLES:
                    self._advance()
                else:
                    self.scan_going_up = True

    # ────────────────────────────────────────────────────────────
    def _do_transit(self, now):
        """Orbit to the next face. Nose held on the WORLD-MODEL bearing
        → orbit is pure sway. Arc progress from WORLD BEARING of the
        robot around the turbine (EKF x,y only). Direction commanded
        from geometry, then verified against measured drift."""
        if self._elapsed() > self.TRANSIT_TIMEOUT:
            self.get_logger().warn("  TRANSIT timeout → SCAN anyway")
            self._go(S.SCAN)
            return

        # ── rise ────────────────────────────────────────────────
        if self.transit_phase == 'rise':
            dep = self.depth_ctrl(self.D_TRANSIT)
            yaw_c = self.bearing_hold()
            self._full(0, 0, dep, yaw_c)
            if abs(self.pz - self.D_TRANSIT) < self.DEPTH_TOL_PHASE:
                beta_now = self.world_bearing()
                tgt_face = 0 if self.returning_to_face0 else self.face_idx
                beta_tgt = math.radians(self.FACE_BEARINGS[tgt_face])
                d = wrap(beta_tgt - beta_now)
                self.orbit_sign = 1.0 if d > 0 else -1.0
                self.orbit_target_arc = abs(d)
                self.orbit_beta_prev = beta_now
                self.orbit_swept = 0.0
                self.orbit_check_t = now
                self.transit_phase = 'orbit'
                self.get_logger().info(
                    f"  TRANSIT: orbit {math.degrees(d):+.0f}° "
                    f"to face {self.face_idx}")
            return

        # ── orbit ───────────────────────────────────────────────
        if self.transit_phase == 'orbit':
            # Reverted to nose-on-turbine + pure sway for the tangential
            # motion (matches CLOSE_IN/SCAN pointing, keeps the turbine
            # in view for SLAM continuity). The earlier travel-heading
            # redesign traded this for surge-based travel, but once the
            # PI fix in vel_ctrl() resolved the real root cause (missing
            # integral term causing ~6x steady-state speed loss on ANY
            # axis, not just sway), that complexity bought nothing —
            # and empirically introduced a new problem: at the much
            # higher post-PI-fix speed, small heading misalignment during
            # the travel-heading reorientation produced radial drift
            # (r fell from 19m toward 12.5m mid-orbit, heading toward the
            # collision guard) faster than the weak radial correction
            # term could counteract. Nose-on-turbine sidesteps that
            # entirely: the radial axis IS the pointing axis, so range
            # hold is exact by construction, not a secondary correction
            # fighting a fast, imperfectly-aligned tangential motion.
            dep   = self.depth_ctrl(self.D_TRANSIT)
            yaw_c = self.bearing_hold()

            beta_now = self.world_bearing()
            db = wrap(beta_now - self.orbit_beta_prev)
            self.orbit_beta_prev = beta_now
            self.orbit_swept += db * self.orbit_sign
            remaining = self.orbit_target_arc - self.orbit_swept

            if not self.orbit_sign_checked and \
                    (now - self.orbit_check_t) > 4.0:
                if self.orbit_swept < math.radians(-3):
                    self.orbit_sign *= -1.0
                    self.orbit_swept = -self.orbit_swept
                    self.get_logger().warn(
                        "  TRANSIT: orbit direction flipped (self-check)")
                self.orbit_sign_checked = True

            v_t = self.V_ORBIT
            if remaining < math.radians(15):
                v_t = max(0.18, self.V_ORBIT * remaining / math.radians(15))
            vy_sp = -self.orbit_sign * v_t

            r = self.tracker.range()
            vx_sp = 0.0
            if r is not None and self.tracker.healthy(now):
                vx_sp = max(-0.3, min(0.3,
                            self.KP_RANGE * (r - self.SCAN_DIST)))
            if self.sonar_valid and self.sonar_range < self.SONAR_STOP:
                vx_sp = -0.3
            elif (not self.sonar_valid and self.sonar_invalid_since is not None
                    and now - self.sonar_invalid_since > self.SONAR_DROPOUT_CAUTION_T):
                # Same reasoning as CLOSE_IN/SCAN's SONAR_DROPOUT_CAUTION_T
                # -- caps radial closing speed only, orbit progress (vy_sp)
                # is left alone since it's not the axis that closes
                # distance to the structure.
                vx_sp = min(vx_sp, self.SONAR_DROPOUT_CREEP)

            ux, uy = self.vel_ctrl(vx_sp, vy_sp)
            self._full(ux, uy, dep, yaw_c)

            self.get_logger().info(
                f"  ORBIT swept={math.degrees(self.orbit_swept):.0f}/"
                f"{math.degrees(self.orbit_target_arc):.0f}° "
                f"r={(r if r else -1):.1f} "
                f"rem={math.degrees(remaining):.0f}° "
                f"v_t_sp={v_t:.2f} vx_b={self.vx_b:.3f} vy_b={self.vy_b:.3f} "
                f"ux={ux:.2f} uy={uy:.2f}",
                throttle_duration_sec=2.0)

            # Widened from 5° — 8° of residual arc at 17m standoff is
            # operationally negligible and this threshold was rarely
            # reached before timeout under the old value.
            if remaining < math.radians(8):
                self.transit_phase = 'descend'
                self.get_logger().info(
                    f"  TRANSIT: arc done → descend face {self.face_idx}")
                return

            # Stall escape: if progress hasn't meaningfully advanced in
            # 30s despite a nonzero commanded speed, something is
            # silently absorbing the tangential command. Rather than
            # ride out the full 500s timeout every time, force the arc
            # closed once we're already this close (<12°) and stalled.
            if not hasattr(self, '_orbit_stall_ref'):
                self._orbit_stall_ref = (now, self.orbit_swept)
            stall_t, stall_swept = self._orbit_stall_ref
            if now - stall_t > 30.0:
                if abs(self.orbit_swept - stall_swept) < math.radians(1.5) \
                        and remaining < math.radians(12):
                    self.get_logger().warn(
                        f"  TRANSIT: orbit stalled at rem="
                        f"{math.degrees(remaining):.0f}° — forcing "
                        f"descend rather than riding out full timeout")
                    self.transit_phase = 'descend'
                    del self._orbit_stall_ref
                    return
                self._orbit_stall_ref = (now, self.orbit_swept)
            return

        # ── descend ─────────────────────────────────────────────
        if self.transit_phase == 'descend':
            dep   = self.depth_ctrl(self.D_BASE)
            yaw_c = self.bearing_hold()
            r = self.tracker.range()
            vx_sp = 0.0
            if r is not None and self.tracker.healthy(now):
                vx_sp = max(-0.3, min(0.3,
                            self.KP_RANGE * (r - self.SCAN_DIST)))
            ux, uy = self.vel_ctrl(vx_sp, 0.0)
            self._full(ux, uy, dep, yaw_c)

            if not hasattr(self, '_descend_enter_t') or self._descend_enter_t is None:
                self._descend_enter_t = now

            # Require both depth AND residual velocity settled before
            # handing off to SCAN — the orbit can leave meaningful sway
            # momentum (observed up to ~0.24 m/s) that depth-only gating
            # let bleed into the start of the depth sweep, showing up as
            # SCAN appearing to begin early/misaligned and tracing a
            # curved path instead of a straight vertical line. A 15s
            # safety timeout prevents an indefinite wait if velocity
            # noise never quite clears the threshold.
            settled = abs(self.vx_b) < 0.05 and abs(self.vy_b) < 0.05
            timed_out = (now - self._descend_enter_t) > 15.0
            if abs(self.pz - self.D_BASE) < self.DEPTH_TOL_PHASE \
                    and (settled or timed_out):
                self._descend_enter_t = None
                if self.returning_to_face0:
                    self.get_logger().info("Back at face 0 → RISE")
                    self._go(S.RISE)
                else:
                    self.get_logger().info(f"At face {self.face_idx} → SCAN")
                    self._go(S.SCAN)

    # ────────────────────────────────────────────────────────────
    def _do_rise(self, now):
        # Final ascent to the surface before returning home. Point
        # toward the spawn waypoint while rising so RETURN_HOME starts
        # already roughly aligned. Turbine-relative machinery (sonar,
        # bearing_hold) is NOT used here — we're leaving the structure.
        dep = self.depth_ctrl(self.D_RISE)
        home_psi = math.atan2(self.home_y - self.py,
                              self.home_x - self.px)
        yaw_c = self.yaw_hold_world(home_psi)
        self._full(0, 0, dep, yaw_c)
        if abs(self.pz - self.D_RISE) < self.DEPTH_TOL_PHASE:
            self._go(S.RETURN_HOME)

    # ────────────────────────────────────────────────────────────
    def _do_return(self, now):
        """Pure spawn-waypoint controller. Deliberately uses NONE of the
        turbine-relative machinery (sonar range guard, tracker, bearing
        hold) — those only make sense near the structure and were the
        cause of the surface thrashing observed at mission end (sonar
        returns garbage at the surface pointing away from the turbine,
        so the SONAR_STOP guard kept flipping the surge command). Here
        it's just: hold surface depth, point at spawn, close distance
        with the PI velocity loop against continuously-updated EKF
        position."""
        dx, dy = self.home_x - self.px, self.home_y - self.py
        dh = math.hypot(dx, dy)
        dep = self.depth_ctrl(self.D_RISE)

        home_psi = math.atan2(dy, dx)
        yaw_c = self.yaw_hold_world(home_psi)

        # Only drive forward once roughly pointed at home, so a large
        # initial heading error produces a clean turn-then-go rather
        # than a curved skid across the surface.
        psi = self.yaw_bias.correct(self.yaw_ekf)
        head_err = abs(wrap(home_psi - psi))
        v = max(0.0, min(0.5, 0.25 * dh)) if head_err < math.radians(35) else 0.0

        wx, wy = v * dx / max(dh, 0.1), v * dy / max(dh, 0.1)
        vx_sp, vy_sp = self.world_vel_to_body(wx, wy)
        ux, uy = self.vel_ctrl(vx_sp, vy_sp)
        self._full(ux, uy, dep, yaw_c)

        self.get_logger().info(
            f"  HOME d={dh:.1f}m head_err={math.degrees(head_err):.0f}°",
            throttle_duration_sec=2.0)
        if dh < self.POS_TOL_HOME:
            self.get_logger().info("=== MISSION COMPLETE ===")
            self._go(S.COMPLETE)
        elif self._elapsed() > self.RETURN_TIMEOUT:
            self.get_logger().warn("=== HOME TIMEOUT ===")
            self._go(S.COMPLETE)

    # ────────────────────────────────────────────────────────────
    def _advance(self):
        self.face_idx += 1
        if self.face_idx >= 4:
            # All faces done — one final transit back to face 0's
            # bearing (staying on the known orbit rather than cutting
            # across open water), then RISE + RETURN_HOME.
            self.get_logger().info(
                "All 4 faces scanned → TRANSIT back to face 0, then RISE")
            self.returning_to_face0 = True
            self._go(S.TRANSIT)
        else:
            self.get_logger().info(f"→ TRANSIT to face {self.face_idx}")
            self._go(S.TRANSIT)

    # ════════════════════════════════════════════════════════════
    # LOGGING / STATUS
    # ════════════════════════════════════════════════════════════
    def log_row(self):
        if self.px is None or self.state == S.IDLE:
            return
        r = self.tracker.range()
        b = self.current_bearing()
        self.lw.writerow([
            f"{time.time():.1f}", self.state.name, self.face_idx,
            f"{self.px:.2f}", f"{self.py:.2f}", f"{self.pz:.2f}",
            f"{math.degrees(self.yaw_ekf):.1f}",
            f"{math.degrees(self.yaw_bias.bias):.1f}",
            f"{r:.2f}" if r is not None else "",
            f"{math.degrees(b):.1f}" if b is not None else "",
            f"{self.sonar_range:.2f}", f"{self.radius.r:.2f}",
            f"{self.vx_b:.3f}", f"{self.vy_b:.3f}",
            self.map_pts, f"{self.nav_conf:.2f}"])
        self.lf.flush()

    def status(self):
        if self.px is None:
            return
        r = self.tracker.range()
        b = self.current_bearing()
        self.get_logger().info(
            f"[{self.state.name}][{self._elapsed():.0f}s] z={self.pz:.1f} "
            f"trk_r={(r if r is not None else -1):.1f}m "
            f"world_b={(math.degrees(b) if b is not None else 999):.0f}° "
            f"bias={math.degrees(self.yaw_bias.bias):.0f}° "
            f"face={self.face_idx} son={self.sonar_range:.1f} "
            f"map={self.map_pts} c={self.nav_conf:.2f}")

    # ════════════════════════════════════════════════════════════
    # THRUSTERS (identical math throughout this project — proven)
    # ════════════════════════════════════════════════════════════
    def _full(self, surge, sway, heave, yaw):
        # No sign negation — the allocation matrix is calibrated to the
        # sim's measured response, so all signs are correct as-is.
        tau = np.array([surge, sway, heave, 0.0, 0.0, yaw]) * self.WRENCH_SCALE
        f = self.T_PINV @ tau
        m = np.max(np.abs(f))
        if m > 1.0:
            f /= m
        msg = Float64MultiArray()
        msg.data = [max(-1.0, min(1.0, float(v))) for v in f]
        self.pwm.publish(msg)

    def _pub_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 8
        self.pwm.publish(msg)


# ═══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = InspectV8()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node._pub_zero()
            node.lf.close()
        except Exception:
            pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
