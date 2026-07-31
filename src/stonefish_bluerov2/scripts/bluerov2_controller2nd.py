#!/usr/bin/env python3
"""
DeepSight — Full Autonomous Inspection Controller v4
Two-turbine inspection mission with sonar fallback for dark water.

Mission sequence:
  T2: APPROACH → FACE+DESCEND → CLOSE_IN → ORBIT_BASE → ORBIT_MID → PASS_UNDER → RISE
  T1: TRANSIT → FACE+DESCEND → CLOSE_IN → ORBIT_BASE → ORBIT_MID → RISE
  RETURN HOME

Features:
  - Waypoint segment list (clean extensible architecture)
  - Polar coordinate orbit (validated stable ±0.1m radius)
  - Two-phase approach: face+descend first, surge second
  - SLAM primary / INS auto-fallback (2s timeout)
  - Sonar proximity guard (collision avoidance in dark water)
  - Per-phase timeouts prevent mission hang
  - Inspection waypoint CSV logged every 45deg
  - Map density monitoring and flagging
  - Colour anomaly detection (OpenCV HSV)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
import math, time, csv, os
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional

try:
    import cv2
    import numpy as np
    from sensor_msgs.msg import Image
    CV2_OK = True
except ImportError:
    CV2_OK = False

# ════════════════════════════════════════════════════════════════════
# PID Controller
# ════════════════════════════════════════════════════════════════════
class PID:
    def __init__(self, kp, ki, kd, out_min=-1.0, out_max=1.0, ilimit=5.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.ilimit = ilimit
        self._i = self._pe = 0.0
        self._pt = None

    def reset(self):
        self._i = self._pe = 0.0
        self._pt = None

    def compute(self, sp, pv, now=None):
        if now is None: now = time.time()
        e  = sp - pv
        dt = max((now - self._pt) if self._pt else 0.05, 1e-4)
        self._i  = max(-self.ilimit, min(self.ilimit, self._i + e*dt))
        d  = (e - self._pe) / dt
        self._pe = e
        self._pt = now
        return max(self.out_min, min(self.out_max,
               self.kp*e + self.ki*self._i + self.kd*d))

# ════════════════════════════════════════════════════════════════════
# Mission segment definitions
# ════════════════════════════════════════════════════════════════════
class Act(Enum):
    FACE_DESCEND = auto()  # rotate to face target + descend, no surge
    CLOSE_IN     = auto()  # surge to approach_dist at constant depth
    ORBIT        = auto()  # polar orbit for N laps
    PASS_UNDER   = auto()  # descend to under_depth, pass through
    RISE         = auto()  # ascend to target depth
    RETURN       = auto()  # return to spawn

@dataclass
class Seg:
    name:         str
    act:          Act
    tx:           float      # target X (world)
    ty:           float      # target Y (world)
    depth:        float      # target depth
    laps:         float = 1.0
    radius:       float = 4.5
    approach_dist:float = 5.0
    timeout:      float = 180.0

# ════════════════════════════════════════════════════════════════════
# World geometry  (from scenario file)
# ════════════════════════════════════════════════════════════════════
T2X, T2Y = 20.0,   0.0   # turbine2 XY
T1X, T1Y = -20.0,  0.0   # turbine1 XY

# Depth reference (INS world Z, larger = deeper):
#   Robot spawns at Z≈0, turbine2 base at Z≈23
D_SAFE  = 18.0   # transit / approach depth
D_T2B   = 23.0   # turbine2 base orbit depth
D_T2M   = 20.0   # turbine2 mid orbit depth
D_UNDER = 26.5   # pass-under depth
D_T1B   = 20.0   # turbine1 base orbit depth (shallower, T1 at Z=20)
D_T1M   = 17.0   # turbine1 mid orbit depth
D_MAX   = 28.0   # emergency stop

MISSION: list[Seg] = [
    # ── Turbine 2 ──────────────────────────────────────────────────
    Seg("t2_face_descend", Act.FACE_DESCEND, T2X, T2Y, D_SAFE,   timeout=90),
    Seg("t2_close_in",     Act.CLOSE_IN,     T2X, T2Y, D_SAFE,   approach_dist=5.0, timeout=90),
    Seg("t2_orbit_base",   Act.ORBIT,        T2X, T2Y, D_T2B,    laps=1.0, radius=4.5, timeout=300),
    Seg("t2_orbit_mid",    Act.ORBIT,        T2X, T2Y, D_T2M,    laps=1.0, radius=5.0, timeout=300),
    Seg("t2_pass_under",   Act.PASS_UNDER,   T2X, T2Y, D_UNDER,  timeout=120),
    Seg("t2_rise",         Act.RISE,         T2X, T2Y, D_SAFE,   timeout=60),
    # ── Transit to Turbine 1 ───────────────────────────────────────
    Seg("t1_face_descend", Act.FACE_DESCEND, T1X, T1Y, D_SAFE,   timeout=90),
    Seg("t1_close_in",     Act.CLOSE_IN,     T1X, T1Y, D_SAFE,   approach_dist=5.0, timeout=180),
    Seg("t1_orbit_base",   Act.ORBIT,        T1X, T1Y, D_T1B,    laps=1.0, radius=4.5, timeout=300),
    Seg("t1_orbit_mid",    Act.ORBIT,        T1X, T1Y, D_T1M,    laps=1.0, radius=5.0, timeout=300),
    Seg("t1_rise",         Act.RISE,         T1X, T1Y, D_SAFE,   timeout=60),
    # ── Return home ────────────────────────────────────────────────
    Seg("return_home",     Act.RETURN,       0.0,  0.0, 5.0,     approach_dist=3.0, timeout=300),
]

# ════════════════════════════════════════════════════════════════════
# Controller node
# ════════════════════════════════════════════════════════════════════
class DeepSightV4(Node):

    ORBIT_SPEED   = 0.04   # rad/s
    SLAM_TIMEOUT  = 2.0    # seconds before INS fallback
    SONAR_STOP    = 3.0    # stop surging if sonar < this (m)
    LOG_INTERVAL  = 45.0   # degrees between inspection waypoints

    def __init__(self):
        super().__init__('deepsight_v4')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        # Publishers
        self.pwm_pub = self.create_publisher(
            Float64MultiArray, '/bluerov2/setpoint/pwm', 10)

        # Subscribers
        self.create_subscription(
            Odometry,    '/bluerov2/odometry',        self._odom_cb,   qos)
        self.create_subscription(
            PoseStamped, '/bluerov2/robot_pose_slam', self._slam_cb,   qos)
        self.create_subscription(
            PointCloud2, '/bluerov2/map_points',      self._map_cb,    qos)
        self.create_subscription(
            LaserScan,   '/bluerov2/multibeam',       self._sonar_cb,  qos)
        if CV2_OK:
            self.create_subscription(
                Image, '/bluerov2/left/image_color',  self._img_cb,    qos)

        # ── INS state ───────────────────────────────────────────────
        self.px = self.py = self.pz = self.yaw = None

        # ── SLAM state ──────────────────────────────────────────────
        self.sx = self.sy = self.sz = self.syaw = None
        self.slam_t   = 0.0
        self.slam_ok  = False

        # ── Sonar ───────────────────────────────────────────────────
        self.sonar_front = 999.0
        self.sonar_min   = 999.0

        # ── Map density ─────────────────────────────────────────────
        self.map_pts = 0

        # ── Anomaly detection ───────────────────────────────────────
        self.anomaly     = False
        self.anomaly_area = 0

        # ── Mission state ───────────────────────────────────────────
        self.seg_idx   = -1
        self.seg: Optional[Seg] = None
        self.phase_t   = None
        self.started   = False

        # Per-segment state
        self.orbit_angle = 0.0
        self.orbit_total = 0.0
        self.last_log_angle = -999.0
        self.exit_x = self.exit_y = None
        self.spawn_x = self.spawn_y = self.spawn_z = None

        # Face+descend: track when facing complete
        self.facing_done = False

        # ── PIDs ────────────────────────────────────────────────────
        self.pid_d = PID(kp=0.5, ki=0.03, kd=0.15, out_min=-1.0, out_max=1.0)
        self.pid_y = PID(kp=0.6, ki=0.02, kd=0.15, out_min=-0.7, out_max=0.7)
        self.pid_s = PID(kp=0.35,ki=0.01, kd=0.10, out_min=-0.6, out_max=0.6)

        # ── Inspection log ──────────────────────────────────────────
        log_dir = os.path.expanduser('~/ros2_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self._lf = open(f'{log_dir}/inspection_{ts}.csv', 'w', newline='')
        self._lw = csv.writer(self._lf)
        self._lw.writerow([
            'time','segment','orbit_deg',
            'ins_x','ins_y','ins_z',
            'slam_x','slam_y','slam_z',
            'map_pts','sonar_front',
            'anomaly','anomaly_area_px'])

        # ── Timers ──────────────────────────────────────────────────
        self.create_timer(0.05, self._ctrl)
        self.create_timer(1.0,  self._status)
        self.create_timer(3.0,  self._start)

        self.get_logger().info(
            "DeepSight v4 ready | "
            f"T2=({T2X},{T2Y})  T1=({T1X},{T1Y}) | "
            f"{len(MISSION)} segments")

    # ════════════════════════════════════════════════════════════════
    # Callbacks
    # ════════════════════════════════════════════════════════════════
    def _odom_cb(self, m):
        self.px  = m.pose.pose.position.x
        self.py  = m.pose.pose.position.y
        self.pz  = m.pose.pose.position.z
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def _slam_cb(self, m):
        self.sx   =  m.pose.position.x
        self.sy   =  m.pose.position.y
        self.sz   = -m.pose.position.z   # invert depth
        q = m.pose.orientation
        self.syaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.slam_t  = time.time()
        self.slam_ok = True

    def _map_cb(self, m):
        self.map_pts = m.width * m.height

    def _sonar_cb(self, m):
        valid = [r for r in m.ranges if 0.1 < r < m.range_max]
        self.sonar_min = min(valid) if valid else 999.0
        mid   = len(m.ranges)//2
        front = [r for r in m.ranges[max(0,mid-5):mid+5]
                 if 0.1 < r < m.range_max]
        self.sonar_front = min(front) if front else 999.0

    def _img_cb(self, m):
        if not CV2_OK: return
        try:
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(
                (m.height, m.width, 3))
            hsv  = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            struc = cv2.inRange(hsv,
                np.array([15,60,40]), np.array([40,255,200]))
            dark  = cv2.inRange(hsv,
                np.array([0,0,0]),   np.array([180,255,60]))
            anom  = cv2.bitwise_and(dark, struc)
            area  = cv2.countNonZero(anom)
            self.anomaly      = area > 500
            self.anomaly_area = area
        except Exception:
            pass

    # ════════════════════════════════════════════════════════════════
    # Best available pose  (SLAM primary, INS fallback)
    # ════════════════════════════════════════════════════════════════
    def _pose(self):
        fresh = (time.time() - self.slam_t) < self.SLAM_TIMEOUT
        if self.sx is not None and fresh:
            if not self.slam_ok:
                self.get_logger().info("SLAM recovered")
            self.slam_ok = True
            return self.sx, self.sy, self.sz, self.syaw, "SLAM"
        else:
            if self.slam_ok:
                self.get_logger().warn("SLAM LOST — INS fallback")
            self.slam_ok = False
            return self.px, self.py, self.pz, self.yaw, "INS"

    # ════════════════════════════════════════════════════════════════
    # Mission management
    # ════════════════════════════════════════════════════════════════
    def _start(self):
        if self.started or self.px is None: return
        self.started  = True
        self.spawn_x  = self.px
        self.spawn_y  = self.py
        self.spawn_z  = self.pz
        # Update return-home target to actual spawn
        MISSION[-1].tx = self.spawn_x
        MISSION[-1].ty = self.spawn_y
        dist = self._d2(self.px,self.py,T2X,T2Y)
        self.get_logger().info(
            f"MISSION START | spawn=({self.px:.1f},{self.py:.1f},"
            f"{self.pz:.2f}) | T2 dist={dist:.1f}m")
        self._next()

    def _next(self):
        self.seg_idx += 1
        if self.seg_idx >= len(MISSION):
            self.get_logger().info("=== ALL SEGMENTS COMPLETE — MISSION DONE ===")
            self.seg = None
            return
        self.seg          = MISSION[self.seg_idx]
        self.phase_t      = time.time()
        self.orbit_angle  = 0.0
        self.orbit_total  = 0.0
        self.last_log_angle = -999.0
        self.facing_done  = False
        for p in [self.pid_d, self.pid_y, self.pid_s]: p.reset()
        self.get_logger().info(
            f"=== SEG {self.seg_idx+1}/{len(MISSION)}: "
            f"{self.seg.name.upper()} ===")

    def _elapsed(self):
        return time.time()-self.phase_t if self.phase_t else 0.0

    def _timeout(self):
        if self.seg and self._elapsed() > self.seg.timeout:
            self.get_logger().warn(
                f"TIMEOUT {self.seg.name} after {self.seg.timeout:.0f}s")
            self._next()
            return True
        return False

    # ════════════════════════════════════════════════════════════════
    # Main control loop  20 Hz
    # ════════════════════════════════════════════════════════════════
    def _ctrl(self):
        if self.seg is None or self.px is None or self.yaw is None:
            self._pwm(0,0,0,0,0,0,0,0); return
        if self.pz > D_MAX:
            self.get_logger().error(f"EMERGENCY STOP pz={self.pz:.1f}m")
            self._pwm(0,0,0,0,0,0,0,0)
            self.seg = None; return
        if self._timeout(): return

        px,py,pz,pyaw,src = self._pose()
        tx,ty = self.seg.tx, self.seg.ty
        now   = time.time()
        dist  = self._d2(px,py,tx,ty)
        des_y = math.atan2(ty-py, tx-px)
        yerr  = abs(self._adiff(des_y, pyaw))
        yaw_c = self.pid_y.compute(des_y, pyaw, now)
        dep_c = self.pid_d.compute(self.seg.depth, pz, now)

        # ── FACE + DESCEND ─────────────────────────────────────────
        if self.seg.act == Act.FACE_DESCEND:
            # Phase A: rotate to face target while descending
            # No surge — SLAM maps the front face top-to-bottom
            self._full(0.0, 0.0, dep_c, yaw_c)
            self.get_logger().info(
                f"  [FACE+DESCEND] depth={pz:.1f}→{self.seg.depth} "
                f"yaw_err={math.degrees(yerr):.1f}deg dist={dist:.1f}m",
                throttle_duration_sec=2.0)
            if abs(pz-self.seg.depth) < 1.5 and yerr < math.radians(15):
                self.get_logger().info(
                    f"Depth {pz:.1f}m reached, facing target. → next")
                self._next()

        # ── CLOSE IN ───────────────────────────────────────────────
        elif self.seg.act == Act.CLOSE_IN:
            # Surge toward target at constant depth
            # Sonar guard: stop if too close to structure
            sonar_ok = self.sonar_front > self.SONAR_STOP
            surge_c  = self.pid_s.compute(
                0.0, dist-self.seg.approach_dist, now) \
                if yerr < math.radians(20) and sonar_ok else 0.0
            self._full(surge_c, 0.0, dep_c, yaw_c)
            self.get_logger().info(
                f"  [CLOSE IN] dist={dist:.1f}m→{self.seg.approach_dist} "
                f"sonar={self.sonar_front:.1f}m",
                throttle_duration_sec=2.0)
            if dist < self.seg.approach_dist+0.5 and abs(pz-self.seg.depth)<2.0:
                self.get_logger().info(
                    f"Arrived dist={dist:.1f}m depth={pz:.1f}m → next")
                self._next()

        # ── ORBIT ──────────────────────────────────────────────────
        elif self.seg.act == Act.ORBIT:
            self._orbit(now, tx, ty, self.seg.depth, self.seg.radius)
            deg = math.degrees(self.orbit_total)
            # Inspection waypoint every 45 degrees
            if deg - self.last_log_angle >= self.LOG_INTERVAL:
                self.last_log_angle = deg
                self._log(deg)
                if 0 < self.map_pts < 500:
                    self.get_logger().warn(
                        f"LOW MAP DENSITY @ {deg:.0f}deg: {self.map_pts} pts")
                if self.anomaly:
                    self.get_logger().warn(
                        f"ANOMALY DETECTED @ {deg:.0f}deg "
                        f"area={self.anomaly_area}px "
                        f"pos=({px:.1f},{py:.1f},{pz:.1f})")
            self.get_logger().info(
                f"  orbit={deg:.0f}/360 depth={pz:.1f} "
                f"map={self.map_pts} sonar={self.sonar_front:.1f}m",
                throttle_duration_sec=2.0)
            if self.orbit_total >= 2*math.pi*self.seg.laps:
                self.get_logger().info(
                    f"Orbit complete {deg:.0f}deg → next")
                self._next()

        # ── PASS UNDER ─────────────────────────────────────────────
        elif self.seg.act == Act.PASS_UNDER:
            surge_c = self.pid_s.compute(0.0, dist-2.5, now) \
                      if yerr < math.radians(25) else 0.0
            self._shy(surge_c, dep_c, yaw_c)
            self.get_logger().info(
                f"  [PASS UNDER] dist={dist:.1f}m depth={pz:.1f}→{self.seg.depth}",
                throttle_duration_sec=2.0)
            if abs(pz-self.seg.depth) < 1.5 and dist < 3.0:
                angle_out = math.atan2(py-ty, px-tx)
                self.exit_x = tx + 8.0*math.cos(angle_out)
                self.exit_y = ty + 8.0*math.sin(angle_out)
                self.get_logger().info(
                    f"Under position. Exit=({self.exit_x:.1f},{self.exit_y:.1f})")
                self._next()

        # ── RISE ───────────────────────────────────────────────────
        elif self.seg.act == Act.RISE:
            self._hy(dep_c, yaw_c)
            self.get_logger().info(
                f"  [RISING] {pz:.1f}→{self.seg.depth}",
                throttle_duration_sec=2.0)
            if abs(pz-self.seg.depth) < 1.0:
                self.get_logger().info("Risen to safe depth → next")
                self._next()

        # ── RETURN HOME ────────────────────────────────────────────
        elif self.seg.act == Act.RETURN:
            surge_c = self.pid_s.compute(
                0.0, dist-self.seg.approach_dist, now) \
                if yerr < math.radians(20) else 0.0
            self._full(surge_c, 0.0, dep_c, yaw_c)
            self.get_logger().info(
                f"  [RETURN] dist={dist:.1f}m depth={pz:.1f}",
                throttle_duration_sec=2.0)
            if dist < self.seg.approach_dist+0.5 and abs(pz-self.seg.depth)<2.0:
                self.get_logger().info("=== HOME — MISSION COMPLETE! ===")
                self.seg = None

    # ════════════════════════════════════════════════════════════════
    # Polar orbit  (validated stable ±0.1m radius)
    # ════════════════════════════════════════════════════════════════
    def _orbit(self, now, tx, ty, depth, radius):
        dx = self.px - tx
        dy = self.py - ty
        r  = math.sqrt(dx*dx + dy*dy)
        a  = math.atan2(dy, dx)
        step = self.ORBIT_SPEED * 0.05
        self.orbit_angle = a + step
        self.orbit_total += abs(step)
        re = r - radius
        tx_ = -math.sin(a);  ty_ =  math.cos(a)   # tangent
        rx_ = -math.cos(a);  ry_ = -math.sin(a)   # radial inward
        rc  = max(-0.4, min(0.4, -re * 0.15))
        wvx = 0.25*tx_ + rc*rx_
        wvy = 0.25*ty_ + rc*ry_
        sb  =  wvx*math.cos(self.yaw) + wvy*math.sin(self.yaw)
        sw  = -wvx*math.sin(self.yaw) + wvy*math.cos(self.yaw)
        des_y = math.atan2(ty-self.py, tx-self.px)
        self._full(
            max(-0.5,min(0.5,sb)),
            max(-0.4,min(0.4,sw)),
            self.pid_d.compute(depth, self.pz, now),
            self.pid_y.compute(des_y, self.yaw, now))

    # ════════════════════════════════════════════════════════════════
    # Inspection logging
    # ════════════════════════════════════════════════════════════════
    def _log(self, deg):
        self._lw.writerow([
            f"{time.time():.2f}",
            self.seg.name if self.seg else "?",
            f"{deg:.0f}",
            f"{self.px:.2f}", f"{self.py:.2f}", f"{self.pz:.2f}",
            f"{self.sx or 0:.2f}", f"{self.sy or 0:.2f}", f"{self.sz or 0:.2f}",
            self.map_pts, f"{self.sonar_front:.1f}",
            int(self.anomaly), self.anomaly_area])
        self._lf.flush()
        self.get_logger().info(
            f"WAYPOINT @ {deg:.0f}deg | "
            f"pos=({self.px:.1f},{self.py:.1f},{self.pz:.1f}) | "
            f"map={self.map_pts} sonar={self.sonar_front:.1f}m | "
            f"anomaly={self.anomaly}")

    # ════════════════════════════════════════════════════════════════
    # Status log  1 Hz
    # ════════════════════════════════════════════════════════════════
    def _status(self):
        if self.px is None or self.seg is None: return
        px,py,pz,_,src = self._pose()
        dist = self._d2(px,py,self.seg.tx,self.seg.ty)
        self.get_logger().info(
            f"[{self.seg.name}][{src}][{self._elapsed():.0f}s] "
            f"pos=({px:.1f},{py:.1f},{pz:.2f}) "
            f"yaw={math.degrees(self.yaw):.1f}deg "
            f"dist={dist:.1f}m orbit={math.degrees(self.orbit_total):.0f}deg "
            f"map={self.map_pts} sonar={self.sonar_front:.1f}m")

    # ════════════════════════════════════════════════════════════════
    # Thruster mixing
    # ════════════════════════════════════════════════════════════════
    def _hy(self, h, y):
        self._pwm(y,-y,y,-y,h,h,h,h)

    def _shy(self, s, h, y):
        self._pwm(s+y,s-y,-s+y,-s-y,h,h,h,h)

    def _full(self, s, sw, h, y):
        self._pwm(s+y+sw, s-y-sw, -s+y-sw, -s-y+sw, h,h,h,h)

    def _pwm(self, fr,fl,br,bl,dfr,dfl,dbr,dbl):
        c = lambda v: max(-1.0,min(1.0,float(v)))
        msg = Float64MultiArray()
        msg.data = [c(fr),c(fl),c(br),c(bl),c(dfr),c(dfl),c(dbr),c(dbl)]
        self.pwm_pub.publish(msg)

    # ════════════════════════════════════════════════════════════════
    # Utilities
    # ════════════════════════════════════════════════════════════════
    @staticmethod
    def _d2(x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2+(y2-y1)**2)

    @staticmethod
    def _adiff(a,b):
        d=a-b
        while d> math.pi: d-=2*math.pi
        while d<-math.pi: d+=2*math.pi
        return d

# ════════════════════════════════════════════════════════════════════
# Entry point
# ════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = DeepSightV4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._pwm(0,0,0,0,0,0,0,0)
        node._lf.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
