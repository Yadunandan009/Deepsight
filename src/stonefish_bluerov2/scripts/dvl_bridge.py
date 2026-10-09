#!/usr/bin/env python3
"""
Converts stonefish_ros2/DVL → geometry_msgs/TwistWithCovarianceStamped
so robot_localization EKF can fuse DVL velocity.

MOUNT ROTATION. The DVL is mounted rolled 180 deg about X:

    <sensor name="dvl" ...>
        <origin rpy="3.1416 0.0 0.0" xyz="0.0 0.0 0.0"/>

so it reports velocity in a frame where Y and Z point opposite to
base_link. This node previously copied m.velocity straight through and
stamped it frame_id='bluerov2/base_link', handing the EKF a Y (and Z)
velocity of the wrong sign.

HOW IT HID. The speed is unchanged by the flip, so every magnitude check
passes -- measured live 2026-10-09 against ground truth: |v| 0.236 m/s
both, while the components were truth (+0.044, +0.228, 0.000) against DVL
(+0.045, -0.229, -0.000). And until odom0 was removed from ekf.yaml on
2026-10-08 the filter was anchored on ground-truth position at 100 Hz,
which corrected the bad velocity away every cycle. It only became
load-bearing -- and only became visible -- once the anchor was cut, where
it drove position error to 15.9 m in 244 s and bearing to 40 deg while
IMU yaw stayed under 0.2 deg.

The rotation is a parameter rather than a hardcoded negation so that
changing the scenario mount does not silently reintroduce this.

SYSTEMATIC ERROR MODEL. Stonefish's DVL has one error term:

    <noise velocity="0.0015" altitude="0.001"/>

which is zero-mean white noise. That is not what limits a real DVL. White
noise integrates as a random walk, growing with sqrt(time) and wandering
in sign; the 2026-10-09 dead-reckoning run drifted 0.03% of distance over
132.8 m, and sigma_v*sqrt(dt*T)*sqrt(2) = 0.037 m predicted the measured
0.040 m with no fitted parameters. Real DVL dead reckoning drifts 0.1-1%
of distance because SCALE FACTOR and BEAM MISALIGNMENT dominate, and both
are fixed properties of the unit, so they integrate LINEARLY with distance
and never average out.

This node adds the two missing terms. The sensor resolves velocity in its
true beam frame, which differs from its assumed mount frame by a small
fixed rotation, and reports each axis with a small fixed gain error:

    v_meas = S . dR . v_dvl        S = diag(1+e_x, 1+e_y, 1+e_z)

The bridge then applies the NOMINAL mount rotation, because the filter has
no knowledge of dR or S -- that is the point. The residual handed to the
EKF is R_mount (S.dR - I) v, which is proportional to velocity and fixed in
the body frame, hence linear in distance travelled.

Both are drawn ONCE at startup from `error_seed` and held for the whole
run; drawing per sample would make them noise again. Changing the seed
draws a different unit, which is how you get a drift distribution across
runs rather than a single number.

Defaults are ZERO so the 2026-10-09 baseline reproduces exactly. Enable
with the datasheet values in `dvl_a50.yaml`. The published covariance is
deliberately NOT inflated to cover these terms: a real unit's reported
covariance describes its random error only, and inflating it here would
partly hide the effect being measured.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistWithCovarianceStamped
from stonefish_ros2.msg import DVL

class DVLBridge(Node):
    def __init__(self):
        super().__init__('dvl_bridge')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/bluerov2/dvl_twist', 10)
        # Mount orientation of the DVL relative to base_link, radians,
        # matching <origin rpy="..."> in the scenario.
        self.declare_parameter('mount_rpy', [math.pi, 0.0, 0.0])
        r, p_, y = self.get_parameter('mount_rpy').value
        self.R = self._rpy_to_matrix(r, p_, y)

        # --- systematic error model, drawn once and held ------------------
        # scale_sigma: 1-sigma per-axis gain error, fraction of velocity.
        #   The Nortek DVL A50 quotes +/-1.01% long-term velocity accuracy,
        #   so 0.0101 is the datasheet figure.
        # misalign_sigma_deg: 1-sigma per-axis angular error between the
        #   true beam frame and the assumed mount frame.
        self.declare_parameter('scale_sigma', 0.0)
        self.declare_parameter('misalign_sigma_deg', 0.0)
        self.declare_parameter('error_seed', 0)
        ss = float(self.get_parameter('scale_sigma').value)
        ms = math.radians(float(self.get_parameter('misalign_sigma_deg').value))
        seed = int(self.get_parameter('error_seed').value)

        rng = np.random.default_rng(seed)
        self.scale = 1.0 + rng.normal(0.0, ss, 3) if ss > 0 else np.ones(3)
        if ms > 0:
            dr, dp, dy = rng.normal(0.0, ms, 3)
        else:
            dr = dp = dy = 0.0
        self.dR = np.array(self._rpy_to_matrix(dr, dp, dy))
        # Fold the fixed terms into one matrix: v_out = Rm . S . dR . v_in.
        self.E = np.array(self.R) @ np.diag(self.scale) @ self.dR
        self.perfect = (ss <= 0.0 and ms <= 0.0)

        self.create_subscription(DVL, '/bluerov2/dvl_sim', self.cb, qos)
        self.get_logger().info(
            f'DVL bridge started; mount_rpy=({r:.4f}, {p_:.4f}, {y:.4f}) rad, '
            f'sign map on (x,y,z) = '
            f'({self.R[0][0]:+.0f}, {self.R[1][1]:+.0f}, {self.R[2][2]:+.0f})')
        if self.perfect:
            self.get_logger().warn(
                'DVL systematic error model DISABLED (scale_sigma=0, '
                'misalign_sigma_deg=0): white noise only, drift will be '
                'optimistic by roughly an order of magnitude.')
        else:
            self.get_logger().warn(
                f'DVL systematic error ENABLED, seed={seed}. Realised unit: '
                f'scale=({self.scale[0]:.5f}, {self.scale[1]:.5f}, '
                f'{self.scale[2]:.5f}), misalignment=({math.degrees(dr):+.3f}, '
                f'{math.degrees(dp):+.3f}, {math.degrees(dy):+.3f}) deg. '
                f'Expect drift linear in distance travelled.')

    @staticmethod
    def _rpy_to_matrix(r, p, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr],
        ]

    def cb(self, m):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'bluerov2/base_link'
        # Apply the unit's fixed misalignment and gain error in the DVL's own
        # frame, then rotate into base_link with the NOMINAL mount rotation,
        # since the frame_id below claims base_link. E folds all three.
        v = np.array([m.velocity.x, m.velocity.y, m.velocity.z])
        vx, vy, vz = self.E @ v
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.linear.z = float(vz)
        # DVL velocity covariance — tune if needed
        msg.twist.covariance[0]  = 0.01   # vx variance
        msg.twist.covariance[7]  = 0.01   # vy variance
        msg.twist.covariance[14] = 0.01   # vz variance
        msg.twist.covariance[21] = 9999.0 # ignore angular
        msg.twist.covariance[28] = 9999.0
        msg.twist.covariance[35] = 9999.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DVLBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
