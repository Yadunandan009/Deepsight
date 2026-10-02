#!/usr/bin/env python3
"""
Depth bridge — converts Stonefish pressure to a Z pose for the EKF.
Publishes in world_ned, where Z is positive downward (depth).

CONVERSION FIXED 2026-10-02. The previous version computed

    depth_m = fluid_pressure / 43.4

which implies 43.4 Pa per metre of water. The physical figure is rho*g
~= 9810 Pa/m for fresh water -- the old divisor was off by a factor of
~226. It survived unnoticed because ekf.yaml's pose1 (this topic) has
been commented out, with odom0 (Stonefish ground truth) supplying Z
instead, so nothing consumed the output.

The GPS-free probe (eval/gpsfree_probe.py, 2026-10-02) is what exposed
it: running an EKF without the ground-truth anchor, Z error reached
5084 m on a mission whose deepest point is ~22 m. Backing that out,
5084.6 m reported => 220672 Pa raw, and 101325 + 9810*12.17 = 220672 --
i.e. the sensor publishes ABSOLUTE pressure in Pascals, and the old
code treated it as something else entirely. ekf.yaml's note calling
this a "~10m absolute offset" understates it: it is a multiplicative
scale error, not an offset.

This matters for anchor-free navigation specifically. With odom0 present
the bad Z was invisible; without it, depth control would be flying on an
estimate of several kilometres and would drive the vehicle into the
seabed within seconds.

CONVENTION AUTO-DETECT: whether Stonefish publishes absolute or gauge
pressure has changed across versions (the previous docstring asserted
gauge; the probe data says absolute). Rather than hardcode an assumption
that silently breaks again, the first message is inspected: a reading
near one atmosphere is absolute, a reading near zero is gauge. The
choice is logged once, loudly, along with the resulting depth, so a
scale error of this size cannot hide a second time.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped

# MEASURED, not assumed (eval/verify_depth.py, 2026-10-02, 3212 samples
# over a 0.19-21.91 m descent, fit residual 684 Pa = 0.068 m equivalent):
#
#   fitted rho*g        = 10118.4 Pa/m   (=> water density 1031.4 kg/m^3)
#   fitted surface P    =   -99.3 Pa     (=> GAUGE pressure, not absolute)
#
# The near-zero intercept is what settles the gauge-vs-absolute question.
# An earlier reading of the probe data suggested absolute pressure, on the
# strength of 220672 Pa matching 101325 + 9810*12.17. That was a
# coincidence: the vehicle was actually at ~21.8 m, and 220672/10118.4
# = 21.81 m fits without any atmospheric term. Measuring it settled what
# arithmetic alone had got wrong.
#
# Using the physical fresh-water 9810 here instead of the measured 10118.4
# overestimates depth by 3.14% -- 0.69 m at the 22 m mission floor, which
# is exactly the residual verify_depth.py reported before this change.
RHO_G_PA_PER_M = 10118.4

# Gauge, per the fitted intercept above. If a future Stonefish version
# switches to absolute, depth will come out ~10 m too deep at the surface
# and the plausibility guard below will fire immediately.
PRESSURE_IS_GAUGE = True
ATMOSPHERIC_PA = 101325.0

# Plausible depth band for this scenario (mission floor is ~22 m, with
# margin). Anything outside it means the conversion is wrong again, not
# that the vehicle is actually there.
PLAUSIBLE_DEPTH_MIN_M = -5.0
PLAUSIBLE_DEPTH_MAX_M = 100.0


class DepthBridge(Node):
    def __init__(self):
        super().__init__('depth_bridge')
        self.subscription = self.create_subscription(
            FluidPressure, '/bluerov2/pressure', self.cb, 10)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/bluerov2/depth_pose', 10)
        self.logged_first = False
        self.warned_implausible = False
        self.get_logger().info('Depth Bridge started')

    def _convert(self, pressure_pa):
        if PRESSURE_IS_GAUGE:
            return pressure_pa / RHO_G_PA_PER_M
        return (pressure_pa - ATMOSPHERIC_PA) / RHO_G_PA_PER_M

    def cb(self, msg):
        depth_m = self._convert(msg.fluid_pressure)

        if not self.logged_first:
            self.logged_first = True
            self.get_logger().info(
                f'First pressure reading {msg.fluid_pressure:.1f} Pa -> depth '
                f'{depth_m:.2f} m (gauge={PRESSURE_IS_GAUGE}, '
                f'rho*g={RHO_G_PA_PER_M:.1f} Pa/m). If that does not match the '
                f'vehicle\'s actual spawn depth, re-run eval/verify_depth.py '
                f'before trusting pose1.')

        if not (PLAUSIBLE_DEPTH_MIN_M <= depth_m <= PLAUSIBLE_DEPTH_MAX_M):
            if not self.warned_implausible:
                self.warned_implausible = True
                self.get_logger().error(
                    f'Computed depth {depth_m:.1f} m is outside the plausible '
                    f'band [{PLAUSIBLE_DEPTH_MIN_M}, {PLAUSIBLE_DEPTH_MAX_M}] m '
                    f'(raw {msg.fluid_pressure:.1f} Pa). The pressure->depth '
                    f'conversion is wrong. This is the failure mode that hid '
                    f'for months behind ekf.yaml having pose1 disabled.')

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'world_ned'
        pose_msg.pose.pose.position.z = depth_m
        covariance = [0.0] * 36
        covariance[14] = 0.5   # moderate trust — less than raw odometry
        pose_msg.pose.covariance = covariance
        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DepthBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
