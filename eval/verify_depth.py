#!/usr/bin/env python3
"""
Empirically derive Stonefish's pressure->depth relationship, instead of
assuming it.

depth_bridge.py was found on 2026-10-02 to be using a divisor (43.4 Pa/m)
that is off from the physical rho*g (~9810 Pa/m) by a factor of ~226. The
replacement uses the physical constants -- but two of them were chosen by
reasoning, not measurement:
  - ATMOSPHERIC_PA = 101325 (is the sim's sensor absolute or gauge?)
  - RHO_G_PA_PER_M = 9810   (fresh water; seawater would be ~10055, a 2.5%
                             difference, i.e. ~0.55 m at the 22 m mission floor)

This script removes the guesswork. It collects (raw pressure, ground-truth
depth) pairs while the vehicle moves through its depth range and least-
squares fits

    pressure = slope * depth + intercept

giving slope = the sim's actual rho*g and intercept = the sim's actual
surface pressure. Those are the numbers that belong in depth_bridge.py.

Run it alongside a mission, ideally one that descends (DESCEND takes the
vehicle to ~22 m, which gives a wide enough baseline for a good fit --
a fit over a narrow depth range is ill-conditioned and will report a
large residual).

Usage:  python3 eval/verify_depth.py [--seconds 120]
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import FluidPressure
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# Below this spread the fit is not trustworthy -- the vehicle has not
# moved through enough depth to separate slope from intercept.
MIN_DEPTH_SPREAD_M = 3.0


class VerifyDepth(Node):
    def __init__(self, seconds):
        super().__init__('verify_depth')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pressure = None
        self.truth_z = None
        self.bridge_z = None
        self.samples = []       # (pressure_pa, truth_depth_m, bridge_depth_m)
        self.seconds = seconds
        self.elapsed = 0.0

        self.create_subscription(FluidPressure, '/bluerov2/pressure',
                                 lambda m: setattr(self, 'pressure', m.fluid_pressure), qos)
        self.create_subscription(Odometry, '/bluerov2/odometry',
                                 lambda m: setattr(self, 'truth_z', m.pose.pose.position.z), qos)
        self.create_subscription(PoseWithCovarianceStamped, '/bluerov2/depth_pose',
                                 lambda m: setattr(self, 'bridge_z', m.pose.pose.position.z), qos)
        self.create_timer(0.2, self._sample)
        self.get_logger().info('Collecting pressure/depth pairs — move the vehicle through its depth range.')

    def _sample(self):
        self.elapsed += 0.2
        if self.pressure is None or self.truth_z is None:
            return
        self.samples.append((self.pressure, self.truth_z,
                             self.bridge_z if self.bridge_z is not None else float('nan')))
        if len(self.samples) % 50 == 0:
            d = [s[1] for s in self.samples]
            self.get_logger().info(
                f'{len(self.samples)} samples, depth range {min(d):.2f} to {max(d):.2f} m')
        if self.seconds and self.elapsed > self.seconds:
            raise SystemExit(0)

    def report(self):
        print('\n' + '=' * 70)
        print('PRESSURE -> DEPTH VERIFICATION')
        print('=' * 70)
        if len(self.samples) < 20:
            print(f'  Only {len(self.samples)} samples — is the sim running and')
            print('  publishing /bluerov2/pressure and /bluerov2/odometry?')
            print('=' * 70)
            return

        P = np.array([s[0] for s in self.samples])
        Z = np.array([s[1] for s in self.samples])
        B = np.array([s[2] for s in self.samples])

        spread = Z.max() - Z.min()
        print(f'  samples      : {len(self.samples)}')
        print(f'  depth range  : {Z.min():.2f} to {Z.max():.2f} m  (spread {spread:.2f} m)')
        print(f'  pressure rng : {P.min():.1f} to {P.max():.1f} Pa')
        print('-' * 70)

        if spread < MIN_DEPTH_SPREAD_M:
            print(f'  INCONCLUSIVE — depth spread under {MIN_DEPTH_SPREAD_M} m.')
            print('  Re-run during DESCEND so the vehicle covers a real depth range;')
            print('  fitting a slope over a near-constant depth is ill-conditioned.')
            print('=' * 70)
            return

        slope, intercept = np.polyfit(Z, P, 1)
        pred = slope * Z + intercept
        resid = float(np.sqrt(np.mean((P - pred) ** 2)))

        print(f'  FITTED    rho*g      = {slope:9.1f} Pa/m')
        print(f'  FITTED    surface P  = {intercept:9.1f} Pa')
        print(f'  fit residual (RMS)   = {resid:9.2f} Pa  ({resid/slope:.4f} m equivalent)')
        print('-' * 70)
        print(f'  for reference: fresh water 9810 Pa/m, seawater ~10055 Pa/m,')
        print(f'                 one atmosphere 101325 Pa')
        print('-' * 70)

        if np.isfinite(B).any():
            err = np.abs(B[np.isfinite(B)] - Z[np.isfinite(B)])
            print(f'  depth_bridge output vs truth: mean err {err.mean():.3f} m, max {err.max():.3f} m')
            if err.max() < 0.5:
                print('  => depth_bridge.py is CORRECT. pose1 is safe to enable.')
            else:
                print('  => depth_bridge.py is STILL WRONG. Put the fitted constants above')
                print('     into ATMOSPHERIC_PA / RHO_G_PA_PER_M and re-run this.')
        else:
            print('  (no /bluerov2/depth_pose seen — is depth_bridge running?)')
        print('=' * 70)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=0.0,
                   help='stop after N seconds (0 = until Ctrl-C)')
    args = p.parse_args()
    rclpy.init()
    node = VerifyDepth(args.seconds)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.report()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
