#!/usr/bin/env python3
"""
ArduSub → Stonefish PWM Bridge Node
=====================================
Subscribes to MAVROS RC output (/mavros/rc/out)
Converts PWM (1100-1900) → normalized thrust (-1.0 to 1.0)
Publishes to Stonefish thruster topics for robot 'sub'

Thruster Mapping (SERVO channel → topic):
  SERVO1 → sub/thruster/surge
  SERVO2 → sub/thruster/sway_bow
  SERVO3 → sub/thruster/heave_stern
  SERVO4 → sub/thruster/heave_bow

Edit THRUSTER_MAP below to change the servo → topic assignment.
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from std_msgs.msg import Float64

# ─────────────────────────────────────────────
# CONFIGURATION — edit this to match your setup
# ─────────────────────────────────────────────
ROBOT_NAME = "sub"

# Maps SERVO channel index (0-based) → Stonefish topic suffix
# Index 0 = SERVO1, Index 1 = SERVO2, etc.
THRUSTER_MAP = {
    0: "thruster/surge",        # SERVO1 → surge
    1: "thruster/sway_bow",     # SERVO2 → sway_bow
    2: "thruster/heave_stern",  # SERVO3 → heave_stern
    3: "thruster/heave_bow",    # SERVO4 → heave_bow
}

# PWM range from ArduSub
PWM_MIN    = 1100.0
PWM_MAX    = 1900.0
PWM_NEUTRAL = 1500.0

# Dead zone: PWM values within ±DEADZONE of neutral → 0.0 output
DEADZONE_PWM = 25.0
# ─────────────────────────────────────────────


def pwm_to_thrust(pwm: float) -> float:
    """
    Convert a PWM value (1100–1900) to normalized thrust (-1.0 to 1.0).
    1500 (neutral) → 0.0
    1900 (full fwd) → +1.0
    1100 (full rev) → -1.0
    Values within DEADZONE of neutral are set to 0.0
    """
    pwm = float(max(PWM_MIN, min(PWM_MAX, pwm)))  # clamp

    # Apply dead zone
    if abs(pwm - PWM_NEUTRAL) < DEADZONE_PWM:
        return 0.0

    if pwm >= PWM_NEUTRAL:
        thrust = (pwm - PWM_NEUTRAL) / (PWM_MAX - PWM_NEUTRAL)
    else:
        thrust = (pwm - PWM_NEUTRAL) / (PWM_NEUTRAL - PWM_MIN)

    return float(max(-1.0, min(1.0, thrust)))  # clamp output


class ArduSubBridge(Node):

    def __init__(self):
        super().__init__('ardusub_stonefish_bridge')

        self.get_logger().info("ArduSub → Stonefish Bridge starting...")
        self.get_logger().info(f"Robot name : {ROBOT_NAME}")
        self.get_logger().info(f"PWM range  : {PWM_MIN} – {PWM_MAX} (neutral {PWM_NEUTRAL})")
        self.get_logger().info(f"Dead zone  : ±{DEADZONE_PWM} PWM around neutral")

        # Create one publisher per thruster
        self.publishers_ = {}
        for servo_idx, topic_suffix in THRUSTER_MAP.items():
            full_topic = f"{ROBOT_NAME}/{topic_suffix}"
            pub = self.create_publisher(Float64, full_topic, 10)
            self.publishers_[servo_idx] = pub
            self.get_logger().info(f"  SERVO{servo_idx+1} → {full_topic}")

        # Subscribe to MAVROS RC output
        self.subscription = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.rc_out_callback,
            10
        )
        self.get_logger().info("Subscribed to /mavros/rc/out — waiting for ArduSub...")

    def rc_out_callback(self, msg: RCOut):
        """Called every time MAVROS publishes RC output (servo PWM values)."""
        channels = msg.channels  # list of PWM values, index 0 = SERVO1

        for servo_idx, pub in self.publishers_.items():
            if servo_idx >= len(channels):
                self.get_logger().warn(
                    f"SERVO{servo_idx+1} not in RC output (only {len(channels)} channels)",
                    throttle_duration_sec=5.0
                )
                continue

            pwm = channels[servo_idx]

            # ArduSub outputs 0 when a channel is unused/disarmed
            if pwm == 0:
                thrust = 0.0
            else:
                thrust = pwm_to_thrust(pwm)

            out_msg = Float64()
            out_msg.data = thrust
            pub.publish(out_msg)

        # Debug log — throttled to once per second
        if len(channels) >= 4:
            self.get_logger().debug(
                f"PWM [{channels[0]}, {channels[1]}, {channels[2]}, {channels[3]}] → "
                f"thrust [{pwm_to_thrust(channels[0]):.2f}, "
                f"{pwm_to_thrust(channels[1]):.2f}, "
                f"{pwm_to_thrust(channels[2]):.2f}, "
                f"{pwm_to_thrust(channels[3]):.2f}]",
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArduSubBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Bridge node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
