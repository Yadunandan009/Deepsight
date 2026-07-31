#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import tty
import termios
import select
import time

msg = """
BlueROV2 Automated Movement for SLAM!
---------------------------
The robot will now move automatically to help ORB-SLAM3 initialize.
Sequence:
1. Surge Forward (5s)
2. Stop (2s)
3. Heave Up (3s)
4. Heave Down (3s)
5. Repeat

CTRL-C to stop
"""

class AutoMoveNode(Node):
    def __init__(self):
        super().__init__('bluerov2_auto_move')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/bluerov2/setpoint/pwm', 10)
        self.get_logger().info("Auto Move Node Started.")

    def run(self):
        try:
            print(msg)
            start_time = time.time()
            
            while rclpy.ok():
                elapsed = (time.time() - start_time) % 15.0 # 15 second loop
                
                surge = 0.0
                sway = 0.0
                heave = 0.0
                yaw = 0.0
                
                if elapsed < 5.0:
                    surge = 6  # Surge forward
                    print(f"\rStatus: Surging Forward... [{elapsed:.1f}/5.0s]", end="", flush=True)
                elif elapsed < 7.0:
                    surge = 0.0  # Pause
                    print(f"\rStatus: Pausing...         [{elapsed-5:.1f}/2.0s]", end="", flush=True)
                elif elapsed < 10.0:
                    heave = 4  # Heave up
                    print(f"\rStatus: Heaving Up...      [{elapsed-7:.1f}/3.0s]", end="", flush=True)
                elif elapsed < 13.0:
                    heave = -0.4 # Heave down
                    print(f"\rStatus: Heaving Down...    [{elapsed-10:.1f}/3.0s]", end="", flush=True)
                else:
                    print(f"\rStatus: Resetting loop...  ", end="", flush=True)

                pwm = [0.0] * 8
                # Surge
                pwm[0] += surge; pwm[1] += surge; pwm[2] -= surge; pwm[3] -= surge;
                # Heave
                pwm[4] += heave; pwm[5] += heave; pwm[6] += heave; pwm[7] += heave;

                # Clip to [-1, 1]
                pwm = [max(min(x, 1.0), -1.0) for x in pwm]
                
                msg_pwm = Float64MultiArray(data=pwm)
                if rclpy.ok():
                    self.publisher_.publish(msg_pwm)
                time.sleep(0.1) # 10Hz control loop
                
        except KeyboardInterrupt:
            pass
        finally:
            # Only publish if context is still valid
            if rclpy.ok():
                try:
                    self.publisher_.publish(Float64MultiArray(data=[0.0]*8))
                except Exception:
                    pass

def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutoMoveNode()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
