#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import socket
import struct
import json
import time 

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tf_transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np

class Patch(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)

        self.namespace = self.get_namespace()[1:]

        # Subscribers
        self.create_subscription(Imu, "imu", self._imu_callback, 1),
        self.create_subscription(NavSatFix, "gps", self._gps_callback, 1),
        self.create_subscription(Odometry, "odometry", self._odom_callback, 1),

        # Publishers
        self.pub_pwm = self.create_publisher(Float64MultiArray, "setpoint/pwm", 1)

        # Publish everything
        self.timer = self.create_timer(1/50, self.looper)

        if self.namespace=='bluerov2':
            PORT = 9002
            self.SITL_PORT = 9003
        elif self.namespace=='blueboat':
            PORT = 9002
            self.SITL_PORT = 9003

        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # We bind to PORT (9002) to receive PWMs
        self.sock_recv.bind(('', PORT))
        self.sock_recv.settimeout(0.01) # Very short timeout so we don't block
        
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # ephemeral send port
        # Address to send sensor data to
        self.sitl_address = ("127.0.0.1", self.SITL_PORT)

        self.imu = None
        self.gps = None
        self.odom = None

        # self.gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPV4, UDP
        # self.gps_addr = ("127.0.0.1", 25100)

    def _imu_callback(self, msg):
        self.imu = msg

    def _gps_callback(self, msg):
        self.gps = msg

    def _odom_callback(self, msg):
        self.odom = msg

    def looper(self):
        if self.imu is None or self.odom is None:
            self.get_logger().info("Waiting for callbacks", once=False)
            return
        
        self.get_logger().info("Callbacks received", once=True)
        
        # 1. Transform Coordinates (NWU to NED)
        # Position: Flip Y and Z
        pos_n = self.odom.pose.pose.position.x
        pos_e = -self.odom.pose.pose.position.y
        pos_d = -self.odom.pose.pose.position.z
        
        # Velocity: Take only the first 3 elements (Linear) and flip Y/Z
        vel_n = self.odom.twist.twist.linear.x
        vel_e = -self.odom.twist.twist.linear.y
        vel_d = -self.odom.twist.twist.linear.z
        
        # IMU: Renamed to 'accel' and fixed frame
        accel = [
            self.imu.linear_acceleration.x,
            -self.imu.linear_acceleration.y,
            -self.imu.linear_acceleration.z
        ]
        gyro = [
            self.imu.angular_velocity.x,
            -self.imu.angular_velocity.y,
            -self.imu.angular_velocity.z
        ]
        
        c_time = self.get_clock().now().to_msg()
        c_time = c_time.sec + c_time.nanosec/1e9

        # 2. Build ArduSub-Compatible JSON
        JSON_fmt = {
            "timestamp": c_time,
            "imu": {
                "gyro": gyro,
                "accel_body": accel
            },
            "position": [pos_n, pos_e, pos_d],
            "velocity": [vel_n, vel_e, vel_d], "no_lockstep": 1 # Exactly 3 elements
        }
        
        # separators ensures no extra whitespace, keeping the packet small
        JSON_string = "\n" + json.dumps(JSON_fmt, separators=(',', ':')) + "\n"

        # 3. Send to ArduSub (Port 9002)
        self.sock_send.sendto(bytes(JSON_string, "ascii"), self.sitl_address)

        # 4. Receive PWM back from SITL
        try:
            data, address = self.sock_recv.recvfrom(100)
            # ... keep your existing PWM decoding logic here ...
        except Exception:
            return
    
        parse_format = 'HHI16H'
        magic = 18458

        if len(data) != struct.calcsize(parse_format):
            print("got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)))
            return 
        
        decoded = struct.unpack(parse_format,data)

        if magic != decoded[0]:
            print("Incorrect protocol magic %u should be %u" % (decoded[0], magic))
            return 

        frame_rate_hz = decoded[1]
        frame_count = decoded[2]
        pwm = decoded[3:]

        if self.namespace=='bluerov2':
            pwm_thrusters = pwm[0:8]
            pwm_setpoint = [(x-1500)/400 for x in pwm_thrusters]

        if self.namespace=='blueboat':
            # NOTE: OG Blueboat
            pwm_setpoint = np.array([(pwm[0]-1500)/600, (pwm[2]-1500)/600])
            if abs(pwm[0]-1500)<=10:
                pwm_setpoint[0] = 0
            if abs(pwm[2]-1500)<=10:
                pwm_setpoint[1] = 0
                
        # print(pwm_setpoint)
        msg_pwm = Float64MultiArray(data=pwm_setpoint)

        # Publish pwm message
        self.pub_pwm.publish(msg_pwm)

        # print(self.gps.latitude)

        # gps_data = {
        #         'time_usec' : int(c_time/1e3),                        # (uint64_t) Timestamp (micros since boot or Unix epoch)
        #         'gps_id' : 0,                           # (uint8_t) ID of the GPS for multiple GPS inputs
        #         # 'ignore_flags' : 8,                     # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
        #         # 'time_week_ms' : 0,                     # (uint32_t) GPS time (milliseconds from start of GPS week)
        #         # 'time_week' : 0,                        # (uint16_t) GPS week number
        #         # 'fix_type' : 3,                         # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        #         'lat' : int(self.gps.latitude*1e7),                              # (int32_t) Latitude (WGS84), in degrees * 1E7
        #         'lon' : int(self.gps.longitude*1e7),                              # (int32_t) Longitude (WGS84), in degrees * 1E7
        #         'alt' : 0,                              # (float) Altitude (AMSL, not WGS84), in m (positive for up)
        #         # 'hdop' : 1,                             # (float) GPS HDOP horizontal dilution of position in m
        #         # 'vdop' : 1,                             # (float) GPS VDOP vertical dilution of position in m
        #         # 'vn' : 0,                               # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        #         # 've' : 0,                               # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
        #         # 'vd' : 0,                               # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        #         # 'speed_accuracy' : 0,                   # (float) GPS speed accuracy in m/s
        #         # 'horiz_accuracy' : 0,                   # (float) GPS horizontal accuracy in m
        #         # 'vert_accuracy' : 0,                    # (float) GPS vertical accuracy in m
        #         # 'satellites_visible' : 7                # (uint8_t) Number of satellites visible.
        # }

        # gps_data = json.dumps(gps_data)
        # self.gps_sock.sendto(gps_data.encode(), ("127.0.0.1", 25100))

def main(args=None):
    rclpy.init(args=args)

    # patch = Patch(node_name="ardusim_patch")
    patch = Patch(node_name="ardusim_patch", namespace='bluerov2')
    
    rclpy.spin(patch)

    # Destroy the node explicitly, otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    patch.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
