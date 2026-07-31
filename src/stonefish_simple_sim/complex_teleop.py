import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
import time

class PIDHoverSub(Node):
    def __init__(self):
        super().__init__('pid_hover_node')
        
        # Publishers & Subscribers
        self.pub_heave_b = self.create_publisher(Float64, '/sub/thruster/heave_bow', 10)
        self.pub_heave_s = self.create_publisher(Float64, '/sub/thruster/heave_stern', 10)
        self.sub_pressure = self.create_subscription(FluidPressure, '/sub/pressure', self.pressure_callback, 10)

        # PID Constants (These may need tuning for Stonefish)
        self.Kp = 40.0   # Proportional gain
        self.Ki = 0.5    # Integral gain
        self.Kd = 20.0   # Derivative gain

        # PID Variables
        self.target_depth = 2.0  # Meters
        self.current_depth = 0.0
        self.error_prior = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.surface_pressure = 101325.0

    def pressure_callback(self, msg):
        # Convert pressure to depth
        self.current_depth = (msg.fluid_pressure - self.surface_pressure) / 9806.65

    def compute_pid(self):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0: return 0.0

        # Calculate error
        error = self.target_depth - self.current_depth
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term (the "brake")
        derivative = (error - self.error_prior) / dt
        D = self.Kd * derivative

        # Total output
        output = P + I + D

        # Save state for next loop
        self.error_prior = error
        self.last_time = now

        # Clamp output to motor limits (e.g., -50 to 50)
        return max(min(output, 50.0), -50.0)

    def run(self):
        self.get_logger().info(f'Starting PID Hover at {self.target_depth}m...')
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                
                # Calculate required thrust (Note: In Stonefish, 
                # you might need to flip the sign of thrust 
                # depending on your thruster setup)
                thrust = -self.compute_pid() 
                
                msg = Float64()
                msg.data = thrust
                self.pub_heave_b.publish(msg)
                self.pub_heave_s.publish(msg)

                self.get_logger().info(
                    f'Depth: {self.current_depth:.2f}m | Thrust: {thrust:.2f}', 
                    throttle_duration_sec=0.5
                )
        except KeyboardInterrupt:
            # Stop motors on exit
            msg.data = 0.0
            self.pub_heave_b.publish(msg)
            self.pub_heave_s.publish(msg)

def main():
    rclpy.init()
    node = PIDHoverSub()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
