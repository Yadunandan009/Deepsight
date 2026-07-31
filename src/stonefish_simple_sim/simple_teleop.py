import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = """
Control Your Stonefish Sub!
---------------------------
Moving around:        Vertical:
    w                     i (Up)
a   s   d                 k (Down)
    x

w/x : Increase/Decrease Surge (Forward/Back)
a/d : Increase/Decrease Sway (Side-to-side)
i/k : Increase/Decrease Heave (Up/Down)

SPACE : STOP ALL
CTRL-C to quit
"""

class TeleopSub(Node):
    def __init__(self):
        super().__init__('teleop_sub')
        
        # Publishers
        self.pub_surge = self.create_publisher(Float64, '/sub/thruster/surge', 10)
        self.pub_heave_b = self.create_publisher(Float64, '/sub/thruster/heave_bow', 10)
        self.pub_heave_s = self.create_publisher(Float64, '/sub/thruster/heave_stern', 10)
        self.pub_sway = self.create_publisher(Float64, '/sub/thruster/sway_bow', 10)

        self.settings = termios.tcgetattr(sys.stdin)
        self.power = 10.0 # Force increment

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                f = Float64()
                
                if key == 'w':
                    f.data = self.power
                    self.pub_surge.publish(f)
                elif key == 'x':
                    f.data = -self.power
                    self.pub_surge.publish(f)
                elif key == 'a':
                    f.data = self.power
                    self.pub_sway.publish(f)
                elif key == 'd':
                    f.data = -self.power
                    self.pub_sway.publish(f)
                elif key == 'i':
                    f.data = self.power
                    self.pub_heave_b.publish(f)
                    self.pub_heave_s.publish(f)
                elif key == 'k':
                    f.data = -self.power
                    self.pub_heave_b.publish(f)
                    self.pub_heave_s.publish(f)
                elif key == ' ':
                    f.data = 0.0
                    self.pub_surge.publish(f)
                    self.pub_sway.publish(f)
                    self.pub_heave_b.publish(f)
                    self.pub_heave_s.publish(f)
                elif key == '\x03': # CTRL-C
                    break
        except Exception as e:
            print(e)
        finally:
            # Safety stop
            f.data = 0.0
            self.pub_surge.publish(f)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = TeleopSub()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
