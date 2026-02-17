#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

msg = """
Control Your Rover! (STANDARD +1 BINDINGS)
---------------------------
Moving around:
        w
   a    s    d

q/z : increase/decrease max speeds by 10%
x   : force stop

w : move forward
s : move backward
a : turn left
d : turn right

CTRL-C to quit
"""

# START WITH STANDARD BINDINGS
# w: x=1 (Forward)
# s: x=-1 (Backward)
# a: th=1 (Turn Left)
# d: th=-1 (Turn Right) 
moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('custom_teleop')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def publish_twist(self, x, th, speed, turn):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_footprint"
        t.twist.linear.x = float(x * speed)
        t.twist.angular.z = float(th * turn)
        self.pub.publish(t)

def main():
    rclpy.init()
    teleop = TeleopNode()
    
    speed = 0.5
    turn = 1.0 # Higher turn speed for better response
    x = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(f"currently:\tspeed {speed}\tturn {turn}")
        
        while True:
            # Check for key press without blocking
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            
            if rlist:
                key = sys.stdin.read(1)
                
                if key in moveBindings:
                    x = moveBindings[key][0]
                    th = moveBindings[key][3]
                    teleop.publish_twist(x, th, speed, turn)
                    
                elif key in speedBindings:
                    speed = speed * speedBindings[key][0]
                    turn = turn * speedBindings[key][1]
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop.settings)
                    # Print status properly (escaping raw mode temporarily)
                    print(f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}")
                    tty.setraw(sys.stdin.fileno())
                    
                elif key == 'x':
                    x = 0
                    th = 0
                    teleop.publish_twist(x, th, speed, turn)
                    
                elif key == '\x03': # CTRL-C
                    break
            else:
                pass

    except Exception as e:
        print(e)
        
    finally:
        # Publish Stop
        teleop.publish_twist(0, 0, speed, turn)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop.settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
