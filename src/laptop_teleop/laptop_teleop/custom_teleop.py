#!/usr/bin/env python3
import sys
import termios
import tty
import select
import os
import rclpy
from rclpy.node import Node
# CHANGE 1: Use TwistStamped
from geometry_msgs.msg import TwistStamped

msg = """
Control Your Rover!
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

moveBindings = {
    'w': (-1, 0, 0, 0),
    's': (1, 0, 0, 0),
    'a': (0, 0, 0, -1),
    'd': (0, 0, 0, 1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
}

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('custom_teleop')
    
    # CHANGE 2: Publish TwistStamped to the correct topic
    pub = node.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)

    speed = 0.5
    turn = 0.5
    
    try:
        print(msg)
        print("Raw Control Mode Active (No Ramping)")
        tty.setraw(sys.stdin.fileno())

        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                if key in moveBindings:
                    x, y, z, th = moveBindings[key]
                    
                    # CHANGE 3: Create Stamped Message
                    t = TwistStamped()
                    t.header.stamp = node.get_clock().now().to_msg()
                    t.header.frame_id = "base_footprint"  # Valid Frame ID
                    
                    # Store velocity in the .twist sub-field
                    t.twist.linear.x = float(x * speed)
                    t.twist.angular.z = float(th * turn)
                    
                    pub.publish(t)
                    
                elif key in speedBindings:
                    speed = speed * speedBindings[key][0]
                    turn = turn * speedBindings[key][1]
                    print(f"currently:\tspeed {speed}\tturn {turn}\r")
                    
                elif key == 'x' or key == '\x03': # Stop or Ctrl-C
                    # CHANGE 4: Stop with a valid stamped message
                    stop_msg = TwistStamped()
                    stop_msg.header.stamp = node.get_clock().now().to_msg()
                    stop_msg.header.frame_id = "base_footprint"
                    pub.publish(stop_msg)
                    if key == '\x03': break
            else:
                pass

    finally:
        # Final stop
        stop_msg = TwistStamped()
        stop_msg.header.frame_id = "base_footprint"
        pub.publish(stop_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main() 