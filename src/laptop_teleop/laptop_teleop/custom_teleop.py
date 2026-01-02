#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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
a : SPIN left (In-Place)
d : SPIN right (In-Place)


CTRL-C to quit
"""

# For a non-holonomic skid-steer/differential drive, only linear.x and angular.z are used.
# w/s: forward/backward, a/d: pure spin in place.
moveBindings = {
    'w': (1, 0, 0, 0),   # forward
    's': (-1, 0, 0, 0),  # backward
    'a': (0, 0, 0, 1),   # spin left
    'd': (0, 0, 0, -1),  # spin right
    'x': (0, 0, 0, 0),   # stop
}

speedBindings = {
    'q': (1.1, 1.1),     # increase both linear and angular limits by 10%
    'z': (0.9, 0.9),     # decrease both by 10%
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('custom_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel_teleop', 10)

    speed = 0.5      # max linear speed (m/s)
    turn = 2.0       # max angular speed (rad/s)
    x = 0
    y = 0
    z = 0
    th = 0

    try:
        print(msg)
        print(f"currently:\tspeed {speed}\tturn {turn}")
        while True:
            key = getKey(settings)

            if key in moveBindings.keys():
                x, y, z, th = moveBindings[key]
                if key == 'x':
                    # explicit emergency stop
                    speed = 0.0
                    turn = 0.0
                    print(f"EMERGENCY STOP: speed {speed}\tturn {turn}")

            elif key == 'q':
                speed = min(speed * speedBindings['q'][0], 1.5)
                turn  = min(turn  * speedBindings['q'][1], 3.0)
                print(f"currently:\tspeed {speed}\tturn {turn}")

            elif key == 'z':
                speed = max(speed * speedBindings['z'][0], 0.0)
                turn  = max(turn  * speedBindings['z'][1], 0.0)
                print(f"currently:\tspeed {speed}\tturn {turn}")

            else:
                # any other key: stop motion
                x = 0
                y = 0
                z = 0
                th = 0
                if key == '\x03':  # CTRL-C
                    break

            twist = Twist()
            twist.linear.x  = x * speed
            twist.linear.y  = 0.0
            twist.linear.z  = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x  = 0.0
        twist.linear.y  = 0.0
        twist.linear.z  = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
