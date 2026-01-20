#!/usr/bin/env python3
import sys
import termios
import tty
import select
import os
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
a : turn left
d : turn right

CTRL-C to quit
"""

moveBindings = {
    'w': (-1, 0, 0, 0),
    's': (1, 0, 0, 0),
    'a': (0, 0, 0, -1),
    'd': (0, 0, 0, 1),
    'x': (0, 0, 0, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (1.5, 1.5),
}

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('custom_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    speed = 0.5
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    # Ramp variables
    target_linear_x = 0.0
    target_linear_y = 0.0
    target_linear_z = 0.0
    target_angular_z = 0.0
    
    control_linear_x = 0.0
    control_linear_y = 0.0
    control_linear_z = 0.0
    control_angular_z = 0.0
    
    RAMP_STEP = 0.5  # 50% step increase per key press (Faster ramp)

    try:
        print(msg)
        print(f"currently:\tspeed {speed}\tturn {turn}")
        
        # Set raw mode once
        tty.setraw(sys.stdin.fileno())

        while True:
            # Wait for input with a timeout (non-blocking check)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            
            key_data = ""
            if rlist:
                # Read a batch of keys
                try:
                    key_data = os.read(sys.stdin.fileno(), 1024).decode('utf-8')
                except OSError:
                    pass

            if not key_data:
                # No input, just continue loop to publish current state (or decay?)
                # For now, we just maintain current control state unless changed
                pass
            
            # Process the batch of keys
            if 'x' in key_data:
                # Force stop takes priority
                control_linear_x = 0.0
                control_linear_y = 0.0
                control_linear_z = 0.0
                control_angular_z = 0.0
                x = 0; y = 0; z = 0; th = 0
                # No need to flush manually since we read everything with os.read
            elif '\x03' in key_data: # CTRL-C
                break
            else:
                # Process other keys
                for key in key_data:
                    if key in moveBindings.keys():
                        x = moveBindings[key][0]
                        y = moveBindings[key][1]
                        z = moveBindings[key][2]
                        th = moveBindings[key][3]
                        
                        # Apply ramp for each key press found in batch
                        # Calculate target speeds based on current key
                        target_linear_x = x * speed
                        target_linear_y = y * speed
                        target_linear_z = z * speed
                        target_angular_z = th * turn

                        # Helper to ramp a single value
                        def ramp_value(current, target, step):
                            if current < target:
                                return min(current + step, target)
                            elif current > target:
                                return max(current - step, target)
                            else:
                                return current

                        step_lin = speed * RAMP_STEP
                        step_ang = turn * RAMP_STEP
                        
                        control_linear_x = ramp_value(control_linear_x, target_linear_x, step_lin)
                        control_linear_y = ramp_value(control_linear_y, target_linear_y, step_lin)
                        control_linear_z = ramp_value(control_linear_z, target_linear_z, step_lin)
                        control_angular_z = ramp_value(control_angular_z, target_angular_z, step_ang)

                    elif key in speedBindings.keys():
                        speed = speed * speedBindings[key][0]
                        turn = turn * speedBindings[key][1]
                        print(f"\rcurrently:\tspeed {speed}\tturn {turn}") # \r for raw mode
                        if (status == 14):
                            print(msg.replace('\n', '\r\n'))
                        status = (status + 1) % 15

            twist = Twist()
            twist.linear.x = control_linear_x
            twist.linear.y = control_linear_y
            twist.linear.z = control_linear_z
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_z
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()