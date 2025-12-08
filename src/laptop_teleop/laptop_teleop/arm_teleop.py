#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

msg = """
Arm Control Keys:
--------------------------
u / j = joint up / down
i / k = bucket up / down

x = reset (joint=0, bucket=0)
CTRL+C to quit
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.pub = self.create_publisher(String, '/arm_command', 10)

        self.joint = 90
        self.bucket = 90

    def send(self):
        msg = String()
        msg.data = f"A{self.joint}_{self.bucket}"
        self.pub.publish(msg)
        self.get_logger().info(f"Sent {msg.data}")

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()

    node = ArmTeleop()
    print(msg)

    try:
        while True:
            key = getKey(settings)

            if key == 'u': node.joint += 5
            elif key == 'j': node.joint -= 5

            elif key == 'i': node.bucket += 5
            elif key == 'k': node.bucket -= 5

            elif key == 'x':
                node.joint = 0
                node.bucket = 0

            else:
                if key == '\x03':
                    break

            # Constrain values
            node.joint = max(0, min(180, node.joint))
            node.bucket = max(0, min(180, node.bucket))

            node.send()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
