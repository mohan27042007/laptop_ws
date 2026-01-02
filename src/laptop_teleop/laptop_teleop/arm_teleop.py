#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

msg = """
Arm Control Keys (360 BUCKET TEMP MODE):
---------------------------------------
u / j = joint up / down
i     = bucket UP (pulse)
k     = bucket DOWN (pulse)
x     = STOP bucket + reset joint

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

        self.joint = 20   # match Arduino JOINT_MIN
        self.bucket = 0  # 0 = STOP

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

            if key == 'u':
                node.joint += 5
                node.bucket = 0

            elif key == 'j':
                node.joint -= 5
                node.bucket = 0

            elif key == 'i':
                node.bucket = 1  # UP

            elif key == 'k':
                node.bucket = 2  # DOWN

            elif key == 'x':
                node.bucket = 0
                node.joint = 20

            else:
                if key == '\x03':
                    break
                continue

            node.joint = max(20, min(150, node.joint))
            node.send()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
