import sys
import termios
import tty
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


def key_to_cmd(key):
    mapping = {
        "LEFT": "L",
        "RIGHT": "R",
        "UP": "S",
        "DOWN": "S",
        "a": "L",
        "d": "R",
        "s": "S",
        "+": "+",
        "-": "-",
        "c": "CAL",
        "r": "RESET",
        "q": "QUIT",
    }
    return mapping.get(key, None)


class KeyboardNode(Node):
    def __init__(self):
        super().__init__("keyboard_node")
        self.device_pub = self.create_publisher(
            String, "/smart_servo/control", 10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            "/servo_velocity_controller/commands",
            10,
        )
        self.speed_dps = 40.0
        self.direction = 0
        self.get_logger().info(
            "Keyboard: arrows=move, +/-=speed, c=calibrate, r=reset, q=quit"
        )

    def publish_device_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.device_pub.publish(msg)
        self.get_logger().info(f"sent: {cmd}")

    def publish_velocity(self):
        msg = Float64MultiArray()
        msg.data = [math.radians(self.direction * self.speed_dps)]
        self.velocity_pub.publish(msg)
        self.get_logger().info(
            f"velocity: {self.direction * self.speed_dps:.0f} deg/s"
        )

    def handle_command(self, cmd):
        if cmd == "L":
            self.direction = -1
            self.publish_velocity()
        elif cmd == "R":
            self.direction = 1
            self.publish_velocity()
        elif cmd == "S":
            self.direction = 0
            self.publish_velocity()
        elif cmd == "+":
            self.speed_dps = min(self.speed_dps + 10.0, 180.0)
            self.publish_velocity()
        elif cmd == "-":
            self.speed_dps = max(self.speed_dps - 10.0, 10.0)
            self.publish_velocity()
        elif cmd in ("CAL", "RESET"):
            self.direction = 0
            self.publish_velocity()
            self.publish_device_cmd(cmd)


def get_key():
    tty_file = None
    try:
        if sys.stdin.isatty():
            tty_file = sys.stdin
        else:
            tty_file = open("/dev/tty", "r")
    except OSError:
        return None

    fd = tty_file.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        first = tty_file.read(1)

        if first == "\x1b":
            seq = tty_file.read(2)
            if seq == "[D":
                return "LEFT"
            if seq == "[C":
                return "RIGHT"
            if seq == "[A":
                return "UP"
            if seq == "[B":
                return "DOWN"
            return first + seq

        return first
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if tty_file is not sys.stdin:
            tty_file.close()


def main():
    rclpy.init()
    node = KeyboardNode()

    try:
        while rclpy.ok():
            key = get_key()
            cmd = key_to_cmd(key)

            if cmd is None:
                rclpy.spin_once(node, timeout_sec=0.01)
                continue

            if cmd == "QUIT":
                break

            node.handle_command(cmd)
            rclpy.spin_once(node, timeout_sec=0.01)

    finally:
        if rclpy.ok():
            node.direction = 0
            node.publish_velocity()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
