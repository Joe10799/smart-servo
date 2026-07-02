import re
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


STATE_PATTERN = re.compile(r"servo=([-0-9.]+).*door=([-0-9.]+).*torque=([-0-9.]+)")


def parse_state_line(line: str):
    match = STATE_PATTERN.search(line)
    if not match:
        return None
    return (float(match.group(1)), float(match.group(2)), float(match.group(3)))


class TcpBridge(Node):
    def __init__(self):
        super().__init__("tcp_bridge")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 3333)
        self.declare_parameter("retry_delay_sec", 2.0)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.retry_delay_sec = float(self.get_parameter("retry_delay_sec").value)

        self.sock = None
        self.file = None

        self.cmd_sub = self.create_subscription(String, "/smart_servo/cmd", self.cmd_callback, 10)

        self.state_pub = self.create_publisher(String, "/smart_servo/state", 10)
        self.servo_pub = self.create_publisher(Float32, "/smart_servo/servo_angle", 10)
        self.door_pub = self.create_publisher(Float32, "/smart_servo/door_angle", 10)
        self.torque_pub = self.create_publisher(Float32, "/smart_servo/torque", 10)

        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(f"Waiting for Wokwi ESP32 TCP {self.host}:{self.port}")

    def _connect(self):
        if self.sock is not None and self.file is not None:
            return True

        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=3.0)
            self.file = self.sock.makefile("r")
            self.get_logger().info(f"Connected to Wokwi ESP32 TCP {self.host}:{self.port}")
            return True
        except OSError as exc:
            self.sock = None
            self.file = None
            self.get_logger().warning(
                f"Could not connect to {self.host}:{self.port}: {exc}. Retrying in {self.retry_delay_sec}s"
            )
            return False

    def _close_connection(self):
        if self.file is not None:
            self.file.close()
            self.file = None
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def cmd_callback(self, msg):
        data = msg.data.strip() + "\n"
        if not self._connect():
            return
        try:
            self.sock.sendall(data.encode())
        except OSError as exc:
            self.get_logger().warning(f"Send failed: {exc}")
            self._close_connection()

    def publish_float(self, pub, value):
        m = Float32()
        m.data = float(value)
        pub.publish(m)

    def read_loop(self):
        while rclpy.ok():
            if self.file is None:
                if not self._connect():
                    time.sleep(self.retry_delay_sec)
                    continue

            try:
                line = self.file.readline().strip()
            except OSError as exc:
                self.get_logger().warning(f"Read failed: {exc}")
                self._close_connection()
                continue

            if not line:
                self.get_logger().warning("TCP connection closed; reconnecting")
                self._close_connection()
                continue

            msg = String()
            msg.data = line
            self.state_pub.publish(msg)

            parsed = parse_state_line(line)
            if parsed is not None:
                servo, door, torque = parsed
                self.publish_float(self.servo_pub, servo)
                self.publish_float(self.door_pub, door)
                self.publish_float(self.torque_pub, torque)


def main():
    rclpy.init()
    node = TcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_connection()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
