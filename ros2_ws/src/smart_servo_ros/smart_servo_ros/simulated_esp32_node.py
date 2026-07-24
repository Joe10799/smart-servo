import os
import random
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from smart_servo_ros.logging_utils import write_row


class SimulatedESP32(Node):
    def __init__(self):
        super().__init__("simulated_esp32_node")

        self.gear_ratio = 15.0 / 40.0
        self.torque_threshold = 80.0

        self.servo_angle = 0.0
        self.door_angle = 0.0
        self.speed = 40.0
        self.direction = 0

        self.closed_real = random.uniform(-30.0, 0.0)
        self.open_real = self.closed_real + random.uniform(40.0, 60.0)

        self.closed_detected = 0.0
        self.open_detected = 0.0

        self.calibrated = False
        self.detecting = False
        self.detect_armed = False
        self.detect_stage = 0

        self.cmd_sub = self.create_subscription(
            String,
            "/smart_servo/cmd",
            self.cmd_callback,
            10,
        )

        self.state_pub = self.create_publisher(String, "/smart_servo/state", 10)
        self.servo_pub = self.create_publisher(Float32, "/smart_servo/servo_angle", 10)
        self.door_pub = self.create_publisher(Float32, "/smart_servo/door_angle", 10)
        self.torque_pub = self.create_publisher(Float32, "/smart_servo/torque", 10)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update)

        self.log_dir = os.path.join(os.getcwd(), "data")
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, "latest.csv")
        if os.path.exists(self.log_path):
            os.remove(self.log_path)

        self.get_logger().info("Pure ROS ESP32 simulation started")
        self.get_logger().info(f"Logging to {self.log_path}")
        self.get_logger().info(
            f"Hidden real endpoints: closed={self.closed_real:.2f}, open={self.open_real:.2f}"
        )

    def fake_torque(self):
        angle = self.door_angle

        if angle <= self.closed_real:
            return 100.0 + abs(angle - self.closed_real)

        if angle >= self.open_real:
            return 100.0 + abs(angle - self.open_real)

        d = min(abs(angle - self.closed_real), abs(self.open_real - angle))

        if d < 8.0:
            return 80.0 - d * 5.0

        return 10.0

    def cmd_callback(self, msg):
        cmd = msg.data.strip()

        if cmd == "L":
            self.direction = -1
        elif cmd == "R":
            self.direction = 1
        elif cmd == "S":
            self.direction = 0
        elif cmd == "+":
            self.speed = min(self.speed + 10.0, 180.0)
        elif cmd == "-":
            self.speed = max(self.speed - 10.0, 10.0)
        elif cmd.startswith("SPD "):
            try:
                self.speed = min(max(float(cmd[4:]), 10.0), 180.0)
            except ValueError:
                self.get_logger().warning(f"Invalid speed command: {cmd}")
        elif cmd == "CAL":
            self.detecting = True
            self.detect_armed = False
            self.calibrated = False
            self.detect_stage = 1
            self.direction = -1
            self.speed = 20.0
            self.get_logger().info("Calibration started")
        elif cmd == "RESET":
            self.reset()

    def reset(self):
        self.servo_angle = 0.0
        self.door_angle = 0.0
        self.direction = 0
        self.speed = 40.0
        self.closed_real = random.uniform(-30.0, 0.0)
        self.open_real = self.closed_real + random.uniform(40.0, 60.0)
        self.closed_detected = 0.0
        self.open_detected = 0.0
        self.calibrated = False
        self.detecting = False
        self.detect_armed = False
        self.detect_stage = 0
        self.get_logger().info(
            f"Reset. New hidden endpoints: closed={self.closed_real:.2f}, open={self.open_real:.2f}"
        )

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        next_servo = self.servo_angle + self.direction * self.speed * dt
        next_door = next_servo * self.gear_ratio

        if self.calibrated and not self.detecting:
            if next_door < self.closed_detected:
                next_door = self.closed_detected
                next_servo = next_door / self.gear_ratio
                self.direction = 0

            if next_door > self.open_detected:
                next_door = self.open_detected
                next_servo = next_door / self.gear_ratio
                self.direction = 0

        self.servo_angle = next_servo
        self.door_angle = next_door

        torque = self.fake_torque()

        if self.detecting and not self.detect_armed:
            if torque < self.torque_threshold:
                self.detect_armed = True
        elif self.detecting and torque >= self.torque_threshold:
            if self.detect_stage == 1:
                self.closed_detected = self.door_angle
                self.direction = 1
                self.detect_stage = 2
                self.detect_armed = False
                self.get_logger().info(f"Closed endpoint detected: {self.closed_detected:.2f}")
            elif self.detect_stage == 2:
                self.open_detected = self.door_angle
                self.direction = 0
                self.detecting = False
                self.calibrated = True
                self.get_logger().info(f"Open endpoint detected: {self.open_detected:.2f}")
                self.get_logger().info("Calibration done")

        state = (
            f"STATE servo={self.servo_angle:.2f} "
            f"door={self.door_angle:.2f} "
            f"speed={self.speed:.2f} "
            f"dir={self.direction} "
            f"torque={torque:.2f} "
            f"calibrated={int(self.calibrated)} "
            f"detecting={int(self.detecting)} "
            f"closed={self.closed_detected:.2f} "
            f"open={self.open_detected:.2f}"
        )

        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

        servo_msg = Float32()
        servo_msg.data = float(self.servo_angle)
        self.servo_pub.publish(servo_msg)

        door_msg = Float32()
        door_msg.data = float(self.door_angle)
        self.door_pub.publish(door_msg)

        torque_msg = Float32()
        torque_msg.data = float(torque)
        self.torque_pub.publish(torque_msg)

        write_row(
            self.log_path,
            {
                "time": time.time(),
                "servo_deg": self.servo_angle,
                "door_deg": self.door_angle,
                "torque": torque,
                "speed": self.speed,
                "direction": self.direction,
                "calibrated": int(self.calibrated),
                "detecting": int(self.detecting),
            },
        )


def main():
    rclpy.init()
    node = SimulatedESP32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
