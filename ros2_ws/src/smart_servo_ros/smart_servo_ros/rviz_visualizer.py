import math
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class RvizVisualizer(Node):
    def __init__(self):
        super().__init__("rviz_visualizer")

        self.servo_angle = 0.0
        self.door_angle = 0.0
        self.closed_limit = 0.0
        self.open_limit = 0.0
        self.speed = 0.0
        self.direction = 0
        self.torque = 0.0
        self.calibrated = False
        self.detecting = False

        self.sub = self.create_subscription(
            String,
            "/smart_servo/state",
            self.state_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/smart_servo/markers",
            10,
        )

        self.timer = self.create_timer(0.05, self.publish_markers)

    def state_callback(self, msg):
        text = msg.data
        if not text.startswith("STATE "):
            return

        def extract(name):
            m = re.search(name + r"=([-0-9.]+)", text)
            return float(m.group(1)) if m else 0.0

        self.servo_angle = extract("servo")
        self.door_angle = extract("door")
        self.speed = extract("speed")
        self.direction = int(extract("dir"))
        self.torque = extract("torque")
        self.closed_limit = extract("closed")
        self.open_limit = extract("open")
        self.calibrated = "calibrated=1" in text
        self.detecting = "detecting=1" in text

    def make_arm(self, marker_id, angle_deg, length, z, name):
        angle = math.radians(angle_deg)

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = z

        marker.pose.orientation.z = math.sin(angle / 2.0)
        marker.pose.orientation.w = math.cos(angle / 2.0)

        marker.scale.x = length
        marker.scale.y = 0.04
        marker.scale.z = 0.04

        marker.color.a = 1.0

        if name == "servo":
            marker.color.r = 0.1
            marker.color.g = 0.4
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.1

        return marker

    def make_text(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "text"
        marker.id = 100
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = -1.15
        marker.pose.position.z = 0.2

        marker.scale.z = 0.13

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        if self.detecting:
            motion = "CALIBRATING ENDPOINTS"
        elif self.direction > 0:
            motion = "CCW (+), RIGHT key"
        elif self.direction < 0:
            motion = "CW (-), LEFT key"
        else:
            motion = "STOPPED"

        calibration = "READY / LIMITS ACTIVE" if self.calibrated else "NOT CALIBRATED"
        marker.text = (
            "TOP VIEW (+Z)  |  Positive angle = CCW\n"
            f"Motion: {motion}  |  Speed: {self.speed:.0f} deg/s\n"
            f"BLUE motor: {self.servo_angle:.1f} deg\n"
            f"ORANGE door: {self.door_angle:.1f} deg  (motor x 15/40)\n"
            f"Torque: {self.torque:.1f}  |  {calibration}\n"
            f"Limits: {self.closed_limit:.1f} to {self.open_limit:.1f} deg"
        )

        return marker

    def make_limit_marker(self, marker_id, angle_deg, name):
        angle = math.radians(angle_deg)

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.02

        marker.pose.orientation.z = math.sin(angle / 2.0)
        marker.pose.orientation.w = math.cos(angle / 2.0)

        marker.scale.x = 0.8
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        marker.color.a = 0.6
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker

    def publish_markers(self):
        markers = [
            self.make_arm(1, self.servo_angle, 1.0, 0.15, "servo"),
            self.make_arm(2, self.door_angle, 0.75, 0.0, "door"),
            self.make_text(),
        ]

        if self.calibrated:
            markers.append(self.make_limit_marker(3, self.closed_limit, "closed_limit"))
            markers.append(self.make_limit_marker(4, self.open_limit, "open_limit"))

        for marker in markers:
            self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = RvizVisualizer()
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
