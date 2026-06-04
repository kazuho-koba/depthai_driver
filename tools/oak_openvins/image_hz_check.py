#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageHzCheck(Node):
    def __init__(self):
        super().__init__("image_hz_check")
        self.count = 0
        self.sub = self.create_subscription(
            Image,
            "/oak/color/image_raw",
            self.cb,
            10
        )
        self.timer = self.create_timer(1.0, self.report)

    def cb(self, msg):
        self.count += 1

    def report(self):
        self.get_logger().info(f"received: {self.count} Hz")
        self.count = 0

def main():
    rclpy.init()
    node = ImageHzCheck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()