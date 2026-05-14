#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu

class StampFreqCheck(Node):
    def __init__(self, topic, msg_type):
        super().__init__('stamp_freq_check')
        self.prev = None
        self.count = 0
        self.topic = topic
        self.create_subscription(msg_type, topic, self.cb, 100)

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev is not None:
            dt = t - self.prev
            print(f"{self.topic}: dt_ms={dt*1000:.3f}")
        self.prev = t
        self.count += 1
        if self.count >= 100:
            rclpy.shutdown()

def main():
    if len(sys.argv) != 3:
        print("usage: python3 check_stamp_freq.py <topic> <image|imu>")
        return

    topic = sys.argv[1]
    kind = sys.argv[2]

    msg_type = Image if kind == "image" else Imu

    rclpy.init()
    node = StampFreqCheck(topic, msg_type)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()