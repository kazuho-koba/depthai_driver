#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFreqCheck(Node):
    def __init__(self):
        super().__init__('imu_freq_check')
        self.prev_header_t = None
        self.prev_recv_t = None
        self.count = 0
        self.create_subscription(Imu, '/imu0', self.cb, 200)

    def cb(self, msg):
        header_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        recv_t = self.get_clock().now().nanoseconds * 1e-9

        if self.prev_header_t is not None:
            dt_header = header_t - self.prev_header_t
            dt_recv = recv_t - self.prev_recv_t
            print(
                f"dt_header_ms={dt_header*1000:.3f}, "
                f"dt_recv_ms={dt_recv*1000:.3f}"
            )

        self.prev_header_t = header_t
        self.prev_recv_t = recv_t

        self.count += 1
        if self.count >= 100:
            rclpy.shutdown()


rclpy.init()
node = ImuFreqCheck()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
