import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class StampCheck(Node):
    def __init__(self):
        super().__init__('stamp_check')
        self.t0 = None
        self.t1 = None
        self.create_subscription(Image, '/cam0/image_raw', self.cb0, 10)
        self.create_subscription(Image, '/cam1/image_raw', self.cb1, 10)

    def cb0(self, msg):
        self.t0 = msg.header.stamp
        self.show()

    def cb1(self, msg):
        self.t1 = msg.header.stamp
        self.show()

    def show(self):
        if self.t0 is None or self.t1 is None:
            return
        a = self.t0.sec + self.t0.nanosec * 1e-9
        b = self.t1.sec + self.t1.nanosec * 1e-9
        print(f"cam0={a:.9f}, cam1={b:.9f}, diff_ms={(a-b)*1000:.3f}")


rclpy.init()
node = StampCheck()

for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.2)

node.destroy_node()
rclpy.shutdown()
