import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


class Check(Node):
    def __init__(self):
        super().__init__("check_time_offset")
        self.create_subscription(Image, "/cam0/image_raw", self.cb_img, 10)
        self.create_subscription(Imu, "/imu0", self.cb_imu, 100)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def cb_img(self, msg):
        diff = self.now_sec() - self.stamp_sec(msg.header.stamp)
        print(f"cam0 now-stamp = {diff:.3f} sec")

    def cb_imu(self, msg):
        diff = self.now_sec() - self.stamp_sec(msg.header.stamp)
        print(f"imu0 now-stamp = {diff:.3f} sec")


rclpy.init()
node = Check()
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
