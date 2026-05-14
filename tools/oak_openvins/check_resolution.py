import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Check(Node):
    def __init__(self):
        super().__init__('check_image_size')
        self.create_subscription(Image, '/cam0/image_raw', self.cb, 10)

    def cb(self, msg):
        print("width:", msg.width, "height:",
              msg.height, "encoding:", msg.encoding)
        rclpy.shutdown()


rclpy.init()
node = Check()
rclpy.spin(node)
