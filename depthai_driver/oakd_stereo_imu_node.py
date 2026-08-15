#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge

import depthai as dai
import numpy as np
from builtin_interfaces.msg import Time


class OakdStereoImuNode(Node):
    def __init__(self):
        super().__init__("oakd_stereo_imu_node")

        self.bridge = CvBridge()

        self.latest_left = None
        self.latest_right = None

        self.pub_left = self.create_publisher(Image, "/oak/stereo/left/image_raw", 10)
        self.pub_right = self.create_publisher(Image, "/oak/stereo/right/image_raw", 10)
        self.pub_imu = self.create_publisher(Imu, "/oak/imu/data", 200)

        self.pipeline = dai.Pipeline()

        # Left mono camera
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(20)

        # Right mono camera
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setFps(20)

        # IMU
        imu = self.pipeline.create(dai.node.IMU)
        # imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
        # imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, 125)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, 125)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        # XLink outputs
        xout_left = self.pipeline.createXLinkOut()
        xout_left.setStreamName("left")
        mono_left.out.link(xout_left.input)

        xout_right = self.pipeline.createXLinkOut()
        xout_right.setStreamName("right")
        mono_right.out.link(xout_right.input)

        xout_imu = self.pipeline.createXLinkOut()
        xout_imu.setStreamName("imu")
        imu.out.link(xout_imu.input)

        self.device = dai.Device(self.pipeline)

        self.q_left = self.device.getOutputQueue("left", maxSize=10, blocking=False)
        self.q_right = self.device.getOutputQueue("right", maxSize=10, blocking=False)
        self.q_imu = self.device.getOutputQueue("imu", maxSize=50, blocking=False)

        self.timer = self.create_timer(0.001, self.poll)

        self.get_logger().info("OAK-D stereo + IMU publisher started")

    def dai_time_to_ros_msg(self, dai_time):
        """
        Convert DepthAI timestamp(datetime.timedelta) to ROS builtin_interfaces/Time.
        We align the first DepthAI timestamp to the current ROS clock.
        """
        dai_sec = dai_time.total_seconds()

        if not hasattr(self, "dai_to_ros_offset"):
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            self.dai_to_ros_offset = ros_now - dai_sec
            self.get_logger().info(
                f"DepthAI-to-ROS time offset initialized: {self.dai_to_ros_offset:.6f} sec"
            )

        ros_sec_float = dai_sec + self.dai_to_ros_offset
        sec = int(ros_sec_float)
        nanosec = int((ros_sec_float - sec) * 1e9)

        msg = Time()
        msg.sec = sec
        msg.nanosec = nanosec
        return msg

    def dai_ts_to_ros_time(self, dai_msg):
        # 最初はホスト受信時刻を使用。厳密なVIOでは後で要改善。
        return self.get_clock().now().to_msg()

    def poll(self):
        # --- Stereo image pair ---
        left = self.q_left.tryGet()
        right = self.q_right.tryGet()

        if left is not None:
            self.latest_left = left

        if right is not None:
            self.latest_right = right

        if self.latest_left is not None and self.latest_right is not None:
            left_msg_dai = self.latest_left
            right_msg_dai = self.latest_right

            # clear buffer after making one pair
            self.latest_left = None
            self.latest_right = None

            # Use DepthAI timestamp from one image.
            # Since left/right are captured by the stereo pair, we publish them with the same stamp.
            stamp = self.dai_time_to_ros_msg(left_msg_dai.getTimestamp())
            left_frame = left_msg_dai.getCvFrame()
            right_frame = right_msg_dai.getCvFrame()

            left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding="mono8")
            left_msg.header.stamp = stamp
            left_msg.header.frame_id = "cam0"

            right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding="mono8")
            right_msg.header.stamp = stamp
            right_msg.header.frame_id = "cam1"

            self.pub_left.publish(left_msg)
            self.pub_right.publish(right_msg)

        # --- IMU ---
        imu_packets = self.q_imu.tryGet()
        if imu_packets is not None:
            for packet in imu_packets.packets:
                msg = Imu()

                # Prefer DepthAI device timestamp.
                # Use accelerometer timestamp as representative timestamp
                # for the combined accel+gyro IMU message.
                if hasattr(packet.acceleroMeter, "getTimestamp"):
                    stamp = self.dai_time_to_ros_msg(
                        packet.acceleroMeter.getTimestamp()
                    )
                elif hasattr(packet.gyroscope, "getTimestamp"):
                    stamp = self.dai_time_to_ros_msg(packet.gyroscope.getTimestamp())
                else:
                    continue

                msg.header.stamp = stamp
                msg.header.frame_id = "imu0"

                accel = packet.acceleroMeter
                gyro = packet.gyroscope

                # Keep raw IMU axes as-is.
                msg.linear_acceleration.x = accel.x
                msg.linear_acceleration.y = accel.y
                msg.linear_acceleration.z = accel.z

                msg.angular_velocity.x = gyro.x
                msg.angular_velocity.y = gyro.y
                msg.angular_velocity.z = gyro.z

                msg.orientation_covariance[0] = -1.0

                self.pub_imu.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OakdStereoImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
