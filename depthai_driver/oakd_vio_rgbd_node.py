#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge

import depthai as dai
import numpy as np
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class OakdVioRgbdNode(Node):
    def __init__(self):
        super().__init__("oakd_vio_rgbd_node")

        self.bridge = CvBridge()

        self.latest_left = None
        self.latest_right = None
        self.latest_color = None
        self.latest_depth = None

        self.vio_frame_count = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # パラメータ類
        self.declare_parameter("mono_fps", 20.0)
        self.declare_parameter("rgb_fps", 10.0)
        self.declare_parameter("imu_fps", 125.0)
        self.declare_parameter("rgbd_publish_every_n", 2)
        mono_fps = float(self.get_parameter("mono_fps").value)
        rgb_fps = float(self.get_parameter("rgb_fps").value)
        imu_fps = int(self.get_parameter("imu_fps").value)
        self.rgbd_publish_every_n = int(
            self.get_parameter("rgbd_publish_every_n").value
        )
        if self.rgbd_publish_every_n < 1:
            self.rgbd_publish_every_n = 1

        # （主に）Visual Odometry用のセンサ情報パブリッシャ
        self.pub_left = self.create_publisher(Image, "/cam0/image_raw", 10)
        self.pub_right = self.create_publisher(Image, "/cam1/image_raw", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu0", 200)

        # その他のセンサ情報パブリッシャ
        self.pub_color = self.create_publisher(
            Image, "/oak/color/image_raw", sensor_qos
        )
        self.pub_depth = self.create_publisher(
            Image, "/oak/depth/image_raw", sensor_qos
        )

        self.pipeline = dai.Pipeline()

        # Left mono camera
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(mono_fps)

        # Right mono camera
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setFps(mono_fps)

        # RGB Camera
        color_cam = self.pipeline.create(dai.node.ColorCamera)
        color_cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        color_cam.setFps(rgb_fps)
        # Use preview output for lightweight RGB publishing.
        color_cam.setPreviewSize(640, 400)
        color_cam.setPreviewKeepAspectRatio(False)
        color_cam.setInterleaved(False)
        color_cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # Stereo Depth
        stereo = self.pipeline.create(dai.node.StereoDepth)

        # Initial conservative settings.
        # Depth output is generated onboard.
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(640, 400)
        stereo.initialConfig.setConfidenceThreshold(245)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # IMU
        imu = self.pipeline.create(dai.node.IMU)
        # imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
        # imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, imu_fps)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, imu_fps)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(1)

        # XLink outputs
        xout_left = self.pipeline.createXLinkOut()
        xout_left.setStreamName("left")
        mono_left.out.link(xout_left.input)

        xout_right = self.pipeline.createXLinkOut()
        xout_right.setStreamName("right")
        mono_right.out.link(xout_right.input)

        xout_color = self.pipeline.createXLinkOut()
        xout_color.setStreamName("color")
        color_cam.preview.link(xout_color.input)

        xout_depth = self.pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        xout_imu = self.pipeline.createXLinkOut()
        xout_imu.setStreamName("imu")
        imu.out.link(xout_imu.input)

        # -------------------------
        # Device and output queues
        # -------------------------
        self.device = dai.Device(self.pipeline)
        self.q_left = self.device.getOutputQueue("left", maxSize=2, blocking=False)
        self.q_right = self.device.getOutputQueue("right", maxSize=2, blocking=False)
        self.q_color = self.device.getOutputQueue("color", maxSize=1, blocking=False)
        self.q_depth = self.device.getOutputQueue("depth", maxSize=1, blocking=False)
        self.q_imu = self.device.getOutputQueue("imu", maxSize=50, blocking=False)

        self.timer = self.create_timer(0.002, self.poll)

        self.get_logger().info(
            "OAK-D VIO + RGB-D publisher started: "
            f"mono_fps={mono_fps}, rgb_fps={rgb_fps}, "
            f"imu_fps={imu_fps}, rgbd_publish_every_n={self.rgbd_publish_every_n}"
        )

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

    def poll(self):
        # -------------------------
        # Read latest queue data
        # -------------------------
        left = self.q_left.tryGet()
        right = self.q_right.tryGet()
        color = self.q_color.tryGet()
        depth = self.q_depth.tryGet()

        if left is not None:
            self.latest_left = left

        if right is not None:
            self.latest_right = right

        if color is not None:
            self.latest_color = color

        if depth is not None:
            self.latest_depth = depth

        # -------------------------
        # Publish stereo mono pair for VIO
        # -------------------------
        if self.latest_left is not None and self.latest_right is not None:
            left_msg_dai = self.latest_left
            right_msg_dai = self.latest_right

            # clear buffer after making one pair
            self.latest_left = None
            self.latest_right = None

            self.vio_frame_count += 1

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

            # -------------------------
            # Publish RGB-D at decimated rate
            # -------------------------
            publish_rgbd = self.vio_frame_count % self.rgbd_publish_every_n == 0
            if publish_rgbd:
                self.publish_latest_rgbd()

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

    def publish_latest_rgbd(self):
        """
        Publish the most recent RGB and depth frames.

        The stamp is intentionally tied to the VIO stereo frame timing.
        This makes RGB-D output approximately synchronized to every Nth
        mono stereo pair.
        """

        if self.latest_color is None or self.latest_depth is None:
            return
        
        color_frame = self.latest_color.getCvFrame()
        color_msg = self.bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")
        color_msg.header.stamp = self.dai_time_to_ros_msg(self.latest_color.getTimestamp())
        color_msg.header.frame_id = "oak_rgb_camera_optical_frame"

        depth_frame = self.latest_depth.getFrame()
        # StereoDepth depth output is usually uint16 depth in millimeters.
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="16UC1")
        depth_msg.header.stamp = self.dai_time_to_ros_msg(self.latest_depth.getTimestamp())
        depth_msg.header.frame_id = "oak_rgb_camera_optical_frame"

        self.pub_color.publish(color_msg)
        self.pub_depth.publish(depth_msg)

        self.latest_color = None
        self.latest_depth = None


def main(args=None):
    rclpy.init(args=args)
    node = OakdVioRgbdNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
