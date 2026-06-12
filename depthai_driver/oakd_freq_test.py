#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from collections import defaultdict

from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge

import depthai as dai
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

        # =========================
        # 検証用統計
        # =========================
        self.stats = defaultdict(float)
        self.last_stat_time = time.time()

        self.last_color_ts = None
        self.last_depth_ts = None
        self.last_left_ts = None
        self.last_right_ts = None

        self.max_poll_ms = 0.0
        self.max_rgbd_ms = 0.0
        self.max_color_cv_ms = 0.0
        self.max_depth_cv_ms = 0.0
        self.max_color_pub_ms = 0.0
        self.max_depth_pub_ms = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.get_logger().info("set depth as 10")

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

        self.pub_left = self.create_publisher(Image, "/oak/stereo/left/image_raw", 5)
        self.pub_right = self.create_publisher(Image, "/oak/stereo/right/image_raw", 5)
        self.pub_imu = self.create_publisher(Imu, "/oak/imu/data", 100)

        self.pub_color = self.create_publisher(
            Image, "/oak/color/image_raw", sensor_qos
        )
        self.pub_depth = self.create_publisher(
            Image, "/oak/depth/image_raw", sensor_qos
        )

        self.pipeline = dai.Pipeline()

        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(mono_fps)

        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setFps(mono_fps)

        color_cam = self.pipeline.create(dai.node.ColorCamera)
        color_cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        color_cam.setFps(rgb_fps)
        color_cam.setPreviewSize(640, 400)
        color_cam.setPreviewKeepAspectRatio(False)
        color_cam.setInterleaved(False)
        color_cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(640, 400)
        stereo.initialConfig.setConfidenceThreshold(240)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        imu = self.pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, imu_fps)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, imu_fps)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(1)

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
            f"imu_fps={imu_fps}, "
            f"rgbd_publish_every_n={self.rgbd_publish_every_n}"
        )

    def dai_time_to_ros_msg(self, dai_time):
        dai_sec = dai_time.total_seconds()

        if not hasattr(self, "dai_to_ros_offset"):
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            self.dai_to_ros_offset = ros_now - dai_sec
            self.get_logger().info(
                f"DepthAI-to-ROS time offset initialized: "
                f"{self.dai_to_ros_offset:.6f} sec"
            )

        ros_sec_float = dai_sec + self.dai_to_ros_offset
        sec = int(ros_sec_float)
        nanosec = int((ros_sec_float - sec) * 1e9)

        msg = Time()
        msg.sec = sec
        msg.nanosec = nanosec
        return msg

    def _update_period_stats(self, name, dai_msg):
        ts = dai_msg.getTimestamp().total_seconds()
        last_attr = f"last_{name}_ts"
        last_ts = getattr(self, last_attr)

        if last_ts is not None:
            dt = ts - last_ts
            if dt > 0:
                self.stats[f"{name}_period_sum"] += dt
                self.stats[f"{name}_period_count"] += 1
                self.stats[f"{name}_period_max"] = max(
                    self.stats[f"{name}_period_max"], dt
                )

        setattr(self, last_attr, ts)

    def poll(self):
        poll_t0 = time.perf_counter()
        self.stats["poll_count"] += 1

        left = self.q_left.tryGet()
        right = self.q_right.tryGet()
        color = self.q_color.tryGet()
        depth = self.q_depth.tryGet()

        if left is not None:
            self.stats["left_rx"] += 1
            self._update_period_stats("left", left)
            self.latest_left = left

        if right is not None:
            self.stats["right_rx"] += 1
            self._update_period_stats("right", right)
            self.latest_right = right

        if color is not None:
            self.stats["color_rx"] += 1
            self._update_period_stats("color", color)
            self.latest_color = color
            color_received = True

        if depth is not None:
            self.stats["depth_rx"] += 1
            self._update_period_stats("depth", depth)
            self.latest_depth = depth

        if self.latest_left is not None and self.latest_right is not None:
            left_msg_dai = self.latest_left
            right_msg_dai = self.latest_right

            self.latest_left = None
            self.latest_right = None

            self.stats["stereo_pair"] += 1
            self.vio_frame_count += 1

            stamp = self.dai_time_to_ros_msg(left_msg_dai.getTimestamp())

            t0 = time.perf_counter()
            left_frame = left_msg_dai.getCvFrame()
            right_frame = right_msg_dai.getCvFrame()
            self.stats["mono_getcv_ms_sum"] += (time.perf_counter() - t0) * 1000.0
            self.stats["mono_getcv_count"] += 1

            left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding="mono8")
            left_msg.header.stamp = stamp
            left_msg.header.frame_id = "cam0"

            right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding="mono8")
            right_msg.header.stamp = stamp
            right_msg.header.frame_id = "cam1"

            t_pub = time.perf_counter()
            self.pub_left.publish(left_msg)
            self.pub_right.publish(right_msg)
            self.stats["mono_pub_ms_sum"] += (time.perf_counter() - t_pub) * 1000.0
            self.stats["mono_pub_count"] += 1

        if color_received:
            self.publish_latest_rgbd()

        imu_packets = self.q_imu.tryGet()
        if imu_packets is not None:
            self.stats["imu_batch_rx"] += 1
            self.stats["imu_packet_rx"] += len(imu_packets.packets)

            for packet in imu_packets.packets:
                msg = Imu()

                if hasattr(packet.acceleroMeter, "getTimestamp"):
                    stamp = self.dai_time_to_ros_msg(
                        packet.acceleroMeter.getTimestamp()
                    )
                elif hasattr(packet.gyroscope, "getTimestamp"):
                    stamp = self.dai_time_to_ros_msg(packet.gyroscope.getTimestamp())
                else:
                    self.stats["imu_no_timestamp"] += 1
                    continue

                msg.header.stamp = stamp
                msg.header.frame_id = "imu0"

                accel = packet.acceleroMeter
                gyro = packet.gyroscope

                msg.linear_acceleration.x = accel.x
                msg.linear_acceleration.y = accel.y
                msg.linear_acceleration.z = accel.z

                msg.angular_velocity.x = gyro.x
                msg.angular_velocity.y = gyro.y
                msg.angular_velocity.z = gyro.z

                msg.orientation_covariance[0] = -1.0

                self.pub_imu.publish(msg)
                self.stats["imu_pub"] += 1

        poll_ms = (time.perf_counter() - poll_t0) * 1000.0
        self.max_poll_ms = max(self.max_poll_ms, poll_ms)
        self.stats["poll_ms_sum"] += poll_ms

        self.print_stats_if_needed()

    def publish_latest_rgbd(self):
        rgbd_t0 = time.perf_counter()

        if self.latest_color is None:
            self.stats["color_missing_at_rgbd"] += 1

        if self.latest_depth is None:
            self.stats["depth_missing_at_rgbd"] += 1

        if self.latest_color is None or self.latest_depth is None:
            return

        # DepthAIフレーム取得
        t0 = time.perf_counter()
        color_frame = self.latest_color.getCvFrame()
        color_get_ms = (time.perf_counter() - t0) * 1000.0
        self.stats["color_getcv_ms_sum"] += color_get_ms
        self.max_color_cv_ms = max(self.max_color_cv_ms, color_get_ms)

        t0 = time.perf_counter()
        depth_frame = self.latest_depth.getFrame()
        depth_get_ms = (time.perf_counter() - t0) * 1000.0
        self.stats["depth_getframe_ms_sum"] += depth_get_ms
        self.max_depth_cv_ms = max(self.max_depth_cv_ms, depth_get_ms)

        # cv_bridge変換
        t0 = time.perf_counter()
        color_msg = self.bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")
        color_cv_ms = (time.perf_counter() - t0) * 1000.0
        self.stats["color_bridge_ms_sum"] += color_cv_ms

        color_msg.header.stamp = self.dai_time_to_ros_msg(
            self.latest_color.getTimestamp()
        )
        color_msg.header.frame_id = "oak_rgb_camera_optical_frame"

        t0 = time.perf_counter()
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="16UC1")
        depth_cv_ms = (time.perf_counter() - t0) * 1000.0
        self.stats["depth_bridge_ms_sum"] += depth_cv_ms

        depth_msg.header.stamp = self.dai_time_to_ros_msg(
            self.latest_depth.getTimestamp()
        )
        depth_msg.header.frame_id = "oak_rgb_camera_optical_frame"

        self.stats["rgbd_ready"] += 1

        # ROS publish処理時間
        t0 = time.perf_counter()
        self.pub_color.publish(color_msg)
        color_pub_ms = (time.perf_counter() - t0) * 1000.0
        self.stats["color_pub_ms_sum"] += color_pub_ms
        self.max_color_pub_ms = max(self.max_color_pub_ms, color_pub_ms)

        t0 = time.perf_counter()
        self.pub_depth.publish(depth_msg)
        depth_pub_ms = (time.perf_counter() - t0) * 1000.0
        self.stats["depth_pub_ms_sum"] += depth_pub_ms
        self.max_depth_pub_ms = max(self.max_depth_pub_ms, depth_pub_ms)

        self.stats["color_pub"] += 1
        self.stats["depth_pub"] += 1

        rgbd_ms = (time.perf_counter() - rgbd_t0) * 1000.0
        self.stats["rgbd_ms_sum"] += rgbd_ms
        self.max_rgbd_ms = max(self.max_rgbd_ms, rgbd_ms)

        # 同じフレームを再publishしない
        self.latest_color = None
        self.latest_depth = None

    def _avg_period_ms(self, name):
        count = self.stats[f"{name}_period_count"]
        if count <= 0:
            return 0.0
        return 1000.0 * self.stats[f"{name}_period_sum"] / count

    def _hz_from_count(self, key, dt):
        if dt <= 0:
            return 0.0
        return self.stats[key] / dt

    def print_stats_if_needed(self):
        now = time.time()
        dt = now - self.last_stat_time

        if dt < 1.0:
            return

        poll_avg_ms = (
            self.stats["poll_ms_sum"] / self.stats["poll_count"]
            if self.stats["poll_count"] > 0
            else 0.0
        )

        rgbd_avg_ms = (
            self.stats["rgbd_ms_sum"] / self.stats["rgbd_ready"]
            if self.stats["rgbd_ready"] > 0
            else 0.0
        )

        color_bridge_avg_ms = (
            self.stats["color_bridge_ms_sum"] / self.stats["rgbd_ready"]
            if self.stats["rgbd_ready"] > 0
            else 0.0
        )

        depth_bridge_avg_ms = (
            self.stats["depth_bridge_ms_sum"] / self.stats["rgbd_ready"]
            if self.stats["rgbd_ready"] > 0
            else 0.0
        )

        self.get_logger().info(
            "\n"
            "===== OAK-D PIPELINE STATS =====\n"
            f"window={dt:.2f}s\n"
            "\n"
            "[DepthAI queue RX]\n"
            f"left_rx ={self.stats['left_rx']:.0f} "
            f"({self._hz_from_count('left_rx', dt):.1f} Hz), "
            f"period_avg={self._avg_period_ms('left'):.1f} ms\n"
            f"right_rx={self.stats['right_rx']:.0f} "
            f"({self._hz_from_count('right_rx', dt):.1f} Hz), "
            f"period_avg={self._avg_period_ms('right'):.1f} ms\n"
            f"color_rx={self.stats['color_rx']:.0f} "
            f"({self._hz_from_count('color_rx', dt):.1f} Hz), "
            f"period_avg={self._avg_period_ms('color'):.1f} ms\n"
            f"depth_rx={self.stats['depth_rx']:.0f} "
            f"({self._hz_from_count('depth_rx', dt):.1f} Hz), "
            f"period_avg={self._avg_period_ms('depth'):.1f} ms\n"
            "\n"
            "[Internal publish path]\n"
            f"stereo_pair={self.stats['stereo_pair']:.0f} "
            f"({self._hz_from_count('stereo_pair', dt):.1f} Hz)\n"
            f"rgbd_call ={self.stats['rgbd_call']:.0f} "
            f"({self._hz_from_count('rgbd_call', dt):.1f} Hz)\n"
            f"rgbd_ready={self.stats['rgbd_ready']:.0f} "
            f"({self._hz_from_count('rgbd_ready', dt):.1f} Hz)\n"
            f"color_pub ={self.stats['color_pub']:.0f} "
            f"({self._hz_from_count('color_pub', dt):.1f} Hz)\n"
            f"depth_pub ={self.stats['depth_pub']:.0f} "
            f"({self._hz_from_count('depth_pub', dt):.1f} Hz)\n"
            "\n"
            "[Missing at RGBD call]\n"
            f"color_missing={self.stats['color_missing_at_rgbd']:.0f}, "
            f"depth_missing={self.stats['depth_missing_at_rgbd']:.0f}\n"
            "\n"
            "[IMU]\n"
            f"imu_batch_rx={self.stats['imu_batch_rx']:.0f}, "
            f"imu_packet_rx={self.stats['imu_packet_rx']:.0f}, "
            f"imu_pub={self.stats['imu_pub']:.0f} "
            f"({self._hz_from_count('imu_pub', dt):.1f} Hz)\n"
            "\n"
            "[Processing time]\n"
            f"poll_avg={poll_avg_ms:.2f} ms, "
            f"poll_max={self.max_poll_ms:.2f} ms\n"
            f"rgbd_avg={rgbd_avg_ms:.2f} ms, "
            f"rgbd_max={self.max_rgbd_ms:.2f} ms\n"
            f"color_bridge_avg={color_bridge_avg_ms:.2f} ms, "
            f"depth_bridge_avg={depth_bridge_avg_ms:.2f} ms\n"
            f"color_pub_max={self.max_color_pub_ms:.2f} ms, "
            f"depth_pub_max={self.max_depth_pub_ms:.2f} ms\n"
            "===============================\n"
        )

        self.stats.clear()
        self.last_stat_time = now

        self.max_poll_ms = 0.0
        self.max_rgbd_ms = 0.0
        self.max_color_cv_ms = 0.0
        self.max_depth_cv_ms = 0.0
        self.max_color_pub_ms = 0.0
        self.max_depth_pub_ms = 0.0


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
