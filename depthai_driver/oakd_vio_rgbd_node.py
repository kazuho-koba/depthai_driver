#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from collections import defaultdict

from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

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
        self.last_sequence = {}
        self.cumulative_sequence_gap = defaultdict(int)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # パラメータ類
        self.declare_parameter("mono_fps", 20.0)
        self.declare_parameter("rgb_fps", 10.0)
        self.declare_parameter("imu_fps", 125.0)
        mono_fps = float(self.get_parameter("mono_fps").value)
        rgb_fps = float(self.get_parameter("rgb_fps").value)
        imu_fps = int(self.get_parameter("imu_fps").value)

        # （主に）Visual Odometry用のセンサ情報パブリッシャ
        self.pub_left = self.create_publisher(Image, "/oak/stereo/left/image_raw", 5)
        self.pub_right = self.create_publisher(Image, "/oak/stereo/right/image_raw", 5)
        self.pub_imu = self.create_publisher(Imu, "/oak/imu/data", 100)

        # Per-sample DepthAI metadata is kept separate from the stable Image/Imu
        # interfaces. This preserves the existing OpenVINS inputs while making
        # device-clock synchronization and sequence gaps observable in rosbag.
        self.pub_left_metadata = self.create_publisher(
            DiagnosticArray, "/oak/diagnostics/left_frame", 100
        )
        self.pub_right_metadata = self.create_publisher(
            DiagnosticArray, "/oak/diagnostics/right_frame", 100
        )
        self.pub_color_metadata = self.create_publisher(
            DiagnosticArray, "/oak/diagnostics/color_frame", 50
        )
        self.pub_depth_metadata = self.create_publisher(
            DiagnosticArray, "/oak/diagnostics/depth_frame", 50
        )
        self.pub_imu_metadata = self.create_publisher(
            DiagnosticArray, "/oak/diagnostics/imu_packet", 200
        )
        self.pub_device_info = self.create_publisher(
            DiagnosticArray, "/oak/diagnostics/device_info", 10
        )

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
        stereo.initialConfig.setConfidenceThreshold(240)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # IMU
        imu = self.pipeline.create(dai.node.IMU)
        # imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
        # imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, imu_fps)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, imu_fps)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

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
        self.mx_id = str(self.device.getMxId())
        self.q_left = self.device.getOutputQueue("left", maxSize=2, blocking=False)
        self.q_right = self.device.getOutputQueue("right", maxSize=2, blocking=False)
        self.q_color = self.device.getOutputQueue("color", maxSize=1, blocking=False)
        self.q_depth = self.device.getOutputQueue("depth", maxSize=1, blocking=False)
        self.q_imu = self.device.getOutputQueue("imu", maxSize=50, blocking=False)

        self.timer = self.create_timer(0.002, self.poll)
        # Repeat device identity so late rosbag discovery still records it.
        self.device_info_timer = self.create_timer(5.0, self.publish_device_info)
        self.publish_device_info()

        self.get_logger().info(
            "OAK-D VIO + RGB-D publisher started: "
            f"mono_fps={mono_fps}, rgb_fps={rgb_fps}, "
        )

    @staticmethod
    def timedelta_to_ns(value):
        """Convert a DepthAI timedelta to integer nanoseconds."""
        return int(round(value.total_seconds() * 1e9))

    @staticmethod
    def key_value(key, value):
        """Create one stable diagnostic key/value entry."""
        return KeyValue(key=key, value=str(value))

    @staticmethod
    def read_device_value(callback):
        """Read optional device information without stopping sensor output."""
        try:
            return str(callback())
        except Exception as error:
            return "unavailable: {}".format(error)

    def update_sequence_gap(self, stream_name, sequence_num):
        """Return missing messages before sequence_num and cumulative count."""
        previous = self.last_sequence.get(stream_name)
        self.last_sequence[stream_name] = sequence_num
        gap = 0
        if previous is not None:
            delta = (sequence_num - previous) & 0xFFFFFFFF
            # A large backwards jump means device/pipeline reset, not a drop.
            if 0 < delta < 0x80000000:
                gap = max(delta - 1, 0)
        self.cumulative_sequence_gap[stream_name] += gap
        return gap, self.cumulative_sequence_gap[stream_name]

    def make_frame_metadata(self, frame, stream_name, image_header):
        """Build metadata corresponding to one published DepthAI ImgFrame."""
        metadata = DiagnosticArray()
        metadata.header = image_header
        sequence_num = frame.getSequenceNum()
        gap, cumulative = self.update_sequence_gap(
            stream_name, sequence_num
        )
        exposure_time_us = int(round(
            frame.getExposureTime().total_seconds() * 1e6
        ))
        receive_time_ns = self.get_clock().now().nanoseconds
        status = DiagnosticStatus()
        # Depth is generated at mono_fps but intentionally published only
        # when a color frame arrives (rgb_fps), so its sequence skips are
        # expected rate conversion rather than evidence of a transport drop.
        unexpected_gap = bool(gap and stream_name != "depth")
        status.level = (
            DiagnosticStatus.WARN if unexpected_gap else DiagnosticStatus.OK
        )
        status.name = "oak/{}/frame_metadata".format(stream_name)
        status.message = (
            "unexpected sequence gap" if unexpected_gap else
            "expected RGB-D rate conversion" if gap else "OK"
        )
        status.hardware_id = self.mx_id
        status.values = [
            self.key_value("schema_version", 1),
            self.key_value("stream_name", stream_name),
            self.key_value("sequence_num", sequence_num),
            self.key_value(
                "device_timestamp_ns",
                self.timedelta_to_ns(frame.getTimestampDevice()),
            ),
            self.key_value(
                "host_synced_timestamp_ns",
                self.timedelta_to_ns(frame.getTimestamp()),
            ),
            self.key_value("host_receive_time_ns", receive_time_ns),
            self.key_value("sequence_gap", gap),
            self.key_value("cumulative_sequence_gap", cumulative),
            self.key_value("exposure_time_us", exposure_time_us),
            # DepthAI 2.30 exposes ISO sensitivity, not analog gain.
            self.key_value("sensitivity_iso", frame.getSensitivity()),
            self.key_value(
                "color_temperature_kelvin", frame.getColorTemperature()
            ),
            self.key_value("lens_position", frame.getLensPosition()),
            self.key_value("width", frame.getWidth()),
            self.key_value("height", frame.getHeight()),
        ]
        metadata.status = [status]
        return metadata

    def publish_device_info(self):
        """Publish physical OAK identity and the active DepthAI runtime."""
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "oak_device"
        device_info = self.device.getDeviceInfo()
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "oak/device_info"
        status.message = "OK"
        status.hardware_id = self.mx_id
        status.values = [
            self.key_value("schema_version", 1),
            self.key_value("mx_id", self.mx_id),
            self.key_value("device_name", getattr(device_info, "name", "")),
            self.key_value(
                "connected_imu",
                self.read_device_value(self.device.getConnectedIMU),
            ),
            self.key_value(
                "imu_firmware_version",
                self.read_device_value(self.device.getIMUFirmwareVersion),
            ),
            self.key_value(
                "embedded_imu_firmware_version",
                self.read_device_value(
                    self.device.getEmbeddedIMUFirmwareVersion
                ),
            ),
            self.key_value(
                "usb_speed", self.read_device_value(self.device.getUsbSpeed)
            ),
            self.key_value("depthai_version", dai.__version__),
        ]
        message.status = [status]
        self.pub_device_info.publish(message)

    def dai_time_to_ros_msg(self, dai_time):
        """
        Convert DepthAI timestamp(datetime.timedelta) to ROS builtin_interfaces/Time.
        We align the first host-synchronized DepthAI timestamp to the ROS clock.
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

        color_received = False
        if color is not None:
            self.latest_color = color
            color_received = True

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

            # Use the host-synchronized DepthAI timestamp from one image.
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
            self.pub_left_metadata.publish(self.make_frame_metadata(
                left_msg_dai, "left", left_msg.header
            ))
            self.pub_right_metadata.publish(self.make_frame_metadata(
                right_msg_dai, "right", right_msg.header
            ))

        # -------------------------
        # Publish RGB-D at decimated rate
        # -------------------------
        if color_received:
            self.publish_latest_rgbd()

        # --- IMU ---
        imu_packets = self.q_imu.tryGet()
        if imu_packets is not None:
            batch_size = len(imu_packets.packets)
            for packet_index, packet in enumerate(imu_packets.packets):
                msg = Imu()

                # Preserve the existing combined Imu header behavior: use the
                # host-synchronized accelerometer timestamp. Raw device-clock
                # accel/gyro timestamps are published in diagnostic metadata.
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

                accel_sequence = accel.getSequenceNum()
                gyro_sequence = gyro.getSequenceNum()
                accel_gap, cumulative_accel_gap = self.update_sequence_gap(
                    "imu_accelerometer", accel_sequence
                )
                gyro_gap, cumulative_gyro_gap = self.update_sequence_gap(
                    "imu_gyroscope", gyro_sequence
                )
                accel_device_time = accel.getTimestampDevice()
                gyro_device_time = gyro.getTimestampDevice()
                accel_device_time_ns = self.timedelta_to_ns(accel_device_time)
                gyro_device_time_ns = self.timedelta_to_ns(gyro_device_time)
                metadata = DiagnosticArray()
                metadata.header = msg.header
                accel_gyro_delta_ns = (
                    gyro_device_time_ns - accel_device_time_ns
                )
                status = DiagnosticStatus()
                # The BNO086 may run accelerometer and gyroscope at different
                # supported internal rates. Combined packets are paced by the
                # accelerometer, so skipped gyro sequence numbers are normal.
                status.level = (
                    DiagnosticStatus.WARN
                    if accel_gap else DiagnosticStatus.OK
                )
                status.name = "oak/imu_packet_metadata"
                status.message = (
                    "unexpected accelerometer sequence gap"
                    if accel_gap else
                    "expected IMU rate conversion" if gyro_gap else "OK"
                )
                status.hardware_id = self.mx_id
                status.values = [
                    self.key_value("schema_version", 1),
                    self.key_value(
                        "accelerometer_sequence_num", accel_sequence
                    ),
                    self.key_value("gyroscope_sequence_num", gyro_sequence),
                    self.key_value(
                        "accelerometer_device_timestamp_ns",
                        accel_device_time_ns,
                    ),
                    self.key_value(
                        "gyroscope_device_timestamp_ns",
                        gyro_device_time_ns,
                    ),
                    self.key_value(
                        "accelerometer_host_synced_timestamp_ns",
                        self.timedelta_to_ns(accel.getTimestamp()),
                    ),
                    self.key_value(
                        "gyroscope_host_synced_timestamp_ns",
                        self.timedelta_to_ns(gyro.getTimestamp()),
                    ),
                    self.key_value(
                        "host_receive_time_ns",
                        self.get_clock().now().nanoseconds,
                    ),
                    self.key_value(
                        "accel_to_gyro_device_delta_ns", accel_gyro_delta_ns
                    ),
                    self.key_value("accelerometer_sequence_gap", accel_gap),
                    self.key_value("gyroscope_sequence_gap", gyro_gap),
                    self.key_value(
                        "cumulative_accelerometer_sequence_gap",
                        cumulative_accel_gap,
                    ),
                    self.key_value(
                        "cumulative_gyroscope_sequence_gap",
                        cumulative_gyro_gap,
                    ),
                    self.key_value("batch_size", batch_size),
                    self.key_value("packet_index", packet_index),
                ]
                metadata.status = [status]
                self.pub_imu_metadata.publish(metadata)

    def publish_latest_rgbd(self):
        """
        Publish the most recent RGB and depth frames.
        The freq of publishment belongs with freq of RGB image
        """

        if self.latest_color is None or self.latest_depth is None:
            return

        color_frame = self.latest_color.getCvFrame()
        color_msg = self.bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")
        color_msg.header.stamp = self.dai_time_to_ros_msg(
            self.latest_color.getTimestamp()
        )
        color_msg.header.frame_id = "oak_rgb_camera_optical_frame"

        depth_frame = self.latest_depth.getFrame()
        # StereoDepth depth output is usually uint16 depth in millimeters.
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="16UC1")
        depth_msg.header.stamp = self.dai_time_to_ros_msg(
            self.latest_depth.getTimestamp()
        )
        depth_msg.header.frame_id = "oak_rgb_camera_optical_frame"

        self.pub_color.publish(color_msg)
        self.pub_depth.publish(depth_msg)
        self.pub_color_metadata.publish(self.make_frame_metadata(
            self.latest_color, "color", color_msg.header
        ))
        self.pub_depth_metadata.publish(self.make_frame_metadata(
            self.latest_depth, "depth", depth_msg.header
        ))

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
