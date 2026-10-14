#!/usr/bin/env python3
"""OAK-D S2のVIO入力、RGB-D画像、非侵襲診断情報をpublishする。

ImageとIMU topicは安定したOpenVINS入力interfaceである。deviceごとのtimestamp、
sequence、露光、USB identityはDiagnosticArrayとして別出力し、診断機能によって
VIO側のROS契約を変えない。

``poll()``はnon-blocking DepthAI queueをdrainし、各packetをROS messageへ変換して、
同じROS headerのmetadataをpublishし、stream別sequence gapを追跡する。定期的な
device-info publishにより、camera nodeより後にrosbag記録を始めてもidentityとUSB状態を残す。
"""

import rclpy
import json
import time
import math
from depthai_driver.publish_timing import TimingBuffer, TimedPublisher
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from collections import defaultdict

from sensor_msgs.msg import CameraInfo, Image, Imu
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import depthai as dai
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from depthai_driver.confidence_recording import configuration_dict, snapshot
from depthai_driver.clock_mapping import ClockMapping, ClockMonitor, timedelta_ns


class OakdVioRgbdNode(Node):
    # 2 msのPython timerは画像変換・publishやrosbag serializationで遅延し得る。20 Hzで
    # stereo frameを2個だけqueueすると履歴は100 msしかなく、小さなhost側stallでもVIO入力を
    # 見えない形で失う。1秒分の上限付きstereo bufferを持たせ、より大きなstallは過大queueで
    # 隠してburst再生せず、sequence診断に現れるようにする。
    STEREO_OUTPUT_QUEUE_SIZE = 20
    RGBD_OUTPUT_QUEUE_SIZE = 10
    IMU_OUTPUT_QUEUE_SIZE = 250

    def __init__(self):
        super().__init__("oakd_vio_rgbd_node")

        self.bridge = CvBridge()
        # 現在時計の対応付けを既定にする。旧方式はパラメータ1つで復元できる。
        # 同じmapperを全画像・IMUで共有し、ストリーム別の時計原点を作らない。
        self.declare_parameter('timestamp_mode', 'host_clock')
        self.clock_mapping = ClockMapping(
            str(self.get_parameter('timestamp_mode').value),
            lambda: self.get_clock().now().nanoseconds,
            lambda: timedelta_ns(dai.Clock.now()))
        # 既存device_info timer（5秒）で監視し、新しい高頻度callbackは増やさない。
        # 5msは初期読み取り幅15µsより十分大きい暫定の警告値で、VO許容誤差ではない。
        self.declare_parameter('clock_monitor_enabled', True)
        self.declare_parameter('clock_warn_threshold_ms', 5.0)
        self.declare_parameter('clock_max_reference_span_ms', 1.0)
        threshold = float(self.get_parameter('clock_warn_threshold_ms').value)
        max_span = float(self.get_parameter('clock_max_reference_span_ms').value)
        if not all(math.isfinite(v) and v > 0 for v in (threshold, max_span)):
            raise ValueError('時計監視のms閾値は正の有限値が必要です')
        self.clock_monitor = (ClockMonitor(self.clock_mapping, round(threshold*1e6), round(max_span*1e6))
                              if self.get_parameter('clock_monitor_enabled').value else None)
        self.declare_parameter("timing_diagnostics", False)
        self.declare_parameter("timing_csv", "/tmp/oak_publish_timing.csv")
        # 診断ON時のみ保持。センサ設定やpublish周期は変更せず、試験時間に
        # 合わせてpoll/publishの履歴容量を指定する（単位はmessageでなくsample）。
        self.declare_parameter("timing_max_samples", 150000)
        self.timing = (TimingBuffer(str(self.get_parameter("timing_csv").value),
                                   int(self.get_parameter("timing_max_samples").value))
                       if self.get_parameter("timing_diagnostics").value else None)

        # stereo frameはpoll()でdevice sequenceにより対にする。両側が揃うまで独立に保持し、
        # host到着順で組み合わせて片側のUSB/host scheduling遅延を隠さない。
        self.pending_left = {}
        self.pending_right = {}
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
        # 既存depth処理は変更しない。追加診断だけをOFFにしてpayloadを抑制できる。
        self.declare_parameter("publish_depth_confidence", True)
        self.declare_parameter("confidence_threshold", 240)
        self.publish_depth_confidence = bool(self.get_parameter("publish_depth_confidence").value)
        confidence_threshold = int(self.get_parameter("confidence_threshold").value)
        if not 0 <= confidence_threshold <= 255:
            raise ValueError("confidence_threshold must be 0..255")
        self.declare_parameter("rgb_optical_frame", "rgb_camera_optical_frame")
        mono_fps = float(self.get_parameter("mono_fps").value)
        rgb_fps = float(self.get_parameter("rgb_fps").value)
        imu_fps = int(self.get_parameter("imu_fps").value)
        self.rgb_optical_frame = str(
            self.get_parameter("rgb_optical_frame").value
        )

        # （主に）Visual Odometry用のセンサ情報パブリッシャ
        self.pub_left = self.create_publisher(Image, "/oak/stereo/left/image_raw", 5)
        self.pub_right = self.create_publisher(Image, "/oak/stereo/right/image_raw", 5)
        self.pub_imu = self.create_publisher(Imu, "/oak/imu/data", 100)

        # sampleごとのDepthAI metadataは安定したImage/Imu interfaceと分離する。これにより
        # 既存OpenVINS入力を保ちつつ、device clock同期とsequence gapをrosbagで観測可能にする。
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
        self.pub_depth_info = self.create_publisher(
            CameraInfo, "/oak/depth/camera_info", 1
        )
        self.pub_color_info = self.create_publisher(
            CameraInfo, "/oak/color/camera_info", 1
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
        stereo.initialConfig.setConfidenceThreshold(confidence_threshold)
        self.stereo = stereo
        self.confidence_frames = {}
        self.disparity_frames = {}
        self.diagnostic_depth_headers = {}
        if self.publish_depth_confidence:
            self.pub_confidence = self.create_publisher(
                Image, "/oak/stereo/confidence/image_raw", sensor_qos)
            self.pub_disparity = self.create_publisher(
                Image, "/oak/stereo/disparity/image_raw", sensor_qos)
            self.pub_confidence_metadata = self.create_publisher(
                DiagnosticArray, "/oak/diagnostics/confidence_frame", 50)
            self.pub_disparity_metadata = self.create_publisher(
                DiagnosticArray, "/oak/diagnostics/disparity_frame", 50)
            snapshot_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_stereo_snapshot = self.create_publisher(
                String, "/oak/stereo/recording_snapshot", snapshot_qos)
            for name, output in (("confidence", stereo.confidenceMap),
                                 ("disparity", stereo.disparity),
                                 ("stereo_config", stereo.outConfig)):
                link = self.pipeline.createXLinkOut()
                link.setStreamName(name)
                output.link(link.input)

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
        if self.publish_depth_confidence:
            self.q_confidence = self.device.getOutputQueue("confidence", maxSize=20, blocking=False)
            self.q_disparity = self.device.getOutputQueue("disparity", maxSize=20, blocking=False)
            self.q_stereo_config = self.device.getOutputQueue("stereo_config", maxSize=5, blocking=False)
            # 起動時の校正・pipeline・SDK設定を保存し、outConfig到着後には実出力設定も追記する。
            self.stereo_snapshot = snapshot(self.device, self.pipeline, stereo, dai, {
                "mono_fps": mono_fps, "rgb_fps": rgb_fps, "imu_fps": imu_fps,
                "confidence_threshold": confidence_threshold, "publish_depth_confidence": True,
            })
            self.stereo_snapshot["out_config"] = None
            self.snapshot_message = String(data=json.dumps(self.stereo_snapshot))
            self.stereo_snapshot_timer = self.create_timer(5.0, self.publish_stereo_snapshot)
            self.publish_stereo_snapshot()
        self.depth_camera_info = self.make_depth_camera_info()
        self.color_camera_info = self.make_color_camera_info()
        # ROS publisher遅延がOAK pipelineをblockしないようhost queueをnon-blockingにする。
        # 一時的なhost stallを吸収できる容量を持たせ、poll()は保留sampleを全てdrainする。
        self.q_left = self.device.getOutputQueue(
            "left", maxSize=self.STEREO_OUTPUT_QUEUE_SIZE, blocking=False
        )
        self.q_right = self.device.getOutputQueue(
            "right", maxSize=self.STEREO_OUTPUT_QUEUE_SIZE, blocking=False
        )
        self.q_color = self.device.getOutputQueue(
            "color", maxSize=self.RGBD_OUTPUT_QUEUE_SIZE, blocking=False
        )
        self.q_depth = self.device.getOutputQueue(
            "depth", maxSize=self.RGBD_OUTPUT_QUEUE_SIZE, blocking=False
        )
        self.q_imu = self.device.getOutputQueue(
            "imu", maxSize=self.IMU_OUTPUT_QUEUE_SIZE, blocking=False
        )

        if self.timing is not None:
            # Nodeが所有する実publisherは保持し、呼出し側だけを計測proxyへ置き換える。
            for name, publisher in list(vars(self).items()):
                if name.startswith('pub_') and hasattr(publisher, 'publish'):
                    setattr(self, name, TimedPublisher(publisher, self.timing))
        self.timer = self.create_timer(0.002, self.poll)
        # rosbagのtopic discoveryが遅れても記録されるようdevice identityを繰り返し出す。
        # ROS時計が逆行しても監視を止めない。既存5秒診断を単調時計で駆動する。
        # 同じexecutorが長時間停止した場合の監視遅延までは防げない。
        self.device_info_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.device_info_timer = self.create_timer(
            5.0, self.publish_device_info, clock=self.device_info_clock)
        self.publish_device_info()

        self.get_logger().info(
            "OAK-D VIO + RGB-D publisher started: "
            f"mono_fps={mono_fps}, rgb_fps={rgb_fps}, "
        )

    def make_depth_camera_info(self):
        """RGB基準でaspect ratioを保って出力するdepth画像のCameraInfoを作る。"""
        return self.make_rgb_camera_info(keep_aspect_ratio=True)

    def make_color_camera_info(self):
        """stretch preview設定に一致するcolor画像のCameraInfoを作る。"""
        return self.make_rgb_camera_info(keep_aspect_ratio=False)

    def make_rgb_camera_info(self, keep_aspect_ratio):
        """RGB EEPROM校正値を指定画像変換に合わせてCameraInfoへ格納する。

        RGB previewは640x400へaspect ratioを保たずstretchする一方、StereoDepthの
        depth出力は既定でaspect ratioを保ってresize/cropする。そのため画像寸法が同じ
        でも、各streamに対応する焦点距離が異なる。ここではKとDを起動時に一度だけ
        作り、各frameではtimestampだけを更新してROS message生成負荷を抑える。
        """
        calibration = self.device.readCalibration()
        intrinsics = calibration.getCameraIntrinsics(
            dai.CameraBoardSocket.RGB, 640, 400,
            keepAspectRatio=keep_aspect_ratio,
        )
        distortion = calibration.getDistortionCoefficients(
            dai.CameraBoardSocket.RGB
        )
        message = CameraInfo()
        message.header.frame_id = self.rgb_optical_frame
        message.width = 640
        message.height = 400
        message.distortion_model = "rational_polynomial"
        message.d = [float(value) for value in distortion[:8]]
        message.k = [
            float(intrinsics[0][0]), 0.0, float(intrinsics[0][2]),
            0.0, float(intrinsics[1][1]), float(intrinsics[1][2]),
            0.0, 0.0, 1.0,
        ]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [
            message.k[0], 0.0, message.k[2], 0.0,
            0.0, message.k[4], message.k[5], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return message

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
            # 大きな逆方向jumpはdropではなくdevice/pipeline resetとみなす。
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
        # depthはmono_fpsで生成するが、意図してcolor frame到着時（rgb_fps）だけpublishする。
        # したがってそのsequence skipは転送dropではなく想定したrate変換である。
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
            # DepthAI 2.30が公開するのはanalog gainではなくISO sensitivityである。
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
            # bagに方式と実際の対応付けを残す。生device/host timestampは変更しない。
            self.key_value('timestamp_mode', self.clock_mapping.mode),
            self.key_value('ros_host_offset_ns', self.clock_mapping.offset_ns),
            self.key_value('clock_reference_span_ns', self.clock_mapping.reference_span_ns),
        ]
        status.values.append(self.key_value('clock_monitor_enabled', self.clock_monitor is not None))
        if self.clock_monitor is not None:
            # 観測差だけを記録し、Headerのオフセットは絶対に再代入しない。
            # 過去の超過回数・最大差も保持し、復帰後に異常履歴を失わない。
            clock_status = self.clock_monitor.check()
            status.values.extend(self.key_value(key, value) for key, value in clock_status.items())
            state = clock_status['clock_monitor_state']
            if state == 'NOT_INITIALIZED':
                status.level = DiagnosticStatus.STALE
            elif state != 'OK':
                status.level = DiagnosticStatus.WARN
            status.message = state
        message.status = [status]
        self.pub_device_info.publish(message)

    def dai_time_to_ros_msg(self, dai_time):
        """取得時刻を共通ROS基準へ変換する。publish時刻への置換・丸めはしない。"""
        initializing = self.clock_mapping.offset_ns is None
        ros_ns = self.clock_mapping.convert(timedelta_ns(dai_time))
        if initializing:
            self.get_logger().info(
                'DepthAI-to-ROS clock mapping: mode={} offset_ns={} reference_span_ns={}'.format(
                    self.clock_mapping.mode, self.clock_mapping.offset_ns,
                    self.clock_mapping.reference_span_ns)
            )
        sec, nanosec = divmod(ros_ns, 10**9)
        msg = Time()
        msg.sec = sec
        msg.nanosec = nanosec
        return msg

    def publish_stereo_pair(self, left_msg_dai, right_msg_dai):
        """Publish one sequence-matched stereo pair without dropping backlog."""
        self.vio_frame_count += 1

        # Use the host-synchronized DepthAI timestamp from one image. Since
        # left/right are captured by the stereo pair, publish the same stamp.
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

    def poll(self):
        """診断ON時だけcallback全体を計測し、実処理は共通の_pollへ渡す。"""
        if self.timing is None:
            return self._poll()
        start = time.perf_counter()
        try:
            return self._poll()
        finally:
            self.timing.add('poll', start, time.perf_counter()-start)

    def _poll(self):
        """各host queueを取得し、stereo全ペア・最新RGB-D・IMUを既存規約で配信する。"""
        if self.publish_depth_confidence:
            # 診断出力は既存depthと同じdevice sequenceだけpublishする。到着順の違いで
            # depthのpublishを待たせず、小さな上限付きcacheで後着packetを照合する。
            for queue, cache in ((self.q_confidence, self.confidence_frames),
                                 (self.q_disparity, self.disparity_frames)):
                for frame in queue.tryGetAll():
                    cache[frame.getSequenceNum()] = frame
                while len(cache) > 20:
                    cache.pop(min(cache))
            configs = self.q_stereo_config.tryGetAll()
            # runtime設定入力は使っていないため、実効設定の初回だけJSON化する。
            # 毎20Hzで同じ大きなsnapshotを生成・配信してCPU/payloadを増やさない。
            if configs and self.stereo_snapshot["out_config"] is None:
                self.stereo_snapshot["out_config"] = configuration_dict(configs[-1].get())
                self.snapshot_message.data = json.dumps(self.stereo_snapshot)
                self.publish_stereo_snapshot()
            self.publish_depth_diagnostics()
        # -------------------------
        # Drain every host-queue item before publishing.  tryGet() alone takes
        # only one item per timer callback; with a small non-blocking queue,
        # that made host scheduler stalls appear as multi-second device drops.
        # -------------------------
        for frame in self.q_left.tryGetAll():
            self.pending_left[frame.getSequenceNum()] = frame
        for frame in self.q_right.tryGetAll():
            self.pending_right[frame.getSequenceNum()] = frame

        color_frames = self.q_color.tryGetAll()
        depth_frames = self.q_depth.tryGetAll()
        if self.timing is not None:
            # これはhost取得batch内のlatest-only省略数で、device内部dropではない。
            for name, frames in (('queue_color', color_frames), ('queue_depth', depth_frames)):
                if frames:
                    self.timing.add(name, time.perf_counter(), 0, len(frames), max(0, len(frames)-1))
        if color_frames:
            self.latest_color = color_frames[-1]
        if depth_frames:
            self.latest_depth = depth_frames[-1]

        # -------------------------
        # Publish stereo mono pair for VIO
        # -------------------------
        for sequence_num in sorted(
            set(self.pending_left).intersection(self.pending_right)
        ):
            self.publish_stereo_pair(
                self.pending_left.pop(sequence_num),
                self.pending_right.pop(sequence_num),
            )

        # -------------------------
        # Publish RGB-D at decimated rate
        # -------------------------
        if color_frames:
            self.publish_latest_rgbd()

        # --- IMU ---
        for imu_packets in self.q_imu.tryGetAll():
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
        color_msg.header.frame_id = self.rgb_optical_frame

        depth_frame = self.latest_depth.getFrame()
        # StereoDepth depth output is usually uint16 depth in millimeters.
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="16UC1")
        depth_msg.header.stamp = self.dai_time_to_ros_msg(
            self.latest_depth.getTimestamp()
        )
        depth_msg.header.frame_id = self.rgb_optical_frame

        self.pub_color.publish(color_msg)
        self.pub_depth.publish(depth_msg)
        # 画像ごとに対応するCameraInfoのtimestampを設定する。K/Dは起動時に生成済み。
        self.color_camera_info.header.stamp = color_msg.header.stamp
        self.pub_color_info.publish(self.color_camera_info)
        self.depth_camera_info.header.stamp = depth_msg.header.stamp
        self.pub_depth_info.publish(self.depth_camera_info)
        self.pub_color_metadata.publish(self.make_frame_metadata(
            self.latest_color, "color", color_msg.header
        ))
        self.pub_depth_metadata.publish(self.make_frame_metadata(
            self.latest_depth, "depth", depth_msg.header
        ))
        if self.publish_depth_confidence:
            self.diagnostic_depth_headers[self.latest_depth.getSequenceNum()] = depth_msg.header
            while len(self.diagnostic_depth_headers) > 20:
                self.diagnostic_depth_headers.pop(min(self.diagnostic_depth_headers))
            self.publish_depth_diagnostics()

        self.latest_color = None
        self.latest_depth = None

    def publish_stereo_snapshot(self):
        """後から開始したmission recorderにも校正・設定が届くよう低頻度で再配信する。"""
        self.pub_stereo_snapshot.publish(self.snapshot_message)

    def publish_depth_diagnostics(self):
        """depth採用sequenceと一致する2画像だけ出力する。片方欠落時には別frameを代用しない。"""
        sequences = (set(self.diagnostic_depth_headers) & set(self.confidence_frames)
                     & set(self.disparity_frames))
        for sequence in sorted(sequences):
            header = self.diagnostic_depth_headers.pop(sequence)
            for cache, publisher, metadata_publisher, name in (
                    (self.confidence_frames, self.pub_confidence, self.pub_confidence_metadata, "confidence"),
                    (self.disparity_frames, self.pub_disparity, self.pub_disparity_metadata, "disparity")):
                packet = cache.pop(sequence)
                data = packet.getFrame()
                encoding = "mono8" if data.dtype.itemsize == 1 else "16UC1"
                message = self.bridge.cv2_to_imgmsg(data, encoding=encoding)
                # 元depthのstampを厳密にコピー。元packetのdevice stampはmetadata側へ保持する。
                message.header.stamp = header.stamp
                message.header.frame_id = ("oak_stereo_right_rectified_optical_frame"
                                           if name == "confidence" else self.rgb_optical_frame)
                publisher.publish(message)
                metadata_publisher.publish(self.make_frame_metadata(packet, name, message.header))


def main(args=None):
    rclpy.init(args=args)
    node = OakdVioRgbdNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            if node.timing is not None:
                node.timing.save()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
