"""原画像の保存経路を変えず、遠隔表示用の小さなJPEGを別プロセスで配信する。

raw画像はBEST_EFFORT/履歴1で受け、最新1枚だけ保持する。timerで最大rateを
制限し、表示subscriberがいない間は変換しない。遠隔publishが待たされても、
OAK本体のexecutorとは独立する。ただしDDS内部資源の完全な隔離は実機で検証する。
"""
import math
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage


def encode_preview(frame, max_width, jpeg_quality):
    """BGR画像を縮小してJPEG bytesにする。縦横比を保ち、元画像は変更しない。"""
    height, width = frame.shape[:2]
    if width > max_width:
        target_height = max(1, round(height * max_width / width))
        frame = cv2.resize(frame, (max_width, target_height), interpolation=cv2.INTER_AREA)
    ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    if not ok:
        raise ValueError('プレビューのJPEG encodeに失敗しました')
    return data.tobytes()


class OakPreviewNode(Node):
    """受信画像を最新1枚に制限し、低帯域の表示専用topicへ変換する。"""

    def __init__(self):
        super().__init__('oak_preview')
        for name, default in (
                ('input_topic', '/oak/color/image_raw'),
                ('output_topic', '/oak/preview/image/compressed'),
                ('publish_rate_hz', 5.0), ('max_width', 160),
                ('jpeg_quality', 50), ('max_age_sec', 0.5)):
            self.declare_parameter(name, default)
        self.rate = float(self.get_parameter('publish_rate_hz').value)
        self.width = int(self.get_parameter('max_width').value)
        self.quality = int(self.get_parameter('jpeg_quality').value)
        self.max_age = float(self.get_parameter('max_age_sec').value)
        if (not math.isfinite(self.rate) or not 0 < self.rate <= 30
                or not math.isfinite(self.max_age) or self.max_age <= 0
                or not 1 <= self.width <= 1920 or not 1 <= self.quality <= 100):
            raise ValueError('previewのrate/width/quality/max_ageが範囲外です')
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        if input_topic == output_topic:
            raise ValueError('入力と出力topicを同じ名前にはできません')
        # readerもwriterも履歴1。遅い表示のために原画像の履歴を保持しない。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(CompressedImage, output_topic, qos)
        self.subscription = self.create_subscription(Image, input_topic, self.receive, qos)
        self.bridge = CvBridge()
        self.latest = None
        self.timer = self.create_timer(1.0 / self.rate, self.publish_latest)
        self.get_logger().info('表示専用JPEG: {} -> {}, {} Hz, 幅上限{} px, quality={}'.format(
            input_topic, output_topic, self.rate, self.width, self.quality))

    def receive(self, message):
        """単一executorで画像参照を置換する。受信callbackでは変換・送信しない。"""
        self.latest = (message, time.monotonic())

    def publish_latest(self):
        """最新1枚を消費する。古い／購読者なしの画像を後から再配信しない。"""
        latest, self.latest = self.latest, None
        if latest is None or self.publisher.get_subscription_count() == 0:
            return
        message, received_at = latest
        # IMU側で既知のheader時計offsetに依存せず、ローカル受信後のageを使う。
        if time.monotonic() - received_at > self.max_age:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            preview = CompressedImage()
            preview.header = message.header
            preview.format = 'bgr8; jpeg compressed bgr8'
            preview.data = encode_preview(frame, self.width, self.quality)
            self.publisher.publish(preview)
        except (ValueError, RuntimeError, CvBridgeError, cv2.error) as error:
            self.get_logger().error('表示専用JPEG生成／publish失敗: {}'.format(error))


def main(args=None):
    """camera driverとは別プロセスで動作し、SIGINTで通常終了する。"""
    rclpy.init(args=args)
    node = None
    try:
        node = OakPreviewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
