"""カメラなしで縮小・JPEG・最新画像のみ採用する動作を検証する。"""
from types import SimpleNamespace
import time
import cv2
import numpy as np
import pytest
from cv_bridge import CvBridge
from depthai_driver.oak_preview_node import encode_preview, OakPreviewNode


def test_jpeg_preserves_aspect_and_input():
    original = np.zeros((400, 640, 3), dtype=np.uint8)
    original[:, :, 2] = 255
    result = encode_preview(original, 320, 70)
    decoded = cv2.imdecode(np.frombuffer(result, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (200, 320, 3)
    assert decoded[0, 0, 2] > 240
    assert original.shape == (400, 640, 3)
    assert len(result) < original.nbytes / 10


def test_small_image_is_not_upscaled():
    result = encode_preview(np.zeros((20, 40, 3), np.uint8), 320, 70)
    assert cv2.imdecode(np.frombuffer(result, np.uint8), 1).shape == (20, 40, 3)


@pytest.mark.parametrize('subscribers, age, expected', [(0, 0, 0), (1, 1, 0), (1, 0, 1)])
def test_latest_consumed_header_and_gui_rgb_decode(subscribers, age, expected):
    bridge = CvBridge()
    frame = np.zeros((400, 640, 3), np.uint8)
    frame[:, :, 2] = 255
    message = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    message.header.frame_id = 'rgb_camera_optical_frame'
    message.header.stamp.sec = 123
    sent = []
    # ROSやハードウェアを起動せず、本物のtimer処理に必要な状態だけを渡す。
    node = SimpleNamespace(latest=(message, time.monotonic()-age), width=320,
        quality=70, max_age=.5, bridge=bridge,
        publisher=SimpleNamespace(get_subscription_count=lambda: subscribers, publish=sent.append))
    OakPreviewNode.publish_latest(node)
    assert node.latest is None
    assert len(sent) == expected
    if sent:
        assert sent[0].header == message.header
        # GUIと同じcv_bridge APIでJPEGをRGBへ変換し、赤／青の入れ替わりを防ぐ。
        decoded = bridge.compressed_imgmsg_to_cv2(sent[0], desired_encoding='rgb8')
        assert decoded.shape == (200, 320, 3)
        assert decoded[0, 0, 0] > 240
    OakPreviewNode.publish_latest(node)
    assert len(sent) == expected
