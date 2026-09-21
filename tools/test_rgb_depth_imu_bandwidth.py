#!/usr/bin/env python3

"""Exercise RGB, stereo depth, and IMU streams and display their rates."""

import time
from collections import deque

import cv2
import depthai as dai
import numpy as np


RGB_WIDTH = 1280
RGB_HEIGHT = 800
RGB_FPS = 30
DEPTH_FPS = 30
DEPTH_MAX_MM = 10000
IMU_FREQUENCY = 200


def colorize_depth(depth_frame, max_depth_mm):
    depth_clipped = np.clip(depth_frame, 0, max_depth_mm)
    depth_8bit = (255 * (1.0 - depth_clipped / max_depth_mm)).astype(np.uint8)
    depth_8bit[depth_frame == 0] = 0
    return cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)


def measured_rate(timestamps):
    if len(timestamps) < 2:
        return 0.0
    duration = timestamps[-1] - timestamps[0]
    return (len(timestamps) - 1) / duration if duration > 0 else 0.0


def create_pipeline():
    pipeline = dai.Pipeline()

    rgb = pipeline.create(dai.node.ColorCamera)
    rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb.setPreviewSize(RGB_WIDTH, RGB_HEIGHT)
    rgb.setPreviewKeepAspectRatio(False)
    rgb.setInterleaved(False)
    rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    rgb.setFps(RGB_FPS)

    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    for camera, socket in (
        (left, dai.CameraBoardSocket.CAM_B),
        (right, dai.CameraBoardSocket.CAM_C),
    ):
        camera.setBoardSocket(socket)
        camera.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        camera.setFps(DEPTH_FPS)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(True)
    stereo.initialConfig.setConfidenceThreshold(50)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, IMU_FREQUENCY)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, IMU_FREQUENCY)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    for name, source in (
        ("rgb", rgb.preview),
        ("depth", stereo.depth),
        ("imu", imu.out),
    ):
        output = pipeline.create(dai.node.XLinkOut)
        output.setStreamName(name)
        source.link(output.input)
    return pipeline


def main():
    rgb_times = deque(maxlen=60)
    depth_times = deque(maxlen=60)
    imu_times = deque(maxlen=300)
    latest_rgb = None
    latest_depth = None
    latest_center_depth = 0
    latest_acceleration = None
    last_print = time.monotonic()

    print("Running test. Press 'q' or Escape to quit.")
    with dai.Device(create_pipeline()) as device:
        print(f"MX ID: {device.getMxId()}")
        print(f"USB speed: {device.getUsbSpeed()}")
        rgb_queue = device.getOutputQueue("rgb", maxSize=4, blocking=False)
        depth_queue = device.getOutputQueue("depth", maxSize=4, blocking=False)
        imu_queue = device.getOutputQueue("imu", maxSize=50, blocking=False)

        while True:
            now = time.monotonic()
            rgb_message = rgb_queue.tryGet()
            if rgb_message is not None:
                latest_rgb = rgb_message.getCvFrame()
                rgb_times.append(now)

            depth_message = depth_queue.tryGet()
            if depth_message is not None:
                depth = depth_message.getFrame()
                latest_depth = colorize_depth(depth, DEPTH_MAX_MM)
                center_y, center_x = depth.shape[0] // 2, depth.shape[1] // 2
                latest_center_depth = int(depth[center_y, center_x])
                cv2.circle(latest_depth, (center_x, center_y), 5, (255, 255, 255), -1)
                depth_times.append(now)

            imu_message = imu_queue.tryGet()
            if imu_message is not None:
                for packet in imu_message.packets:
                    acceleration = packet.acceleroMeter
                    latest_acceleration = (
                        acceleration.x, acceleration.y, acceleration.z
                    )
                    imu_times.append(time.monotonic())

            if latest_rgb is not None:
                image = latest_rgb.copy()
                cv2.putText(image, f"RGB FPS: {measured_rate(rgb_times):.1f}",
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0), 2)
                cv2.imshow("RGB", image)
            if latest_depth is not None:
                image = latest_depth.copy()
                cv2.putText(image, f"Depth FPS: {measured_rate(depth_times):.1f}",
                            (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (255, 255, 255), 2)
                cv2.imshow("Depth", image)

            if now - last_print >= 1.0:
                last_print = now
                acceleration_norm = None
                if latest_acceleration is not None:
                    acceleration_norm = sum(
                        value * value for value in latest_acceleration
                    ) ** 0.5
                print(
                    f"RGB={measured_rate(rgb_times):5.1f} fps | "
                    f"Depth={measured_rate(depth_times):5.1f} fps | "
                    f"IMU={measured_rate(imu_times):6.1f} Hz | "
                    f"depth={latest_center_depth:5d} mm | "
                    f"|acc|={'None' if acceleration_norm is None else f'{acceleration_norm:.2f} m/s^2'}"
                )

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
