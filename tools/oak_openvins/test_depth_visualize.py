import cv2
import numpy as np
import depthai as dai


def colorize_depth(depth_frame: np.ndarray, max_depth_mm: int = 5000) -> np.ndarray:
    depth_clipped = np.clip(depth_frame, 0, max_depth_mm)

    depth_8bit = (255 * (1.0 - depth_clipped / max_depth_mm)).astype(np.uint8)
    depth_8bit[depth_frame == 0] = 0

    return cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)


pipeline = dai.Pipeline()

# 左右のモノクロカメラ
left = pipeline.create(dai.node.MonoCamera)
left.setBoardSocket(dai.CameraBoardSocket.LEFT)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
left.setFps(30)

right = pipeline.create(dai.node.MonoCamera)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right.setFps(30)

# StereoDepth
stereo = pipeline.create(dai.node.StereoDepth)

stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(False)
stereo.setSubpixel(True)
stereo.initialConfig.setConfidenceThreshold(200)

conf = stereo.initialConfig.getConfidenceThreshold()
print("confidence =", conf)

# Subpixel有効時はMedianFilterと併用できない場合があるため、まず無効のまま
# stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)

left.out.link(stereo.left)
right.out.link(stereo.right)

# XLink出力
xout_depth = pipeline.createXLinkOut()
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

with dai.Device(pipeline) as device:
    depth_queue = device.getOutputQueue(
        name="depth",
        maxSize=4,
        blocking=False,
    )

    while True:
        depth_msg = depth_queue.get()
        depth = depth_msg.getFrame()  # uint16, mm単位

        depth_vis = colorize_depth(depth, max_depth_mm=5000)

        center_y = depth.shape[0] // 2
        center_x = depth.shape[1] // 2
        center_depth = int(depth[center_y, center_x])

        cv2.circle(depth_vis, (center_x, center_y), 5, (255, 255, 255), -1)
        cv2.putText(
            depth_vis,
            f"center: {center_depth} mm",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        cv2.imshow("OAK-D S2 Depth", depth_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

cv2.destroyAllWindows()