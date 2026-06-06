import cv2
import depthai as dai

# Pipeline
pipeline = dai.Pipeline()

# Color camera
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

# OAK-D S2のRGBカメラは1080P以上推奨
cam_rgb.setResolution(
    dai.ColorCameraProperties.SensorResolution.THE_1080_P
)

cam_rgb.setPreviewSize(640, 350)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(
    dai.ColorCameraProperties.ColorOrder.BGR
)
cam_rgb.setFps(10)
cam_rgb.setPreviewKeepAspectRatio(False)

# 出力
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")

cam_rgb.preview.link(xout_rgb.input)

# Device
with dai.Device(pipeline) as device:

    q_rgb = device.getOutputQueue(
        name="rgb",
        maxSize=4,
        blocking=False
    )

    while True:

        rgb_msg = q_rgb.get()
        img = rgb_msg.getCvFrame()

        cv2.imshow("OAK-D S2 RGB", img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()