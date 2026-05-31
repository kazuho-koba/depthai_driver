#!/usr/bin/env python3

import cv2
import depthai as dai

pipeline = dai.Pipeline()

mono_left = pipeline.create(dai.node.MonoCamera)
mono_right = pipeline.create(dai.node.MonoCamera)

mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

mono_left.setFps(20)
mono_right.setFps(20)

xout_left = pipeline.create(dai.node.XLinkOut)
xout_right = pipeline.create(dai.node.XLinkOut)

xout_left.setStreamName("left")
xout_right.setStreamName("right")

mono_left.out.link(xout_left.input)
mono_right.out.link(xout_right.input)

with dai.Device(pipeline) as device:
    print("MX ID:", device.getMxId())
    print("USB speed:", device.getUsbSpeed())

    q_left = device.getOutputQueue("left", maxSize=4, blocking=False)
    q_right = device.getOutputQueue("right", maxSize=4, blocking=False)

    while True:
        left_msg = q_left.tryGet()
        right_msg = q_right.tryGet()

        if left_msg is not None:
            frame = left_msg.getCvFrame()
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            cv2.putText(frame, "CameraBoardSocket.LEFT",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2)
            cv2.imshow("socket LEFT", frame)

        if right_msg is not None:
            frame = right_msg.getCvFrame()
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            cv2.putText(frame, "CameraBoardSocket.RIGHT",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2)
            cv2.imshow("socket RIGHT", frame)

        key = cv2.waitKey(1)
        if key == ord("q") or key == 27:
            break

cv2.destroyAllWindows()