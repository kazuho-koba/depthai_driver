#!/usr/bin/env python3

"""Receive a bounded RGB preview without opening a GUI window."""

import depthai as dai


def main():
    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.ColorCamera)
    camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camera.setResolution(
        dai.ColorCameraProperties.SensorResolution.THE_1080_P
    )
    camera.setPreviewSize(300, 300)
    camera.setInterleaved(False)
    camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camera.setFps(15)

    output = pipeline.create(dai.node.XLinkOut)
    output.setStreamName("rgb")
    camera.preview.link(output.input)

    with dai.Device(pipeline) as device:
        queue = device.getOutputQueue("rgb", maxSize=4, blocking=False)
        for index in range(30):
            image = queue.get().getCvFrame()
            print(f"frame {index}: shape={image.shape}, dtype={image.dtype}")

    print("OK")


if __name__ == "__main__":
    main()
