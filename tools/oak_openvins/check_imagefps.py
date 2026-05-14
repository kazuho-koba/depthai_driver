import depthai as dai
import time

pipeline = dai.Pipeline()

left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)

left.setBoardSocket(dai.CameraBoardSocket.LEFT)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
left.setFps(20)
right.setFps(20)

xout_l = pipeline.create(dai.node.XLinkOut)
xout_r = pipeline.create(dai.node.XLinkOut)
xout_l.setStreamName("left")
xout_r.setStreamName("right")
left.out.link(xout_l.input)
right.out.link(xout_r.input)

with dai.Device(pipeline) as device:
    print("USB speed: ", device.getUsbSpeed())
    ql = device.getOutputQueue("left", maxSize=30, blocking=False)
    qr = device.getOutputQueue("right", maxSize=30, blocking=False)

    cl = cr = 0
    t0 = time.time()
    while time.time() - t0 < 10:
        if ql.tryGet() is not None:
            cl += 1
        if qr.tryGet() is not None:
            cr += 1

    dt = time.time() - t0
    print("left fps: ", cl / dt)
    print("right fps: ", cr / dt)
