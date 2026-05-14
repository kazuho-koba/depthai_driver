import depthai as dai
with dai.Device() as device:
    print("MxId:", device.getMxId())
    print("USB speed:", device.getUsbSpeed())