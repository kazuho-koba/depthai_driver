import depthai as dai

with dai.Device() as device:
    print("MX ID:", device.getMxId())
    print("USB speed:", device.getUsbSpeed())