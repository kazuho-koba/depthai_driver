import depthai as dai

with dai.Device() as device:
    calib = device.readCalibration()

    print("MX ID:", device.getMxId())

    print("Board name:", calib.getEepromData().boardName)
    print("Board rev:", calib.getEepromData().boardRev)