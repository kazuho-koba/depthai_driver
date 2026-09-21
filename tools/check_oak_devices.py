#!/usr/bin/env python3

"""List OAK devices visible to the DepthAI runtime without starting streams."""

import depthai as dai


def main():
    devices = dai.Device.getAllAvailableDevices()
    print(f"available devices: {len(devices)}")

    for index, device in enumerate(devices):
        print(f"\nDevice {index}")
        print(f"raw: {device}")
        for attribute in ("mxid", "name", "state", "protocol", "platform"):
            if hasattr(device, attribute):
                print(f"{attribute}: {getattr(device, attribute)}")
        if hasattr(device, "getMxId"):
            print(f"getMxId(): {device.getMxId()}")


if __name__ == "__main__":
    main()
