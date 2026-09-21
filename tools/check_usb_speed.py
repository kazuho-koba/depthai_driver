#!/usr/bin/env python3

"""Check that an OAK device negotiated a USB 3.x SuperSpeed link."""

import argparse
import sys

import depthai as dai


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mxid",
        help="MX ID to check; the only attached device is used when omitted",
    )
    return parser.parse_args()


def select_device(mxid):
    devices = dai.Device.getAllAvailableDevices()
    if mxid:
        devices = [device for device in devices if device.getMxId() == mxid]
    if not devices:
        raise RuntimeError("No matching OAK device was found")
    if len(devices) > 1:
        ids = ", ".join(device.getMxId() for device in devices)
        raise RuntimeError(f"Multiple OAK devices found ({ids}); specify --mxid")
    return devices[0]


def main():
    args = parse_args()
    try:
        device_info = select_device(args.mxid)
        with dai.Device(device_info) as device:
            speed = device.getUsbSpeed()
            mxid = device.getMxId()
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    is_usb3 = speed in (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS)
    print(f"MX ID: {mxid}")
    print(f"DepthAI negotiated speed: {speed}")
    if is_usb3:
        print("PASS: USB 3.x SuperSpeed is active (USB 3.0 signaling is 5000 Mb/s).")
        print("Note: 5000 Mb/s is 5 Gb/s, not 5000 MB/s; payload throughput is lower.")
        return 0

    print("FAIL: USB 3.x SuperSpeed is not active.", file=sys.stderr)
    print("Expected UsbSpeed.SUPER or UsbSpeed.SUPER_PLUS.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
