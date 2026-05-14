#!/usr/bin/env python3
import depthai as dai
import numpy as np

def print_mat(name, mat):
    print(f"\n{name}:")
    print(np.array(mat))

with dai.Device() as device:
    calib = device.readCalibration()

    # OAK-D系では多くの場合:
    # CAM_A = RGB, CAM_B = LEFT mono, CAM_C = RIGHT mono
    cams = {
        "RGB_CAM_A": dai.CameraBoardSocket.CAM_A,
        "LEFT_CAM_B": dai.CameraBoardSocket.CAM_B,
        "RIGHT_CAM_C": dai.CameraBoardSocket.CAM_C,
    }

    print("EEPROM data:")
    print(calib.getEepromData())

    for name, socket in cams.items():
        try:
            M = calib.getCameraIntrinsics(socket, 640, 400)
            D = calib.getDistortionCoefficients(socket)
            print_mat(f"{name} intrinsics 640x400", M)
            print(f"{name} distortion:")
            print(D)
        except Exception as e:
            print(f"{name}: failed to read intrinsics/distortion: {e}")

    try:
        ext_left_right = calib.getCameraExtrinsics(
            dai.CameraBoardSocket.CAM_B,
            dai.CameraBoardSocket.CAM_C
        )
        print_mat("Extrinsics LEFT_CAM_B -> RIGHT_CAM_C", ext_left_right)
        print("NOTE: DepthAI translation is usually in centimeters.")
    except Exception as e:
        print(f"Failed to read LEFT->RIGHT extrinsics: {e}")

    for name, socket in cams.items():
        try:
            ext_cam_imu = calib.getCameraToImuExtrinsics(socket)
            print_mat(f"Extrinsics {name} -> IMU", ext_cam_imu)
            print("NOTE: DepthAI translation is usually in centimeters.")
        except Exception as e:
            print(f"{name}: failed to read camera->IMU extrinsics: {e}")