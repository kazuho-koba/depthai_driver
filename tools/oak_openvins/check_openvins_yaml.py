#!/usr/bin/env python3
import cv2
import sys

path = sys.argv[1]
fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

if not fs.isOpened():
    print("FAILED to open:", path)
    sys.exit(1)

for cam in ["cam0", "cam1"]:
    print(f"\n--- {cam} ---")
    node = fs.getNode(cam)
    if node.empty():
        print("missing")
        continue

    for key in ["T_imu_cam", "intrinsics", "distortion_coeffs", "resolution"]:
        n = node.getNode(key)
        print(f"{key}: empty={n.empty()} isSeq={n.isSeq()} isMap={n.isMap()}")

        try:
            m = n.mat()
            print("mat shape:", None if m is None else m.shape)
            print(m)
        except Exception as e:
            print("mat read error:", e)

    for key in ["camera_model", "distortion_model", "rostopic"]:
        try:
            print(f"{key}:", node.getNode(key).string())
        except Exception as e:
            print(f"{key} read error:", e)

fs.release()