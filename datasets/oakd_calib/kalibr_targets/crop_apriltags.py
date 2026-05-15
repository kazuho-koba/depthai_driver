#!/usr/bin/env python3

from PIL import Image
import os

src_dir = "apriltag-imgs/tag36h11"
dst_dir = "apriltag-imgs-cropped/tag36h11"

os.makedirs(dst_dir, exist_ok=True)

for fname in os.listdir(src_dir):

    if not fname.endswith(".png"):
        continue

    path = os.path.join(src_dir, fname)

    img = Image.open(path).convert("L")

    # 白以外領域抽出
    bbox = img.point(lambda p: p < 250 and 255).getbbox()

    if bbox is None:
        continue

    cropped = img.crop(bbox)

    out_path = os.path.join(dst_dir, fname)
    cropped.save(out_path)

    print(f"{fname}: {img.size} -> {cropped.size}")