#!/usr/bin/env python3

import hashlib

_orig_md5 = hashlib.md5

def md5_patch(*args, **kwargs):
    kwargs.pop("usedforsecurity", None)
    return _orig_md5(*args, **kwargs)

hashlib.md5 = md5_patch

from reportlab.pdfgen import canvas
from reportlab.lib.units import mm

# ===== settings =====

rows = 5
cols = 5

tag_size_mm = 45.0
spacing_ratio = 0.3

spacing_mm = tag_size_mm * spacing_ratio

page_w_mm = 420
page_h_mm = 297

src_dir = "apriltag-imgs-cropped/tag36h11"

output_pdf = "aprilgrid_5x5_tag36h11_cropped_A3.pdf"

# ====================

grid_w = cols * tag_size_mm + (cols - 1) * spacing_mm
grid_h = rows * tag_size_mm + (rows - 1) * spacing_mm

offset_x = (page_w_mm - grid_w) / 2
offset_y = (page_h_mm - grid_h) / 2

c = canvas.Canvas(output_pdf, pagesize=(page_w_mm * mm, page_h_mm * mm))

for r in range(rows):
    for col in range(cols):

        tag_id = r * cols + col

        img_path = f"{src_dir}/tag36_11_{tag_id:05d}.png"

        x = offset_x + col * (tag_size_mm + spacing_mm)
        y = offset_y + (rows - 1 - r) * (tag_size_mm + spacing_mm)

        c.drawImage(
            img_path,
            x * mm,
            y * mm,
            width=tag_size_mm * mm,
            height=tag_size_mm * mm,
            preserveAspectRatio=True,
            mask="auto",
        )

c.save()

print(output_pdf)