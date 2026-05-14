#!/usr/bin/env python3

import hashlib

# Patch for old OpenSSL / reportlab compatibility
_orig_md5 = hashlib.md5

def md5_patch(*args, **kwargs):
    kwargs.pop("usedforsecurity", None)
    return _orig_md5(*args, **kwargs)

hashlib.md5 = md5_patch

from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A3

# ===== Settings =====
tag_family = "tag36h11"
tag_filename = "tag36_11"
rows = 5
cols = 5

tag_size_mm = 45.0
spacing_ratio = 0.3
tag_spacing_mm = tag_size_mm * spacing_ratio

output_pdf = "aprilgrid_5x5_tag36h11_A3.pdf"
apriltag_dir = "apriltag-imgs"
# ====================

# A3 landscape: 420 x 297 mm
page_width_mm = 420.0
page_height_mm = 297.0

grid_width_mm = cols * tag_size_mm + (cols - 1) * tag_spacing_mm
grid_height_mm = rows * tag_size_mm + (rows - 1) * tag_spacing_mm

offset_x_mm = (page_width_mm - grid_width_mm) / 2.0
offset_y_mm = (page_height_mm - grid_height_mm) / 2.0

if offset_x_mm < 0 or offset_y_mm < 0:
    raise ValueError("Grid does not fit on A3. Reduce tag_size_mm or rows/cols.")

c = canvas.Canvas(output_pdf, pagesize=(page_width_mm * mm, page_height_mm * mm))

for r in range(rows):
    for col in range(cols):
        tag_id = r * cols + col
        img_path = f"{apriltag_dir}/{tag_family}/{tag_filename}_{tag_id:05d}.png"

        x = offset_x_mm + col * (tag_size_mm + tag_spacing_mm)
        y = offset_y_mm + (rows - 1 - r) * (tag_size_mm + tag_spacing_mm)

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

print(f"Generated: {output_pdf}")
print(f"Page size: {page_width_mm} x {page_height_mm} mm")
print(f"Grid size: {grid_width_mm:.1f} x {grid_height_mm:.1f} mm")
print(f"Tag size: {tag_size_mm:.1f} mm")
print(f"Tag spacing: {tag_spacing_mm:.1f} mm")