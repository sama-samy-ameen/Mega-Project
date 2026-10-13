import os
import re
import cv2
import numpy as np

# ---------- EDIT THESE ----------
SET_DIR = "../canvases"  # folder with your image set
ANN_DIR = "../canvases"  # folder with original YOLO .txt annotations
OVERLAY_PATH = "water.jpeg"  # the image that goes on top
OUT_DIR = "../overlaid"  # where new images and their labels are saved
LEVELS = [80, 60, 40]  # overlay opacity in percent
# --------------------------------

os.makedirs(OUT_DIR, exist_ok=True)

# IMREAD_UNCHANGED keeps the alpha channel if the overlay has one
overlay = cv2.imread(OVERLAY_PATH, cv2.IMREAD_UNCHANGED)
if overlay is None:
    raise FileNotFoundError(f"Could not load overlay image from '{OVERLAY_PATH}'. Check the path.")


def get_annotation_content(stem):
    """Finds and reads the YOLO .txt file corresponding to an image stem,
       handling clean stems or existing suffixes."""
    for s in (stem, re.sub(r"_\d+$", "", stem)):
        path = os.path.join(ANN_DIR, s + ".txt")
        if os.path.exists(path):
            with open(path, "r") as f:
                return f.read().strip()
    return None


for name in sorted(os.listdir(SET_DIR)):
    if not name.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".webp")):
        continue

    img_path = os.path.join(SET_DIR, name)
    img = cv2.imread(img_path)  # 3 channels (BGR)
    if img is None:
        continue

    h, w = img.shape[:2]

    # Resize overlay to cover the whole image
    ov = cv2.resize(overlay, (w, h))
    ov_bgr = ov[:, :, :3].astype(float)
    if ov.shape[2] == 4:
        ov_alpha = ov[:, :, 3:4] / 255.0  # logo's own transparency
    else:
        ov_alpha = np.ones((h, w, 1))  # no alpha: fully visible

    base, ext = os.path.splitext(name)

    # Get original annotations for this image (bounding boxes don't change position during an overlay)
    annotation_text = get_annotation_content(base)

    for level in LEVELS:
        a = ov_alpha * (level / 100.0)
        # Fixed syntax error: changed (1 a) to (1.0 - a)
        result = img * (1.0 - a) + ov_bgr * a

        new_filename_base = f"{base}_{level}"

        # Save modified image
        img_out_path = os.path.join(OUT_DIR, f"{new_filename_base}{ext}")
        cv2.imwrite(img_out_path, result.astype(np.uint8))

        # Save copied/corresponding YOLO label file
        if annotation_text:
            label_out_path = os.path.join(OUT_DIR, f"{new_filename_base}.txt")
            with open(label_out_path, "w") as f:
                f.write(annotation_text)

        print("saved", f"{new_filename_base}{ext} + label")