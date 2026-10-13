import os
import re
import random
import cv2
import numpy as np

# ---------- EDIT THESE ----------
SET_DIR = "data_set/train/images"  # folder with the image set to scatter
ANN_DIR = "data_set/train/labels"  # folder with the YOLO .txt annotation of each image
OUT_DIR = "../canvases"  # where canvases and their new annotations are saved
HOW_MANY = 50  # number of canvases to create
CANVAS_W, CANVAS_H = 1280, 720

# Grid Configuration (e.g., 4x4 up to 8x8)
GRID_ROWS, GRID_COLS = 6, 6  # Change between 4 and 8 as needed
FILL_PROBABILITY = 0.6  # Probability (0.0 to 1.0) that a grid box gets an image
MIN_VISIBLE = 0.3  # Drop a box if less than 30% of it is validly on the canvas
# --------------------------------

os.makedirs(OUT_DIR, exist_ok=True)

# Calculate dimensions for each individual grid cell
CELL_W = CANVAS_W // GRID_COLS
CELL_H = CANVAS_H // GRID_ROWS


def load_boxes(stem, w, h):
    """Read a YOLO file supporting both bounding boxes (5 items) and polygons (>=7 items).
       Returns a list of tuples: (class_id, list_of_pixel_points)"""
    for s in (stem, re.sub(r"_\d+$", "", stem)):
        path = os.path.join(ANN_DIR, s + ".txt")
        if os.path.exists(path):
            annotations = []
            for line in open(path):
                p = line.split()
                if len(p) < 5:
                    continue
                c = int(p[0])

                if len(p) == 5:
                    # Standard YOLO Bounding Box: class cx cy w h (normalized 0-1)
                    cx, cy, bw, bh = map(float, p[1:])
                    x1 = (cx - bw / 2) * w
                    y1 = (cy - bh / 2) * h
                    x2 = (cx + bw / 2) * w
                    y2 = (cy + bh / 2) * h
                    # Convert box into 4 corner points for uniform processing
                    pts = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
                    annotations.append((c, pts))

                elif len(p) > 5:
                    # YOLO Segmentation Polygon: class x1 y1 x2 y2 x3 y3 ... (normalized 0-1)
                    coords = list(map(float, p[1:]))
                    pts = []
                    for i in range(0, len(coords), 2):
                        if i + 1 < len(coords):
                            px = coords[i] * w
                            py = coords[i + 1] * h
                            pts.append([px, py])
                    if pts:
                        annotations.append((c, pts))
            return annotations

    print("no annotation found for", stem)
    return []


# Load every image once as BGRA (4 channels) together with its annotations
images = []
for name in sorted(os.listdir(SET_DIR)):
    if name.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".webp")):
        im = cv2.imread(os.path.join(SET_DIR, name), cv2.IMREAD_UNCHANGED)
        if im is None:
            continue
        if im.ndim == 2:
            im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGRA)
        elif im.shape[2] == 3:
            im = cv2.cvtColor(im, cv2.COLOR_BGR2BGRA)
        h, w = im.shape[:2]
        images.append((im, load_boxes(os.path.splitext(name)[0], w, h)))

if not images:
    raise ValueError(f"No valid images found in {SET_DIR}. Please check your path.")


def rotate(im, angle):
    """Rotate without cutting the corners; new corners are transparent."""
    h, w = im.shape[:2]
    center = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)

    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))

    M[0, 2] += (new_w / 2.0) - center[0]
    M[1, 2] += (new_h / 2.0) - center[1]

    rotated = cv2.warpAffine(im, M, (new_w, new_h), borderValue=(0, 0, 0, 0))
    return rotated, M


def paste(canvas, im, x, y):
    """Blend im onto canvas at (x, y); anything outside the canvas is cropped."""
    h, w = im.shape[:2]
    x1, y1 = max(x, 0), max(y, 0)
    x2, y2 = min(x + w, CANVAS_W), min(y + h, CANVAS_H)
    if x1 >= x2 or y1 >= y2:
        return

    part = im[y1 - y:y2 - y, x1 - x:x2 - x]
    a = part[:, :, 3:4] / 255.0

    canvas[y1:y2, x1:x2] = (part[:, :, :3] * a + canvas[y1:y2, x1:x2] * (1.0 - a)).astype(np.uint8)


def move_annotation(annot, M, x, y):
    """Apply rotation matrix M and paste offset (x, y) to object points,
       then compute the enclosing bounding box (x1, y1, x2, y2)."""
    c, pts = annot
    pts_array = np.array([pts], dtype=np.float32)
    transformed_pts = cv2.transform(pts_array, M)[0] + (x, y)

    x1 = transformed_pts[:, 0].min()
    y1 = transformed_pts[:, 1].min()
    x2 = transformed_pts[:, 0].max()
    y2 = transformed_pts[:, 1].max()

    return c, x1, y1, x2, y2


for n in range(1, HOW_MANY + 1):
    canvas = np.full((CANVAS_H, CANVAS_W, 3), 255, np.uint8)  # white background
    labels = []

    # Iterate through each cell in the grid
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            # Randomly decide whether to place an image in this specific grid box or leave it empty
            if random.random() > FILL_PROBABILITY:
                continue

            im, annotations = random.choice(images)
            sh, sw = im.shape[:2]

            # Scale down the image so it safely fits inside the grid cell bounds (with margins)
            max_cell_dim = min(CELL_W, CELL_H) * 0.8
            if sw > max_cell_dim or sh > max_cell_dim:
                scale = max_cell_dim / max(sw, sh)
                im = cv2.resize(im, (int(sw * scale), int(sh * scale)), interpolation=cv2.INTER_AREA)
                # Rescale annotation points accordingly
                scaled_annotations = []
                for c, pts in annotations:
                    new_pts = [[pt[0] * scale, pt[1] * scale] for pt in pts]
                    scaled_annotations.append((c, new_pts))
                annotations = scaled_annotations
                sh, sw = im.shape[:2]

            # Random rotation state
            angle = random.uniform(0, 360)
            rotated_im, M = rotate(im, angle)
            rh, rw = rotated_im.shape[:2]

            # Determine top-left origin of current grid cell
            cell_x = col * CELL_W
            cell_y = row * CELL_H

            # Random state / jitter inside this specific grid box
            min_x = cell_x
            max_x = cell_x + max(0, CELL_W - rw)
            if min_x > max_x: min_x, max_x = max_x, min_x

            min_y = cell_y
            max_y = cell_y + max(0, CELL_H - rh)
            if min_y > max_y: min_y, max_y = max_y, min_y

            x = random.randint(min_x, max_x)
            y = random.randint(min_y, max_y)

            # Paste onto canvas
            paste(canvas, rotated_im, x, y)

            # Compute new annotation for each object on the canvas
            for annot in annotations:
                c, x1, y1, x2, y2 = move_annotation(annot, M, x, y)
                full_area = (x2 - x1) * (y2 - y1)
                if full_area <= 0:
                    continue

                # Clip coordinates to canvas boundaries
                cx1, cy1 = max(x1, 0), max(y1, 0)
                cx2, cy2 = min(x2, CANVAS_W), min(y2, CANVAS_H)

                if cx2 <= cx1 or cy2 <= cy1:
                    continue

                visible_area = (cx2 - cx1) * (cy2 - cy1)
                if visible_area < MIN_VISIBLE * full_area:
                    continue

                # Convert back to YOLO normalized format [class, x_center, y_center, width, height]
                box_w = cx2 - cx1
                box_h = cy2 - cy1
                box_xc = cx1 + box_w / 2.0
                box_yc = cy1 + box_h / 2.0

                labels.append(f"{c} {box_xc / CANVAS_W:.6f} {box_yc / CANVAS_H:.6f} "
                              f"{box_w / CANVAS_W:.6f} {box_h / CANVAS_H:.6f}")

    cv2.imwrite(os.path.join(OUT_DIR, f"canvas_{n}.png"), canvas)
    with open(os.path.join(OUT_DIR, f"canvas_{n}.txt"), "w") as f:
        f.write("\n".join(labels))
    print(f"Saved canvas {n} with {len(labels)} boxes")