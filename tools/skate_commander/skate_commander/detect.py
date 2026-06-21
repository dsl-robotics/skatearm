"""Pluggable object detector for smarter pick.

The built-in detector labels each graspable cloud cluster by colour (mean RGB ->
nearest named hue) and coarse shape (footprint aspect -> cube / box / bar), so a
multi-object scene can be picked BY NAME -- no per-object colour threshold, no
learned weights. An optional YOLO backend (ultralytics) slots in behind the same
interface when ``SKATE_YOLO`` is set and ultralytics + a model are available: it
overlays class names onto the matched objects and falls back to the built-in
labels otherwise -- the deterministic-core + optional-heavy-backend pattern of
``nl.py``. Pure numpy for the built-in path; no torch unless YOLO is opted in.
"""
from __future__ import annotations

import importlib.util
import os

import numpy as np

from . import vision

# rough hue anchors (RGB, 0..1) for nearest-colour naming -- not a classifier
PALETTE = {
    "magenta": (0.85, 0.15, 0.80), "red": (0.80, 0.16, 0.16),
    "orange": (0.90, 0.55, 0.15), "yellow": (0.86, 0.80, 0.22),
    "green": (0.20, 0.70, 0.32), "cyan": (0.15, 0.75, 0.85),
    "blue": (0.20, 0.35, 0.82), "purple": (0.52, 0.26, 0.75),
    "white": (0.88, 0.88, 0.88), "grey": (0.50, 0.50, 0.50),
    "black": (0.08, 0.08, 0.09),
}


def colour_name(rgb):
    """Nearest named hue to an RGB triple (0..1)."""
    rgb = np.asarray(rgb, float)
    return min(PALETTE, key=lambda k: float(np.linalg.norm(rgb - PALETTE[k])))


def shape_name(g):
    """Coarse shape from the grasp footprint (mm)."""
    L, W = float(g.get("length_mm", 0.0)), float(g.get("width_mm", 0.0))
    if L < 1.0:
        return "point"
    ar = W / max(L, 1e-9)
    return "cube" if ar > 0.78 else "box" if ar > 0.45 else "bar"


def label_objects(objects):
    """Add ``colour`` / ``shape`` / ``label`` to each grasp object in place
    (built-in, deterministic). Returns the list."""
    for i, g in enumerate(objects):
        g.setdefault("id", i)
        g["colour"] = colour_name(g.get("mean_rgb", (0.5, 0.5, 0.5)))
        g["shape"] = shape_name(g)
        g["label"] = f"{g['colour']} {g['shape']}"
        g["detector"] = "builtin"
    return objects


def yolo_available():
    """True only when explicitly opted in AND ultralytics is importable."""
    return bool(os.environ.get("SKATE_YOLO")) and \
        importlib.util.find_spec("ultralytics") is not None


_YOLO = None


def _yolo_model():
    global _YOLO
    if _YOLO is None:
        from ultralytics import YOLO              # heavy; only on opt-in
        _YOLO = YOLO(os.environ.get("SKATE_YOLO_MODEL", "yolov8n.pt"))
    return _YOLO


def _yolo_overlay(objects, rgb_image, cam):
    """Run YOLO on the work-camera frame and overlay class names onto objects
    whose projected centre falls inside a detection box. Built-in label stays
    when nothing matches."""
    model = _yolo_model()
    H, W = rgb_image.shape[:2]
    f, cx, cy = vision.intrinsics(cam["fovy"], W, H)
    pos = np.asarray(cam["pos"], float)
    mat = np.asarray(cam["mat"], float).reshape(3, 3)
    res = model.predict(rgb_image, verbose=False)[0]
    boxes = [(*(float(v) for v in b.xyxy[0]), res.names[int(b.cls[0])],
              float(b.conf[0])) for b in res.boxes]
    for g in objects:
        u, v, _ = vision.project(np.asarray(g["center"], float),
                                 pos, mat, f, cx, cy)
        for x1, y1, x2, y2, name, conf in boxes:
            if x1 <= u <= x2 and y1 <= v <= y2:
                g["label"], g["yolo_conf"], g["detector"] = name, round(conf, 2), "yolo"
                break
    return objects


def detect(objects, rgb_image=None, cam=None):
    """Label graspable objects. Built-in colour/shape always; if ``SKATE_YOLO``
    is opted in and ultralytics + a frame are available, overlay YOLO class
    names on top (per-object fallback to the built-in label). Returns the
    labeled list."""
    label_objects(objects)
    if rgb_image is not None and cam is not None and yolo_available():
        try:
            _yolo_overlay(objects, np.asarray(rgb_image), cam)
        except Exception:
            pass                                  # keep built-in labels
    return objects


def pick_target(objects, target=None):
    """Choose one object: ``target`` may be an id/index (int or digit string),
    a colour / label substring (case-insensitive), or None for the best
    (first = most-sampled). Returns the object dict or None."""
    if not objects:
        return None
    if target is None or target == "":
        return objects[0]
    s = str(target).strip().lower()
    if s.lstrip("-").isdigit():
        return next((g for g in objects if g.get("id") == int(s)), None)
    return next((g for g in objects
                 if s in str(g.get("label", "")).lower()
                 or s == str(g.get("colour", "")).lower()), None)
