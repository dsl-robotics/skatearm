"""Tiny vision pipeline for the workspace camera: find the magenta target in a
rendered frame and back-project its centroid to a known world plane.

Pure numpy. The camera model matches MuJoCo's: the camera looks down its local
-z axis, +x is image-right, +y is image-up; cam_mat columns are the camera axes
in world. Validated against simulator ground truth to ~2 mm (see
skate_capture/validate_vision.py).
"""
from __future__ import annotations

import numpy as np

# magenta target (robot meshes are red/green/blue/grey — magenta is unique)
MIN_PIXELS = 30


def intrinsics(fovy_deg, W, H):
    f = (H / 2.0) / np.tan(np.radians(fovy_deg) / 2.0)
    return f, W / 2.0, H / 2.0


def project(P, cam_pos, cam_mat, f, cx, cy):
    pc = cam_mat.T @ (np.asarray(P, float) - cam_pos)
    xn, yn = pc[0] / (-pc[2]), pc[1] / (-pc[2])
    return cx + f * xn, cy - f * yn, pc[2]


def backproject(u, v, z0, cam_pos, cam_mat, f, cx, cy):
    xn, yn = (u - cx) / f, (cy - v) / f
    dw = cam_mat @ np.array([xn, yn, -1.0])
    t = (z0 - cam_pos[2]) / dw[2]
    return cam_pos + t * dw


def depth_cloud(depth, rgb, cam_pos, cam_mat, fovy_deg, stride=5, zmax=0.75):
    """Back-project a rendered depth image to a world point cloud, coloured by
    the matching RGB pixel. ``depth`` is MuJoCo's camera-frame depth (metres
    along -z). Returns an (M, 6) array of [x, y, z, r, g, b] (rgb in [0, 1]),
    downsampled by ``stride`` and clipped to ``zmax`` metres (drops the far
    background / floor). Same camera model as project/backproject."""
    H, W = depth.shape[:2]
    f, cx, cy = intrinsics(fovy_deg, W, H)
    cam_pos = np.asarray(cam_pos, float).reshape(3)
    cam_mat = np.asarray(cam_mat, float).reshape(3, 3)
    vv, uu = np.mgrid[0:H:stride, 0:W:stride]
    uu = uu.ravel(); vv = vv.ravel()
    D = depth[vv, uu]
    keep = (D > 0.05) & (D < zmax)
    uu, vv, D = uu[keep], vv[keep], D[keep]
    xn = (uu - cx) / f
    yn = (cy - vv) / f
    pc = np.stack([xn * D, yn * D, -D], axis=1)          # camera frame
    P = cam_pos.reshape(1, 3) + pc @ cam_mat.T            # world
    C = rgb[vv, uu, :3].astype(float) / 255.0            # per-point colour
    return np.hstack([P, C])


def find_magenta(img):
    """Centroid (u, v, n_pixels) of the magenta blob, or None."""
    R = img[:, :, 0].astype(int)
    G = img[:, :, 1].astype(int)
    B = img[:, :, 2].astype(int)
    mask = (R > 110) & (B > 100) & (R - G > 60) & (B - G > 40)
    ys, xs = np.nonzero(mask)
    if len(xs) < MIN_PIXELS:
        return None
    return float(xs.mean()), float(ys.mean()), int(len(xs))


def detect(img, cam_pos, cam_mat, fovy_deg, grasp_z):
    """img: HxWx3 RGB; cam_pos (3,), cam_mat (3,3) from MuJoCo; returns a dict."""
    H, W = img.shape[:2]
    f, cx, cy = intrinsics(fovy_deg, W, H)
    hit = find_magenta(img)
    if hit is None:
        return {"found": False}
    u, v, n = hit
    cam_pos = np.asarray(cam_pos, float).reshape(3)
    cam_mat = np.asarray(cam_mat, float).reshape(3, 3)
    P = backproject(u, v, grasp_z, cam_pos, cam_mat, f, cx, cy)
    return {"found": True,
            "world": [float(P[0]), float(P[1]), float(P[2])],
            "world_mm": [round(float(P[0]) * 1000, 1),
                         round(float(P[1]) * 1000, 1),
                         round(float(P[2]) * 1000, 1)],
            "pixel": [round(u, 1), round(v, 1)], "n": n}
