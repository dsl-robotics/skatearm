"""Image-based visual servoing (IBVS) for the workspace camera.

The open-loop pick back-projects the target's pixel through the camera model
ONCE and moves there; if the camera is mis-calibrated, the move misses. IBVS
closes the loop instead: it drives the end-effector so that, in the IMAGE, the
gripper feature lands on the target feature. Aligning two points in the same
image aligns them in the world regardless of camera-calibration error — the
classic IBVS robustness result.

The controller only uses its BELIEVED camera model to build the image Jacobian
(how a pixel moves when the EE moves in the table plane). The error it nulls is
the OBSERVED pixel error (target pixel from the camera; gripper pixel from a
marker / the wrist pose as the camera sees it). A wrong believed model slows
convergence but does not bias the converged pose — that is what makes it robust.

Pure numpy; reuses vision.project for the camera model.
"""
from __future__ import annotations

import numpy as np

from . import vision


class ImageServo:
    """Eye-to-hand point IBVS in the table plane (fixed overhead camera)."""

    def __init__(self, cam_pos, cam_mat, fovy_deg, W, H,
                 gain=0.7, max_step=0.03):
        self.cam_pos = np.asarray(cam_pos, float).reshape(3)
        self.cam_mat = np.asarray(cam_mat, float).reshape(3, 3)
        self.f, self.cx, self.cy = vision.intrinsics(fovy_deg, W, H)
        self.gain = float(gain)
        self.max_step = float(max_step)     # m, per-iteration cap

    def pixel_of(self, P):
        """Where the believed camera thinks world point P projects."""
        u, v, _ = vision.project(P, self.cam_pos, self.cam_mat,
                                 self.f, self.cx, self.cy)
        return np.array([u, v])

    def image_jacobian(self, ee_world, eps=1e-3):
        """2x2 d(pixel)/d(EE x,y) at ee_world, from the BELIEVED camera."""
        ee_world = np.asarray(ee_world, float)
        J = np.zeros((2, 2))
        for k in range(2):                  # world x, then y
            dp = np.zeros(3)
            dp[k] = eps
            J[:, k] = (self.pixel_of(ee_world + dp)
                       - self.pixel_of(ee_world - dp)) / (2 * eps)
        return J

    def step(self, target_pixel, ee_pixel, ee_world):
        """One servo increment: the world (x, y) delta that reduces the
        OBSERVED image error (target_pixel - ee_pixel). The caller owns z
        (held at / lowered toward the grasp height). Returns (dxy[2], err_px).
        """
        e = np.asarray(target_pixel, float) - np.asarray(ee_pixel, float)
        J = self.image_jacobian(ee_world)
        try:
            dxy = self.gain * np.linalg.solve(J, e)
        except np.linalg.LinAlgError:
            dxy = np.zeros(2)
        n = float(np.linalg.norm(dxy))
        if n > self.max_step:
            dxy *= self.max_step / n
        return dxy, float(np.linalg.norm(e))
