"""Grasp synthesis on the work-camera point cloud.

The colour+plane pick (``vision.detect``) back-projects one magenta pixel to a
fixed grasp height -- it assumes the object's colour and its height. This module
is the geometric step the v0.7.10 point cloud unlocks: take the back-projected
cloud, remove the support surface (the table), cluster what's left into objects,
and synthesise a top-down parallel-jaw grasp from the cluster's OWN geometry --
no colour prior, a measured grasp height, an estimated footprint, and a
gripper-width feasibility check.

Pure numpy (no sklearn / scipy). The plane fit is a small RANSAC; clustering is
voxel connected-components. Validated against the simulator's ground-truth cube
pose (test/test_grasp.py and the live ``/api/grasp`` cross-check).

Frames: world metres in, world metres out. The cockpit IK is position-only, so
the synthesised yaw / footprint are reported for the twin overlay and for the
day the wrist can be oriented; execution (``/api/smart_pick``) uses the measured
centre + height, top-down.
"""
from __future__ import annotations

from collections import defaultdict, deque

import numpy as np


def segment_plane(xyz, tol=0.008, iters=120, seed=0):
    """RANSAC the dominant plane (the table top). Returns ``((normal, d),
    above)`` where the plane is ``normal . p + d = 0`` with ``|normal| = 1`` and
    the normal oriented up (+z), and ``above`` is a boolean mask of points on
    the +normal side beyond ``tol`` (object candidates). Falls back to a level
    plane at the median height when the cloud is too small to fit."""
    P = np.asarray(xyz, float)
    n = len(P)
    if n < 8:
        d = -float(np.median(P[:, 2])) if n else 0.0
        return (np.array([0.0, 0.0, 1.0]), d), np.zeros(n, bool)
    rng = np.random.default_rng(seed)
    best_in, best = -1, None
    for _ in range(iters):
        a, b, c = P[rng.choice(n, 3, replace=False)]
        nrm = np.cross(b - a, c - a)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        d = -float(nrm @ a)
        ninl = int((np.abs(P @ nrm + d) < tol).sum())
        if ninl > best_in:
            best_in, best = ninl, (nrm, d)
    nrm, d = best
    if nrm[2] < 0:                      # orient the normal upward
        nrm, d = -nrm, -d
    above = (P @ nrm + d) > tol
    return (nrm, float(d)), above


def voxel_clusters(P, eps=0.02):
    """Connected-components of points by voxel adjacency (26-neighbourhood at
    cell size ``eps``). Returns a list of index arrays into ``P``, largest
    cluster first. Pure-numpy / dict BFS -- no scipy."""
    P = np.asarray(P, float)
    if len(P) == 0:
        return []
    keys = np.floor(P / eps).astype(np.int64)
    vox = defaultdict(list)
    for idx, k in enumerate(map(tuple, keys)):
        vox[k].append(idx)
    neigh = [(dx, dy, dz) for dx in (-1, 0, 1)
             for dy in (-1, 0, 1) for dz in (-1, 0, 1)]
    seen, comps = set(), []
    for start in list(vox):
        if start in seen:
            continue
        seen.add(start)
        q, members = deque([start]), []
        while q:
            kx, ky, kz = q.popleft()
            members.extend(vox[(kx, ky, kz)])
            for dx, dy, dz in neigh:
                nk = (kx + dx, ky + dy, kz + dz)
                if nk in vox and nk not in seen:
                    seen.add(nk)
                    q.append(nk)
        comps.append(np.array(members, dtype=int))
    comps.sort(key=len, reverse=True)
    return comps


def _footprint(P):
    """2-D PCA of a cluster's horizontal footprint. Returns
    ``(major_len, minor_len, major_ax2, minor_ax2)`` (lengths in m)."""
    xy = P[:, :2] - P[:, :2].mean(axis=0)
    _, evecs = np.linalg.eigh(xy.T @ xy / max(len(xy), 1))
    minor_ax, major_ax = evecs[:, 0], evecs[:, 1]    # eigh: ascending
    return (float(np.ptp(xy @ major_ax)), float(np.ptp(xy @ minor_ax)),
            major_ax, minor_ax)


def synthesize_grasp(cluster_xyz, plane, max_width=0.09, clearance=0.006):
    """Top-down parallel-jaw grasp from one object cluster.

    A 2-D PCA of the cluster's horizontal footprint gives the major / minor
    axes; the jaws close ACROSS the minor axis (the narrower span fits the
    gripper). Grasp height is the midpoint between the support plane and the
    cluster's top -- a MEASURED height, not the hard-coded ``GRASP_Z``. Returns
    a dict of world metres + rounded-mm fields for the twin overlay."""
    P = np.asarray(cluster_xyz, float)
    nrm, d = np.asarray(plane[0], float), float(plane[1])
    cx, cy = float(P[:, 0].mean()), float(P[:, 1].mean())
    z_plane = -(nrm[0] * cx + nrm[1] * cy + d) / nrm[2]
    top_z = float(P[:, 2].max())
    grasp_z = 0.5 * (z_plane + top_z)
    length, width, major_ax, minor_ax = _footprint(P)
    yaw = float(np.degrees(np.arctan2(minor_ax[1], minor_ax[0])))
    c = np.array([cx, cy, grasp_z])
    ma = np.array([major_ax[0], major_ax[1], 0.0])
    mi = np.array([minor_ax[0], minor_ax[1], 0.0])
    foot = [c + sx * (length / 2) * ma + sy * (width / 2) * mi
            for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    jaws = [c + s * (width / 2 + clearance) * mi for s in (1, -1)]
    mm = lambda v: [round(float(x) * 1000, 1) for x in v]
    return {
        "center": [round(float(v), 5) for v in c], "center_mm": mm(c),
        "grasp_z_mm": round(grasp_z * 1000, 1), "top_mm": round(top_z * 1000, 1),
        "plane_z_mm": round(z_plane * 1000, 1), "yaw_deg": round(yaw, 1),
        "width_mm": round(width * 1000, 1), "length_mm": round(length * 1000, 1),
        "feasible": bool(width <= max_width), "max_width_mm": round(max_width * 1000, 1),
        "footprint": [mm(p) for p in foot], "jaws": [mm(p) for p in jaws],
        "approach": [0.0, 0.0, -1.0],
    }


def plan_grasp(cloud, max_width=0.09, plane_tol=0.008, cluster_eps=0.02,
               min_cluster=18, workspace=None,
               max_obj=0.18, flat_max=0.02, elong_max=0.72):
    """Full pipeline: cloud -> remove the table -> cluster -> pick the object
    -> synthesise a grasp.

    ``cloud`` is an (M, 6) ``[x, y, z, r, g, b]`` array (or list) from
    ``vision.depth_cloud`` / ``camera.cloud``. ``workspace`` is an optional
    ``(x0, x1, y0, y1)`` AABB that keeps only above-plane points over the work
    surface.

    Object selection is geometric, NOT "largest cluster": the cloud also holds
    the robot's own legs / arms, which out-number a small part. A graspable
    object resting on the table shows the overhead camera a FLAT, COMPACT top
    face, whereas a limb is elongated with a large height spread. A cluster
    qualifies when its top-height spread <= ``flat_max``, its footprint
    elongation <= ``elong_max`` and it fits ``max_obj``; among those the
    best-sampled (most points) wins. Returns ``synthesize_grasp`` + meta, or
    ``{"found": False, "reason": ...}``."""
    A = np.asarray(cloud, float)
    if A.ndim != 2 or A.shape[0] < min_cluster or A.shape[1] < 3:
        return {"found": False, "reason": "cloud too small"}
    xyz = A[:, :3]
    plane, above = segment_plane(xyz, tol=plane_tol)
    nrm, d = plane
    pts = xyz[above]
    if workspace is not None and len(pts):
        x0, x1, y0, y1 = workspace
        pts = pts[(pts[:, 0] >= x0) & (pts[:, 0] <= x1) &
                  (pts[:, 1] >= y0) & (pts[:, 1] <= y1)]
    if len(pts) < min_cluster:
        return {"found": False, "reason": "nothing above the surface"}
    comps = [c for c in voxel_clusters(pts, cluster_eps) if len(c) >= min_cluster]
    if not comps:
        return {"found": False, "reason": "no object cluster"}
    cand = []
    for c in comps:
        P = pts[c]
        major, minor, _, _ = _footprint(P)
        flat = float(np.ptp(P @ nrm + d))
        elong = 1.0 - minor / max(major, 1e-9)
        if major <= max_obj and flat <= flat_max and elong <= elong_max:
            cand.append((c, len(c)))
    if not cand:
        return {"found": False,
                "reason": "no flat compact object (only limbs / clutter)"}
    best = max(cand, key=lambda t: t[1])[0]
    g = synthesize_grasp(pts[best], plane, max_width=max_width)
    g["found"] = True
    g["n"] = int(len(best))
    g["n_clusters"] = len(comps)
    g["n_candidates"] = len(cand)
    return g
