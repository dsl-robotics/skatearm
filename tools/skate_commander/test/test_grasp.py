"""Grasp synthesis on the work-camera point cloud (v0.7.11) — headless.

Pure-numpy: synthesise clouds of a table plane + box top face(s) at known
poses and check the pipeline recovers the grasp (centre, height, footprint,
yaw, feasibility). No mujoco needed. A live cross-check against the rendered
sim scene runs separately via /api/grasp.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_commander import grasp, detect, vision  # noqa: E402

# the generated scene: table top at z=-0.08, magenta cube top at z=-0.03
WS = (-0.14, 0.24, 0.18, 0.50)             # work-surface AABB (x0,x1,y0,y1)


def _table(n=2500, z=-0.08, cx=0.05, cy=0.34, hx=0.18, hy=0.15, seed=1):
    rng = np.random.default_rng(seed)
    x = rng.uniform(cx - hx, cx + hx, n)
    y = rng.uniform(cy - hy, cy + hy, n)
    zz = np.full(n, z) + rng.normal(0, 0.0008, n)
    rgb = np.tile([0.42, 0.32, 0.24], (n, 1))
    return np.column_stack([x, y, zz, rgb])


def _box_top(center, half, yaw=0.0, n=500, seed=2, rgb=(0.9, 0.12, 0.86)):
    """Points on a box's TOP face (what a top-down camera sees)."""
    rng = np.random.default_rng(seed)
    cx, cy, topz = center
    hx, hy = half
    u = rng.uniform(-hx, hx, n)
    v = rng.uniform(-hy, hy, n)
    cy_, sy_ = np.cos(yaw), np.sin(yaw)
    x = cx + u * cy_ - v * sy_
    y = cy + u * sy_ + v * cy_
    z = np.full(n, topz) + rng.normal(0, 0.0005, n)
    return np.column_stack([x, y, z, np.tile(rgb, (n, 1))])


def _limb(center, length=0.12, width=0.02, n=350, seed=7):
    """An elongated, height-climbing structure (a robot leg/arm over the table)
    — NOT a flat resting object. Mimics the clusters the live scene shows."""
    rng = np.random.default_rng(seed)
    cx, cy = center
    u = rng.uniform(-length / 2, length / 2, n)
    v = rng.uniform(-width / 2, width / 2, n)
    x, y = cx + u, cy + v
    z = -0.08 + 0.012 + (u + length / 2) / length * 0.04 + rng.normal(0, 0.001, n)
    return np.column_stack([x, y, z, np.tile([0.5, 0.5, 0.55], (n, 1))])


def test_segment_plane_keeps_object():
    cloud = np.vstack([_table(), _box_top((0.13, 0.35, -0.03), (0.025, 0.025))])
    (nrm, d), above = grasp.segment_plane(cloud[:, :3])
    assert abs(nrm[2]) > 0.99, f"table normal should be ~vertical: {nrm}"
    z_plane = -d / nrm[2]
    assert abs(z_plane - (-0.08)) < 0.01, f"plane z {z_plane:.3f} != -0.08"
    kept = int(above.sum())
    assert 400 <= kept <= 650, f"expected ~500 object points, got {kept}"
    print(f"PASS segment_plane: plane z={z_plane*1000:.0f}mm, {kept} above-points")


def test_grasp_center_recovers_truth():
    cloud = np.vstack([_table(), _box_top((0.13, 0.35, -0.03), (0.025, 0.025))])
    g = grasp.plan_grasp(cloud, workspace=WS)
    assert g["found"], g
    cx, cy, cz = g["center_mm"]
    assert abs(cx - 130) < 5 and abs(cy - 350) < 5, f"xy off: {g['center_mm']}"
    assert abs(cz - (-55)) < 6, f"grasp z {cz}mm != -55 (mid table/top)"
    assert abs(g["plane_z_mm"] - (-80)) < 10 and abs(g["top_mm"] - (-30)) < 8
    print(f"PASS grasp center: {g['center_mm']} mm (truth 130/350/-55)")


def test_grasp_dims_and_yaw():
    yaw = np.radians(30.0)
    cloud = np.vstack([_table(),
                       _box_top((0.10, 0.34, -0.03), (0.045, 0.018), yaw=yaw, n=700)])
    g = grasp.plan_grasp(cloud, workspace=WS)
    assert g["found"], g
    assert abs(g["width_mm"] - 36) < 6, f"width {g['width_mm']} != ~36"
    assert abs(g["length_mm"] - 90) < 8, f"length {g['length_mm']} != ~90"
    # the jaws close across the box's SHORT axis: recovered minor ~ true short
    yr = np.radians(g["yaw_deg"])
    rec = np.array([np.cos(yr), np.sin(yr)])
    true_short = np.array([-np.sin(yaw), np.cos(yaw)])
    assert abs(float(rec @ true_short)) > 0.96, f"yaw {g['yaw_deg']} off short axis"
    print(f"PASS dims+yaw: w={g['width_mm']} l={g['length_mm']} yaw={g['yaw_deg']}")


def test_infeasible_when_too_wide():
    # a 120x110mm block: minor span 110mm > 90mm max gripper opening
    cloud = np.vstack([_table(),
                       _box_top((0.13, 0.35, -0.03), (0.06, 0.055), n=900)])
    g = grasp.plan_grasp(cloud, workspace=WS, max_width=0.09)
    assert g["found"], g
    assert not g["feasible"], f"width {g['width_mm']}mm should exceed the gripper"
    print(f"PASS infeasible: width {g['width_mm']}mm > {g['max_width_mm']}mm -> flagged")


def test_no_object_found_false():
    g = grasp.plan_grasp(_table(), workspace=WS)
    assert not g["found"], g
    print(f"PASS no object: found=False ({g.get('reason')})")


def test_workspace_filter_excludes_outside():
    # box sitting OFF the work surface (a stray arm/body return) is ignored
    cloud = np.vstack([_table(), _box_top((0.45, 0.34, -0.03), (0.03, 0.03))])
    assert not grasp.plan_grasp(cloud, workspace=WS)["found"]
    # same box, no AABB -> it IS clustered
    assert grasp.plan_grasp(cloud)["found"]
    print("PASS workspace filter: off-surface cluster dropped by the AABB")


def test_picks_largest_of_two():
    cloud = np.vstack([_table(),
                       _box_top((0.13, 0.35, -0.03), (0.025, 0.025), n=500, seed=3),
                       _box_top((-0.05, 0.28, -0.05), (0.015, 0.015), n=90, seed=4)])
    g = grasp.plan_grasp(cloud, workspace=WS)
    assert g["found"] and g["n_clusters"] == 2, g
    assert abs(g["center_mm"][0] - 130) < 6 and abs(g["center_mm"][1] - 350) < 6
    print(f"PASS two objects: {g['n_clusters']} clusters, picked the larger at "
          f"{g['center_mm'][:2]} mm")


def test_selects_flat_object_over_limb():
    # the live wrinkle: the cloud holds the robot's legs (big elongated, height-
    # climbing clusters) plus the cube (a smaller flat square). Naive "largest"
    # picks a leg; geometric selection must pick the flat compact cube.
    cloud = np.vstack([
        _table(),
        _limb((0.10, 0.20), n=350, seed=7),
        _limb((-0.09, 0.20), n=300, seed=8),
        _box_top((0.13, 0.35, -0.03), (0.025, 0.025), n=180),
    ])
    g = grasp.plan_grasp(cloud, workspace=WS)
    assert g["found"], g
    assert g["n_clusters"] >= 3 and g["n_candidates"] == 1, g
    assert abs(g["center_mm"][0] - 130) < 6 and abs(g["center_mm"][1] - 350) < 6, g
    print(f"PASS selects object over limbs: {g['n_clusters']} clusters, "
          f"{g['n_candidates']} graspable, picked {g['center_mm'][:2]} mm")


def test_plan_grasps_multi_object():
    # two objects on the table (magenta cube + cyan box) + the robot legs
    cloud = np.vstack([
        _table(),
        _limb((0.10, 0.20), n=320, seed=7),                       # robot leg
        _box_top((0.13, 0.35, -0.03), (0.025, 0.025), n=420, seed=3),
        _box_top((0.00, 0.30, -0.04), (0.02, 0.02), n=240, seed=5,
                 rgb=(0.12, 0.75, 0.85)),
    ])
    r = grasp.plan_grasps(cloud, workspace=WS)
    assert r["found"] and len(r["objects"]) == 2, r          # leg excluded
    assert r["objects"][0]["n"] >= r["objects"][1]["n"], "not ranked by points"
    assert all("mean_rgb" in o and "id" in o for o in r["objects"])
    detect.label_objects(r["objects"])
    cols = {o["colour"] for o in r["objects"]}
    assert cols == {"magenta", "cyan"}, cols
    print(f"PASS plan_grasps multi: 2 objects ({cols}), leg dropped, ranked")


def test_depth_cloud_mask_drops_points():
    # the PCL self-filter passes a keep-mask to vision.depth_cloud so the
    # robot's own pixels are dropped; here a left-half mask keeps ~half the cloud
    H, W = 48, 64
    depth = np.full((H, W), 0.4)               # all in range (0.05..zmax)
    rgb = np.zeros((H, W, 3), np.uint8)
    cam_pos, cam_mat = np.array([0.0, 0.0, 0.5]), np.eye(3)
    full = vision.depth_cloud(depth, rgb, cam_pos, cam_mat, 55, stride=2)
    mask = np.zeros((H, W), bool); mask[:, : W // 2] = True
    half = vision.depth_cloud(depth, rgb, cam_pos, cam_mat, 55, stride=2, mask=mask)
    assert len(full) > 0 and 0 < len(half) < len(full)
    assert abs(len(half) - len(full) / 2) <= 0.12 * len(full)
    print(f"PASS depth_cloud mask: {len(full)} -> {len(half)} pts (~half kept)")


if __name__ == "__main__":
    test_segment_plane_keeps_object()
    test_grasp_center_recovers_truth()
    test_grasp_dims_and_yaw()
    test_infeasible_when_too_wide()
    test_no_object_found_false()
    test_workspace_filter_excludes_outside()
    test_picks_largest_of_two()
    test_selects_flat_object_over_limb()
    test_plan_grasps_multi_object()
    test_depth_cloud_mask_drops_points()
    print("GRASP OK")
