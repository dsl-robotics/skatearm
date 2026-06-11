#!/usr/bin/env python3
"""Load the official Skate skt_v3 MuJoCo model, set a bimanual pose and render PNGs.

The converted MJCF in Rbotic/skate_teleop has no floor, lights or offscreen
framebuffer config, so we patch it on the fly (the original file is untouched).

Usage:
    python render_skate.py --model /path/to/skate_teleop/skt_v3 [--out renders]

Headless Linux: MUJOCO_GL=egl (GPU) or MUJOCO_GL=osmesa (CPU, needs libosmesa6).
"""
import argparse
import os

import imageio
import mujoco

SCENE_PATCH = """>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.7 0.7 0.7" specular="0.2 0.2 0.2"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.92 0.93 0.95" rgb2="0.82 0.84 0.88" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>"""

WORLD_PATCH = """<worldbody>
    <geom name="floor" type="plane" size="4 4 0.1" material="grid"/>
    <light pos="1.5 1.5 3" dir="-0.4 -0.4 -1" diffuse="0.6 0.6 0.6"/>"""

# Poses use the same values for both mirrored arm chains, and MUST respect the
# joint ranges from the URDF (e.g. a1 abduction: -0.79..2.36, a3 elbow: 0..2.64 —
# the elbow cannot bend backwards). set_joint() clamps to the range as a guard.
RELAXED_POSE = {"a1": 0.2, "a3": 0.5}                       # arms at sides, light elbow bend
WORK_POSE = {"a0": 0.4, "a1": 0.3, "a3": 0.9, "a5": 0.2}    # forearms forward, ready to manipulate
HEAD_TILT = 0.25
BASE_LIFT = 0.95  # raise the free joint so the wheels sit on the floor


def build_model(model_dir: str) -> mujoco.MjModel:
    src = os.path.join(model_dir, "skt_v3_converted.xml")
    xml = open(src).read()
    xml = xml.replace(">", SCENE_PATCH, 1)
    xml = xml.replace("<worldbody>", WORLD_PATCH, 1)
    patched = os.path.join(model_dir, "skt_v3_render.xml")
    open(patched, "w").write(xml)
    return mujoco.MjModel.from_xml_path(patched)


def set_joint(m, d, name, val):
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    if j == -1:
        raise KeyError(f"joint not found: {name}")
    lo, hi = m.jnt_range[j]
    clamped = min(max(val, lo), hi)
    if clamped != val:
        print(f"warning: {name}={val} outside range [{lo:.2f}, {hi:.2f}], clamped")
    d.qpos[m.jnt_qposadr[j]] = clamped


def set_pose(m, d, arm_pose):
    d.qpos[:] = 0
    d.qpos[2] = BASE_LIFT
    for k, v in arm_pose.items():
        idx = int(k[1])
        set_joint(m, d, f"{k}_armL_a{8 + idx}", v)
        set_joint(m, d, f"{k}_armR_a{16 + idx}", v)
    set_joint(m, d, "a1_head_a25", HEAD_TILT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to skate_teleop/skt_v3")
    ap.add_argument("--out", default="renders")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    m = build_model(args.model)
    d = mujoco.MjData(m)

    r = mujoco.Renderer(m, 960, 1280)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0, 0, 0.8]
    cam.distance = 2.4
    cam.elevation = -8

    shots = [
        (RELAXED_POSE, 90, "front.png"),
        (WORK_POSE, 55, "three_quarter.png"),
        (RELAXED_POSE, 180, "side.png"),
    ]
    for pose, az, name in shots:
        set_pose(m, d, pose)
        cam.azimuth = az
        mujoco.mj_forward(m, d)
        r.update_scene(d, camera=cam)
        path = os.path.join(args.out, name)
        imageio.imwrite(path, r.render())
        print("saved", path)

    print(f"model: nq={m.nq} njnt={m.njnt} nu={m.nu} (nu=0 -> no actuators yet)")


if __name__ == "__main__":
    main()
