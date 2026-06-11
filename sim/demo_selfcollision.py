#!/usr/bin/env python3
"""Self-collision demo on skt_v3_collision.xml: a staged out→up→together
trajectory brings both hands to meet at chest height — they stop in a stable
wrist↔wrist contact instead of passing through each other. Halfway through,
the transparent orange collision-box layer (geom group 3) is revealed.

Route note: the direct path to the "hands together" pose drags the wrists
through the hip zone (AABB wrist boxes are fat), so the trajectory goes
sideways first: OUT (abduct) → UP (raise forward) → MEET (adduct + rotate in).

Usage:
    python make_control_model.py /path/to/skate_teleop/skt_v3      # first
    python make_collision_model.py /path/to/skate_teleop/skt_v3    # second
    python demo_selfcollision.py --model /path/to/skate_teleop/skt_v3 --out demo.mp4
"""
import argparse
import os

import imageio
import mujoco
import numpy as np

SCENE_PATCH = """>
  <visual><global offwidth="1280" offheight="960"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.7 0.7 0.7" specular="0.2 0.2 0.2"/></visual>
  <asset><texture name="grid" type="2d" builtin="checker" rgb1="0.92 0.93 0.95" rgb2="0.82 0.84 0.88" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/></asset>"""

WORLD_PATCH = """<worldbody>
    <geom name="floor" type="plane" pos="0 0 -1.05" size="4 4 0.1" material="grid"/>
    <light pos="1.5 1.5 2" dir="-0.4 -0.4 -1" diffuse="0.6 0.6 0.6"/>"""

REST = {"a0": 0, "a1": 0.2, "a2": 0, "a3": 0.5}
OUT = {"a0": 0, "a1": 0.7, "a2": 0, "a3": 0.4}
UP = {"a0": 0.9, "a1": 0.7, "a2": 0, "a3": 1.3}
MEET = {"a0": 0.9, "a1": 0.15, "a2": 0.55, "a3": 1.35}


def lerp(p1, p2, s):
    return {k: p1.get(k, 0) * (1 - s) + p2.get(k, 0) * s for k in set(p1) | set(p2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to skate_teleop/skt_v3")
    ap.add_argument("--out", default="collision_demo.mp4", help=".mp4 or .gif")
    ap.add_argument("--fps", type=int, default=None)
    ap.add_argument("--size", default=None, help="WxH")
    args = ap.parse_args()

    is_mp4 = args.out.lower().endswith(".mp4")
    fps = args.fps or (30 if is_mp4 else 13)
    size = args.size or ("1280x960" if is_mp4 else "640x480")
    w, h = (int(x) for x in size.lower().split("x"))

    xml = open(os.path.join(args.model, "skt_v3_collision.xml")).read()
    xml = xml.replace(">", SCENE_PATCH, 1).replace("<worldbody>", WORLD_PATCH, 1)
    demo_path = os.path.join(args.model, "skt_v3_colldemo.xml")
    open(demo_path, "w").write(xml)

    m = mujoco.MjModel.from_xml_path(demo_path)
    d = mujoco.MjData(m)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
    aidx = {nm[4:]: i for i, nm in enumerate(names)}

    def set_targets(pose):
        for k, v in pose.items():
            i = int(k[1])
            d.ctrl[aidx[f"{k}_armL_a{8 + i}"]] = v
            d.ctrl[aidx[f"{k}_armR_a{16 + i}"]] = v

    r = mujoco.Renderer(m, h, w)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0, 0, 0.0]
    cam.distance = 2.5
    cam.elevation = -12
    vopt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(vopt)
    vopt.geomgroup[3] = 0  # collision boxes hidden at start

    if is_mp4:
        writer = imageio.get_writer(args.out, fps=fps, codec="libx264",
                                    quality=8, pixelformat="yuv420p")
    else:
        writer = imageio.get_writer(args.out, fps=fps, loop=0)

    spf = int(1 / (fps * m.opt.timestep))
    az = [75.0]

    def run(seconds, fpose, d_az=0.0, boxes=None):
        n = int(seconds * fps)
        for f in range(n):
            s = (f + 1) / n
            sm = 0.5 - 0.5 * np.cos(np.pi * s)
            set_targets(fpose(sm))
            if boxes is not None:
                vopt.geomgroup[3] = boxes
            az[0] += d_az / n
            cam.azimuth = az[0]
            for _ in range(spf):
                mujoco.mj_step(m, d)
            r.update_scene(d, camera=cam, scene_option=vopt)
            writer.append_data(r.render())

    run(0.8, lambda s: REST)
    run(1.4, lambda s: lerp(REST, OUT, s), d_az=8)
    run(1.4, lambda s: lerp(OUT, UP, s), d_az=8)
    run(1.0, lambda s: UP, boxes=1, d_az=6)          # collision layer revealed
    run(2.0, lambda s: lerp(UP, MEET, s), d_az=10)   # hands meet & stop
    run(1.4, lambda s: MEET, d_az=-25)               # hold contact
    run(1.2, lambda s: lerp(MEET, REST, s), boxes=0)

    writer.close()
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
