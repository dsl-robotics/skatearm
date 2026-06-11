#!/usr/bin/env python3
"""Closed-loop demo on the control-ready model: independent left/right arm
trajectories under physics (position servos), rendered to GIF or MP4.

Sequence: settle to rest → left arm reaches while right holds (head pans) →
arms swap → both move to the bimanual work pose. The camera orbits slowly.

Usage:
    python make_control_model.py /path/to/skate_teleop/skt_v3     # first
    python demo_wave.py --model /path/to/skate_teleop/skt_v3 --out demo.mp4
    python demo_wave.py --model /path/to/skate_teleop/skt_v3 --out demo.gif --size 640x480

Output format follows the --out extension: .mp4 (h264, needs imageio-ffmpeg)
or .gif. MP4 at 1280x960/30fps is README-quality; GIF is best kept small.
Headless: MUJOCO_GL=egl or MUJOCO_GL=osmesa.
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

REST = {"a1": 0.2, "a3": 0.5, "a0": 0.0, "a5": 0.0}
REACH = {"a0": 0.9, "a1": 0.25, "a3": 1.3, "a5": 0.3}
WORK = {"a0": 0.4, "a1": 0.3, "a3": 0.9, "a5": 0.2}


def lerp(p1, p2, s):
    return {k: p1.get(k, 0) * (1 - s) + p2.get(k, 0) * s for k in set(p1) | set(p2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to skate_teleop/skt_v3")
    ap.add_argument("--out", default="control_demo.mp4", help=".mp4 or .gif")
    ap.add_argument("--fps", type=int, default=None, help="default: 30 for mp4, 13 for gif")
    ap.add_argument("--size", default=None, help="WxH; default: 1280x960 mp4, 640x480 gif")
    args = ap.parse_args()

    is_mp4 = args.out.lower().endswith(".mp4")
    fps = args.fps or (30 if is_mp4 else 13)
    size = args.size or ("1280x960" if is_mp4 else "640x480")
    w, h = (int(x) for x in size.lower().split("x"))

    xml = open(os.path.join(args.model, "skt_v3_control.xml")).read()
    xml = xml.replace(">", SCENE_PATCH, 1).replace("<worldbody>", WORLD_PATCH, 1)
    demo_path = os.path.join(args.model, "skt_v3_demo.xml")
    open(demo_path, "w").write(xml)

    m = mujoco.MjModel.from_xml_path(demo_path)
    d = mujoco.MjData(m)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
    aidx = {nm[4:]: i for i, nm in enumerate(names)}

    def set_targets(L, R, head=0.0):
        for k, v in L.items():
            d.ctrl[aidx[f"{k}_armL_a{8 + int(k[1])}"]] = v
        for k, v in R.items():
            d.ctrl[aidx[f"{k}_armR_a{16 + int(k[1])}"]] = v
        d.ctrl[aidx["a0_head_a24"]] = head

    r = mujoco.Renderer(m, h, w)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0, 0, -0.1]
    cam.distance = 2.3
    cam.elevation = -10
    az = [70.0]

    if is_mp4:
        writer = imageio.get_writer(args.out, fps=fps, codec="libx264",
                                    quality=8, pixelformat="yuv420p")
    else:
        writer = imageio.get_writer(args.out, fps=fps, loop=0)

    spf = int(1 / (fps * m.opt.timestep))  # sim steps per frame

    def run(seconds, fL, fR, fH, d_az=0.0):
        n = int(seconds * fps)
        for f in range(n):
            s = (f + 1) / n
            sm = 0.5 - 0.5 * np.cos(np.pi * s)  # smooth ease in/out
            set_targets(fL(sm), fR(sm), fH(sm))
            az[0] += d_az / n
            cam.azimuth = az[0]
            for _ in range(spf):
                mujoco.mj_step(m, d)
            r.update_scene(d, camera=cam)
            writer.append_data(r.render())

    run(1.0, lambda s: REST, lambda s: REST, lambda s: 0)
    run(1.8, lambda s: lerp(REST, REACH, s), lambda s: REST, lambda s: 0.5 * s, d_az=10)
    run(1.8, lambda s: lerp(REACH, REST, s), lambda s: lerp(REST, REACH, s), lambda s: 0.5 - 1.0 * s, d_az=15)
    run(1.8, lambda s: lerp(REST, WORK, s), lambda s: lerp(REACH, WORK, s), lambda s: -0.5 + 0.5 * s, d_az=15)
    run(1.2, lambda s: WORK, lambda s: WORK, lambda s: 0, d_az=-40)

    writer.close()
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
