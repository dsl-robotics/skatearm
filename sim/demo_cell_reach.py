#!/usr/bin/env python3
"""Phase 1 demo: bimanual closed-loop REACH in the work-cell scene.

Sequence (all under physics, position servos only):
  1. fold elbows  — hands rise close to the chest (joint-space, eased)
  2. raise arms   — hands come over the table FROM ABOVE (joint-space, eased)
  3. IK hover     — both EEs servo above the base part / peg
  4. IK descend   — to working height over the parts
  5. IK lift      — retreat upward, then hold

Why the two joint-space stages: the hanging rest pose is below the tabletop;
any straight Cartesian path from it crosses the table front edge (the arm is
long — the upswing reaches the table plane while the hand is still low). The
fold-then-raise route keeps the hands close to the chest until they are above
table height. See sim/README.md "Motion-quality lessons".

Usage:
    python make_control_model.py /path/to/skate_teleop/skt_v3
    python make_collision_model.py /path/to/skate_teleop/skt_v3
    python make_cell_scene.py /path/to/skate_teleop/skt_v3
    python demo_cell_reach.py --model /path/to/skate_teleop/skt_v3 --out demo.mp4
"""
import argparse
import os
import sys

import imageio
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from primitives import reach, hold, move_joints  # noqa: E402

BASE_XY = (-0.12, 0.44)   # parts moved closer to center for the assembly demo
PEG_XY = (0.12, 0.44)
HOVER_Z, WORK_Z, LIFT_Z = 0.24, 0.13, 0.30
TOTAL_S = 15.9  # nominal timeline for the camera ease


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to skate_teleop/skt_v3")
    ap.add_argument("--out", default="cell_reach_demo.mp4", help=".mp4 or .gif")
    ap.add_argument("--fps", type=int, default=None)
    ap.add_argument("--size", default=None, help="WxH")
    args = ap.parse_args()

    is_mp4 = args.out.lower().endswith(".mp4")
    fps = args.fps or (25 if is_mp4 else 12)
    size = args.size or ("960x720" if is_mp4 else "480x360")
    w, h = (int(x) for x in size.lower().split("x"))

    m = mujoco.MjModel.from_xml_path(os.path.join(args.model, "skt_v3_cell.xml"))
    d = mujoco.MjData(m)
    for _ in range(500):
        mujoco.mj_step(m, d)  # settle parts on the table

    r = mujoco.Renderer(m, h, w)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0, 0.30, 0.15]
    cam.distance = 1.9
    cam.elevation = -20

    if is_mp4:
        writer = imageio.get_writer(args.out, fps=fps, codec="libx264",
                                    quality=7, pixelformat="yuv420p")
    else:
        writer = imageio.get_writer(args.out, fps=fps, loop=0)

    state = {"n": 0, "t": 0.0}
    render_every = 5

    def on_frame(_):
        state["n"] += 1
        if state["n"] % render_every:
            return
        state["t"] += render_every * 0.008
        s = state["t"] / TOTAL_S
        # NB: Skate camera convention — azimuth ~240..285 faces the robot's
        # FRONT (the working side, table toward the viewer); azimuth ~60..105
        # films its back.
        cam.azimuth = 240 + 45 * (0.5 - 0.5 * np.cos(np.pi * min(s, 1.0)))  # eased pan
        r.update_scene(d, camera=cam)
        writer.append_data(r.render())

    move_joints(m, d, {"a1": 0.3, "a3": 2.2}, seconds=2.0, on_frame=on_frame)
    move_joints(m, d, {"a0": 0.9, "a1": 0.3, "a3": 1.3}, seconds=2.0, on_frame=on_frame)
    e1 = reach(m, d, {"left": [*BASE_XY, HOVER_Z], "right": [*PEG_XY, HOVER_Z]},
               seconds=3.5, on_frame=on_frame, tol=0.012)
    e2 = reach(m, d, {"left": [*BASE_XY, WORK_Z], "right": [*PEG_XY, WORK_Z]},
               seconds=3.0, on_frame=on_frame, tol=0.010)
    e3 = reach(m, d, {"left": [-0.12, 0.40, LIFT_Z], "right": [0.12, 0.40, LIFT_Z]},
               seconds=3.0, on_frame=on_frame, tol=0.012)
    hold(m, d, 1.2, on_frame=on_frame)

    writer.close()
    print(f"saved {args.out}")
    for name, e in (("hover", e1), ("work", e2), ("lift", e3)):
        print(f"{name} errors:", {k: round(v, 4) for k, v in e.items()})


if __name__ == "__main__":
    main()
