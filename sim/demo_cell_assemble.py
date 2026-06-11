#!/usr/bin/env python3
"""Phase 1 capstone demo: FULL BIMANUAL ASSEMBLY — the demonstrator task's
manipulation core, end to end under physics:

  1. fold + raise            — collision-aware route over the table edge
  2. lateral-offset grasps   — wrists grab OUTBOARD of the parts (±5 cm), so
                               the two hands don't collide when the parts meet
  3. orientation-locked carry— both arms servo to the meet point with 6-DOF IK
                               holding the grasp orientation (peg stays ≤2° off
                               vertical; without the lock it tilts 25°+ and the
                               insertion fails)
  4. ALIGN                   — relative servoing: peg xy -> pocket xy, measured
                               from body poses (the future QC camera's job)
  5. INSERT                  — slow force-guarded descent (tau watchdog on the
                               right arm), live xy correction; pocket walls
                               funnel the last ~2 mm
  6. release right, place    — left arm lowers the ASSEMBLED unit to the table
  7. release left, retreat   — clip ends at rest, peg still in the pocket

Verified: insertion depth 18.5 mm (target 18), peg tilt ~1.5 deg, peg stays in
the pocket through placement (rel pose [0, 0.001, 0.025]).

Usage:
    python make_control_model.py /path/to/skate_teleop/skt_v3
    python make_collision_model.py /path/to/skate_teleop/skt_v3
    python make_cell_scene.py /path/to/skate_teleop/skt_v3
    python demo_cell_assemble.py --model /path/to/skate_teleop/skt_v3 --out demo.mp4
"""
import argparse
import os
import sys

import imageio
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from primitives import reach, hold, move_joints, grasp, release, Arm  # noqa: E402

MEET_L = [-0.053, 0.41, 0.21]   # left wrist: block (offset +5.3 cm) at x ~ 0
STAGE_R = [0.053, 0.41, 0.30]   # right wrist: peg staged above the pocket
DEPTH_TARGET = 0.018            # peg bottom below the pocket opening, m
TAU_LIMIT = 25.0                # insertion abort threshold, N*m above baseline
TOTAL_S = 38.0                  # nominal timeline for the camera ease


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to skate_teleop/skt_v3")
    ap.add_argument("--out", default="cell_assemble_demo.mp4", help=".mp4 or .gif")
    ap.add_argument("--fps", type=int, default=None)
    ap.add_argument("--size", default=None, help="WxH")
    args = ap.parse_args()

    is_mp4 = args.out.lower().endswith(".mp4")
    fps = args.fps or (20 if is_mp4 else 12)
    size = args.size or ("960x720" if is_mp4 else "480x360")
    w, h = (int(x) for x in size.lower().split("x"))

    m = mujoco.MjModel.from_xml_path(os.path.join(args.model, "skt_v3_cell.xml"))
    d = mujoco.MjData(m)
    for _ in range(500):
        mujoco.mj_step(m, d)
    bp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_part")
    pg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "peg")

    r = mujoco.Renderer(m, h, w)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0, 0.30, 0.15]
    cam.distance = 1.8
    cam.elevation = -18

    if is_mp4:
        writer = imageio.get_writer(args.out, fps=fps, codec="libx264",
                                    quality=7, pixelformat="yuv420p")
    else:
        writer = imageio.get_writer(args.out, fps=fps, loop=0)

    state = {"n": 0, "t": 0.0}

    def on_frame(_=None):
        state["n"] += 1
        if state["n"] % 6:
            return
        state["t"] += 6 * 0.008
        s = state["t"] / TOTAL_S
        cam.azimuth = 235 + 50 * (0.5 - 0.5 * np.cos(np.pi * min(s, 1.0)))
        r.update_scene(d, camera=cam)
        writer.append_data(r.render())

    armL, armR = Arm(m, d, "left"), Arm(m, d, "right")

    def servo6_both(tL, tR, seconds, tol=0.010):
        sL, sR = armL.ee_pos(), armR.ee_pos()
        gL, gR = np.asarray(tL, float), np.asarray(tR, float)
        n = int(seconds / (m.opt.timestep * 4))
        for i in range(n + 400):
            ss = min(1, (i + 1) / n)
            ss = ss * ss * (3 - 2 * ss)
            qL, _ = armL.ik_step6(sL + (gL - sL) * ss)
            armL.set_ctrl(qL)
            qR, _ = armR.ik_step6(sR + (gR - sR) * ss)
            armR.set_ctrl(qR)
            for _ in range(4):
                mujoco.mj_step(m, d)
            on_frame()
            if i >= n and np.linalg.norm(gL - armL.ee_pos()) < tol \
                    and np.linalg.norm(gR - armR.ee_pos()) < tol:
                break

    def pocket_top():
        return d.xpos[bp] + np.array([0, 0, 0.027])

    def peg_bottom():
        return d.xpos[pg] + np.array([0, 0, -0.020])

    # 1-2: approach and lateral-offset grasps
    move_joints(m, d, {"a1": 0.3, "a3": 2.2}, seconds=1.8, on_frame=on_frame)
    move_joints(m, d, {"a0": 0.9, "a1": 0.3, "a3": 1.3}, seconds=1.8, on_frame=on_frame)
    reach(m, d, {"left": [-0.18, 0.44, 0.20], "right": [0.18, 0.44, 0.20]},
          seconds=2.4, on_frame=on_frame, tol=0.012)
    reach(m, d, {"left": [-0.18, 0.44, 0.115], "right": [0.18, 0.44, 0.115]},
          seconds=2.0, on_frame=on_frame, tol=0.010)
    grasp(m, d, "left")
    grasp(m, d, "right")
    hold(m, d, 0.4, on_frame=on_frame)
    armL.lock_orientation()
    armR.lock_orientation()

    # 3: orientation-locked carry to the meet point
    servo6_both(MEET_L, STAGE_R, seconds=4.5)

    # 4: align peg over the pocket (relative servoing)
    for _ in range(400):
        err_xy = (pocket_top() - d.xpos[pg])[:2]
        zerr = (pocket_top()[2] + 0.030) - peg_bottom()[2]
        q, _ = armR.ik_step6(armR.ee_pos() + np.array([err_xy[0], err_xy[1], zerr]))
        armR.set_ctrl(q)
        qL, _ = armL.ik_step6(np.asarray(MEET_L))
        armL.set_ctrl(qL)
        for _ in range(4):
            mujoco.mj_step(m, d)
        on_frame()
        if np.linalg.norm(err_xy) < 0.0035 and abs(zerr) < 0.007:
            break

    # 5: force-guarded insertion
    tau_ids = [m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR,
               f"tau_a{k}_armR_a{16+k}")] for k in range(8)]
    tau0 = sum(abs(d.sensordata[a]) for a in tau_ids)
    aborted = False
    for _ in range(1500):
        err_xy = (pocket_top() - d.xpos[pg])[:2]
        if pocket_top()[2] - peg_bottom()[2] >= DEPTH_TARGET:
            break
        q, _ = armR.ik_step6(armR.ee_pos()
                             + np.array([err_xy[0] * 0.8, err_xy[1] * 0.8, -0.0014]))
        armR.set_ctrl(q)
        qL, _ = armL.ik_step6(np.asarray(MEET_L))
        armL.set_ctrl(qL)
        for _ in range(4):
            mujoco.mj_step(m, d)
        on_frame()
        if sum(abs(d.sensordata[a]) for a in tau_ids) > tau0 + TAU_LIMIT:
            aborted = True
            break
    depth = pocket_top()[2] - peg_bottom()[2]

    # 6-7: release right, place the assembled unit, release left, retreat
    release(m, d, "right")
    hold(m, d, 0.5, on_frame=on_frame)
    reach(m, d, {"right": [0.20, 0.36, 0.30]}, seconds=2.2, on_frame=on_frame, tol=0.015)
    sL, gL = armL.ee_pos(), np.array([-0.053, 0.42, 0.116])
    n = int(2.6 / (m.opt.timestep * 4))
    for i in range(n + 300):
        ss = min(1, (i + 1) / n)
        ss = ss * ss * (3 - 2 * ss)
        qL, _ = armL.ik_step6(sL + (gL - sL) * ss)
        armL.set_ctrl(qL)
        for _ in range(4):
            mujoco.mj_step(m, d)
        on_frame()
        if i >= n and np.linalg.norm(gL - armL.ee_pos()) < 0.008:
            break
    release(m, d, "left")
    hold(m, d, 0.5, on_frame=on_frame)
    reach(m, d, {"left": [-0.20, 0.38, 0.26]}, seconds=2.0, on_frame=on_frame, tol=0.015)
    hold(m, d, 1.0, on_frame=on_frame)

    writer.close()
    rel = d.xpos[pg] - d.xpos[bp]
    print(f"saved {args.out}")
    print(f"insertion: depth {depth*1000:.1f} mm (target {DEPTH_TARGET*1000:.0f}), aborted={aborted}")
    print(f"final: block at {d.xpos[bp].round(3)}, peg rel block {rel.round(3)} "
          f"(in pocket: |xy|<6mm, z 0.015-0.03)")


if __name__ == "__main__":
    main()
