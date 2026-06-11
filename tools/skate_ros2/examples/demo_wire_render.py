"""Render the skate_ros2 wire demo: a scripted client teleoperates the MuJoCo
sim endpoint over REAL UDP (same packets a physical Skate accepts), including
a live demonstration of the firmware deadman watchdog.

Two phases so the wire run stays real-time even on slow render hardware:
1. RECORD — endpoint + client exchange real UDP at 60/50 Hz for ~18 s while
   joint states and wire stats are sampled at 25 Hz;
2. REPLAY — frames are re-rendered offline from the recording with a stats
   overlay (cmd rate, telemetry rate, deadman state).

Usage:
    MUJOCO_GL=osmesa python3 examples/demo_wire_render.py \
        --model /path/to/skt_v3_demo.xml --outdir /tmp/wire_media

Timeline: 0-2 s no client (dampened) | 2-11 s connect, raise arms, wave
counter-phase + head pan | 11-13 s client goes SILENT -> watchdog dampens,
arms freeze | 13-18 s client resumes and returns the robot home.
"""

import argparse
import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

_pkg = os.environ.get("SKATE_ROS2_PATH",
                      str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, _pkg)

from skate_ros2.protocol import SkateLink            # noqa: E402
from skate_ros2.sim_endpoint import SkateSimEndpoint  # noqa: E402

T_TOTAL = 18.0
FPS = 25.0

shared = {"label": "no client — robot DAMPENED (firmware watchdog)",
          "cmds_sent": 0}


def client_script(port, t_wall0):
    """Wall-clock-paced scripted teleop client (runs in its own thread)."""
    link = SkateLink("127.0.0.1", port)

    def now():
        return time.monotonic() - t_wall0

    while now() < 2.0:
        time.sleep(0.01)
    shared["label"] = "client connecting ..."
    while not link.connected and now() < 5.0:
        link.poll()
        time.sleep(0.02)
    pose0 = np.array(link.state.dof_pos())
    targ = pose0.copy()

    def stream(deadman=(1, 1, 1)):
        link.send_command(targ, deadman=deadman)
        link.poll()
        shared["cmds_sent"] += 1
        time.sleep(1.0 / 60.0)

    # 2-4 s: smooth raise
    shared["label"] = "streaming: raise arms (60 Hz commands)"
    while (t := now()) < 4.0:
        s = (t - 2.0) / 2.0
        s = max(0.0, min(s, 1.0))
        s = s * s * (3 - 2 * s)
        targ[:] = pose0
        targ[9] = targ[17] = s * 0.5          # shoulder abduction L/R
        targ[11] = targ[19] = (1 - s) * pose0[11] + s * 1.3   # elbows
        stream()
    raised = targ.copy()

    # 4-11 s: counter-phase wave + head pan
    shared["label"] = "streaming: bimanual wave over UDP"
    while (t := now()) < 11.0:
        targ[:] = raised
        targ[11] = 1.3 + 0.45 * np.sin(2 * np.pi * 0.5 * (t - 4.0))
        targ[19] = 1.3 + 0.45 * np.sin(2 * np.pi * 0.5 * (t - 4.0) + np.pi)
        targ[24] = 0.4 * np.sin(2 * np.pi * 0.25 * (t - 4.0))
        stream()

    # 11-13 s: dead silence — the robot must dampen itself
    shared["label"] = "client SILENT — watchdog dampens in 0.3 s"
    while now() < 13.0:
        time.sleep(0.05)               # no packets at all

    # 13-18 s: reconnect and bring it home
    shared["label"] = "client resumed: returning home"
    link.poll()
    time.sleep(0.05)
    link.poll()
    cur = np.array(link.state.dof_pos())
    while (t := now()) < T_TOTAL - 0.2:
        s = (t - 13.0) / 3.5
        s = max(0.0, min(s, 1.0))
        s = s * s * (3 - 2 * s)
        targ[:] = (1 - s) * cur + s * pose0
        stream()
    link.close()


def record(model_path, port):
    import mujoco
    ep = SkateSimEndpoint(model_path, port=port, bind="127.0.0.1",
                          telemetry_hz=50.0, realtime=True, verbose=False)
    t_wall0 = time.monotonic()
    th = threading.Thread(target=client_script, args=(port, t_wall0),
                          daemon=True)
    th.start()

    records = []
    next_tel, next_rec = 0.0, 0.0
    while ep.d.time < T_TOTAL:
        wall = time.monotonic() - t_wall0
        if ep.d.time > wall:
            time.sleep(min(ep.d.time - wall, 0.005))
            continue
        ep.pump_network()
        ep._apply_command()
        mujoco.mj_step(ep.m, ep.d)
        if wall >= next_tel:
            ep._update_temps(ep.telemetry_period)
            ep.send_telemetry()
            next_tel = wall + ep.telemetry_period
        if ep.d.time >= next_rec:
            records.append(dict(
                t=ep.d.time,
                qpos=ep.d.qpos[:26].copy(),
                ctrl=ep.d.ctrl[:26].copy(),
                n_cmds=ep.n_cmds,
                n_tel=ep.n_telemetry,
                dampened=ep.dampened,
                label=shared["label"],
                client=ep.client))
            next_rec += 1.0 / FPS
    th.join(timeout=2)
    ep.close()
    return records


def overlay(img, rec, rate_cmd, rate_tel):
    from PIL import Image, ImageDraw, ImageFont
    base = "/usr/share/fonts/truetype/dejavu/"
    try:
        f_title = ImageFont.truetype(base + "DejaVuSerif-Bold.ttf", 22)
        f_mono = ImageFont.truetype(base + "DejaVuSansMono.ttf", 16)
    except OSError:
        f_title = f_mono = ImageFont.load_default()
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im, "RGBA")
    dr.rectangle([0, 0, im.width, 96], fill=(12, 14, 18, 215))
    dr.text((14, 8), "skate_ros2 — one UDP wire for sim and real robot",
            font=f_title, fill=(235, 238, 242))
    client = rec["client"]
    client_s = f"{client[0]}:{client[1]}" if client else "—"
    state = "DAMPENED" if rec["dampened"] else "ACTIVE"
    color = (255, 120, 90) if rec["dampened"] else (120, 230, 140)
    dr.text((14, 42),
            f"client {client_s:<17} cmd → {rate_cmd:4.0f} Hz   "
            f"telemetry ← {rate_tel:4.0f} pkt/s",
            font=f_mono, fill=(200, 205, 212))
    dr.text((14, 66), f"t = {rec['t']:5.2f} s   robot ", font=f_mono,
            fill=(200, 205, 212))
    dr.text((14 + f_mono.getlength(f"t = {rec['t']:5.2f} s   robot "), 66),
            state, font=f_mono, fill=color)
    dr.text((14 + f_mono.getlength(f"t = {rec['t']:5.2f} s   robot "
                                   + state + "  "), 66),
            "· " + rec["label"], font=f_mono, fill=(160, 168, 178))
    return np.asarray(im)


def replay(model_path, records, outdir):
    import imageio
    import mujoco
    out = Path(outdir)
    out.mkdir(parents=True, exist_ok=True)
    m = mujoco.MjModel.from_xml_path(str(model_path))
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, 720, 960)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, -0.1]
    cam.distance = 2.3
    cam.elevation = -10
    cam.azimuth = 255          # front side of the robot

    mp4 = imageio.get_writer(out / "ros2_wire_demo.mp4", fps=int(FPS),
                             codec="libx264", quality=8)
    gif = imageio.get_writer(out / "ros2_wire_demo.gif", fps=13, loop=0)
    still_at = {8.0: "16_ros2_wire_wave.png",
                12.2: "17_ros2_wire_dampened.png"}
    done_stills = set()

    win = int(FPS)  # 1 s sliding window for rates
    from PIL import Image
    for i, rec in enumerate(records):
        d.qpos[:26] = rec["qpos"]
        mujoco.mj_forward(m, d)
        r.update_scene(d, cam)
        img = r.render()
        j = max(0, i - win)
        dt = max(rec["t"] - records[j]["t"], 1e-6)
        rate_cmd = (rec["n_cmds"] - records[j]["n_cmds"]) / dt
        rate_tel = (rec["n_tel"] - records[j]["n_tel"]) / dt
        frame = overlay(img, rec, rate_cmd, rate_tel)
        mp4.append_data(frame)
        if i % 2 == 0 and 1.5 <= rec["t"] <= 14.5:
            small = Image.fromarray(frame).resize((480, 360))
            gif.append_data(np.asarray(small))
        for t_still, name in still_at.items():
            if name not in done_stills and rec["t"] >= t_still:
                Image.fromarray(frame).save(out / name)
                done_stills.add(name)
    mp4.close()
    gif.close()
    print(f"media written to {out}")


def stats_plot(records, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.array([r["t"] for r in records])
    win = int(FPS)
    rc, rt = [], []
    for i, rec in enumerate(records):
        j = max(0, i - win)
        dt = max(rec["t"] - records[j]["t"], 1e-6)
        rc.append((rec["n_cmds"] - records[j]["n_cmds"]) / dt)
        rt.append((rec["n_tel"] - records[j]["n_tel"]) / dt)
    err = np.array([np.abs(r["qpos"][11] - r["ctrl"][11]) for r in records])
    damp = np.array([r["dampened"] for r in records])

    fig, ax = plt.subplots(2, 1, figsize=(9, 5.6), sharex=True)
    ax[0].plot(t, rc, label="commands in (Hz)", lw=1.6)
    ax[0].plot(t, rt, label="telemetry out (pkt/s)", lw=1.6)
    ax[0].set_ylabel("rate")
    ax[0].legend(loc="upper right")
    ax[1].plot(t, err, label="left elbow |target − actual| (rad)", lw=1.6,
               color="tab:purple")
    ax[1].set_ylabel("tracking err, rad")
    ax[1].set_xlabel("time, s")
    ax[1].legend(loc="upper right")
    for a in ax:
        a.grid(alpha=0.3)
        for s0, s1 in _spans(t, damp):
            a.axvspan(s0, s1, color="tab:red", alpha=0.12)
    ax[0].set_title("skate_ros2 wire demo — UDP rates and tracking "
                    "(red = robot dampened by watchdog)")
    fig.tight_layout()
    fig.savefig(Path(outdir) / "ros2_wire_stats.png", dpi=130)
    print("stats plot written")


def _spans(t, mask):
    spans, start = [], None
    for i, v in enumerate(mask):
        if v and start is None:
            start = t[i]
        if not v and start is not None:
            spans.append((start, t[i]))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True,
                    help="render-ready MJCF (skt_v3_demo.xml from sim/)")
    ap.add_argument("--outdir", default="wire_media")
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()

    print("phase 1: real-time UDP run (no rendering) ...")
    records = record(args.model, port)
    n_active = sum(1 for r in records if not r["dampened"])
    print(f"recorded {len(records)} frames, {n_active} active")
    print("phase 2: offline replay render ...")
    replay(args.model, records, args.outdir)
    stats_plot(records, args.outdir)


if __name__ == "__main__":
    main()
