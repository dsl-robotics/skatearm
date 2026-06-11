#!/usr/bin/env python3
"""Telemetry demo: run the wave trajectory on the collision model while logging
the model's sensors (joint pos/vel, actuator torque, end-effector sites), then
plot target-vs-actual tracking, torques and EE paths.

This is the seed of the future SCADA dashboard: the same sensor names
(qpos_*/qvel_*/tau_*/ee_*) will be the telemetry schema for both the sim and
the real Skate's state stream.

Usage:
    python make_control_model.py /path/to/skate_teleop/skt_v3      # first
    python make_collision_model.py /path/to/skate_teleop/skt_v3    # second
    python telemetry_demo.py --model /path/to/skate_teleop/skt_v3 \
        [--out sensor_tracking.png] [--csv telemetry.csv]
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

REST = {"a1": 0.2, "a3": 0.5, "a0": 0.0, "a5": 0.0}
REACH = {"a0": 0.9, "a1": 0.25, "a3": 1.3, "a5": 0.3}
WORK = {"a0": 0.4, "a1": 0.3, "a3": 0.9, "a5": 0.2}


def lerp(p1, p2, s):
    return {k: p1.get(k, 0) * (1 - s) + p2.get(k, 0) * s for k in set(p1) | set(p2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to skate_teleop/skt_v3")
    ap.add_argument("--out", default="sensor_tracking.png")
    ap.add_argument("--csv", default=None, help="optional raw telemetry CSV")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(os.path.join(args.model, "skt_v3_collision.xml"))
    d = mujoco.MjData(m)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
    aidx = {nm[4:]: i for i, nm in enumerate(names)}

    def sread(name):
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
        adr, dim = m.sensor_adr[sid], m.sensor_dim[sid]
        return d.sensordata[adr:adr + dim].copy()

    def set_t(L, R):
        for k, v in L.items():
            d.ctrl[aidx[f"{k}_armL_a{8 + int(k[1])}"]] = v
        for k, v in R.items():
            d.ctrl[aidx[f"{k}_armR_a{16 + int(k[1])}"]] = v

    FPS = 100
    spf = int(1 / (FPS * m.opt.timestep))
    cols = ["t", "tgtL0", "qL0", "tauL0", "tgtL3", "qL3", "tauL3", "tgtR0", "qR0", "tauR0"]
    log = {k: [] for k in cols}
    ee_log = {"eeL": [], "eeR": []}
    t = [0.0]

    def run(seconds, fL, fR):
        n = int(seconds * FPS)
        for f in range(n):
            s = (f + 1) / n
            sm = 0.5 - 0.5 * np.cos(np.pi * s)
            L, R = fL(sm), fR(sm)
            set_t(L, R)
            for _ in range(spf):
                mujoco.mj_step(m, d)
            t[0] += 1 / FPS
            log["t"].append(t[0])
            log["tgtL0"].append(L.get("a0", 0))
            log["qL0"].append(sread("qpos_a0_armL_a8")[0])
            log["tauL0"].append(sread("tau_a0_armL_a8")[0])
            log["tgtL3"].append(L.get("a3", 0))
            log["qL3"].append(sread("qpos_a3_armL_a11")[0])
            log["tauL3"].append(sread("tau_a3_armL_a11")[0])
            log["tgtR0"].append(R.get("a0", 0))
            log["qR0"].append(sread("qpos_a0_armR_a16")[0])
            log["tauR0"].append(sread("tau_a0_armR_a16")[0])
            ee_log["eeL"].append(sread("ee_left_pos"))
            ee_log["eeR"].append(sread("ee_right_pos"))

    run(1.0, lambda s: REST, lambda s: REST)
    run(1.8, lambda s: lerp(REST, REACH, s), lambda s: REST)
    run(1.8, lambda s: lerp(REACH, REST, s), lambda s: lerp(REST, REACH, s))
    run(1.8, lambda s: lerp(REST, WORK, s), lambda s: lerp(REACH, WORK, s))
    run(1.0, lambda s: WORK, lambda s: WORK)

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols + ["eeL_x", "eeL_y", "eeL_z", "eeR_x", "eeR_y", "eeR_z"])
            for i in range(len(log["t"])):
                w.writerow([log[c][i] for c in cols]
                           + list(ee_log["eeL"][i]) + list(ee_log["eeR"][i]))
        print("wrote", args.csv)

    T = np.array(log["t"])
    eeL, eeR = np.array(ee_log["eeL"]), np.array(ee_log["eeR"])
    fig, ax = plt.subplots(2, 2, figsize=(12, 7.5), dpi=110)
    fig.suptitle("SkateArm telemetry — joint tracking & torques under position control", fontsize=11)

    ax[0, 0].plot(T, log["tgtL0"], "k--", lw=1, label="target")
    ax[0, 0].plot(T, log["qL0"], lw=1.6, label="actual")
    ax[0, 0].set_title("Left shoulder a0 — tracking")
    ax[0, 0].set_ylabel("rad")
    ax[0, 0].legend()

    ax[0, 1].plot(T, log["tgtL3"], "k--", lw=1, label="target L a3")
    ax[0, 1].plot(T, log["qL3"], lw=1.6, label="actual L a3")
    ax[0, 1].plot(T, log["tgtR0"], "k:", lw=1, label="target R a0")
    ax[0, 1].plot(T, log["qR0"], lw=1.6, label="actual R a0")
    ax[0, 1].set_title("Left elbow a3 & right shoulder a0")
    ax[0, 1].legend(fontsize=7)

    ax[1, 0].plot(T, log["tauL0"], lw=1.2, label="L shoulder a0")
    ax[1, 0].plot(T, log["tauL3"], lw=1.2, label="L elbow a3")
    ax[1, 0].axhline(28, color="r", ls=":", lw=1)
    ax[1, 0].axhline(-28, color="r", ls=":", lw=1)
    ax[1, 0].set_title("Actuator torque (red dotted = ±28 N·m limit)")
    ax[1, 0].set_ylabel("N·m")
    ax[1, 0].set_xlabel("s")
    ax[1, 0].legend(fontsize=8)

    ax[1, 1].plot(T, eeL[:, 2], lw=1.4, label="left EE z")
    ax[1, 1].plot(T, eeR[:, 2], lw=1.4, label="right EE z")
    ax[1, 1].plot(T, eeL[:, 1], lw=1, ls="--", label="left EE y (fwd)")
    ax[1, 1].plot(T, eeR[:, 1], lw=1, ls="--", label="right EE y (fwd)")
    ax[1, 1].set_title("End-effector position (site sensors)")
    ax[1, 1].set_xlabel("s")
    ax[1, 1].set_ylabel("m")
    ax[1, 1].legend(fontsize=8)

    for a in ax.flat:
        a.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(args.out)
    print("saved", args.out)


if __name__ == "__main__":
    main()
