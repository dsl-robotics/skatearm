"""Minimal no-ROS client: stream a bimanual wave to a Skate (real or sim).

    # terminal 1 — the sim endpoint (or skip and use the real robot)
    python3 -m skate_ros2.sim_endpoint --model /path/to/skt_v3_control.xml

    # terminal 2
    python3 examples/wave_no_ros.py --host 127.0.0.1

Demonstrates the whole client contract in ~40 lines: poll() for telemetry,
send_command() at 60 Hz with deadman (1,1,1) while you want the robot live.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_ros2 import names
from skate_ros2.protocol import SkateLink


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="r.local")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--duration", type=float, default=15.0)
    args = ap.parse_args()

    link = SkateLink(args.host, args.port)
    print(f"waiting for telemetry from {args.host}:{args.port} ...")
    while not link.connected:
        link.poll()
        time.sleep(0.02)
    start_pose = np.array(link.state.dof_pos())
    print("telemetry up — starting wave")

    targ = start_pose.copy()
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t > args.duration:
            break
        # raise both arms and wave the forearms in counter-phase
        targ[:] = start_pose
        targ[9] = 0.6 * min(t / 2.0, 1.0)                  # left shoulder abd
        targ[17] = 0.6 * min(t / 2.0, 1.0)                 # right shoulder abd
        targ[11] = 1.2 + 0.5 * np.sin(2.0 * t)             # left elbow
        targ[19] = 1.2 + 0.5 * np.sin(2.0 * t + np.pi)     # right elbow
        link.send_command(targ, deadman=(1, 1, 1))
        link.poll()
        time.sleep(1.0 / 60.0)

    err = np.abs(np.array(link.state.dof_pos()) - targ)
    print(f"done. final tracking error: max {err.max():.3f} rad")
    link.close()


if __name__ == "__main__":
    main()
