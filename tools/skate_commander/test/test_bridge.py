"""RobotBridge vs the skate_ros2 MuJoCo sim endpoint over real UDP."""

import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skate_ros2"))

from skate_commander.bridge import RobotBridge          # noqa: E402


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close()
    return p


def _spin(bridge, seconds, ui=True, hz=60):
    end = time.monotonic() + seconds
    snap = None
    while time.monotonic() < end:
        snap = bridge.tick(1.0 / hz, ui_attached=ui)
        time.sleep(1.0 / hz)
    return snap


def test_bridge_full_cycle():
    model = os.environ.get("SKATE_MJCF", "/tmp/skate_teleop/skt_v3/skt_v3_control.xml")
    if not Path(model).exists():
        print("SKIP: no control model"); return
    from skate_ros2.sim_endpoint import SkateSimEndpoint

    port = _free_port()
    ep = SkateSimEndpoint(model, port=port, bind="127.0.0.1", verbose=False)
    th = threading.Thread(target=ep.run, kwargs={"duration": 30.0}, daemon=True)
    th.start()

    br = RobotBridge(sim_host="127.0.0.1", sim_port=port, jog_rate=0.6)
    # starts estopped; arming happens from telemetry
    snap = _spin(br, 0.6)
    assert snap["connected"] and snap["armed"] and snap["estop"]
    assert not snap["live"]

    # jog while estopped must not move the target
    t0 = np.array(br.targ)
    br.jog_start(11, +1)
    _spin(br, 0.3)
    assert np.allclose(br.targ, t0)

    # resume -> jog moves the elbow, clamped to limits eventually
    assert br.resume()
    snap = _spin(br, 1.5)
    assert snap["live"]
    assert br.targ[11] > t0[11] + 0.4
    br.jog_stop(11)
    elbow_target = br.targ[11]
    snap = _spin(br, 1.0)
    assert abs(snap["q"][11] - elbow_target) < 0.08   # sim tracks the target

    # ui detached -> deadman drops, target freezes
    br.jog_start(11, +1)
    before = br.targ[11]
    snap = _spin(br, 0.4, ui=False)
    assert not snap["live"] and abs(br.targ[11] - before) < 1e-9

    # mode switch re-latches estop and disarms
    br.set_mode("real")
    assert br.estop and br.targ is None and br.mode == "real"
    br.set_mode("sim")
    snap = _spin(br, 0.6)
    assert snap["armed"] and snap["estop"]

    br.close(); ep.close()


if __name__ == "__main__":
    test_bridge_full_cycle(); print("PASS test_bridge_full_cycle")
