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


def test_cart_step_and_mirror():
    """v0.5: cartesian step-jog auto-clears on arrival; mirror mode reflects
    jog / slider / IK input onto the other arm with the measured sign map."""
    model_xml = os.environ.get("SKATE_MJCF",
                               "/tmp/skate_teleop/skt_v3/skt_v3_control.xml")
    if not Path(model_xml).exists():
        print("SKIP: no control model"); return
    urdf = Path(model_xml).parent / "skt_v3.urdf"
    if not urdf.exists():
        print("SKIP: no URDF next to the control model"); return
    from skate_commander.kinematics import ArmKinematics
    from skate_commander.server import compute_mirror_map
    from skate_commander.urdf import joint_limits, parse_urdf
    from skate_ros2.sim_endpoint import SkateSimEndpoint

    model = parse_urdf(urdf)
    kin = {a: ArmKinematics(model, a) for a in ("left", "right")}
    port = _free_port()
    ep = SkateSimEndpoint(model_xml, port=port, bind="127.0.0.1",
                          verbose=False)
    th = threading.Thread(target=ep.run, kwargs={"duration": 40.0},
                          daemon=True)
    th.start()

    br = RobotBridge(sim_host="127.0.0.1", sim_port=port,
                     limits=joint_limits(model), kin=kin)
    br.mirror_signs, br.mirror_axis = compute_mirror_map(kin)
    _spin(br, 0.6)
    assert br.targ is not None
    br.resume()

    # -- cartesian step: TCP glides 5 cm up, target auto-clears -------------
    p0 = kin["right"].fk(br.targ)
    br.cart_step("right", [0.0, 0.0, 0.05])
    assert br.ik_targets["right"] is not None and br.ik_auto["right"]
    end = time.monotonic() + 4.0
    while br.ik_targets["right"] is not None and time.monotonic() < end:
        br.tick(1 / 60, ui_attached=True)
        time.sleep(1 / 60)
    assert br.ik_targets["right"] is None, "cart target must auto-clear"
    moved = kin["right"].fk(br.targ) - p0
    assert abs(moved[2] - 0.05) < 0.01, f"dz={moved[2]:.3f}, wanted 0.05"
    print(f"PASS cart_step: dz={moved[2]*1000:.1f} mm, auto-cleared")

    # -- mirror: slider input reflects with the measured sign ----------------
    br.home()                                  # symmetric documented pose
    _spin(br, 0.5)
    br.mirror = True
    s2 = br.mirror_signs[2]                    # arm slot of joint 10/18
    br.set_joint(10, 0.5)
    assert abs(br.targ[18] - s2 * 0.5) < 1e-9, "mirrored slider value"
    # jog reflects too (and both stop together)
    q11, q19 = br.targ[11], br.targ[19]
    br.jog_start(11, +1)
    _spin(br, 0.5)
    br.jog_stop(11)
    d11, d19 = br.targ[11] - q11, br.targ[19] - q19
    s3 = br.mirror_signs[3]
    assert d11 > 0.05 and abs(d19 - s3 * d11) < 1e-9
    # wrists end up mirror-symmetric in the world
    pl, pr = kin["left"].fk(br.targ), kin["right"].fk(br.targ)
    mirr = pl.copy()
    mirr[br.mirror_axis] = -mirr[br.mirror_axis]
    assert np.linalg.norm(pr - mirr) < 5e-3, \
        f"wrists not mirror-symmetric: {np.linalg.norm(pr - mirr)*1000:.1f} mm"
    print(f"PASS mirror: jog/slider reflected (signs={br.mirror_signs[:4]}), "
          "wrists symmetric")

    # -- mirrored IK target + estop clears everything -------------------------
    br.set_ik_target("left", pl + [0.0, 0.0, 0.03])
    assert br.ik_targets["right"] is not None
    br.trigger_estop()
    assert br.ik_targets == {"left": None, "right": None}
    print("PASS mirrored IK target + estop clears both")
    br.close(); ep.close()


if __name__ == "__main__":
    test_bridge_full_cycle(); print("PASS test_bridge_full_cycle")
    test_cart_step_and_mirror()
