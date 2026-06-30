"""Collision guard e2e: impossible (self-colliding) targets must be rejected
before they are ever sent — and the contact-enabled twin must agree.

Needs mujoco + skt_v3_collision.xml (sim/make_collision_model.py):
    SKT_DIR=.../skt_v3 python3 test/test_guard.py
"""

import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skate_ros2"))

SKT = Path(os.environ.get("SKT_DIR", "/tmp/skate_teleop/skt_v3"))
CXML = SKT / "skt_v3_collision.xml"



def _skip(msg):
    """Real pytest.skip under pytest; clean print when run as a standalone script."""
    import sys
    if "pytest" in sys.modules:
        import pytest
        pytest.skip(msg)
    print(f"SKIP: {msg}")


def test_guard_blocks_self_collision():
    try:
        import mujoco
    except ImportError:
        _skip("mujoco not installed"); return
    if not CXML.exists():
        _skip(f"{CXML} missing (run sim/make_collision_model.py)"); return

    from skate_commander.bridge import RobotBridge
    from skate_commander.server import make_collision_guard
    from skate_ros2.sim_endpoint import SkateSimEndpoint

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    ep = SkateSimEndpoint(str(CXML), port=port, bind="127.0.0.1",
                          verbose=False)          # twin WITH contacts
    th = threading.Thread(target=ep.run, kwargs={"duration": 30.0},
                          daemon=True)
    th.start()

    br = RobotBridge(sim_host="127.0.0.1", sim_port=port, jog_rate=0.8)
    br.guard = make_collision_guard(CXML)

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            br.tick(1 / 60, ui_attached=True)
            time.sleep(1 / 60)

    spin(0.6)
    assert br.targ is not None
    br.resume()

    # 1) a slider jump INTO the torso is rejected outright
    br.set_joint(9, -0.79)
    assert br.guard_blocking and abs(br.targ[9]) < 0.05
    print("PASS slider jump into torso rejected")

    # 2) continuous jog stops at the contact boundary, not the joint limit
    br.jog_start(9, -1); spin(1.5); br.jog_stop(9)
    stop_at = float(br.targ[9])
    assert stop_at > -0.79 + 0.05 and br.guard_blocking
    print(f"PASS jog blocked at {stop_at:+.3f} rad (joint limit -0.790)")

    # 3) the contact-enabled twin agrees (no penetration past the block)
    spin(0.8)
    q9 = br.snapshot()["q"][9]
    assert q9 > stop_at - 0.08
    print(f"PASS twin settled at {q9:+.3f} rad")

    # 4) moving away unblocks
    br.set_joint(9, 0.4); spin(0.5)
    assert not br.guard_blocking and abs(br.targ[9] - 0.4) < 1e-6
    print("PASS moving away unblocks")

    br.close(); ep.close(); th.join(timeout=5)


def test_guard_sees_legs_and_blocks_tunneling():
    """The physics model excludes hand<->hip pairs (they touch at neutral);
    the guard variant must still catch the arm sweeping into the legs — and
    a big slider jump must not tunnel through a collision between two safe
    endpoints."""
    try:
        import mujoco  # noqa: F401
    except ImportError:
        _skip("mujoco not installed"); return
    if not CXML.exists():
        _skip("no collision model"); return
    from skate_commander.bridge import RobotBridge
    from skate_commander.server import make_collision_guard

    guard = make_collision_guard(CXML)
    assert not guard(np.zeros(26)), "neutral pose must be allowed"

    # sweep the hanging arm fore/aft (J1, idx 8) and fold the elbow toward
    # the body — somewhere along these sweeps the hand/forearm reaches the
    # hip/leg boxes, which the OLD guard (physics excludes) never saw
    found = None
    for q8 in np.linspace(-1.2, 1.2, 49):
        q = np.zeros(26)
        q[8] = q8
        if guard(q):
            found = ("J1", q8)
            break
    if found is None:
        for q9 in np.linspace(-0.79, 0.3, 23):
            q = np.zeros(26)
            q[9] = q9
            if guard(q):
                found = ("J2", q9)
                break
    assert found, "guard variant should detect arm-into-leg/hip somewhere"
    print(f"PASS guard sees the lower body (blocked at {found[0]}="
          f"{found[1]:+.2f} rad)")

    # tunneling: synthetic guard with a thin forbidden slab in the middle
    br = RobotBridge()
    br.targ = np.zeros(26)
    br.estop = False
    br.guard = lambda q: 0.40 < q[11] < 0.60      # forbidden band
    br.set_joint(11, 1.2)                          # jump straight across it
    assert br.guard_blocking and br.targ[11] == 0.0, \
        "big jump across a collision band must be rejected"
    print("PASS interpolated path check stops tunneling")
    br.close()


if __name__ == "__main__":
    test_guard_blocks_self_collision()
    test_guard_sees_legs_and_blocks_tunneling()
    print("ALL COLLISION-GUARD E2E GREEN")
