"""Loopback tests: SkateLink <-> a minimal fake firmware over localhost UDP.

No MuJoCo, no ROS — just the wire contract.
"""

import pickle
import socket
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_ros2 import names                    # noqa: E402
from skate_ros2 import shared_classes_def as SCD  # noqa: E402
from skate_ros2.protocol import (COMMAND_ID, SkateLink, pack_command,  # noqa: E402
                                 unpack_packet)


def make_fake_robot():
    """Bind an ephemeral UDP socket acting as the firmware side."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(1.0)
    return sock, sock.getsockname()[1]


def test_pack_unpack_roundtrip():
    targ = np.arange(26, dtype=np.float64) / 10.0
    data = pack_command(targ, (0.1, -0.2, 0.3), 0.9, (1, 0, 1))
    pkt_id, (t, v, h, dm) = unpack_packet(data)
    assert pkt_id == COMMAND_ID
    assert np.allclose(t, targ)
    assert np.allclose(v, [0.1, -0.2, 0.3])
    assert h == 0.9
    assert dm == (1, 0, 1)


def test_pack_command_validates_shape():
    try:
        pack_command([0.0] * 25)
        raise AssertionError("should have raised")
    except ValueError:
        pass


def test_command_reaches_robot_and_telemetry_comes_back():
    robot, port = make_fake_robot()
    link = SkateLink("127.0.0.1", port)

    # client -> robot: command arrives and decodes
    targ = np.array(names.DEFAULT_POSE)
    assert link.send_command(targ, deadman=(1, 1, 1))
    data, client_addr = robot.recvfrom(65536)
    pkt_id, (t, v, h, dm) = pickle.loads(data)
    assert pkt_id == COMMAND_ID and dm == (1, 1, 1)
    assert np.allclose(t, targ)

    # robot -> client: telemetry decodes through the vendored classes
    ms = SCD.motor_state()
    ms.motor_pos = names.vector_to_can_dict(np.linspace(0, 1, 26))
    robot.sendto(pickle.dumps((1, ms)), client_addr)
    se = SCD.state_est()
    se.dof_pos = names.vector_to_can_dict(np.linspace(1, 2, 26))
    robot.sendto(pickle.dumps((2, se)), client_addr)

    deadline = time.time() + 1.0
    while time.time() < deadline and link.state.state_estimates is None:
        link.poll()
        time.sleep(0.01)

    assert link.state.motor_states is not None
    assert link.state.state_estimates is not None
    assert link.connected
    pos = link.state.dof_pos()
    assert abs(pos[0] - 1.0) < 1e-9 and abs(pos[25] - 2.0) < 1e-9

    # staleness: after 0.3 s with no packets the link reports disconnected
    time.sleep(0.35)
    assert not link.connected

    link.close()
    robot.close()


def test_heartbeat_is_official_yo():
    robot, port = make_fake_robot()
    link = SkateLink("127.0.0.1", port)
    link.poll()  # first poll fires a heartbeat
    data, _ = robot.recvfrom(65536)
    assert data == b"yo"
    link.close()
    robot.close()


if __name__ == "__main__":
    for f in [test_pack_unpack_roundtrip, test_pack_command_validates_shape,
              test_command_reaches_robot_and_telemetry_comes_back,
              test_heartbeat_is_official_yo]:
        f()
        print(f"PASS {f.__name__}")
