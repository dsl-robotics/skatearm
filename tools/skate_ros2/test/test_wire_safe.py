"""The wire decoder is a restricted unpickler: legitimate telemetry/command
packets round-trip, but malicious globals / reduce gadgets are refused so a
hostile UDP packet can never execute code.

Hardware-free: needs only numpy + the protocol module (no MuJoCo, no ROS).

    python -m pytest -q tools/skate_ros2/test/test_wire_safe.py
"""
import os
import pickle
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_ros2 import protocol                       # noqa: E402
from skate_ros2 import shared_classes_def as SCD       # noqa: E402


def test_command_roundtrip_safe():
    data = protocol.pack_command(np.zeros(26), (0.0, 0.0, 0.0), 1.0, (1, 1, 1))
    pkt_id, payload = protocol.decode_packet(data)
    targ, vel, height, dm = payload
    assert pkt_id == protocol.COMMAND_ID
    assert targ.shape == (26,) and tuple(dm) == (1, 1, 1)


def test_telemetry_class_roundtrip_safe():
    for pkt_id, obj in ((1, SCD.motor_state()), (2, SCD.state_est()),
                        (3, SCD.INS_fusion_state())):
        got_id, got = protocol.decode_packet(pickle.dumps((pkt_id, obj)))
        assert got_id == pkt_id and type(got) is type(obj)


def test_blocks_forbidden_global():
    """A packet referencing builtins.eval must be refused, not resolved."""
    evil = pickle.dumps(eval)
    try:
        protocol.decode_packet(evil)
        assert False, "restricted unpickler must block builtins.eval"
    except pickle.UnpicklingError:
        pass


_SENTINEL = Path(__file__).with_name("_pwned_marker.tmp")


class _Boom:
    def __reduce__(self):
        # If the unpickler resolved os.system this would run on load.
        return (os.system, (f'echo pwned > "{_SENTINEL}"',))


def test_blocks_reduce_gadget_no_execution():
    if _SENTINEL.exists():
        _SENTINEL.unlink()
    payload = pickle.dumps(_Boom())
    try:
        protocol.decode_packet(payload)
        assert False, "restricted unpickler must block the os.system gadget"
    except pickle.UnpicklingError:
        pass
    assert not _SENTINEL.exists(), "gadget executed — RCE not prevented!"


def test_raw_mode_opts_out():
    os.environ["SKATE_WIRE"] = "raw"
    try:
        pkt_id, _ = protocol.decode_packet(protocol.pack_command(np.zeros(26)))
        assert pkt_id == protocol.COMMAND_ID
    finally:
        os.environ.pop("SKATE_WIRE", None)


def test_numpy_wildcard_closed():
    """The old numpy.* startswith hole is closed: an arbitrary numpy global that
    is NOT an array-reconstruction entry point (numpy.f2py / numpy.distutils
    command-exec helpers, or even numpy.add) must be refused, while the real
    reconstruction names still resolve."""
    import io
    import pytest
    u = protocol._RestrictedUnpickler(io.BytesIO(b""))
    for mod, name in [("numpy", "add"),
                      ("numpy.f2py.diagnose", "run_command"),
                      ("numpy.distutils", "exec_command"),
                      ("numpy.ctypeslib", "load_library"),
                      ("os", "system"), ("builtins", "eval")]:
        with pytest.raises(pickle.UnpicklingError):
            u.find_class(mod, name)
    assert u.find_class("numpy", "ndarray") is np.ndarray


def test_numpy_array_still_roundtrips():
    """Legit numpy arrays (why numpy is allow-listed at all) still decode."""
    arr = np.arange(6, dtype=np.float64).reshape(2, 3)
    _id, got = protocol.decode_packet(pickle.dumps((2, arr)))
    assert isinstance(got, np.ndarray) and got.shape == (2, 3) and got[1, 2] == 5.0
