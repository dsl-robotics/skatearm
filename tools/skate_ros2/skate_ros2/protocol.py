"""Skate UDP wire protocol — pure Python, no ROS dependencies.

Wire contract (reverse-engineered from the official Rbotic/skate_teleop client
and confirmed by the official Skate docs):

* Transport: UDP, robot listens on port 2000 (``r.local`` via mDNS).
* Telemetry  robot -> client: ``pickle.dumps((id, obj))`` where id is
  0 motor_commands, 1 motor_states, 2 state_estimates, 3 INS_state_estimates,
  4 controller_states (classes in :mod:`skate_ros2.shared_classes_def`).
* Command  client -> robot: ``pickle.dumps((5, (targ_pos[26], vel_cmd[3],
  height_cmd, (estop_WB, estop_LA, estop_RA))))``; flags 0 = dampen.
* Heartbeat: the robot streams only to the address it last heard from; the
  official client pings ``b"yo"`` every 0.3 s. If the robot hears nothing for
  0.3 s it assumes deadman ``(0, 0, 0)`` and dampens — that watchdog lives in
  the firmware, not here.

SECURITY NOTE: the wire format is Python pickle, which can normally execute
arbitrary code when loading. The firmware's choice of pickle is fixed, but the
decoder here defends against it -- :func:`decode_packet` uses a *restricted*
unpickler by default (only the known telemetry classes + numpy are resolvable),
so a hostile packet can't run code. Set ``SKATE_WIRE=raw`` to opt out. Even so,
prefer a trusted local network (the same assumption the official stack makes).
"""

from __future__ import annotations

import io
import os
import pickle
import socket
import sys
import time

import numpy as np

from . import names
from . import shared_classes_def as SCD

# The firmware pickles its telemetry classes under the top-level module name
# 'shared_classes_def'. Register our vendored copy so packets unpickle.
sys.modules.setdefault("shared_classes_def", SCD)

DEFAULT_PORT = 2000
DEFAULT_HOST = "r.local"
BUFFER_SIZE = 4096 * 10
HEARTBEAT = b"yo"
HEARTBEAT_PERIOD = 0.3   # s, official client value
STALE_AFTER = 0.3        # s, telemetry older than this counts as disconnected
COMMAND_ID = 5

TELEMETRY_IDS = {
    0: "motor_commands",
    1: "motor_states",
    2: "state_estimates",
    3: "ins",
    4: "controller_states",
}


def pack_command(targ_pos, vel_cmd=(0.0, 0.0, 0.0), height_cmd=1.0,
                 deadman=(0, 0, 0)):
    """Serialize one command packet exactly as the official client does."""
    targ = np.asarray(targ_pos, dtype=np.float64)
    if targ.shape != (names.N_JOINTS,):
        raise ValueError(
            f"targ_pos must have shape ({names.N_JOINTS},), got {targ.shape}")
    vel = np.asarray(vel_cmd, dtype=np.float64)
    if vel.shape != (3,):
        raise ValueError(f"vel_cmd must have shape (3,), got {vel.shape}")
    dm = (int(deadman[0]), int(deadman[1]), int(deadman[2]))
    payload = (targ, vel, float(height_cmd), dm)
    data = pickle.dumps((COMMAND_ID, payload))
    if len(data) > BUFFER_SIZE:
        raise ValueError("command packet exceeds UDP buffer size")
    return data


# Globals a legitimate packet is allowed to reconstruct. The firmware pickles
# its telemetry classes (under the top-level module name 'shared_classes_def')
# and numpy arrays; nothing else should ever appear on the wire. Whitelisting
# these turns pickle from an arbitrary-code-execution primitive into a
# fixed-shape decoder.
_SAFE_SCD_CLASSES = {
    "motor_command", "motor_state", "state_est",
    "INS_fusion_state", "FeedbackResp",
}
_SAFE_SCD_MODULES = {"shared_classes_def", "skate_ros2.shared_classes_def"}


class _RestrictedUnpickler(pickle.Unpickler):
    """Unpickler that only resolves the known telemetry classes + numpy.

    Every other ``find_class`` is refused, so a malicious packet cannot pull in
    ``os.system`` / ``builtins.eval`` / an arbitrary ``__reduce__`` gadget.
    """

    def find_class(self, module, name):
        if module in _SAFE_SCD_MODULES and name in _SAFE_SCD_CLASSES:
            return super().find_class(module, name)
        if module == "numpy" or module.startswith("numpy."):
            return super().find_class(module, name)
        if module == "copyreg" and name in {"_reconstructor", "__newobj__"}:
            return super().find_class(module, name)
        raise pickle.UnpicklingError(
            f"blocked unpickling of {module}.{name} (untrusted wire packet)")


def decode_packet(data):
    """Decode one telemetry/command packet -> (id, obj).

    Uses a restricted unpickler by default (``SKATE_WIRE`` unset or ``safe``):
    it accepts every legitimate firmware/sim packet but refuses arbitrary
    globals, so a hostile packet can't execute code. Set ``SKATE_WIRE=raw`` to
    fall back to plain ``pickle.loads`` (only on a fully trusted link, e.g. to
    decode an unforeseen class).
    """
    if os.environ.get("SKATE_WIRE", "safe").lower() == "raw":
        return pickle.loads(data)
    return _RestrictedUnpickler(io.BytesIO(data)).load()


def unpack_packet(data):
    """Decode one telemetry/command packet -> (id, obj). Trusted LAN only.

    Back-compat alias for :func:`decode_packet` (restricted unpickler by default).
    """
    return decode_packet(data)


class TelemetryState:
    """Latest decoded telemetry plus receive timestamps."""

    def __init__(self):
        self.motor_commands = None   # SCD.motor_command
        self.motor_states = None     # SCD.motor_state
        self.state_estimates = None  # SCD.state_est
        self.ins = None              # SCD.INS_fusion_state
        self.controller_states = None
        self.stamps = {}             # field name -> time.monotonic()
        self.n_packets = 0

    def update(self, pkt_id, obj, now=None):
        field = TELEMETRY_IDS.get(pkt_id)
        if field is None:
            return False
        setattr(self, field, obj)
        self.stamps[field] = now if now is not None else time.monotonic()
        self.n_packets += 1
        return True

    def age(self, now=None):
        """Seconds since the newest telemetry packet (inf if none yet)."""
        if not self.stamps:
            return float("inf")
        now = now if now is not None else time.monotonic()
        return now - max(self.stamps.values())

    @property
    def connected(self):
        return self.age() < STALE_AFTER

    def dof_pos(self):
        """Calibrated joint positions as a flat 26-list (None if not seen)."""
        if self.state_estimates is None:
            return None
        return names.can_dict_to_vector(self.state_estimates.dof_pos)

    def dof_vel(self):
        if self.state_estimates is None:
            return None
        return names.can_dict_to_vector(self.state_estimates.dof_vel)

    def dof_torque(self):
        if self.state_estimates is None:
            return None
        return names.can_dict_to_vector(self.state_estimates.dof_torque)

    def motor_temps(self):
        if self.motor_states is None:
            return None
        return names.can_dict_to_vector(self.motor_states.motor_temp)


class SkateLink:
    """UDP client to a Skate robot (or :mod:`skate_ros2.sim_endpoint`).

    Non-blocking; call :meth:`poll` often (e.g. from a 60 Hz timer). Heartbeats
    are sent automatically from :meth:`poll`, so the robot keeps streaming even
    when no commands are being sent.
    """

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT):
        self.host_name = host
        self.port = port
        self.addr = None          # resolved (ip, port)
        self.state = TelemetryState()
        self._last_heartbeat = 0.0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self.decode_errors = 0

    # -- connection -------------------------------------------------------
    def resolve(self):
        """Resolve the robot hostname. Returns True on success."""
        try:
            ip = socket.gethostbyname(self.host_name)
        except socket.gaierror:
            self.addr = None
            return False
        self.addr = (ip, self.port)
        return True

    @property
    def connected(self):
        return self.state.connected

    # -- io ----------------------------------------------------------------
    def heartbeat(self, now=None):
        if self.addr is None and not self.resolve():
            return False
        now = now if now is not None else time.monotonic()
        try:
            self._sock.sendto(HEARTBEAT, self.addr)
        except OSError:
            return False
        self._last_heartbeat = now
        return True

    def poll(self):
        """Drain all pending telemetry; auto-heartbeat. Returns packet count."""
        now = time.monotonic()
        if now - self._last_heartbeat > HEARTBEAT_PERIOD:
            self.heartbeat(now)
        n = 0
        while True:
            try:
                data, _addr = self._sock.recvfrom(BUFFER_SIZE)
            except BlockingIOError:
                break
            except OSError:
                break
            try:
                pkt_id, obj = unpack_packet(data)
            except Exception:
                self.decode_errors += 1
                continue
            if self.state.update(pkt_id, obj, now=time.monotonic()):
                n += 1
        return n

    def send_command(self, targ_pos, vel_cmd=(0.0, 0.0, 0.0), height_cmd=1.0,
                     deadman=(0, 0, 0)):
        """Send one command packet. Returns True if it left the socket."""
        if self.addr is None and not self.resolve():
            return False
        data = pack_command(targ_pos, vel_cmd, height_cmd, deadman)
        try:
            self._sock.sendto(data, self.addr)
        except OSError:
            return False
        self._last_heartbeat = time.monotonic()  # a command is also a heartbeat
        return True

    def close(self):
        self._sock.close()
