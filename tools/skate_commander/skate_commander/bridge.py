"""RobotBridge — the commander's connection to a Skate (sim or real).

Wraps :class:`skate_ros2.protocol.SkateLink` with the same safety model the
skate_ros2 driver uses, plus a jog engine for the UI:

* SIM/REAL are the same wire — switching mode just retargets the UDP host
  (127.0.0.1 vs r.local). After ANY mode switch the bridge is DAMPENED and
  must be explicitly resumed from the UI.
* Arms at the robot's measured pose; jog input before arming is ignored.
* Hold-to-jog integrates at the tx rate, clamped to URDF joint limits.
* Deadman (1,1,1) only while a UI client is attached, resumed, and not
  estopped/overtemp — close the browser tab and the robot dampens itself.

No ROS, no FastAPI here: pure logic, fully testable headless.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

# prototype-phase import of the sibling tool (documented in README)
_SKATE_ROS2 = Path(__file__).resolve().parents[2] / "skate_ros2"
if str(_SKATE_ROS2) not in sys.path:
    sys.path.insert(0, str(_SKATE_ROS2))

from skate_ros2 import names                    # noqa: E402
from skate_ros2.protocol import SkateLink       # noqa: E402

OVERTEMP_C = 58.0          # PETG limit, per official docs
OVERTEMP_RELEASE_C = 53.0


class RobotBridge:
    def __init__(self, sim_host="127.0.0.1", sim_port=2000,
                 real_host="r.local", real_port=2000,
                 jog_rate=0.35, limits=None):
        """jog_rate: rad/s while a jog button is held.
        limits: optional (lo[26], hi[26]) from the URDF; clamps jog targets."""
        self.sim_addr = (sim_host, sim_port)
        self.real_addr = (real_host, real_port)
        self.mode = "sim"                  # "sim" | "real"
        self.link = SkateLink(*self.sim_addr)
        self.jog_rate = float(jog_rate)
        if limits is not None:
            self.lo = np.asarray(limits[0], dtype=float)
            self.hi = np.asarray(limits[1], dtype=float)
        else:
            self.lo = np.full(names.N_JOINTS, -np.pi)
            self.hi = np.full(names.N_JOINTS, np.pi)

        self.targ = None                   # armed target (np[26]) or None
        self.estop = True                  # latched; starts SAFE (dampened)
        self.overtemp = False
        self.jog_dir = np.zeros(names.N_JOINTS)   # -1/0/+1 per joint
        self._last_tx = None

    # -- mode / safety -------------------------------------------------------
    def set_mode(self, mode):
        if mode not in ("sim", "real"):
            raise ValueError(mode)
        if mode == self.mode:
            return
        self.link.close()
        addr = self.sim_addr if mode == "sim" else self.real_addr
        self.link = SkateLink(*addr)
        self.mode = mode
        self.targ = None                   # re-arm from the new robot's pose
        self.jog_dir[:] = 0
        self.estop = True                  # explicit resume required

    def trigger_estop(self):
        self.estop = True
        self.jog_dir[:] = 0

    def resume(self):
        """Clear the estop latch. Only possible once telemetry is up."""
        if self.link.connected:
            self.estop = False
        return not self.estop

    # -- jog input from the UI -----------------------------------------------
    def jog_start(self, idx, direction):
        if 0 <= idx < names.N_JOINTS:
            self.jog_dir[idx] = 1.0 if direction > 0 else -1.0

    def jog_stop(self, idx=None):
        if idx is None:
            self.jog_dir[:] = 0
        elif 0 <= idx < names.N_JOINTS:
            self.jog_dir[idx] = 0

    def jog_step(self, idx, delta):
        """Single click: move target by delta radians."""
        if self.targ is None or self.estop:
            return
        self.targ[idx] = float(np.clip(self.targ[idx] + delta,
                                       self.lo[idx], self.hi[idx]))

    def set_joint(self, idx, value):
        """Absolute target for one joint (slider input)."""
        if self.targ is None or self.estop:
            return
        self.targ[idx] = float(np.clip(value, self.lo[idx], self.hi[idx]))

    def home(self):
        """Glide the target to the documented default pose (handled by the
        position servos; the pose itself is the official safe default)."""
        if self.targ is None or self.estop:
            return
        self.targ = np.array(names.DEFAULT_POSE)

    # -- periodic work (call at ~60 Hz from the server loop) -------------------
    def tick(self, dt, ui_attached):
        """Poll telemetry, integrate jog, send one command. Returns state."""
        self.link.poll()
        st = self.link.state

        pos = st.dof_pos()
        if pos is not None and self.targ is None:
            self.targ = np.asarray(pos, dtype=float).copy()  # arm at pose

        temps = st.motor_temps()
        if temps is not None:
            tmax = max(temps)
            if not self.overtemp and tmax > OVERTEMP_C:
                self.overtemp = True
            elif self.overtemp and tmax < OVERTEMP_RELEASE_C:
                self.overtemp = False

        live = (ui_attached and not self.estop and not self.overtemp
                and self.targ is not None and self.link.connected)

        if self.targ is not None:
            if live and self.jog_dir.any():
                self.targ = np.clip(self.targ + self.jog_dir * self.jog_rate * dt,
                                    self.lo, self.hi)
            deadman = (1, 1, 1) if live else (0, 0, 0)
            self.link.send_command(self.targ, deadman=deadman)
            self._last_tx = time.monotonic()
        return self.snapshot(ui_attached)

    # -- state for the UI ------------------------------------------------------
    def snapshot(self, ui_attached=True):
        st = self.link.state
        return {
            "mode": self.mode,
            "connected": self.link.connected,
            "armed": self.targ is not None,
            "estop": self.estop,
            "overtemp": self.overtemp,
            "live": (ui_attached and not self.estop and not self.overtemp
                     and self.targ is not None and self.link.connected),
            "q": st.dof_pos(),
            "dq": st.dof_vel(),
            "tau": st.dof_torque(),
            "temps": st.motor_temps(),
            "targ": None if self.targ is None else self.targ.tolist(),
        }

    def close(self):
        self.link.close()
