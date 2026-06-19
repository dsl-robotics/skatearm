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
                 jog_rate=0.35, limits=None, kin=None):
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
        self.kin = kin or {}               # {"left"/"right": ArmKinematics}
        self.guard = None                  # callable(q26)->True if SELF-COLLIDING
        self.guard_blocking = False
        self.ik_targets = {"left": None, "right": None}
        self.ik_err = {"left": None, "right": None}
        self.ik_manip = {"left": None, "right": None}   # singularity awareness
        # cartesian-step targets auto-clear on arrival (or stall); drag-gizmo
        # targets are cleared by the UI on release
        self.ik_auto = {"left": False, "right": False}
        self._ik_prev = {"left": None, "right": None}
        self._ik_stall = {"left": 0.0, "right": 0.0}
        # comfort posture for the IK null space: the documented default pose
        # nudged 30% toward each joint's range center. While an IK target is
        # active the arm continuously relaxes toward this shape WITHOUT
        # moving the TCP — the robot picks its own elbow configuration, no
        # winding, no limit-hugging.
        self.ik_comfort = (0.7 * np.array(names.DEFAULT_POSE)
                           + 0.3 * 0.5 * (self.lo + self.hi))
        # mirror mode: commands to one arm are reflected onto the other.
        # signs/axis are derived numerically from FK by the server (no
        # guessing about the URDF's mirroring convention).
        self.mirror = False
        self.mirror_signs = None           # per-arm-slot sign array, len 8
        self.mirror_axis = 1               # world axis flipped by the mirror
        self.carry = False                 # dual-arm carry: both wrists move one object together
        self.tool_names = {"left": "flange", "right": "flange"}
        # waypoint sequencer
        self.waypoints = []                # list of np[26] (commanded poses)
        self.wp_names = []
        self.recorder = None               # optional teach-in PoseRecorder
        self.seq_active = False           # gliding toward waypoints[seq_idx]
        self.seq_playing = False          # auto-advance through the list
        self.seq_loop = False
        self.seq_idx = None
        self.seq_rate = 0.6               # rad/s glide speed
        self.seq_dwell = 0.6              # s pause on each waypoint
        self._seq_wait = 0.0
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
        self.ik_targets = {"left": None, "right": None}
        self.carry = False
        self.estop = True                  # explicit resume required

    def trigger_estop(self):
        self.estop = True
        self.jog_dir[:] = 0
        self.carry = False
        self.clear_ik_target()
        self.seq_stop()

    def resume(self):
        """Clear the estop latch. Only possible once telemetry is up."""
        if self.link.connected:
            self.estop = False
        return not self.estop

    # -- collision guard -------------------------------------------------------
    def _guard_ok(self, prev):
        """Reject self-colliding targets: revert to ``prev`` if the new targ
        would interpenetrate. Large jumps (slider clicks, goto) are checked
        along the interpolated path so a collision can't be tunnelled through
        between two safe endpoints."""
        if self.guard is None or prev is None:
            return True
        delta = np.abs(self.targ - prev)
        n = max(1, int(np.ceil(float(delta.max()) / 0.05)))
        goal = self.targ.copy()
        for k in range(1, n + 1):
            q = prev + (goal - prev) * (k / n)
            if self.guard(q):
                self.targ = prev
                self.guard_blocking = True
                return False
        self.guard_blocking = False
        return True

    # -- waypoint sequencer ----------------------------------------------------
    def seq_stop(self):
        self.seq_active = False
        self.seq_playing = False
        self._seq_wait = 0.0

    def wp_add(self):
        """Record the current commanded pose as a waypoint."""
        if self.targ is not None:
            self.waypoints.append(self.targ.copy())
            self.wp_names.append(f"WP{len(self.waypoints)}")

    def wp_delete(self, i):
        if 0 <= i < len(self.waypoints):
            self.waypoints.pop(i)
            self.wp_names.pop(i)
            self.seq_stop()
            self.seq_idx = None

    def wp_clear(self):
        self.waypoints = []
        self.wp_names = []
        self.seq_stop()
        self.seq_idx = None

    def wp_goto(self, i):
        """Glide to one waypoint, no auto-advance."""
        if 0 <= i < len(self.waypoints) and self.targ is not None \
                and not self.estop:
            self.seq_idx = i
            self.seq_active = True
            self.seq_playing = False

    def wp_play(self, loop=False):
        if self.waypoints and self.targ is not None and not self.estop:
            self.seq_idx = 0
            self.seq_loop = bool(loop)
            self.seq_active = True
            self.seq_playing = True
            self._seq_wait = 0.0

    def _seq_tick(self, dt):
        """Glide targ toward the active waypoint; advance when playing."""
        if not self.seq_active or self.seq_idx is None \
                or self.seq_idx >= len(self.waypoints):
            return
        wp = self.waypoints[self.seq_idx]
        step = self.seq_rate * dt
        self.targ = np.clip(self.targ + np.clip(wp - self.targ, -step, step),
                            self.lo, self.hi)
        if float(np.max(np.abs(wp - self.targ))) < 0.01:   # arrived
            if not self.seq_playing:
                self.seq_active = False
                return
            self._seq_wait += dt
            if self._seq_wait < self.seq_dwell:
                return
            self._seq_wait = 0.0
            nxt = self.seq_idx + 1
            if nxt >= len(self.waypoints):
                if self.seq_loop:
                    self.seq_idx = 0
                else:
                    self.seq_stop()
            else:
                self.seq_idx = nxt

    # -- mirror mode -----------------------------------------------------------
    def _mirror_joint(self, idx):
        """(other_idx, sign) if idx is an arm joint and mirroring is ready."""
        if self.mirror_signs is None:
            return None
        if 8 <= idx < 16:
            other = idx + 8
        elif 16 <= idx < 24:
            other = idx - 8
        else:
            return None
        return other, float(self.mirror_signs[idx % 8])

    def _mirror_vec(self, v):
        v = np.asarray(v, dtype=float).copy()
        v[self.mirror_axis] = -v[self.mirror_axis]
        return v

    def _joint_locked(self, idx):
        """Lower chain is untouchable in REAL mode — balance belongs to the
        firmware. (The UI greys it out; this enforces it for programs too.)"""
        return self.mode == "real" and 0 <= idx < 8

    # -- jog input from the UI -----------------------------------------------
    def jog_start(self, idx, direction):
        if 0 <= idx < names.N_JOINTS and not self._joint_locked(idx):
            self.seq_stop()                # manual input overrides playback
            d = 1.0 if direction > 0 else -1.0
            self.jog_dir[idx] = d
            if self.mirror and (m := self._mirror_joint(idx)):
                self.jog_dir[m[0]] = m[1] * d

    def jog_stop(self, idx=None):
        if idx is None:
            self.jog_dir[:] = 0
        elif 0 <= idx < names.N_JOINTS:
            self.jog_dir[idx] = 0
            if self.mirror and (m := self._mirror_joint(idx)):
                self.jog_dir[m[0]] = 0

    def jog_step(self, idx, delta):
        """Single click: move target by delta radians."""
        if self.targ is None or self.estop or self._joint_locked(idx):
            return
        self.seq_stop()
        prev = self.targ.copy()
        self.targ[idx] = float(np.clip(self.targ[idx] + delta,
                                       self.lo[idx], self.hi[idx]))
        if self.mirror and (m := self._mirror_joint(idx)):
            o, s = m
            self.targ[o] = float(np.clip(self.targ[o] + s * delta,
                                         self.lo[o], self.hi[o]))
        self._guard_ok(prev)

    def set_joint(self, idx, value):
        """Absolute target for one joint (slider input)."""
        if self.targ is None or self.estop or self._joint_locked(idx):
            return
        self.seq_stop()
        prev = self.targ.copy()
        self.targ[idx] = float(np.clip(value, self.lo[idx], self.hi[idx]))
        if self.mirror and (m := self._mirror_joint(idx)):
            o, s = m
            self.targ[o] = float(np.clip(s * value, self.lo[o], self.hi[o]))
        self._guard_ok(prev)

    def set_joints(self, items):
        """Bulk absolute targets: [(idx, rad), ...] applied as ONE pose with
        a single guard check. Deliberately NOT mirrored — an absolute
        multi-joint pose (e.g. a recorded teach-in keypose) already says
        where every joint goes. Returns False if the guard rejected it."""
        if self.targ is None or self.estop:
            return False
        self.seq_stop()
        prev = self.targ.copy()
        for idx, value in items:
            if 0 <= idx < names.N_JOINTS and not self._joint_locked(idx):
                self.targ[idx] = float(np.clip(value,
                                               self.lo[idx], self.hi[idx]))
        return self._guard_ok(prev)

    def _ik_one(self, arm, pos, auto):
        self.ik_targets[arm] = np.asarray(pos, dtype=float)
        self.ik_auto[arm] = bool(auto)
        self._ik_prev[arm] = None
        self._ik_stall[arm] = 0.0

    def set_ik_target(self, arm, pos, auto=False):
        """Drag-gizmo target (world meters); ignored unless armed & resumed."""
        if (arm in self.kin and self.targ is not None and not self.estop
                and len(pos) == 3):
            self.seq_stop()                # manual input overrides playback
            self._ik_one(arm, pos, auto)
            if self.mirror:
                other = "right" if arm == "left" else "left"
                if other in self.kin:
                    self._ik_one(other, self._mirror_vec(pos), auto)

    def cart_step(self, arm, delta):
        """Cartesian nudge: glide the TCP by ``delta`` (world meters) via IK.
        The target auto-clears once reached (or once it stops improving, e.g.
        out of reach / blocked by the guard)."""
        if (arm not in self.kin or self.targ is None or self.estop
                or len(delta) != 3):
            return
        self.seq_stop()
        delta = np.asarray(delta, dtype=float)
        self._cart_one(arm, delta)
        if self.mirror:
            other = "right" if arm == "left" else "left"
            if other in self.kin:
                self._cart_one(other, self._mirror_vec(delta))

    def _cart_one(self, arm, delta):
        base = self.ik_targets[arm]
        if base is None:
            base = self.kin[arm].fk(self.targ)
        self._ik_one(arm, np.asarray(base, dtype=float) + delta, auto=True)

    def _clear_one(self, arm):
        if arm in self.ik_targets:
            self.ik_targets[arm] = None
            self.ik_err[arm] = None
            self.ik_manip[arm] = None
            self.ik_auto[arm] = False
            self._ik_prev[arm] = None
            self._ik_stall[arm] = 0.0

    def clear_ik_target(self, arm=None):
        if self.mirror:
            arm = None                     # mirrored arms stop together
        for a in ([arm] if arm else ["left", "right"]):
            self._clear_one(a)

    # -- dual-arm carry --------------------------------------------------------
    def carry_grab(self):
        """Hold one virtual object with BOTH wrists: pin each EE at its current
        pose. carry_step then translates both targets together — a rigid
        two-handed carry (not mirrored), at the arms' natural separation."""
        if (self.targ is None or self.estop
                or "left" not in self.kin or "right" not in self.kin):
            return
        self.seq_stop()
        self.mirror = False                # carry drives both arms explicitly
        self.carry = True
        for a in ("left", "right"):
            self._ik_one(a, self.kin[a].fk(self.targ), auto=False)

    def carry_step(self, delta):
        """Translate the held object by ``delta`` (world meters): both wrists
        move by the SAME delta, preserving their separation. Guard-checked in
        tick() like any IK move."""
        if not self.carry or self.targ is None or self.estop or len(delta) != 3:
            return
        self.seq_stop()
        delta = np.asarray(delta, dtype=float)
        for a in ("left", "right"):
            base = self.ik_targets[a]
            if base is None:
                base = self.kin[a].fk(self.targ)
            self._ik_one(a, np.asarray(base, dtype=float) + delta, auto=False)

    def carry_release(self):
        self.carry = False
        self.clear_ik_target()

    # -- tool / TCP offsets ----------------------------------------------------
    def set_tool(self, arm, name, offset_m):
        """Attach a named TCP offset (wrist-link frame, meters) to one arm.
        Any active IK target is cleared — its meaning just changed."""
        if arm in self.kin and len(offset_m) == 3:
            self.kin[arm].tool = np.asarray(offset_m, dtype=float)
            self.tool_names[arm] = str(name)
            self._clear_one(arm)

    def home(self):
        """Glide the target to the documented default pose (handled by the
        position servos; the pose itself is the official safe default)."""
        if self.targ is None or self.estop:
            return
        self.seq_stop()
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
            prev = self.targ.copy()
            if live and self.jog_dir.any():
                self.targ = np.clip(self.targ + self.jog_dir * self.jog_rate * dt,
                                    self.lo, self.hi)
            if live:
                for arm, target in self.ik_targets.items():
                    if target is not None and arm in self.kin:
                        self.targ, err = self.kin[arm].ik_step(
                            self.targ, target, q_ref=self.ik_comfort)
                        self.ik_err[arm] = err
                        self.ik_manip[arm] = self.kin[arm].manipulability(self.targ)
                        if self.ik_auto.get(arm):      # cart-step target
                            p = self._ik_prev[arm]
                            if p is not None and p - err < 1e-5:
                                self._ik_stall[arm] += dt    # not improving
                            else:
                                self._ik_stall[arm] = 0.0
                            self._ik_prev[arm] = err
                            if err < 0.003 or self._ik_stall[arm] > 0.8:
                                self._clear_one(arm)         # arrived/stuck
                self._seq_tick(dt)
                if not np.array_equal(self.targ, prev):
                    self._guard_ok(prev)
            elif self.seq_active:
                self.seq_stop()            # dampened: playback must not resume
            deadman = (1, 1, 1) if live else (0, 0, 0)
            self.link.send_command(self.targ, deadman=deadman)
            self._last_tx = time.monotonic()
        if self.recorder is not None:
            self.recorder.observe(self.targ, dt)
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
            "ik": {a: (None if e is None else round(e, 4))
                   for a, e in self.ik_err.items()},
            "manip": {a: (None if v is None else round(v, 3))
                      for a, v in self.ik_manip.items()},
            "seq": {"n": len(self.waypoints), "names": list(self.wp_names),
                    "idx": self.seq_idx, "playing": self.seq_playing,
                    "loop": self.seq_loop, "active": self.seq_active},
            "guard": {"on": self.guard is not None,
                      "blocking": self.guard_blocking},
            "mirror": self.mirror,
            "carry": self.carry,
            "tools": {a: {"name": self.tool_names.get(a, "flange"),
                          "offset_mm": [round(v * 1000, 1)
                                        for v in self.kin[a].tool]}
                      for a in self.kin},
        }

    def close(self):
        self.link.close()
