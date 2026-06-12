"""Program runner — Waldo-style Python programs over the safe bridge.

User programs run on a worker thread in a tiny sandboxed namespace: ``rbt``
(the robot API below), ``math`` and a whitelist of builtins. Every motion
call goes through the SAME RobotBridge paths as the UI — joint limits, the
collision guard, E-STOP, overtemp and the manual-input-interrupts rule all
apply. (A convenience sandbox for a local tool, not a security boundary.)

Click-to-Step: started in step mode the runner pauses BEFORE every motion
command and reports the next command + source line; each STEP executes
exactly one command, RUN releases the program to free-run.

API (joints in degrees, cartesian in millimeters, world axes):
    rbt.movej(joint, deg)        joint = "L4" / "R2" / "H1", URDF name or index
    rbt.movel(arm, dx=, dy=, dz=)  glide the TCP; auto-stops when blocked
    rbt.home()                   glide to the documented safe pose
    rbt.gripper(arm, deg)        open/close one gripper
    rbt.waypoint(i_or_name)      glide to a recorded sequencer pose
    rbt.wait(seconds)            dwell
    rbt.tcp(arm) / rbt.q() / rbt.status()   readouts
"""

from __future__ import annotations

import builtins
import math
import sys
import threading
import time
import traceback

from skate_ros2 import names

PROGRAM_FILE = "<skate program>"
LOG_MAX = 300

_SAFE = {n: getattr(builtins, n)
         for n in ("abs", "min", "max", "round", "range", "len", "enumerate",
                   "zip", "float", "int", "str", "bool", "list", "dict",
                   "tuple", "sorted", "sum", "reversed", "divmod", "repr",
                   "Exception", "ValueError", "StopIteration", "isinstance")}


class ProgramStopped(Exception):
    pass


def _resolve_joint(joint):
    """Accept "L1".."L8" / "R1".."R8" / "H1","H2", a URDF name, or an index."""
    if isinstance(joint, int):
        if 0 <= joint < names.N_JOINTS:
            return joint
        raise ValueError(f"joint index {joint} out of range")
    s = str(joint).strip()
    if s in names.INDEX:
        return names.INDEX[s]
    base = {"L": 8, "R": 16, "H": 24}.get(s[:1].upper())
    if base is not None and s[1:].isdigit():
        n = int(s[1:])
        limit = 2 if base == 24 else 8
        if 1 <= n <= limit:
            return base + n - 1
    raise ValueError(f"unknown joint {joint!r} — use L1..L8 / R1..R8 / "
                     "H1..H2, a URDF joint name, or a protocol index")


class RobotAPI:
    """The ``rbt`` object visible to user programs."""

    def __init__(self, runner):
        self._r = runner

    # -- readouts (no gate: free to call anywhere) ---------------------------
    def q(self):
        """All 26 commanded joint angles, degrees."""
        t = self._r.bridge.targ
        return ([round(float(math.degrees(v)), 2) for v in t]
                if t is not None else None)

    def tcp(self, arm="right"):
        """Current commanded TCP position, world millimeters."""
        br = self._r.bridge
        if arm not in br.kin or br.targ is None:
            return None
        p = br.kin[arm].fk(br.targ)
        return tuple(round(float(v) * 1000, 1) for v in p)

    def status(self):
        return self._r.bridge.snapshot()

    # -- motion (gated: honors Click-to-Step / STOP / E-STOP) ----------------
    def movej(self, joint, deg, tol=3.0, timeout=10.0):
        idx = _resolve_joint(joint)
        self._r._gate(f"movej({joint!r}, {deg:g})")
        br = self._r.bridge
        br.set_joint(idx, math.radians(deg))
        goal = None if br.targ is None else float(br.targ[idx])
        if goal is None or (abs(goal - math.radians(deg)) > 1e-6
                            and br.guard_blocking):
            self._r.emit(f"! movej {joint}: blocked by the collision guard")
            return False
        ok = self._r._wait_until(
            lambda: (q := br.link.state.dof_pos()) is not None
                    and abs(q[idx] - goal) < math.radians(tol), timeout)
        if not ok:
            self._r.emit(f"! movej {joint}: timeout (still "
                         f"tracking toward {deg:g} deg)")
        return ok

    def movel(self, arm, dx=0.0, dy=0.0, dz=0.0, timeout=15.0):
        self._r._gate(f"movel({arm!r}, {dx:g}, {dy:g}, {dz:g})")
        br = self._r.bridge
        br.cart_step(arm, [dx / 1000.0, dy / 1000.0, dz / 1000.0])
        if br.ik_targets.get(arm) is None:
            self._r.emit(f"! movel {arm}: rejected (not armed / estopped?)")
            return False
        self._r._wait_until(lambda: br.ik_targets.get(arm) is None, timeout)
        return self.tcp(arm)

    def home(self, timeout=12.0):
        self._r._gate("home()")
        br = self._r.bridge
        br.home()
        import numpy as np
        goal = np.array(names.DEFAULT_POSE)
        ok = self._r._wait_until(
            lambda: (q := br.link.state.dof_pos()) is not None
                    and float(np.max(np.abs(np.asarray(q) - goal))) < 0.06,
            timeout)
        if not ok:
            self._r.emit("! home(): timeout while tracking")
        return ok

    def gripper(self, arm, deg, timeout=5.0):
        idx = {"left": names.LEFT_GRIPPER, "right": names.RIGHT_GRIPPER}.get(arm)
        if idx is None:
            raise ValueError("gripper arm must be 'left' or 'right'")
        return self.movej(idx, deg, tol=4.0, timeout=timeout)

    def waypoint(self, which, timeout=30.0):
        """Glide to a recorded sequencer pose (1-based index or name)."""
        br = self._r.bridge
        if isinstance(which, str):
            if which not in br.wp_names:
                raise ValueError(f"no waypoint named {which!r}")
            i = br.wp_names.index(which)
        else:
            i = int(which) - 1
        self._r._gate(f"waypoint({which!r})")
        br.wp_goto(i)
        if not br.seq_active:
            self._r.emit(f"! waypoint {which}: rejected (empty list / "
                         "estopped?)")
            return False
        return self._r._wait_until(lambda: not br.seq_active, timeout)

    def wait(self, seconds):
        self._r._gate(f"wait({seconds:g})")
        end = time.monotonic() + float(seconds)
        while time.monotonic() < end:
            self._r._check()
            time.sleep(0.02)
        return True


class ProgramRunner:
    def __init__(self, bridge):
        self.bridge = bridge
        self.lock = threading.Lock()
        self.code = ""
        self.running = False
        self.paused = False
        self.step_mode = False
        self.line = None
        self.counter = 0
        self.current = None
        self.log = []
        self._thread = None
        self._stop = False
        self._stop_reason = ""
        self._step_evt = threading.Event()

    # -- control ---------------------------------------------------------------
    def run(self, code=None, step=False):
        with self.lock:
            if self.running:
                # RUN releases a stepping program; STEP keeps it stepping
                self.step_mode = step
                self._step_evt.set()
                return True
            if code is not None:
                self.code = str(code)
            try:
                obj = compile(self.code, PROGRAM_FILE, "exec")
            except SyntaxError as e:
                self.log = [f"x syntax error, line {e.lineno}: {e.msg}"]
                return False
            self.running = True
            self.paused = False
            self.step_mode = bool(step)
            self.counter = 0
            self.line = None
            self.current = None
            self.log = [("> program started (Click-to-Step)" if step
                         else "> program started")]
            self._stop = False
            self._stop_reason = ""
            self._step_evt.clear()
            self._thread = threading.Thread(target=self._main, args=(obj,),
                                            daemon=True)
            self._thread.start()
            return True

    def step(self, code=None):
        with self.lock:
            if self.running:
                self.step_mode = True
                self._step_evt.set()
                return True
        return self.run(code, step=True)

    def stop(self, reason=""):
        if self.running:
            self._stop_reason = reason
            self._stop = True
            self._step_evt.set()

    # -- worker ------------------------------------------------------------------
    def _main(self, obj):
        api = RobotAPI(self)
        env = {"rbt": api, "math": math,
               "__builtins__": dict(_SAFE, print=self._print),
               "__name__": "__main__"}
        try:
            exec(obj, env)
            self.emit("* program finished")
        except ProgramStopped:
            r = self._stop_reason
            self.emit(f"# stopped{' — ' + r if r else ''}")
        except Exception as e:
            ln = None
            for fr in traceback.extract_tb(sys.exc_info()[2]):
                if fr.filename == PROGRAM_FILE:
                    ln = fr.lineno
            where = f", line {ln}" if ln else ""
            self.emit(f"x {type(e).__name__}{where}: {e}")
        finally:
            self.bridge.seq_stop()
            self.bridge.clear_ik_target()
            with self.lock:
                self.running = False
                self.paused = False
                self.current = None

    def _print(self, *a, **k):
        self.emit(" ".join(str(x) for x in a))

    def emit(self, line):
        with self.lock:
            self.log.append(str(line))
            del self.log[:-LOG_MAX]

    # -- gating ------------------------------------------------------------------
    def _check(self):
        if self._stop:
            raise ProgramStopped(self._stop_reason)
        if self.bridge.estop:
            self._stop_reason = self._stop_reason or "E-STOP"
            raise ProgramStopped("E-STOP")

    def _user_line(self):
        f = sys._getframe(2)
        while f is not None:
            if f.f_code.co_filename == PROGRAM_FILE:
                return f.f_lineno
            f = f.f_back
        return None

    def _gate(self, desc):
        self._check()
        self.counter += 1
        self.current = desc
        self.line = self._user_line()
        if self.step_mode:
            self.paused = True
            try:
                while not self._step_evt.wait(0.1):
                    self._check()
            finally:
                self._step_evt.clear()
                self.paused = False
            self._check()

    def _wait_until(self, pred, timeout):
        end = time.monotonic() + float(timeout)
        while time.monotonic() < end:
            self._check()
            try:
                if pred():
                    return True
            except Exception:
                pass
            time.sleep(0.02)
        return False

    # -- state for the UI ----------------------------------------------------------
    def snapshot(self):
        with self.lock:
            return {"running": self.running, "paused": self.paused,
                    "step": self.step_mode, "line": self.line,
                    "n": self.counter, "current": self.current,
                    "log": list(self.log[-40:])}
