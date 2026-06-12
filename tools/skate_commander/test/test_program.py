"""Program runner e2e: the rbt API drives the real bridge over UDP into the
MuJoCo sim endpoint; Click-to-Step, STOP, E-STOP and the sandbox all hold.

    SKATE_MJCF=.../skt_v3_control.xml python3 test/test_program.py
"""

import math
import os
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skate_ros2"))

from skate_commander.bridge import RobotBridge          # noqa: E402
from skate_commander.program import ProgramRunner       # noqa: E402

MODEL = os.environ.get("SKATE_MJCF",
                       "/tmp/skate_teleop/skt_v3/skt_v3_control.xml")


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close()
    return p


class _Rig:
    """Sim endpoint + bridge + 60 Hz tick thread, like the server runs."""

    def __init__(self):
        from skate_ros2.sim_endpoint import SkateSimEndpoint
        port = _free_port()
        self.ep = SkateSimEndpoint(MODEL, port=port, bind="127.0.0.1",
                                   verbose=False)
        self.epth = threading.Thread(target=self.ep.run,
                                     kwargs={"duration": 120.0}, daemon=True)
        self.epth.start()
        urdf = Path(MODEL).parent / "skt_v3.urdf"
        kin, limits = {}, None
        if urdf.exists():
            from skate_commander.kinematics import ArmKinematics
            from skate_commander.urdf import joint_limits, parse_urdf
            model = parse_urdf(urdf)
            kin = {a: ArmKinematics(model, a) for a in ("left", "right")}
            limits = joint_limits(model)
        self.br = RobotBridge(sim_host="127.0.0.1", sim_port=port,
                              limits=limits, kin=kin)
        self._stop = threading.Event()
        self.tick = threading.Thread(target=self._loop, daemon=True)
        self.tick.start()
        t0 = time.monotonic()
        while self.br.targ is None and time.monotonic() - t0 < 5:
            time.sleep(0.05)
        assert self.br.targ is not None, "bridge never armed"
        self.br.resume()

    def _loop(self):
        while not self._stop.is_set():
            self.br.tick(1 / 60, ui_attached=True)
            time.sleep(1 / 60)

    def close(self):
        self._stop.set()
        self.tick.join(timeout=2)
        self.br.close()
        self.ep.close()


def _wait(pred, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_program_runs_moves_and_logs():
    if not Path(MODEL).exists():
        print("SKIP: no control model"); return
    rig = _Rig()
    r = ProgramRunner(rig.br)
    code = "\n".join([
        "rbt.movej('L4', 60)",
        "rbt.movel('right', dz=40)",
        "print('tcp:', rbt.tcp('right'))",
        "rbt.wait(0.2)",
    ])
    assert r.run(code)
    assert _wait(lambda: not r.running, 30), "program never finished"
    log = "\n".join(r.log)
    assert "* program finished" in log, log
    assert "tcp:" in log
    assert abs(rig.br.targ[11] - math.radians(60)) < 0.02
    print("PASS program ran: movej + movel + print")
    rig.close()


def test_click_to_step_and_stop():
    if not Path(MODEL).exists():
        print("SKIP: no control model"); return
    rig = _Rig()
    r = ProgramRunner(rig.br)
    code = "rbt.movej('L4', 30)\nrbt.movej('L4', 70)\nrbt.wait(30)"
    assert r.run(code, step=True)
    assert _wait(lambda: r.paused, 5), "did not pause at the first command"
    assert r.line == 1 and "movej" in r.current
    before = float(rig.br.targ[11])
    time.sleep(0.4)                      # paused = nothing moves
    assert abs(rig.br.targ[11] - before) < 1e-9
    r.step()                             # execute command 1, pause at 2
    assert _wait(lambda: r.paused and r.line == 2, 15)
    assert abs(rig.br.targ[11] - math.radians(30)) < 0.02
    print("PASS Click-to-Step: paused, stepped exactly one command")
    r.step()                             # command 2 runs...
    assert _wait(lambda: r.paused and r.line == 3, 15)
    r.run()                              # ...RUN releases into wait(30)
    time.sleep(0.3)
    assert r.running and not r.paused
    r.stop("test")
    assert _wait(lambda: not r.running, 5), "STOP did not interrupt wait()"
    assert any("stopped" in ln for ln in r.log)
    print("PASS STOP interrupts a long wait")
    rig.close()


def test_estop_kills_program_and_sandbox_holds():
    if not Path(MODEL).exists():
        print("SKIP: no control model"); return
    rig = _Rig()
    r = ProgramRunner(rig.br)
    assert r.run("rbt.wait(30)")
    time.sleep(0.3)
    rig.br.trigger_estop()
    assert _wait(lambda: not r.running, 5), "E-STOP did not kill the program"
    assert any("E-STOP" in ln for ln in r.log)
    print("PASS E-STOP aborts a running program")

    rig.br.resume()
    r2 = ProgramRunner(rig.br)
    assert r2.run("import os\nprint(os.getcwd())")
    assert _wait(lambda: not r2.running, 5)
    assert any(ln.startswith("x ") for ln in r2.log), r2.log
    assert r2.run("open('x.txt', 'w')")
    assert _wait(lambda: not r2.running, 5)
    assert any("NameError" in ln for ln in r2.log), r2.log
    print("PASS sandbox: import / open are not available")

    bad = ProgramRunner(rig.br)
    assert not bad.run("def broken(:\n  pass")
    assert any("syntax error" in ln for ln in bad.log)
    print("PASS syntax errors are reported with a line number")
    rig.close()


if __name__ == "__main__":
    test_program_runs_moves_and_logs()
    test_click_to_step_and_stop()
    test_estop_kills_program_and_sandbox_holds()
    print("ALL PROGRAM-RUNNER E2E GREEN")
