"""IBVS robustness: with a deliberately mis-calibrated BELIEVED camera model,
closed-loop image servoing still drives the EE onto the target, while one-shot
open-loop back-projection (same wrong model) misses badly.

Needs mujoco + Pillow + the official clone. Drives the real bridge + the
CameraStreamer against the skate_ros2 MuJoCo sim endpoint:

    SKT_DIR=.../skt_v3 SKATE_MJCF=.../skt_v3_control.xml python3 test_ibvs.py
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
MJCF = os.environ.get("SKATE_MJCF", str(SKT / "skt_v3_control.xml"))
DT = 1 / 60.0
CUBE_XY = np.array([0.13, 0.35])          # true target in the generated scene


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close()
    return p


def test_ibvs_robust_to_miscalibration():
    try:
        import mujoco  # noqa: F401
        import PIL      # noqa: F401
    except ImportError:
        print("SKIP: mujoco / Pillow not installed"); return
    if not Path(MJCF).exists():
        print("SKIP: no control model"); return

    from skate_commander import camera, vision
    from skate_commander.bridge import RobotBridge
    from skate_commander.ibvs import ImageServo
    from skate_commander.kinematics import ArmKinematics
    from skate_commander.urdf import joint_limits, parse_urdf
    from skate_ros2.sim_endpoint import SkateSimEndpoint

    model = parse_urdf(SKT / "skt_v3.urdf")
    kin = {a: ArmKinematics(model, a) for a in ("left", "right")}
    port = _free_port()
    ep = SkateSimEndpoint(MJCF, port=port, bind="127.0.0.1", verbose=False)
    threading.Thread(target=ep.run, kwargs={"duration": 60.0},
                     daemon=True).start()
    br = RobotBridge(sim_host="127.0.0.1", sim_port=port,
                     limits=joint_limits(model), kin=kin)
    scene = camera.build_scene_xml(str(SKT))
    cam = camera.CameraStreamer(scene,
                                lambda: (None if br.targ is None else list(br.targ)))

    def spin(secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            br.tick(DT, ui_attached=True); time.sleep(DT)

    def track(secs=0.5):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            br.tick(DT, ui_attached=True); time.sleep(DT)
            if br.ik_targets.get("right") is None:
                break

    spin(0.7); br.resume()
    kr = kin["right"]

    # detect the static target ONCE while the arm is clear of it
    obs = cam.detect(ee_world=list(kr.fk(br.targ)))
    assert obs.get("found") and "cam" in obs, obs
    s_cube = np.array(obs["pixel"])
    C = obs["cam"]
    f0, cx0, cy0 = vision.intrinsics(C["fovy"], C["W"], C["H"])
    pos0 = np.array(C["pos"]); mat0 = np.array(C["mat"]).reshape(3, 3)

    def ee_pixel():
        return np.array(vision.project(kr.fk(br.targ), pos0, mat0,
                                       f0, cx0, cy0)[:2])

    approach_z = camera.GRASP_Z + 0.08
    br.set_ik_target("right", [0.10, 0.32, approach_z], auto=True)
    track(3.0)

    # deliberately MIS-CALIBRATED believed model (3 cm offset, 12% wrong fovy)
    bad_pos = pos0 + np.array([0.03, 0.0, 0.0])
    bad_fovy = C["fovy"] * 1.12
    servo = ImageServo(bad_pos, mat0, bad_fovy, C["W"], C["H"],
                       gain=0.7, max_step=0.03)

    # open-loop baseline: back-project the cube pixel through the bad model
    fb, cxb, cyb = vision.intrinsics(bad_fovy, C["W"], C["H"])
    P_ol = vision.backproject(s_cube[0], s_cube[1], camera.GRASP_Z,
                              bad_pos, mat0, fb, cxb, cyb)
    open_err = float(np.linalg.norm(P_ol[:2] - CUBE_XY))

    # IBVS closed loop: drive the gripper pixel onto s_cube while descending
    z = approach_z
    err_px = 999.0
    it = 0
    for it in range(45):
        dxy, err_px = servo.step(s_cube, ee_pixel(), kr.fk(br.targ))
        z = max(camera.GRASP_Z + 0.03, z - 0.006)
        ee = kr.fk(br.targ)
        br.set_ik_target("right", [ee[0] + dxy[0], ee[1] + dxy[1], z],
                         auto=True)
        track(0.5)
        if err_px < 3.0 and z <= camera.GRASP_Z + 0.035:
            break
    ibvs_err = float(np.linalg.norm(kr.fk(br.targ)[:2] - CUBE_XY))

    assert open_err > 0.020, f"perturbation too small ({open_err*1000:.1f}mm)"
    assert ibvs_err < 0.012, f"IBVS did not converge ({ibvs_err*1000:.1f}mm)"
    assert ibvs_err < open_err / 2.5, "IBVS not clearly better than open-loop"
    assert err_px < 6.0, f"image error not nulled ({err_px:.1f}px)"
    print(f"PASS IBVS: open-loop miss {open_err*1000:.1f} mm vs closed-loop "
          f"{ibvs_err*1000:.1f} mm ({open_err/ibvs_err:.1f}x), image err "
          f"{err_px:.1f}px in {it+1} iters")
    cam.close(); br.close(); ep.close()


if __name__ == "__main__":
    test_ibvs_robust_to_miscalibration()
