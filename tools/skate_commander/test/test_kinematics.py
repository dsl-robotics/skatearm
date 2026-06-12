"""FK must match MuJoCo exactly; DLS IK must converge inside joint limits.

Needs mujoco + the official clone:
    SKT_DIR=.../skt_v3 SKATE_MJCF=.../skt_v3_control.xml python3 test_kinematics.py
"""

import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_commander.kinematics import ArmKinematics  # noqa: E402
from skate_commander.urdf import parse_urdf           # noqa: E402

SKT = Path(os.environ.get("SKT_DIR", "/tmp/skate_teleop/skt_v3"))
MJCF = os.environ.get("SKATE_MJCF", str(SKT / "skt_v3_control.xml"))


def _clamped_random(model, rng, scale=0.5):
    q = rng.uniform(-scale, scale, 26)
    for j in model["joints"]:
        if j["index"] is not None and j["lower"] is not None:
            q[j["index"]] = np.clip(q[j["index"]], j["lower"], j["upper"])
    return q


def test_fk_matches_mujoco_and_ik_converges():
    try:
        import mujoco
    except ImportError:
        print("SKIP: mujoco not installed"); return
    if not Path(MJCF).exists():
        print("SKIP: no control model"); return
    model = parse_urdf(SKT / "skt_v3.urdf")
    mm = mujoco.MjModel.from_xml_path(MJCF)
    dd = mujoco.MjData(mm)
    rng = np.random.default_rng(3)

    for arm in ("left", "right"):
        kin = ArmKinematics(model, arm)
        ee_link = next(j for j in model["joints"]
                       if j["index"] == kin.idx[-1])["child"]
        bid = mm.body(ee_link).id
        worst = 0.0
        for _ in range(5):
            q = _clamped_random(model, rng)
            dd.qpos[:26] = q
            mujoco.mj_forward(mm, dd)
            worst = max(worst, float(np.linalg.norm(dd.xpos[bid] - kin.fk(q))))
        assert worst < 1e-6, f"{arm}: FK diverges from MuJoCo ({worst} m)"

        ok = 0
        for _ in range(8):
            target = kin.fk(_clamped_random(model, rng, 0.8))
            q = np.zeros(26)
            err = 1.0
            for _ in range(300):
                q, err = kin.ik_step(q, target)
                if err < 0.005:
                    break
            ok += err < 0.005
            for k, i in enumerate(kin.idx):
                assert kin.lo[k] - 1e-9 <= q[i] <= kin.hi[k] + 1e-9
        assert ok >= 7, f"{arm}: IK converged only {ok}/8"
        print(f"PASS {arm}: FK exact, IK {ok}/8")


def test_tool_offset_tracks_mujoco():
    """FK with a TCP offset must equal MuJoCo xpos + xmat @ tool, and the
    IK must drive the OFFSET point (not the wrist) onto the target."""
    try:
        import mujoco
    except ImportError:
        print("SKIP: mujoco not installed"); return
    if not Path(MJCF).exists():
        print("SKIP: no control model"); return
    model = parse_urdf(SKT / "skt_v3.urdf")
    mm = mujoco.MjModel.from_xml_path(MJCF)
    dd = mujoco.MjData(mm)
    rng = np.random.default_rng(7)
    tool = np.array([0.02, -0.015, 0.12])     # an off-axis 12 cm "tool"

    for arm in ("left", "right"):
        kin = ArmKinematics(model, arm)
        kin.tool = tool
        ee_link = next(j for j in model["joints"]
                       if j["index"] == kin.idx[-1])["child"]
        bid = mm.body(ee_link).id
        worst = 0.0
        for _ in range(5):
            q = _clamped_random(model, rng)
            dd.qpos[:26] = q
            mujoco.mj_forward(mm, dd)
            want = dd.xpos[bid] + dd.xmat[bid].reshape(3, 3) @ tool
            worst = max(worst, float(np.linalg.norm(want - kin.fk(q))))
        assert worst < 1e-6, f"{arm}: tool FK diverges ({worst} m)"

        target = kin.fk(_clamped_random(model, rng, 0.7))
        q = np.zeros(26)
        err = 1.0
        for _ in range(300):
            q, err = kin.ik_step(q, target)
            if err < 0.005:
                break
        assert err < 0.005, f"{arm}: tool IK did not converge ({err} m)"
        print(f"PASS {arm}: tool-offset FK exact ({worst:.2e} m), IK converges")


def test_posture_hold_no_winding():
    """Jogging the TCP out and back must return (almost) the same joint
    pose. Without the null-space posture anchor the 4 redundant DoF drift
    and the arm slowly winds itself up (the v0.5.0 'выкручивает' bug)."""
    if not Path(MJCF).exists():
        print("SKIP: no control model"); return
    model = parse_urdf(SKT / "skt_v3.urdf")
    import numpy as np

    for arm in ("left", "right"):
        kin = ArmKinematics(model, arm)
        q0 = np.zeros(26)
        q0[11] = q0[19] = np.radians(90)            # elbows-bent home
        p0 = kin.fk(q0)

        def roundtrip(q_ref):
            q = q0.copy()
            for tgt in (p0 + [0, 0.15, 0], p0 + [0, 0, 0.12], p0):
                for _ in range(250):
                    q, err = kin.ik_step(q, tgt, q_ref=q_ref)
                    if err < 1e-3:
                        break
            return float(np.max(np.abs(q - q0)))

        anchored = roundtrip(q0)
        free = roundtrip(None)
        assert anchored < 0.06, \
            f"{arm}: wound up {np.degrees(anchored):.1f} deg with anchor"
        print(f"PASS {arm}: out-and-back posture drift "
              f"{np.degrees(anchored):.2f} deg (anchored) vs "
              f"{np.degrees(free):.1f} deg (free)")


if __name__ == "__main__":
    test_fk_matches_mujoco_and_ik_converges()
    test_tool_offset_tracks_mujoco()
    test_posture_hold_no_winding()
