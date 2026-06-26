"""TCP-force estimator (bridge._tcp_force): recover an applied end-effector
force from the joint torques via F = (J·Jᵀ)⁻¹·J·tau over the 3×N position
Jacobian. Pure linear algebra + a real-arm Jacobian sanity check.

    SKT_DIR=.../skt_v3 python -m pytest test/test_force.py
"""

import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_commander.kinematics import ArmKinematics   # noqa: E402
from skate_commander.urdf import parse_urdf             # noqa: E402

SKT = Path(os.environ.get("SKT_DIR", "/tmp/skate_teleop/skt_v3"))


def _estimate(J, tau_arm):
    """Exactly the bridge's estimator: F = (J·Jᵀ)⁻¹·J·tau."""
    JJt = J @ J.T + 1e-9 * np.eye(3)
    return np.linalg.solve(JJt, J @ tau_arm)


def test_recovers_applied_force_synthetic():
    """For any full-rank 3×N Jacobian, tau = Jᵀ·F must invert back to F."""
    rng = np.random.default_rng(0)
    for _ in range(50):
        J = rng.standard_normal((3, 7))          # full row rank almost surely
        F = rng.standard_normal(3) * 10.0
        tau = J.T @ F                            # joint torques an external F produces
        assert np.allclose(_estimate(J, tau), F, atol=1e-6)


def test_ignores_nullspace_torque():
    """Torque in the Jacobian null space (dofs that don't move the TCP) must
    contribute ~zero estimated TCP force."""
    J = np.zeros((3, 7))
    J[0, 0] = J[1, 1] = J[2, 2] = 1.0            # only the first 3 dofs move the TCP
    tau = np.array([0, 0, 0, 7, -4, 2, 9], float)
    assert np.allclose(_estimate(J, tau), [0, 0, 0], atol=1e-9)


def test_real_arm_jacobian_recovery():
    """With the actual SkateArm position Jacobian (3×N), the estimator recovers
    an applied TCP force at a well-conditioned pose."""
    urdf = SKT / "skt_v3.urdf"
    if not urdf.exists():
        pytest.skip("no skt_v3.urdf (set SKT_DIR)")
    model = parse_urdf(urdf)
    rng = np.random.default_rng(1)
    for arm in ("left", "right"):
        kin = ArmKinematics(model, arm)
        q = np.zeros(26)
        q[np.asarray(kin.idx, dtype=int)] = np.linspace(0.2, 0.8, len(kin.idx))
        _, J = kin._fk_jac_fast(q)
        assert J.shape == (3, len(kin.idx))
        if np.linalg.svd(J, compute_uv=False)[-1] < 1e-3:
            continue                             # singular here; skip this arm/pose
        F = rng.standard_normal(3) * 5.0
        tau = J.T @ F
        assert np.allclose(_estimate(J, tau), F, atol=1e-6)
