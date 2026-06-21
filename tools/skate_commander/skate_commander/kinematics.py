"""Pure-numpy kinematics for the Skate arms — FK, numeric Jacobian, DLS IK.

No MuJoCo dependency: forward kinematics is computed straight from the parsed
URDF (the same math the browser viewer uses; validated against MuJoCo link
positions to < 1e-6 m). The Jacobian is central-difference numeric — at 7
joints per arm and 20-60 Hz it is far below the cost of anything else.

The IK is deliberately conservative for teleop:
* position-only (3 DoF target), wrist orientation is free;
* one damped-least-squares step per call, error capped at ``step_m`` and
  joint motion capped at ``dq_max`` — the arm *glides* toward the target;
* joint limits clamped every step.

Tool (TCP) offsets: ``self.tool`` is a 3-vector in the wrist-link frame
(meters). FK returns the offset point and the numeric Jacobian/IK follow it
automatically — switch tools and every cartesian feature speaks TCP.
"""

from __future__ import annotations

import numpy as np

ARM_JOINTS = {"left": list(range(8, 15)),    # a0..a6 of the left arm
              "right": list(range(16, 23))}  # gripper (a7) excluded


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rpy(r, p, y):                      # URDF fixed-axis == intrinsic ZYX
    return _rz(y) @ _ry(p) @ _rx(r)


def _axis_rot(ax, th):
    ax = np.asarray(ax, float)
    ax = ax / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


class ArmKinematics:
    """FK/IK for one arm chain of the skt_v3 model."""

    def __init__(self, model, arm):
        if arm not in ARM_JOINTS:
            raise ValueError(arm)
        self.arm = arm
        self.idx = ARM_JOINTS[arm]
        self.tool = np.zeros(3)            # TCP offset, wrist-link frame (m)
        # chain of joints from root to the arm's last wrist joint (a6)
        joints = {j["child"]: j for j in model["joints"]}
        last = next(j for j in model["joints"] if j["index"] == self.idx[-1])
        chain = [last]
        while chain[0]["parent"] in joints:
            chain.insert(0, joints[chain[0]["parent"]])
        self.chain = chain                       # root -> ... -> a6
        # NB: limits can legitimately be 0.0 (the elbow's lower bound!) —
        # `x or default` would silently replace it, so test for None.
        def _lim(v, default):
            return default if v is None else v
        self.lo = np.array([_lim(next(j for j in model["joints"]
                                      if j["index"] == i)["lower"], -np.pi)
                            for i in self.idx])
        self.hi = np.array([_lim(next(j for j in model["joints"]
                                      if j["index"] == i)["upper"], np.pi)
                            for i in self.idx])

    def fk(self, q26):
        """World position of the TCP (wrist-link origin + tool offset)."""
        x = np.zeros(3)
        R = np.eye(3)
        for j in self.chain:
            x = x + R @ np.asarray(j["xyz"])
            Rj = _rpy(*j["rpy"])
            if j["index"] is not None:
                Rj = Rj @ _axis_rot(j["axis"], q26[j["index"]])
            R = R @ Rj
        return x + R @ self.tool

    def jacobian(self, q26, eps=1e-5):
        """3x7 numeric Jacobian of the wrist position wrt this arm's joints."""
        J = np.zeros((3, len(self.idx)))
        q = np.array(q26, dtype=float)
        for k, i in enumerate(self.idx):
            q[i] += eps
            p_hi = self.fk(q)
            q[i] -= 2 * eps
            p_lo = self.fk(q)
            q[i] += eps
            J[:, k] = (p_hi - p_lo) / (2 * eps)
        return J

    def manipulability(self, q26):
        """Reciprocal condition number of the position Jacobian in [0, 1]:
        1 = isotropic, → 0 near a singularity (some cartesian direction needs
        huge joint speed for a small wrist move). A cheap teleop warning."""
        s = np.linalg.svd(self.jacobian(q26), compute_uv=False)
        return float(s[-1] / s[0]) if s[0] > 1e-12 else 0.0

    def _fk_jac_fast(self, q26):
        """One forward pass returning (TCP position, 3x7 position Jacobian wrt
        this arm's joints) via the geometric (axis x lever) Jacobian — ~15x
        cheaper than the central-difference jacobian(), for bulk manipulability
        sampling (matches it to ~1e-6)."""
        x = np.zeros(3)
        R = np.eye(3)
        axes, origins = [], []
        for j in self.chain:
            x = x + R @ np.asarray(j["xyz"])
            Rfix = _rpy(*j["rpy"])
            if j["index"] is not None:
                if j["index"] in self.idx:
                    ax = np.asarray(j["axis"], float)
                    axes.append((R @ Rfix) @ (ax / np.linalg.norm(ax)))
                    origins.append(x.copy())
                Rj = Rfix @ _axis_rot(j["axis"], q26[j["index"]])
            else:
                Rj = Rfix
            R = R @ Rj
        p = x + R @ self.tool
        J = np.zeros((3, len(self.idx)))
        for k in range(len(axes)):
            J[:, k] = np.cross(axes[k], p - origins[k])
        return p, J

    def manipulability_fast(self, q26):
        """Same metric as manipulability() but one FK pass (geometric J)."""
        _, J = self._fk_jac_fast(q26)
        s = np.linalg.svd(J, compute_uv=False)
        return float(s[-1] / s[0]) if s[0] > 1e-12 else 0.0

    def ik_step(self, q26, target, lam=0.05, step_m=0.04, dq_max=0.06,
                q_ref=None, k_null=0.15):
        """One DLS step toward ``target`` (world, meters).

        Returns (new_q26 copy, err_m_before_step). Call repeatedly (e.g. each
        bridge tick) and the wrist glides to the target.

        ``q_ref``: optional posture anchor (full 26-vector). The arm has 7
        joints for a 3-DoF position task — without a secondary objective the
        4 redundant DoF drift, and jogging back-and-forth slowly winds the
        arm into contorted poses. The anchor term pulls toward ``q_ref``
        **inside the null space only** (projected through I − J⁺J), so it
        reshapes the elbow without moving the TCP: same target in = same
        posture back.
        """
        q = np.array(q26, dtype=float)
        cur = self.fk(q)
        e = np.asarray(target, float) - cur
        err = float(np.linalg.norm(e))
        if err < 1e-4:
            return q, err
        if err > step_m:                          # glide, don't jump
            e = e * (step_m / err)
        J = self.jacobian(q)
        JJt = J @ J.T + (lam ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(JJt, e)
        if q_ref is not None:
            qa = np.array([q[i] for i in self.idx])
            ra = np.array([q_ref[i] for i in self.idx])
            N = np.eye(len(self.idx)) - J.T @ np.linalg.solve(JJt, J)
            dq = dq + N @ np.clip(k_null * (ra - qa), -0.02, 0.02)
        dq = np.clip(dq, -dq_max, dq_max)
        for k, i in enumerate(self.idx):
            q[i] = np.clip(q[i] + dq[k], self.lo[k], self.hi[k])
        return q, err


def reach_map(kin, base, n=3000, guard=None, seed=0):
    """Sample ``n`` configs of one arm (its joints uniform in [lo, hi], every
    other joint held at ``base``), forward-kinematic each to the TCP and score
    **manipulability** (reciprocal Jacobian condition number, 0 = singular,
    1 = isotropic). Self-colliding samples are dropped when a ``guard(q26)``
    is supplied. Returns ``[[x, y, z, manip], ...]`` — a dexterity point cloud
    over the arm's reachable workspace (the 'manipulability heat-volume')."""
    rng = np.random.default_rng(seed)
    base = np.asarray(base, dtype=float)
    lo = np.asarray(kin.lo, dtype=float)
    hi = np.asarray(kin.hi, dtype=float)
    pts = []
    for _ in range(n):
        q = base.copy()
        q[kin.idx] = rng.uniform(lo, hi)
        if guard is not None and guard(q):
            continue
        p, J = kin._fk_jac_fast(q)
        s = np.linalg.svd(J, compute_uv=False)
        manip = float(s[-1] / s[0]) if s[0] > 1e-12 else 0.0
        pts.append([float(p[0]), float(p[1]), float(p[2]), manip])
    return pts
