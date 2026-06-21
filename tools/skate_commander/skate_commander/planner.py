"""Collision-free joint-space path planning for the cockpit.

The collision guard REJECTS any move whose straight-line joint path clips a
self-collision, so home(), waypoint moves and cartesian glides stall at the
obstacle (e.g. the elbow can't fold from the hanging pose to the 90 deg home
pose — the hand sweeps through the thigh, so the move just gives up). This
planner finds a path AROUND it: a bidirectional RRT (RRT-Connect) over a
chosen set of joints, checked against the same guard, then shortened by
random shortcutting.

Pure logic — no ROS / MuJoCo here. The caller supplies ``collision_free(q)``
(typically ``lambda q: not bridge.guard(q)``), so the planner is fully
testable headless with a synthetic obstacle.
"""

from __future__ import annotations

import time

import numpy as np


def _edge_clear(a, b, collision_free, res):
    """True if every interpolated config from a to b is collision-free (b
    included, a assumed already valid), at spacing <= ``res`` rad."""
    d = b - a
    n = max(1, int(np.ceil(float(np.max(np.abs(d))) / res)))
    for k in range(1, n + 1):
        if not collision_free(a + d * (k / n)):
            return False
    return True


def _steer(q_from, q_to, step):
    d = q_to - q_from
    dist = float(np.max(np.abs(d)))
    if dist <= step:
        return q_to.copy()
    return q_from + d * (step / dist)


class _Tree:
    __slots__ = ("nodes", "parent")

    def __init__(self, root):
        self.nodes = [root]
        self.parent = [-1]

    def nearest(self, q):
        best, bi = None, 0
        for i, nd in enumerate(self.nodes):
            dist = float(np.sum((nd - q) ** 2))
            if best is None or dist < best:
                best, bi = dist, i
        return bi

    def add(self, q, parent):
        self.nodes.append(q)
        self.parent.append(parent)
        return len(self.nodes) - 1

    def chain(self, i):
        """Nodes from i back to the root (leaf-first)."""
        out = []
        while i != -1:
            out.append(self.nodes[i])
            i = self.parent[i]
        return out


def _shortcut(path, collision_free, res, rng, rounds=150):
    """Replace random sub-segments path[i..j] with a straight cut when it's
    clear — trims the RRT zig-zag toward a near-shortest route."""
    path = [np.asarray(p, dtype=float) for p in path]
    for _ in range(rounds):
        if len(path) <= 2:
            break
        i = int(rng.integers(0, len(path) - 2))
        j = int(rng.integers(i + 2, len(path)))
        if _edge_clear(path[i], path[j], collision_free, res):
            path = path[:i + 1] + path[j:]
    return path


def plan(q_start, q_goal, collision_free, lo, hi, joints=None,
         step=0.12, res=0.05, max_iter=6000, time_budget=0.8,
         goal_bias=0.2, seed=0):
    """Return a collision-free path ``[q_start, ..., q_goal]`` (list of np[N])
    or ``None``.

    ``collision_free(q)`` -> True if q is allowed. ``joints`` is the index
    array the planner may move; every other joint is pinned at ``q_start``
    (joints where start and goal differ are added automatically). RRT-Connect
    with ``step`` rad per extension and ``res`` rad edge-resolution, bounded by
    ``max_iter`` and ``time_budget`` seconds so it can never hang the server.
    """
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    lo = np.asarray(lo, dtype=float)
    hi = np.asarray(hi, dtype=float)
    n = q_start.shape[0]
    joints = np.arange(n) if joints is None else np.asarray(joints, dtype=int)
    diff = np.where(np.abs(q_goal - q_start) > 1e-9)[0]
    joints = np.union1d(joints, diff).astype(int)     # must move all differing

    if not collision_free(q_start) or not collision_free(q_goal):
        return None
    if _edge_clear(q_start, q_goal, collision_free, res):
        return [q_start.copy(), q_goal.copy()]        # straight line is clear

    rng = np.random.default_rng(seed)
    pin = q_start.copy()
    deadline = time.monotonic() + time_budget

    def sample():
        q = pin.copy()
        q[joints] = rng.uniform(lo[joints], hi[joints])
        return q

    def extend(tree, target):
        ni = tree.nearest(target)
        q_near = tree.nodes[ni]
        q_new = _steer(q_near, target, step)
        if not _edge_clear(q_near, q_new, collision_free, res):
            return "trapped", -1
        idx = tree.add(q_new, ni)
        return ("reached" if np.array_equal(q_new, target)
                else "advanced"), idx

    def connect(tree, target):
        status, idx = "advanced", -1
        while status == "advanced":
            status, idx = extend(tree, target)
        return status, idx

    t_start, t_goal = _Tree(q_start.copy()), _Tree(q_goal.copy())
    for it in range(max_iter):
        if time.monotonic() > deadline:
            return None
        grow, other, grow_is_start = ((t_start, t_goal, True) if it % 2 == 0
                                      else (t_goal, t_start, False))
        # goal-biased: now and then aim the growing tree straight at the other
        # tree's root so the two reach toward each other (big speedup vs pure
        # random exploration).
        target = (other.nodes[0] if rng.random() < goal_bias else sample())
        status, idx = extend(grow, target)
        if status == "trapped":
            continue
        cstatus, cidx = connect(other, grow.nodes[idx])
        if cstatus == "reached":
            grow_chain = grow.chain(idx)              # q_link -> grow.root
            other_chain = other.chain(cidx)           # q_link -> other.root
            grow_chain.reverse()                      # grow.root -> q_link
            full = grow_chain + other_chain[1:]       # grow.root -> other.root
            if not grow_is_start:
                full.reverse()                        # orient start -> goal
            return _shortcut(full, collision_free, res, rng)
    return None
