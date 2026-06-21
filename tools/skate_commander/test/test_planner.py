"""RRT-Connect planner — headless tests with a synthetic config-space obstacle.
No sim model needed: the collision test is a plain 2-D function."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_commander import planner          # noqa: E402

LO, HI = np.array([-1.0, -1.0]), np.array([1.0, 1.0])


def _wall(gap_top):
    """Vertical wall band in 2-D config space: blocked when -0.2<q0<0.2; with
    gap_top, the upper strip q1>=0.5 is open (a doorway)."""
    def collision_free(q):
        in_band = -0.2 < q[0] < 0.2
        if not in_band:
            return True
        return gap_top and q[1] >= 0.5
    return collision_free


def _assert_valid(path, cf, res=0.02):
    assert path is not None
    for q in path:
        assert cf(q), f"node in collision: {q}"
        assert np.all(q >= LO - 1e-9) and np.all(q <= HI + 1e-9), "out of limits"
    for a, b in zip(path[:-1], path[1:]):
        d = b - a
        n = max(1, int(np.ceil(float(np.max(np.abs(d))) / res)))
        for k in range(n + 1):
            assert cf(a + d * (k / n)), "edge passes through collision"


def test_trivial_when_clear():
    cf = lambda q: True
    path = planner.plan(np.array([-0.5, 0.0]), np.array([0.5, 0.0]),
                        cf, LO, HI)
    assert path is not None and len(path) == 2
    print("PASS plan trivial: clear straight line -> 2 nodes")


def test_routes_around_obstacle():
    cf = _wall(gap_top=True)
    start, goal = np.array([-0.8, -0.8]), np.array([0.8, -0.8])
    assert not planner._edge_clear(start, goal, cf, 0.05)   # straight line blocked
    path = planner.plan(start, goal, cf, LO, HI, step=0.1, res=0.02, seed=1)
    _assert_valid(path, cf)
    assert np.allclose(path[0], start) and np.allclose(path[-1], goal)
    assert max(q[1] for q in path) >= 0.45, "did not route up through the gap"
    print(f"PASS plan routes around: {len(path)} nodes, "
          f"peak q1={max(q[1] for q in path):.2f}")


def test_none_when_goal_blocked():
    cf = _wall(gap_top=False)
    # goal sits inside the full-height wall -> unplannable
    assert planner.plan(np.array([-0.8, 0.0]), np.array([0.0, 0.0]),
                        cf, LO, HI, time_budget=0.3) is None
    print("PASS plan returns None when the goal is blocked")


def test_none_when_no_path():
    cf = lambda q: not (-0.2 < q[0] < 0.2)          # full-height wall, no gap
    assert planner.plan(np.array([-0.8, 0.0]), np.array([0.8, 0.0]),
                        cf, LO, HI, time_budget=0.4) is None
    print("PASS plan returns None when no path exists")


if __name__ == "__main__":
    test_trivial_when_clear()
    test_routes_around_obstacle()
    test_none_when_goal_blocked()
    test_none_when_no_path()
    test_home_routes_around_real_guard()
    print("PLANNER OK")


def test_home_routes_around_real_guard():
    """With the REAL guard, home() from a standing pose (bent legs, arms
    hanging) routes the ARMS around the elbow-fold self-collision while leaving
    the leg / balance chain untouched — the planner is never asked to move the
    legs (that was the bug that made the full-DoF plan blow up)."""
    import os
    try:
        import mujoco  # noqa: F401
    except ImportError:
        print("SKIP: mujoco not installed"); return
    skt = Path(os.environ.get("SKT_DIR", "/tmp/skate_teleop/skt_v3"))
    cxml = skt / "skt_v3_collision.xml"
    if not cxml.exists():
        print("SKIP: no collision model"); return
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skate_ros2"))

    from skate_commander.bridge import RobotBridge
    from skate_commander.server import make_collision_guard
    from skate_commander.urdf import joint_limits, parse_urdf
    from skate_ros2 import names

    guard = make_collision_guard(cxml)
    cfree = lambda q: not guard(q)
    default = np.array(names.DEFAULT_POSE, dtype=float)

    br = RobotBridge(limits=joint_limits(parse_urdf(skt / "skt_v3.urdf")))
    br.guard = guard
    br.estop = False
    start = np.zeros(names.N_JOINTS)
    start[2] = start[6] = 0.08                        # a slight standing leg bend
    assert cfree(start), "standing start must be collision-free"
    br.targ = start.copy()
    legs0 = start[:8].copy()

    straight = start.copy(); straight[8:] = default[8:]      # direct arm-fold goal
    blocked = not planner._edge_clear(start, straight, cfree, 0.05)
    br.home()
    # home must keep the legs either way (planned route or direct glide)
    assert np.allclose(br.home_goal[:8], legs0), "home must not move the legs"
    if blocked:
        assert br.plan_nodes is not None, "blocked straight path -> home must route around"
        route = [br.targ] + br.plan_nodes
        for q in route:
            assert cfree(q), "planned node in collision"
        for a, b in zip(route[:-1], route[1:]):
            assert planner._edge_clear(a, b, cfree, 0.05), "planned edge in collision"
        final = br.plan_nodes[-1]
        assert np.allclose(final[:8], legs0), "route must leave the legs untouched"
        assert (abs(final[11] - default[11]) < 1e-2
                and abs(final[19] - default[19]) < 1e-2), "elbows must reach default"
        print(f"PASS home routes the arms around ({len(br.plan_nodes)} nodes); "
              "legs kept, elbows at the default fold")
    else:
        assert br.home_active, "clear straight path -> direct glide"
        print("PASS home direct (straight arm-fold was clear); legs kept")
    br.close()
