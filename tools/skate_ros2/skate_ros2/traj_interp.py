"""Pure-Python trajectory interpolation for the MoveIt bridge.

No ROS, no numpy — just the math that turns a sparse MoveIt JointTrajectory
into a dense stream of joint setpoints. Kept separate from the rclpy node so it
is unit-tested directly (the node is a thin action-server shell around it).
"""
from __future__ import annotations

import bisect


def sample_trajectory(positions_by_point, times, t):
    """Linearly interpolate a joint trajectory at time ``t`` (seconds).

    ``positions_by_point`` — one row of joint positions per trajectory point
    (all rows in the same joint order); ``times`` — the monotonically
    increasing ``time_from_start`` of each point. The endpoints are held: any
    ``t`` before the first point returns the first, after the last returns the
    last (so a controller that keeps sampling just holds the goal).
    """
    if not times:
        return None
    if t <= times[0]:
        return list(positions_by_point[0])
    if t >= times[-1]:
        return list(positions_by_point[-1])
    i = bisect.bisect_right(times, t) - 1
    t0, t1 = times[i], times[i + 1]
    a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
    p0, p1 = positions_by_point[i], positions_by_point[i + 1]
    return [p0[k] + a * (p1[k] - p0[k]) for k in range(len(p0))]


def validate_trajectory(joint_names, positions_by_point):
    """Return ``(ok, reason)``. A FollowJointTrajectory goal is executable only
    if it names joints and every point carries one position per joint — a bad
    goal should be rejected, not streamed to the robot."""
    if not joint_names:
        return False, "empty joint_names"
    if not positions_by_point:
        return False, "empty trajectory"
    n = len(joint_names)
    for i, p in enumerate(positions_by_point):
        if len(p) != n:
            return False, f"point {i} has {len(p)} positions, expected {n}"
    return True, "ok"


def plan_setpoints(names, positions_by_point, times, rate_hz):
    """Densify a trajectory into ``[(t, {joint: position}), ...]`` at
    ``rate_hz`` — exactly the setpoint stream the bridge publishes to
    ``skate/joint_position_cmd``. Pure function for testing/offline use."""
    if not times:
        return []
    dt = 1.0 / rate_hz
    n = int(times[-1] / dt)
    out = []
    for s in range(n + 1):
        t = min(s * dt, times[-1])
        pos = sample_trajectory(positions_by_point, times, t)
        out.append((t, dict(zip(names, pos))))
    if out and out[-1][0] < times[-1]:
        out.append((times[-1], dict(zip(names, positions_by_point[-1]))))
    return out
