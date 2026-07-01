"""Trajectory-interpolation tests for the MoveIt bridge (pure Python, no ROS).

The rclpy action-server node is a thin shell; the logic that matters — turning
a sparse MoveIt JointTrajectory into a dense, monotone setpoint stream — lives
in skate_ros2.traj_interp and is fully testable without a ROS 2 install.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_ros2.traj_interp import (  # noqa: E402
    sample_trajectory, plan_setpoints, validate_trajectory)


def test_sample_holds_endpoints_and_interpolates():
    pts = [[0.0, 1.0], [1.0, 3.0]]      # 2 joints, 2 points
    times = [0.0, 2.0]
    assert sample_trajectory(pts, times, -1.0) == [0.0, 1.0]   # before start -> first
    assert sample_trajectory(pts, times, 5.0) == [1.0, 3.0]    # after end -> last
    mid = sample_trajectory(pts, times, 1.0)                   # halfway
    assert abs(mid[0] - 0.5) < 1e-9 and abs(mid[1] - 2.0) < 1e-9
    q = sample_trajectory(pts, times, 0.5)                     # quarter way
    assert abs(q[0] - 0.25) < 1e-9 and abs(q[1] - 1.5) < 1e-9
    print("PASS sample_trajectory: endpoints held + linear interp")


def test_sample_single_point_and_empty():
    assert sample_trajectory([[0.2, 0.3]], [0.0], 9.9) == [0.2, 0.3]
    assert sample_trajectory([], [], 1.0) is None
    print("PASS sample_trajectory: single-point hold + empty")


def test_plan_setpoints_rate_and_coverage():
    names = ["a3_armL_a11", "a0_armL_a8"]
    pts = [[0.0, 0.0], [1.0, 2.0]]
    times = [0.0, 1.0]
    sp = plan_setpoints(names, pts, times, rate_hz=10.0)
    assert sp[0][0] == 0.0 and sp[0][1] == {"a3_armL_a11": 0.0, "a0_armL_a8": 0.0}
    assert abs(sp[-1][0] - 1.0) < 1e-9
    assert abs(sp[-1][1]["a3_armL_a11"] - 1.0) < 1e-9
    assert abs(sp[-1][1]["a0_armL_a8"] - 2.0) < 1e-9
    assert 10 <= len(sp) <= 12                                 # ~11 at 10 Hz over 1 s
    ts = [t for t, _ in sp]
    assert ts == sorted(ts) and all(0.0 <= t <= 1.0 for t in ts)
    print(f"PASS plan_setpoints: {len(sp)} monotone setpoints at 10 Hz, exact endpoints")


def test_validate_trajectory():
    ok, _ = validate_trajectory(["j1", "j2"], [[0.0, 1.0], [0.5, 0.5]])
    assert ok
    bad, reason = validate_trajectory([], [[0.0]])
    assert not bad and "joint_names" in reason
    bad, reason = validate_trajectory(["j1", "j2"], [[0.0, 1.0], [0.0]])
    assert not bad and "positions" in reason
    bad, reason = validate_trajectory(["j1"], [])
    assert not bad
    print("PASS validate_trajectory: names / per-point length / empty checks")


if __name__ == "__main__":
    test_sample_holds_endpoints_and_interpolates()
    test_sample_single_point_and_empty()
    test_plan_setpoints_rate_and_coverage()
    test_validate_trajectory()
