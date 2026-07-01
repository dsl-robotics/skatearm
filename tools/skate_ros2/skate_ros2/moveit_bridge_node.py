"""skate_moveit_bridge — MoveIt 2 execution bridge for the Skate.

MoveIt's simple controller manager sends each arm a FollowJointTrajectory goal;
this node runs one action server per arm, interpolates the planned trajectory
(:mod:`skate_ros2.traj_interp`) and streams the setpoints to
``skate/joint_position_cmd``. The existing :mod:`skate_ros2.driver_node` then
does the UDP wire + deadman/estop safety — MoveIt never touches the robot
directly, and inherits the audited safety model instead of duplicating it.

Thin by design: the interpolation is pure Python (tested without ROS); only the
action plumbing needs rclpy. On hardware, a ros2_control JointTrajectory-
Controller + a Skate ``SystemInterface`` is the production alternative; this
bridge keeps the whole loop in Python and reuses the driver.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .traj_interp import sample_trajectory, validate_trajectory

LEFT_ACTION = "skate_left_arm_controller/follow_joint_trajectory"
RIGHT_ACTION = "skate_right_arm_controller/follow_joint_trajectory"


class SkateMoveItBridge(Node):
    def __init__(self):
        super().__init__("skate_moveit_bridge")
        self.declare_parameter("rate", 60.0)
        self.rate = float(self.get_parameter("rate").value)
        self.pub = self.create_publisher(
            JointState, "skate/joint_position_cmd", 10)
        self._left = ActionServer(
            self, FollowJointTrajectory, LEFT_ACTION, self.execute)
        self._right = ActionServer(
            self, FollowJointTrajectory, RIGHT_ACTION, self.execute)
        self.get_logger().info(
            "skate_moveit_bridge: FollowJointTrajectory (left/right arm) "
            "-> skate/joint_position_cmd")

    def _publish(self, names, positions):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(names)
        js.position = [float(x) for x in positions]
        self.pub.publish(js)

    def _feedback(self, goal_handle, names, positions, t):
        fb = FollowJointTrajectory.Feedback()
        fb.header.stamp = self.get_clock().now().to_msg()
        fb.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        fb.desired = pt
        fb.actual = pt          # the bridge streams open-loop; no separate feedback loop
        goal_handle.publish_feedback(fb)

    def execute(self, goal_handle):
        """Interpolate the goal trajectory and stream it to the driver."""
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        pts = [list(p.positions) for p in traj.points]
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                 for p in traj.points]
        result = FollowJointTrajectory.Result()
        if not pts:                              # empty trajectory = nothing to do
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        ok, reason = validate_trajectory(names, pts)
        if not ok:                               # malformed goal — refuse, don't drive
            self.get_logger().warning(f"rejecting FollowJointTrajectory goal: {reason}")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = reason
            return result

        t0 = self.get_clock().now().nanoseconds * 1e-9
        dt = 1.0 / self.rate
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            t = self.get_clock().now().nanoseconds * 1e-9 - t0
            pos = sample_trajectory(pts, times, t)
            self._publish(names, pos)
            self._feedback(goal_handle, names, pos, t)
            if t >= times[-1]:
                break
            time.sleep(dt)
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SkateMoveItBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
