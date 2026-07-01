#!/usr/bin/env python3
"""Plan and execute a bimanual motion on the Skate with MoveItPy.

Run it inside the configured MoveIt environment (the sim endpoint on :2000, the
skate_driver, the moveit_bridge and move_group all up — e.g. via
`ros2 launch skate_moveit_config demo.launch.py`), then:

    python plan_bimanual.py

It moves each arm to its documented `home` group state, then plans a Cartesian
pose goal for the right wrist — demonstrating both a named-target plan and a
pose-goal plan through the same config. Every plan flows:

    MoveItPy -> move_group -> FollowJointTrajectory bridge -> skate_driver -> robot/sim

so the arms inherit the driver's deadman / e-stop / overtemp safety.

NOTE: authored against the MoveItPy API; not executed in this repo's CI/sandbox
(no ROS 2 here) — see ../README.md's honesty note.
"""
import time

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


def plan_and_execute(robot, component, label):
    """Plan the component's currently-set goal and execute it if planning
    succeeds. Returns True on success."""
    plan = component.plan()
    if not plan:
        print(f"[{label}] planning FAILED")
        return False
    n = len(plan.trajectory.joint_trajectory.points)
    print(f"[{label}] planned {n} waypoints — executing")
    robot.execute(plan.trajectory, controllers=[])
    time.sleep(0.5)
    return True


def main():
    robot = MoveItPy(node_name="skate_plan_bimanual")
    left = robot.get_planning_component("left_arm")
    right = robot.get_planning_component("right_arm")

    # 1) both arms to their documented home pose (a named group state)
    for comp, label in ((left, "left_arm"), (right, "right_arm")):
        comp.set_start_state_to_current_state()
        comp.set_goal_state(configuration_name="home")
        plan_and_execute(robot, comp, f"{label} -> home")

    # 2) a Cartesian pose goal for the right wrist (in the fixed base frame)
    goal = PoseStamped()
    goal.header.frame_id = "base_link2"
    goal.pose.position.x = 0.10
    goal.pose.position.y = -0.20
    goal.pose.position.z = 0.30
    goal.pose.orientation.w = 1.0
    right.set_start_state_to_current_state()
    right.set_goal_state(pose_stamped_msg=goal, pose_link="wrist_a2_Mirror__1")
    plan_and_execute(robot, right, "right_arm -> pose")

    print("done — trajectories streamed through the bridge to the robot/sim")


if __name__ == "__main__":
    main()
