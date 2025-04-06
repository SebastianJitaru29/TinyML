#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit_py.robot_trajectory import RobotTrajectory
from moveit_py.planning import MoveItPy, PlanningComponent

def main():
    rclpy.init()
    node = Node("braccio_ik_move_node")

    moveit = MoveItPy(node=node)
    planning_component = PlanningComponent("braccio_arm", moveit)

    # Define a target pose for the end effector
    target_pose = PoseStamped()
    target_pose.header.frame_id = "braccio_base_link"  # change if different
    target_pose.pose.position.x = 0.25
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.2
    target_pose.pose.orientation.w = 1.0

    # Set target pose
    planning_component.set_goal(goal=target_pose, pose=True)

    # Plan
    plan_result = planning_component.plan()

    if plan_result.successful:
        node.get_logger().info("Plan succeeded! Executing...")
        planning_component.execute()
    else:
        node.get_logger().error("Planning failed!")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
