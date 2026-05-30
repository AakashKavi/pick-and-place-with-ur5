#!/usr/bin/env python3
"""
pick_place_node.py

Subscribes to /detected_objects (geometry_msgs/PoseArray) published by
the perception node, then commands the UR5e to execute a pick-and-place
sequence using MoveIt! 2.

Pick sequence for each object:
  1. Move to PRE-GRASP pose  (15 cm directly above the object)
  2. Move to GRASP pose      (at the object centroid)
  3. Simulate gripper close  (sleep + log)
  4. RETREAT back to pre-grasp
  5. Move to HOME configuration
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, Pose, PoseStamped

# MoveIt! 2 Python bindings (moveit_py — available in ROS2 Humble)
from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState

import time
import copy


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PLANNING_GROUP   = "ur_manipulator"   # As defined in the UR MoveIt! config
END_EFFECTOR     = "tool0"            # UR5e TCP frame
PRE_GRASP_OFFSET = 0.15               # metres above the object centroid
HOME_CONFIG_NAME = "home"             # Named config in the SRDF


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_place_node")

        # ------------------------------------------------------------------
        # MoveIt! interface
        # ------------------------------------------------------------------
        self.moveit = MoveItPy(node_name="pick_place_moveit_py")
        self.arm: PlanningComponent = self.moveit.get_planning_component(PLANNING_GROUP)
        self.get_logger().info("MoveIt! interface ready")

        # ------------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------------
        self.object_sub = self.create_subscription(
            PoseArray,
            "/detected_objects",
            self.objects_callback,
            10
        )

        # ------------------------------------------------------------------
        # State guard — prevents re-entrant execution
        # ------------------------------------------------------------------
        self.is_executing = False

        self.get_logger().info(
            "Pick-and-place node ready. Waiting for objects on /detected_objects ..."
        )

    # -----------------------------------------------------------------------
    # Callback
    # -----------------------------------------------------------------------
    def objects_callback(self, msg: PoseArray) -> None:
        if self.is_executing:
            return

        if not msg.poses:
            self.get_logger().info("No objects detected yet.")
            return

        # Pick only the first detected object for now
        target_pose = msg.poses[0]
        self.get_logger().info(
            f"Target object at: x={target_pose.position.x:.3f}  "
            f"y={target_pose.position.y:.3f}  "
            f"z={target_pose.position.z:.3f}"
        )

        self.is_executing = True
        self._execute_pick_sequence(target_pose, msg.header.frame_id)
        self.is_executing = False

    # -----------------------------------------------------------------------
    # Full pick sequence
    # -----------------------------------------------------------------------
    def _execute_pick_sequence(self, grasp_pose: Pose, frame_id: str) -> None:

        # 1. Build pre-grasp pose (directly above the object)
        pre_grasp_pose = copy.deepcopy(grasp_pose)
        pre_grasp_pose.position.z += PRE_GRASP_OFFSET

        # 2. Move to pre-grasp
        self.get_logger().info("Step 1/5 — Moving to pre-grasp position...")
        if not self._move_to_pose(pre_grasp_pose, frame_id):
            self.get_logger().error("Pre-grasp motion planning failed. Aborting.")
            return

        # 3. Descend to grasp
        self.get_logger().info("Step 2/5 — Descending to grasp...")
        if not self._move_to_pose(grasp_pose, frame_id):
            self.get_logger().error("Grasp descent failed. Aborting.")
            return

        # 4. Close gripper (simulated — replace with real gripper action later)
        self.get_logger().info("Step 3/5 — Closing gripper (simulated)...")
        time.sleep(0.5)
        self.get_logger().info("Gripper closed. Object grasped!")

        # 5. Retreat
        self.get_logger().info("Step 4/5 — Retreating to pre-grasp...")
        if not self._move_to_pose(pre_grasp_pose, frame_id):
            self.get_logger().warn("Retreat motion failed — continuing to home anyway.")

        # 6. Return home
        self.get_logger().info("Step 5/5 — Returning to home configuration...")
        self._move_to_named_config(HOME_CONFIG_NAME)

        self.get_logger().info("Pick sequence complete!")

    # -----------------------------------------------------------------------
    # Motion helpers
    # -----------------------------------------------------------------------
    def _move_to_pose(self, pose: Pose, frame_id: str) -> bool:
        """Plan and execute a Cartesian end-effector pose goal."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(
            pose_stamped_msg=pose_stamped,
            pose_link=END_EFFECTOR
        )

        plan_result = self.arm.plan()
        if not plan_result:
            return False

        self.moveit.execute(plan_result.trajectory, controllers=[])
        return True

    def _move_to_named_config(self, config_name: str) -> bool:
        """Plan and execute a named joint configuration (e.g. 'home')."""
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=config_name)

        plan_result = self.arm.plan()
        if not plan_result:
            self.get_logger().error(f"Planning to '{config_name}' failed.")
            return False

        self.moveit.execute(plan_result.trajectory, controllers=[])
        return True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
