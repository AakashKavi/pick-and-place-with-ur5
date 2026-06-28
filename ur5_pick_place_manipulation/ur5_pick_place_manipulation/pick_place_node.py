#!/usr/bin/env python3
"""
pick_place_node.py

Subscribes to /detected_objects (geometry_msgs/PoseArray published in the
camera frame), transforms each pose into the robot base frame using TF2,
then executes a pick-and-place sequence via the MoveGroup action interface.

Key insight: MoveIt! plans in the robot's base frame. Detected object poses
arrive in the camera's optical frame. TF2 bridges this gap by looking up the
transform between the two frames at the time the detection was made.

Pick sequence:
  1. Transform detected pose from camera frame -> base_link
  2. Move to PRE-GRASP pose (offset above object)
  3. Move to GRASP pose
  4. Simulate gripper close
  5. RETREAT to pre-grasp
  6. Return to HOME joint configuration
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive

import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped with tf2

import copy
import time


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PLANNING_GROUP    = "ur_manipulator"
END_EFFECTOR      = "tool0"
ROBOT_BASE_FRAME  = "base_link_inertia"
PRE_GRASP_OFFSET  = 0.10     # metres above object in z (robot base frame)
PLANNING_TIME     = 10.0
MAX_VELOCITY      = 0.2
MAX_ACCEL         = 0.2


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_place_node")

        # ------------------------------------------------------------------
        # TF2 buffer and listener
        # ------------------------------------------------------------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # MoveGroup action client
        # ------------------------------------------------------------------
        self._action_client = ActionClient(self, MoveGroup, "/move_action")
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected.")

        # ------------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------------
        self.object_sub = self.create_subscription(
            PoseArray,
            "/detected_objects",
            self.objects_callback,
            10
        )

        self.is_executing = False
        self.get_logger().info(
            "Pick-and-place node ready. Waiting for objects on /detected_objects ..."
        )

    # -----------------------------------------------------------------------
    # Callback
    # -----------------------------------------------------------------------
    
    def objects_callback(self, msg: PoseArray) -> None:
        if self.is_executing or not msg.poses:
            return

        cam = msg.poses[0]

        # base_link_inertia is rotated 180deg around Z relative to camera mount
        # so lateral axes (the y component) flip sign
        robot_x =  cam.position.z + 0.600
        robot_y = cam.position.y
        robot_z = -cam.position.x + 1.500

        self.get_logger().info(
            f"Object in robot frame: x={robot_x:.3f} y={robot_y:.3f} z={robot_z:.3f}"
        )

        target_pose = Pose()
        target_pose.position.x = robot_x
        target_pose.position.y = robot_y
        target_pose.position.z = robot_z
        target_pose.orientation.x = 1.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0

        self.is_executing = True
        self._execute_pick_sequence(target_pose)
        self.is_executing = False
    # -----------------------------------------------------------------------
    # Full pick sequence
    # -----------------------------------------------------------------------
    def _execute_pick_sequence(self, grasp_pose: Pose) -> None:

        # Pre-grasp: 10cm above the object in z (robot base frame, z = up)
        pre_grasp_pose = copy.deepcopy(grasp_pose)
        pre_grasp_pose.position.z += PRE_GRASP_OFFSET

        # Set a top-down grasp orientation (pointing straight down)
        # This quaternion represents a 180 degree rotation around x axis
        # so the end effector points downward
        pre_grasp_pose.orientation.x = 1.0
        pre_grasp_pose.orientation.y = 0.0
        pre_grasp_pose.orientation.z = 0.0
        pre_grasp_pose.orientation.w = 0.0
        grasp_pose.orientation = pre_grasp_pose.orientation

        self.get_logger().info("Step 1/5 — Moving to pre-grasp position...")
        if not self._move_to_pose(pre_grasp_pose):
            self.get_logger().error("Pre-grasp failed. Aborting.")
            return

        self.get_logger().info("Step 2/5 — Descending to grasp...")
        if not self._move_to_pose(grasp_pose):
            self.get_logger().error("Grasp descent failed. Aborting.")
            return

        self.get_logger().info("Step 3/5 — Closing gripper (simulated)...")
        time.sleep(0.5)
        self.get_logger().info("Object grasped!")

        self.get_logger().info("Step 4/5 — Retreating...")
        self._move_to_pose(pre_grasp_pose)

        self.get_logger().info("Step 5/5 — Returning to home...")
        self._move_to_home()

        self.get_logger().info("Pick sequence complete!")

    # -----------------------------------------------------------------------
    # Move to Cartesian pose
    # -----------------------------------------------------------------------
    def _move_to_pose(self, pose: Pose) -> bool:
        goal    = MoveGroup.Goal()
        request = MotionPlanRequest()

        request.group_name                     = PLANNING_GROUP
        request.num_planning_attempts          = 10
        request.allowed_planning_time          = PLANNING_TIME
        request.max_velocity_scaling_factor    = MAX_VELOCITY
        request.max_acceleration_scaling_factor = MAX_ACCEL

        request.workspace_parameters                   = WorkspaceParameters()
        request.workspace_parameters.header.frame_id  = ROBOT_BASE_FRAME
        request.workspace_parameters.min_corner.x     = -2.0
        request.workspace_parameters.min_corner.y     = -2.0
        request.workspace_parameters.min_corner.z     = -0.5
        request.workspace_parameters.max_corner.x     = 2.0
        request.workspace_parameters.max_corner.y     = 2.0
        request.workspace_parameters.max_corner.z     = 2.0

        # Position constraint — 2cm tolerance sphere around target
        pos_c                        = PositionConstraint()
       # pos_c.header.frame_id        = base_link_inertia
        pos_c.header.frame_id        = ROBOT_BASE_FRAME
        pos_c.link_name              = END_EFFECTOR
        pos_c.weight                 = 1.0

        volume                       = BoundingVolume()
        sphere                       = SolidPrimitive()
        sphere.type                  = SolidPrimitive.SPHERE
        sphere.dimensions            = [0.02]

        sphere_pose                  = Pose()
        sphere_pose.position         = pose.position
        sphere_pose.orientation.w   = 1.0

        volume.primitives.append(sphere)
        volume.primitive_poses.append(sphere_pose)
        pos_c.constraint_region      = volume

        # Orientation constraint — loose tolerance for flexibility
        ori_c                              = OrientationConstraint()
       # ori_c.header.frame_id             = ROBOT_BASE_FRAME
        ori_c.header.frame_id              = "base_link_inertia"
        ori_c.link_name                   = END_EFFECTOR
        ori_c.orientation                 = pose.orientation
        ori_c.absolute_x_axis_tolerance   = 3.14
        ori_c.absolute_y_axis_tolerance   = 3.14
        ori_c.absolute_z_axis_tolerance   = 3.14
        ori_c.weight                      = 0.1

        constraints = Constraints()
        constraints.position_constraints.append(pos_c)
        constraints.orientation_constraints.append(ori_c)
        request.goal_constraints.append(constraints)

        goal.request                          = request
        goal.planning_options.plan_only       = False
        goal.planning_options.replan          = True
        goal.planning_options.replan_attempts = 5

        return self._send_goal(goal)

    # -----------------------------------------------------------------------
    # Move to home using joint value target
    # -----------------------------------------------------------------------
    def _move_to_home(self) -> bool:
        from moveit_msgs.msg import JointConstraint

        goal    = MoveGroup.Goal()
        request = MotionPlanRequest()

        request.group_name                      = PLANNING_GROUP
        request.num_planning_attempts           = 10
        request.allowed_planning_time           = PLANNING_TIME
        request.max_velocity_scaling_factor     = MAX_VELOCITY
        request.max_acceleration_scaling_factor = MAX_ACCEL

        # UR5e home position joint values (all zeros = upright)
        joint_names  = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        home_values = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]

        constraints = Constraints()
        for name, value in zip(joint_names, home_values):
            jc                    = JointConstraint()
            jc.joint_name         = name
            jc.position           = value
            jc.tolerance_above    = 0.05
            jc.tolerance_below    = 0.05
            jc.weight             = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)

        goal.request                          = request
        goal.planning_options.plan_only       = False
        goal.planning_options.replan          = True
        goal.planning_options.replan_attempts = 5

        return self._send_goal(goal)

    # -----------------------------------------------------------------------
    # Send goal and wait for result
    # -----------------------------------------------------------------------
    def _send_goal(self, goal: MoveGroup.Goal) -> bool:
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        error_code = result_future.result().result.error_code.val
        if error_code == 1:
            self.get_logger().info("Motion executed successfully")
            return True
        else:
            self.get_logger().error(f"Motion failed — MoveIt error code: {error_code}")
            return False


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