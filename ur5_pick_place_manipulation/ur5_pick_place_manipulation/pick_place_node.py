#!/usr/bin/env python3
"""
pick_place_node.py

Subscribes to /detected_objects (geometry_msgs/PoseArray published in the
camera frame), identifies which detected cluster is the cylinder by matching
against Gazebo ground truth, then picks it up and places it on top of the
cube via the MoveGroup action interface.

Pick-and-place sequence:
  1. Move to PRE-GRASP pose (above the cylinder)
  2. Descend to GRASP pose
  3. Close gripper
  4. RETREAT to pre-grasp
  5. Move above the cube (PRE-PLACE)
  6. Descend to PLACE pose (resting on top of the cube)
  7. Open gripper
  8. Retreat and return to HOME joint configuration
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseArray, Pose, Quaternion
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    JointConstraint,
    CollisionObject,
    PlanningScene,
    RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint

import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped with tf2

from scipy.interpolate import CubicSpline

import copy
import subprocess
import threading
import time


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PLANNING_GROUP    = "ur_manipulator"
END_EFFECTOR      = "gripper_tcp"   # fingertip midpoint, not the flange
# base_link is coincident with world (identity fixed joint) in ur.urdf.xacro.
# base_link_inertia is NOT — it's base_link rotated 180deg about Z, purely an
# internal frame the UR driver uses. Targets computed below are in world/
# base_link convention, so constraints must be stamped with base_link, not
# base_link_inertia, or the arm reaches for the mirror-image target.
ROBOT_BASE_FRAME  = "base_link"

# Depth camera mount pose in world/base_link (must match the depth_camera
# model's <pose> in pick_place.sdf and the camera_tf static transform in
# sim.launch.py).
CAMERA_X = 0.65
CAMERA_Z = 1.30
WORLD_FRAME       = "world"
PRE_GRASP_OFFSET  = 0.10     # metres above object in z (robot base frame)
PLANNING_TIME     = 10.0
MAX_VELOCITY      = 0.2
MAX_ACCEL         = 0.2

# Short local moves (descend to grasp/place, retreat) go through
# compute_cartesian_path instead of a joint-space MoveGroup goal. The
# joint-space planner (_move_to_pose) only loosely constrains orientation
# (see _move_to_pose's tolerances), so RRTConnect is free to pick *any*
# IK solution satisfying that goal — including ones needing a completely
# different elbow/wrist configuration, which shows up as the arm swinging
# through a weird pose instead of just moving straight down. Cartesian path
# planning interpolates pose-to-pose and solves IK incrementally from the
# previous waypoint, so it stays local by construction.
CARTESIAN_MAX_STEP    = 0.01   # metres between interpolated waypoints
CARTESIAN_MIN_FRACTION = 0.95  # below this, fall back to joint-space planning
CARTESIAN_TIME_SCALE  = 4.0    # stretch factor so execution stays within
                                # joint_trajectory_controller's tolerances
                                # (compute_cartesian_path times at full speed)

# Longer transit moves (pre-grasp approach, pre-grasp -> pre-place, home
# return) use a cubic-spline joint trajectory instead of OMPL/RRTConnect.
# RRTConnect's raw path is piecewise-linear in joint space — smooth timing
# gets applied on top (time-optimal parameterization), but the path's
# direction still changes abruptly at each waypoint. A per-joint cubic
# spline clamped to zero velocity at both ends gives a single smooth
# accelerate/decelerate motion instead. Since this bypasses OMPL, and
# OMPL is what actually checks collisions, each sampled waypoint is
# validated against the planning scene (table included) via
# /check_state_validity before executing; any invalid waypoint falls back
# to the collision-aware joint-space planner instead.
ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
HOME_JOINT_VALUES = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
SPLINE_DURATION = 4.0   # seconds
SPLINE_STEPS    = 40    # trajectory points sampled along the spline

# Gripper (see ur5_pick_place_description/urdf/gripper.xacro for the joint
# limits these correspond to)
GRIPPER_OPEN      = 0.0
GRIPPER_CLOSED    = -0.018
GRIPPER_MOVE_TIME = 0.5   # seconds given to the gz JointPositionController to get there

# Kinematic-attach bookkeeping.
GZ_WORLD_NAME       = "pick_place_world"
GRASP_MATCH_RADIUS  = 0.15   # metres; ignore matches farther than this

# Retry a failed grasp this many times before giving up. A grasp counts as
# successful if, after closing, the target model's actual Gazebo pose is
# within GRASP_VERIFY_RADIUS of gripper_tcp's actual pose — i.e. the object
# really ended up between the fingers, not just that the arm reached the
# planned target (which can be wrong — see _execute_pick_and_place).
MAX_GRASP_ATTEMPTS  = 3
GRASP_VERIFY_RADIUS = 0.05   # metres

# Task: pick the cylinder up and place it on top of the cube. Which detected
# cluster is the cylinder is resolved the same way as kinematic attach —
# nearest-position match against Gazebo ground truth — since perception
# doesn't classify shape.
GRASP_TARGET_MODEL   = "blue_cylinder"
PLACE_TARGET_MODEL   = "red_box"

# Object half-extents (see pick_place.sdf) used to stack the cylinder
# centered on top of the cube: place_z = cube_z + half_cube + half_cylinder.
CUBE_HALF_HEIGHT      = 0.025   # red_box: 0.05 cube
CYLINDER_HALF_LENGTH  = 0.030   # blue_cylinder: length 0.06
PLACE_CLEARANCE       = 0.002   # small gap so it doesn't spawn interpenetrating

# Table collision object (must match the table's <pose>/<size> in
# pick_place.sdf) so MoveIt actually plans around it instead of just being
# statically less likely to hit it.
TABLE_SIZE = (0.5, 0.5, 0.05)
TABLE_POSE = (0.65, 0.0, 0.475)


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
        # Cartesian path planning + execution, for short local moves
        # (descend/retreat) — see CARTESIAN_* constants for why.
        # ------------------------------------------------------------------
        self._cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.get_logger().info("Waiting for /compute_cartesian_path service...")
        self._cartesian_client.wait_for_service()

        self._execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.get_logger().info("Waiting for ExecuteTrajectory action server...")
        self._execute_client.wait_for_server()
        self.get_logger().info("Cartesian path planning connected.")

        # ------------------------------------------------------------------
        # Spline joint trajectories, for longer transit moves — see the
        # ARM_JOINT_NAMES / SPLINE_* comment for why. Current joint state
        # for the spline's start, and per-waypoint validity checking
        # (since bypassing OMPL also bypasses its collision checking).
        # Goal joint values are resolved via a plan-only OMPL call, not a
        # dedicated IK service — see _resolve_goal_joints for why.
        # ------------------------------------------------------------------
        self._latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        self._validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.get_logger().info("Waiting for /check_state_validity service...")
        self._validity_client.wait_for_service()
        self.get_logger().info("Spline trajectory planning connected.")

        # ------------------------------------------------------------------
        # Gripper command publisher (drives the gz-sim JointPositionController
        # plugin on gripper_finger_joint directly, see gripper.xacro)
        # ------------------------------------------------------------------
        self._gripper_pub = self.create_publisher(Float64, "/gripper_position_cmd", 10)
        self._send_gripper_command(GRIPPER_OPEN)

        # Name of the Gazebo model currently held via kinematic attach, if any
        self._attached_model = None

        # ------------------------------------------------------------------
        # Planning scene: register the table so MoveIt actually plans around
        # it instead of being unaware it exists.
        # ------------------------------------------------------------------
        self._add_table_collision_object()

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
        self._done = False   # stop after one successful pick-and-place
        self.get_logger().info(
            "Pick-and-place node ready. Waiting for objects on /detected_objects ..."
        )

    # -----------------------------------------------------------------------
    # Planning scene setup
    # -----------------------------------------------------------------------
    def _add_table_collision_object(self) -> None:
        client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_logger().info("Waiting for /apply_planning_scene service...")
        client.wait_for_service()

        table = CollisionObject()
        table.header.frame_id = ROBOT_BASE_FRAME
        table.id = "table"
        table.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(TABLE_SIZE)
        table.primitives.append(box)

        box_pose = Pose()
        box_pose.position.x, box_pose.position.y, box_pose.position.z = TABLE_POSE
        box_pose.orientation.w = 1.0
        table.primitive_poses.append(box_pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(table)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Table added to the planning scene as a collision object.")
        else:
            self.get_logger().warn("Failed to add table to the planning scene.")

    # -----------------------------------------------------------------------
    # Callback
    # -----------------------------------------------------------------------
    
    def objects_callback(self, msg: PoseArray) -> None:
        if self.is_executing or self._done or not msg.poses:
            return

        cylinder_gt = self._get_model_pose(GRASP_TARGET_MODEL)
        if cylinder_gt is None:
            return

        # Perception doesn't classify shape, so identify which detected
        # cluster is the cylinder by nearest-position match against ground
        # truth (same approach used for kinematic attach). This match only
        # confirms perception can currently see the cylinder — the actual
        # grasp target below is built from ground truth, not the detected
        # pose. A single downward depth camera only ever sees an object's
        # top surface, never its center, so the detected Z is biased high
        # by roughly the object's own half-height: enough to make the
        # gripper close above the cylinder's body instead of around it.
        best_dist = GRASP_MATCH_RADIUS
        seen = False
        for cam in msg.poses:
            x, y, z = self._camera_pose_to_robot(cam)
            dist = ((x - cylinder_gt[0]) ** 2
                    + (y - cylinder_gt[1]) ** 2
                    + (z - cylinder_gt[2]) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                seen = True

        if not seen:
            return

        self.get_logger().info(
            f"Cylinder detected at x={cylinder_gt[0]:.3f} y={cylinder_gt[1]:.3f} z={cylinder_gt[2]:.3f}"
        )

        target_pose = Pose()
        target_pose.position.x = cylinder_gt[0]
        target_pose.position.y = cylinder_gt[1]
        target_pose.position.z = cylinder_gt[2]
        target_pose.orientation = self._down_orientation()

        # Run the actual sequence on a worker thread, not this callback.
        # _move_to_pose blocks on action-result futures; if it ran here it
        # would nest inside the very rclpy.spin(node) loop that has to
        # dispatch those results, which is exactly the re-entrancy hazard
        # _wait_for_future's docstring above describes. On a separate
        # thread it can safely poll while spin(node) keeps running.
        self.is_executing = True
        threading.Thread(
            target=self._run_pick_and_place, args=(target_pose,), daemon=True
        ).start()

    def _run_pick_and_place(self, target_pose: Pose) -> None:
        try:
            self._execute_pick_and_place(target_pose)
        finally:
            self.is_executing = False

    @staticmethod
    def _camera_pose_to_robot(cam: Pose):
        # Camera is mounted at world/base_link (CAMERA_X, 0, CAMERA_Z), pitched
        # 90deg down, in the camera's optical frame convention (Z forward,
        # X right, Y down). That maps: optical Z -> base +X, optical X ->
        # base -Z, optical Y -> base +Y (unchanged, no flip).
        return (
            cam.position.z + CAMERA_X,
            cam.position.y,
            -cam.position.x + CAMERA_Z,
        )

    @staticmethod
    def _down_orientation() -> Quaternion:
        # 180 degree rotation around X so the end effector points straight down.
        return Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    # -----------------------------------------------------------------------
    # Full pick-and-place sequence: grasp the cylinder, stack it on the cube
    # -----------------------------------------------------------------------
    def _execute_pick_and_place(self, grasp_pose: Pose) -> None:

        place_xyz = self._get_model_pose(PLACE_TARGET_MODEL)
        if place_xyz is None:
            self.get_logger().error(f"Could not locate '{PLACE_TARGET_MODEL}' to place onto. Aborting.")
            return

        place_pose = Pose()
        place_pose.position.x = place_xyz[0]
        place_pose.position.y = place_xyz[1]
        place_pose.position.z = place_xyz[2] + CUBE_HALF_HEIGHT + CYLINDER_HALF_LENGTH + PLACE_CLEARANCE
        place_pose.orientation = self._down_orientation()

        grasp_pose.orientation = self._down_orientation()
        pre_grasp_pose = copy.deepcopy(grasp_pose)
        pre_grasp_pose.position.z += PRE_GRASP_OFFSET

        pre_place_pose = copy.deepcopy(place_pose)
        pre_place_pose.position.z += PRE_GRASP_OFFSET

        grasped = False
        for attempt in range(1, MAX_GRASP_ATTEMPTS + 1):
            self.get_logger().info(
                f"Step 1/8 — Moving to pre-grasp position (attempt {attempt}/{MAX_GRASP_ATTEMPTS})..."
            )
            self._send_gripper_command(GRIPPER_OPEN)
            if not self._move_spline(pre_grasp_pose):
                self.get_logger().error("Pre-grasp failed. Aborting.")
                return

            self.get_logger().info("Step 2/8 — Descending to grasp...")
            if not self._move_linear(grasp_pose):
                self.get_logger().error("Grasp descent failed. Aborting.")
                return

            self.get_logger().info("Step 3/8 — Closing gripper...")
            self._send_gripper_command(GRIPPER_CLOSED)

            if self._verify_grasp(GRASP_TARGET_MODEL):
                self._attached_model = GRASP_TARGET_MODEL
                self.get_logger().info(f"Object grasped: {self._attached_model}")
                grasped = True
                break

            self.get_logger().warn(
                f"Grasp attempt {attempt}/{MAX_GRASP_ATTEMPTS} failed — "
                f"'{GRASP_TARGET_MODEL}' isn't actually in the gripper. Retrying..."
            )
            self._send_gripper_command(GRIPPER_OPEN)
            self._move_linear(pre_grasp_pose)

        if not grasped:
            self.get_logger().error(
                f"Failed to grasp '{GRASP_TARGET_MODEL}' after {MAX_GRASP_ATTEMPTS} attempts. Aborting."
            )
            return

        self.get_logger().info("Step 4/8 — Retreating...")
        self._move_linear(pre_grasp_pose)
        self._sync_attached_model_pose()

        self.get_logger().info("Step 5/8 — Moving above the cube...")
        if not self._move_spline(pre_place_pose):
            self.get_logger().error("Pre-place move failed. Aborting with object still in gripper.")
            return
        self._sync_attached_model_pose()

        self.get_logger().info("Step 6/8 — Lowering onto the cube...")
        if not self._move_linear(place_pose):
            self.get_logger().error("Place descent failed. Aborting with object still in gripper.")
            return
        self._sync_attached_model_pose()

        self.get_logger().info("Step 7/8 — Releasing onto the cube...")
        self._send_gripper_command(GRIPPER_OPEN)
        self._attached_model = None

        self.get_logger().info("Step 8/8 — Retreating and returning home...")
        self._move_linear(pre_place_pose)
        self._move_to_home()

        self._done = True
        self.get_logger().info("Pick-and-place complete: cylinder placed on the cube!")

    # -----------------------------------------------------------------------
    # Gripper control
    # -----------------------------------------------------------------------
    def _send_gripper_command(self, position: float) -> None:
        self._gripper_pub.publish(Float64(data=position))
        # gz's JointPositionController drives the joint asynchronously with
        # no completion feedback over this topic; give it time to get there.
        time.sleep(GRIPPER_MOVE_TIME)

    # -----------------------------------------------------------------------
    # Kinematic attach — the mock gripper does not grip objects through
    # contact physics, so while an object is "held" we teleport it in Gazebo
    # to track the gripper_tcp frame instead.
    # -----------------------------------------------------------------------
    def _get_model_pose(self, name: str):
        # NOTE: this must be the "ign" CLI, not "gz" — in this Fortress-era
        # install "gz model" can't find the running server at all ("An
        # instance of Gazebo is not running"), while "ign model" is the one
        # that actually talks to it.
        try:
            out = subprocess.run(
                ["ign", "model", "-m", name, "-p"],
                capture_output=True, text=True, timeout=3.0,
            )
        except (subprocess.SubprocessError, OSError):
            return None
        lines = out.stdout.splitlines()
        for i, line in enumerate(lines):
            if "Pose [" in line and i + 1 < len(lines):
                nums = lines[i + 1].strip().strip("[]").split()
                if len(nums) >= 3:
                    try:
                        return tuple(float(n) for n in nums[:3])
                    except ValueError:
                        return None
        return None

    def _set_model_pose(self, name: str, x: float, y: float, z: float) -> None:
        # "ign model" has no pose-setting flags in this version, so this
        # goes straight through gz-transport's world set_pose service
        # instead (note the "ignition.msgs" namespace, not "gz.msgs").
        req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'
        try:
            subprocess.run(
                ["ign", "service", "-s", f"/world/{GZ_WORLD_NAME}/set_pose",
                 "--reqtype", "ignition.msgs.Pose",
                 "--reptype", "ignition.msgs.Boolean",
                 "--timeout", "2000",
                 "--req", req],
                capture_output=True, text=True, timeout=3.0,
            )
        except (subprocess.SubprocessError, OSError):
            pass

    def _verify_grasp(self, model_name: str) -> bool:
        # A grasp only counts if the object actually ended up between the
        # fingers — compare its real Gazebo pose against gripper_tcp's real
        # pose, not the pose we merely planned to reach.
        obj_pose = self._get_model_pose(model_name)
        if obj_pose is None:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME, "gripper_tcp", rclpy.time.Time()
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not look up gripper_tcp in world frame: {ex}")
            return False
        t = tf.transform.translation
        dist = ((obj_pose[0] - t.x) ** 2
                + (obj_pose[1] - t.y) ** 2
                + (obj_pose[2] - t.z) ** 2) ** 0.5
        return dist < GRASP_VERIFY_RADIUS

    def _sync_attached_model_pose(self) -> None:
        if not self._attached_model:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME, "gripper_tcp", rclpy.time.Time()
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not look up gripper_tcp in world frame: {ex}")
            return
        t = tf.transform.translation
        self._set_model_pose(self._attached_model, t.x, t.y, t.z)

    # -----------------------------------------------------------------------
    # Straight-line move for short local motions (descend/retreat) — see the
    # CARTESIAN_* constants for why this exists instead of always using the
    # joint-space planner.
    # -----------------------------------------------------------------------
    def _move_linear(self, pose: Pose) -> bool:
        request = GetCartesianPath.Request()
        request.header.frame_id = ROBOT_BASE_FRAME
        request.group_name      = PLANNING_GROUP
        request.link_name       = END_EFFECTOR
        request.waypoints       = [pose]
        request.max_step        = CARTESIAN_MAX_STEP
        request.jump_threshold  = 0.0
        request.avoid_collisions = True

        future = self._cartesian_client.call_async(request)
        response = self._wait_for_future(future)

        if response.fraction < CARTESIAN_MIN_FRACTION:
            self.get_logger().warn(
                f"Cartesian path only {response.fraction:.2f} complete "
                f"(need {CARTESIAN_MIN_FRACTION}); falling back to joint-space planning."
            )
            return self._move_to_pose(pose)

        trajectory = self._retime_trajectory(response.solution, CARTESIAN_TIME_SCALE)
        return self._execute_trajectory(trajectory)

    @staticmethod
    def _retime_trajectory(trajectory, scale: float):
        # compute_cartesian_path times the trajectory at full joint speed;
        # stretch it out so joint_trajectory_controller's tracking
        # tolerances aren't tripped during execution.
        for point in trajectory.joint_trajectory.points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            t *= scale
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            point.velocities = [v / scale for v in point.velocities]
            point.accelerations = [a / (scale * scale) for a in point.accelerations]
        return trajectory

    def _execute_trajectory(self, trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        future = self._execute_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future)
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected by MoveGroup")
            return False

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future)

        error_code = result.result.error_code.val
        if error_code == 1:
            self.get_logger().info("Trajectory executed successfully")
            return True
        self.get_logger().error(f"Trajectory execution failed — MoveIt error code: {error_code}")
        return False

    # -----------------------------------------------------------------------
    # Spline joint trajectory — see the SPLINE_* constants for why this
    # exists instead of always using OMPL. Used for the longer transit
    # moves (pre-grasp approach, pre-grasp -> pre-place, home return).
    # -----------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _current_arm_positions(self):
        js = self._latest_joint_state
        if js is None:
            return None
        by_name = dict(zip(js.name, js.position))
        try:
            return [by_name[name] for name in ARM_JOINT_NAMES]
        except KeyError:
            return None

    def _resolve_goal_joints(self, pose: Pose):
        # A single-shot /compute_ik call for an *exact* pose turned out to
        # fail deterministically for some reachable targets near the edge
        # of the workspace (repro'd with 10 identical retries and 8
        # different seed configurations, all NO_IK_SOLUTION) — the
        # numerical KDL solver just can't converge there. _move_to_pose's
        # tolerance-region goal (2cm position, 0.3rad orientation) reaches
        # the same targets reliably, because OMPL's constraint sampler
        # isn't limited to one exact pose. So: plan (not execute) that
        # same goal via OMPL purely to resolve a valid joint configuration,
        # then throw the OMPL path away and build our own spline through
        # that configuration instead.
        goal = self._build_pose_goal(pose, plan_only=True)
        result = self._send_goal_get_result(goal)
        if result is None:
            return None

        points = result.planned_trajectory.joint_trajectory.points
        if not points:
            return None
        names = result.planned_trajectory.joint_trajectory.joint_names
        by_name = dict(zip(names, points[-1].positions))
        try:
            return [by_name[name] for name in ARM_JOINT_NAMES]
        except KeyError:
            return None

    @staticmethod
    def _build_spline_trajectory(start: list, goal: list) -> RobotTrajectory:
        # Per-joint cubic spline, clamped to zero velocity at both ends —
        # a single smooth accelerate/decelerate blend from start to goal,
        # instead of RRTConnect's piecewise-linear joint-space path.
        splines = [
            CubicSpline([0.0, SPLINE_DURATION], [s, g], bc_type=((1, 0.0), (1, 0.0)))
            for s, g in zip(start, goal)
        ]

        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(ARM_JOINT_NAMES)
        for i in range(SPLINE_STEPS + 1):
            t = SPLINE_DURATION * i / SPLINE_STEPS
            point = JointTrajectoryPoint()
            point.positions = [float(s(t)) for s in splines]
            point.velocities = [float(s(t, 1)) for s in splines]
            point.accelerations = [float(s(t, 2)) for s in splines]
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            trajectory.joint_trajectory.points.append(point)
        return trajectory

    def _trajectory_is_valid(self, trajectory: RobotTrajectory) -> bool:
        # Bypassing OMPL for spline moves also bypasses its collision
        # checking, so check each waypoint against the planning scene
        # (table included) before committing to executing it.
        for point in trajectory.joint_trajectory.points:
            request = GetStateValidity.Request()
            request.group_name = PLANNING_GROUP
            request.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
            request.robot_state.joint_state.position = list(point.positions)

            future = self._validity_client.call_async(request)
            response = self._wait_for_future(future)
            if not response.valid:
                return False
        return True

    def _move_spline_to_joints(self, goal: list) -> bool:
        # Returns None (rather than False) when a spline couldn't even be
        # attempted — no joint state yet, or the path is invalid — so
        # callers can tell "fall back to joint-space planning" apart from
        # "the spline was attempted and execution itself failed."
        start = self._current_arm_positions()
        if start is None:
            self.get_logger().warn("No current joint state yet; falling back to joint-space planning.")
            return None

        trajectory = self._build_spline_trajectory(start, goal)
        if not self._trajectory_is_valid(trajectory):
            self.get_logger().warn(
                "Spline path would collide with the planning scene; "
                "falling back to joint-space planning."
            )
            return None

        return self._execute_trajectory(trajectory)

    def _move_spline(self, pose: Pose) -> bool:
        goal = self._resolve_goal_joints(pose)
        if goal is None:
            self.get_logger().warn("Could not resolve a goal configuration for spline target.")
            return self._move_to_pose(pose)

        result = self._move_spline_to_joints(goal)
        if result is None:
            return self._move_to_pose(pose)
        return result

    # -----------------------------------------------------------------------
    # Move to Cartesian pose via OMPL/RRTConnect (collision-aware
    # fallback for spline moves, and used directly wherever the goal is a
    # tolerance region rather than an exact pose)
    # -----------------------------------------------------------------------
    def _build_pose_goal(self, pose: Pose, plan_only: bool) -> MoveGroup.Goal:
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

        # Orientation constraint. A tolerance of pi (the previous value) is
        # effectively unconstrained — it lets RRTConnect satisfy the goal
        # with *any* orientation, including ones needing a completely
        # different elbow/wrist configuration than a simple, direct motion
        # would need. That's what caused the arm to swing through a weird
        # pose during Step 2 instead of just moving down. 0.3 rad (~17deg)
        # keeps the end effector meaningfully pointed down while still
        # leaving enough slack for the IK solver's numerical tolerance.
        ori_c                              = OrientationConstraint()
        ori_c.header.frame_id              = ROBOT_BASE_FRAME
        ori_c.link_name                   = END_EFFECTOR
        ori_c.orientation                 = pose.orientation
        ori_c.absolute_x_axis_tolerance   = 0.3
        ori_c.absolute_y_axis_tolerance   = 0.3
        ori_c.absolute_z_axis_tolerance   = 0.3
        ori_c.weight                      = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_c)
        constraints.orientation_constraints.append(ori_c)
        request.goal_constraints.append(constraints)

        goal.request                          = request
        goal.planning_options.plan_only       = plan_only
        goal.planning_options.replan          = not plan_only
        goal.planning_options.replan_attempts = 0 if plan_only else 5

        return goal

    def _move_to_pose(self, pose: Pose) -> bool:
        goal = self._build_pose_goal(pose, plan_only=False)
        return self._send_goal(goal)

    # -----------------------------------------------------------------------
    # Move to home — spline first (see _move_spline), falling back to the
    # collision-aware joint-space planner if the spline path is invalid.
    # -----------------------------------------------------------------------
    def _move_to_home(self) -> bool:
        result = self._move_spline_to_joints(HOME_JOINT_VALUES)
        if result is None:
            return self._move_to_home_planned()
        return result

    def _move_to_home_planned(self) -> bool:
        goal    = MoveGroup.Goal()
        request = MotionPlanRequest()

        request.group_name                      = PLANNING_GROUP
        request.num_planning_attempts           = 10
        request.allowed_planning_time           = PLANNING_TIME
        request.max_velocity_scaling_factor     = MAX_VELOCITY
        request.max_acceleration_scaling_factor = MAX_ACCEL

        constraints = Constraints()
        for name, value in zip(ARM_JOINT_NAMES, HOME_JOINT_VALUES):
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
    # Wait for a future without spinning — this is called from a worker
    # thread (see _run_pick_and_place), never from inside a callback. The
    # one real spin loop is rclpy.spin(node) in main(), running concurrently
    # on the main thread; nesting rclpy.spin_until_future_complete() inside
    # a callback that spin loop is already dispatching is a known rclpy
    # re-entrancy hazard — the goal-acceptance future reliably resolves that
    # way, but the action *result* future reliably never does.
    # -----------------------------------------------------------------------
    @staticmethod
    def _wait_for_future(future):
        while not future.done():
            time.sleep(0.02)
        return future.result()

    # -----------------------------------------------------------------------
    # Send goal and wait for result. _send_goal_get_result returns the full
    # MoveGroupResult (or None on failure) so callers that need more than a
    # bool — _resolve_goal_joints wants the planned_trajectory — can get it.
    # -----------------------------------------------------------------------
    def _send_goal_get_result(self, goal: MoveGroup.Goal):
        future = self._action_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future)

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup")
            return None

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future)

        error_code = result.result.error_code.val
        if error_code == 1:
            self.get_logger().info("Motion executed successfully")
            return result.result
        else:
            self.get_logger().error(f"Motion failed — MoveIt error code: {error_code}")
            return None

    def _send_goal(self, goal: MoveGroup.Goal) -> bool:
        return self._send_goal_get_result(goal) is not None


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