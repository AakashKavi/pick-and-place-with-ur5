# ROS2 Vision-Based Pick and Place with UR5e

[![ROS2 Build](https://github.com/AakashKavi/ros2_ur5_vision_grasp/actions/workflows/ci.yml/badge.svg)](https://github.com/AakashKavi/ros2_ur5_vision_grasp/actions/workflows/ci.yml)

### Closing the loop between 3D point cloud perception and industrial arm motion planning — entirely in simulation, no custom robot configuration required.

---

##  Project Overview

Standard pick-and-place tutorials either skip perception entirely (hardcoded object positions) or skip manipulation (just detect, never act). This project builds the full pipeline: a depth camera sees objects on a table, a PCL-based perception node detects them, and MoveIt 2 commands a UR5e arm — fitted with a custom parallel-jaw gripper — to pick one up and place it on another.

The simulation runs on ROS2 Humble with the new Gazebo (gz-sim), using the official Universal Robots ROS2 driver as the base — the arm and controllers are stock, with a custom gripper and MoveIt config layered on top.

**Current status:** Full pipeline working end-to-end — perception detects the objects, MoveIt 2 plans collision-aware motions around a registered table, the gripper picks up the cylinder and places it on top of the cube.

---

## System Architecture

<p align="center">
  <img src="architecture.png" alt="System architecture: depth camera publishes PointCloud2 to the PCL perception node, which sends detected object poses to the MoveIt 2 manipulation node commanding the UR5e" width="850"/>
</p>

---

##  Key Features

* **Full perception-to-manipulation pipeline** — depth camera → PCL clustering → MoveIt 2 pick-and-place sequence, all connected over ROS2 topics
* **Custom mock parallel-jaw gripper** — added on top of the stock UR5e, driven directly by a gz-sim `JointPositionController` plugin, sized to actually fit around the scene's objects
* **Pick-and-place, not just pick** — identifies the cylinder vs. the cube (via nearest-match against ground truth, since perception doesn't classify shape), picks the cylinder up and stacks it on the cube
* **Collision-aware planning** — the table is registered with MoveIt as a real `CollisionObject`, so motions are actually planned around it instead of just being statically unlikely to hit it
* **Mixed trajectory types per motion** — long transit moves use a cubic-spline joint trajectory (smooth single accelerate/decelerate blend, collision-checked per waypoint); short local moves (descend/retreat to grasp or place) use Cartesian linear interpolation so the end effector travels a straight line instead of an arbitrary joint-space path
* **Grasp verification + retry** — after closing the gripper, checks the object's actual Gazebo pose against the gripper's actual pose (not just the planned target) before declaring success; retries the whole approach up to 3 times if the grasp missed
* **Kinematic attach** — since the gripper doesn't grip via contact physics, a held object's Gazebo pose is teleported to track the gripper between waypoints
* **New Gazebo (gz-sim) compatible** — world file written in SDF 1.9 with gz-sim system plugins, not Gazebo Classic syntax
* **ros_gz_bridge integration** — explicit bridge nodes translate `gz.msgs.PointCloudPacked` and the gripper command topic between ROS2 and Gazebo
* **C++ PCL perception node** — four-stage pipeline: voxel grid downsampling → passthrough filter → RANSAC plane removal → Euclidean cluster extraction
* **RViz marker visualization** — detected object centroids published as `visualization_msgs/MarkerArray` for real-time inspection
* **GitHub Actions CI** — automated build check on every push

---

##  System Requirements

* OS: Ubuntu 22.04 LTS
* Middleware: ROS2 Humble Hawksbill
* Simulation: Gazebo (gz-sim / Ignition Fortress)

Dependencies:
```bash
sudo apt install \
  ros-humble-ur \
  ros-humble-ur-simulation-gazebo \
  ros-humble-moveit \
  ros-humble-pcl-ros \
  ros-humble-ros-gz-bridge \
  ros-humble-gazebo-ros-pkgs \
  python3-scipy
```

---

##  Installation

```bash
# Clone into your ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/AakashKavi/ros2_ur5_vision_grasp.git

# Resolve dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

---

##  Usage

Simulation and the pick-and-place operation are launched separately.

```bash
# 1. Launch the simulation: Gazebo + UR5e + gripper + MoveIt 2
ros2 launch ur5_pick_place_bringup sim.launch.py
```

Launch sequence:
1. Gazebo starts with the custom world (table, objects, depth camera)
2. MoveIt 2 move_group connects (5s delay)
3. Camera and gripper command bridges start

Wait for RViz/MoveIt to finish connecting, then in a second terminal:

```bash
# 2. Start perception + the pick-and-place manipulation node
ros2 launch ur5_pick_place_bringup pick_place.launch.py
```

This starts the perception node (publishing `/detected_objects`) and the
manipulation node, which picks up the cylinder and places it on the cube.

To verify the camera bridge is active:
```bash
ros2 topic info /camera/depth/points
# Expected: Publisher count: 1
```

To inspect detected objects:
```bash
ros2 topic echo /detected_objects
```

---

## Package Structure

```
ros2_ur5_vision_grasp/
├── ur5_pick_place_bringup/           # Launch files, Gazebo world, config
│   ├── launch/sim.launch.py          # Simulation only: Gazebo + UR5e + gripper + MoveIt 2
│   ├── launch/pick_place.launch.py   # Perception + manipulation, run against a live sim
│   ├── worlds/pick_place.sdf         # SDF 1.9 world: table + cube + cylinder + camera
│   └── config/ur5e_gripper_controllers.yaml
├── ur5_pick_place_description/       # UR5e + gripper xacro (overlay on ur_description)
│   └── urdf/gripper.xacro            # Mock parallel-jaw gripper + gz JointPositionController
├── ur5_pick_place_moveit_config/     # Custom SRDF (gripper_tcp as the planning tip) + kinematics
├── ur5_pick_place_perception/        # C++ PCL object detection node
│   ├── include/.../object_detector.hpp
│   └── src/object_detector.cpp       # 4-stage PCL pipeline
└── ur5_pick_place_manipulation/      # Python MoveIt 2 pick-and-place sequence
    └── .../pick_place_node.py        # 8-step pick-cylinder-place-on-cube execution
```

---

## Key Insight: Gazebo Classic vs New Gazebo

The biggest debugging challenge in this project was not the perception or manipulation code — it was the camera.

`ur_simulation_gz` (the official UR ROS2 driver) runs on the **new Gazebo (gz-sim/Ignition)**, not Gazebo Classic. The two versions look similar on the surface but use completely different plugin architectures under the hood.

The initial world file used Gazebo Classic syntax (`libgazebo_ros_camera.so`, `<ros>` topic remappings). This produced no errors — it simply did nothing, silently, in the new Gazebo.

The fix required three changes:
1. Rewrite the world file in SDF 1.9 with gz-sim system plugins (`gz-sim-sensors-system`, `gz-sim-physics-system`, etc.)
2. Declare the camera sensor using gz-sim's native `depth_camera` type with a `<topic>` tag
3. Add an explicit `ros_gz_bridge` node in the launch file to translate `gz.msgs.PointCloudPacked` into ROS2 `sensor_msgs/PointCloud2`

---

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | Camera → Perception |
| `/detected_objects` | `geometry_msgs/PoseArray` | Perception → Manipulation |
| `/detected_objects_markers` | `visualization_msgs/MarkerArray` | Perception → RViz |
| `/gripper_position_cmd` | `std_msgs/Float64` | Manipulation → Gripper (gz `JointPositionController`) |

---

## Future Scope

* Fix ogre2 depth camera rendering to produce valid (non-NaN) point cloud values in headless Gazebo
* Replace centroid heuristic with learned grasp pose estimation (GraspNet or similar)
* Add gripper state feedback using a simulated force/torque sensor, and grip via real contact physics instead of kinematic attach
* Implement multi-object sequencing (pick and place all objects in order)
* Add eye-in-hand camera configuration as an alternative to fixed overhead mount
* Publish planned trajectories to RViz's `MotionPlanning` display for visualization

---

## Author

**Aakash Kavi** — Robotics and Automation Engineer  
M.Sc. Mechatronics, Hochschule Ravensburg-Weingarten  
[LinkedIn](https://linkedin.com/in/YOUR_PROFILE) | [GitHub](https://github.com/AakashKavi)
