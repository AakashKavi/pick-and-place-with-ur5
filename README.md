# ROS2 Vision-Based Pick and Place with UR5e

[![ROS2 Build](https://github.com/AakashKavi/ros2_ur5_vision_grasp/actions/workflows/ci.yml/badge.svg)](https://github.com/AakashKavi/ros2_ur5_vision_grasp/actions/workflows/ci.yml)

### Closing the loop between 3D point cloud perception and industrial arm motion planning — entirely in simulation, no custom robot configuration required.

---

##  Project Overview

Standard pick-and-place tutorials either skip perception entirely (hardcoded object positions) or skip manipulation (just detect, never act). This project builds the full pipeline: a depth camera sees objects on a table, a PCL-based perception node detects them, and MoveIt 2 commands a UR5e arm to pick them up.

The simulation runs on ROS2 Humble with the new Gazebo (gz-sim), using the official Universal Robots ROS2 driver — no custom URDF or robot configuration needed, just the application layer on top.

**Current status:** Simulation running, MoveIt 2 connected, depth camera bridge active. Object detection and pick execution in progress (Part 2).

---

## System Architecture

<p align="center">
  <img src="architecture.png" alt="System architecture: depth camera publishes PointCloud2 to the PCL perception node, which sends detected object poses to the MoveIt 2 manipulation node commanding the UR5e" width="850"/>
</p>

---

##  Key Features

* **Full perception-to-manipulation pipeline** — depth camera → PCL clustering → MoveIt 2 pick sequence, all connected over ROS2 topics
* **New Gazebo (gz-sim) compatible** — world file written in SDF 1.9 with gz-sim system plugins, not Gazebo Classic syntax
* **ros_gz_bridge integration** — explicit bridge node translates `gz.msgs.PointCloudPacked` into ROS2 `sensor_msgs/PointCloud2`
* **C++ PCL perception node** — four-stage pipeline: voxel grid downsampling → passthrough filter → RANSAC plane removal → Euclidean cluster extraction
* **Python MoveIt 2 manipulation node** — five-step pick sequence: pre-grasp → descend → grasp → retreat → home
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
  ros-humble-gazebo-ros-pkgs
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

```bash
# Launch full simulation: Gazebo + UR5e + MoveIt 2 + perception + manipulation
ros2 launch ur5_pick_place_bringup sim.launch.py
```

The launch sequence:
1. Gazebo starts with the custom world (table, objects, depth camera)
2. MoveIt 2 move_group connects (5s delay)
3. Camera bridge starts publishing `/camera/depth/points`
4. Perception node starts processing point clouds (10s delay)
5. Manipulation node starts listening for detected objects (12s delay)

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
├── ur5_pick_place_bringup/         # Launch files, Gazebo world, config
│   ├── launch/sim.launch.py        # Master launch file
│   └── worlds/pick_place.sdf       # SDF 1.9 world: table + objects + camera
├── ur5_pick_place_perception/      # C++ PCL object detection node
│   ├── include/.../object_detector.hpp
│   └── src/object_detector.cpp     # 4-stage PCL pipeline
└── ur5_pick_place_manipulation/    # Python MoveIt 2 pick sequence
    └── .../pick_place_node.py      # 5-step pick execution
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

---

## Future Scope

* Fix ogre2 depth camera rendering to produce valid (non-NaN) point cloud values in headless Gazebo
* Replace centroid heuristic with learned grasp pose estimation (GraspNet or similar)
* Add gripper state feedback using a simulated force/torque sensor
* Implement multi-object sequencing (pick all objects in order)
* Add eye-in-hand camera configuration as an alternative to fixed overhead mount

---

## Author

**Aakash Kavi** — Robotics and Automation Engineer  
M.Sc. Mechatronics, Hochschule Ravensburg-Weingarten  
[LinkedIn](https://linkedin.com/in/YOUR_PROFILE) | [GitHub](https://github.com/AakashKavi)
