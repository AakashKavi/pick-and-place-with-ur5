"""
pick_place.launch.py

Starts the pick-and-place operation on top of an already-running
simulation (see sim.launch.py):
  1. Perception node (object_detector) — publishes detected object poses
  2. Manipulation node (pick_place_node) — picks the cylinder up and
     places it on the cube via MoveIt!

Run sim.launch.py first and wait for MoveIt/RViz to come up, then run this.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    perception_node = Node(
        package='ur5_pick_place_perception',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'cluster_tolerance': 0.02,
            'min_cluster_size': 50,
            'max_cluster_size': 25000,
            'passthrough_min_z': 0.55,
            'passthrough_max_z': 0.79,
            'plane_distance_threshold': 0.01,
            'voxel_leaf_size': 0.005,
        }]
    )

    manipulation_node = Node(
        package='ur5_pick_place_manipulation',
        executable='pick_place_node',
        name='pick_place_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        perception_node,
        manipulation_node,
    ])
