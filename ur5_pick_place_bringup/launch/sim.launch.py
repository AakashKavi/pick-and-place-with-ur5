"""
sim.launch.py

Launches the full pick-and-place simulation:
  1. Gazebo with custom world (table + objects + depth camera)
  2. UR5e robot (via ur_simulation_gazebo)
  3. MoveIt! 2 move_group
  4. Perception node (object_detector)
  5. Manipulation node (pick_place_node)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='UR robot type (ur3, ur5e, ur10e, etc.)'
    )

    ur_type = LaunchConfiguration('ur_type')

    # --------------------------------------------------------------------------
    # 1. UR5e Gazebo simulation (from Universal Robots ROS2 driver)
    #    This starts Gazebo, spawns the robot, and loads ros2_control.
    # --------------------------------------------------------------------------
    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_simulation_gazebo'),
                'launch',
                'ur_sim_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': 'false',         # We launch our own RViz below
        }.items()
    )

    # --------------------------------------------------------------------------
    # 2. MoveIt! move_group
    #    Delay slightly to ensure ros2_control is ready first.
    # --------------------------------------------------------------------------
    moveit = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ur_moveit_config'),
                        'launch',
                        'ur_moveit.launch.py'
                    ])
                ]),
                launch_arguments={
                    'ur_type': ur_type,
                    'use_sim_time': 'true',
                    'launch_rviz': 'true',
                }.items()
            )
        ]
    )

    # --------------------------------------------------------------------------
    # 3. Perception node — subscribes to /camera/depth/points,
    #    publishes detected object poses to /detected_objects
    # --------------------------------------------------------------------------
    perception_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ur5_pick_place_perception',
                executable='object_detector',
                name='object_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'cluster_tolerance': 0.02,
                    'min_cluster_size': 100,
                    'max_cluster_size': 25000,
                    'passthrough_min_z': 0.05,
                    'passthrough_max_z': 0.80,
                }]
            )
        ]
    )

    # --------------------------------------------------------------------------
    # 4. Manipulation node — subscribes to /detected_objects,
    #    executes pick-and-place via MoveIt!
    # --------------------------------------------------------------------------
    manipulation_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ur5_pick_place_manipulation',
                executable='pick_place_node',
                name='pick_place_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                }]
            )
        ]
    )

    return LaunchDescription([
        ur_type_arg,
        ur_sim,
        moveit,
        perception_node,
        manipulation_node,
    ])
