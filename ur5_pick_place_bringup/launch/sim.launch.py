"""
sim.launch.py

Launches the full pick-and-place simulation:
  1. Gazebo with custom world (table + objects + depth camera) via ur_sim_control
  2. MoveIt! 2 move_group (delayed 5s) via ur_moveit
  3. ros_gz_bridge for depth camera point cloud
  4. Perception node (object_detector, delayed 10s)
  5. Manipulation node (pick_place_node, delayed 12s)
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

    world_file = PathJoinSubstitution([
        FindPackageShare('ur5_pick_place_bringup'),
        'worlds',
        'pick_place.sdf'
    ])

    # --------------------------------------------------------------------------
    # 1. UR5e Gazebo simulation (gz-sim + ros2_control)
    # --------------------------------------------------------------------------
    ur_sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_simulation_gz'),
                'launch',
                'ur_sim_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'world_file': world_file,
        }.items()
    )

    # --------------------------------------------------------------------------
    # 2. MoveIt! 2 (delayed 5s to let Gazebo + controllers come up)
    # --------------------------------------------------------------------------
    ur_moveit = TimerAction(
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
    # 3. ros_gz_bridge: bridge depth point cloud from gz to ROS2
    # --------------------------------------------------------------------------
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_depth_bridge',
        arguments=[
            '/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # --------------------------------------------------------------------------
    # 4. Perception node — subscribes to /camera/depth/points,
    #    publishes detected object poses to /detected_objects
    # --------------------------------------------------------------------------
    perception_node = TimerAction(
        period=10.0,
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
    # 5. Manipulation node — subscribes to /detected_objects,
    #    executes pick-and-place via MoveIt!
    # --------------------------------------------------------------------------
    manipulation_node = TimerAction(
        period=12.0,
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
        ur_sim_control,
        ur_moveit,
        camera_bridge,
        perception_node,
        manipulation_node,
    ])
