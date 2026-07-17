"""
sim.launch.py

Launches only the simulation (no perception/manipulation):
  1. Gazebo with custom world (table + objects + depth camera) via ur_sim_control
  2. MoveIt! 2 move_group (delayed 5s) via ur_moveit
  3. ros_gz_bridge for depth camera point cloud
  4. ros_gz_bridge for the gripper position command topic

Once this is up and MoveIt is connected, run pick_place.launch.py separately
to start perception + the pick-and-place manipulation node.
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

    # Static transform: base_link -> depth_camera/link/depth_camera
    # Camera is fixed at 0.65m forward, 1.3m up, pointing straight down —
    # must match the depth_camera model's pose in pick_place.sdf exactly.
    # Parented to base_link (not base_link_inertia): base_link is coincident
    # with world (identity fixed joint in ur.urdf.xacro), matching the
    # camera's actual world-frame pose in pick_place.sdf.
    # base_link_inertia is base_link rotated 180deg about Z, which would
    # place this TF frame on the wrong side of the robot.
    camera_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_tf_publisher',
    arguments=[
        '0.65', '0', '1.3',      # x y z translation
        '0', '1.5708', '0',      # roll pitch yaw (pointing down)
        'base_link',              # parent frame
        'depth_camera/link/depth_camera'  # child frame
    ]
    )

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
            'description_package': 'ur5_pick_place_description',
            'description_file': 'ur5e_gripper.urdf.xacro',
            'runtime_config_package': 'ur5_pick_place_bringup',
            'controllers_file': 'ur5e_gripper_controllers.yaml',
        }.items()
    )

    # --------------------------------------------------------------------------
    # Gripper bridge: the gripper's prismatic joint is driven directly by a
    # gz-sim JointPositionController plugin (see gripper.xacro) rather than
    # ros2_control, so it just needs a ROS2 -> Gazebo topic bridge.
    # --------------------------------------------------------------------------
    gripper_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gripper_position_bridge',
        arguments=[
            '/gripper_position_cmd@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
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
                    'description_package': 'ur5_pick_place_description',
                    'description_file': 'ur5e_gripper.urdf.xacro',
                    'moveit_config_package': 'ur5_pick_place_moveit_config',
                    'moveit_config_file': 'ur5e_gripper.srdf.xacro',
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

    return LaunchDescription([
        camera_tf,
        ur_type_arg,
        ur_sim_control,
        ur_moveit,
        camera_bridge,
        gripper_bridge,
    ])
