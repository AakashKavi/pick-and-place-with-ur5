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

    # --------------------------------------------------------------------------
    # Camera mount pose: declared here, once, as this launch file's own
    # arguments. This is the single source of truth for where the camera
    # is — pick_place_node.py no longer hardcodes it (it looks the published
    # TF transform up via tf2 instead, see objects_callback in
    # pick_place_node.py). pick_place.sdf still has its own independent copy
    # of these same numbers in the depth_camera model's <pose>, since gz-sim
    # reads the world file directly and has no way to consume a ROS2 launch
    # argument; that copy is commented there to say it must match these
    # defaults.
    # --------------------------------------------------------------------------
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.65')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='0')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='1.3')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='1.5708')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0')

    # Static transform: base_link -> depth_camera/link/depth_camera
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
            LaunchConfiguration('camera_x'),
            LaunchConfiguration('camera_y'),
            LaunchConfiguration('camera_z'),
            LaunchConfiguration('camera_roll'),
            LaunchConfiguration('camera_pitch'),
            LaunchConfiguration('camera_yaw'),
            'base_link',                        # parent frame
            'depth_camera/link/depth_camera',   # child frame
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

    # --------------------------------------------------------------------------
    # 5. ros_gz_bridge: ground-truth model poses, read + write, for
    # pick_place_node.py. Replaces the "ign model -p" / "ign service
    # set_pose" subprocess CLI calls that version used to shell out to.
    #
    # Read side: gz-sim's own dynamic_pose/info topic (ignition.msgs.Pose_V,
    # one entry per non-static entity, keyed by bare entity name in
    # child_frame_id, confirmed live: "ros2 topic echo
    # /world/pick_place_world/dynamic_pose/info" showed child_frame_id:
    # blue_cylinder / red_box with the expected spawn coordinates) bridges
    # cleanly to tf2_msgs/msg/TFMessage; ros_gz_bridge has that conversion
    # built in (confirmed in its compiled symbol table before relying on
    # it: FactoryI<tf2_msgs::msg::TFMessage, ignition::msgs::Pose_V>).
    #
    # Write side: gz-sim's /world/<world>/set_pose service bridges cleanly
    # to ros_gz_interfaces/srv/SetEntityPose (also confirmed in the
    # compiled symbol table, then verified live: calling it on
    # blue_cylinder and reading its pose back immediately after showed the
    # teleport actually happened, including the object subsequently
    # falling under gravity exactly as a real teleport-then-physics-steps
    # would produce). No subprocess fallback was needed for either
    # direction.
    # --------------------------------------------------------------------------
    world_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='world_pose_bridge',
        arguments=[
            '/world/pick_place_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    world_set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='world_set_pose_bridge',
        arguments=[
            '/world/pick_place_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        ur_type_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        camera_tf,
        ur_sim_control,
        ur_moveit,
        camera_bridge,
        gripper_bridge,
        world_pose_bridge,
        world_set_pose_bridge,
    ])
