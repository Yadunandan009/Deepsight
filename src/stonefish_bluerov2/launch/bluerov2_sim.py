import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Define package directories FIRST
    stonefish_ros2_dir = get_package_share_directory('stonefish_ros2')
    stonefish_bluerov2_dir = get_package_share_directory('stonefish_bluerov2')

    # 2. Define File Paths
    ekf_config_path = os.path.join(stonefish_bluerov2_dir, 'ekf.yaml')
    ekf_dr_config_path = os.path.join(stonefish_bluerov2_dir, 'ekf_dr.yaml')
    rviz_config_path = os.path.join(stonefish_bluerov2_dir, 'rviz', 'bluerov2_conf.rviz')

    # 2a. Scenario selection -- was previously hardcoded to bluerov2_turbine.scn
    # despite CLAUDE.md documenting `scenario:=<name>` as a real launch argument.
    # It wasn't; this restores that documented behavior.
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='bluerov2_turbine',
        description='Scenario name (without .scn) from stonefish_bluerov2/scenarios/'
    )
    scenario = LaunchConfiguration('scenario')

    # 2a1. Reference for the SLAM->world alignment. See slam_pose_bridge below.
    dr_reference_arg = DeclareLaunchArgument(
        'dr_reference',
        default_value='false',
        description='Align SLAM against a SLAM-free dead-reckoning filter '
                    '(/bluerov2/odometry/dr) instead of the filter the bridge '
                    'feeds. Opens the SLAM->bridge->EKF->bridge loop.'
    )
    dr_reference = LaunchConfiguration('dr_reference')

    # 2a2. Rendering quality -- was hardcoded to 'high', which drives
    # sustained GPU load/heat hard enough to cause thermal throttling on
    # this laptop (observed: 86C, and camera framerate visibly decaying
    # from ~5Hz to ~3Hz over a 10s window as temperature climbed). Made
    # tunable so a lighter setting can be tried without editing this file.
    quality_arg = DeclareLaunchArgument(
        'rendering_quality', default_value='high',
        description='Stonefish rendering quality: low|medium|high'
    )
    rendering_quality = LaunchConfiguration('rendering_quality')

    # 2b. RViz toggle -- for unattended batch evaluation runs, popping up a
    # GUI window per trial repeatedly steals desktop focus for no benefit.
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to launch RViz2 (set false for unattended batch runs)'
    )
    rviz_enabled = LaunchConfiguration('rviz')

    dvl_error_arg = DeclareLaunchArgument(
        'dvl_error', default_value='none',
        description="DVL error model: 'none' (white noise only, as Stonefish "
                    "models it) or 'a50' (adds datasheet scale factor and beam "
                    "misalignment, which drift linearly with distance)")
    dvl_error = LaunchConfiguration('dvl_error')

    dvl_seed_arg = DeclareLaunchArgument(
        'dvl_seed', default_value='0',
        description='Selects which DVL unit is simulated; sweep for a spread')
    dvl_seed = LaunchConfiguration('dvl_seed')

    # 3. Stonefish Simulator
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(stonefish_ros2_dir, 'launch', 'stonefish_simulator.launch.py')),
        launch_arguments={
            'simulation_data': os.path.join(stonefish_bluerov2_dir, 'data', ''),
            'scenario_desc': [os.path.join(stonefish_bluerov2_dir, 'scenarios') + os.sep, scenario, '.scn'],
            'simulation_rate': '200.0',
            'window_res_x': '960',
            'window_res_y': '1056',
            'rendering_quality': rendering_quality,
        }.items()
    )

    # 4. ArduSim Patch
    ardusim_patch = Node(
        package='stonefish_bluerov2',
        namespace='bluerov2',
        executable='ardusim_patch.py',
        name='ardusim_patch',
        output='screen',
        emulate_tty=True,
    )

    # 5. Depth Bridge
    depth_bridge = Node(
        package='stonefish_bluerov2',
        executable='depth_bridge.py',
        name='depth_bridge',
        output='screen'
    )

    # 5a. SLAM Pose Bridge
    #
    # dr_reference:=false  (default) the bridge aligns the SLAM track against
    #                      /bluerov2/odometry/filtered -- the output of the
    #                      filter it feeds through pose0. The rotation is then
    #                      fitted against a signal it produces, which is not
    #                      observable: seeds 0 and 1 both locked theta at
    #                      +152.4 deg where +14.4 deg was correct and settled
    #                      ~41 m from truth. This is the behaviour under test,
    #                      kept as the default so existing bags stay
    #                      comparable.
    # dr_reference:=true   the bridge aligns against /bluerov2/odometry/dr, a
    #                      second filter (ekf_dr.yaml) that fuses DVL, depth
    #                      and IMU and has never seen SLAM. The loop opens.
    #
    # NOT /bluerov2/odometry under any circumstances: that is Stonefish ground
    # truth and would re-anchor the system to it.
    bridge_reference_odom = PythonExpression([
        "'/bluerov2/odometry/dr' if '", dr_reference,
        "'.lower() == 'true' else '/bluerov2/odometry/filtered'"
    ])
    slam_pose_bridge = Node(
        package='stonefish_bluerov2',
        executable='slam_pose_bridge.py',
        name='slam_pose_bridge',
        output='screen',
        # ParameterValue, not the bare substitution: a PythonExpression is a
        # substitution object, and handing one straight to parameters= is what
        # silently voided the first a50 run.
        parameters=[{'reference_odom': ParameterValue(bridge_reference_odom,
                                                      value_type=str)}],
    )

    # 5b. DVL Bridge
    #
    # dvl_error:=none      white noise only, as Stonefish models it. This is
    #                      optimistic: it drifts as sqrt(time), not as distance.
    # dvl_error:=a50       adds the scale factor and beam misalignment a real
    #                      unit has, at Nortek DVL A50 datasheet level. These
    #                      are fixed per unit and integrate linearly.
    # dvl_seed:=<int>      which unit you have. Sweep it for a distribution.
    #
    # Two nodes under opposite conditions rather than one node with a
    # conditional parameter list.  PythonExpression returns a STRING, so
    # building the list inside one meant `parameters` received the literal
    # text "['/.../dvl_a50.yaml']" -- a filename that does not exist, which
    # ROS silently ignored, leaving the zero defaults.  The 2026-10-09
    # seed 0 run was recorded that way and is a white-noise bag wearing an
    # a50 name; only the scale factor recovered from /bluerov2/dvl_twist
    # caught it.  A condition cannot fail this way.
    _dvl_is_a50 = PythonExpression(["'", dvl_error, "' == 'a50'"])
    _dvl_common = dict(
        package='stonefish_bluerov2',
        executable='dvl_bridge.py',
        name='dvl_bridge',
        output='screen',
    )
    dvl_bridge = Node(
        condition=UnlessCondition(_dvl_is_a50),
        parameters=[{'error_seed': ParameterValue(dvl_seed, value_type=int)}],
        **_dvl_common,
    )
    dvl_bridge_a50 = Node(
        condition=IfCondition(_dvl_is_a50),
        parameters=[
            os.path.join(stonefish_bluerov2_dir, 'dvl_a50.yaml'),
            {'error_seed': ParameterValue(dvl_seed, value_type=int)},
        ],
        **_dvl_common,
    )

    # 5c. Adaptive Fusion Node — confidence-driven SLAM/EKF blending
    adaptive_fusion = Node(
        package='stonefish_bluerov2',
        executable='adaptive_fusion_node.py',
        name='adaptive_fusion_node',
        output='screen',
        emulate_tty=True,
    )

    # 5d. Image Enhancement — CLAHE + gamma + sharpening for the main
    # inspection camera, to counter contrast/color loss under degraded
    # (high-Jerlov) visibility. Publishes alongside the raw feed rather
    # than replacing it. One instance per stereo eye with identical
    # parameters -- ORB-SLAM3's stereo matching needs both eyes processed
    # the same way, or the mismatched image statistics between an enhanced
    # left and a raw right would corrupt correspondence. (Previously only
    # the left eye had an enhancement instance, and neither eye's enhanced
    # topic was actually wired into ORB-SLAM3's input remap -- see
    # run_stereo_slam.sh -- so the enhancement was invisible in the
    # Pangolin viewer regardless of tuning.)
    image_enhancement_left = Node(
        package='stonefish_bluerov2',
        executable='image_enhancement_node.py',
        name='image_enhancement_node_left',
        output='screen',
        parameters=[{
            'input_topic': '/bluerov2/left/image_color',
            'output_topic': '/bluerov2/left/image_enhanced',
        }],
    )
    image_enhancement_right = Node(
        package='stonefish_bluerov2',
        executable='image_enhancement_node.py',
        name='image_enhancement_node_right',
        output='screen',
        parameters=[{
            'input_topic': '/bluerov2/right/image_color',
            'output_topic': '/bluerov2/right/image_enhanced',
        }],
    )

    # 6. EKF Filter
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', '/bluerov2/odometry/filtered')]
    )

    # 6a. Dead-reckoning EKF -- only when the bridge is told to reference it.
    #
    # Nothing consumes /bluerov2/odometry/dr except slam_pose_bridge: the
    # controller still flies on /bluerov2/odometry/filtered, so SLAM remains
    # the absolute position reference and this filter only supplies the
    # short-horizon baseline that makes the alignment observable. It publishes
    # no TF (ekf_dr.yaml), so the primary filter keeps the transform tree.
    ekf_dr_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_dr_node',
        output='screen',
        parameters=[ekf_dr_config_path],
        remappings=[('odometry/filtered', '/bluerov2/odometry/dr')],
        condition=IfCondition(dr_reference),
    )

    # 7. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(rviz_enabled),
    )

    # 8. Static TF (World to Map)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_link',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    # 9. Static TF (world_ned to world) — NED to ENU conversion
    ned_to_enu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ned_to_enu',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'world_ned']
    )

    # 10. Static TF (world to odom)
    world_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    # 11. Static TF (base_link to imu_filter) -- identity, matching
    # bluerov2.scn's imu_filter sensor origin (rpy 0 0 0, xyz 0 0 0). Nothing
    # published this before: Stonefish's ROS2Interface::PublishTF exists but
    # is never called, and there's no robot_state_publisher/URDF here either.
    # ekf.yaml's imu0 needs base_link<->bluerov2/imu_filter (the frame_id on
    # /bluerov2/imu) to resolve, since they're different frame names -- with
    # no transform published, that lookup fails on every single IMU message,
    # which makes robot_localization skip fusing it entirely (confirmed by
    # reading ros_filter.cpp's prepareIMU/preparePose path: the whole fusion
    # block is gated on a successful lookupTransformSafe). Roll/pitch have no
    # other correcting sensor in this config, so with imu0 silently never
    # landing, they've been running open-loop/uncorrected the whole session --
    # the actual root cause of the pitch drift into +/-90deg gimbal lock this
    # session chased through two rounds of covariance/process-noise tuning
    # that could never have worked, since the measurement never reached the
    # filter in the first place.
    base_link_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_filter',
        arguments=['0', '0', '0', '0', '0', '0', 'bluerov2/base_link', 'bluerov2/imu_filter']
    )

    return LaunchDescription([
        scenario_arg,
        quality_arg,
        rviz_arg,
        dvl_error_arg,
        dvl_seed_arg,
        dr_reference_arg,
        launch_include,
        ardusim_patch,
        depth_bridge,
        slam_pose_bridge,
        dvl_bridge,
        dvl_bridge_a50,
        adaptive_fusion,
        image_enhancement_left,
        image_enhancement_right,
        ekf_node,
        ekf_dr_node,
        rviz_node,
        static_tf,
        ned_to_enu_tf,
        world_to_odom_tf,
        base_link_to_imu_tf,
    ])
