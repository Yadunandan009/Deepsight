import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Define package directories FIRST
    stonefish_ros2_dir = get_package_share_directory('stonefish_ros2')
    stonefish_bluerov2_dir = get_package_share_directory('stonefish_bluerov2')

    # 2. Define File Paths
    ekf_config_path = os.path.join(stonefish_bluerov2_dir, 'ekf.yaml')
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
    slam_pose_bridge = Node(
        package='stonefish_bluerov2',
        executable='slam_pose_bridge.py',
        name='slam_pose_bridge',
        output='screen'
    )

    # 5b. DVL Bridge
    dvl_bridge = Node(
        package='stonefish_bluerov2',
        executable='dvl_bridge.py',
        name='dvl_bridge',
        output='screen'
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
    # than replacing it.
    image_enhancement = Node(
        package='stonefish_bluerov2',
        executable='image_enhancement_node.py',
        name='image_enhancement_node',
        output='screen',
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

    return LaunchDescription([
        scenario_arg,
        quality_arg,
        rviz_arg,
        launch_include,
        ardusim_patch,
        depth_bridge,
        slam_pose_bridge,
        dvl_bridge,
        adaptive_fusion,
        image_enhancement,
        ekf_node,
        rviz_node,
        static_tf,
        ned_to_enu_tf,
        world_to_odom_tf,
    ])
