import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Define package directories FIRST
    stonefish_ros2_dir = get_package_share_directory('stonefish_ros2')
    stonefish_bluerov2_dir = get_package_share_directory('stonefish_bluerov2')

    # 2. Define File Paths
    ekf_config_path = os.path.join(stonefish_bluerov2_dir, 'ekf.yaml')
    rviz_config_path = os.path.join(stonefish_bluerov2_dir, 'rviz', 'bluerov2_conf.rviz')

    # 3. Stonefish Simulator
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(stonefish_ros2_dir, 'launch', 'stonefish_simulator.launch.py')),
        launch_arguments={
            'simulation_data': os.path.join(stonefish_bluerov2_dir, 'data', ''),
            'scenario_desc': os.path.join(stonefish_bluerov2_dir, 'scenarios', 'bluerov2_turbine.scn'),
            'simulation_rate': '200.0',
            'window_res_x': '960',
            'window_res_y': '1056',
            'rendering_quality': 'high',
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
        arguments=['-d', rviz_config_path]
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
        launch_include,
        ardusim_patch,
        depth_bridge,
        slam_pose_bridge,
        dvl_bridge,
        adaptive_fusion,
        ekf_node,
        rviz_node,
        static_tf,
        ned_to_enu_tf,
        world_to_odom_tf,
    ])
