import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'stonefish_simple_sim'
    pkg_share = get_package_share_directory(pkg_name)

    # ─────────────────────────────────────────────
    # PATHS
    # ─────────────────────────────────────────────
    data_dir_path     = os.path.join(pkg_share, 'data')
    scenario_file_path = os.path.join(pkg_share, 'config', 'test.scn')

    # ─────────────────────────────────────────────
    # ARGUMENTS
    # ─────────────────────────────────────────────
    simulation_data_arg = DeclareLaunchArgument(
        'simulation_data',
        default_value='/home/yadunandan/ros2_ws/src/stonefish_simple_sim/data',
        description='Path to the simulation data directory'
    )
    scenario_desc_arg = DeclareLaunchArgument(
        'scenario_desc',
        default_value='/home/yadunandan/ros2_ws/src/stonefish_simple_sim/config/test.scn',
        description='Path to the scenario description file'
    )
    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate', default_value='30.0', description='Simulation rate'
    )
    window_res_x_arg = DeclareLaunchArgument(
        'window_res_x', default_value='1200', description='Window Width'
    )
    window_res_y_arg = DeclareLaunchArgument(
        'window_res_y', default_value='800', description='Window Height'
    )
    rendering_quality_arg = DeclareLaunchArgument(
        'rendering_quality', default_value='high', description='Quality'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='sub', description='Robot name'
    )
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='tcp://127.0.0.1:5760',
        description='ArduSub SITL connection URL'
    )

    # ─────────────────────────────────────────────
    # NODE 1: Stonefish Simulator
    # ─────────────────────────────────────────────
    stonefish_node = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        output='screen',
        arguments=[
            LaunchConfiguration('simulation_data'),
            LaunchConfiguration('scenario_desc'),
            LaunchConfiguration('simulation_rate'),
            LaunchConfiguration('window_res_x'),
            LaunchConfiguration('window_res_y'),
            LaunchConfiguration('rendering_quality'),
        ]
    )

    # ─────────────────────────────────────────────
    # NODE 2: MAVROS (bridges ArduSub MAVLink → ROS2)
    # ─────────────────────────────────────────────
#    mavros_node = Node(
 #       package='mavros',
  #      executable='mavros_node',
   #     name='mavros',
    #    output='screen',
     #   parameters=[{
      #      'fcu_url': LaunchConfiguration('fcu_url'),
       #     'gcs_url': 'udp://@127.0.0.1:14550',
        #    'target_system_id': 1,
#            'target_component_id': 1,
 #       }]
  #  )

    # ─────────────────────────────────────────────
    # NODE 3: ArduSub → Stonefish Bridge
    # ─────────────────────────────────────────────
    bridge_node = Node(
        package='stonefish_simple_sim',
        executable='ardusub_bridge.py',
        name='ardusub_stonefish_bridge',
        output='screen',
    )

    # ─────────────────────────────────────────────
    # LAUNCH DESCRIPTION
    # ─────────────────────────────────────────────
    return LaunchDescription([
        # Arguments
        simulation_data_arg,
        scenario_desc_arg,
        simulation_rate_arg,
        window_res_x_arg,
        window_res_y_arg,
        rendering_quality_arg,
        robot_name_arg,
        fcu_url_arg,
        # Nodes
        stonefish_node,
       # mavros_node,
        bridge_node,
    ])
