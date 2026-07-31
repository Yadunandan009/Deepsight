#!/usr/bin/env python3
"""
Launch file for mono ORB-SLAM3 with image enhancement.
This runs inside Docker and processes raw Stonefish images with contrast enhancement
before feeding to mono SLAM.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    orb_wrapper_pkg = "/root/colcon_ws/src/orb_slam3_ros2_wrapper"

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation clock if true",
    )

    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace", default_value="bluerov2_down", description="The namespace"
    )

    def all_nodes_launch(context, robot_namespace):
        vocab_file = "/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt"
        config_file = "/tmp/mono_down.yaml"
        ros_params_file = "/tmp/mono_down_ros_params.yaml"

        ns = robot_namespace.perform(context)

        # Image enhancer node - runs inside Docker, subscribes to Stonefish raw images
        enhancer_node = Node(
            package="image_processing",  # We'll need to create this package
            executable="image_enhancer",
            output="screen",
            namespace=ns,
            parameters=[
                {"input_topic": "/bluerov2/down/image_color"},
                {"output_topic": "/bluerov2/down/enhanced"},
                {"use_sim_time": True},
            ],
        )

        # Mono ORB-SLAM3 node - subscribes to enhanced images
        orb_slam3_node = Node(
            package="orb_slam3_ros2_wrapper",
            executable="mono",
            output="screen",
            namespace=ns,
            arguments=[vocab_file, config_file],
            parameters=[
                {
                    "image_topic_name": "/bluerov2/down/enhanced",
                    "robot_base_frame": "bluerov2/base_link",
                    "use_sim_time": True,
                }
            ],
        )

        return [enhancer_node, orb_slam3_node]

    from launch.actions import OpaqueFunction

    opaque_function = OpaqueFunction(
        function=all_nodes_launch, args=[robot_namespace]
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            robot_namespace_arg,
            opaque_function,
        ]
    )
