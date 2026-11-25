#!/usr/bin/env python3
"""Headless TurtleBot3 Burger Empty World launch (gzserver only, no IMU/LiDAR)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")

    default_world = os.path.join(sim_dir, "worlds", "empty_world.world")
    default_params = os.path.join(sim_dir, "config", "gazebo_params.yaml")
    model_sdf = os.path.join(
        sim_dir, "models", "turtlebot3_burger_odom", "model.sdf"
    )
    urdf_path = os.path.join(pkg_tb3_gazebo, "urdf", "turtlebot3_burger.urdf")

    with open(urdf_path, "r", encoding="utf-8") as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw = LaunchConfiguration("yaw")
    world = LaunchConfiguration("world")
    params_file = LaunchConfiguration("params_file")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "params_file": params_file,
            "server_required": "true",
        }.items(),
    )

    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_desc,
            }
        ],
    )

    spawn_turtlebot_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "turtlebot3_burger",
            "-file",
            model_sdf,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
            "-Y",
            yaw,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock",
            ),
            DeclareLaunchArgument(
                "x_pose", default_value="0.0", description="Initial X pose (m)"
            ),
            DeclareLaunchArgument(
                "y_pose", default_value="0.0", description="Initial Y pose (m)"
            ),
            DeclareLaunchArgument(
                "z_pose", default_value="0.01", description="Initial Z pose (m)"
            ),
            DeclareLaunchArgument(
                "yaw", default_value="0.0", description="Initial yaw (rad)"
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo world file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Gazebo ROS parameters YAML file",
            ),
            gzserver_cmd,
            robot_state_publisher_cmd,
            spawn_turtlebot_cmd,
        ]
    )
