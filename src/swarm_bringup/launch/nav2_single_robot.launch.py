"""
nav2_single_robot.launch.py

Launch Nav2 stack + SLAM toolbox for 1 robot.
Called 4 times from nav2_swarm.launch.py with different namespace.

All TF frames are prefixed with the robot name (robot_0/map, robot_0/odom,
robot_0/base_footprint, ...) so every robot can publish to the global /tf
without clashing. That lets a single RViz show all robots together.

Args:
    robot_name: namespace of robot (ex: robot_1)
    x_pose:     position spawn x (use only for initial pose AMCL if needed)
    y_pose:     position spawn y
"""

import os
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def make_robot_params(config_name: str, robot_name: str) -> str:
    """
    Generate a per-robot params file by replacing the __NS__ placeholder in the
    shared `config_name` (nav2_params.yaml / slam_params.yaml) with the robot
    name, giving prefixed TF frames (robot_0/map, robot_0/odom, ...).
    """
    bringup_dir = get_package_share_directory("swarm_bringup")
    template = os.path.join(bringup_dir, "config", config_name)

    with open(template, "r") as f:
        content = f.read()
    content = content.replace("__NS__", robot_name)

    out_dir = os.path.join(tempfile.gettempdir(), "swarm_params")
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(config_name)[0]
    out_path = os.path.join(out_dir, f"{robot_name}_{stem}.yaml")
    with open(out_path, "w") as f:
        f.write(content)
    return out_path


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    params_file = make_robot_params("nav2_params.yaml", robot_name)
    slam_params_file = make_robot_params("slam_params.yaml", robot_name)

    # All nodes run under the robot namespace but publish/subscribe TF on the
    # global /tf (frames are already prefixed, so no cross-robot collision).
    nav2_group = GroupAction([
        PushRosNamespace(robot_name),

        # --- SLAM toolbox (mapping mode, no prior map) ---
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[
                slam_params_file,
                {"use_sim_time": True},
            ],
            remappings=[
                ('scan',          'scan'),
                ('/map',          'map'),
                ('/map_metadata', 'map_metadata'),
                ('/map_updates',  'map_updates'),
            ],
        ),

        # --- BT (behavior tree) Navigator ---
        # Coordinate navigation behavior: receive a NavigateToPose goal and execute the BT XML.
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=[params_file, {"use_sim_time": True}],
        ),

        # --- Planner Server ---
        # Calculate global path from current position to goal (NavFn / Smac).
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[params_file, {"use_sim_time": True}],
            remappings=[
                ("scan", "scan"),
            ],
        ),

        # --- Controller Server ---
        # Follows the global path and publishes cmd_vel commands (DWB local planner).
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[params_file, {"use_sim_time": True}],
            remappings=[
                ("scan",    "scan"),
                ("cmd_vel", "cmd_vel_nav"),
                ("odom",    "odom"),
            ],
        ),

        # --- Behaviors Server ---
        # Handles robot recovery when stuck: spin, back_up, wait.
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[params_file, {'use_sim_time': True}],
        ),

        # --- Lifecycle Manager — Nav2 ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': [
                    'bt_navigator',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                ]},
            ],
        ),

    ])

    return [nav2_group]


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="robot_1",
        description="Namespace of the robot"
    )
    x_pose_arg = DeclareLaunchArgument("x_pose", default_value="0.0")
    y_pose_arg = DeclareLaunchArgument("y_pose", default_value="0.0")

    return LaunchDescription([
        robot_name_arg,
        x_pose_arg,
        y_pose_arg,
        OpaqueFunction(function=launch_setup),
    ])
