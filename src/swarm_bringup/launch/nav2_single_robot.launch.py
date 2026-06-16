"""
nav2_single_robot.launch.py

Launch Nav2 stack + SLAM toolbox for 1 robot.
Called 4 times from nav2_swarm.launch.py with different namespace.

Args:
    robot_name: namespace of robot (ex: robot_1)
    x_pose:     position spawn x (use only for initial pose AMCL if needed)
    y_pose:     position spawn y
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Arguments ---
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="robot_1",
        description="Namespace of the robot"
    )
    x_pose_arg = DeclareLaunchArgument("x_pose", default_value="0.0")
    y_pose_arg = DeclareLaunchArgument("y_pose", default_value="0.0")
    
    robot_name  = LaunchConfiguration("robot_name")
    x_pose      = LaunchConfiguration("x_pose")
    y_pose      = LaunchConfiguration("y_pose")

    # --- Path ---
    bringup_dir = get_package_share_directory("swarm_bringup")
    nav2_dir    = get_package_share_directory("nav2_bringup")
    params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")

    # --- Group all node with namespace robot_name
    nav2_group = GroupAction([
        PushRosNamespace(robot_name),

        # --- SLAM toolbox (AMCL because no map) ---
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[
                params_file,
                {"use_sim_time": True},
            ],
            remappings=[
                ('/tf',         'tf'),
                ('/tf_static',  'tf_static'),
                ('scan',        'scan'),
                ('map',         'map')
,            ],
        ),

        # --- BT(behavior tree) Navigator ---
        # Coordinate navigation behavior: receive a NavigateToPose goal and execute the BT XML.
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=[params_file, {"use_sim_time": True}],
            remappings=[
                ('/tf',         'tf'),
                ('/tf_static',  'tf_static'),
            ],
        ),

        # --- Planner Server ---
        # Calculate global map from current position to goal (NavFn / Smac).
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[params_file, {"use_sime_time": True}],
            remappings=[
                ("/tf",         "tf"),
                ("/tf_static",  "tf_static"),
                ("scan",        "scan"),
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
                ("/tf",         "tf"),
                ("/tf_static",  "tf_static"),
                ("scan",        "scan"),
                ("cmd_vel",     "cmd_vel"),
                ("odom",        "odom"),
            ],
        ),

        # --- Behaviors Server ---
        # Handles robot recovery when stuck: spin, back_up, wait.
        # Humble: nav2_recoveries/recoveries_server → nav2_behaviors/behavior_server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[params_file, {'use_sim_time': True}],
            remappings=[
                ('/tf',        'tf'),
                ('/tf_static', 'tf_static'),
            ],
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
        # NOTE: slam_toolbox (async_slam_toolbox_node) is NOT a lifecycle node
        # No lifecycle_manager_slam needed — the node starts automatically
    ])

    return LaunchDescription([
        robot_name_arg,
        x_pose_arg,
        y_pose_arg,
        nav2_group,
    ])
