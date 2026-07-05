"""
nav2_swarm.launch.py

Entry point: launch Nav2 + SLAM cho car 4 robot.
Call nav2_single_robot.launch.py 4 times with namespace and spawn different pose.

Usage:
    ros2 launch swarm_bringup nav2_swarm.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# Config 4 robots
# Setup x_pose, y_pose match position spawn in simulator
ROBOTS = [
    {"name": "robot_0", "x_pose":  2.0,  "y_pose":  2.0},
    {"name": "robot_1", "x_pose": -2.0,  "y_pose":  2.0},
    {"name": "robot_2", "x_pose":  2.0,  "y_pose": -2.0},
    {"name": "robot_3", "x_pose": -2.0,  "y_pose": -2.0},
]

# Delay between each robot (seconds) to avoid disputes TF
SPAWN_DELAY = 2.0

def generate_launch_description():

    bringup_dir = get_package_share_directory("swarm_bringup")
    single_robot_launch = os.path.join(
        bringup_dir, "launch", "nav2_single_robot.launch.py"
    )

    ld = LaunchDescription()

    for i, robot in enumerate(ROBOTS):
        nav2_instance = TimerAction(
            period=float(i) * SPAWN_DELAY,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(single_robot_launch),
                    launch_arguments={
                        "robot_name":   robot["name"],
                        "x_pose":       str(robot["x_pose"]),
                        "y_pose":       str(robot["y_pose"]),
                    }.items(),
                )
            ],
        )
        ld.add_action(nav2_instance)

    return ld