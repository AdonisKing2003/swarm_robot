import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('swarm_simulator'),
            'launch', 'swarm_simulation.launch.py'
        ))
    )

    # Delay Nav2 to allow Gazebo enough time to finish spawning the robot
    # If Nav2 starts before the robot TF is available -> SLAM may crash
    nav2_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('swarm_bringup'),
                    'launch', 'nav2_swarm.launch.py'
                ))
            )
        ]
    )

    return LaunchDescription([
        sim_launch,
        # nav2_launch,
    ])