# test_slam.launch.py
import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pkg        = get_package_share_directory("swarm_simulator")
gazebo_pkg = get_package_share_directory("ros_gz_sim")
bringup_pkg = get_package_share_directory("swarm_bringup")

URDF_PATH  = os.path.join(pkg, "model",  "swarm_robot.urdf")
WORLD_PATH = os.path.join(pkg, "worlds", "swarm_world.sdf")

def generate_launch_description():
    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {WORLD_PATH}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # 2. robot_state_publisher
    with open(URDF_PATH, "r") as f:
        robot_desc = f.read()
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot_0",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
        remappings=[('/tf', '/robot_0/tf'), ('/tf_static', '/robot_0/tf_static')],
    )

    # 3. Spawn
    spawn = TimerAction(period=5.0, actions=[Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot_0",
        arguments=["-name", "robot_0", "-file", URDF_PATH,
                   "-x", "2.0", "-y", "2.0", "-z", "0.15"],
    )])

    # 4. Bridge
    bridge = TimerAction(period=8.0, actions=[Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_robot_0",
        arguments=[
            "/model/robot_0/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/robot_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/world/swarm_world/model/robot_0/link/base_footprint/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/model/robot_0/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/world/swarm_world/model/robot_0/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        ros_arguments=[
            "--remap", "/model/robot_0/odometry:=/robot_0/odom",
            "--remap", "/model/robot_0/tf:=/robot_0/tf",
            "--remap", "/world/swarm_world/model/robot_0/link/base_footprint/sensor/lidar/scan:=/robot_0/scan",
            "--remap", "/world/swarm_world/model/robot_0/joint_state:=/robot_0/joint_states",
        ],
    )])

    # 5. SLAM only — không có Nav2
    slam = TimerAction(period=10.0, actions=[Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="robot_0",
        parameters=[
            os.path.join(bringup_pkg, "config", "nav2_params.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[
            ('/tf',        '/robot_0/tf'),
            ('/tf_static', '/robot_0/tf_static'),
            ('scan',       'scan'),
        ],
        output="screen",
    )])

    return LaunchDescription([gazebo, rsp, spawn, bridge, slam])