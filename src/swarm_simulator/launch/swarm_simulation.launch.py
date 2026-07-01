import os
import math
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ─────────────────────────────────────────────
pkg        = get_package_share_directory("swarm_simulator")
gazebo_pkg = get_package_share_directory("ros_gz_sim")

URDF_PATH  = os.path.join(pkg, "model",  "swarm_robot.urdf")
WORLD_PATH = os.path.join(pkg, "worlds", "swarm_world.sdf")

ROBOTS = [
    ("robot_0",  2.0,  2.0,   0),
    ("robot_1", -2.0,  2.0,   0),
    ("robot_2",  2.0, -2.0, 180),
    ("robot_3", -2.0, -2.0, 180),
]
# ROBOTS = [
#     ("robot_0",  2.0,  2.0,   0),
# ]

# Dir holding the per-robot frame-prefixed URDFs (spawned into Gazebo)
URDF_GEN_DIR = os.path.join(tempfile.gettempdir(), "swarm_urdf")
os.makedirs(URDF_GEN_DIR, exist_ok=True)
# ─────────────────────────────────────────────


def make_namespaced_urdf(name: str) -> str:
    """
    Generate a per-robot URDF for `name`, prefixing the frames that Gazebo
    emits (odom / base_footprint / laser_link / camera_optical_link) to
    `{name}/...` so the 4 robots don't clash on the global /tf.

    Only the Gazebo-published frames are changed. Link/joint names stay the
    same -- the URDF TF chain (base_footprint->base_link->...) is published by
    robot_state_publisher and prefixed there via `frame_prefix`.
    """
    with open(URDF_PATH, "r") as f:
        urdf = f.read()

    replacements = {
        "<frame_id>odom</frame_id>":
            f"<frame_id>{name}/odom</frame_id>",
        "<child_frame_id>base_footprint</child_frame_id>":
            f"<child_frame_id>{name}/base_footprint</child_frame_id>",
        "<gz_frame_id>laser_link</gz_frame_id>":
            f"<gz_frame_id>{name}/laser_link</gz_frame_id>",
        "<gz_frame_id>camera_optical_link</gz_frame_id>":
            f"<gz_frame_id>{name}/camera_optical_link</gz_frame_id>",
    }
    for old, new in replacements.items():
        urdf = urdf.replace(old, new)

    out_path = os.path.join(URDF_GEN_DIR, f"{name}.urdf")
    with open(out_path, "w") as f:
        f.write(urdf)
    return out_path


def make_robot_state_publisher(name: str):
    with open(URDF_PATH, "r") as f:
        robot_desc = f.read()
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=name,
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": True,
            # Prefix every URDF frame to {name}/... so we can publish to the
            # global /tf without clashing between robots.
            "frame_prefix": f"{name}/",
        }],
        # Publishes to the global /tf, /tf_static (frames unique via frame_prefix)
        output="screen",
    )


def make_spawn_action(name: str, x: float, y: float, yaw_deg: float, delay: float):
    yaw_rad = yaw_deg * 3.14159 / 180.0
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{name}",
        arguments=[
            "-name",  name,
            # URDF with prefixed Gazebo frames ({name}/odom, {name}/laser_link, ...)
            "-file",  make_namespaced_urdf(name),
            "-x",     str(x),
            "-y",     str(y),
            "-z",     "0.15",
            "-Y",     str(yaw_rad),
        ],
        output="screen",
    )
    return TimerAction(period=delay, actions=[spawn_node])


def make_bridge(name: str):
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"bridge_{name}",
        arguments=[
            # cmd_vel: ROS 2 -> Gz
            f"/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # odom: Gz -> ROS 2
            f"/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # scan: Gz -> ROS 2
            f"/world/swarm_world/model/{name}/link/base_footprint/sensor/lidar/scan"
            "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # FIX: dùng /{name}/tf thay vì /tf global
            # Trước: "/tf@..." → Gazebo publish odom→base_footprint lên /tf global
            #         SLAM dưới /robot_0/ listen /robot_0/tf → không nhận được
            # Sau: "/{name}/tf@..." → Gazebo publish lên /robot_0/tf
            #       SLAM dưới /robot_0/ listen /robot_0/tf → nhận đúng
            f"/model/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # f"/world/swarm_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # joint_state
            f"/world/swarm_world/model/{name}/joint_state" 
            "@sensor_msgs/msg/JointState[gz.msgs.Model",
            # clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        # Dùng ros_arguments để remap đúng cách
        ros_arguments=[
            "--remap", f"/model/{name}/cmd_vel:=/{name}/cmd_vel",
            "--remap", f"/model/{name}/odometry:=/{name}/odom",
            # odom->base_footprint TF to the global /tf (frames prefixed {name}/... in URDF)
            "--remap", f"/model/{name}/tf:=/tf",
            "--remap", f"/world/swarm_world/model/{name}/joint_state:=/{name}/joint_states",
            "--remap", f"/world/swarm_world/model/{name}/link/base_footprint/sensor/lidar/scan:=/{name}/scan",
        ],
        output="screen",
    )

def make_rviz():
    """Single RViz for the whole swarm, reads the global /tf (frames are prefixed per robot)."""
    rviz_config = os.path.join(
        get_package_share_directory('swarm_simulator'),
        'rviz',
        'swarm.rviz'
    )
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )


def make_world_to_map_tf(name: str, x: float, y: float, yaw_deg: float):
    """
    Static TF: world -> {name}/map, placed at the robot's spawn pose.

    Each robot runs its own SLAM, so each has a private map frame ({name}/map).
    A map's origin is roughly the robot's spawn position in the world, so we
    anchor world->{name}/map at the exact spawn pose. This makes all robots
    show up at the correct relative positions inside a single RViz.
    """
    yaw_rad = yaw_deg * math.pi / 180.0
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'world_to_{name}_map',
        arguments=[
            '--x', str(x), '--y', str(y), '--z', '0.0',
            '--yaw', str(yaw_rad), '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'world', '--child-frame-id', f'{name}/map',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

def generate_launch_description():
    actions = []

    actions.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation clock",
        )
    )

    # 1. Khởi động Gz Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {WORLD_PATH}",
            "on_exit_shutdown": "true",
        }.items(),
    )
    actions.append(gazebo)

    # 2. robot_state_publisher cho từng robot
    for name, x, y, yaw in ROBOTS:
        actions.append(make_robot_state_publisher(name))

    # 3. Spawn robot (delay tăng dần)
    for i, (name, x, y, yaw) in enumerate(ROBOTS):
        delay = 5.0 + i * 1.5
        actions.append(make_spawn_action(name, x, y, yaw, delay))

    # 4. Bridge ROS 2 <-> Gz cho từng robot (delay sau spawn)
    for i, (name, x, y, yaw) in enumerate(ROBOTS):
        delay = 8.0 + i * 1.5   # sau spawn ~3s
        actions.append(
            TimerAction(period=delay, actions=[make_bridge(name)])
        )
    
    # 5. Static TF world -> {name}/map so all robots share one "world" frame
    for name, x, y, yaw in ROBOTS:
        actions.append(make_world_to_map_tf(name, x, y, yaw))

    # 6. One RViz for the whole swarm (delay after bridge)
    actions.append(
        TimerAction(period=10.0, actions=[make_rviz()])
    )

    return LaunchDescription(actions)