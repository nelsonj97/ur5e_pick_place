#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder

from scipy.spatial.transform import Rotation as R
import numpy as np


def generate_launch_description():
    pkg_golf_share = get_package_share_directory("ur5e_golf_pick_place")

    # Make env var available immediately for xacro $(env ...) substitutions
    gz2_prefix = get_package_prefix("gazebo_ros2_control")
    gz2_lib = os.path.join(gz2_prefix, "lib")
    gz2_plugin_abs = os.path.join(gz2_lib, "libgazebo_ros2_control.so")
    os.environ["GZ_ROS2_CONTROL_LIB"] = gz2_plugin_abs
    
    # Get linkattacher plugin path
    linkattacher_prefix = get_package_prefix("ros2_linkattacher")
    linkattacher_lib = os.path.join(linkattacher_prefix, "lib")

    # ------------------------
    # Args
    # ------------------------
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")  # ✅ No rotation

    default_world = os.path.join(pkg_golf_share, "worlds", "golf_workspace.world")
    world_arg = DeclareLaunchArgument("world", default_value=default_world)

    # ------------------------
    # Env for spawned processes
    # ------------------------
    forced_gazebo_plugin_path = (
        gz2_lib
        + ":" + linkattacher_lib
        + ":/opt/ros/humble/lib"
        + ":/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
    )
    forced_ld_library_path = gz2_lib + ":" + linkattacher_lib + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    
    set_gz_ros2_control_lib = SetEnvironmentVariable("GZ_ROS2_CONTROL_LIB", gz2_plugin_abs)
    set_gazebo_plugin_path = SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", forced_gazebo_plugin_path)
    set_ld_library_path = SetEnvironmentVariable("LD_LIBRARY_PATH", forced_ld_library_path)
    set_gazebo_model_path = SetEnvironmentVariable("GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models")
    set_gazebo_model_db = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    # ------------------------
    # robot_description for simulation: use the Gazebo xacro
    # ------------------------
    robot_xacro_path = os.path.join(pkg_golf_share, "urdf", "ur5e_robotiq85.urdf.xacro")
    robot_description_xml = Command(["xacro ", robot_xacro_path])
    robot_description = {"robot_description": ParameterValue(robot_description_xml, value_type=str)}
    use_sim_time = {"use_sim_time": True}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, use_sim_time],
    )
    
    # ═══════════════════════════════════════════════════════════════════════
    # ✅ CAMERA TF PUBLISHERS (CORRECTED)
    # ═══════════════════════════════════════════════════════════════════════
    
    # Overhead camera position (above table, looking down)
    world_cam_x = 1.2
    world_cam_y = 0.0
    world_cam_z = 1.8
    world_cam_roll = 0.0
    world_cam_pitch = 0.785398  # 45° down
    world_cam_yaw = 3.14159     # 180° (looking back toward robot)

    # Convert Euler (roll, pitch, yaw) to quaternion
    cam_rotation = R.from_euler('xyz', [world_cam_roll, world_cam_pitch, world_cam_yaw])
    cam_quat = cam_rotation.as_quat()  # Returns [x, y, z, w]

    print(f"📷 Camera position: ({world_cam_x}, {world_cam_y}, {world_cam_z})")
    print(f"📷 Camera quaternion (xyzw): [{cam_quat[0]:.6f}, {cam_quat[1]:.6f}, {cam_quat[2]:.6f}, {cam_quat[3]:.6f}]")
    
    # TF 1: world -> camera_link
    overhead_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='overhead_camera_tf_publisher',
        output='screen',
        arguments=[
            str(world_cam_x), str(world_cam_y), str(world_cam_z),  # Translation
            str(cam_quat[0]), str(cam_quat[1]), str(cam_quat[2]), str(cam_quat[3]),  # Rotation
            'world',
            'camera_link'
        ],
        parameters=[use_sim_time],
    )
    
    # TF 2: camera_link -> camera_link_optical (ROS2 optical convention)
    # This transform converts from ROS camera frame to optical frame
    # Optical frame: X=right, Y=down, Z=forward (into scene)
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_frame_publisher',
        output='screen',
        arguments=[
            '0', '0', '0',              # No translation
            '-0.5', '0.5', '-0.5', '0.5',  # Quaternion for optical frame rotation
            'camera_link',
            'camera_link_optical'
        ],
        parameters=[use_sim_time],
    )
    
    # ❌ REMOVED: robot_base_tf - This conflicts with robot_state_publisher!
    # The robot_state_publisher already publishes world -> base_link based on spawn position
    
    # TF 3: Wrist camera optical frame (for wrist-mounted camera if you have one)
    wrist_camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wrist_camera_optical_tf',
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',  # Standard optical frame rotation
            'wrist_camera_link',
            'wrist_camera_optical_link'
        ],
        parameters=[use_sim_time],
        output='screen'
    )
    
    print("✅ Camera TF publishers configured")
    # ═══════════════════════════════════════════════════════════════════════

    # ------------------------
    # MoveIt
    # ------------------------
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur5e_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    moveit_params = moveit_config.to_dict()
    moveit_params.update(use_sim_time)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("ur5e_gripper_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            use_sim_time,
        ],
    )

    # ------------------------
    # Gazebo server/client
    # ------------------------
    ros_lib = "/opt/ros/humble/lib"
    gz_init = os.path.join(ros_lib, "libgazebo_ros_init.so")
    gz_factory = os.path.join(ros_lib, "libgazebo_ros_factory.so")
    gz_force = os.path.join(ros_lib, "libgazebo_ros_force_system.so")

    # Start paused so the arm doesn't collide before controllers are up
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-u",
            LaunchConfiguration("world"),
            "-s", gz_init,
            "-s", gz_factory,
            "-s", gz_force,
        ],
        output="screen",
        additional_env={
            "GZ_ROS2_CONTROL_LIB": gz2_plugin_abs,
            "GAZEBO_PLUGIN_PATH": forced_gazebo_plugin_path,
            "LD_LIBRARY_PATH": forced_ld_library_path,
            "GAZEBO_MODEL_PATH": "/usr/share/gazebo-11/models",
            "GAZEBO_MODEL_DATABASE_URI": "",
        },
    )

    # Start GUI only AFTER HOME is reached
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        additional_env={
            "GZ_ROS2_CONTROL_LIB": gz2_plugin_abs,
            "GAZEBO_PLUGIN_PATH": forced_gazebo_plugin_path,
            "LD_LIBRARY_PATH": forced_ld_library_path,
        },
    )

    # ------------------------
    # Spawn robot
    # ------------------------
    spawn_the_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "cobot",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    # ------------------------
    # Unpause physics
    # ------------------------
    unpause_physics = ExecuteProcess(
        cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
        output="screen",
    )

    # ------------------------
    # Controllers
    # ------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "60",
        ],
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "60",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "robotiq_gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "60",
        ],
    )

    # ------------------------
    # Send robot to HOME pose
    # ------------------------
    go_home_trajectory = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "ros2 action send_goal "
            "/joint_trajectory_controller/follow_joint_trajectory "
            "control_msgs/action/FollowJointTrajectory "
            "\"{trajectory: {"
            "joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'], "
            "points: [{positions: [0.0, -1.57, 0.0, -0.6, -1.7, -0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}]"
            "}}\""
        ],
        output="screen",
    )

    # ------------------------
    # Event chain
    # ------------------------
    after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_the_robot,
            on_exit=[unpause_physics],
        )
    )

    after_unpause = RegisterEventHandler(
        OnProcessExit(
            target_action=unpause_physics,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_trajectory_controller_spawner],
        )
    )

    after_arm_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_trajectory_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    after_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[
                TimerAction(period=0.5, actions=[go_home_trajectory]),
            ],
        )
    )

    after_go_home = RegisterEventHandler(
        OnProcessExit(
            target_action=go_home_trajectory,
            on_exit=[gzclient],
        )
    )

    # ═══════════════════════════════════════════════════════════════════════
    # LAUNCH DESCRIPTION - PROPER ORDER
    # ═══════════════════════════════════════════════════════════════════════
    ld = LaunchDescription()

    # Phase 1: Arguments
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(yaw_arg)
    ld.add_action(world_arg)

    # Phase 2: Environment variables
    ld.add_action(set_gz_ros2_control_lib)
    ld.add_action(set_gazebo_plugin_path)
    ld.add_action(set_ld_library_path)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(set_gazebo_model_db)

    # Phase 3: Core nodes (start immediately)
    print("🚀 Starting Gazebo server...")
    ld.add_action(gzserver)
    
    print("🤖 Starting robot state publisher...")
    ld.add_action(robot_state_publisher)
    
    # Phase 4: TF Publishers (start early, before spawn)
    print("📷 Starting camera TF publishers...")
    ld.add_action(overhead_camera_tf)
    ld.add_action(camera_optical_tf)
    ld.add_action(wrist_camera_optical_tf)
    
    # Phase 5: Robot spawn and initialization
    print("🏗️ Spawning robot...")
    ld.add_action(spawn_the_robot)
    
    # Phase 6: Event chain
    ld.add_action(after_spawn)
    ld.add_action(after_unpause)
    ld.add_action(after_jsb)
    ld.add_action(after_arm_ctrl)
    ld.add_action(after_gripper)
    ld.add_action(after_go_home)
    
    # ❌ REMOVED: robot_base_tf - Conflicts with robot_state_publisher
    # ld.add_action(robot_base_tf)  # DELETE THIS LINE

    # Phase 7: Planning and visualization
    print("🧠 Starting MoveIt...")
    ld.add_action(move_group_node)
    
    print("👁️ Starting RViz (delayed)...")
    ld.add_action(TimerAction(period=2.0, actions=[rviz_node]))

    print("✅ Launch file configured successfully!")
    return ld
