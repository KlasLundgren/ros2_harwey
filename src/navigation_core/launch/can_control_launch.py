import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("navigation_core")

    # --- Paths ---------------------------------------------------------------
    bus_config = PathJoinSubstitution(
        [pkg_share, "config", "bot_bus", "bus.yml"]
    )
    master_dcf = PathJoinSubstitution(
        [pkg_share, "config", "bot_bus", "master.dcf"]
    )
    controllers_config = PathJoinSubstitution(
        [pkg_share, "config", "my_controllers.yaml"]
    )

    # --- Arguments -----------------------------------------------------------
    can_interface_arg = DeclareLaunchArgument(
        "can_interface", default_value="can0",
        description="SocketCAN interface name",
    )

    # --- Bring up CAN interface ----------------------------------------------
    # setup_can = ExecuteProcess(
    #     cmd=["bash", "-c",
    #          "sudo ip link set can0 up type can bitrate 1000000 txqueuelen 1000 || true"],
    #     name="setup_can",
    # )

    # --- Robot description (URDF from xacro) ---------------------------------
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([pkg_share, "description", "robot.urdf.xacro"]),
        " bus_config:=", bus_config,
        " master_config:=", master_dcf,
        " can_interface:=", LaunchConfiguration("can_interface"),
    ])
    robot_description = {"robot_description": robot_description_content}

    # --- Controller manager --------------------------------------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config],
        output="screen",
    )

    # --- Robot state publisher (TF from URDF) --------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # --- Spawn controllers (delayed to let hardware init) --------------------

    # 1. Robot controller: runs CiA 402 state machine (init + enable drives)
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # 2. Joint state broadcaster: publishes /joint_states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    # 3. Diff drive controller: /cmd_vel → wheels, encoder → /odom
    diffbot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        can_interface_arg,
        # setup_can,
        control_node,
        robot_state_publisher,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        diffbot_controller_spawner,
    ])