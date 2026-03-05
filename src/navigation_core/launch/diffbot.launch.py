# dmke_diffbot.launch.py
# Launches ros2_control with diff_drive_controller using the custom
# DMKE CAN hardware interface.
#
# WHY controller_manager node + spawner pattern:
#   This is the standard ros2_control launch pattern.
#   The controller_manager loads the hardware plugin from the URDF,
#   then the spawner activates the diff_drive_controller.
#
# PREREQUISITE: CAN interface must be up before launching:
#   sudo ip link set can0 up type can bitrate 1000000
#
# Usage:
#   ros2 launch navigation_core dmke_diffbot.launch.py

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('navigation_core')

    # Build robot_description from xacro
    # WHY Command + xacro: standard ROS 2 pattern for parameterized URDF.
    # Your main xacro should xacro:include the dmke_ros2_control.urdf.xacro block.
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    ])

    # Replace the current robot_description dict with:
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    # Controller config
    # WHY separate YAML for controller params: diff_drive_controller has ~20
    # tunable parameters (wheel separation, radius, limits, odom frame, etc).
    # These change per robot, so they belong in a config file.
    controller_config = os.path.join(pkg_share, 'config', 'diff_controller.yaml')

    # ros2_control_node: loads hardware plugin, runs control loop
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    # Robot state publisher: publishes TF from URDF joint states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    # Spawn diff_drive_controller after controller_manager is up
    # WHY TimerAction with 3s delay: controller_manager needs time to load
    # the hardware plugin and complete on_init + on_activate (CiA 402 enable
    # sequence takes ~500ms). A proper approach would use a lifecycle event,
    # but a timer is simpler and sufficient.
    diff_drive_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diffbot_base_controller',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            ),
        ],
    )

    # Spawn joint_state_broadcaster (publishes /joint_states for robot_state_publisher)
    joint_state_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher,
        diff_drive_spawner,
        joint_state_spawner,
    ])