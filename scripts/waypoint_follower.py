#!/usr/bin/env python3
"""
Load waypoints from YAML and send them to Nav2 via FollowWaypoints action.

Parameters:
  - waypoint_file: path to YAML file. Default './recorded_waypoints.yaml'

Usage:
  ros2 run navigation_core waypoint_follower --ros-args -p waypoint_file:=./recorded_waypoints.yaml

Requires Nav2 stack to be fully active (AMCL localized, Nav2 lifecycle up).
"""

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml


def load_waypoints(filepath: str) -> list[dict]:
    """Load waypoint list from YAML. Raises on missing file or bad format."""
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or 'waypoints' not in data:
        raise ValueError(f'YAML must contain a top-level "waypoints" key. Got: {list(data.keys())}')

    waypoints = data['waypoints']
    if not waypoints:
        raise ValueError('Waypoint list is empty.')

    # Validate required fields exist in each waypoint
    required_fields = {'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'}
    for i, wp in enumerate(waypoints):
        missing = required_fields - set(wp.keys())
        if missing:
            raise ValueError(f'Waypoint {i} missing fields: {missing}')

    return waypoints


def waypoints_to_poses(waypoints: list[dict], navigator: BasicNavigator) -> list[PoseStamped]:
    """Convert YAML waypoint dicts to PoseStamped messages."""
    poses = []
    for wp in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        pose.pose.position.z = float(wp['z'])
        pose.pose.orientation.x = float(wp['qx'])
        pose.pose.orientation.y = float(wp['qy'])
        pose.pose.orientation.z = float(wp['qz'])
        pose.pose.orientation.w = float(wp['qw'])
        poses.append(pose)
    return poses


def main() -> None:
    rclpy.init()

    navigator = BasicNavigator()

    # Declare and read parameter
    navigator.declare_parameter('waypoint_file', './recorded_waypoints.yaml')
    waypoint_file: str = navigator.get_parameter('waypoint_file').value

    # --- Load waypoints ---
    try:
        waypoints = load_waypoints(waypoint_file)
    except (OSError, ValueError) as e:
        navigator.get_logger().error(f'Failed to load waypoints: {e}')
        navigator.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    navigator.get_logger().info(f'Loaded {len(waypoints)} waypoints from {waypoint_file}')

    # --- Wait for Nav2 ---
    # WHY waitUntilNav2Active: ensures AMCL is localized and Nav2 lifecycle
    # nodes are in active state before sending goals. Without this,
    # followWaypoints will silently fail or get rejected.
    navigator.waitUntilNav2Active()

    # --- Build and send poses ---
    poses = waypoints_to_poses(waypoints, navigator)
    navigator.get_logger().info(f'Sending {len(poses)} waypoints to Nav2...')
    navigator.goThroughPoses(poses)

    # --- Monitor progress ---
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            current = feedback.current_waypoint
            navigator.get_logger().info(
                f'Executing waypoint {current + 1}/{len(poses)}',
                throttle_duration_sec=2.0,
            )

    # --- Result ---
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('All waypoints completed successfully.')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('Waypoint following was canceled.')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('Waypoint following failed.')
    else:
        navigator.get_logger().error(f'Unexpected result: {result}')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
