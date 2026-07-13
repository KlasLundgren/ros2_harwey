#!/usr/bin/env python3
"""
Load waypoints from YAML and track them as ONE continuous path via Nav2 FollowPath.

WHY followPath instead of goThroughPoses/followWaypoints:
- goThroughPoses treats intermediate poses as planner hints; on a closed loop
  the robot is already "at" the final goal and never moves.
- followWaypoints decelerates and stops at every waypoint (per-waypoint goal
  checking in the waypoint action server).
- followPath sends a nav_msgs/Path straight to the controller server, which
  tracks it as a single trajectory with no intermediate goal checks.

Parameters:
  - waypoint_file: path to YAML file, or comma-separated list of files to
    follow in sequence (the mission continues to the next path only after
    the previous one SUCCEEDED). Default './recorded_waypoints.yaml'
  - interpolation_spacing: max distance (m) between consecutive path poses.
    Recorded waypoints (~1 m apart) are densified to this spacing because
    MPPI's path critics assume planner-density paths (their offset/occupancy
    params count path POINTS, not meters). Set to 0.0 to disable. Default 0.1
  - backup_distance: if > 0, reverse STRAIGHT this many meters (Nav2 BackUp
    behavior) before following the first path. Default 0.0 (disabled)
  - backup_speed: reverse speed in m/s for the backup stage. Default 0.2

WHY BackUp instead of a recorded reverse path: any path tracker (MPPI
included) steers to correct lateral error, and localization jitter on the
sand surface guarantees there is always some error to correct. Steering
while reversing twists the trailer. The BackUp behavior commands zero
angular velocity and measures distance by odometry only — localization
noise cannot cause steering corrections.

Usage:
  python3 scripts/waypoint_follower.py --ros-args -p waypoint_file:=./recorded_waypoints.yaml
  # Mission: reverse straight 10 m out of the station, then cover the field:
  python3 scripts/waypoint_follower.py --ros-args \
      -p backup_distance:=10.0 -p waypoint_file:=./field.yaml

Requires Nav2 stack to be fully active (AMCL localized, Nav2 lifecycle up).
Ctrl+C cancels the FollowPath action so the robot stops.
"""

import math
import signal
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml

# WHY module-level flag + custom SIGINT handler: rclpy's default handler tears
# down the context on Ctrl+C, which kills the action client BEFORE we can send
# a cancel — leaving the robot driving the path with nobody watching. We keep
# the context alive, cancel the task cleanly, then exit.
_cancel_requested = False
_task_started = False


def _sigint_handler(sig, frame) -> None:
    global _cancel_requested
    if not _task_started:
        # No goal in flight yet (still loading / waiting for Nav2): plain exit.
        sys.exit(0)
    _cancel_requested = True


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


def _yaw(wp: dict) -> float:
    """Extract yaw from a 2D-relevant quaternion (qx=qy~0)."""
    return 2.0 * math.atan2(wp['qz'], wp['qw'])


def _make_pose(x: float, y: float, yaw: float, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def waypoints_to_path(waypoints: list[dict], spacing: float, navigator: BasicNavigator) -> Path:
    """Build a nav_msgs/Path, densifying to `spacing` meters between poses.

    Positions are interpolated linearly, yaw along the shortest arc.
    WHY yaw interpolation matters: with use_path_orientations enabled, MPPI
    aligns the robot heading to path orientations — stair-stepped headings
    from sparse waypoints would produce jerky tracking through turns.
    """
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = navigator.get_clock().now().to_msg()
    stamp = path.header.stamp

    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        yaw_a, yaw_b = _yaw(a), _yaw(b)
        path.poses.append(_make_pose(float(a['x']), float(a['y']), yaw_a, stamp))

        if spacing <= 0.0:
            continue
        dist = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        n = int(dist / spacing)
        dyaw = (yaw_b - yaw_a + math.pi) % (2.0 * math.pi) - math.pi
        for k in range(1, n):
            t = k / n
            path.poses.append(_make_pose(
                float(a['x']) + t * (float(b['x']) - float(a['x'])),
                float(a['y']) + t * (float(b['y']) - float(a['y'])),
                yaw_a + t * dyaw,
                stamp,
            ))

    last = waypoints[-1]
    path.poses.append(_make_pose(float(last['x']), float(last['y']), _yaw(last), stamp))
    return path


def follow_one(navigator: BasicNavigator, path: Path, label: str) -> TaskResult:
    """Send one path via FollowPath and block until it finishes or is canceled."""
    global _task_started
    _task_started = True
    navigator.followPath(path)

    canceled = False
    try:
        while not navigator.isTaskComplete():
            if _cancel_requested and not canceled:
                navigator.get_logger().warn('Ctrl+C received — canceling path following...')
                navigator.cancelTask()
                canceled = True
            feedback = navigator.getFeedback()
            if feedback:
                navigator.get_logger().info(
                    f'[{label}] Distance to path end: {feedback.distance_to_goal:.2f} m, '
                    f'speed: {feedback.speed:.2f} m/s',
                    throttle_duration_sec=2.0,
                )
    finally:
        # Belt-and-suspenders: any other exit path (exception, shutdown) also
        # stops the robot instead of leaving the controller tracking the path.
        if not canceled and not navigator.isTaskComplete():
            navigator.cancelTask()

    return navigator.getResult()


def backup_straight(navigator: BasicNavigator, distance: float, speed: float) -> TaskResult:
    """Reverse straight `distance` meters via the Nav2 BackUp behavior.

    BackUp commands pure -x velocity (zero angular) and tracks distance from
    odometry, so it cannot steer — safe with the trailer attached.
    """
    global _task_started
    # WHY generous time_allowance: 3x nominal duration + margin covers the
    # velocity smoother's accel ramp; too tight and the behavior aborts mid-reverse.
    time_allowance = int(3 * distance / speed) + 10
    navigator.get_logger().info(f'[backup] Reversing straight {distance:.1f} m at {speed:.2f} m/s...')
    _task_started = True
    navigator.backup(backup_dist=distance, backup_speed=speed, time_allowance=time_allowance)

    canceled = False
    try:
        while not navigator.isTaskComplete():
            if _cancel_requested and not canceled:
                navigator.get_logger().warn('Ctrl+C received — canceling backup...')
                navigator.cancelTask()
                canceled = True
            feedback = navigator.getFeedback()
            if feedback:
                navigator.get_logger().info(
                    f'[backup] Reversed {feedback.distance_traveled:.2f}/{distance:.1f} m',
                    throttle_duration_sec=2.0,
                )
    finally:
        if not canceled and not navigator.isTaskComplete():
            navigator.cancelTask()

    return navigator.getResult()


def main() -> None:
    # Disable rclpy's own SIGINT handling so ours runs (see note at top).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _sigint_handler)

    navigator = BasicNavigator()

    # Declare and read parameters
    navigator.declare_parameter('waypoint_file', './recorded_waypoints.yaml')
    navigator.declare_parameter('interpolation_spacing', 0.1)
    navigator.declare_parameter('backup_distance', 0.0)
    navigator.declare_parameter('backup_speed', 0.2)
    waypoint_file: str = navigator.get_parameter('waypoint_file').value
    spacing: float = navigator.get_parameter('interpolation_spacing').value
    backup_distance: float = navigator.get_parameter('backup_distance').value
    backup_speed: float = navigator.get_parameter('backup_speed').value

    files = [f.strip() for f in waypoint_file.split(',') if f.strip()]

    # --- Load all waypoint files up front (fail before the robot moves) ---
    missions: list[tuple[str, list[dict]]] = []
    for f in files:
        try:
            missions.append((f, load_waypoints(f)))
        except (OSError, ValueError) as e:
            navigator.get_logger().error(f'Failed to load waypoints from {f}: {e}')
            navigator.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    for f, waypoints in missions:
        navigator.get_logger().info(f'Loaded {len(waypoints)} waypoints from {f}')

        # WHY this check: the controller server tests "goal reached" against the
        # path's LAST pose from the very first cycle. On a closed loop that ends
        # where the robot starts (charging station), FollowPath can succeed
        # instantly without moving. End the recording >0.4 m (xy_goal_tolerance)
        # short of the start point to avoid this.
        loop_gap = math.hypot(
            waypoints[-1]['x'] - waypoints[0]['x'],
            waypoints[-1]['y'] - waypoints[0]['y'],
        )
        if loop_gap < 0.5:
            navigator.get_logger().warn(
                f'{f}: path start and end are only {loop_gap:.2f} m apart. If the '
                f'robot starts near the path end, FollowPath may report SUCCESS '
                f'immediately without moving (goal checker xy_goal_tolerance is '
                f'0.4 m). Trim the last waypoint(s) or end recording short of the start.'
            )

    # --- Wait for Nav2 ---
    # WHY waitUntilNav2Active: ensures AMCL is localized and Nav2 lifecycle
    # nodes are in active state before sending goals. Without this,
    # followPath will silently fail or get rejected.
    navigator.waitUntilNav2Active()

    # --- Optional stage 0: straight reverse out of the charging station ---
    mission_ok = True
    if backup_distance > 0.0:
        result = backup_straight(navigator, backup_distance, backup_speed)
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('[backup] Completed successfully.')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn('[backup] Canceled — aborting mission.')
            mission_ok = False
        else:
            navigator.get_logger().error(f'[backup] Failed ({result}) — aborting mission.')
            mission_ok = False

    # --- Follow each path in sequence ---
    for i, (f, waypoints) in enumerate(missions):
        if not mission_ok:
            break
        if _cancel_requested:
            mission_ok = False
            break

        path = waypoints_to_path(waypoints, spacing, navigator)
        label = f'path {i + 1}/{len(missions)}'
        navigator.get_logger().info(
            f'[{label}] Sending {f}: {len(path.poses)} poses '
            f'({len(waypoints)} waypoints, interpolated to {spacing} m)...'
        )
        result = follow_one(navigator, path, label)

        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f'[{label}] Completed successfully.')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn(f'[{label}] Canceled — aborting mission.')
            mission_ok = False
            break
        else:
            navigator.get_logger().error(f'[{label}] Failed ({result}) — aborting mission.')
            mission_ok = False
            break

    if mission_ok:
        navigator.get_logger().info('Mission complete: all paths finished.')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
