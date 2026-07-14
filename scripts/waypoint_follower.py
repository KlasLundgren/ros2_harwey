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
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf2_ros
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


def get_robot_pose(navigator: BasicNavigator, tf_buffer: tf2_ros.Buffer,
                   timeout_sec: float = 5.0) -> tuple[float, float] | None:
    """Return (x, y) of base_link in the map frame, or None if TF unavailable."""
    deadline = navigator.get_clock().now() + Duration(seconds=timeout_sec)
    while navigator.get_clock().now() < deadline:
        # WHY spin_once: the TF listener only receives transforms while the
        # node is spun; BasicNavigator has no background executor.
        rclpy.spin_once(navigator, timeout_sec=0.1)
        try:
            tf = tf_buffer.lookup_transform('map', 'base_link', Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            continue
    return None


def trim_start_to_robot(waypoints: list[dict], rx: float, ry: float,
                        search_arc: float = 10.0) -> tuple[list[dict], int, float]:
    """Drop leading waypoints so the path starts at the waypoint nearest the robot.

    WHY: MPPI only matches the robot to the path inside the local costmap
    window (~half its width, 1.5 m here). Odometry slip in sand and AMCL
    drift mean the robot never lands exactly on the recorded start; if the
    gap exceeds that window the transformed plan is empty and FollowPath
    aborts with 'Resulting plan has 0 poses in it'. The search is limited to
    the first `search_arc` meters of the path so a coverage row passing near
    the start point cannot swallow the mission.

    Returns (trimmed waypoints, number dropped, distance robot->new start).
    """
    best_i = 0
    best_d = math.hypot(waypoints[0]['x'] - rx, waypoints[0]['y'] - ry)
    arc = 0.0
    for i in range(1, len(waypoints)):
        arc += math.hypot(
            waypoints[i]['x'] - waypoints[i - 1]['x'],
            waypoints[i]['y'] - waypoints[i - 1]['y'],
        )
        if arc > search_arc:
            break
        d = math.hypot(waypoints[i]['x'] - rx, waypoints[i]['y'] - ry)
        if d < best_d:
            best_i, best_d = i, d
    return waypoints[best_i:], best_i, best_d


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

    # TF listener for trimming each path to the robot's actual position
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, navigator)

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

        label = f'path {i + 1}/{len(missions)}'

        # Start the path from wherever the robot actually is, not where the
        # recording assumed it would be (see trim_start_to_robot docstring).
        robot_pose = get_robot_pose(navigator, tf_buffer)
        if robot_pose is None:
            navigator.get_logger().warn(
                f'[{label}] Could not get robot pose from TF — sending path untrimmed.'
            )
        else:
            waypoints, dropped, gap = trim_start_to_robot(waypoints, *robot_pose)
            if dropped:
                navigator.get_logger().info(
                    f'[{label}] Trimmed {dropped} leading waypoint(s); '
                    f'path now starts {gap:.2f} m from robot.'
                )
            if len(waypoints) < 2:
                navigator.get_logger().error(
                    f'[{label}] Fewer than 2 waypoints remain after trimming — '
                    f'robot is already near the end of this path. Aborting mission.'
                )
                mission_ok = False
                break
            # WHY a connector instead of a bigger costmap: MPPI can only match
            # the robot to path poses inside the local costmap (~1.5 m here).
            # A recording deliberately started ahead of the backup-end position
            # (good practice — gives runway for the first turn) leaves a gap no
            # amount of trimming can close. Prepending the robot's own pose,
            # oriented toward the path start, turns the gap into a straight
            # FORWARD lead-in segment (densified by the normal interpolation),
            # which is trailer-safe. Cheaper than enlarging the costmap on a Pi.
            if gap > 0.5:
                rx, ry = robot_pose
                bearing = math.atan2(waypoints[0]['y'] - ry, waypoints[0]['x'] - rx)
                connector = {
                    'x': rx, 'y': ry, 'z': 0.0, 'qx': 0.0, 'qy': 0.0,
                    'qz': math.sin(bearing / 2.0), 'qw': math.cos(bearing / 2.0),
                }
                waypoints = [connector] + waypoints
                navigator.get_logger().info(
                    f'[{label}] Robot is {gap:.2f} m from the path start — '
                    f'prepending a straight forward connector from the robot pose.'
                )

        path = waypoints_to_path(waypoints, spacing, navigator)
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
