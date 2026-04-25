#!/usr/bin/env python3
"""
Record robot poses from TF (map -> base_link) at distance intervals.
Dump to YAML on shutdown (Ctrl+C or ros2 lifecycle).

Parameters:
  - record_distance: minimum travel distance (meters) between saved waypoints. Default 1.0
  - output_file: path to YAML file. Default './recorded_waypoints.yaml'
  - tf_timeout: TF lookup timeout in seconds. Default 0.1

WHY TF instead of /amcl_pose: TF updates continuously via odom even when
AMCL's particle filter hasn't triggered, giving consistent sampling.
"""

import math
import signal
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
import yaml


def quaternion_to_yaw(qz: float, qw: float) -> float:
    """Extract yaw from a 2D-relevant quaternion (qx=qy=0)."""
    return 2.0 * math.atan2(qz, qw)


def distance_2d(p1: dict, p2: dict) -> float:
    return math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])


class WaypointRecorder(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_recorder')

        # --- Parameters (all configurable via launch or CLI) ---
        self.declare_parameter('record_distance', 1.0)
        self.declare_parameter('output_file', './recorded_waypoints.yaml')
        self.declare_parameter('tf_timeout', 0.1)

        self._record_dist: float = self.get_parameter('record_distance').value
        self._output_file: str = self.get_parameter('output_file').value
        self._tf_timeout: float = self.get_parameter('tf_timeout').value

        # --- TF ---
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- State ---
        self._waypoints: list[dict] = []
        self._last_recorded: Optional[dict] = None

        # Poll TF at 10 Hz.
        # WHY 10 Hz: at 1 m/s robot speed and 1 m record distance,
        # 10 Hz gives ~10 cm positioning granularity. Plenty for waypoint recording.
        self._timer = self.create_timer(0.1, self._timer_callback)

        # Catch Ctrl+C to dump before exit
        signal.signal(signal.SIGINT, self._signal_handler)

        self.get_logger().info(
            f'Recording waypoints every {self._record_dist:.2f} m '
            f'-> {self._output_file}'
        )

    def _timer_callback(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', Time(), rclpy.duration.Duration(seconds=self._tf_timeout)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            # WHY not crash: TF may not be available yet at startup,
            # or AMCL may briefly lose track. Log throttled, keep trying.
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        current = {
            'x': float(t.x),
            'y': float(t.y),
            'z': float(t.z),
            'qx': float(r.x),
            'qy': float(r.y),
            'qz': float(r.z),
            'qw': float(r.w),
        }

        if self._last_recorded is None:
            # Always save the starting pose
            self._save_waypoint(current)
            return

        if distance_2d(current, self._last_recorded) >= self._record_dist:
            self._save_waypoint(current)

    def _save_waypoint(self, pose: dict) -> None:
        self._waypoints.append(pose)
        self._last_recorded = pose
        yaw_deg = math.degrees(quaternion_to_yaw(pose['qz'], pose['qw']))
        self.get_logger().info(
            f'Waypoint {len(self._waypoints)}: '
            f'x={pose["x"]:.3f} y={pose["y"]:.3f} yaw={yaw_deg:.1f}°'
        )

    def _dump_yaml(self) -> None:
        if not self._waypoints:
            self.get_logger().warn('No waypoints recorded, nothing to save.')
            return

        # WHY this YAML structure: flat list of pose dicts, directly maps to
        # PoseStamped fields. Human-readable, easy to hand-edit coordinates.
        data = {'waypoints': self._waypoints}

        try:
            with open(self._output_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(
                f'Saved {len(self._waypoints)} waypoints to {self._output_file}'
            )
        except OSError as e:
            self.get_logger().error(f'Failed to write YAML: {e}')

    def _signal_handler(self, sig: int, frame) -> None:
        self.get_logger().info('Shutdown requested, dumping waypoints...')
        self._dump_yaml()
        sys.exit(0)

    def destroy_node(self) -> None:
        # Belt-and-suspenders: also dump on normal ROS shutdown
        self._dump_yaml()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
