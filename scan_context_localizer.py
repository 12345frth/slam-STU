#!/usr/bin/env python3
"""Localize by matching live Scan Context descriptors against a recorded DB."""

from __future__ import annotations

from pathlib import Path
import math
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from scan_context_common import (
    ExportLogger,
    ScanContextDatabase,
    build_descriptor_from_points,
    build_ring_key,
    circular_descriptor_distance,
    load_database,
    transform_point_2d,
    vector_distance,
    quaternion_to_yaw,
    wrap_angle,
    yaw_to_quaternion,
)


def _make_color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    color = ColorRGBA()
    color.r = r
    color.g = g
    color.b = b
    color.a = a
    return color


class ScanContextLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("scan_context_localizer")

        self.declare_parameter("database_dir", "scan_context")
        self.declare_parameter("log_dir", "logs")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("publish_initial_pose", True)
        self.declare_parameter("publish_cooldown_sec", 5.0)
        self.declare_parameter("min_valid_points", 30)
        self.declare_parameter("top_k_key_candidates", 60)
        self.declare_parameter("key_distance_threshold", 1.2)
        self.declare_parameter("desc_distance_threshold", 0.25)
        self.declare_parameter("tf_lookup_timeout_sec", 0.2)
        self.declare_parameter("show_amcl_refined_pose", True)

        self.database_dir = Path(str(self.get_parameter("database_dir").value)).expanduser()
        if not self.database_dir.is_absolute():
            self.database_dir = Path.cwd() / self.database_dir

        self.log_dir = Path(str(self.get_parameter("log_dir").value)).expanduser()
        if not self.log_dir.is_absolute():
            self.log_dir = Path.cwd() / self.log_dir

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self.publish_initial_pose = bool(self.get_parameter("publish_initial_pose").value)
        self.publish_cooldown_sec = float(self.get_parameter("publish_cooldown_sec").value)
        self.min_valid_points = int(self.get_parameter("min_valid_points").value)
        self.top_k_key_candidates = int(self.get_parameter("top_k_key_candidates").value)
        self.key_distance_threshold = float(self.get_parameter("key_distance_threshold").value)
        self.desc_distance_threshold = float(self.get_parameter("desc_distance_threshold").value)
        self.tf_lookup_timeout_sec = float(self.get_parameter("tf_lookup_timeout_sec").value)
        self.show_amcl_refined_pose = bool(self.get_parameter("show_amcl_refined_pose").value)
        self.log = ExportLogger("scan_context_localizer", self.get_logger(), self.log_dir)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_pose_topic, self._on_amcl_pose, 10
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, "/scan_context_markers", 10)

        self.db: ScanContextDatabase = load_database(self.database_dir)
        self.num_rings = self.db.num_rings
        self.num_sectors = self.db.num_sectors
        self.descriptor_min_range = float(
            self.db.metadata.get("scan_context", {}).get("sc_min_radius", 0.5)
        )
        self.descriptor_max_range = float(
            self.db.metadata.get("scan_context", {}).get("sc_max_radius", 0.0)
        )
        if self.min_valid_points <= 0:
            self.min_valid_points = int(self.db.metadata.get("min_points", 80))

        self.received_scans = 0
        self.last_initialpose_publish_ns = -1
        self.latest_amcl_pose: Optional[Tuple[float, float, float]] = None
        self.latest_amcl_received_ns = -1
        # Sliding window of recent descriptor distances for threshold tuning.
        self._recent_desc_dists: list = []
        self._match_log_interval = 20  # log match-quality summary every N matches

        self.log.info(
            "Loaded database %s with %d keyframes (rings=%d sectors=%d)"
            % (self.database_dir, len(self.db.entries), self.num_rings, self.num_sectors)
        )
        self.log.info(
            "Localization parameters: top_k=%d key_thresh=%.3f desc_thresh=%.3f min_points=%d"
            % (
                self.top_k_key_candidates,
                self.key_distance_threshold,
                self.desc_distance_threshold,
                self.min_valid_points,
            )
        )

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self.latest_amcl_pose = (pose.position.x, pose.position.y, wrap_angle(yaw))
        self.latest_amcl_received_ns = self.get_clock().now().nanoseconds

        if self.show_amcl_refined_pose:
            self._publish_pose_markers(
                namespace="amcl_refined",
                text=(
                    "AMCL refined\nx=%.2f y=%.2f yaw=%.1fdeg"
                    % (
                        self.latest_amcl_pose[0],
                        self.latest_amcl_pose[1],
                        math.degrees(self.latest_amcl_pose[2]),
                    )
                ),
                pose=self.latest_amcl_pose,
                color=(0.1, 0.9, 0.3),
            )

    def _on_scan(self, msg: LaserScan) -> None:
        self.received_scans += 1
        scan_frame = msg.header.frame_id or self.base_frame

        scan_to_base = self._lookup_scan_to_base(msg.header.stamp, scan_frame)
        if scan_to_base is None:
            return

        points = self._extract_points_in_base_frame(msg, scan_to_base)
        descriptor_max_range = (
            self.descriptor_max_range
            if self.descriptor_max_range > self.descriptor_min_range
            else float(msg.range_max)
        )
        descriptor, valid_points = build_descriptor_from_points(
            points,
            self.num_rings,
            self.num_sectors,
            self.descriptor_min_range,
            descriptor_max_range,
        )
        if valid_points < self.min_valid_points:
            self.log.warn(
                "frame=%d skipped: valid_points=%d < min_valid_points=%d"
                % (self.received_scans, valid_points, self.min_valid_points)
            )
            self._publish_status_marker(
                namespace="scan_context_status",
                text="Scan too sparse",
                position=(0.0, 0.0),
                color=(1.0, 0.2, 0.2),
                show_arrow=False,
                show_sphere=False,
            )
            return

        ring_key = build_ring_key(descriptor)
        match = self._match_query(descriptor, ring_key)
        if match is None:
            self.log.info(
                "frame=%d no confident match" % self.received_scans
            )
            self._publish_status_marker(
                namespace="scan_context_status",
                text="No confident match",
                position=(0.0, 0.0),
                color=(1.0, 0.2, 0.2),
                show_arrow=False,
                show_sphere=False,
            )
            return

        estimated_pose = self._estimate_pose(match)
        loop_candidate = match["descriptor_distance"] <= self.desc_distance_threshold
        self.log.info(
            "frame=%d match=(x=%.3f, y=%.3f, yaw=%.1fdeg) key_dist=%.3f desc_dist=%.3f shift=%d loop_candidate=%s"
            % (
                self.received_scans,
                estimated_pose[0],
                estimated_pose[1],
                math.degrees(estimated_pose[2]),
                match["key_distance"],
                match["descriptor_distance"],
                match["shift"],
                "yes" if loop_candidate else "no",
            )
        )

        # Collect descriptor distance for periodic quality summary.
        self._recent_desc_dists.append(match["descriptor_distance"])
        if len(self._recent_desc_dists) >= self._match_log_interval:
            import statistics
            vals = self._recent_desc_dists
            self.log.info(
                "desc_dist[last-%d] min=%.3f median=%.3f max=%.3f  (threshold=%.3f)"
                % (len(vals), min(vals), statistics.median(vals), max(vals),
                   self.desc_distance_threshold)
            )
            self._recent_desc_dists.clear()

        if loop_candidate and self.publish_initial_pose:
            self._publish_initial_pose(
                estimated_pose=estimated_pose,
                descriptor_distance=match["descriptor_distance"],
            )

        self._publish_status_marker(
            namespace="scan_context_initial",
            text=(
                "match %.2f / %.2f\nx=%.2f y=%.2f yaw=%.1fdeg"
                % (
                    match["key_distance"],
                    match["descriptor_distance"],
                    estimated_pose[0],
                    estimated_pose[1],
                    math.degrees(estimated_pose[2]),
                )
            ),
            position=(estimated_pose[0], estimated_pose[1]),
            color=(0.1, 0.9, 0.3) if loop_candidate else (1.0, 0.7, 0.1),
            show_arrow=True,
            show_sphere=True,
            pose=estimated_pose,
        )

    def _lookup_scan_to_base(self, stamp, scan_frame: str):
        # Try exact timestamp first, fall back to latest available when
        # sim-time clock drift causes extrapolation rejections.
        for attempt, req_time in enumerate((stamp, rclpy.time.Time())):
            try:
                return self.tf_buffer.lookup_transform(
                    self.base_frame,
                    scan_frame,
                    req_time,
                    timeout=Duration(seconds=self.tf_lookup_timeout_sec),
                )
            except TransformException:
                if attempt == 1:
                    self.log.warn(
                        "TF lookup failed for %s -> %s" % (self.base_frame, scan_frame)
                    )
                    return None
                # Extrapolation failure on first try: silently retry with latest.
                continue

    def _extract_points_in_base_frame(self, msg: LaserScan, scan_to_base) -> list:
        transform = scan_to_base.transform
        yaw = quaternion_to_yaw(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        tx = float(transform.translation.x)
        ty = float(transform.translation.y)
        max_range = (
            self.descriptor_max_range
            if self.descriptor_max_range > self.descriptor_min_range
            else float(msg.range_max)
        )
        points = []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < self.descriptor_min_range or distance > max_range:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x_scan = distance * math.cos(angle)
            y_scan = distance * math.sin(angle)
            points.append(transform_point_2d(x_scan, y_scan, tx, ty, yaw))
        return points

    def _match_query(self, query_descriptor, query_ring_key):
        candidates = []
        for entry in self.db.entries:
            key_distance = vector_distance(query_ring_key, entry.ring_key)
            candidates.append((key_distance, entry))

        candidates.sort(key=lambda item: item[0])
        candidates = candidates[: self.top_k_key_candidates]
        filtered_candidates = [
            (key_distance, entry)
            for key_distance, entry in candidates
            if key_distance <= self.key_distance_threshold
        ]
        if filtered_candidates:
            candidates = filtered_candidates
        else:
            self.log.warn(
                "No candidates passed key threshold %.3f; falling back to top-%d relaxed search"
                % (self.key_distance_threshold, self.top_k_key_candidates)
            )
        if not candidates:
            return None

        best = None
        for key_distance, entry in candidates:
            descriptor_distance, shift = circular_descriptor_distance(
                query_descriptor, entry.descriptor
            )
            candidate = {
                "entry": entry,
                "key_distance": key_distance,
                "descriptor_distance": descriptor_distance,
                "shift": shift,
            }
            if best is None or candidate["descriptor_distance"] < best["descriptor_distance"]:
                best = candidate
        return best

    def _estimate_pose(self, match) -> Tuple[float, float, float]:
        entry = match["entry"]
        yaw_shift = 2.0 * math.pi * float(match["shift"]) / float(self.num_sectors)
        yaw = wrap_angle(
            quaternion_to_yaw(entry.pose.qx, entry.pose.qy, entry.pose.qz, entry.pose.qw)
            + yaw_shift
        )
        return entry.pose.x, entry.pose.y, yaw

    def _publish_initial_pose(
        self, estimated_pose: Tuple[float, float, float], descriptor_distance: float
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (
            self.last_initialpose_publish_ns >= 0
            and self.publish_cooldown_sec > 0.0
            and (now_ns - self.last_initialpose_publish_ns)
            < int(self.publish_cooldown_sec * 1e9)
        ):
            return

        pos_sigma = max(0.15, min(1.0, 0.20 + 1.2 * descriptor_distance))
        yaw_sigma = max(0.15, min(1.2, 0.20 + 1.6 * descriptor_distance))
        qx, qy, qz, qw = yaw_to_quaternion(estimated_pose[2])

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = estimated_pose[0]
        msg.pose.pose.position.y = estimated_pose[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = pos_sigma * pos_sigma
        msg.pose.covariance[7] = pos_sigma * pos_sigma
        msg.pose.covariance[35] = yaw_sigma * yaw_sigma
        self.initial_pose_pub.publish(msg)
        self.last_initialpose_publish_ns = now_ns
        self.log.info(
            "Published /initialpose at x=%.3f y=%.3f yaw=%.1fdeg"
            % (
                estimated_pose[0],
                estimated_pose[1],
                math.degrees(estimated_pose[2]),
            )
        )

    def _publish_status_marker(
        self,
        namespace: str,
        text: str,
        position: Tuple[float, float],
        color: Tuple[float, float, float],
        show_arrow: bool,
        show_sphere: bool,
        pose: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        markers = MarkerArray()

        text_marker = Marker()
        text_marker.header.frame_id = self.map_frame
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = namespace
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = position[0]
        text_marker.pose.position.y = position[1]
        text_marker.pose.position.z = 0.6
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.35
        text_marker.color = _make_color(color[0], color[1], color[2], 1.0)
        text_marker.text = text
        markers.markers.append(text_marker)

        if show_arrow and pose is not None:
            arrow_marker = Marker()
            arrow_marker.header.frame_id = self.map_frame
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = namespace
            arrow_marker.id = 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.scale.x = 0.35
            arrow_marker.scale.y = 0.08
            arrow_marker.scale.z = 0.08
            arrow_marker.color = _make_color(color[0], color[1], color[2], 1.0)
            arrow_marker.points = [
                Point(x=pose[0], y=pose[1], z=0.05),
                Point(
                    x=pose[0] + 0.5 * math.cos(pose[2]),
                    y=pose[1] + 0.5 * math.sin(pose[2]),
                    z=0.05,
                ),
            ]
            markers.markers.append(arrow_marker)
        else:
            delete_arrow = Marker()
            delete_arrow.header.frame_id = self.map_frame
            delete_arrow.header.stamp = self.get_clock().now().to_msg()
            delete_arrow.ns = namespace
            delete_arrow.id = 1
            delete_arrow.action = Marker.DELETE
            markers.markers.append(delete_arrow)

        if show_sphere and pose is not None:
            sphere_marker = Marker()
            sphere_marker.header.frame_id = self.map_frame
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = namespace
            sphere_marker.id = 2
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = pose[0]
            sphere_marker.pose.position.y = pose[1]
            sphere_marker.pose.position.z = 0.05
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.18
            sphere_marker.scale.y = 0.18
            sphere_marker.scale.z = 0.18
            sphere_marker.color = _make_color(color[0], color[1], color[2], 0.85)
            markers.markers.append(sphere_marker)
        else:
            delete_sphere = Marker()
            delete_sphere.header.frame_id = self.map_frame
            delete_sphere.header.stamp = self.get_clock().now().to_msg()
            delete_sphere.ns = namespace
            delete_sphere.id = 2
            delete_sphere.action = Marker.DELETE
            markers.markers.append(delete_sphere)

        self.marker_pub.publish(markers)

    def _publish_pose_markers(
        self,
        namespace: str,
        text: str,
        pose: Tuple[float, float, float],
        color: Tuple[float, float, float],
    ) -> None:
        self._publish_status_marker(
            namespace=namespace,
            text=text,
            position=(pose[0], pose[1]),
            color=color,
            show_arrow=True,
            show_sphere=True,
            pose=pose,
        )

    def destroy_node(self) -> None:
        try:
            self.log.close()
        finally:
            super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[ScanContextLocalizer] = None
    try:
        node = ScanContextLocalizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
