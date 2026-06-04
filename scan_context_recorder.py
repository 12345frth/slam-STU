#!/usr/bin/env python3
"""Record Scan Context keyframes while mapping.

This node subscribes to a live LaserScan topic, transforms each scan into the
base frame using TF, and records keyframes whenever the motion/pose threshold
is exceeded. The resulting database is written to:

- descriptors.bin
- poses.csv
- metadata.yaml

The file layout intentionally mirrors the reference map export directory.
"""

from __future__ import annotations

from pathlib import Path
import math
from datetime import datetime, timezone
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from scan_context_common import (
    ExportLogger,
    PoseRecord,
    ScanContextEntry,
    build_database_metadata,
    build_descriptor_from_points,
    build_ring_key,
    load_database,
    pose_distance_2d,
    quaternion_to_yaw,
    save_database,
    transform_point_2d,
    wrap_angle,
    yaw_difference,
)


class ScanContextRecorder(Node):
    def __init__(self) -> None:
        super().__init__("scan_context_recorder")

        self.declare_parameter("output_dir", "scan_context")
        self.declare_parameter("log_dir", "logs")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("keyframe_min_interval_sec", 0.4)
        self.declare_parameter("keyframe_min_translation_m", 0.20)
        self.declare_parameter("keyframe_min_yaw_deg", 6.0)
        self.declare_parameter("sc_num_rings", 40)
        self.declare_parameter("sc_num_sectors", 120)
        # Retain near-field structure for narrow-corridor TurtleBot3 scans.
        self.declare_parameter("descriptor_min_range", 0.35)
        self.declare_parameter("descriptor_max_range", 8.0)
        self.declare_parameter("min_points", 80)
        self.declare_parameter("tf_lookup_timeout_sec", 0.2)
        self.declare_parameter("save_on_every_keyframe", True)

        self.log_dir = Path(str(self.get_parameter("log_dir").value)).expanduser()
        if not self.log_dir.is_absolute():
            self.log_dir = Path.cwd() / self.log_dir
        self.log = ExportLogger("scan_context_recorder", self.get_logger(), self.log_dir)

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        if not self.output_dir.is_absolute():
            relative_output_dir = self.output_dir
            resolved_output_dir = Path.cwd() / relative_output_dir
            if Path.cwd().name == relative_output_dir.name:
                self.log.warn(
                    "Relative output_dir=%s resolves to nested path %s. Use output_dir:=. or an absolute path if you want files in the current directory."
                    % (relative_output_dir, resolved_output_dir)
                )
            self.output_dir = resolved_output_dir

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.keyframe_min_interval_sec = float(
            self.get_parameter("keyframe_min_interval_sec").value
        )
        self.keyframe_min_translation_m = float(
            self.get_parameter("keyframe_min_translation_m").value
        )
        self.keyframe_min_yaw_deg = float(
            self.get_parameter("keyframe_min_yaw_deg").value
        )
        self.sc_num_rings = int(self.get_parameter("sc_num_rings").value)
        self.sc_num_sectors = int(self.get_parameter("sc_num_sectors").value)
        self.descriptor_min_range = float(
            self.get_parameter("descriptor_min_range").value
        )
        self.descriptor_max_range = float(
            self.get_parameter("descriptor_max_range").value
        )
        self.min_points = int(self.get_parameter("min_points").value)
        self.tf_lookup_timeout_sec = float(
            self.get_parameter("tf_lookup_timeout_sec").value
        )
        self.save_on_every_keyframe = bool(
            self.get_parameter("save_on_every_keyframe").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )

        self.entries: List[ScanContextEntry] = []
        self.next_index = 0
        self.received_scans = 0
        self.saved_keyframes = 0
        self.skipped_no_pose = 0
        self.skipped_tf_error = 0
        self.skipped_low_points = 0
        self.skipped_keyframe_gap = 0
        self.tf_lookup_failure_count = 0
        self.scan_frame: Optional[str] = None
        self.last_recorded_pose: Optional[PoseRecord] = None
        self.last_recorded_stamp_sec: Optional[float] = None
        self.last_descriptor_max_range: Optional[float] = None
        self._periodic_report_interval = 200
        self._shutdown_stats_printed = False

        self._load_existing_database_if_present()

        self.log.info(
            "Scan context recorder ready: scan=%s map_frame=%s base_frame=%s output=%s"
            % (self.scan_topic, self.map_frame, self.base_frame, self.output_dir)
        )
        self.log.info(
            "Keyframe gating: interval=%.2fs translation=%.2fm yaw=%.1fdeg rings=%d sectors=%d min_points=%d"
            % (
                self.keyframe_min_interval_sec,
                self.keyframe_min_translation_m,
                self.keyframe_min_yaw_deg,
                self.sc_num_rings,
                self.sc_num_sectors,
                self.min_points,
            )
        )

    def _load_existing_database_if_present(self) -> None:
        metadata_path = self.output_dir / "metadata.yaml"
        descriptors_path = self.output_dir / "descriptors.bin"
        poses_path = self.output_dir / "poses.csv"
        if metadata_path.exists() and descriptors_path.exists() and poses_path.exists():
            db = load_database(self.output_dir)
            self.entries = list(db.entries)
            self.next_index = len(self.entries)
            if self.entries:
                self.last_recorded_pose = self.entries[-1].pose
                self.last_recorded_stamp_sec = self.entries[-1].pose.stamp_sec
            self.scan_frame = db.metadata.get("scan_frame") or db.metadata.get(
                "cloud_frame"
            )
            if db.num_rings != self.sc_num_rings or db.num_sectors != self.sc_num_sectors:
                self.log.warn(
                    "Existing database ring/sector size (%d/%d) differs from current parameters (%d/%d); continuing with existing database geometry."
                    % (
                        db.num_rings,
                        db.num_sectors,
                        self.sc_num_rings,
                        self.sc_num_sectors,
                    )
                )
                self.sc_num_rings = db.num_rings
                self.sc_num_sectors = db.num_sectors
            self.log.info(
                "Resumed existing scan context database with %d keyframes"
                % len(self.entries)
            )

    def _on_scan(self, msg: LaserScan) -> None:
        self.received_scans += 1

        scan_frame = msg.header.frame_id or self.scan_frame
        if not scan_frame:
            self.skipped_no_pose += 1
            self.log.warn("Scan frame is empty; skipping frame")
            return
        if self.scan_frame is None:
            self.scan_frame = scan_frame
        elif self.scan_frame != scan_frame:
            self.log.warn(
                "Scan frame changed from %s to %s" % (self.scan_frame, scan_frame)
            )

        # Periodic diagnostics every N frames — must fire before any
        # early-return so you can see skip distribution even when TF fails.
        if self.received_scans % self._periodic_report_interval == 0:
            self.log.info(
                "frame=%d keyframes=%d saved=%d skip(no_pose=%d tf=%d low_pts=%d gap=%d)"
                % (self.received_scans, len(self.entries),
                   self.saved_keyframes, self.skipped_no_pose,
                   self.skipped_tf_error, self.skipped_low_points,
                   self.skipped_keyframe_gap)
            )

        current_pose = self._lookup_map_pose(msg.header.stamp)
        if current_pose is None:
            self.skipped_no_pose += 1
            return

        scan_to_base = self._lookup_scan_to_base(msg.header.stamp, scan_frame)
        if scan_to_base is None:
            self.skipped_tf_error += 1
            return

        points = self._extract_points_in_base_frame(msg, scan_to_base, scan_frame)
        descriptor_max_range = (
            self.descriptor_max_range
            if self.descriptor_max_range > self.descriptor_min_range
            else float(msg.range_max)
        )
        self.last_descriptor_max_range = descriptor_max_range
        descriptor, valid_points = build_descriptor_from_points(
            points,
            self.sc_num_rings,
            self.sc_num_sectors,
            self.descriptor_min_range,
            descriptor_max_range,
        )
        if valid_points < self.min_points:
            self.skipped_low_points += 1
            return

        if self._should_record_keyframe(
            current_pose, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ):
            recorded_pose = PoseRecord(
                stamp_sec=current_pose.stamp_sec,
                x=current_pose.x,
                y=current_pose.y,
                z=current_pose.z,
                qx=current_pose.qx,
                qy=current_pose.qy,
                qz=current_pose.qz,
                qw=current_pose.qw,
                points=valid_points,
            )
            entry = ScanContextEntry(
                index=self.next_index,
                pose=recorded_pose,
                descriptor=descriptor,
                ring_key=build_ring_key(descriptor),
            )
            self.entries.append(entry)
            self.next_index += 1
            self.saved_keyframes += 1
            self.last_recorded_pose = recorded_pose
            self.last_recorded_stamp_sec = (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            )

            if self.save_on_every_keyframe:
                self._persist_database()

            self.log.info(
                "Recorded keyframe %d: stamp=%.3f pose=(%.3f, %.3f, %.1fdeg) points=%d total=%d"
                % (
                    entry.index,
                    recorded_pose.stamp_sec,
                    recorded_pose.x,
                    recorded_pose.y,
                    math.degrees(
                        wrap_angle(
                            quaternion_to_yaw(
                                recorded_pose.qx,
                                recorded_pose.qy,
                                recorded_pose.qz,
                                recorded_pose.qw,
                            )
                        )
                    ),
                    valid_points,
                    len(self.entries),
                )
            )
        else:
            self.skipped_keyframe_gap += 1

    def _log_shutdown_stats(self) -> None:
        if self._shutdown_stats_printed:
            return
        self._shutdown_stats_printed = True
        self.log.info(
            "=== Shutdown: %d scans, %d keyframes saved ==="
            % (self.received_scans, len(self.entries))
        )
        self.log.info(
            "Skipped: no_pose=%d tf_err=%d low_pts=%d gap=%d"
            % (self.skipped_no_pose, self.skipped_tf_error,
               self.skipped_low_points, self.skipped_keyframe_gap)
        )

    def _lookup_map_pose(self, stamp) -> Optional[PoseRecord]:
        # Try exact timestamp first, then fall back to latest available
        # when sim-time clock drift causes extrapolation rejections.
        for attempt, req_time in enumerate((stamp, rclpy.time.Time())):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    req_time,
                    timeout=Duration(seconds=self.tf_lookup_timeout_sec),
                )
                break
            except TransformException as exc:
                if attempt == 1 or "extrapolation" not in str(exc).lower():
                    self.tf_lookup_failure_count += 1
                    stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                    can_transform_now = self.tf_buffer.can_transform(
                        self.map_frame,
                        self.base_frame,
                        stamp,
                        timeout=Duration(seconds=0.0),
                    )
                    self.log.warn(
                        "TF lookup failed #%d for %s -> %s at scan_stamp=%.3f can_transform_now=%s timeout=%.2fs error=%s"
                        % (
                            self.tf_lookup_failure_count,
                            self.map_frame,
                            self.base_frame,
                            stamp_sec,
                            "yes" if can_transform_now else "no",
                            self.tf_lookup_timeout_sec,
                            exc,
                        )
                    )
                    return None
                # Extrapolation-only failure on first attempt: silently
                # retry with latest available transform.
                continue

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return PoseRecord(
            stamp_sec=stamp_sec,
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
            qx=float(rotation.x),
            qy=float(rotation.y),
            qz=float(rotation.z),
            qw=float(rotation.w),
            points=0,
        )

    def _lookup_scan_to_base(self, stamp, scan_frame: str):
        for attempt, req_time in enumerate((stamp, rclpy.time.Time())):
            try:
                return self.tf_buffer.lookup_transform(
                    self.base_frame,
                    scan_frame,
                    req_time,
                    timeout=Duration(seconds=self.tf_lookup_timeout_sec),
                )
            except TransformException as exc:
                if attempt == 1 or "extrapolation" not in str(exc).lower():
                    stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                    can_transform_now = self.tf_buffer.can_transform(
                        self.base_frame,
                        scan_frame,
                        stamp,
                        timeout=Duration(seconds=0.0),
                    )
                    self.log.warn(
                        "TF lookup failed for %s -> %s at scan_stamp=%.3f can_transform_now=%s timeout=%.2fs error=%s"
                        % (
                            self.base_frame,
                            scan_frame,
                            stamp_sec,
                            "yes" if can_transform_now else "no",
                            self.tf_lookup_timeout_sec,
                            exc,
                        )
                    )
                    return None

    def _extract_points_in_base_frame(
        self, msg: LaserScan, scan_to_base, scan_frame: str
    ) -> List[Tuple[float, float]]:
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

        points: List[Tuple[float, float]] = []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < self.descriptor_min_range or distance > max_range:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x_scan = distance * math.cos(angle)
            y_scan = distance * math.sin(angle)
            x_base, y_base = transform_point_2d(x_scan, y_scan, tx, ty, yaw)
            points.append((x_base, y_base))
        return points

    def _should_record_keyframe(self, current_pose: PoseRecord, stamp_sec: float) -> bool:
        if not self.entries:
            return True
        if self.last_recorded_pose is None or self.last_recorded_stamp_sec is None:
            return True

        dt = stamp_sec - self.last_recorded_stamp_sec
        if dt < self.keyframe_min_interval_sec:
            return False

        current_yaw = quaternion_to_yaw(
            current_pose.qx,
            current_pose.qy,
            current_pose.qz,
            current_pose.qw,
        )
        last_yaw = quaternion_to_yaw(
            self.last_recorded_pose.qx,
            self.last_recorded_pose.qy,
            self.last_recorded_pose.qz,
            self.last_recorded_pose.qw,
        )
        translation = pose_distance_2d(current_pose, self.last_recorded_pose)
        yaw_delta_deg = math.degrees(yaw_difference(current_yaw, last_yaw))
        return (
            translation >= self.keyframe_min_translation_m
            or yaw_delta_deg >= self.keyframe_min_yaw_deg
        )

    def _persist_database(self) -> None:
        metadata = build_database_metadata(
            generated_by="scan_context_recorder",
            created_at=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            map_frame=self.map_frame,
            base_frame=self.base_frame,
            cloud_frame=self.base_frame,
            scan_frame=self.scan_frame,
            input_scan_topic=self.scan_topic,
            keyframe_min_interval_sec=self.keyframe_min_interval_sec,
            keyframe_min_translation_m=self.keyframe_min_translation_m,
            keyframe_min_yaw_deg=self.keyframe_min_yaw_deg,
            min_points=self.min_points,
            files={
                "descriptors": "descriptors.bin",
                "poses": "poses.csv",
                "metadata": "metadata.yaml",
            },
            scan_context={
                "sc_lidar_height": 0.0,
                "sc_max_radius": float(
                    self.last_descriptor_max_range
                    if self.last_descriptor_max_range is not None
                    else self.descriptor_max_range
                ),
                "sc_min_radius": self.descriptor_min_range,
                "sc_num_rings": self.sc_num_rings,
                "sc_num_sectors": self.sc_num_sectors,
            },
            counts={
                "received_scans": self.received_scans,
                "skipped_no_pose": self.skipped_no_pose,
                "skipped_tf_error": self.skipped_tf_error,
                "skipped_low_points": self.skipped_low_points,
                "skipped_keyframe_gap": self.skipped_keyframe_gap,
            },
        )
        save_database(self.output_dir, self.entries, metadata)

    def destroy_node(self) -> None:
        try:
            self._persist_database()
            self._log_shutdown_stats()
        except Exception as exc:
            self.log.error("Failed to persist database on shutdown: %s" % exc)
        finally:
            self.log.close()
            super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[ScanContextRecorder] = None
    try:
        node = ScanContextRecorder()
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
