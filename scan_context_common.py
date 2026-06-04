#!/usr/bin/env python3
"""Shared helpers for Scan Context recording and localization."""

from __future__ import annotations

from array import array
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import csv
import math
import os
import logging
import struct
import tempfile
from typing import Dict, List, Optional, Sequence, Tuple

import yaml


SCDB_MAGIC = b"SCDBv1\x00\x00"
SCDB_VERSION = 1

Descriptor = List[List[float]]
RingKey = List[float]


@dataclass(frozen=True)
class PoseRecord:
    stamp_sec: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    points: int


@dataclass
class ScanContextEntry:
    index: int
    pose: PoseRecord
    descriptor: Descriptor
    ring_key: RingKey = field(default_factory=list)


@dataclass
class ScanContextDatabase:
    metadata: Dict
    entries: List[ScanContextEntry]
    num_rings: int
    num_sectors: int


class ExportLogger:
    """Write logs to both ROS and a timestamped file."""

    def __init__(self, node_name: str, ros_logger, log_dir: Path) -> None:
        self._ros_logger = ros_logger
        self._node_name = node_name
        self.log_path = self._create_log_file(node_name, log_dir)
        self._py_logger = logging.getLogger(f"scan_context.{node_name}.{id(self)}")
        self._py_logger.setLevel(logging.INFO)
        self._py_logger.propagate = False
        handler = logging.FileHandler(self.log_path, encoding="utf-8")
        handler.setFormatter(
            logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
        )
        self._py_logger.addHandler(handler)
        self._file_handler = handler

    @staticmethod
    def _create_log_file(node_name: str, log_dir: Path) -> Path:
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        return log_dir / f"{node_name}-{timestamp}.log"

    def _emit(self, level: str, message: str) -> None:
        message = str(message)
        # rclpy Humble locks logger severity after first call; mixing
        # info/warn/error would raise ValueError.  Route everything
        # through .info() on the ROS side and prefix with the actual
        # level so console output stays readable.  The file logger
        # (Python logging) still uses the real severity.
        prefixed = "[%s] %s" % (level.upper(), message)
        self._ros_logger.info(prefixed)
        getattr(self._py_logger, level)(message)

    def debug(self, message: str) -> None:
        self._emit("debug", message)

    def info(self, message: str) -> None:
        self._emit("info", message)

    def warning(self, message: str) -> None:
        self._emit("warning", message)

    def warn(self, message: str) -> None:
        self.warning(message)

    def error(self, message: str) -> None:
        self._emit("error", message)

    def fatal(self, message: str) -> None:
        self._emit("fatal", message)

    def close(self) -> None:
        handlers = list(self._py_logger.handlers)
        for handler in handlers:
            handler.flush()
            handler.close()
            self._py_logger.removeHandler(handler)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def pose_distance_2d(a: PoseRecord, b: PoseRecord) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def yaw_difference(a: float, b: float) -> float:
    return abs(wrap_angle(a - b))


def transform_point_2d(
    x: float, y: float, tx: float, ty: float, yaw: float
) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        tx + x * cos_yaw - y * sin_yaw,
        ty + x * sin_yaw + y * cos_yaw,
    )


def build_descriptor_from_points(
    points: Sequence[Tuple[float, float]],
    num_rings: int,
    num_sectors: int,
    min_radius: float,
    max_radius: float,
) -> Tuple[Descriptor, int]:
    if num_rings <= 0:
        raise ValueError("num_rings must be positive")
    if num_sectors <= 0:
        raise ValueError("num_sectors must be positive")
    if max_radius <= min_radius:
        raise ValueError("max_radius must be greater than min_radius")

    descriptor: Descriptor = [
        [0.0 for _ in range(num_sectors)] for _ in range(num_rings)
    ]

    radius_span = max_radius - min_radius
    ring_width = radius_span / float(num_rings)
    sector_width = (2.0 * math.pi) / float(num_sectors)
    valid_points = 0

    for x, y in points:
        radius = math.hypot(x, y)
        if radius < min_radius or radius > max_radius:
            continue

        angle = math.atan2(y, x)
        sector = int((angle + math.pi) / sector_width)
        if sector < 0:
            continue
        if sector >= num_sectors:
            sector = num_sectors - 1

        ring = int((radius - min_radius) / ring_width)
        if ring < 0:
            continue
        if ring >= num_rings:
            ring = num_rings - 1

        score = 1.0 - (radius - min_radius) / radius_span
        if score > descriptor[ring][sector]:
            descriptor[ring][sector] = score
        valid_points += 1

    return descriptor, valid_points


def build_ring_key(descriptor: Descriptor) -> RingKey:
    return [max(row) if row else 0.0 for row in descriptor]


def vector_distance(a: Sequence[float], b: Sequence[float]) -> float:
    total = 0.0
    for value_a, value_b in zip(a, b):
        delta = value_a - value_b
        total += delta * delta
    return math.sqrt(total)


def circular_descriptor_distance(
    query: Descriptor, reference: Descriptor
) -> Tuple[float, int]:
    if not query or not reference:
        return float("inf"), 0
    num_sectors = len(query[0])
    best_distance = float("inf")
    best_shift = 0

    for shift in range(num_sectors):
        total = 0.0
        compared = 0
        for row_query, row_reference in zip(query, reference):
            shifted = row_reference[-shift:] + row_reference[:-shift] if shift else row_reference
            for value_query, value_reference in zip(row_query, shifted):
                if value_query == 0.0 and value_reference == 0.0:
                    continue
                total += abs(value_query - value_reference)
                compared += 1
        if compared == 0:
            continue
        distance = total / float(compared)
        if distance < best_distance:
            best_distance = distance
            best_shift = shift

    return best_distance, best_shift


def build_database_metadata(**kwargs) -> Dict:
    metadata: Dict = dict(kwargs)
    return metadata


def _atomic_write(path: Path, writer) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(fd, "wb") as tmp:
            writer(tmp)
            tmp.flush()
            os.fsync(tmp.fileno())
        os.replace(tmp_path, path)
    finally:
        if tmp_path.exists():
            try:
                tmp_path.unlink()
            except FileNotFoundError:
                pass


def save_database(
    directory: Path,
    entries: Sequence[ScanContextEntry],
    metadata: Dict,
    descriptors_name: str = "descriptors.bin",
    poses_name: str = "poses.csv",
    metadata_name: str = "metadata.yaml",
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    if entries:
        num_rings = len(entries[0].descriptor)
        num_sectors = len(entries[0].descriptor[0]) if num_rings else 0
    else:
        num_rings = int(metadata.get("scan_context", {}).get("sc_num_rings", 0))
        num_sectors = int(metadata.get("scan_context", {}).get("sc_num_sectors", 0))

    metadata = dict(metadata)
    metadata["keyframes"] = len(entries)
    metadata.setdefault("files", {})
    metadata["files"] = {
        "descriptors": descriptors_name,
        "poses": poses_name,
        "metadata": metadata_name,
    }

    def write_descriptors(handle) -> None:
        handle.write(
            struct.pack(
                "<8sIIII",
                SCDB_MAGIC,
                SCDB_VERSION,
                len(entries),
                num_rings,
                num_sectors,
            )
        )
        flat = array("d")
        for entry in entries:
            for row in entry.descriptor:
                flat.extend(row)
        handle.write(flat.tobytes())

    _atomic_write(directory / descriptors_name, write_descriptors)

    poses_path = directory / poses_name
    poses_fd, poses_tmp_name = tempfile.mkstemp(
        prefix=poses_path.name + ".", suffix=".tmp", dir=str(directory)
    )
    poses_tmp_path = Path(poses_tmp_name)
    try:
        with os.fdopen(poses_fd, "w", encoding="utf-8", newline="") as tmp:
            writer = csv.writer(tmp)
            writer.writerow(
                ["index", "stamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw", "points"]
            )
            for entry in entries:
                pose = entry.pose
                writer.writerow(
                    [
                        entry.index,
                        f"{pose.stamp_sec:.9f}",
                        f"{pose.x:.9f}",
                        f"{pose.y:.9f}",
                        f"{pose.z:.9f}",
                        f"{pose.qx:.12f}",
                        f"{pose.qy:.12f}",
                        f"{pose.qz:.12f}",
                        f"{pose.qw:.12f}",
                        pose.points,
                    ]
                )
            tmp.flush()
            os.fsync(tmp.fileno())
        os.replace(poses_tmp_path, poses_path)
    finally:
        if poses_tmp_path.exists():
            try:
                poses_tmp_path.unlink()
            except FileNotFoundError:
                pass

    metadata_path = directory / metadata_name
    metadata_fd, metadata_tmp_name = tempfile.mkstemp(
        prefix=metadata_path.name + ".", suffix=".tmp", dir=str(directory)
    )
    metadata_tmp_path = Path(metadata_tmp_name)
    try:
        with os.fdopen(metadata_fd, "w", encoding="utf-8") as tmp:
            yaml.safe_dump(metadata, tmp, sort_keys=False, allow_unicode=True)
            tmp.flush()
            os.fsync(tmp.fileno())
        os.replace(metadata_tmp_path, metadata_path)
    finally:
        if metadata_tmp_path.exists():
            try:
                metadata_tmp_path.unlink()
            except FileNotFoundError:
                pass


def load_database(directory: Path) -> ScanContextDatabase:
    metadata_path = directory / "metadata.yaml"
    if not metadata_path.exists():
        raise FileNotFoundError(f"metadata file not found: {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as handle:
        metadata = yaml.safe_load(handle) or {}

    files = metadata.get("files", {})
    descriptors_path = directory / files.get("descriptors", "descriptors.bin")
    poses_path = directory / files.get("poses", "poses.csv")

    with descriptors_path.open("rb") as handle:
        header = handle.read(struct.calcsize("<8sIIII"))
        if len(header) != struct.calcsize("<8sIIII"):
            raise ValueError(f"descriptor header too short: {descriptors_path}")
        magic, version, count, num_rings, num_sectors = struct.unpack("<8sIIII", header)
        if magic != SCDB_MAGIC:
            raise ValueError(f"invalid descriptor magic in {descriptors_path}")
        if version != SCDB_VERSION:
            raise ValueError(
                f"unsupported descriptor version {version} in {descriptors_path}"
            )
        raw = array("d")
        raw.frombytes(handle.read())

    expected_values = count * num_rings * num_sectors
    if len(raw) != expected_values:
        raise ValueError(
            "descriptor payload size mismatch: expected %d doubles, got %d"
            % (expected_values, len(raw))
        )

    descriptors: List[Descriptor] = []
    offset = 0
    for _ in range(count):
        descriptor: Descriptor = []
        for _ in range(num_rings):
            row = list(raw[offset : offset + num_sectors])
            offset += num_sectors
            descriptor.append(row)
        descriptors.append(descriptor)

    entries: List[ScanContextEntry] = []
    with poses_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        pose_rows = list(reader)

    if len(pose_rows) != count:
        raise ValueError(
            "pose count mismatch: descriptors=%d poses=%d"
            % (count, len(pose_rows))
        )

    for row, descriptor in zip(pose_rows, descriptors):
        pose = PoseRecord(
            stamp_sec=float(row["stamp_sec"]),
            x=float(row["x"]),
            y=float(row["y"]),
            z=float(row["z"]),
            qx=float(row["qx"]),
            qy=float(row["qy"]),
            qz=float(row["qz"]),
            qw=float(row["qw"]),
            points=int(row["points"]),
        )
        entries.append(
            ScanContextEntry(
                index=int(row["index"]),
                pose=pose,
                descriptor=descriptor,
                ring_key=build_ring_key(descriptor),
            )
        )

    return ScanContextDatabase(
        metadata=metadata,
        entries=entries,
        num_rings=num_rings,
        num_sectors=num_sectors,
    )
