#!/usr/bin/env python3

"""Validate OAK image/IMU diagnostics against their source ROS messages."""

import argparse
import json
import statistics
import time
from collections import defaultdict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


SOURCE_TOPICS = {
    "left": (Image, "/oak/stereo/left/image_raw"),
    "right": (Image, "/oak/stereo/right/image_raw"),
    "color": (Image, "/oak/color/image_raw"),
    "depth": (Image, "/oak/depth/image_raw"),
    "imu": (Imu, "/oak/imu/data"),
}

DIAGNOSTIC_TOPICS = {
    "left": "/oak/diagnostics/left_frame",
    "right": "/oak/diagnostics/right_frame",
    "color": "/oak/diagnostics/color_frame",
    "depth": "/oak/diagnostics/depth_frame",
    "imu": "/oak/diagnostics/imu_packet",
    "device": "/oak/diagnostics/device_info",
}

FRAME_KEYS = {
    "schema_version", "stream_name", "sequence_num",
    "device_timestamp_ns", "host_synced_timestamp_ns",
    "host_receive_time_ns", "sequence_gap",
    "cumulative_sequence_gap", "exposure_time_us",
    "sensitivity_iso", "width", "height",
}

IMU_KEYS = {
    "schema_version", "accelerometer_sequence_num",
    "gyroscope_sequence_num", "accelerometer_device_timestamp_ns",
    "gyroscope_device_timestamp_ns",
    "accelerometer_host_synced_timestamp_ns",
    "gyroscope_host_synced_timestamp_ns", "host_receive_time_ns",
    "accel_to_gyro_device_delta_ns", "accelerometer_sequence_gap",
    "gyroscope_sequence_gap", "batch_size", "packet_index",
}

DEVICE_KEYS = {
    "schema_version", "mx_id", "connected_imu", "usb_speed",
    "depthai_version",
}


def stamp_ns(header):
    """Convert a ROS header stamp to integer nanoseconds."""
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


def values_dict(message):
    """Convert the first diagnostic status values to a dictionary."""
    if not message.status:
        return {}
    return {item.key: item.value for item in message.status[0].values}


class DiagnosticValidator(Node):
    """Collect a bounded sample and check source/metadata correspondence."""

    def __init__(self):
        super().__init__("validate_oak_diagnostics")
        self.started = time.monotonic()
        self.counts = defaultdict(int)
        self.source_stamps = defaultdict(set)
        self.diagnostic_stamps = defaultdict(set)
        self.first_values = {}
        self.last_values = {}
        self.sequence_numbers = defaultdict(list)
        self.missing_keys = defaultdict(set)
        self.status_levels = defaultdict(lambda: defaultdict(int))
        self.status_messages = defaultdict(set)
        self.left_device_by_sequence = {}
        self.right_device_by_sequence = {}
        self.imu_deltas_ns = []

        for name, (message_type, topic) in SOURCE_TOPICS.items():
            self.create_subscription(
                message_type,
                topic,
                lambda message, stream=name: self.source_callback(
                    stream, message
                ),
                200,
            )
        for name, topic in DIAGNOSTIC_TOPICS.items():
            self.create_subscription(
                DiagnosticArray,
                topic,
                lambda message, stream=name: self.diagnostic_callback(
                    stream, message
                ),
                200,
            )

    def source_callback(self, stream, message):
        """Record source message stamps without retaining payloads."""
        self.counts["source_" + stream] += 1
        self.source_stamps[stream].add(stamp_ns(message.header))

    def diagnostic_callback(self, stream, message):
        """Record diagnostics and validate their stable schema."""
        self.counts["diagnostic_" + stream] += 1
        values = values_dict(message)
        if message.status:
            level = message.status[0].level
            if isinstance(level, bytes):
                level = int.from_bytes(level, byteorder="little")
            self.status_levels[stream][str(int(level))] += 1
            self.status_messages[stream].add(message.status[0].message)
        self.first_values.setdefault(stream, values)
        self.last_values[stream] = values
        required = DEVICE_KEYS if stream == "device" else (
            IMU_KEYS if stream == "imu" else FRAME_KEYS
        )
        self.missing_keys[stream].update(required - set(values))
        if stream != "device":
            self.diagnostic_stamps[stream].add(stamp_ns(message.header))
        if stream in ("left", "right", "color", "depth") and values:
            sequence = int(values["sequence_num"])
            self.sequence_numbers[stream].append(sequence)
        if stream in ("left", "right") and values:
            sequence = int(values["sequence_num"])
            device_time = int(values["device_timestamp_ns"])
            destination = (
                self.left_device_by_sequence
                if stream == "left" else self.right_device_by_sequence
            )
            destination[sequence] = device_time
        if stream == "imu" and values:
            self.sequence_numbers["imu_accelerometer"].append(
                int(values["accelerometer_sequence_num"])
            )
            self.sequence_numbers["imu_gyroscope"].append(
                int(values["gyroscope_sequence_num"])
            )
            self.imu_deltas_ns.append(
                int(values["accel_to_gyro_device_delta_ns"])
            )

    def report(self):
        """Build a JSON-serializable validation report and failure list."""
        elapsed = max(time.monotonic() - self.started, 1e-9)
        failures = []
        streams = list(SOURCE_TOPICS)
        correspondence = {}
        rates = {}
        for stream in streams:
            source_count = self.counts["source_" + stream]
            diagnostic_count = self.counts["diagnostic_" + stream]
            rates[stream] = {
                "source_hz": source_count / elapsed,
                "diagnostic_hz": diagnostic_count / elapsed,
                "source_count": source_count,
                "diagnostic_count": diagnostic_count,
            }
            if source_count == 0 or diagnostic_count == 0:
                failures.append("missing source or diagnostic: " + stream)
            source_stamps = self.source_stamps[stream]
            diagnostic_stamps = self.diagnostic_stamps[stream]
            matches = len(source_stamps & diagnostic_stamps)
            correspondence[stream] = {
                "matching_unique_header_stamps": matches,
                "source_unique_header_stamps": len(source_stamps),
                "diagnostic_unique_header_stamps": len(diagnostic_stamps),
            }
            if source_stamps and matches == 0:
                failures.append("no matching header stamps: " + stream)
        if self.counts["diagnostic_device"] == 0:
            failures.append("missing device info")
        for stream, keys in self.missing_keys.items():
            if keys:
                failures.append(
                    "missing keys {}: {}".format(stream, sorted(keys))
                )

        common_sequences = (
            set(self.left_device_by_sequence)
            & set(self.right_device_by_sequence)
        )
        stereo_deltas = [
            self.right_device_by_sequence[sequence]
            - self.left_device_by_sequence[sequence]
            for sequence in common_sequences
        ]

        def distribution(values):
            if not values:
                return None
            return {
                "count": len(values),
                "min": min(values),
                "median": statistics.median(values),
                "max": max(values),
            }

        sequence_gaps = {}
        sequence_observations = {}
        for stream in streams:
            values = self.last_values.get(stream, {})
            first_values = self.first_values.get(stream, {})
            if stream == "imu":
                sequence_gaps[stream] = {
                    "accelerometer_cumulative": int(values.get(
                        "cumulative_accelerometer_sequence_gap", -1
                    )),
                    "gyroscope_cumulative": int(values.get(
                        "cumulative_gyroscope_sequence_gap", -1
                    )),
                    "accelerometer_during_test": int(values.get(
                        "cumulative_accelerometer_sequence_gap", -1
                    )) - int(first_values.get(
                        "cumulative_accelerometer_sequence_gap", -1
                    )),
                    "gyroscope_during_test": int(values.get(
                        "cumulative_gyroscope_sequence_gap", -1
                    )) - int(first_values.get(
                        "cumulative_gyroscope_sequence_gap", -1
                    )),
                }
            else:
                sequence_gaps[stream] = int(values.get(
                    "cumulative_sequence_gap", -1
                ))
                sequence_gaps[stream + "_during_test"] = (
                    sequence_gaps[stream]
                    - int(first_values.get("cumulative_sequence_gap", -1))
                )

        for stream, values in self.sequence_numbers.items():
            steps = [
                (current - previous) & 0xFFFFFFFF
                for previous, current in zip(values, values[1:])
            ]
            sequence_observations[stream] = {
                "first": values[0] if values else None,
                "last": values[-1] if values else None,
                "step": distribution(steps),
            }

        report = {
            "duration_s": elapsed,
            "rates": rates,
            "header_correspondence": correspondence,
            "sequence_gaps": sequence_gaps,
            "sequence_observations": sequence_observations,
            "stereo_device_delta_ns": distribution(stereo_deltas),
            "accel_to_gyro_device_delta_ns": distribution(
                self.imu_deltas_ns
            ),
            "device_info": self.last_values.get("device"),
            "diagnostic_status": {
                stream: {
                    "levels": dict(levels),
                    "messages": sorted(self.status_messages[stream]),
                }
                for stream, levels in self.status_levels.items()
            },
            "failures": failures,
        }
        return report, failures


def main():
    """Run the bounded live validation."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output")
    args = parser.parse_args()

    rclpy.init()
    node = DiagnosticValidator()
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    report, failures = node.report()
    node.destroy_node()
    rclpy.shutdown()
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
