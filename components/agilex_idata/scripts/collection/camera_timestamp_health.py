#!/usr/bin/env python3
"""Fail-closed preflight probe for RGB camera payload timestamps."""

import argparse
import json
import math
import statistics
import sys
import time
from collections import Counter, defaultdict
from typing import Iterable


CAMERA_TOPICS = (
    "/camera_l/color/image_raw/compressed",
    "/camera_f/color/image_raw/compressed",
    "/camera_r/color/image_raw/compressed",
    "/camera_h/color/image_raw/compressed",
)


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    fraction = position - lower
    return float(ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction)


def evaluate_camera_timestamps(
    samples: Iterable[tuple[str, int, int]],
    *,
    max_age_ms: float = 70.0,
    max_spread_ms: float = 60.0,
    min_samples_per_topic: int = 30,
) -> dict:
    grouped: dict[str, list[float]] = defaultdict(list)
    for topic, arrival_ns, header_ns in samples:
        if topic in CAMERA_TOPICS:
            grouped[topic].append((int(arrival_ns) - int(header_ns)) / 1_000_000.0)

    topic_results = {}
    medians = []
    for topic in CAMERA_TOPICS:
        ages = grouped.get(topic, [])
        sample_failed = len(ages) < min_samples_per_topic
        median_age = float(statistics.median(ages)) if ages else None
        p90_absolute_age = percentile([abs(value) for value in ages], 0.90) if ages else None
        age_failed = bool(
            ages
            and (
                abs(float(median_age)) > float(max_age_ms)
                or float(p90_absolute_age) > float(max_age_ms)
            )
        )
        if median_age is not None:
            medians.append(median_age)
        topic_results[topic] = {
            "samples": len(ages),
            "median_age_ms": median_age,
            "p90_absolute_age_ms": p90_absolute_age,
            "max_allowed_age_ms": float(max_age_ms),
            "sample_failed": sample_failed,
            "age_failed": age_failed,
            "failed": sample_failed or age_failed,
        }

    spread = max(medians) - min(medians) if len(medians) == len(CAMERA_TOPICS) else None
    spread_failed = spread is None or spread > float(max_spread_ms)
    return {
        "schema_version": 1,
        "ok": not spread_failed and not any(item["failed"] for item in topic_results.values()),
        "max_age_ms": float(max_age_ms),
        "max_spread_ms": float(max_spread_ms),
        "minimum_samples_per_topic": int(min_samples_per_topic),
        "median_age_spread_ms": spread,
        "spread_failed": spread_failed,
        "topics": topic_results,
    }


def _has_minimum_samples(
    samples: Iterable[tuple[str, int, int]],
    minimum: int,
) -> bool:
    counts = Counter(topic for topic, _arrival_ns, _header_ns in samples)
    return all(counts[topic] >= minimum for topic in CAMERA_TOPICS)


def collect_samples(
    sample_seconds: float,
    *,
    min_samples_per_topic: int = 30,
    discovery_timeout_seconds: float = 3.0,
) -> list[tuple[str, int, int]]:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    class CameraTimestampProbe(Node):
        def __init__(self):
            super().__init__("camera_timestamp_health")
            self.samples: list[tuple[str, int, int]] = []
            self.subscriptions_ = []
            for topic in CAMERA_TOPICS:
                self.subscriptions_.append(
                    self.create_subscription(
                        CompressedImage,
                        topic,
                        lambda message, topic_name=topic: self.on_message(topic_name, message),
                        qos_profile_sensor_data,
                    )
                )

        def on_message(self, topic: str, message: CompressedImage) -> None:
            arrival_ns = self.get_clock().now().nanoseconds
            header_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            self.samples.append((topic, arrival_ns, header_ns))

    rclpy.init()
    node = CameraTimestampProbe()
    try:
        # A newly-created DDS participant can spend most of a short sampling
        # window discovering the four camera publishers. Warm all
        # subscriptions first so discovery latency is not mistaken for camera
        # frame loss.
        discovery_deadline = time.monotonic() + max(
            float(discovery_timeout_seconds),
            0.1,
        )
        while rclpy.ok() and time.monotonic() < discovery_deadline:
            if _has_minimum_samples(node.samples, 1):
                break
            rclpy.spin_once(node, timeout_sec=0.05)

        node.samples.clear()
        sample_deadline = time.monotonic() + max(float(sample_seconds), 0.1)
        minimum = max(1, int(min_samples_per_topic))
        while rclpy.ok() and time.monotonic() < sample_deadline:
            if _has_minimum_samples(node.samples, minimum):
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        return node.samples
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample-seconds", type=float, default=3.0)
    parser.add_argument("--discovery-timeout-seconds", type=float, default=3.0)
    parser.add_argument("--min-samples-per-topic", type=int, default=30)
    parser.add_argument("--max-age-ms", type=float, default=70.0)
    parser.add_argument("--max-spread-ms", type=float, default=60.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    result = evaluate_camera_timestamps(
        collect_samples(
            args.sample_seconds,
            min_samples_per_topic=args.min_samples_per_topic,
            discovery_timeout_seconds=args.discovery_timeout_seconds,
        ),
        max_age_ms=args.max_age_ms,
        max_spread_ms=args.max_spread_ms,
        min_samples_per_topic=args.min_samples_per_topic,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if result["ok"]:
        print(
            "相机时间戳自检通过："
            f"中位 age 跨相机差 {result['median_age_spread_ms']:.3f} ms"
        )
        return 0
    for topic, detail in result["topics"].items():
        if detail["sample_failed"]:
            print(
                f"相机时间戳自检失败：{topic} 仅收到 {detail['samples']} 帧，"
                f"至少需要 {result['minimum_samples_per_topic']} 帧",
                file=sys.stderr,
            )
        if detail["age_failed"]:
            print(
                f"相机时间戳自检失败：{topic} median={detail['median_age_ms']:.3f} ms, "
                f"P90(abs)={detail['p90_absolute_age_ms']:.3f} ms，阈值={args.max_age_ms:g} ms",
                file=sys.stderr,
            )
    if result["spread_failed"]:
        print(
            "相机时间戳自检失败：中位 age 跨相机差 "
            f"{result['median_age_spread_ms']} ms，阈值={args.max_spread_ms:g} ms",
            file=sys.stderr,
        )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
