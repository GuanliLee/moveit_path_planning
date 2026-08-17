#!/usr/bin/env python3
"""Build a persistent capture-health summary from recorder topic counters."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Any


def is_rgb_camera_topic(topic: str) -> bool:
    return topic.startswith("/camera_") and topic.endswith("/color/image_raw")


def build_capture_health(
    *,
    topic_counts: Mapping[str, int],
    topic_min_rates: Mapping[str, float],
    duration_seconds: float,
    rate_failed_topics: Iterable[str],
    min_coverage_ratio: float = 0.8,
    topic_max_gaps_seconds: Mapping[str, float] | None = None,
    max_camera_gap_seconds: float = 0.05,
) -> dict[str, Any]:
    """Summarize coverage, rate latches, and burst camera frame loss."""
    duration = max(float(duration_seconds), 0.0)
    rate_failed = set(rate_failed_topics)
    maximum_gaps = topic_max_gaps_seconds or {}
    allowed_camera_gap = max(float(max_camera_gap_seconds), 0.0)
    topics: dict[str, dict[str, Any]] = {}
    failed_topics: list[str] = []

    for topic in sorted(topic_min_rates):
        count = max(int(topic_counts.get(topic, 0)), 0)
        min_rate = max(float(topic_min_rates.get(topic, 0.0)), 0.0)
        expected_count = min_rate * duration
        if expected_count > 0:
            coverage_ratio = count / expected_count
            coverage_failed = coverage_ratio < min_coverage_ratio
        else:
            coverage_ratio = None
            coverage_failed = count == 0
        topic_rate_failed = topic in rate_failed
        maximum_gap = max(float(maximum_gaps.get(topic, 0.0)), 0.0)
        camera_topic = is_rgb_camera_topic(topic)
        gap_failed = camera_topic and maximum_gap > allowed_camera_gap
        failed = coverage_failed or topic_rate_failed or gap_failed
        if failed:
            failed_topics.append(topic)
        topics[topic] = {
            "count": count,
            "minimum_hz": min_rate,
            "actual_hz": count / duration if duration > 0 else 0.0,
            "coverage_ratio": coverage_ratio,
            "coverage_failed": coverage_failed,
            "rate_failed": topic_rate_failed,
            "max_gap_seconds": maximum_gap,
            "max_allowed_gap_seconds": allowed_camera_gap if camera_topic else None,
            "gap_failed": gap_failed,
            "failed": failed,
        }

    failed_camera_topics = [topic for topic in failed_topics if is_rgb_camera_topic(topic)]
    camera_topics = [topic for topic in sorted(topic_min_rates) if is_rgb_camera_topic(topic)]
    return {
        "version": 2,
        "duration_seconds": duration,
        "minimum_coverage_ratio": float(min_coverage_ratio),
        "max_camera_gap_seconds": allowed_camera_gap,
        "camera_ok": bool(camera_topics) and not failed_camera_topics,
        "camera_topics": camera_topics,
        "failed_camera_topics": failed_camera_topics,
        "failed_topics": failed_topics,
        "topics": topics,
    }
