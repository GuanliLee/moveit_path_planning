"""Cross-platform LeRobot discovery, consistency analysis, and batch review.

The module deliberately has no dependency on the MCAP/HDF5 QC pipeline.  It
reads LeRobot v2 datasets, compares simulation and real-world conventions, and
can build a reviewed copy after the operator has queued all grade/exclusion
changes.  Source datasets are never modified by the review operation.
"""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import unicodedata
import uuid
from typing import Any, Callable


PLATFORMS = {"simulation": "仿真", "real": "真机"}
QUALITY_GRADES = ("A", "B", "C", "F")
CROSS_PLATFORM_GRADE_DIRS = ("A", "B", "F")
LIFT_EXPECTED_MM = 200.0
LIFT_TOLERANCE_MM = 0.5
GRIPPER_EXPECTED_LOW = 0.0
GRIPPER_EXPECTED_HIGH = 0.1
GRIPPER_BINARY_LEVEL_TOLERANCE = 0.01
GRIPPER_CONTINUOUS_MIN = 0.0
GRIPPER_CONTINUOUS_MAX = 0.1
GRIPPER_CONTINUOUS_SOFT_MIN = -0.05
GRIPPER_CONTINUOUS_SOFT_MAX = 0.15
MAX_DISCOVERED_DATASETS = 500
MAX_DISCOVERY_DEPTH = 12

ACTION_LAYOUT = [
    *(f"left_joint_{index}" for index in range(1, 7)),
    "left_gripper",
    *(f"right_joint_{index}" for index in range(1, 7)),
    "right_gripper",
    "lift_mm",
    "base_vx",
    "base_vy",
    "base_wz",
]
STATE_LAYOUT = [
    *(f"left_joint_{index}" for index in range(1, 7)),
    "left_gripper",
    *(f"right_joint_{index}" for index in range(1, 7)),
    "right_gripper",
    "lift_mm",
    "base_x",
    "base_y",
    "base_yaw",
    "base_vx",
    "base_vy",
    "base_wz",
]
VECTOR_LAYOUTS = {"action": ACTION_LAYOUT, "state": STATE_LAYOUT}
VECTOR_DIMS = {"action": 18, "state": 21}
JOINT_POSITION_ALIASES = {
    1: ("joint_1", "waist"),
    2: ("joint_2", "shoulder"),
    3: ("joint_3", "elbow"),
    4: ("joint_4", "forearm_roll"),
    5: ("joint_5", "wrist_angle"),
    6: ("joint_6", "wrist_rotate"),
}
GRIPPER_DIMS = {
    ("state", "left"): 6,
    ("state", "right"): 13,
    ("action", "left"): 6,
    ("action", "right"): 13,
}

KNOWN_ITEMS = (
    "Coca-Cola",
    "Daily C Grape Juice",
    "Guangming Probiotic Milk",
    "Daily C Orange Juice",
    "AD Calcium Milk",
    "Robuk Velvet Latte",
    "Aojiru",
    "HK Orange Fanta",
    "Taro Milk",
    "Yili Peach Yogurt",
    "NEVER Coconut Latte",
    "Yili Strawberry Yogurt",
    "Wanglaoji",
    "Sprite",
    "Yakult",
    "Dahongpao Milk Tea",
)


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (FileNotFoundError, OSError):
        return rows
    for line in lines:
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            rows.append(value)
    return rows


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows)
    path.write_text(text, encoding="utf-8")


def natural_key(value: str) -> list[Any]:
    return [int(part) if part.isdigit() else part.lower() for part in re.split(r"(\d+)", value)]


def as_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def is_lerobot_dataset(path: Path) -> bool:
    return bool(
        path.is_dir()
        and (
            (path / "meta" / "info.json").is_file()
            or (path / "meta" / "tasks.jsonl").is_file()
            or next(path.glob("data/chunk-*/episode_*.parquet"), None) is not None
        )
    )


def dataset_id(path: Path) -> str:
    return hashlib.sha256(str(path.resolve()).encode("utf-8")).hexdigest()[:16]


def _safe_children(path: Path) -> list[Path]:
    try:
        return sorted((child for child in path.iterdir() if child.is_dir()), key=lambda item: natural_key(item.name))
    except OSError:
        return []


def discover_under(root: Path, recursive: bool) -> list[Path]:
    root = root.expanduser().resolve()
    if is_lerobot_dataset(root):
        return [root]
    if not root.is_dir():
        return []
    found: list[Path] = []
    pending: list[tuple[Path, int]] = [(root, 0)]
    while pending and len(found) < MAX_DISCOVERED_DATASETS:
        current, depth = pending.pop()
        if current != root and is_lerobot_dataset(current):
            found.append(current)
            continue
        if not recursive and depth >= 1:
            continue
        if depth >= MAX_DISCOVERY_DEPTH:
            continue
        for child in reversed(_safe_children(current)):
            pending.append((child, depth + 1))
    return sorted(set(found), key=lambda item: natural_key(str(item)))


def _validate_platform(value: Any) -> str:
    platform = str(value or "").strip().lower()
    if platform not in PLATFORMS:
        raise ValueError(f"unsupported platform type: {value!r}")
    return platform


def _feature_dim(feature: Any) -> int | None:
    if not isinstance(feature, dict):
        return None
    shape = feature.get("shape")
    if not isinstance(shape, list) or not shape:
        return None
    return as_int(shape[-1])


def _feature_names(feature: Any, dim: int | None) -> list[str]:
    if not isinstance(feature, dict) or not dim:
        return []
    names = feature.get("names")
    while isinstance(names, list) and len(names) == 1 and isinstance(names[0], list):
        names = names[0]
    if not isinstance(names, list) or len(names) != dim:
        return []
    values = [str(item).strip() for item in names]
    return values if all(values) else []


def vector_feature_keys(info: dict[str, Any]) -> tuple[str, str]:
    features = info.get("features") if isinstance(info.get("features"), dict) else {}
    state_candidates = ("observation.state", "state")
    action_candidates = ("action", "actions")
    state_key = next((key for key in state_candidates if key in features), "")
    action_key = next((key for key in action_candidates if key in features), "")
    if not state_key:
        state_key = next((str(key) for key in features if "state" in str(key).lower()), "")
    if not action_key:
        action_key = next((str(key) for key in features if "action" in str(key).lower()), "")
    return state_key, action_key


def video_feature_keys(info: dict[str, Any]) -> list[str]:
    features = info.get("features") if isinstance(info.get("features"), dict) else {}
    return sorted(
        str(key)
        for key, feature in features.items()
        if isinstance(feature, dict) and str(feature.get("dtype") or "").lower() == "video"
    )


def task_rows(path: Path) -> list[dict[str, Any]]:
    return load_jsonl(path / "meta" / "tasks.jsonl")


def episode_rows(path: Path) -> list[dict[str, Any]]:
    return load_jsonl(path / "meta" / "episodes.jsonl")


def _episode_indices_from_data(path: Path) -> list[int]:
    indices: list[int] = []
    for parquet in path.glob("data/chunk-*/episode_*.parquet"):
        match = re.search(r"episode_(\d+)\.parquet$", parquet.name)
        if match:
            indices.append(int(match.group(1)))
    return sorted(set(indices))


def describe_dataset(path: Path, platform: str) -> dict[str, Any]:
    path = path.expanduser().resolve()
    platform = _validate_platform(platform)
    if not is_lerobot_dataset(path):
        raise ValueError(f"not a LeRobot dataset: {path}")
    info = load_json(path / "meta" / "info.json")
    features = info.get("features") if isinstance(info.get("features"), dict) else {}
    state_key, action_key = vector_feature_keys(info)
    state_dim = _feature_dim(features.get(state_key))
    action_dim = _feature_dim(features.get(action_key))
    tasks = [str(row.get("task") or "").strip() for row in task_rows(path)]
    tasks = list(dict.fromkeys(task for task in tasks if task))
    episodes = episode_rows(path)
    total_episodes = as_int(info.get("total_episodes"))
    if total_episodes is None:
        total_episodes = len(episodes) or len(_episode_indices_from_data(path))
    issues: list[str] = []
    if state_dim != 21:
        issues.append(f"state 维度为 {state_dim if state_dim is not None else '未定义'}，期望 21")
    if action_dim != 18:
        issues.append(f"action 维度为 {action_dim if action_dim is not None else '未定义'}，期望 18")
    state_names = _feature_names(features.get(state_key), state_dim)
    action_names = _feature_names(features.get(action_key), action_dim)
    source_grade = path.name.upper() if path.name.upper() in CROSS_PLATFORM_GRADE_DIRS else ""
    grade_group_path = path.parent if source_grade else path
    return {
        "id": dataset_id(path),
        "name": path.name,
        "path": str(path),
        "platform": platform,
        "platform_label": PLATFORMS[platform],
        "robot_type": str(info.get("robot_type") or ""),
        "codebase_version": str(info.get("codebase_version") or ""),
        "fps": info.get("fps"),
        "total_episodes": total_episodes,
        "total_frames": as_int(info.get("total_frames")),
        "state_key": state_key,
        "action_key": action_key,
        "state_dim": state_dim,
        "action_dim": action_dim,
        "state_names": state_names,
        "action_names": action_names,
        "video_keys": video_feature_keys(info),
        "tasks": tasks,
        "issues": issues,
        "source_grade": source_grade,
        "grade_group_path": str(grade_group_path),
        "grade_group_name": grade_group_path.name,
        "default_output_path": str(path.with_name(f"{path.name}_reviewed")),
    }


def discover_datasets(sources: list[Any]) -> dict[str, Any]:
    datasets: dict[Path, dict[str, Any]] = {}
    source_results: list[dict[str, Any]] = []
    for raw in sources:
        if not isinstance(raw, dict):
            continue
        text = str(raw.get("path") or "").strip()
        if not text:
            continue
        platform = _validate_platform(raw.get("platform"))
        recursive = bool(raw.get("recursive", True))
        root = Path(text).expanduser().resolve()
        matches = discover_under(root, recursive)
        expanded_matches = set(matches)
        for path in matches:
            if path.name.upper() not in CROSS_PLATFORM_GRADE_DIRS:
                continue
            for grade in CROSS_PLATFORM_GRADE_DIRS:
                sibling = path.parent / grade
                if is_lerobot_dataset(sibling):
                    expanded_matches.add(sibling.resolve())
        matches = sorted(expanded_matches, key=lambda item: natural_key(str(item)))
        source_result = {
            "path": str(root),
            "platform": platform,
            "recursive": recursive,
            "exists": root.exists(),
            "dataset_count": len(matches),
            "error": "" if root.exists() else f"目录不存在: {root}",
        }
        source_results.append(source_result)
        for path in matches:
            existing = datasets.get(path)
            if existing and existing["platform"] != platform:
                existing.setdefault("issues", []).append("同一路径被同时标记为仿真和真机，请在列表中确认")
                continue
            try:
                datasets[path] = describe_dataset(path, platform)
            except Exception as exc:
                source_result["error"] = str(exc)
    ordered = sorted(datasets.values(), key=lambda item: (item["platform"], natural_key(item["path"])))
    return {
        "sources": source_results,
        "datasets": ordered,
        "summary": {
            "total": len(ordered),
            "simulation": sum(item["platform"] == "simulation" for item in ordered),
            "real": sum(item["platform"] == "real" for item in ordered),
        },
    }


def _choose_evenly(values: list[Any], limit: int) -> list[Any]:
    if limit <= 0 or len(values) <= limit:
        return values
    if limit == 1:
        return [values[0]]
    indices = {round(index * (len(values) - 1) / (limit - 1)) for index in range(limit)}
    return [values[index] for index in sorted(indices)]


def _matrix_from_values(values: Any, expected_dim: int | None) -> tuple[Any, int]:
    import numpy as np  # type: ignore

    rows: list[Any] = []
    bad = 0
    for value in values:
        try:
            array = np.asarray(value, dtype=float).reshape(-1)
        except (TypeError, ValueError):
            bad += 1
            continue
        if expected_dim is not None and len(array) != expected_dim:
            bad += 1
            continue
        rows.append(array)
    if not rows:
        return np.empty((0, expected_dim or 0), dtype=float), bad
    try:
        return np.vstack(rows), bad
    except ValueError:
        return np.empty((0, expected_dim or 0), dtype=float), bad + len(rows)


def _dimension_stats(values: Any) -> dict[str, Any]:
    import numpy as np  # type: ignore

    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {"count": 0}
    quantiles = np.quantile(finite, [0.01, 0.05, 0.5, 0.95, 0.99])
    return {
        "count": int(finite.size),
        "min": round(float(np.min(finite)), 8),
        "p01": round(float(quantiles[0]), 8),
        "p05": round(float(quantiles[1]), 8),
        "median": round(float(quantiles[2]), 8),
        "p95": round(float(quantiles[3]), 8),
        "p99": round(float(quantiles[4]), 8),
        "max": round(float(np.max(finite)), 8),
        "mean": round(float(np.mean(finite)), 8),
        "std": round(float(np.std(finite)), 8),
        "unique_rounded": int(min(10000, np.unique(np.round(finite, 6)).size)),
    }


def _read_dataset_samples(descriptor: dict[str, Any], max_episodes: int, max_frames: int) -> tuple[dict[str, Any], dict[str, Any]]:
    import numpy as np  # type: ignore
    import pandas as pd  # type: ignore

    path = Path(descriptor["path"])
    parquets = sorted(path.glob("data/chunk-*/episode_*.parquet"), key=lambda item: natural_key(str(item)))
    selected = _choose_evenly(parquets, max_episodes)
    matrices: dict[str, list[Any]] = {"state": [], "action": []}
    bad_rows = {"state": 0, "action": 0}
    sampled_frames = 0
    errors: list[str] = []
    for parquet in selected:
        try:
            frame = pd.read_parquet(parquet)
        except Exception as exc:
            errors.append(f"{parquet.name}: {exc}")
            continue
        if max_frames > 0 and len(frame) > max_frames:
            positions = sorted({round(index * (len(frame) - 1) / (max_frames - 1)) for index in range(max_frames)})
            frame = frame.iloc[positions]
        sampled_frames += len(frame)
        for vector, key_name in (("state", "state_key"), ("action", "action_key")):
            key = str(descriptor.get(key_name) or "")
            if not key or key not in frame.columns:
                bad_rows[vector] += len(frame)
                continue
            matrix, bad = _matrix_from_values(frame[key].tolist(), descriptor.get(f"{vector}_dim"))
            bad_rows[vector] += bad
            if matrix.size:
                matrices[vector].append(matrix)
    arrays: dict[str, Any] = {}
    vector_stats: dict[str, list[dict[str, Any]]] = {}
    for vector in ("state", "action"):
        dim = descriptor.get(f"{vector}_dim") or 0
        arrays[vector] = np.concatenate(matrices[vector], axis=0) if matrices[vector] else np.empty((0, dim))
        vector_stats[vector] = [
            _dimension_stats(arrays[vector][:, index])
            for index in range(arrays[vector].shape[1])
        ]
    public = {
        "sampled_episodes": len(selected),
        "sampled_frames": sampled_frames,
        "bad_vector_rows": bad_rows,
        "sample_errors": errors[:20],
        "vector_stats": vector_stats,
    }
    return public, arrays


def _normalise_name(value: str) -> str:
    text = unicodedata.normalize("NFKC", str(value or "")).strip().lower()
    return re.sub(r"[^a-z0-9]+", "", text)


def _dimension_name_aliases(canonical: str) -> set[str]:
    aliases = {canonical}
    match = re.fullmatch(r"(left|right)_joint_(\d+)", canonical)
    if match:
        side, index_text = match.groups()
        for suffix in JOINT_POSITION_ALIASES.get(int(index_text), ()):
            aliases.add(f"{side}_{suffix}")
            aliases.add(f"{side}_arm_{suffix}")
    elif canonical == "lift_mm":
        aliases.update({"lift", "lift_height", "waist_height", "elevator", "elevator_height"})
    elif canonical.endswith("_gripper"):
        side = canonical.split("_", 1)[0]
        aliases.update({f"{side}_gripper_position", f"{side}_gripper_joint"})
    elif canonical.startswith("base_"):
        suffix = canonical.removeprefix("base_")
        aliases.add(suffix)
        if suffix in {"vx", "vy"}:
            axis = suffix[-1]
            aliases.update({f"base_velocity_{axis}", f"base_linear_velocity_{axis}"})
        elif suffix == "wz":
            aliases.update({"base_velocity_yaw", "base_angular_velocity_z", "angular_velocity_z"})
        elif suffix == "yaw":
            aliases.update({"base_theta", "base_heading"})
    return {_normalise_name(alias) for alias in aliases}


def _definition_status(descriptors: list[dict[str, Any]], vector: str, index: int, canonical: str) -> tuple[str, list[str]]:
    explicit: list[tuple[str, str]] = []
    for item in descriptors:
        names = item.get(f"{vector}_names")
        if isinstance(names, list) and index < len(names):
            explicit.append((item["name"], str(names[index])))
    accepted_names = _dimension_name_aliases(canonical)
    misplaced = [
        (dataset, name)
        for dataset, name in explicit
        if _normalise_name(name) not in accepted_names
    ]
    if misplaced:
        return "fail", [
            f"第 {index} 维应为 {canonical}，但显式 metadata 中的位置不正确: "
            + "；".join(f"{dataset}={name}" for dataset, name in misplaced)
        ]
    return "pass", []


def _aggregate_dimension(arrays: list[Any], index: int) -> dict[str, Any]:
    import numpy as np  # type: ignore

    values = [array[:, index] for array in arrays if getattr(array, "ndim", 0) == 2 and array.shape[1] > index]
    return _dimension_stats(np.concatenate(values)) if values else {"count": 0}


def compare_ranges(simulation: dict[str, Any], real: dict[str, Any]) -> tuple[str, list[str], dict[str, Any]]:
    if not simulation.get("count") or not real.get("count"):
        return "warn", ["缺少仿真或真机采样，无法进行跨平台数值比较"], {}
    sim_low, sim_high = float(simulation["p01"]), float(simulation["p99"])
    real_low, real_high = float(real["p01"]), float(real["p99"])
    sim_raw_width = max(sim_high - sim_low, 0.0)
    real_raw_width = max(real_high - real_low, 0.0)
    constant_epsilon = 1e-8
    if sim_raw_width <= constant_epsilon and real_raw_width <= constant_epsilon:
        sim_value = (sim_low + sim_high) / 2
        real_value = (real_low + real_high) / 2
        same_value = math.isclose(sim_value, real_value, rel_tol=1e-6, abs_tol=constant_epsilon)
        metrics = {
            "scale_ratio": 1.0,
            "overlap_ratio": 1.0 if same_value else 0.0,
            "center_gap_normalized": 0.0 if same_value else None,
            "constant_values": {"simulation": round(sim_value, 8), "real": round(real_value, 8)},
        }
        if same_value:
            return "pass", [], metrics
        return "warn", [
            f"仿真和真机均为常量，但数值不同：仿真 {sim_value:g}，真机 {real_value:g}"
        ], metrics

    sim_width = max(sim_raw_width, 1e-9)
    real_width = max(real_raw_width, 1e-9)
    scale_ratio = max(sim_width, real_width) / max(min(sim_width, real_width), 1e-9)
    intersection = max(0.0, min(sim_high, real_high) - max(sim_low, real_low))
    overlap_ratio = intersection / max(min(sim_width, real_width), 1e-9)
    center_gap = abs((sim_low + sim_high) / 2 - (real_low + real_high) / 2)
    center_gap_normalized = center_gap / max(sim_width, real_width, 1e-9)
    metrics = {
        "scale_ratio": round(scale_ratio, 4),
        "overlap_ratio": round(overlap_ratio, 4),
        "center_gap_normalized": round(center_gap_normalized, 4),
    }
    warnings: list[str] = []
    if scale_ratio > 10:
        warnings.append(f"稳健范围宽度相差 {scale_ratio:.2f} 倍")
    elif scale_ratio > 4:
        warnings.append(f"稳健范围宽度相差 {scale_ratio:.2f} 倍")
    if overlap_ratio < 0.2:
        warnings.append(f"仿真/真机范围重叠率仅 {overlap_ratio * 100:.1f}%")
    if center_gap_normalized > 1.5:
        warnings.append(f"范围中心偏移达到 {center_gap_normalized:.2f} 个范围宽度")
    severe = scale_ratio > 10 or (overlap_ratio == 0 and center_gap_normalized > 2.5)
    return ("fail" if severe else "warn" if warnings else "pass"), warnings, metrics


def _item_names_from_episodes(path: Path) -> list[str]:
    names = list(KNOWN_ITEMS)
    for episode in episode_rows(path):
        items = episode.get("items")
        if not isinstance(items, list):
            continue
        for item in items:
            if not isinstance(item, dict):
                continue
            for key in ("product_en", "product_cn", "name"):
                value = str(item.get(key) or "").strip()
                if value:
                    names.append(value)
    return list(dict.fromkeys(names))


def normalise_prompt(prompt: str, item_names: list[str]) -> str:
    text = unicodedata.normalize("NFKC", str(prompt or "")).strip().lower()
    text = re.sub(r"\b(left|right)[ -]?hand\b", "{hand}", text)
    text = re.sub(r"\b(left|right)\b", "{hand}", text)
    for name in sorted(set(item_names), key=len, reverse=True):
        if name.strip():
            text = re.sub(re.escape(name.lower()), "{item}", text, flags=re.IGNORECASE)
    text = re.sub(
        r"\b(grasp|grab|pick up|pickup)\s+(.+?)\s+(with|using)\s+\{hand\}",
        lambda match: f"{match.group(1)} {{item}} {match.group(3)} {{hand}}",
        text,
    )
    text = re.sub(
        r"\b(place|put)\s+(.+?)\s+(into|in|on|onto)\b",
        lambda match: f"{match.group(1)} {{item}} {match.group(3)}",
        text,
    )
    text = re.sub(r"\s+", " ", text).strip()
    return text


def _prompt_report(descriptors: list[dict[str, Any]]) -> dict[str, Any]:
    template_sources: dict[str, dict[str, Any]] = {}
    platform_templates: dict[str, set[str]] = {key: set() for key in PLATFORMS}
    dataset_rows: list[dict[str, Any]] = []
    for descriptor in descriptors:
        path = Path(descriptor["path"])
        tasks = list(descriptor.get("tasks") or [])
        if not tasks:
            for episode in episode_rows(path):
                raw = episode.get("full_instructions_en") or episode.get("tasks")
                if isinstance(raw, list):
                    tasks.extend(str(value) for value in raw if str(value).strip())
                elif episode.get("task"):
                    tasks.append(str(episode["task"]))
        tasks = list(dict.fromkeys(task.strip() for task in tasks if task.strip()))
        item_names = _item_names_from_episodes(path)
        templates = sorted({normalise_prompt(task, item_names) for task in tasks if task})
        platform_templates[descriptor["platform"]].update(templates)
        dataset_rows.append({
            "dataset_id": descriptor["id"],
            "dataset": descriptor["name"],
            "platform": descriptor["platform"],
            "task_count": len(tasks),
            "template_count": len(templates),
            "tasks": tasks,
            "templates": templates,
        })
        for task in tasks:
            template = normalise_prompt(task, item_names)
            entry = template_sources.setdefault(template, {"template": template, "simulation": [], "real": []})
            entry[descriptor["platform"]].append({"dataset": descriptor["name"], "task": task})
    sim = platform_templates["simulation"]
    real = platform_templates["real"]
    only_sim = sorted(sim - real)
    only_real = sorted(real - sim)
    if not sim or not real:
        status = "warn"
        warnings = ["需要同时选择至少一个仿真数据集和一个真机数据集"]
    elif only_sim or only_real:
        status = "fail"
        warnings = [f"仅仿真存在 {len(only_sim)} 种模板，仅真机存在 {len(only_real)} 种模板"]
    else:
        status = "pass"
        warnings = []
    return {
        "status": status,
        "warnings": warnings,
        "simulation_templates": sorted(sim),
        "real_templates": sorted(real),
        "only_simulation": only_sim,
        "only_real": only_real,
        "formats": sorted(template_sources.values(), key=lambda item: item["template"]),
        "datasets": dataset_rows,
    }


def _binary_profile(values: Any) -> dict[str, Any]:
    import numpy as np  # type: ignore

    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {"count": 0, "binary": None}
    unique = np.unique(np.round(finite, 6))
    quantile_low = float(np.quantile(finite, 0.05))
    quantile_high = float(np.quantile(finite, 0.95))
    span = max(quantile_high - quantile_low, 1e-9)
    tolerance = max(span * 0.08, 1e-5)
    clustered = float(
        np.mean(
            (np.abs(finite - quantile_low) <= tolerance)
            | (np.abs(finite - quantile_high) <= tolerance)
        )
    )
    # Exactly two levels are binary.  A high endpoint-cluster ratio also keeps
    # slightly noisy binary recordings from being misclassified as continuous.
    binary = bool(unique.size == 2 or (unique.size > 2 and span > 1e-8 and clustered >= 0.97))
    # For a literal two-value signal, inspect the actual two levels even when
    # one level is rare.  Quantiles remain appropriate for noisy two-cluster
    # recordings because they reject isolated outliers.
    low = float(unique[0]) if unique.size == 2 else quantile_low
    high = float(unique[-1]) if unique.size == 2 else quantile_high
    threshold = (low + high) / 2
    return {
        "count": int(finite.size),
        "binary": binary,
        "mode": "binary" if binary else "continuous",
        "unique_rounded": int(min(unique.size, 10000)),
        "unique_values": [round(float(value), 8) for value in unique[:8]],
        "cluster_ratio": round(clustered, 4),
        "low_level": round(low, 8),
        "high_level": round(high, 8),
        "threshold": round(threshold, 8),
        "min": round(float(np.min(finite)), 8),
        "max": round(float(np.max(finite)), 8),
    }


def _severity_rank(value: str) -> int:
    return {"pass": 0, "warn": 1, "fail": 2}.get(value, 0)


def _merge_status(*values: str) -> str:
    return max(values, key=_severity_rank, default="pass")


def _dimension_report(descriptors: list[dict[str, Any]], arrays_by_id: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    by_platform = {
        platform: [item for item in descriptors if item["platform"] == platform]
        for platform in PLATFORMS
    }
    for vector in ("action", "state"):
        layout = VECTOR_LAYOUTS[vector]
        for index, canonical in enumerate(layout):
            definition_status, definition_warnings = _definition_status(descriptors, vector, index, canonical)
            sim_arrays = [arrays_by_id[item["id"]][vector] for item in by_platform["simulation"] if item["id"] in arrays_by_id]
            real_arrays = [arrays_by_id[item["id"]][vector] for item in by_platform["real"] if item["id"] in arrays_by_id]
            sim_stats = _aggregate_dimension(sim_arrays, index)
            real_stats = _aggregate_dimension(real_arrays, index)
            # Grippers and the lift have explicit absolute-value rules below.
            # Running the generic distribution-overlap heuristic as well would
            # duplicate those checks and flags constant 200 mm lift data as a
            # false warning.
            if canonical in {"left_gripper", "right_gripper", "lift_mm"}:
                range_status, range_warnings, metrics = "pass", [], {}
            else:
                range_status, range_warnings, metrics = compare_ranges(sim_stats, real_stats)
            per_dataset: list[dict[str, Any]] = []
            for item in descriptors:
                array = arrays_by_id.get(item["id"], {}).get(vector)
                stats = _dimension_stats(array[:, index]) if getattr(array, "ndim", 0) == 2 and array.shape[1] > index else {"count": 0}
                per_dataset.append({"dataset_id": item["id"], "dataset": item["name"], "platform": item["platform"], "stats": stats})
            rows.append({
                "vector": vector,
                "index": index,
                "name": canonical,
                "unit": "mm" if canonical == "lift_mm" else "m/s or rad/s" if canonical.startswith("base_v") or canonical == "base_wz" else "rad or native gripper",
                "status": _merge_status(definition_status, range_status),
                "definition_status": definition_status,
                "range_status": range_status,
                "warnings": [*definition_warnings, *range_warnings],
                "simulation": sim_stats,
                "real": real_stats,
                "metrics": metrics,
                "datasets": per_dataset,
            })
    return rows


def _gripper_profile_status(profile: dict[str, Any], platform_label: str) -> tuple[str, list[str]]:
    if not profile.get("count"):
        return "warn", [f"{platform_label}缺少夹爪数据"]

    low = float(profile.get("min", math.nan))
    high = float(profile.get("max", math.nan))
    if profile.get("binary"):
        low_level = float(profile.get("low_level", math.nan))
        high_level = float(profile.get("high_level", math.nan))
        valid = (
            math.isfinite(low_level)
            and math.isfinite(high_level)
            and abs(low_level - GRIPPER_EXPECTED_LOW) <= GRIPPER_BINARY_LEVEL_TOLERANCE
            and abs(high_level - GRIPPER_EXPECTED_HIGH) <= GRIPPER_BINARY_LEVEL_TOLERANCE
        )
        if valid:
            return "pass", []
        return "fail", [
            f"{platform_label}二值夹爪档位为 {low_level:g}/{high_level:g}，"
            f"期望 {GRIPPER_EXPECTED_LOW:g}/{GRIPPER_EXPECTED_HIGH:g}"
            f"（容差±{GRIPPER_BINARY_LEVEL_TOLERANCE:g}）"
        ]

    if not math.isfinite(low) or not math.isfinite(high):
        return "fail", [f"{platform_label}连续夹爪包含非有限数值"]
    if low >= GRIPPER_CONTINUOUS_MIN and high <= GRIPPER_CONTINUOUS_MAX:
        return "pass", []
    if low >= GRIPPER_CONTINUOUS_SOFT_MIN and high <= GRIPPER_CONTINUOUS_SOFT_MAX:
        return "warn", [
            f"{platform_label}连续夹爪范围 {low:g}～{high:g}，超出标准 "
            f"{GRIPPER_CONTINUOUS_MIN:g}～{GRIPPER_CONTINUOUS_MAX:g}，"
            f"但仍在允许范围 {GRIPPER_CONTINUOUS_SOFT_MIN:g}～{GRIPPER_CONTINUOUS_SOFT_MAX:g}"
        ]
    return "fail", [
        f"{platform_label}连续夹爪范围 {low:g}～{high:g}，超出允许范围 "
        f"{GRIPPER_CONTINUOUS_SOFT_MIN:g}～{GRIPPER_CONTINUOUS_SOFT_MAX:g}"
    ]


def _gripper_report(descriptors: list[dict[str, Any]], arrays_by_id: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    import numpy as np  # type: ignore

    rows: list[dict[str, Any]] = []
    for (vector, side), index in GRIPPER_DIMS.items():
        profiles: dict[str, dict[str, Any]] = {}
        for platform in PLATFORMS:
            values = []
            for item in descriptors:
                if item["platform"] != platform:
                    continue
                array = arrays_by_id.get(item["id"], {}).get(vector)
                if getattr(array, "ndim", 0) == 2 and array.shape[1] > index:
                    values.append(array[:, index])
            profiles[platform] = _binary_profile(np.concatenate(values) if values else np.array([]))
        sim, real = profiles["simulation"], profiles["real"]
        warnings: list[str] = []
        statuses: list[str] = []
        for platform, platform_label in PLATFORMS.items():
            profile_status, profile_warnings = _gripper_profile_status(profiles[platform], platform_label)
            statuses.append(profile_status)
            warnings.extend(profile_warnings)
        if sim.get("count") and real.get("count") and sim.get("binary") != real.get("binary"):
            statuses.append("fail")
            warnings.append(
                "夹爪值类型不一致："
                f"仿真为{'二值' if sim.get('binary') else '连续值'}，"
                f"真机为{'二值' if real.get('binary') else '连续值'}"
            )
        status = _merge_status(*statuses)
        rows.append({
            "vector": vector,
            "side": side,
            "index": index,
            "name": f"{side}_{vector}_gripper",
            "status": status,
            "warnings": warnings,
            "simulation": sim,
            "real": real,
        })
    return rows


def _lift_report(descriptors: list[dict[str, Any]], arrays_by_id: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    lift_index = ACTION_LAYOUT.index("lift_mm")
    for vector in ("action", "state"):
        platform_profiles: dict[str, dict[str, Any]] = {}
        warnings: list[str] = []
        statuses: list[str] = []
        for platform, platform_label in PLATFORMS.items():
            members = [item for item in descriptors if item["platform"] == platform]
            arrays = [
                arrays_by_id[item["id"]][vector]
                for item in members
                if item["id"] in arrays_by_id
            ]
            stats = _aggregate_dimension(arrays, lift_index)
            platform_profiles[platform] = stats
            if not members:
                continue
            if not stats.get("count"):
                statuses.append("fail")
                warnings.append(f"{platform_label}{vector} 缺少第 {lift_index} 维升降柱数据")
                continue
            low = float(stats.get("min", math.nan))
            high = float(stats.get("max", math.nan))
            if (
                not math.isfinite(low)
                or not math.isfinite(high)
                or abs(low - LIFT_EXPECTED_MM) > LIFT_TOLERANCE_MM
                or abs(high - LIFT_EXPECTED_MM) > LIFT_TOLERANCE_MM
            ):
                statuses.append("fail")
                warnings.append(
                    f"{platform_label}{vector} 升降柱范围 {low:g}～{high:g} mm，"
                    f"期望 {LIFT_EXPECTED_MM:g}±{LIFT_TOLERANCE_MM:g} mm"
                )
            else:
                statuses.append("pass")
        rows.append(
            {
                "vector": vector,
                "index": lift_index,
                "name": "lift_mm",
                "expected_mm": LIFT_EXPECTED_MM,
                "tolerance_mm": LIFT_TOLERANCE_MM,
                "status": _merge_status(*statuses),
                "warnings": warnings,
                "simulation": platform_profiles.get("simulation", {"count": 0}),
                "real": platform_profiles.get("real", {"count": 0}),
            }
        )
    return rows


def _episode_review_rows(descriptor: dict[str, Any]) -> list[dict[str, Any]]:
    path = Path(descriptor["path"])
    tasks = {as_int(item.get("task_index")): str(item.get("task") or "") for item in task_rows(path)}
    rows = episode_rows(path)
    if not rows:
        rows = [{"episode_index": index} for index in _episode_indices_from_data(path)]
    output: list[dict[str, Any]] = []
    for row in rows:
        index = as_int(row.get("episode_index"))
        if index is None:
            continue
        raw_tasks = row.get("full_instructions_en") or row.get("tasks")
        task = ""
        if isinstance(raw_tasks, list) and raw_tasks:
            task = str(raw_tasks[0])
        task = str(row.get("task") or task or tasks.get(as_int(row.get("task_index")), ""))
        grade = str(row.get("quality_grade") or row.get("manual_quality_grade") or "").strip().upper()
        output.append({
            "key": f"{descriptor['id']}:{index}",
            "dataset_id": descriptor["id"],
            "logical_id": descriptor.get("logical_id") or descriptor["id"],
            "dataset": descriptor["name"],
            "dataset_path": descriptor["path"],
            "logical_path": descriptor.get("logical_path") or descriptor["path"],
            "source_grade": descriptor.get("source_grade") or "",
            "platform": descriptor["platform"],
            "episode_index": index,
            "episode_name": f"episode_{index:06d}",
            "task": task,
            "quality_grade": grade if grade in QUALITY_GRADES else "",
            "length": as_int(row.get("length")) or 0,
            "subtask_segments": row.get("subtask_segments") if isinstance(row.get("subtask_segments"), list) else [],
        })
    return sorted(output, key=lambda item: item["episode_index"])


def analyze_datasets(selections: list[Any], max_episodes: int = 20, max_frames: int = 1000) -> dict[str, Any]:
    max_episodes = max(1, min(int(max_episodes), 200))
    max_frames = max(50, min(int(max_frames), 5000))
    descriptors: list[dict[str, Any]] = []
    seen: set[Path] = set()
    for raw in selections:
        if not isinstance(raw, dict) or not raw.get("selected", True):
            continue
        path = Path(str(raw.get("path") or "")).expanduser().resolve()
        if path in seen:
            continue
        seen.add(path)
        descriptor = describe_dataset(path, _validate_platform(raw.get("platform")))
        logical_path = str(raw.get("logical_path") or descriptor.get("grade_group_path") or path)
        logical_name = str(raw.get("logical_name") or descriptor.get("grade_group_name") or descriptor["name"])
        descriptor["source_name"] = descriptor["name"]
        descriptor["name"] = logical_name
        descriptor["logical_path"] = str(Path(logical_path).expanduser().resolve())
        descriptor["logical_id"] = dataset_id(Path(descriptor["logical_path"]))
        descriptor["source_grade"] = str(raw.get("source_grade") or descriptor.get("source_grade") or "")
        descriptors.append(descriptor)
    if not descriptors:
        raise ValueError("请至少选择一个 LeRobot 数据集")

    arrays_by_id: dict[str, dict[str, Any]] = {}
    for descriptor in descriptors:
        try:
            sample, arrays = _read_dataset_samples(descriptor, max_episodes, max_frames)
            descriptor.update(sample)
            arrays_by_id[descriptor["id"]] = arrays
        except Exception as exc:
            descriptor.setdefault("issues", []).append(f"采样失败: {exc}")
            descriptor.update({"sampled_episodes": 0, "sampled_frames": 0, "vector_stats": {"state": [], "action": []}})

    prompts = _prompt_report(descriptors)
    dimensions = _dimension_report(descriptors, arrays_by_id)
    grippers = _gripper_report(descriptors, arrays_by_id)
    lift = _lift_report(descriptors, arrays_by_id)
    episodes = [row for descriptor in descriptors for row in _episode_review_rows(descriptor)]
    statuses = [
        prompts["status"],
        *(row["status"] for row in dimensions),
        *(row["status"] for row in grippers),
        *(row["status"] for row in lift),
    ]
    return {
        "generated_at": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "contract": {
            "action_dim": 18,
            "state_dim": 21,
            "action_layout": ACTION_LAYOUT,
            "state_layout": STATE_LAYOUT,
        },
        "sampling": {"max_episodes_per_dataset": max_episodes, "max_frames_per_episode": max_frames},
        "summary": {
            "status": _merge_status(*statuses),
            "dataset_count": len({item["logical_id"] for item in descriptors}),
            "simulation_count": len({item["logical_id"] for item in descriptors if item["platform"] == "simulation"}),
            "real_count": len({item["logical_id"] for item in descriptors if item["platform"] == "real"}),
            "episode_count": len(episodes),
            "dimension_failures": sum(row["status"] == "fail" for row in dimensions),
            "dimension_warnings": sum(row["status"] == "warn" for row in dimensions),
            "gripper_failures": sum(row["status"] == "fail" for row in grippers),
            "lift_failures": sum(row["status"] == "fail" for row in lift),
            "lift_warnings": sum(row["status"] == "warn" for row in lift),
        },
        "datasets": descriptors,
        "prompts": prompts,
        "dimensions": dimensions,
        "grippers": grippers,
        "lift": lift,
        "episodes": episodes,
    }


def _format_lerobot_path(pattern: str, episode_index: int, chunks_size: int, video_key: str = "") -> str:
    return pattern.format(
        episode_chunk=episode_index // max(1, chunks_size),
        episode_index=episode_index,
        video_key=video_key,
    )


def _hardlink_or_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, target)
    except OSError:
        shutil.copy2(source, target)


def _indexed_rows(rows: list[dict[str, Any]], key: str = "episode_index") -> dict[int, dict[str, Any]]:
    output: dict[int, dict[str, Any]] = {}
    for row in rows:
        index = as_int(row.get(key))
        if index is not None:
            output[index] = row
    return output


def _annotation_map(annotations: list[Any]) -> dict[int, dict[str, Any]]:
    output: dict[int, dict[str, Any]] = {}
    for raw in annotations:
        if not isinstance(raw, dict):
            continue
        index = as_int(raw.get("episode_index"))
        if index is None or index < 0:
            raise ValueError(f"invalid episode index: {raw.get('episode_index')!r}")
        grade = str(raw.get("quality_grade") or "").strip().upper()
        if grade and grade not in QUALITY_GRADES:
            raise ValueError(f"invalid quality grade for episode {index}: {grade}")
        output[index] = {
            "episode_index": index,
            "quality_grade": grade,
            "exclude": bool(raw.get("exclude")),
            "reason": str(raw.get("reason") or "").strip(),
        }
    return output


def validate_review_request(source: Path, output: Path, annotations: list[Any]) -> tuple[Path, Path, dict[int, dict[str, Any]]]:
    source = source.expanduser().resolve()
    output = output.expanduser().resolve()
    if not is_lerobot_dataset(source):
        raise ValueError(f"not a LeRobot dataset: {source}")
    if source == output:
        raise ValueError("输出目录不能与源数据集相同")
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"输出目录已存在，请更换目录: {output}")
    try:
        output.relative_to(source)
    except ValueError:
        pass
    else:
        raise ValueError("输出目录不能位于源数据集内部")
    try:
        source.relative_to(output)
    except ValueError:
        pass
    else:
        raise ValueError("输出目录不能是源数据集的父目录")
    return source, output, _annotation_map(annotations)


def rebuild_reviewed_dataset(
    source: Path,
    output: Path,
    annotations: list[Any],
    *,
    progress: Callable[[str], None] | None = None,
    stop_requested: Callable[[], bool] | None = None,
) -> dict[str, Any]:
    import pandas as pd  # type: ignore

    source, output, changes = validate_review_request(source, output, annotations)
    info = load_json(source / "meta" / "info.json")
    chunks_size = int(info.get("chunks_size") or 1000)
    data_pattern = str(info.get("data_path") or "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet")
    video_pattern = str(info.get("video_path") or "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4")
    video_keys = video_feature_keys(info)
    source_data: dict[int, Path] = {}
    for path in source.glob("data/chunk-*/episode_*.parquet"):
        match = re.search(r"episode_(\d+)\.parquet$", path.name)
        if match:
            source_data[int(match.group(1))] = path
    if not source_data:
        raise ValueError(f"没有找到 LeRobot parquet episode: {source}")

    episodes_by_index = _indexed_rows(episode_rows(source))
    stats_by_index = _indexed_rows(load_jsonl(source / "meta" / "episodes_stats.jsonl"))
    mapping_payload = load_json(source / "meta" / "episode_name_mapping.json")
    mapping_rows = mapping_payload.get("episodes") if isinstance(mapping_payload.get("episodes"), list) else []
    mapping_by_index = _indexed_rows(
        [row for row in mapping_rows if isinstance(row, dict)],
        "lerobot_episode_index",
    )
    unknown = sorted(set(changes) - set(source_data))
    if unknown:
        raise ValueError(f"标注包含不存在的 episode: {unknown}")
    kept = [index for index in sorted(source_data) if not changes.get(index, {}).get("exclude")]
    if not kept:
        raise ValueError("不能剔除全部 episode")

    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.parent / f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
    if staging.exists():
        raise FileExistsError(staging)
    staging.mkdir(parents=True)
    new_episodes: list[dict[str, Any]] = []
    new_stats: list[dict[str, Any]] = []
    new_mapping: list[dict[str, Any]] = []
    index_mapping: list[dict[str, Any]] = []
    total_frames = 0
    total_videos = 0
    try:
        source_meta = source / "meta"
        target_meta = staging / "meta"
        target_meta.mkdir(parents=True)
        managed_meta = {
            "info.json",
            "episodes.jsonl",
            "episodes_stats.jsonl",
            "episode_name_mapping.json",
            "manual_review.json",
        }
        if source_meta.is_dir():
            for item in source_meta.iterdir():
                if item.name in managed_meta:
                    continue
                target = target_meta / item.name
                if item.is_dir():
                    shutil.copytree(item, target)
                elif item.is_file():
                    shutil.copy2(item, target)

        global_index = 0
        for new_index, old_index in enumerate(kept):
            if stop_requested and stop_requested():
                raise RuntimeError("用户停止了批量重建")
            change = changes.get(old_index, {})
            source_parquet = source_data[old_index]
            frame = pd.read_parquet(source_parquet)
            frame_count = len(frame)
            frame["episode_index"] = new_index
            if "frame_index" in frame.columns:
                frame["frame_index"] = list(range(frame_count))
            if "index" in frame.columns:
                frame["index"] = list(range(global_index, global_index + frame_count))
            global_index += frame_count
            total_frames += frame_count
            data_rel = _format_lerobot_path(data_pattern, new_index, chunks_size)
            data_target = staging / data_rel
            data_target.parent.mkdir(parents=True, exist_ok=True)
            frame.to_parquet(data_target, index=False, compression="zstd")

            episode_meta = dict(episodes_by_index.get(old_index, {}))
            episode_meta["episode_index"] = new_index
            episode_meta["length"] = frame_count
            grade = str(change.get("quality_grade") or episode_meta.get("quality_grade") or "").strip().upper()
            if grade in QUALITY_GRADES:
                episode_meta["quality_grade"] = grade
                episode_meta["manual_quality_grade"] = grade
            reason = str(change.get("reason") or "").strip()
            if reason:
                episode_meta["manual_review_reason"] = reason
            new_episodes.append(episode_meta)

            if old_index in stats_by_index:
                stats = dict(stats_by_index[old_index])
                stats["episode_index"] = new_index
                new_stats.append(stats)

            map_row = dict(mapping_by_index.get(old_index, {}))
            source_video_files = (
                dict(map_row.get("lerobot_video_files"))
                if isinstance(map_row.get("lerobot_video_files"), dict)
                else {}
            )
            map_row.update(
                {
                    "source_lerobot_episode_index": old_index,
                    "lerobot_episode_index": new_index,
                    "lerobot_episode_name": f"episode_{new_index:06d}",
                    "lerobot_data_file": data_rel,
                    "num_frames": frame_count,
                }
            )
            if grade in QUALITY_GRADES:
                map_row["quality_grade"] = grade
            video_files: dict[str, str] = {}
            for video_key in video_keys:
                old_rel = str(source_video_files.get(video_key) or _format_lerobot_path(video_pattern, old_index, chunks_size, video_key))
                new_rel = _format_lerobot_path(video_pattern, new_index, chunks_size, video_key)
                source_video = source / old_rel
                if source_video.is_file():
                    _hardlink_or_copy(source_video, staging / new_rel)
                    video_files[video_key] = new_rel
                    total_videos += 1
            map_row["lerobot_video_files"] = video_files
            new_mapping.append(map_row)
            index_mapping.append(
                {
                    "source_episode_index": old_index,
                    "reviewed_episode_index": new_index,
                    "quality_grade": grade,
                    "reason": reason,
                }
            )
            if progress:
                progress(f"{source.name}: {old_index} -> {new_index} ({frame_count} frames)")

        new_info = dict(info)
        new_info["total_episodes"] = len(new_episodes)
        new_info["total_frames"] = total_frames
        new_info["total_videos"] = total_videos
        new_info["total_chunks"] = max(1, math.ceil(len(new_episodes) / max(1, chunks_size)))
        new_info["splits"] = {"train": f"0:{len(new_episodes)}"}
        write_json(target_meta / "info.json", new_info)
        write_jsonl(target_meta / "episodes.jsonl", new_episodes)
        if new_stats:
            write_jsonl(target_meta / "episodes_stats.jsonl", new_stats)
        new_mapping_payload = dict(mapping_payload)
        new_mapping_payload.update(
            {
                "source_lerobot_dataset": str(source),
                "reviewed_lerobot_dataset": str(output),
                "episodes": new_mapping,
            }
        )
        write_json(target_meta / "episode_name_mapping.json", new_mapping_payload)
        review_record = {
            "version": 1,
            "created_at": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
            "source_dataset": str(source),
            "output_dataset": str(output),
            "source_episode_count": len(source_data),
            "output_episode_count": len(new_episodes),
            "excluded_episode_indices": sorted(index for index, change in changes.items() if change.get("exclude")),
            "requested_changes": [changes[index] for index in sorted(changes)],
            "index_mapping": index_mapping,
        }
        write_json(target_meta / "manual_review.json", review_record)
        staging.replace(output)
        return {
            "source": str(source),
            "output": str(output),
            "source_episodes": len(source_data),
            "output_episodes": len(new_episodes),
            "excluded": len(source_data) - len(new_episodes),
            "changed_grades": sum(bool(change.get("quality_grade")) for change in changes.values()),
            "total_frames": total_frames,
            "index_mapping": index_mapping,
        }
    except Exception:
        if staging.exists():
            shutil.rmtree(staging)
        raise
