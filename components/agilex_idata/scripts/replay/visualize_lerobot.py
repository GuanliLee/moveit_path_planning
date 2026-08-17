#!/usr/bin/env python3
"""Render a local LeRobot episode to an MP4 preview."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def import_pandas():
    try:
        import pandas as pd
    except ModuleNotFoundError as exc:
        raise SystemExit("pandas/pyarrow is required to read LeRobot parquet files.") from exc
    return pd


def read_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def find_dataset_roots(start: Path) -> list[Path]:
    if start.is_file():
        return []
    return sorted(info.parent.parent for info in start.rglob("meta/info.json") if (info.parent.parent / "data").exists())


def resolve_episode_path(path: Path, episode_index: int) -> tuple[Path, Path | None, dict]:
    if path.is_file():
        return path, None, {}

    dataset_root = path
    info_path = dataset_root / "meta" / "info.json"
    if not info_path.exists():
        roots = find_dataset_roots(path)
        if len(roots) == 1:
            dataset_root = roots[0]
            info_path = dataset_root / "meta" / "info.json"
        elif roots:
            raise FileNotFoundError(
                "Multiple LeRobot datasets found; pass one exact dataset root:\n"
                + "\n".join(f"  {root}" for root in roots)
            )
        else:
            raise FileNotFoundError(f"LeRobot meta/info.json not found under: {path}")

    info = read_json(info_path)
    chunks_size = int(info.get("chunks_size", 1000))
    episode_chunk = episode_index // chunks_size
    rel_template = info.get(
        "data_path",
        "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    )
    episode_path = dataset_root / rel_template.format(
        episode_chunk=episode_chunk,
        episode_index=episode_index,
    )
    if not episode_path.exists():
        matches = sorted(dataset_root.glob(f"data/**/episode_{episode_index:06d}.parquet"))
        if not matches:
            raise FileNotFoundError(f"Episode parquet not found: {episode_path}")
        episode_path = matches[0]
    return episode_path, dataset_root, info


def feature_names(info: dict, column: str) -> list[str]:
    names = info.get("features", {}).get(column, {}).get("names", [])
    if isinstance(names, list) and len(names) == 1 and isinstance(names[0], list):
        names = names[0]
    return [str(name) for name in names] if isinstance(names, list) else []


def stack_vector_column(df, column: str) -> np.ndarray | None:
    if column not in df.columns:
        return None
    values = [np.asarray(value, dtype=np.float64).reshape(-1) for value in df[column].to_numpy()]
    if not values:
        return None
    return np.stack(values, axis=0)


def camera_names(info: dict, dataset_root: Path | None) -> list[str]:
    names = []
    for key in info.get("features", {}):
        if key.startswith("observation.images."):
            names.append(key.split(".")[-1])
    if not names and dataset_root is not None:
        image_root = dataset_root / "images"
        for path in sorted(image_root.glob("observation.images.*")):
            names.append(path.name.split(".")[-1])
    preferred = ["left", "front", "right", "head"]
    return [name for name in preferred if name in names] + [name for name in names if name not in preferred]


def load_image(dataset_root: Path | None, camera: str, episode_index: int, frame_index: int):
    if dataset_root is None:
        return None
    image_path = (
        dataset_root
        / "images"
        / f"observation.images.{camera}"
        / f"episode_{episode_index:06d}"
        / f"frame_{frame_index:06d}.png"
    )
    if image_path.exists():
        return cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    return None


def render_plot(history: np.ndarray | None, width: int, height: int, title: str) -> np.ndarray:
    canvas = np.full((height, width, 3), 30, dtype=np.uint8)
    cv2.putText(canvas, title, (16, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
    if history is None or history.size == 0:
        return canvas
    history = np.asarray(history, dtype=np.float64)
    if history.ndim != 2:
        return canvas
    max_dims = min(history.shape[1], 14)
    values = history[:, :max_dims]
    vmin = float(np.nanmin(values))
    vmax = float(np.nanmax(values))
    if not np.isfinite(vmin) or not np.isfinite(vmax):
        return canvas
    if vmax - vmin < 1e-6:
        vmax = vmin + 1.0
    margin = 18
    plot_y = 32
    plot_w = max(width - margin * 2, 1)
    plot_h = max(height - plot_y - margin, 1)
    palette = [
        (255, 80, 80), (80, 220, 80), (80, 120, 255), (255, 210, 80),
        (220, 80, 255), (80, 230, 230), (235, 235, 235),
    ]
    for j in range(max_dims):
        pts = []
        for t, value in enumerate(values[:, j]):
            x = int(margin + t / max(len(values) - 1, 1) * plot_w)
            y = int(plot_y + plot_h - (value - vmin) / (vmax - vmin) * plot_h)
            pts.append((x, y))
        for k in range(1, len(pts)):
            cv2.line(canvas, pts[k - 1], pts[k], palette[j % len(palette)], 1, cv2.LINE_AA)
    return canvas


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", help="LeRobot dataset root or episode parquet")
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--output", required=True, help="Output mp4 path")
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--window", type=int, default=200)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means all frames")
    args = parser.parse_args()

    pd = import_pandas()
    episode_path, dataset_root, info = resolve_episode_path(Path(args.dataset).expanduser(), args.episode)
    if not info and dataset_root is None:
        maybe_root = episode_path.parents[2] if len(episode_path.parents) >= 3 else None
        if maybe_root and (maybe_root / "meta" / "info.json").exists():
            dataset_root = maybe_root
            info = read_json(maybe_root / "meta" / "info.json")

    df = pd.read_parquet(episode_path)
    n_frames = len(df)
    if args.max_frames > 0:
        n_frames = min(n_frames, args.max_frames)
    if n_frames <= 0:
        raise ValueError(f"No frames in {episode_path}")

    timestamps = np.asarray(df["timestamp"].to_numpy(), dtype=np.float64) if "timestamp" in df.columns else None
    fps = args.fps or float(info.get("fps") or 0) if info else args.fps
    if not fps or fps <= 0:
        if timestamps is not None and len(timestamps) > 1 and timestamps[-1] > timestamps[0]:
            fps = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
        else:
            fps = 30.0

    action = stack_vector_column(df, "action")
    state = stack_vector_column(df, "observation.state")
    cams = camera_names(info, dataset_root)

    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)

    writer = None
    for idx in range(n_frames):
        images = []
        for camera in cams:
            img = load_image(dataset_root, camera, args.episode, idx)
            if img is None:
                img = np.full((480, 640, 3), 58, dtype=np.uint8)
                cv2.putText(img, f"missing {camera}", (24, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (245, 245, 245), 1)
            cv2.putText(img, camera, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA)
            images.append(img)
        if images:
            h = min(image.shape[0] for image in images)
            resized = [cv2.resize(image, (int(image.shape[1] * h / image.shape[0]), h)) for image in images]
            camera_row = np.hstack(resized)
        else:
            camera_row = np.full((480, 1280, 3), 58, dtype=np.uint8)
            cv2.putText(camera_row, "no camera images", (40, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (245, 245, 245), 1)

        start = max(0, idx - args.window)
        plot_w = camera_row.shape[1] // 2
        action_plot = render_plot(action[start : idx + 1] if action is not None else None, plot_w, 210, "action")
        state_plot = render_plot(state[start : idx + 1] if state is not None else None, plot_w, 210, "observation.state")
        plot_row = np.hstack([action_plot, state_plot])
        if plot_row.shape[1] != camera_row.shape[1]:
            plot_row = cv2.resize(plot_row, (camera_row.shape[1], plot_row.shape[0]))

        t_text = f"{timestamps[idx] - timestamps[0]:.2f}s" if timestamps is not None else f"{idx / fps:.2f}s"
        info_bar = np.full((34, camera_row.shape[1], 3), 48, dtype=np.uint8)
        cv2.putText(info_bar, f"LeRobot episode {args.episode}  frame {idx}/{n_frames - 1}  t={t_text}",
                    (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
        canvas = np.vstack([camera_row, info_bar, plot_row])
        if canvas.shape[1] > 1600:
            scale = 1600 / canvas.shape[1]
            canvas = cv2.resize(canvas, (1600, int(canvas.shape[0] * scale)))

        if writer is None:
            h, w = canvas.shape[:2]
            writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            if not writer.isOpened():
                raise RuntimeError(f"Cannot create video: {output}")
            print(f"Render {episode_path} -> {output} ({w}x{h}, {fps:.2f} fps)")
        writer.write(canvas)
        if idx % 100 == 0 or idx == n_frames - 1:
            print(f"  frame {idx + 1}/{n_frames}")

    if writer is not None:
        writer.release()
    print(f"Done: {output}")


if __name__ == "__main__":
    main()
