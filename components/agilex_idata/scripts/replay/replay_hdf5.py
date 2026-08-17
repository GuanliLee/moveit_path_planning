#!/usr/bin/env python3
"""
回放 ALOHA HDF5 数据集：同时显示 3 路相机 + 双臂关节曲线
用法:
    python3 replay_hdf5.py /home/agilex/data/aloha/episode1/episode1.hdf5
快捷键:
    SPACE  暂停 / 继续
    ←/→    单帧后退/前进
    q      退出
"""
import argparse
import os
import sys
import time

import cv2
import h5py
import numpy as np


def load_image(episode_dir: str, rel_path: bytes | str) -> np.ndarray | None:
    if isinstance(rel_path, bytes):
        rel_path = rel_path.decode()
    full = os.path.join(episode_dir, rel_path)
    if not os.path.exists(full):
        return None
    return cv2.imread(full, cv2.IMREAD_COLOR)


def render_joint_plot(history: np.ndarray, width: int, height: int, title: str) -> np.ndarray:
    """history: shape (T, J)，画到一张 (height, width, 3) 的 BGR 图上。"""
    canvas = np.full((height, width, 3), 30, dtype=np.uint8)
    if history.size == 0:
        return canvas
    T, J = history.shape
    margin = 20
    plot_w = width - 2 * margin
    plot_h = height - 2 * margin - 16

    vmin = history.min()
    vmax = history.max()
    if vmax - vmin < 1e-6:
        vmax = vmin + 1.0

    cv2.putText(canvas, title, (margin, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

    palette = [
        (255, 0, 0), (0, 255, 0), (0, 0, 255),
        (255, 255, 0), (255, 0, 255), (0, 255, 255),
        (255, 255, 255),
    ]
    for j in range(J):
        color = palette[j % len(palette)]
        pts = []
        for t in range(T):
            x = int(margin + t / max(T - 1, 1) * plot_w)
            y = int(margin + 16 + plot_h - (history[t, j] - vmin) / (vmax - vmin) * plot_h)
            pts.append((x, y))
        if len(pts) >= 2:
            for k in range(1, len(pts)):
                cv2.line(canvas, pts[k - 1], pts[k], color, 1, cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("hdf5", help="HDF5 文件路径")
    parser.add_argument("--rate", type=float, default=30.0, help="回放帧率 (Hz)")
    parser.add_argument("--window", type=int, default=200, help="关节曲线窗口长度（帧数）")
    parser.add_argument("--output", type=str, default="",
                        help="导出 mp4 视频路径（不需要 GUI），不指定则实时显示")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="最多导出/回放多少帧，0 表示全部")
    args = parser.parse_args()

    hdf5_path = os.path.abspath(args.hdf5)
    episode_dir = os.path.dirname(hdf5_path)

    with h5py.File(hdf5_path, "r") as f:
        size = int(f["size"][()])
        ts = f["timestamp"][()]
        master_l = f["arm/jointStatePosition/masterLeft"][()]
        master_r = f["arm/jointStatePosition/masterRight"][()]
        puppet_l = f["arm/jointStatePosition/puppetLeft"][()]
        puppet_r = f["arm/jointStatePosition/puppetRight"][()]
        cam_keys = ["camera/color/left", "camera/color/front", "camera/color/right"]
        cam_paths = {k: f[k][()] for k in cam_keys if k in f}
        mobile_parts = []
        mobile_names = []
        for key, names in [
            ("robotBase/action/chassis", ["vx_cmd", "vy_cmd", "wz_cmd"]),
            ("robotBase/state/chassis", ["x", "y", "yaw", "vx", "vy", "wz"]),
            ("lift/motor/column", ["lift_state"]),
            ("action/lifting/column", ["lift_action"]),
        ]:
            if key not in f:
                continue
            data = np.asarray(f[key][()])
            if data.ndim == 1:
                data = data.reshape(-1, 1)
            mobile_parts.append(data)
            mobile_names.extend(names[:data.shape[1]])
        mobile = np.hstack(mobile_parts) if mobile_parts else None

    render_size = size if args.max_frames <= 0 else min(size, args.max_frames)
    print(f"加载 {hdf5_path}")
    print(f"  episode 长度: {size} 帧, 时长 ≈ {ts[-1] - ts[0]:.1f} s")
    if render_size != size:
        print(f"  本次只渲染前 {render_size} 帧")
    print(f"  相机: {list(cam_paths.keys())}")
    print(f"  关节维度: master_l={master_l.shape}, puppet_l={puppet_l.shape}")
    if mobile is not None:
        print(f"  移动/升降维度: {mobile.shape} {mobile_names}")
    print()
    print("快捷键: 空格=暂停/继续, ←/→=单帧, q=退出")

    frame_interval = 1.0 / args.rate
    idx = 0
    paused = False
    last_time = time.time()

    export_mode = bool(args.output)
    writer = None

    while True:
        idx = max(0, min(render_size - 1, idx))

        imgs = []
        for k in cam_keys:
            if k in cam_paths:
                img = load_image(episode_dir, cam_paths[k][idx])
                if img is None:
                    img = np.full((480, 640, 3), 60, dtype=np.uint8)
                    cv2.putText(img, f"missing: {k}", (20, 240),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(img, k.split('/')[-1], (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                imgs.append(img)
        if imgs:
            h = min(im.shape[0] for im in imgs)
            imgs_resized = [cv2.resize(im, (int(im.shape[1] * h / im.shape[0]), h)) for im in imgs]
            cams_row = np.hstack(imgs_resized)
        else:
            cams_row = np.zeros((480, 1920, 3), dtype=np.uint8)

        win_start = max(0, idx - args.window)
        plot_w = cams_row.shape[1] // 2
        plot_h = 200
        left_plot = render_joint_plot(
            np.hstack([master_l[win_start:idx + 1], puppet_l[win_start:idx + 1]]),
            plot_w, plot_h, "Left arm: master(0-6) + puppet(7-13)"
        )
        right_plot = render_joint_plot(
            np.hstack([master_r[win_start:idx + 1], puppet_r[win_start:idx + 1]]),
            plot_w, plot_h, "Right arm: master(0-6) + puppet(7-13)"
        )
        plots_row = np.hstack([left_plot, right_plot])
        if plots_row.shape[1] != cams_row.shape[1]:
            plots_row = cv2.resize(plots_row, (cams_row.shape[1], plots_row.shape[0]))
        rows = [cams_row, None, plots_row]
        if mobile is not None:
            mobile_plot = render_joint_plot(
                mobile[win_start:idx + 1],
                cams_row.shape[1], 160, "Mobile/base/lift: " + ", ".join(mobile_names[:12])
            )
            rows.append(mobile_plot)

        info = f"frame {idx}/{render_size - 1}   t={ts[idx] - ts[0]:.2f}s   {'PAUSED' if paused else 'PLAY'}"
        info_bar = np.full((30, cams_row.shape[1], 3), 50, dtype=np.uint8)
        cv2.putText(info_bar, info, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        rows[1] = info_bar

        canvas = np.vstack(rows)
        screen_w = 1600
        if canvas.shape[1] > screen_w:
            scale = screen_w / canvas.shape[1]
            canvas = cv2.resize(canvas, (screen_w, int(canvas.shape[0] * scale)))

        if export_mode:
            if writer is None:
                h, w = canvas.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(args.output, fourcc, args.rate, (w, h))
                if not writer.isOpened():
                    print(f"[error] 无法创建视频文件: {args.output}")
                    return
                print(f"导出视频: {args.output}  ({w}x{h}, {args.rate} fps)")
            writer.write(canvas)
            if idx % 50 == 0 or idx == render_size - 1:
                sys.stdout.write(f"\r  导出进度: {idx + 1}/{render_size} ({(idx + 1) / render_size * 100:.1f}%)")
                sys.stdout.flush()
            idx += 1
            if idx >= render_size:
                print("\n导出完成")
                break
        else:
            cv2.imshow("ALOHA replay", canvas)
            if paused:
                key = cv2.waitKey(0) & 0xFF
            else:
                elapsed = time.time() - last_time
                wait = max(1, int((frame_interval - elapsed) * 1000))
                key = cv2.waitKey(wait) & 0xFF
                last_time = time.time()

            if key == ord('q'):
                break
            elif key == ord(' '):
                paused = not paused
            elif key in (81, 2):
                idx -= 1
                paused = True
            elif key in (83, 3):
                idx += 1
                paused = True
            else:
                if not paused:
                    idx += 1
                    if idx >= render_size:
                        print("回放结束")
                        break

    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
