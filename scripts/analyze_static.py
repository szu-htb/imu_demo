#!/usr/bin/env python3
"""
IMU 静态数据分析脚本

用法：
    python3 analyze_static.py <bag_path> [--name <experiment_name>]

示例：
    # 自动从路径提取实验名
    python3 analyze_static.py data/imu/raw/2026-03-18_static_17min_default/bag

    # 手动指定实验名
    python3 analyze_static.py ~/some/bag --name 2026-03-18_static_17min_default

输出目录：data/imu/analysis/<experiment_name>/
"""

import argparse
import io
import json
import sys
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# ---------------------------------------------------------------------------
# 数据读取
# ---------------------------------------------------------------------------

def read_bag(bag_path: str) -> dict:
    """从 bag 文件读取 IMU 相关话题的所有消息"""

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {}
    for topic_info in reader.get_all_topics_and_types():
        topic_types[topic_info.name] = topic_info.type

    target_topics = {
        "/imu/data_raw",
        "/imu/data_calibrated",
        "/imu/data",
        "/imu/rpy/filtered",
    }

    data = {t: [] for t in target_topics}

    while reader.has_next():
        topic, raw_data, timestamp_ns = reader.read_next()
        if topic in target_topics:
            msg_type_str = topic_types[topic]
            msg_type = get_message(msg_type_str)
            msg = deserialize_message(raw_data, msg_type)
            data[topic].append((timestamp_ns, msg))

    return data


def extract_imu_arrays(messages: list) -> dict:
    """从 sensor_msgs/Imu 消息列表提取 numpy 数组"""
    if not messages:
        return None

    n = len(messages)
    t = np.zeros(n)
    acc = np.zeros((n, 3))
    gyro = np.zeros((n, 3))

    t0 = messages[0][0]
    for i, (ts_ns, msg) in enumerate(messages):
        t[i] = (ts_ns - t0) * 1e-9
        acc[i] = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        gyro[i] = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]

    return {"t": t, "acc": acc, "gyro": gyro}


def extract_rpy_arrays(messages: list) -> dict:
    """从 geometry_msgs/Vector3Stamped 消息列表提取欧拉角数组"""
    if not messages:
        return None

    n = len(messages)
    t = np.zeros(n)
    rpy = np.zeros((n, 3))

    t0 = messages[0][0]
    for i, (ts_ns, msg) in enumerate(messages):
        t[i] = (ts_ns - t0) * 1e-9
        rpy[i] = [msg.vector.x, msg.vector.y, msg.vector.z]

    return {"t": t, "rpy": rpy}


# ---------------------------------------------------------------------------
# 统计计算
# ---------------------------------------------------------------------------

def compute_imu_stats(arrays: dict) -> dict:
    """计算单个 IMU 话题的统计量，返回结构化字典"""
    acc, gyro, t = arrays["acc"], arrays["gyro"], arrays["t"]
    duration = t[-1] - t[0]
    n = len(t)
    axes = ["x", "y", "z"]
    expected_acc = [0.0, 0.0, 9.81]

    stats = {
        "sample_count": n,
        "duration_s": round(duration, 2),
        "avg_frequency_hz": round(n / duration, 1) if duration > 0 else 0,
        "gyroscope": {
            "bias_rad_s": {},
            "noise_std_rad_s": {},
            "drift_deg_per_min": {},
        },
        "accelerometer": {
            "mean_m_s2": {},
            "noise_std_m_s2": {},
            "bias_m_s2": {},
        },
    }

    for i, ax in enumerate(axes):
        g_mean = float(np.mean(gyro[:, i]))
        g_std = float(np.std(gyro[:, i]))
        stats["gyroscope"]["bias_rad_s"][ax] = round(g_mean, 7)
        stats["gyroscope"]["noise_std_rad_s"][ax] = round(g_std, 7)
        stats["gyroscope"]["drift_deg_per_min"][ax] = round(np.degrees(g_mean) * 60.0, 4)

        a_mean = float(np.mean(acc[:, i]))
        a_std = float(np.std(acc[:, i]))
        stats["accelerometer"]["mean_m_s2"][ax] = round(a_mean, 6)
        stats["accelerometer"]["noise_std_m_s2"][ax] = round(a_std, 6)
        stats["accelerometer"]["bias_m_s2"][ax] = round(a_mean - expected_acc[i], 6)

    gz_mean = float(np.mean(gyro[:, 2]))
    stats["yaw_drift_from_gyro_deg_per_min"] = round(np.degrees(gz_mean) * 60.0, 4)

    return stats


def compute_rpy_stats(rpy_arrays: dict) -> dict:
    """计算欧拉角统计量"""
    t, rpy = rpy_arrays["t"], rpy_arrays["rpy"]
    duration = t[-1] - t[0]
    labels = ["roll", "pitch", "yaw"]

    stats = {"sample_count": len(t), "duration_s": round(duration, 2)}

    for i, label in enumerate(labels):
        start_deg = float(np.degrees(rpy[0, i]))
        end_deg = float(np.degrees(rpy[-1, i]))
        delta = end_deg - start_deg
        drift = delta / (duration / 60.0) if duration > 0 else 0
        stats[label] = {
            "start_deg": round(start_deg, 4),
            "end_deg": round(end_deg, 4),
            "delta_deg": round(delta, 4),
            "drift_deg_per_min": round(drift, 4),
        }

    yaw_deg = np.degrees(rpy[:, 2])
    coeffs = np.polyfit(t, yaw_deg, 1)
    stats["yaw_linear_fit_deg_per_min"] = round(float(coeffs[0]) * 60.0, 4)

    return stats


# ---------------------------------------------------------------------------
# 终端打印
# ---------------------------------------------------------------------------

def print_imu_stats(name: str, arrays: dict):
    acc, gyro, t = arrays["acc"], arrays["gyro"], arrays["t"]
    duration = t[-1] - t[0]
    n = len(t)

    print(f"\n{'='*60}")
    print(f"  {name}")
    print(f"  样本数: {n}  |  时长: {duration:.1f}s  |  平均频率: {n/duration:.1f}Hz")
    print(f"{'='*60}")

    print(f"\n  【陀螺仪】(rad/s)")
    print(f"  {'轴':<6} {'均值':>12} {'标准差':>12} {'均值(°/s)':>12} {'漂移(°/min)':>12}")
    print(f"  {'-'*54}")
    axis_names = ["X", "Y", "Z"]
    for i, ax in enumerate(axis_names):
        mean = np.mean(gyro[:, i])
        std = np.std(gyro[:, i])
        mean_deg = np.degrees(mean)
        drift_deg_min = mean_deg * 60.0
        print(f"  {ax:<6} {mean:>12.6f} {std:>12.6f} {mean_deg:>12.4f} {drift_deg_min:>12.2f}")

    print(f"\n  【加速度计】(m/s²)")
    print(f"  {'轴':<6} {'均值':>12} {'标准差':>12} {'期望值':>12} {'偏差':>12}")
    print(f"  {'-'*54}")
    expected = [0.0, 0.0, 9.81]
    for i, ax in enumerate(axis_names):
        mean = np.mean(acc[:, i])
        std = np.std(acc[:, i])
        bias = mean - expected[i]
        print(f"  {ax:<6} {mean:>12.6f} {std:>12.6f} {expected[i]:>12.2f} {bias:>+12.6f}")

    gz_mean = np.mean(gyro[:, 2])
    gz_mean_deg = np.degrees(gz_mean)
    drift_per_min = gz_mean_deg * 60.0
    print(f"\n  >>> Yaw 漂移估算: {drift_per_min:+.2f} °/min  "
          f"({gz_mean_deg:+.4f} °/s, 残余零偏 {gz_mean:+.6f} rad/s)")


def print_rpy_stats(rpy_arrays: dict):
    t, rpy = rpy_arrays["t"], rpy_arrays["rpy"]
    duration = t[-1] - t[0]

    print(f"\n{'='*60}")
    print(f"  滤波后欧拉角 (/imu/rpy/filtered)")
    print(f"  样本数: {len(t)}  |  时长: {duration:.1f}s")
    print(f"{'='*60}")

    labels = ["Roll", "Pitch", "Yaw"]
    print(f"\n  {'角度':<8} {'起始(°)':>10} {'终止(°)':>10} {'变化(°)':>10} {'漂移(°/min)':>12}")
    print(f"  {'-'*52}")
    for i, label in enumerate(labels):
        start = np.degrees(rpy[0, i])
        end = np.degrees(rpy[-1, i])
        delta = end - start
        drift = delta / (duration / 60.0) if duration > 0 else 0
        print(f"  {label:<8} {start:>10.4f} {end:>10.4f} {delta:>+10.4f} {drift:>+12.2f}")

    yaw_deg = np.degrees(rpy[:, 2])
    coeffs = np.polyfit(t, yaw_deg, 1)
    print(f"\n  >>> Yaw 线性拟合漂移速率: {coeffs[0]*60:+.2f} °/min  "
          f"({coeffs[0]:+.4f} °/s)")


# ---------------------------------------------------------------------------
# 画图
# ---------------------------------------------------------------------------

def plot_gyro_timeseries(arrays: dict, title_suffix: str, fig_path: str):
    t, gyro = arrays["t"], arrays["gyro"]
    t_min = t / 60.0

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axis_names = ["X", "Y", "Z"]
    colors = ["#e74c3c", "#27ae60", "#2980b9"]

    for i, (ax, name, color) in enumerate(zip(axes, axis_names, colors)):
        gyro_deg = np.degrees(gyro[:, i])
        ax.plot(t_min, gyro_deg, color=color, linewidth=0.3, alpha=0.6)
        window = min(200, len(gyro_deg) // 4)
        if window > 1:
            smoothed = np.convolve(gyro_deg, np.ones(window)/window, mode="valid")
            t_smooth = t_min[window//2 : window//2 + len(smoothed)]
            ax.plot(t_smooth, smoothed, color="black", linewidth=1.5,
                    label=f"Moving avg ({window})")
        ax.set_ylabel(f"Gyro {name} (°/s)")
        ax.axhline(y=0, color="gray", linestyle="--", linewidth=0.5)
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (min)")
    fig.suptitle(f"Gyroscope - {title_suffix}", fontsize=14)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=150)
    plt.close(fig)
    print(f"  [图表已保存] {fig_path}")


def plot_acc_timeseries(arrays: dict, title_suffix: str, fig_path: str):
    t, acc = arrays["t"], arrays["acc"]
    t_min = t / 60.0

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axis_names = ["X", "Y", "Z"]
    colors = ["#e74c3c", "#27ae60", "#2980b9"]
    expected = [0.0, 0.0, 9.81]

    for i, (ax, name, color) in enumerate(zip(axes, axis_names, colors)):
        ax.plot(t_min, acc[:, i], color=color, linewidth=0.3, alpha=0.6)
        ax.axhline(y=expected[i], color="gray", linestyle="--", linewidth=0.5,
                    label=f"Expected {expected[i]}")
        ax.set_ylabel(f"Acc {name} (m/s²)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (min)")
    fig.suptitle(f"Accelerometer - {title_suffix}", fontsize=14)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=150)
    plt.close(fig)
    print(f"  [图表已保存] {fig_path}")


def plot_rpy_timeseries(rpy_arrays: dict, fig_path: str):
    t, rpy = rpy_arrays["t"], rpy_arrays["rpy"]
    t_min = t / 60.0
    rpy_deg = np.degrees(rpy)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    labels = ["Roll", "Pitch", "Yaw"]
    colors = ["#e74c3c", "#27ae60", "#2980b9"]

    for i, (ax, label, color) in enumerate(zip(axes, labels, colors)):
        ax.plot(t_min, rpy_deg[:, i], color=color, linewidth=0.5)
        ax.set_ylabel(f"{label} (°)")
        ax.grid(True, alpha=0.3)

        if label == "Yaw":
            coeffs = np.polyfit(t, rpy_deg[:, i], 1)
            fit_line = np.polyval(coeffs, t)
            ax.plot(t_min, fit_line, color="black", linestyle="--", linewidth=1.5,
                    label=f"Linear fit: {coeffs[0]*60:+.2f} deg/min")
            ax.legend(loc="upper left")

    axes[-1].set_xlabel("Time (min)")
    fig.suptitle("Filtered Euler Angles (Roll / Pitch / Yaw)", fontsize=14)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=150)
    plt.close(fig)
    print(f"  [图表已保存] {fig_path}")


# ---------------------------------------------------------------------------
# 输出目录 & 实验名推断
# ---------------------------------------------------------------------------

def resolve_experiment_name(bag_path: Path, name_override: str | None) -> str:
    """从 bag 路径推断实验名，或使用手动指定的名称"""
    if name_override:
        return name_override

    # 尝试匹配 data/imu/raw/<name>/bag 结构
    parts = bag_path.resolve().parts
    for i, part in enumerate(parts):
        if part == "raw" and i + 2 < len(parts) and parts[i + 2] == "bag":
            return parts[i + 1]

    # 自动：日期时分 + bag 目录名
    timestamp = datetime.now().strftime("%Y-%m-%d_%H%M")
    bag_name = bag_path.resolve().name
    return f"{timestamp}_{bag_name}"


def resolve_output_dir(bag_path: Path, experiment_name: str) -> Path:
    """确定分析结果的输出目录"""
    # 尝试从 bag 路径向上找 data/imu/ 来定位工作区
    resolved = bag_path.resolve()
    for parent in resolved.parents:
        candidate = parent / "data" / "imu" / "analysis"
        if candidate.parent.parent.exists():
            return candidate / experiment_name

    # 兜底：在 cwd 下创建
    return Path.cwd() / "data" / "imu" / "analysis" / experiment_name


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="IMU 静态数据分析")
    parser.add_argument("bag_path", help="rosbag2 文件路径")
    parser.add_argument("--name", default=None,
                        help="实验名称（默认从路径推断）")
    args = parser.parse_args()

    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f"错误: 路径不存在 '{bag_path}'")
        sys.exit(1)

    experiment_name = resolve_experiment_name(bag_path, args.name)
    output_dir = resolve_output_dir(bag_path, experiment_name)
    plots_dir = output_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n实验名称: {experiment_name}")
    print(f"Bag 路径: {bag_path}")
    print(f"输出目录: {output_dir}")

    # 同时捕获终端输出，保存为 summary.txt
    tee = io.StringIO()

    class Tee:
        def __init__(self, *streams):
            self.streams = streams
        def write(self, data):
            for s in self.streams:
                s.write(data)
        def flush(self):
            for s in self.streams:
                s.flush()

    old_stdout = sys.stdout
    sys.stdout = Tee(old_stdout, tee)

    print(f"\n读取 bag 文件: {bag_path}")
    data = read_bag(str(bag_path))

    print("\n话题消息统计:")
    for topic, msgs in data.items():
        print(f"  {topic}: {len(msgs)} 条")

    # 收集所有统计量用于 summary.json
    summary = {"experiment": experiment_name, "bag_path": str(bag_path)}

    if data["/imu/data_raw"]:
        arrays = extract_imu_arrays(data["/imu/data_raw"])
        print_imu_stats("/imu/data_raw (原始数据)", arrays)
        summary["data_raw"] = compute_imu_stats(arrays)
        plot_gyro_timeseries(arrays, "data_raw",
                             str(plots_dir / "gyro_raw.png"))
        plot_acc_timeseries(arrays, "data_raw",
                            str(plots_dir / "acc_raw.png"))

    if data["/imu/data_calibrated"]:
        arrays = extract_imu_arrays(data["/imu/data_calibrated"])
        print_imu_stats("/imu/data_calibrated (校准后)", arrays)
        summary["data_calibrated"] = compute_imu_stats(arrays)
        plot_gyro_timeseries(arrays, "data_calibrated",
                             str(plots_dir / "gyro_calibrated.png"))
        plot_acc_timeseries(arrays, "data_calibrated",
                            str(plots_dir / "acc_calibrated.png"))

    if data["/imu/rpy/filtered"]:
        rpy_arrays = extract_rpy_arrays(data["/imu/rpy/filtered"])
        print_rpy_stats(rpy_arrays)
        summary["rpy_filtered"] = compute_rpy_stats(rpy_arrays)
        plot_rpy_timeseries(rpy_arrays,
                            str(plots_dir / "rpy_filtered.png"))

    if data["/imu/data"]:
        arrays = extract_imu_arrays(data["/imu/data"])
        print_imu_stats("/imu/data (融合节点输出)", arrays)
        summary["data_fused"] = compute_imu_stats(arrays)

    # 恢复 stdout
    sys.stdout = old_stdout

    # 写 summary.json
    json_path = output_dir / "summary.json"
    with open(json_path, "w") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"  [统计已保存] {json_path}")

    # 写 summary.txt
    txt_path = output_dir / "summary.txt"
    with open(txt_path, "w") as f:
        f.write(tee.getvalue())
    print(f"  [文本已保存] {txt_path}")

    # 写 command.txt
    cmd_path = output_dir / "command.txt"
    with open(cmd_path, "w") as f:
        f.write(" ".join(sys.argv) + "\n")
    print(f"  [命令已保存] {cmd_path}")

    print(f"\n{'='*60}")
    print(f"  分析完成！结果保存在: {output_dir}")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
