#!/usr/bin/env python3
import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import json
import csv
from datetime import datetime

def load_centerline(csv_path: str):
    if not os.path.exists(csv_path):
        print(f"[WARN] Centerline CSV not found at {csv_path}. Using Y-axis fallback.")
        return None, None, None, 0.0

    pts = []
    s_accum = [0.0]
    with open(csv_path, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split(',')
            pts.append([float(parts[0]), float(parts[1])])

    pts = np.array(pts)
    for i in range(1, len(pts)):
        ds = np.linalg.norm(pts[i] - pts[i - 1])
        s_accum.append(s_accum[-1] + ds)

    s_accum = np.array(s_accum)
    kdtree = KDTree(pts)
    return pts, s_accum, kdtree, s_accum[-1]


def read_bag(bag_path: str):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    odom_msgs = []
    drive_msgs = []

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t_sec = stamp * 1e-9

        if topic == '/ego_racecar/odom':
            msg = deserialize_message(data, Odometry)
            odom_msgs.append((t_sec, msg))
        elif topic == '/drive':
            msg = deserialize_message(data, AckermannDriveStamped)
            drive_msgs.append((t_sec, msg))

    return odom_msgs, drive_msgs

def export_summary_metrics(output_dir, file_name, bag_path, t_odom, lap_times, min_gaps, drive_intervals_ms):
    # Structure the hierarchical JSON metrics dictionary
    metrics = {
        "metadata": {
            "timestamp": datetime.now().isoformat(),
            "total_duration_sec": float(t_odom[-1])
        },
        "laps": {
            "completed_count": int(len(lap_times)),
            "lap_times_sec": [float(t) for t in lap_times],
            "mean_lap_sec": float(np.mean(lap_times)) if len(lap_times) > 0 else None,
            "std_lap_sec": float(np.std(lap_times)) if len(lap_times) > 0 else None,
            "fastest_lap_sec": float(np.min(lap_times)) if len(lap_times) > 0 else None
        },
        "safety": {
            "min_obstacle_margin_m": float(np.min(min_gaps)),
            "collision_detected": bool(np.min(min_gaps) <= 0.0)
        },
        "timing_50hz": {
            "target_period_ms": 20.0,
            "mean_interval_ms": float(np.mean(drive_intervals_ms)) if len(drive_intervals_ms) > 0 else None,
            "std_interval_ms": float(np.std(drive_intervals_ms)) if len(drive_intervals_ms) > 0 else None,
            "p95_interval_ms": float(np.percentile(drive_intervals_ms, 95)) if len(drive_intervals_ms) > 0 else None,
            "p99_interval_ms": float(np.percentile(drive_intervals_ms, 99)) if len(drive_intervals_ms) > 0 else None,
            "worst_case_ms": float(np.max(drive_intervals_ms)) if len(drive_intervals_ms) > 0 else None
        }
    }

    # Write JSON to the bag folder (or current directory)
    json_path = os.path.join(output_dir, f"{file_name}_benchmark_metrics.json")
    with open(json_path, "w") as f:
        json.dump(metrics, f, indent=4)
    print(f"[SUCCESS] Exported JSON metrics to {json_path}")

    # (Optional) Append to a cumulative master CSV for comparing tuning runs
    master_csv_path = os.path.join(output_dir, f"benchmark_history.csv")
    file_exists = os.path.isfile(master_csv_path)

    row = {
        "timestamp": metrics["metadata"]["timestamp"],
        "bag": os.path.basename(os.path.normpath(bag_path)),
        "duration_s": f"{metrics['metadata']['total_duration_sec']:.2f}",
        "laps": metrics["laps"]["completed_count"],
        "mean_lap_s": f"{metrics['laps']['mean_lap_sec']:.3f}" if metrics['laps']['mean_lap_sec'] else "N/A",
        "min_margin_m": f"{metrics['safety']['min_obstacle_margin_m']:.3f}",
        "p99_latency_ms": f"{metrics['timing_50hz']['p99_interval_ms']:.2f}" if metrics['timing_50hz']['p99_interval_ms'] else "N/A",
        "wcet_ms": f"{metrics['timing_50hz']['worst_case_ms']:.2f}" if metrics['timing_50hz']['worst_case_ms'] else "N/A"
    }

    with open(master_csv_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)
    print(f"[SUCCESS] Appended run entry to {master_csv_path}")

def main():
    parser = argparse.ArgumentParser(description="Offline RMPC ros2 bag Benchmark Processor")
    parser.add_argument('-b', '--bag_path', type=str, help="Path to recorded ros2 bag folder")
    parser.add_argument('--centerline', type=str,
                        default='src/rmpc_f1tenth/waypoints/IMS_centerline.csv',
                        help="Path to centerline CSV")
    parser.add_argument('--window', type=float, default=14.0,
                        help="Time window in seconds for the slalom plot")
    parser.add_argument('--filename', type=str, default='no_name', help="Name for the output results folder")
    parser.add_argument('--obstacles', type=str, default='no', help="Include obstacles in the evaluation (default: no)")
    args = parser.parse_args()
    file_name = args.filename
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    results_path = os.path.join(package_dir, 'results') 
    os.makedirs(results_path, exist_ok=True)
    print(f"Reading bag: {args.bag_path} ...")
    odom_data, drive_data = read_bag(args.bag_path)

    if not odom_data:
        print("[ERROR] No Odometry messages found in bag.")
        sys.exit(1)

    t0 = odom_data[0][0]
    c_pts, s_accum, kdtree, track_length = load_centerline(args.centerline)

    # Unpack Odometry
    t_odom = np.array([item[0] - t0 for item in odom_data])
    x = np.array([item[1].pose.pose.position.x for item in odom_data])
    y = np.array([item[1].pose.pose.position.y for item in odom_data])
    vx = np.array([item[1].twist.twist.linear.x for item in odom_data])
    wz = np.array([item[1].twist.twist.angular.z for item in odom_data])

    wheelbase = 0.3302
    steer_applied = np.arctan2(wz * wheelbase, np.maximum(0.5, vx))

    # Unpack Drive Commands & Loop Intervals
    t_drive = np.array([item[0] - t0 for item in drive_data])
    steer_cmd = np.array([item[1].drive.steering_angle for item in drive_data])
    drive_intervals_ms = np.diff(t_drive) * 1000.0

    # Match commanded steering to odom timestamps via 1D interpolation
    steer_cmd_interp = np.interp(t_odom, t_drive, steer_cmd) if len(drive_data) > 0 else np.zeros_like(t_odom)

    # Frenet Lateral Offset (d) and Longitudinal Progress (s)
    if kdtree is not None:
        _, idxs = kdtree.query(np.column_stack((x, y)))
        next_idxs = (idxs + 1) % len(c_pts)
        tangents = c_pts[next_idxs] - c_pts[idxs]
        tangents /= (np.linalg.norm(tangents, axis=1, keepdims=True) + 1e-6)
        normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
        d_arr = np.sum((np.column_stack((x, y)) - c_pts[idxs]) * normals, axis=1)
        s_arr = s_accum[idxs]
    else:
        d_arr = y
        s_arr = np.zeros_like(x)

    # Obstacle Distance Analysis
   ## Obstacle positions are hardcoded for the slalom benchmark.
   ## In a real scenario, these would be dynamically determined or loaded from a configuration.
    if args.obstacles.lower() == 'yes':
        obstacles = [
            np.array([0.190728, -10.2845, 0.25]),
            np.array([26.9882, 70.4085, 0.25]),
            np.array([52.2316, 9.98813, 0.25]),
        ]
        car_radius = 0.20
        car_positions = np.column_stack((x, y))
        min_gaps = np.full(len(x), 10.0)
        for obs in obstacles:
                    dist = np.linalg.norm(car_positions - obs[:2], axis=1) - (obs[2] + car_radius)
                    min_gaps = np.minimum(min_gaps, dist)
        
    else:
        min_gaps = np.full(len(x), 10000.0)

        
    # Lap Times Detection
    lap_times = []
    if kdtree is not None and track_length > 0.0:
        lap_start_t = t_odom[0]
        for i in range(1, len(s_arr)):
            if s_arr[i - 1] > (track_length * 0.9) and s_arr[i] < (track_length * 0.1):
                lap_time = t_odom[i] - lap_start_t
                lap_times.append(lap_time)
                lap_start_t = t_odom[i]

    # Plot 1: Slalom Benchmark 
    mask = (t_odom <= args.window)
    fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    plt.subplots_adjust(hspace=0.28)

    # Lateral Offset
    axs[0].plot(t_odom[mask], d_arr[mask], color='#1f77b4', lw=2.2)
    axs[0].axhline(4.0, color='gray', linestyle='--', alpha=0.7)
    axs[0].axhline(-4.0, color='gray', linestyle='--', alpha=0.7)
    axs[0].set_ylabel('d [m]', fontweight='bold')
    axs[0].set_title('Lateral offset', loc='left', fontweight='bold', fontsize=12)
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].set_ylim([-4.5, 4.5])

    # Gap to nearest obstacle
    axs[1].plot(t_odom[mask], min_gaps[mask], color='#1f77b4', lw=2.2)
    axs[1].axhline(0.0, color='black', lw=1.2)
    axs[1].set_ylabel('gap [m]', fontweight='bold')
    axs[1].set_title('Gap to nearest obstacle', loc='left', fontweight='bold', fontsize=12)
    axs[1].grid(True, linestyle=':', alpha=0.6)
    axs[1].set_ylim([-0.5, 6.0])

    # Commanded vs Applied Steer
    axs[2].plot(t_odom[mask], steer_cmd_interp[mask], color='#1f77b4', lw=2.0, label='commanded')
    axs[2].plot(t_odom[mask], steer_applied[mask], color='#d95f02', lw=2.0, label='applied')
    axs[2].axhline(0.3, color='gray', linestyle='--', alpha=0.7)
    axs[2].axhline(-0.3, color='gray', linestyle='--', alpha=0.7)
    axs[2].set_ylabel(r'$\delta$ [rad]', fontweight='bold')
    axs[2].set_xlabel('time [s]', fontweight='bold')
    axs[2].set_title('Steer angle', loc='left', fontweight='bold', fontsize=12)
    axs[2].grid(True, linestyle=':', alpha=0.6)
    axs[2].legend(loc='upper right', frameon=False, ncol=2)
    axs[2].set_ylim([-0.45, 0.45])

    slalom_png = f"{results_path}/{file_name}_bag_slalom_benchmark.png"
    plt.savefig(slalom_png, dpi=300, bbox_inches='tight')
    print(f"[SUCCESS] Saved slalom figure to {slalom_png}")

    #  Plot 2: Timing, Jitter, and Endurance Report 
    fig2, axs2 = plt.subplots(1, 2, figsize=(13, 5))
    plt.subplots_adjust(wspace=0.25)

    if len(drive_intervals_ms) > 0:
        axs2[0].hist(drive_intervals_ms, bins=40, color='#1f77b4', edgecolor='black', alpha=0.8, density=True)
        axs2[0].axvline(20.0, color='red', linestyle='--', lw=2.0, label='20 ms (50 Hz Target)')
        axs2[0].set_xlabel('Publish Interval [ms]', fontweight='bold')
        axs2[0].set_ylabel('Density', fontweight='bold')
        axs2[0].set_title('Control Loop Jitter (50 Hz)', fontweight='bold')
        axs2[0].legend()
        axs2[0].grid(True, linestyle=':', alpha=0.6)

        sorted_dt = np.sort(drive_intervals_ms)
        cdf = np.arange(1, len(sorted_dt) + 1) / len(sorted_dt)
        axs2[1].plot(sorted_dt, cdf, color='#d62728', lw=2.0)
        axs2[1].axvline(20.0, color='black', linestyle=':', label='20 ms Target')
        axs2[1].set_xlabel('Publish Interval [ms]', fontweight='bold')
        axs2[1].set_ylabel('Cumulative Probability', fontweight='bold')
        axs2[1].set_title('Latency CDF Profile', fontweight='bold')
        axs2[1].legend()
        axs2[1].grid(True, linestyle=':', alpha=0.6)

    timing_png = f"{results_path}/{file_name}_bag_timing_benchmark.png"
    plt.savefig(timing_png, dpi=300, bbox_inches='tight')
    print(f"[SUCCESS] Saved timing figure to {timing_png}")

    # Summary Output
    print("\n" + "=" * 60)
    print("         OFFLINE ROS2 BAG BENCHMARK EVALUATION SUMMARY      ")
    print("=" * 60)
    print(f"Total Duration Analyzed: {t_odom[-1]:.2f} s")
    print(f"Completed Laps Count   : {len(lap_times)}")
    if lap_times:
        print(f"Mean Lap Time          : {np.mean(lap_times):.3f} s (std: {np.std(lap_times):.3f} s)")
        print(f"Fastest Lap Time       : {np.min(lap_times):.3f} s")
    print(f"Minimum Obstacle Margin: {np.min(min_gaps):.3f} m")
    if len(drive_intervals_ms) > 0:
        print("-" * 60)
        print(f"Control Loop Interval  : Mean = {np.mean(drive_intervals_ms):.2f} ms | Std = {np.std(drive_intervals_ms):.2f} ms")
        print(f"P95 / P99 Latency      : P95 = {np.percentile(drive_intervals_ms, 95):.2f} ms | P99 = {np.percentile(drive_intervals_ms, 99):.2f} ms")
        print(f"Worst-Case Cycle Time  : {np.max(drive_intervals_ms):.2f} ms")
    print("=" * 60 + "\n")

    
    export_summary_metrics(
        output_dir=results_path,
        file_name=file_name,
        bag_path=args.bag_path,
        t_odom=t_odom,
        lap_times=lap_times,
        min_gaps=min_gaps,
        drive_intervals_ms=drive_intervals_ms
    )

if __name__ == '__main__':
    main()