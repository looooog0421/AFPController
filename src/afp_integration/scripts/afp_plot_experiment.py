#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


DEFAULT_ROLLER_DIAMETER = 0.05
DEFAULT_REDUCTION_RATIO = 36.0
DEFAULT_SPEED_SOURCE = "motor_velocity"
VALID_SPEED_SOURCES = {"motor_velocity", "logged_demo", "filtered_diff", "position_red", "all"}
RED_R = 80.0
RED_H = 0.005
FILTER_WINDOW = 9
MAX_SPEED_JUMP = 0.02


def to_float(value):
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def read_csv(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: to_float(v) for k, v in row.items()})
    return rows


def column(rows, key):
    return [row.get(key, math.nan) for row in rows]


def is_valid_number(value):
    return value is not None and not math.isnan(value)


def motor_velocity_to_linear_speed(speed_raw, roller_diameter, reduction_ratio):
    if not is_valid_number(speed_raw):
        return math.nan
    output_deg_s = speed_raw * 6.0 / reduction_ratio
    circumference = math.pi * roller_diameter
    return output_deg_s / 360.0 * circumference


def deg_s_to_linear_speed(output_deg_s, roller_diameter):
    if not is_valid_number(output_deg_s):
        return math.nan
    circumference = math.pi * roller_diameter
    return output_deg_s / 360.0 * circumference


def fal(e, alpha, delta):
    if not is_valid_number(e):
        return math.nan
    if abs(e) <= delta:
        return e / (delta ** (1.0 - alpha))
    return math.copysign(abs(e) ** alpha, e)


def moving_average_valid(values, window_size):
    if window_size <= 1:
        return list(values)

    half = window_size // 2
    filtered = []
    for i in range(len(values)):
        start = max(0, i - half)
        end = min(len(values), i + half + 1)
        window = [v for v in values[start:end] if is_valid_number(v)]
        if window:
            filtered.append(sum(window) / len(window))
        else:
            filtered.append(math.nan)
    return filtered


def clamp_speed_jumps(values, max_jump=MAX_SPEED_JUMP):
    clamped = []
    last_valid = None
    for value in values:
        if not is_valid_number(value):
            clamped.append(math.nan)
            continue

        if last_valid is None:
            clamped.append(value)
            last_valid = value
            continue

        delta = value - last_valid
        if abs(delta) > max_jump:
            value = last_valid + math.copysign(max_jump, delta)

        clamped.append(value)
        last_valid = value

    return clamped


def build_motor_velocity_speed(rows, roller_diameter, reduction_ratio):
    motor_status_velocity = column(rows, "motor_status_velocity")
    return [
        motor_velocity_to_linear_speed(v, roller_diameter, reduction_ratio)
        for v in motor_status_velocity
    ]


def build_filtered_diff_speed(rows, window_size=FILTER_WINDOW, max_jump=MAX_SPEED_JUMP):
    raw_speed = column(rows, "actual_linear_speed")
    clamped = clamp_speed_jumps(raw_speed, max_jump=max_jump)
    return moving_average_valid(clamped, window_size)


def build_position_red_speed(rows, roller_diameter, reduction_ratio, red_r=RED_R, red_h=RED_H):
    positions = column(rows, "motor_status_position")
    time_rel = column(rows, "time_rel")

    valid_times = [t for t in time_rel if is_valid_number(t)]
    if len(valid_times) >= 2:
        diffs = [valid_times[i] - valid_times[i - 1] for i in range(1, len(valid_times)) if valid_times[i] > valid_times[i - 1]]
        if diffs:
            red_h = sum(diffs) / len(diffs)

    z1 = None
    z2 = 0.0
    red_speed = []

    for pos in positions:
        if not is_valid_number(pos):
            red_speed.append(math.nan)
            continue

        if z1 is None:
            z1 = pos
            red_speed.append(0.0)
            continue

        e = z1 - pos
        z1 = z1 + red_h * (z2 - red_r * fal(e, 0.5, red_h))
        z2 = z2 - red_h * (red_r * red_r * fal(e, 0.25, red_h))

        output_deg_s = z2 / reduction_ratio
        red_speed.append(deg_s_to_linear_speed(output_deg_s, roller_diameter))

    return red_speed


def build_speed_series(rows, roller_diameter, reduction_ratio):
    return {
        "motor_velocity": build_motor_velocity_speed(rows, roller_diameter, reduction_ratio),
        "logged_demo": column(rows, "actual_linear_speed"),
        "filtered_diff": build_filtered_diff_speed(rows),
        "position_red": build_position_red_speed(rows, roller_diameter, reduction_ratio),
    }


def make_plot(csv_path, speed_source=DEFAULT_SPEED_SOURCE, roller_diameter=DEFAULT_ROLLER_DIAMETER, reduction_ratio=DEFAULT_REDUCTION_RATIO):
    rows = read_csv(csv_path)
    if not rows:
        raise RuntimeError("CSV 为空，无法绘图")

    if speed_source not in VALID_SPEED_SOURCES:
        raise RuntimeError(f"无效 speed_source: {speed_source}")

    t = column(rows, "time_rel")
    motor_target = column(rows, "motor_target_angle")
    final_target = column(rows, "final_target")
    motor_status_pos = column(rows, "motor_status_position")
    motor_cmd_pos = column(rows, "motor_cmd_position")
    tension = column(rows, "tension")
    tension_corr = column(rows, "tension_correction_angle")
    desired_linear_speed = column(rows, "desired_linear_speed")
    speed_series = build_speed_series(rows, roller_diameter, reduction_ratio)

    out_dir = os.path.splitext(csv_path)[0] + "_figures"
    os.makedirs(out_dir, exist_ok=True)

    plt.figure(figsize=(10, 5))
    plt.plot(t, motor_target, label="feedforward target angle")
    plt.plot(t, final_target, label="final target angle")
    plt.plot(t, motor_status_pos, label="motor actual position")
    plt.plot(t, motor_cmd_pos, label="motor cmd position", alpha=0.7)
    plt.xlabel("time (s)")
    plt.ylabel("angle (deg)")
    plt.title("Feedforward / final target / motor position")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "01_position_tracking.png"), dpi=200)
    plt.close()

    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(t, tension, color="tab:red", label="tension")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("tension", color="tab:red")
    ax1.tick_params(axis="y", labelcolor="tab:red")
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(t, motor_status_pos, color="tab:blue", label="motor actual position")
    ax2.plot(t, tension_corr, color="tab:green", label="tension correction angle", alpha=0.8)
    ax2.set_ylabel("angle (deg)", color="tab:blue")
    ax2.tick_params(axis="y", labelcolor="tab:blue")

    lines = ax1.get_lines() + ax2.get_lines()
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, loc="best")
    plt.title("Tension and motor output")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "02_tension_vs_motor.png"), dpi=200)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(t, desired_linear_speed, label="desired linear speed", color="black", linewidth=1.8)
    if speed_source == "all":
        axes[0].plot(t, speed_series["motor_velocity"], label="actual linear speed (motor_velocity)", alpha=0.9)
        axes[0].plot(t, speed_series["logged_demo"], label="actual linear speed (logged_demo)", alpha=0.7)
        axes[0].plot(t, speed_series["filtered_diff"], label="actual linear speed (filtered_diff)", alpha=0.95, linewidth=2.0)
        axes[0].plot(t, speed_series["position_red"], label="actual linear speed (position_red)", alpha=0.9)
    else:
        axes[0].plot(t, speed_series[speed_source], label=f"actual linear speed ({speed_source})")
    axes[0].set_ylabel("speed (m/s)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(t, motor_target, label="feedforward target angle")
    axes[1].plot(t, final_target, label="final target angle")
    axes[1].plot(t, motor_status_pos, label="motor actual position")
    axes[1].set_ylabel("angle (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    axes[2].plot(t, tension, label="tension")
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylabel("tension")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    plt.suptitle("AFP periodic experiment overview")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "03_overview.png"), dpi=200)
    plt.close(fig)

    print("Figures saved to:", out_dir)
    print("speed_source:", speed_source)
    print("Filter params:", f"window={FILTER_WINDOW}", f"max_jump={MAX_SPEED_JUMP}")
    print("RED params:", f"r={RED_R}", f"h={RED_H}")
    print(os.path.join(out_dir, "01_position_tracking.png"))
    print(os.path.join(out_dir, "02_tension_vs_motor.png"))
    print(os.path.join(out_dir, "03_overview.png"))


if __name__ == "__main__":
    if len(sys.argv) < 2 or len(sys.argv) > 5:
        print("Usage: python3 afp_plot_experiment.py /path/to/file.csv [speed_source] [roller_diameter] [reduction_ratio]")
        print("speed_source: motor_velocity | logged_demo | filtered_diff | position_red | all")
        sys.exit(1)

    csv_path = sys.argv[1]
    speed_source = sys.argv[2] if len(sys.argv) >= 3 else DEFAULT_SPEED_SOURCE
    roller_diameter = float(sys.argv[3]) if len(sys.argv) >= 4 else DEFAULT_ROLLER_DIAMETER
    reduction_ratio = float(sys.argv[4]) if len(sys.argv) >= 5 else DEFAULT_REDUCTION_RATIO

    make_plot(csv_path, speed_source, roller_diameter, reduction_ratio)
