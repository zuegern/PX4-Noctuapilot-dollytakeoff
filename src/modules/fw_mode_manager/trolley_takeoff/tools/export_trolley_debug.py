#!/usr/bin/env python3

"""Export PX4 trolley debug and vibration data from a ULog file to CSV and PNG graphs.

One command produces everything next to the log file:

    python3 export_trolley_debug.py flight.ulg

    flight_trolley.csv      controller inputs, internals and steering outputs
    flight_vibration.csv    body-frame accelerations and angular rates
    flight_plots/*.png      ready-made graphs for all steering modes and vibration

Passing several logs compares their vibration spectra per axis (for judging
trolley/suspension design changes):

    python3 export_trolley_debug.py old_design.ulg new_design.ulg

Requires: pip install pyulog matplotlib
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np
from pyulog import ULog


TROLLEY_DEBUG_ID = 4242

FIELD_NAMES = [
    "state",
    "steering_mode",
    "steering_source",
    "path_type",
    "navigation_available",
    "xy_valid",
    "v_xy_valid",
    "heading_good",
    "dead_reckoning",
    "start_north_m",
    "start_east_m",
    "north_m",
    "east_m",
    "delta_north_m",
    "delta_east_m",
    "velocity_north_m_s",
    "velocity_east_m_s",
    "ground_speed_m_s",
    "yaw_rad",
    "ground_course_rad",
    "reference_bearing_rad",
    "desired_course_rad",
    "desired_minus_yaw_rad",
    "desired_minus_course_rad",
    "path_error_m",
    "raw_wheel_setpoint",
    "slewed_wheel_setpoint",
    "yaw_nudge",
    "final_wheel_command",
    "effective_radius_m",
    "eph_m",
    "evh_m_s",
    "heading_variance_rad2",
    "local_position_age_s",
    "link_required",
    "link_healthy",
    "servo_control_mode",
    "wheel_steering_enabled",
    "xy_reset_counter",
    "heading_reset_counter",
    "lateral_acceleration_m_s2",
    "npfg_signed_track_error_m",
    "npfg_track_error_bound_m",
    "path_heading_rad",
    "path_curvature_1_m",
    "controller_heading_error_rad",
    "desired_yaw_offset_rad",
    "steering_feedforward_rad",
    "steering_feedback_rad",
    "steering_limit_rad",
    "path_feasible",
    "longitudinal_speed_m_s",
    "minimum_cross_track_gain_1_m",
    "stability_condition_met",
    "wheel_yaw_setpoint_rad",
    "yaw_rate_feedforward_rad_s",
]

ANGLE_FIELDS = [
    "yaw_rad",
    "ground_course_rad",
    "reference_bearing_rad",
    "desired_course_rad",
    "desired_minus_yaw_rad",
    "desired_minus_course_rad",
    "path_heading_rad",
    "controller_heading_error_rad",
    "desired_yaw_offset_rad",
    "wheel_yaw_setpoint_rad",
    "steering_feedforward_rad",
    "steering_feedback_rad",
    "steering_limit_rad",
]

STATE_NAMES = {
    0: "align",
    1: "throttle ramp",
    2: "clamped",
    3: "climbout",
    4: "flying",
    5: "aborted",
}

STATE_COLORS = {
    0: "#c6b3e6",
    1: "#ffd9a8",
    2: "#ffb3b3",
    3: "#b3d9ff",
    4: "#b3e6b3",
    5: "#d9d9d9",
}

STEERING_MODE_NAMES = {
    0: "heading",
    1: "path tracking",
    2: "open loop",
    3: "PIFF path",
}


def find_dataset(ulog, name, multi_id=0):
    for dataset in ulog.data_list:
        if dataset.name == name and getattr(dataset, "multi_id", 0) == multi_id:
            return dataset

    return None


def extract_wheel_samples(ulog):
    dataset = find_dataset(ulog, "landing_gear_wheel")

    if dataset is None:
        return np.empty(0, dtype=np.int64), np.empty(0)

    timestamps = np.asarray(dataset.data["timestamp"], dtype=np.int64)
    values = np.asarray(dataset.data["normalized_wheel_setpoint"], dtype=float)
    order = np.argsort(timestamps)
    return timestamps[order], values[order]


def nearest_samples(target_timestamps, source_timestamps, source_values):
    """For each target timestamp return the nearest source value and its age in seconds."""
    if len(source_timestamps) == 0:
        nan = np.full(len(target_timestamps), math.nan)
        return nan, nan.copy()

    indices = np.searchsorted(source_timestamps, target_timestamps)
    indices = np.clip(indices, 0, len(source_timestamps) - 1)
    previous = np.clip(indices - 1, 0, len(source_timestamps) - 1)
    use_previous = (np.abs(source_timestamps[previous] - target_timestamps)
                    < np.abs(source_timestamps[indices] - target_timestamps))
    indices[use_previous] = previous[use_previous]
    ages = np.abs(source_timestamps[indices] - target_timestamps) * 1e-6
    return source_values[indices], ages


def extract_trolley_data(ulog):
    """Return a dict of column name -> numpy array for all trolley debug rows."""
    timestamps = []
    rows = []

    for dataset in ulog.data_list:
        if dataset.name != "debug_array":
            continue

        dataset_timestamps = np.asarray(dataset.data["timestamp"], dtype=np.int64)
        identifiers = np.asarray(dataset.data["id"], dtype=int)
        mask = identifiers == TROLLEY_DEBUG_ID

        if not mask.any():
            continue

        columns = np.column_stack([
            np.asarray(dataset.data[f"data[{field_index}]"], dtype=float)[mask]
            for field_index in range(len(FIELD_NAMES))
        ])
        timestamps.append(dataset_timestamps[mask])
        rows.append(columns)

    if not rows:
        return None

    timestamps = np.concatenate(timestamps)
    rows = np.vstack(rows)
    order = np.argsort(timestamps)
    timestamps = timestamps[order]
    rows = rows[order]

    data = {"timestamp_us": timestamps}

    for field_index, field_name in enumerate(FIELD_NAMES):
        data[field_name] = rows[:, field_index]

    for field_name in ANGLE_FIELDS:
        data[field_name.replace("_rad", "_deg")] = np.degrees(data[field_name])

    return data


def extract_vibration_data(ulog):
    """Return body-frame accelerations and angular rates (FRD: X forward, Y right, Z down)."""
    dataset = find_dataset(ulog, "sensor_combined")

    if dataset is not None:
        timestamps = np.asarray(dataset.data["timestamp"], dtype=np.int64)
        data = {
            "timestamp_us": timestamps,
            "accel_forward_m_s2": np.asarray(dataset.data["accelerometer_m_s2[0]"], dtype=float),
            "accel_right_m_s2": np.asarray(dataset.data["accelerometer_m_s2[1]"], dtype=float),
            "accel_down_m_s2": np.asarray(dataset.data["accelerometer_m_s2[2]"], dtype=float),
            "gyro_roll_rad_s": np.asarray(dataset.data["gyro_rad[0]"], dtype=float),
            "gyro_pitch_rad_s": np.asarray(dataset.data["gyro_rad[1]"], dtype=float),
            "gyro_yaw_rad_s": np.asarray(dataset.data["gyro_rad[2]"], dtype=float),
        }
        return data

    # Fallback for logs without sensor_combined: filtered low-rate topics.
    accel = find_dataset(ulog, "vehicle_acceleration")
    rates = find_dataset(ulog, "vehicle_angular_velocity")

    if accel is None:
        return None

    timestamps = np.asarray(accel.data["timestamp"], dtype=np.int64)
    data = {
        "timestamp_us": timestamps,
        "accel_forward_m_s2": np.asarray(accel.data["xyz[0]"], dtype=float),
        "accel_right_m_s2": np.asarray(accel.data["xyz[1]"], dtype=float),
        "accel_down_m_s2": np.asarray(accel.data["xyz[2]"], dtype=float),
    }

    if rates is not None:
        for axis_index, axis_name in enumerate(["gyro_roll_rad_s", "gyro_pitch_rad_s", "gyro_yaw_rad_s"]):
            values, _ = nearest_samples(timestamps,
                                        np.asarray(rates.data["timestamp"], dtype=np.int64),
                                        np.asarray(rates.data[f"xyz[{axis_index}]"], dtype=float))
            data[axis_name] = values

    else:
        for axis_name in ["gyro_roll_rad_s", "gyro_pitch_rad_s", "gyro_yaw_rad_s"]:
            data[axis_name] = np.full(len(timestamps), math.nan)

    return data


def sample_rate_hz(timestamps):
    if len(timestamps) < 3:
        return math.nan

    dt = np.median(np.diff(timestamps)) * 1e-6
    return 1.0 / dt if dt > 0 else math.nan


def rolling_mean_and_rms(values, window_samples):
    """Rolling mean (low-passed signal, shows sustained accelerations / slides) and
    rolling RMS about that mean (vibration intensity)."""
    window_samples = max(1, int(window_samples))
    kernel = np.ones(window_samples) / window_samples
    finite = np.where(np.isfinite(values), values, 0.0)
    weight = np.convolve(np.isfinite(values).astype(float), kernel, mode="same")
    weight[weight == 0] = math.nan
    mean = np.convolve(finite, kernel, mode="same") / weight
    deviation = np.convolve(finite ** 2, kernel, mode="same") / weight - mean ** 2
    rms = np.sqrt(np.clip(deviation, 0.0, None))
    return mean, rms


def welch_psd(values, rate_hz, segment_length=512):
    """One-sided power spectral density with Hann windowing and 50 % overlap."""
    values = values[np.isfinite(values)]

    if len(values) < 32 or not math.isfinite(rate_hz):
        return None, None

    segment_length = int(min(segment_length, 2 ** math.floor(math.log2(len(values)))))
    step = segment_length // 2
    window = np.hanning(segment_length)
    window_power = (window ** 2).sum()
    spectra = []

    for start in range(0, len(values) - segment_length + 1, step):
        segment = values[start:start + segment_length]
        segment = (segment - segment.mean()) * window
        spectrum = np.abs(np.fft.rfft(segment)) ** 2 / (rate_hz * window_power)
        spectrum[1:-1] *= 2.0
        spectra.append(spectrum)

    frequencies = np.fft.rfftfreq(segment_length, d=1.0 / rate_hz)
    return frequencies, np.mean(spectra, axis=0)


ACCEL_SPECTRUM_AXES = [
    ("accel_forward_m_s2", "X forward (front-back)"),
    ("accel_right_m_s2", "Y right (left-right)"),
    ("accel_down_m_s2", "Z down (vertical / suspension)"),
]

SPECTRUM_BANDS = [(0.5, 10.0), (10.0, 30.0), (30.0, math.inf)]


def band_rms(frequencies, psd, low_hz, high_hz):
    """RMS acceleration inside one frequency band, integrated from the PSD."""
    mask = (frequencies >= low_hz) & (frequencies < high_hz)

    if not mask.any():
        return math.nan

    integrate = getattr(np, "trapezoid", None) or np.trapz
    return math.sqrt(integrate(psd[mask], frequencies[mask]))


def compute_spectra(vibration, mask, rate_hz):
    return {name: welch_psd(vibration[name][mask], rate_hz) for name, _ in ACCEL_SPECTRUM_AXES}


def print_band_summary(label, spectra):
    parts = []

    for name, axis_label in ACCEL_SPECTRUM_AXES:
        frequencies, psd = spectra[name]

        if frequencies is None:
            continue

        bands = " / ".join(f"{band_rms(frequencies, psd, low_hz, high_hz):.3f}"
                           for low_hz, high_hz in SPECTRUM_BANDS)
        parts.append(f"{axis_label.split()[0]}: {bands}")

    if parts:
        print(f"Vibration RMS [m/s^2] in bands 0.5-10 / 10-30 / 30+ Hz, {label}  "
              + "   ".join(parts))


def plot_spectra(plt, spectra_by_label, title, output_path):
    """One panel per body axis; one PSD curve per log so designs can be overlaid."""
    figure, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 10))
    figure.suptitle(title)
    wrote_any = False

    for axis, (name, axis_label) in zip(axes, ACCEL_SPECTRUM_AXES):
        for label, spectra in spectra_by_label.items():
            frequencies, psd = spectra[name]

            if frequencies is not None:
                axis.semilogy(frequencies[1:], psd[1:], linewidth=1.0, label=label)
                wrote_any = True

        axis.set_title(axis_label, fontsize=9)
        axis.set_ylabel("PSD [(m/s^2)^2 / Hz]")
        axis.grid(alpha=0.3, which="both")

        if axis.get_legend_handles_labels()[0]:
            axis.legend(fontsize=8)

    axes[-1].set_xlabel("frequency [Hz]")

    if wrote_any:
        figure.tight_layout()
        figure.savefig(output_path, dpi=130)
        print(f"Wrote {output_path}")


def write_csv(path, data, first_timestamp):
    names = list(data.keys())
    columns = [data[name] for name in names]
    time_s = (data["timestamp_us"] - first_timestamp) * 1e-6

    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["timestamp_us", "time_s"] + names[1:])

        for row_index in range(len(time_s)):
            writer.writerow([int(columns[0][row_index]), float(time_s[row_index])]
                            + [float(column[row_index]) for column in columns[1:]])

    print(f"Wrote {len(time_s)} rows to {path}")


def shade_states(axis, time_s, states, show_legend_labels=False):
    """Color the plot background by trolley takeoff state."""
    if len(time_s) == 0:
        return

    used_states = set()
    run_start = 0

    for index in range(1, len(states) + 1):
        if index == len(states) or states[index] != states[run_start]:
            state = int(states[run_start]) if math.isfinite(states[run_start]) else -1
            color = STATE_COLORS.get(state)

            if color is not None:
                label = STATE_NAMES.get(state) if show_legend_labels and state not in used_states else None
                axis.axvspan(time_s[run_start], time_s[min(index, len(states) - 1)],
                             color=color, alpha=0.35, linewidth=0, label=label)
                used_states.add(state)

            run_start = index


def new_figure(plt, rows, title):
    figure, axes = plt.subplots(rows, 1, sharex=True, figsize=(12, 2.6 * rows + 1.2))
    figure.suptitle(title)

    if rows == 1:
        axes = [axes]

    return figure, axes


def finish_figure(figure, axes, output_path):
    for axis in axes:
        axis.grid(alpha=0.3)

        if axis.get_legend_handles_labels()[0]:
            axis.legend(loc="upper right", fontsize=8, ncol=2)

    axes[-1].set_xlabel("time [s]")
    figure.tight_layout()
    figure.savefig(output_path, dpi=130)
    print(f"Wrote {output_path}")


def plot_trolley(plt, trolley, time_s, plots_dir):
    # 1: what the steering pipeline commanded and what actually reached the wheels
    figure, axes = new_figure(plt, 3, "Steering: commands through the pipeline")
    shade_states(axes[0], time_s, trolley["state"], show_legend_labels=True)
    axes[0].plot(time_s, trolley["state"], "k", drawstyle="steps-post", label="state")
    axes[0].plot(time_s, trolley["steering_mode"], "b--", drawstyle="steps-post", label="steering mode")
    axes[0].plot(time_s, trolley["steering_source"], "g:", drawstyle="steps-post", label="steering source")
    axes[0].set_ylabel("state / mode")
    shade_states(axes[1], time_s, trolley["state"])
    axes[1].plot(time_s, trolley["raw_wheel_setpoint"], label="raw setpoint")
    axes[1].plot(time_s, trolley["slewed_wheel_setpoint"], label="slew-limited setpoint")
    axes[1].plot(time_s, trolley["final_wheel_command"], label="final command (mode manager)")
    axes[1].plot(time_s, trolley["actual_wheel_output"], "k", alpha=0.6, label="actual wheel output")
    axes[1].set_ylabel("wheel cmd [-1..1]")
    shade_states(axes[2], time_s, trolley["state"])
    axes[2].plot(time_s, trolley["yaw_nudge"], label="pilot yaw nudge")
    axes[2].plot(time_s, trolley["wheel_steering_enabled"], drawstyle="steps-post", label="wheel steering enabled")
    axes[2].set_ylabel("nudge / flag")
    finish_figure(figure, axes, plots_dir / "steering.png")

    # 2: what the controller saw (path error, headings) and its angular errors
    figure, axes = new_figure(plt, 3, "Path tracking: errors the controller saw")
    shade_states(axes[0], time_s, trolley["state"], show_legend_labels=True)
    axes[0].plot(time_s, trolley["path_error_m"], label="path error (+ = right of path)")
    axes[0].plot(time_s, trolley["npfg_signed_track_error_m"], "--", label="NPFG track error")
    axes[0].set_ylabel("cross-track [m]")
    shade_states(axes[1], time_s, trolley["state"])
    axes[1].plot(time_s, trolley["yaw_deg"], label="yaw")
    axes[1].plot(time_s, trolley["ground_course_deg"], label="ground course")
    axes[1].plot(time_s, trolley["desired_course_deg"], "--", label="desired course")
    axes[1].plot(time_s, trolley["path_heading_deg"], ":", label="path heading")
    axes[1].plot(time_s, trolley["reference_bearing_deg"], "k:", alpha=0.6, label="reference bearing")
    axes[1].set_ylabel("heading [deg]")
    shade_states(axes[2], time_s, trolley["state"])
    axes[2].plot(time_s, trolley["desired_minus_yaw_deg"], label="desired - yaw")
    axes[2].plot(time_s, trolley["desired_minus_course_deg"], label="desired - course")
    axes[2].plot(time_s, trolley["controller_heading_error_deg"], label="controller heading error")
    axes[2].set_ylabel("error [deg]")
    finish_figure(figure, axes, plots_dir / "path_tracking.png")

    # 3: controller internals (mode 1 terms, PIFF references, stability checks)
    figure, axes = new_figure(plt, 3, "Controller internals (all steering modes)")
    shade_states(axes[0], time_s, trolley["state"], show_legend_labels=True)
    axes[0].plot(time_s, trolley["steering_feedforward_deg"], label="feedforward angle")
    axes[0].plot(time_s, trolley["steering_feedback_deg"], label="feedback angle")
    axes[0].plot(time_s, trolley["steering_limit_deg"], "k--", alpha=0.6, label="steering limit")
    axes[0].plot(time_s, -trolley["steering_limit_deg"], "k--", alpha=0.6)
    axes[0].set_ylabel("steer angle [deg]")
    shade_states(axes[1], time_s, trolley["state"])
    axes[1].plot(time_s, trolley["wheel_yaw_setpoint_deg"], label="wheel yaw setpoint (PIFF ref)")
    axes[1].plot(time_s, trolley["yaw_deg"], alpha=0.7, label="yaw")
    axes[1].plot(time_s, trolley["desired_yaw_offset_deg"], ":", label="offset-point yaw offset")
    axes[1].set_ylabel("yaw [deg]")
    shade_states(axes[2], time_s, trolley["state"])
    axes[2].plot(time_s, trolley["yaw_rate_feedforward_rad_s"], label="yaw-rate feedforward [rad/s]")
    axes[2].plot(time_s, trolley["minimum_cross_track_gain_1_m"], label="min. cross-track gain [1/m]")
    axes[2].plot(time_s, trolley["stability_condition_met"], drawstyle="steps-post", label="stability condition met")
    axes[2].plot(time_s, trolley["path_feasible"], drawstyle="steps-post", label="path feasible")
    axes[2].set_ylabel("checks")
    finish_figure(figure, axes, plots_dir / "controller.png")

    # 4: speeds and lateral acceleration (rollover margin)
    figure, axes = new_figure(plt, 2, "Speeds and lateral acceleration")
    shade_states(axes[0], time_s, trolley["state"], show_legend_labels=True)
    axes[0].plot(time_s, trolley["ground_speed_m_s"], label="ground speed")
    axes[0].plot(time_s, trolley["longitudinal_speed_m_s"], label="body-longitudinal speed")
    axes[0].set_ylabel("speed [m/s]")
    shade_states(axes[1], time_s, trolley["state"])
    axes[1].plot(time_s, trolley["lateral_acceleration_m_s2"], label="guidance lateral accel")
    axes[1].set_ylabel("accel [m/s^2]")
    finish_figure(figure, axes, plots_dir / "speeds.png")

    # 5: estimator and link health
    figure, axes = new_figure(plt, 3, "Health: estimator and Pico link")
    shade_states(axes[0], time_s, trolley["state"], show_legend_labels=True)

    for offset, name in enumerate(["navigation_available", "xy_valid", "v_xy_valid",
                                   "heading_good", "dead_reckoning"]):
        axes[0].plot(time_s, trolley[name] * 0.8 + offset, drawstyle="steps-post", label=name)

    axes[0].set_ylabel("flags (offset)")
    shade_states(axes[1], time_s, trolley["state"])
    axes[1].plot(time_s, trolley["link_required"] * 0.8, drawstyle="steps-post", label="link required")
    axes[1].plot(time_s, trolley["link_healthy"] * 0.8 + 1, drawstyle="steps-post", label="link healthy")
    axes[1].plot(time_s, trolley["xy_reset_counter"] - trolley["xy_reset_counter"][0] + 2,
                 drawstyle="steps-post", label="xy resets (offset +2)")
    axes[1].plot(time_s, trolley["heading_reset_counter"] - trolley["heading_reset_counter"][0] + 3,
                 drawstyle="steps-post", label="heading resets (offset +3)")
    axes[1].set_ylabel("flags (offset)")
    shade_states(axes[2], time_s, trolley["state"])
    axes[2].plot(time_s, trolley["eph_m"], label="eph [m]")
    axes[2].plot(time_s, trolley["evh_m_s"], label="evh [m/s]")
    axes[2].plot(time_s, trolley["local_position_age_s"], label="position age [s]")
    axes[2].set_ylabel("accuracy")
    finish_figure(figure, axes, plots_dir / "health.png")

    # 6: ground track (east/north map view)
    figure, axis = plt.subplots(figsize=(8, 8))
    figure.suptitle("Ground track (map view)")
    valid = np.isfinite(trolley["east_m"]) & np.isfinite(trolley["north_m"])
    axis.plot(trolley["east_m"][valid], trolley["north_m"][valid], ".-", markersize=2, label="vehicle")

    start_valid = np.isfinite(trolley["start_east_m"]) & np.isfinite(trolley["start_north_m"])

    if start_valid.any():
        start_east = trolley["start_east_m"][start_valid][0]
        start_north = trolley["start_north_m"][start_valid][0]
        axis.plot(start_east, start_north, "r*", markersize=12, label="start")
        bearing = trolley["reference_bearing_rad"][start_valid][0]

        if math.isfinite(bearing) and valid.any():
            length = max(10.0, float(np.nanmax(np.hypot(trolley["delta_north_m"], trolley["delta_east_m"]))))
            axis.plot([start_east, start_east + length * math.sin(bearing)],
                      [start_north, start_north + length * math.cos(bearing)],
                      "r--", alpha=0.6, label="reference bearing")

    axis.set_xlabel("east [m]")
    axis.set_ylabel("north [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(plots_dir / "trajectory.png", dpi=130)
    print(f"Wrote {plots_dir / 'trajectory.png'}")


def plot_vibration(plt, vibration, time_s, plots_dir, trolley_window, rms_window_s):
    rate = sample_rate_hz(vibration["timestamp_us"])
    window_samples = rms_window_s * rate if math.isfinite(rate) else 1

    accel_axes = [
        ("accel_forward_m_s2", "X forward: braking/thrust; sustained offset while clamped = front-back sliding"),
        ("accel_right_m_s2", "Y right: cornering; sustained offset = left-right sliding, oscillation = shimmy"),
        ("accel_down_m_s2", "Z down: ~-9.8 static; vibration band = ground roughness / suspension"),
    ]

    figure, axes = new_figure(plt, 3, f"Body accelerations (sampled at {rate:.0f} Hz)"
                              if math.isfinite(rate) else "Body accelerations")

    for axis, (name, description) in zip(axes, accel_axes):
        if trolley_window is not None:
            axis.axvspan(trolley_window[0], trolley_window[1], color="#ffe08a", alpha=0.3,
                         label="trolley takeoff active")

        mean, rms = rolling_mean_and_rms(vibration[name], window_samples)
        axis.plot(time_s, vibration[name], color="0.6", linewidth=0.4, label="raw")
        axis.plot(time_s, mean, "b", label=f"rolling mean ({rms_window_s:.1f} s) = sustained accel")
        axis.plot(time_s, mean + rms, "r", linewidth=0.8, label="mean +/- vibration RMS")
        axis.plot(time_s, mean - rms, "r", linewidth=0.8)
        axis.set_ylabel("[m/s^2]")
        axis.set_title(description, fontsize=9)

    finish_figure(figure, axes, plots_dir / "vibration_accel.png")

    gyro_axes = [
        ("gyro_roll_rad_s", "roll rate: wing rocking on the cradle"),
        ("gyro_pitch_rad_s", "pitch rate: pitching over bumps"),
        ("gyro_yaw_rad_s", "yaw rate: steering response; oscillation = wheel shimmy"),
    ]

    figure, axes = new_figure(plt, 3, "Body angular rates")

    for axis, (name, description) in zip(axes, gyro_axes):
        if trolley_window is not None:
            axis.axvspan(trolley_window[0], trolley_window[1], color="#ffe08a", alpha=0.3,
                         label="trolley takeoff active")

        values_deg = np.degrees(vibration[name])
        mean, rms = rolling_mean_and_rms(values_deg, window_samples)
        axis.plot(time_s, values_deg, color="0.6", linewidth=0.4, label="raw")
        axis.plot(time_s, mean, "b", label=f"rolling mean ({rms_window_s:.1f} s)")
        axis.plot(time_s, mean + rms, "r", linewidth=0.8, label="mean +/- RMS")
        axis.plot(time_s, mean - rms, "r", linewidth=0.8)
        axis.set_ylabel("[deg/s]")
        axis.set_title(description, fontsize=9)

    finish_figure(figure, axes, plots_dir / "vibration_gyro.png")

    # Frequency content per axis, restricted to the trolley-active window when one exists.
    if trolley_window is not None:
        window_mask = (time_s >= trolley_window[0]) & (time_s <= trolley_window[1])
        spectrum_note = "trolley takeoff window"

    else:
        window_mask = np.ones(len(time_s), dtype=bool)
        spectrum_note = "whole log"

    spectra = compute_spectra(vibration, window_mask, rate)
    print_band_summary(spectrum_note, spectra)
    plot_spectra(plt, {spectrum_note: spectra},
                 "Acceleration spectra per axis (Welch power spectral density)",
                 plots_dir / "vibration_spectrum.png")
    plt.close("all")


def process(ulog, log_path, make_csv=True, make_plots=True, tmin=None, tmax=None, rms_window_s=0.5):
    trolley = extract_trolley_data(ulog)
    vibration = extract_vibration_data(ulog)

    if trolley is None:
        print(f"Note: no trolley debug_array records with id {TROLLEY_DEBUG_ID} in {log_path} "
              "(trolley takeoff not active, or SDLOG_PROFILE debug bit not set)")

    if vibration is None:
        print(f"Note: no sensor_combined or vehicle_acceleration data in {log_path}")

    if trolley is None and vibration is None:
        raise SystemExit("Nothing to export")

    if trolley is not None:
        wheel_timestamps, wheel_values = extract_wheel_samples(ulog)
        trolley["actual_wheel_output"], trolley["actual_wheel_sample_age_s"] = nearest_samples(
            trolley["timestamp_us"], wheel_timestamps, wheel_values)

    # One shared time origin so trolley and vibration data line up across files and plots.
    first_timestamp = trolley["timestamp_us"][0] if trolley is not None else vibration["timestamp_us"][0]

    trolley_window = None

    if trolley is not None:
        trolley_window = (0.0, float((trolley["timestamp_us"][-1] - first_timestamp) * 1e-6))

    if vibration is not None and (tmin is not None or tmax is not None):
        time_s = (vibration["timestamp_us"] - first_timestamp) * 1e-6
        mask = np.ones(len(time_s), dtype=bool)

        if tmin is not None:
            mask &= time_s >= tmin

        if tmax is not None:
            mask &= time_s <= tmax

        vibration = {name: values[mask] for name, values in vibration.items()}

        if len(vibration["timestamp_us"]) == 0:
            print("Note: --tmin/--tmax excluded all vibration samples")
            vibration = None

    if make_csv:
        if trolley is not None:
            write_csv(log_path.with_name(f"{log_path.stem}_trolley.csv"), trolley, first_timestamp)

        if vibration is not None:
            write_csv(log_path.with_name(f"{log_path.stem}_vibration.csv"), vibration, first_timestamp)

    if make_plots:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

        except ImportError:
            raise SystemExit("matplotlib is required for plots: python3 -m pip install matplotlib "
                             "(or rerun with --no-plots)")

        plots_dir = log_path.with_name(f"{log_path.stem}_plots")
        plots_dir.mkdir(parents=True, exist_ok=True)

        if trolley is not None:
            trolley_time_s = (trolley["timestamp_us"] - first_timestamp) * 1e-6
            plot_trolley(plt, trolley, trolley_time_s, plots_dir)

        if vibration is not None:
            vibration_time_s = (vibration["timestamp_us"] - first_timestamp) * 1e-6
            plot_vibration(plt, vibration, vibration_time_s, plots_dir, trolley_window, rms_window_s)

        plt.close("all")


def compare_logs(ulogs_by_label, output_path):
    """Overlay the per-axis acceleration spectra of several logs, e.g. two suspension designs."""
    spectra_by_label = {}

    for label, ulog in ulogs_by_label.items():
        vibration = extract_vibration_data(ulog)

        if vibration is None:
            print(f"Note: no IMU data in {label}, skipped")
            continue

        timestamps = vibration["timestamp_us"]
        rate = sample_rate_hz(timestamps)
        trolley = extract_trolley_data(ulog)
        mask = np.ones(len(timestamps), dtype=bool)
        note = "whole log"

        if trolley is not None:
            window = ((timestamps >= trolley["timestamp_us"][0])
                      & (timestamps <= trolley["timestamp_us"][-1]))

            if window.any():
                mask = window
                note = "trolley window"

        spectra = compute_spectra(vibration, mask, rate)
        print_band_summary(f"{label} ({note})", spectra)
        spectra_by_label[label] = spectra

    if not spectra_by_label:
        raise SystemExit("No IMU data in any of the given logs")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

    except ImportError:
        raise SystemExit("matplotlib is required for plots: python3 -m pip install matplotlib")

    plot_spectra(plt, spectra_by_label,
                 "Acceleration spectra comparison (Welch power spectral density)", output_path)
    plt.close("all")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ulog", type=Path, nargs="+",
                        help="Input PX4 .ulg flight log. Passing several logs switches to "
                             "comparison mode: their vibration spectra are overlaid per axis "
                             "in one vibration_compare.png")
    parser.add_argument("--no-plots", action="store_true", help="Only write CSV files")
    parser.add_argument("--no-csv", action="store_true", help="Only write graphs")
    parser.add_argument("--tmin", type=float, help="Vibration window start [s, relative to first trolley sample]")
    parser.add_argument("--tmax", type=float, help="Vibration window end [s]")
    parser.add_argument("--rms-window", type=float, default=0.5,
                        help="Rolling mean/RMS window for vibration plots [s] (default 0.5)")
    return parser.parse_args()


def main():
    args = parse_args()

    if len(args.ulog) > 1:
        ulogs_by_label = {path.stem: ULog(str(path)) for path in args.ulog}
        compare_logs(ulogs_by_label, args.ulog[0].with_name("vibration_compare.png"))
        return

    log_path = args.ulog[0]
    ulog = ULog(str(log_path))
    process(ulog, log_path, make_csv=not args.no_csv, make_plots=not args.no_plots,
            tmin=args.tmin, tmax=args.tmax, rms_window_s=args.rms_window)


if __name__ == "__main__":
    main()
