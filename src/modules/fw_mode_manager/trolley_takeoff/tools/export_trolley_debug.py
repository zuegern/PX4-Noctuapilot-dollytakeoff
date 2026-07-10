#!/usr/bin/env python3

"""Export PX4 trolley debug data from a ULog file to graph-ready CSV."""

import argparse
import bisect
import csv
import math
from pathlib import Path

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


def nearest_wheel_sample(timestamp, wheel_timestamps, wheel_values):
    if not wheel_timestamps:
        return math.nan, math.nan

    insertion_index = bisect.bisect_left(wheel_timestamps, timestamp)
    candidate_indices = []

    if insertion_index < len(wheel_timestamps):
        candidate_indices.append(insertion_index)

    if insertion_index > 0:
        candidate_indices.append(insertion_index - 1)

    sample_index = min(candidate_indices, key=lambda index: abs(wheel_timestamps[index] - timestamp))
    sample_age_s = abs(wheel_timestamps[sample_index] - timestamp) * 1e-6
    return wheel_values[sample_index], sample_age_s


def extract_wheel_samples(ulog):
    samples = []

    for dataset in ulog.data_list:
        if dataset.name != "landing_gear_wheel":
            continue

        timestamps = dataset.data["timestamp"]
        values = dataset.data["normalized_wheel_setpoint"]
        samples.extend((int(timestamp), float(value)) for timestamp, value in zip(timestamps, values))

    samples.sort(key=lambda sample: sample[0])
    return [sample[0] for sample in samples], [sample[1] for sample in samples]


def extract_trolley_rows(ulog):
    rows = []

    for dataset in ulog.data_list:
        if dataset.name != "debug_array":
            continue

        timestamps = dataset.data["timestamp"]
        identifiers = dataset.data["id"]

        for row_index, timestamp in enumerate(timestamps):
            if int(identifiers[row_index]) != TROLLEY_DEBUG_ID:
                continue

            values = [
                float(dataset.data[f"data[{field_index}]"][row_index])
                for field_index in range(len(FIELD_NAMES))
            ]
            rows.append((int(timestamp), values))

    rows.sort(key=lambda row: row[0])
    return rows


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path, help="Input PX4 .ulg flight log")
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        help="Output CSV path (default: <input>_trolley.csv)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    output_path = args.csv or args.ulog.with_name(f"{args.ulog.stem}_trolley.csv")
    ulog = ULog(str(args.ulog))
    trolley_rows = extract_trolley_rows(ulog)

    if not trolley_rows:
        raise SystemExit(
            f"No trolley debug_array records with id {TROLLEY_DEBUG_ID} found in {args.ulog}"
        )

    wheel_timestamps, wheel_values = extract_wheel_samples(ulog)
    first_timestamp = trolley_rows[0][0]
    angle_indices = [FIELD_NAMES.index(field_name) for field_name in ANGLE_FIELDS]
    angle_headers = [field_name.replace("_rad", "_deg") for field_name in ANGLE_FIELDS]
    header = [
        "timestamp_us",
        "time_s",
        *FIELD_NAMES,
        *angle_headers,
        "actual_wheel_output",
        "actual_wheel_sample_age_s",
    ]

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(header)

        for timestamp, values in trolley_rows:
            angle_values = [
                math.degrees(values[index]) if math.isfinite(values[index]) else math.nan
                for index in angle_indices
            ]
            actual_wheel_output, actual_wheel_age = nearest_wheel_sample(
                timestamp, wheel_timestamps, wheel_values
            )
            writer.writerow(
                [
                    timestamp,
                    (timestamp - first_timestamp) * 1e-6,
                    *values,
                    *angle_values,
                    actual_wheel_output,
                    actual_wheel_age,
                ]
            )

    print(f"Wrote {len(trolley_rows)} trolley samples to {output_path}")


if __name__ == "__main__":
    main()
